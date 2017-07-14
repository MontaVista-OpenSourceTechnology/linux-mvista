/*
 * Copyright (C) 2017 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>

#define IPROC_GPIO_CCA_ID	0
#define IPROC_GPIO_CCB_ID	1
#define IPROC_GPIO_CCG_ID	2

#define REGOFFSET_GPIO_DIN	0x0
#define REGOFFSET_GPIO_DOUT	0x4
#define REGOFFSET_GPIO_OUT_EN	0x8

#define CCA_INT_F_GPIOINT		1
#define CCA_INT_STS 			0x20
#define CCA_INT_MASK			0x24
#define GPIO_CCA_DIN 			0x0
#define GPIO_CCA_INT_LEVEL		0x10
#define GPIO_CCA_INT_LEVEL_MASK 	0x14
#define GPIO_CCA_INT_EVENT		0x18
#define GPIO_CCA_INT_EVENT_MASK 	0x1C
#define GPIO_CCA_INT_EDGE		0x24

#define GPIO_CCB_INT_TYPE		0xC
#define GPIO_CCB_INT_DE 		0x10
#define GPIO_CCB_INT_EDGE		0x14
#define GPIO_CCB_INT_MSTAT		0x20
#define GPIO_CCB_INT_CLR		0x24
#define GPIO_CCB_INT_MASK		0x18

/* locking wrappers to deal with multiple access to the same gpio bank */
#define iproc_gpio_lock(_oc, _fl) spin_lock_irqsave(&(_oc)->lock, _fl)
#define iproc_gpio_unlock(_oc, _fl) spin_unlock_irqrestore(&(_oc)->lock, _fl)

struct iproc_gpio_irqcfg {
	unsigned long flags;
	irqreturn_t (*handler)(int irq, void *dev);
	void (*ack)(unsigned int irq);
	void (*unmask)(unsigned int irq);
	void (*mask)(unsigned int irq);
	int (*set_type)(unsigned int irq, unsigned int type);
};

struct iproc_gpio_chip {
	int id;
	struct gpio_chip	chip;
	struct iproc_gpio_cfg	*config;
	void __iomem		*ioaddr;
	void __iomem		*intr_ioaddr;
	spinlock_t		lock;
	struct irq_domain	*irq_domain;
	struct resource 	*resource;
	int irq;
	struct iproc_gpio_irqcfg	*irqcfg;
	/* GPIO register bit shift */
	int pin_reg_bit_shift;
};

static inline struct iproc_gpio_chip *to_iproc_gpio(struct gpio_chip *gpc)
{
	return container_of(gpc, struct iproc_gpio_chip, chip);
}

static u32 _iproc_gpio_readl(struct iproc_gpio_chip *chip, int reg)
{
	return readl(chip->ioaddr + reg);
}

static void _iproc_gpio_writel(struct iproc_gpio_chip *chip, u32 val, int reg)
{
	writel(val, chip->ioaddr + reg);
}

/*
@ pin : GPIO register bit
*/
static int iproc_gpio_to_irq(struct iproc_gpio_chip *chip, u32 pin)
{
	return irq_linear_revmap(chip->irq_domain, pin - chip->pin_reg_bit_shift);
}

/* returns the corresponding gpio register bit */
static int iproc_irq_to_gpio(struct iproc_gpio_chip *chip, u32 irq)
{
	struct irq_data *data = irq_domain_get_irq_data(chip->irq_domain, irq);

	return data->hwirq + chip->pin_reg_bit_shift;
}

static void iproc_gpio_irq_ack(struct irq_data *d)
{
	u32 irq = d->irq;
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);

	if (ourchip) {
		struct iproc_gpio_irqcfg *irqcfg = ourchip->irqcfg;
		if (irqcfg && irqcfg->ack)
			irqcfg->ack(irq);
	}
}

static void iproc_gpio_irq_unmask(struct irq_data *d)
{
	u32 irq = d->irq;
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);

	if (ourchip) {
		struct iproc_gpio_irqcfg *irqcfg = ourchip->irqcfg;
		if (irqcfg && irqcfg->unmask)
			irqcfg->unmask(irq);
	}
}

static void iproc_gpio_irq_mask(struct irq_data *d)
{
	u32 irq = d->irq;
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);

	if (ourchip) {
		struct iproc_gpio_irqcfg *irqcfg = ourchip->irqcfg;
		if (irqcfg && irqcfg->mask)
			irqcfg->mask(irq);
	}
}


static int iproc_gpio_irq_set_type(struct irq_data *d, u32 type)
{
	u32 irq = d->irq;
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);

	if (ourchip) {
		struct iproc_gpio_irqcfg *irqcfg = ourchip->irqcfg;
		if (irqcfg && irqcfg->set_type)
			return irqcfg->set_type(irq, type);
	}

	return -EINVAL;
}

static irqreturn_t iproc_gpio_irq_handler_cca(int irq, void *data)
{
	struct iproc_gpio_chip *iproc_gpio = (struct iproc_gpio_chip *)data;
	struct gpio_chip gc = iproc_gpio->chip;
	int bit;
	unsigned long int_bits = 0;
	u32 int_status;

	/* go through the entire GPIOs and handle all interrupts */
	int_status = readl(iproc_gpio->intr_ioaddr + CCA_INT_STS);
	if (int_status & CCA_INT_F_GPIOINT) {
		u32 event, level;

		/* Get level and edge interrupts */
		event = readl(iproc_gpio->ioaddr + GPIO_CCA_INT_EVENT_MASK);
		event &= readl(iproc_gpio->ioaddr + GPIO_CCA_INT_EVENT);
		level = readl(iproc_gpio->ioaddr + GPIO_CCA_DIN);
		level ^= readl(iproc_gpio->ioaddr + GPIO_CCA_INT_LEVEL);
		level &= readl(iproc_gpio->ioaddr + GPIO_CCA_INT_LEVEL_MASK);
		int_bits = level | event;

		for_each_set_bit(bit, &int_bits, gc.ngpio)
			generic_handle_irq(
				irq_linear_revmap(iproc_gpio->irq_domain, bit));
	}

	return  int_bits ? IRQ_HANDLED : IRQ_NONE;
}


static void iproc_gpio_irq_ack_cca(u32 irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	u32 irq_type, event_status = 0;

	pin = iproc_irq_to_gpio(ourchip, irq);
	irq_type = irq_get_trigger_type(irq);
	if (irq_type & IRQ_TYPE_EDGE_BOTH) {
		event_status |= (1 << pin);
		_iproc_gpio_writel(ourchip, event_status, GPIO_CCA_INT_EVENT);
	}
}

static void iproc_gpio_irq_unmask_cca(u32 irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	u32 int_mask, irq_type, event_mask;

	pin = iproc_irq_to_gpio(ourchip, irq);
	irq_type = irq_get_trigger_type(irq);
	event_mask = _iproc_gpio_readl(ourchip, GPIO_CCA_INT_EVENT_MASK);
	int_mask = _iproc_gpio_readl(ourchip, GPIO_CCA_INT_LEVEL_MASK);

	if (irq_type & IRQ_TYPE_EDGE_BOTH) {
		event_mask |= 1 << pin;
		_iproc_gpio_writel(ourchip, event_mask, GPIO_CCA_INT_EVENT_MASK);
	} else {
		int_mask |= 1 << pin;
		_iproc_gpio_writel(ourchip, int_mask, GPIO_CCA_INT_LEVEL_MASK);
	}
}

static void iproc_gpio_irq_mask_cca(u32 irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	u32 irq_type, int_mask, event_mask;

	pin = iproc_irq_to_gpio(ourchip, irq);
	irq_type = irq_get_trigger_type(irq);
	event_mask = _iproc_gpio_readl(ourchip, GPIO_CCA_INT_EVENT_MASK);
	int_mask = _iproc_gpio_readl(ourchip, GPIO_CCA_INT_LEVEL_MASK);

	if (irq_type & IRQ_TYPE_EDGE_BOTH) {
		event_mask &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, event_mask, GPIO_CCA_INT_EVENT_MASK);
	} else {
		int_mask &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, int_mask, GPIO_CCA_INT_LEVEL_MASK);
	}
}

static int iproc_gpio_irq_set_type_cca(u32 irq, u32 type)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	u32 event_pol, int_pol;

	pin = iproc_irq_to_gpio(ourchip, irq);

	switch (type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_EDGE_RISING:
		event_pol = _iproc_gpio_readl(ourchip, GPIO_CCA_INT_EDGE);
		event_pol &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, event_pol, GPIO_CCA_INT_EDGE);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		event_pol = _iproc_gpio_readl(ourchip, GPIO_CCA_INT_EDGE);
		event_pol |= (1 << pin);
		_iproc_gpio_writel(ourchip, event_pol, GPIO_CCA_INT_EDGE);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		int_pol = _iproc_gpio_readl(ourchip, GPIO_CCA_INT_LEVEL);
		int_pol &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, int_pol, GPIO_CCA_INT_LEVEL);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		int_pol = _iproc_gpio_readl(ourchip,GPIO_CCA_INT_LEVEL);
		int_pol |= (1 << pin);
		_iproc_gpio_writel(ourchip, int_pol, GPIO_CCA_INT_LEVEL);
		break;
	default:
		/* should not come here */
		return -EINVAL;
	}

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		irq_set_handler_locked(irq_get_irq_data(irq), handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		irq_set_handler_locked(irq_get_irq_data(irq), handle_edge_irq);

	return 0;
}

struct iproc_gpio_irqcfg cca_gpio_irqcfg = {
	/*
	 * Remove IRQF_NO_SUSPEND to be consistent with 8250_core.c setting,
	 * since CCA gpio and uart share the same IRQ.
	 */
	.flags = IRQF_SHARED | IRQF_NO_THREAD,
	.handler = iproc_gpio_irq_handler_cca,
	.ack = iproc_gpio_irq_ack_cca,
	.mask = iproc_gpio_irq_mask_cca,
	.unmask = iproc_gpio_irq_unmask_cca,
	.set_type = iproc_gpio_irq_set_type_cca,
};

static irqreturn_t iproc_gpio_irq_handler_ccb(int irq, void *dev)
{
	struct iproc_gpio_chip *ourchip = dev;
	int iter, max_pin;
	u32  val;

	val = _iproc_gpio_readl(ourchip, GPIO_CCB_INT_MSTAT);
	if (!val)
		return IRQ_NONE;

	/* Max GPIO register bit */
	max_pin = ourchip->pin_reg_bit_shift + ourchip->chip.ngpio;
	for (iter = ourchip->pin_reg_bit_shift; iter < max_pin; iter++)
		if (val & (1 << iter))
			generic_handle_irq(iproc_gpio_to_irq(ourchip, iter));

	return IRQ_HANDLED;
}

static void iproc_gpio_irq_ack_ccb(u32 irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	u32 int_clear = 0;

	pin = iproc_irq_to_gpio(ourchip, irq);
	int_clear |= (1 << pin);
	_iproc_gpio_writel(ourchip, int_clear, GPIO_CCB_INT_CLR);
}

static void iproc_gpio_irq_unmask_ccb(u32 irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	u32 int_mask;

	pin = iproc_irq_to_gpio(ourchip, irq);
	int_mask = _iproc_gpio_readl(ourchip, GPIO_CCB_INT_MASK);
	int_mask |= (1 << pin);
	_iproc_gpio_writel(ourchip, int_mask, GPIO_CCB_INT_MASK);
}

static void iproc_gpio_irq_mask_ccb(u32 irq)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	u32 int_mask;

	pin = iproc_irq_to_gpio(ourchip, irq);
	int_mask = _iproc_gpio_readl(ourchip, GPIO_CCB_INT_MASK);
	int_mask &= ~(1 << pin);
	_iproc_gpio_writel(ourchip, int_mask,GPIO_CCB_INT_MASK);
}

static int iproc_gpio_irq_set_type_ccb(u32 irq, u32 type)
{
	struct iproc_gpio_chip *ourchip = irq_get_chip_data(irq);
	int pin;
	u32  int_type, int_de, int_edge;

	pin = iproc_irq_to_gpio(ourchip, irq);
	int_type = _iproc_gpio_readl(ourchip, GPIO_CCB_INT_TYPE);
	int_edge = _iproc_gpio_readl(ourchip, GPIO_CCB_INT_EDGE);

	switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
		int_type &= ~(1 << pin);
		int_de = _iproc_gpio_readl(ourchip, GPIO_CCB_INT_DE);
		int_de |= (1 << pin);
		_iproc_gpio_writel(ourchip, int_de, GPIO_CCB_INT_DE);
		break;
	case IRQ_TYPE_EDGE_RISING:
		int_type &= ~(1 << pin);
		int_edge |= (1 << pin);
		int_de = _iproc_gpio_readl(ourchip, GPIO_CCB_INT_DE);
		int_de  &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, int_de, GPIO_CCB_INT_DE);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		int_type &= ~(1 << pin);
		int_edge &= ~(1 << pin);
		int_de = _iproc_gpio_readl(ourchip, GPIO_CCB_INT_DE);
		int_de  &= ~(1 << pin);
		_iproc_gpio_writel(ourchip, int_de, GPIO_CCB_INT_DE);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		int_type |= (1 << pin);
		int_edge |= (1 << pin);
		break;
	case IRQ_TYPE_LEVEL_LOW:
		int_type |= (1 << pin);
		int_edge &= ~(1 << pin);
		break;
	default:
		/* should not come here */
		return -EINVAL;
	}

	_iproc_gpio_writel(ourchip, int_type, GPIO_CCB_INT_TYPE);
	_iproc_gpio_writel(ourchip, int_edge, GPIO_CCB_INT_EDGE);

	if (type & (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH))
		irq_set_handler_locked(irq_get_irq_data(irq), handle_level_irq);
	else if (type & (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		irq_set_handler_locked(irq_get_irq_data(irq), handle_edge_irq);

	return 0;
}

struct iproc_gpio_irqcfg ccb_gpio_irqcfg = {
	.flags = IRQF_NO_SUSPEND | IRQF_NO_THREAD,
	.handler = iproc_gpio_irq_handler_ccb,
	.ack = iproc_gpio_irq_ack_ccb,
	.mask = iproc_gpio_irq_mask_ccb,
	.unmask = iproc_gpio_irq_unmask_ccb,
	.set_type = iproc_gpio_irq_set_type_ccb,
};

static struct irq_chip iproc_gpio_irq_chip = {
	.name         = "IPROC-GPIO",
	.irq_ack      = (void *) iproc_gpio_irq_ack,
	.irq_mask     = (void *) iproc_gpio_irq_mask,
	.irq_unmask   = (void *) iproc_gpio_irq_unmask,
	.irq_set_type = (void *) iproc_gpio_irq_set_type,
};


static int iproc_gpiolib_input(struct gpio_chip *chip, u32 gpio)
{
	struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
	unsigned long flags;
	u32  val, nBitMask;
	int reg_offset;
	u32 pin_offset = gpio + ourchip->pin_reg_bit_shift;

	iproc_gpio_lock(ourchip, flags);

	nBitMask = 1 << pin_offset;
	reg_offset = REGOFFSET_GPIO_OUT_EN;

	val = _iproc_gpio_readl(ourchip, reg_offset);
	val &= ~nBitMask;
	_iproc_gpio_writel(ourchip, val, reg_offset);

	iproc_gpio_unlock(ourchip, flags);

	return 0;
}

static int iproc_gpiolib_output(struct gpio_chip *chip, u32 gpio, int value)
{
	struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
	unsigned long flags, val;
	u32 nBitMask;
	int reg_offset;
	/* GPIO register bit */
	u32 pin_offset = gpio + ourchip->pin_reg_bit_shift;

	iproc_gpio_lock(ourchip, flags);

	nBitMask = 1 << pin_offset;
	reg_offset = REGOFFSET_GPIO_OUT_EN;

	val = _iproc_gpio_readl(ourchip, reg_offset);
	val |= nBitMask;
	_iproc_gpio_writel(ourchip, val, reg_offset);

	iproc_gpio_unlock(ourchip, flags);

	return 0;
}

static void iproc_gpiolib_set(struct gpio_chip *chip, u32 gpio, int value)
{
	struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
	unsigned long flags, val;
	u32 nBitMask;
	/* GPIO register bit */
	u32 pin_offset = gpio + ourchip->pin_reg_bit_shift;

	iproc_gpio_lock(ourchip, flags);

	nBitMask = 1 << pin_offset;

	val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_OUT_EN);
	val &= nBitMask;

	/* this function only applies to output pin */
	if (!val)
		return;

	val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_DOUT);
	if (value == 0)
		/* Set the pin to zero */
		val &= ~nBitMask;
	else
		/* Set the pin to 1 */
		val |= nBitMask;

	_iproc_gpio_writel(ourchip, val, REGOFFSET_GPIO_DOUT);

	iproc_gpio_unlock(ourchip, flags);
}

static int iproc_gpiolib_get(struct gpio_chip *chip, u32 gpio)
{
	struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);
	unsigned long flags;
	u32 val, offset, nBitMask;
	/* GPIO register bit */
	u32 pin_offset = gpio + ourchip->pin_reg_bit_shift;

	iproc_gpio_lock(ourchip, flags);

	nBitMask = 1 << pin_offset;

	/* determine the GPIO pin direction */
	offset = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_OUT_EN);
	offset &= nBitMask;

	if (offset)
		val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_DOUT);
	else
		val = _iproc_gpio_readl(ourchip, REGOFFSET_GPIO_DIN);

	val >>= pin_offset;
	val &= 1;

	iproc_gpio_unlock(ourchip, flags);

	return val;
}

/*
@offset : the gpio pin index number from gpiolib view (minus gpio base only)
*/
static int iproc_gpiolib_to_irq(struct gpio_chip *chip, u32 offset)
{
	struct iproc_gpio_chip *ourchip = to_iproc_gpio(chip);

	return irq_linear_revmap(ourchip->irq_domain, offset);
}

static struct __initconst of_device_id bcm_iproc_gpio_of_match[] = {
	{ .compatible = "brcm,iproc-gpio-cca" },
	{ .compatible = "brcm,iproc-gpio-ccb" },
	{ .compatible = "brcm,iproc-gpio-ccg" },
	{}
};
MODULE_DEVICE_TABLE(of, bcm_iproc_gpio_of_match);

static int iproc_gpio_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct iproc_gpio_chip *iproc_gpio;
	u32 num_gpios, count;
	int ret;

	iproc_gpio = devm_kzalloc(&pdev->dev, sizeof(*iproc_gpio), GFP_KERNEL);
	if (!iproc_gpio) {
		dev_err(&pdev->dev, "Error allocating memory\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, iproc_gpio);

	/* Determine type of gpio controller to allocate */
	if (of_device_is_compatible(dn, "brcm,iproc-gpio-cca")) {
		iproc_gpio->chip.label = "gpio_cca";
		iproc_gpio->id = IPROC_GPIO_CCA_ID;
		iproc_gpio->irqcfg = &cca_gpio_irqcfg;

		iproc_gpio->intr_ioaddr = of_iomap(dn, 1);
		if (!iproc_gpio->intr_ioaddr) {
			dev_err(&pdev->dev, "gpio interrupt base addr fail\n");
			return -ENOMEM;
		}

		dev_info(&pdev->dev, "%s intr_ioaddr: %p\n",
			iproc_gpio->chip.label, iproc_gpio->intr_ioaddr);
	} else if (of_device_is_compatible(dn, "brcm,iproc-gpio-ccb")) {
		iproc_gpio->chip.label = "gpio_ccb";
		iproc_gpio->id = IPROC_GPIO_CCB_ID;
		iproc_gpio->irqcfg = &ccb_gpio_irqcfg;
	} else if (of_device_is_compatible(dn, "brcm,iproc-gpio-ccg")) {
		iproc_gpio->chip.label = "gpio_ccg";
		iproc_gpio->id = IPROC_GPIO_CCG_ID;
		iproc_gpio->irqcfg = &ccb_gpio_irqcfg;
	} else {
		dev_err(&pdev->dev, "Error parsing device tree of GPIO\n");
		return -ENODEV;
	}

	/* Map gpio base ioaddr address */
	iproc_gpio->ioaddr = of_iomap(dn, 0);
	if (!iproc_gpio->ioaddr) {
		dev_err(&pdev->dev, "can't iomap gpio base address\n");
		return -ENOMEM;
	}
	dev_info(&pdev->dev, "%s ioaddr: %p\n", iproc_gpio->chip.label,
			iproc_gpio->ioaddr);

	/* pin_base: gpio pin base */
	if (of_property_read_u32(dn, "pin-base", &iproc_gpio->chip.base)) {
		dev_err(&pdev->dev, "Missing pin-base property\n");
		return -EINVAL;
	}

	/* pin_reg_bit_shift: GPIO register bit offset */
	if (of_property_read_u32(dn, "pin-reg-bit-shift",
			&iproc_gpio->pin_reg_bit_shift)) {
		dev_err(&pdev->dev, "Missing pin-reg-bit-shift property\n");
		return -EINVAL;
	}

	/* Get number of GPIO pin */
	if (of_property_read_u32(dn, "ngpios", &num_gpios)) {
		dev_err(&pdev->dev, "Missing ngpios property\n");
		return -EINVAL;
	}
	iproc_gpio->chip.ngpio = num_gpios;

	iproc_gpio->chip.parent = &pdev->dev;
	iproc_gpio->chip.of_node = dn;

	iproc_gpio->chip.direction_input = iproc_gpiolib_input;
	iproc_gpio->chip.direction_output = iproc_gpiolib_output;
	iproc_gpio->chip.set = iproc_gpiolib_set;
	iproc_gpio->chip.get = iproc_gpiolib_get;
	iproc_gpio->chip.to_irq = iproc_gpiolib_to_irq;

	ret = gpiochip_add_data(&iproc_gpio->chip, iproc_gpio);
	if (ret) {
		dev_err(&pdev->dev, "Could not add gpiochip for %s\n",
				iproc_gpio->chip.label);
		return ret;
	}

	iproc_gpio->irq = platform_get_irq(pdev, 0);

	if (iproc_gpio->irq < 0) {
		dev_warn(&pdev->dev, "No IRQ specified for %s\n",
				iproc_gpio->chip.label);
		return 0;
	}

	/* Create irq domain */
	iproc_gpio->irq_domain = irq_domain_add_linear(dn, num_gpios,
					&irq_domain_simple_ops, iproc_gpio);

	if (!iproc_gpio->irq_domain) {
		dev_err(&pdev->dev, "Couldn't allocate IRQ domain\n");
		ret = -ENODEV;
		goto err_irq_domain;
	}

	/* Map each gpio pin to an IRQ and set the handler */
	for (count = 0; count < num_gpios; count++) {
		int irq;

		irq = irq_create_mapping(iproc_gpio->irq_domain, count);
		irq_set_chip_and_handler(irq, &iproc_gpio_irq_chip,
					handle_simple_irq);
		irq_set_chip_data(irq, iproc_gpio);
	}

	/* Enable GPIO interrupts for CCA GPIO */
	if (iproc_gpio->id == IPROC_GPIO_CCA_ID) {
		u32 val;
		val = readl(iproc_gpio->intr_ioaddr + CCA_INT_MASK);
		val |= CCA_INT_F_GPIOINT;
		writel(val, iproc_gpio->intr_ioaddr + CCA_INT_MASK);
	}

	/* Install ISR for this GPIO controller */
	if (iproc_gpio->irqcfg) {
		struct iproc_gpio_irqcfg *irqcfg = iproc_gpio->irqcfg;
		if (irqcfg->handler) {
			ret = request_irq(iproc_gpio->irq, irqcfg->handler,
					irqcfg->flags, iproc_gpio->chip.label,
					iproc_gpio);
			if (ret) {
				dev_err(&pdev->dev, "Fail to request IRQ%d\n",
						iproc_gpio->irq);
				goto err_irq_request;
			}
		} else {
			dev_warn(&pdev->dev, "%s has no isr!\n",
					iproc_gpio->chip.label);
		}
	}

	return 0;

err_irq_request:
	irq_domain_remove(iproc_gpio->irq_domain);
	iproc_gpio->irq_domain = NULL;

err_irq_domain:
	gpiochip_remove(&iproc_gpio->chip);

	return ret;
}

static int __exit iproc_gpio_remove(struct platform_device *pdev)
{
	struct iproc_gpio_chip *iproc_gpio;

	iproc_gpio = platform_get_drvdata(pdev);
	if (iproc_gpio == NULL)
		return -ENODEV;

	if (iproc_gpio->intr_ioaddr) {
		/* Disable GPIO interrupts for CCA GPIO */
		if (iproc_gpio->id == IPROC_GPIO_CCA_ID) {
			u32 val;
			val = readl(iproc_gpio->intr_ioaddr + CCA_INT_MASK);
			val &= ~(CCA_INT_F_GPIOINT);
			writel(val, iproc_gpio->intr_ioaddr + CCA_INT_MASK);
		}
	}

	gpiochip_remove(&iproc_gpio->chip);

	return 0;
}

static struct platform_driver bcm_iproc_gpio_driver = {
	.driver = {
		.name = "iproc-gpio",
		.owner = THIS_MODULE,
		.of_match_table = bcm_iproc_gpio_of_match,
	},
	.probe = iproc_gpio_probe,
	.remove = iproc_gpio_remove,
};

module_platform_driver(bcm_iproc_gpio_driver);

MODULE_DESCRIPTION("XGS IPROC GPIO driver");
MODULE_LICENSE("GPL v2");
