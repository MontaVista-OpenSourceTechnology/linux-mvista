// SPDX-License-Identifier: (GPL-2.0)
/*
 * Microchip PolarFire SoC (MPFS) GPIO controller driver
 *
 * Copyright (c) 2018-2022 Microchip Technology Inc. and its subsidiaries
 *
 * Author: Lewis Hanly <lewis.hanly@microchip.com>
 *
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define NUM_GPIO			32
#define MPFS_GPIO_X_CFG_EN_INT		3
#define MPFS_GPIO_X_CFG_BIT_GPIO_OE	2
#define MPFS_GPIO_X_CFG_BIT_EN_IN	1
#define MPFS_GPIO_X_CFG_BIT_EN_OUT	0

#define MPFS_GPIO_INTR_EDGE_BOTH_MASK		0x80
#define MPFS_GPIO_INTR_EDGE_NEGATIVE_MASK	0x60
#define MPFS_GPIO_INTR_EDGE_POSITIVE_MASK	0x40
#define MPFS_GPIO_INTR_LEVEL_LOW_MASK		0x20
#define MPFS_GPIO_INTR_LEVEL_HIGH_MASK		0x00
#define MPFS_GPIO_IRQ_MASK			GENMASK(31, 0)
#define IRQ_OFFSET				0x80
#define INP_OFFSET				0x84
#define OUTP_OFFSET				0x88

struct mpfs_gpio_chip {
	spinlock_t lock; /* lock */
	struct gpio_chip gc;
	struct clk *clk;
	void __iomem *base;
	unsigned int irq_parent[NUM_GPIO];
};

static void mpfs_gpio_assign_bit(void __iomem *base_addr,
				 int bit_offset, int value)
{
	u32 output = readl(base_addr);

	if (value)
		output |= BIT(bit_offset);
	else
		output &= ~BIT(bit_offset);

	writel(output, base_addr);
}

static int mpfs_gpio_direction_input(struct gpio_chip *gc,
				     unsigned int gpio_index)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	u32 gpio_cfg;
	unsigned long flags;

	if (gpio_index >= gc->ngpio)
		return -EINVAL;

	spin_lock_irqsave(&mpfs_gpio->lock, flags);

	gpio_cfg = readl(mpfs_gpio->base + (gpio_index << 2));
	gpio_cfg |= BIT(MPFS_GPIO_X_CFG_BIT_EN_IN);
	gpio_cfg &= ~(BIT(MPFS_GPIO_X_CFG_BIT_EN_OUT) |
		BIT(MPFS_GPIO_X_CFG_BIT_GPIO_OE));
	writel(gpio_cfg, mpfs_gpio->base + (gpio_index << 2));

	spin_unlock_irqrestore(&mpfs_gpio->lock, flags);

	return 0;
}

static int mpfs_gpio_direction_output(struct gpio_chip *gc,
				      unsigned int gpio_index, int value)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	u32 gpio_cfg;
	unsigned long flags;

	if (gpio_index >= gc->ngpio)
		return -EINVAL;

	spin_lock_irqsave(&mpfs_gpio->lock, flags);

	gpio_cfg = readl(mpfs_gpio->base + (gpio_index << 2));
	gpio_cfg |= BIT(MPFS_GPIO_X_CFG_BIT_EN_OUT) |
		BIT(MPFS_GPIO_X_CFG_BIT_GPIO_OE);
	gpio_cfg &= ~BIT(MPFS_GPIO_X_CFG_BIT_EN_IN);
	writel(gpio_cfg, mpfs_gpio->base + (gpio_index << 2));

	mpfs_gpio_assign_bit(mpfs_gpio->base + OUTP_OFFSET, gpio_index, value);

	spin_unlock_irqrestore(&mpfs_gpio->lock, flags);

	return 0;
}

static int mpfs_gpio_get_direction(struct gpio_chip *gc,
				   unsigned int gpio_index)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	u32 gpio_cfg;
	int result = 0;

	if (gpio_index >= gc->ngpio)
		return -EINVAL;

	gpio_cfg = readl(mpfs_gpio->base + (gpio_index << 2));

	if (gpio_cfg & BIT(MPFS_GPIO_X_CFG_BIT_EN_IN))
		result = 1;

	return result;
}

static int mpfs_gpio_get_value(struct gpio_chip *gc,
			       unsigned int gpio_index)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);

	if (gpio_index >= gc->ngpio)
		return -EINVAL;

	return !!(readl(mpfs_gpio->base + INP_OFFSET) & BIT(gpio_index));
}

static void mpfs_gpio_set_value(struct gpio_chip *gc,
				unsigned int gpio_index, int value)
{
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	unsigned long flags;

	if (gpio_index >= gc->ngpio)
		return;

	spin_lock_irqsave(&mpfs_gpio->lock, flags);

	mpfs_gpio_assign_bit(mpfs_gpio->base + OUTP_OFFSET,
			     gpio_index, value);

	spin_unlock_irqrestore(&mpfs_gpio->lock, flags);
}

static int microchip_mpfs_gpio_irq_set_type(struct irq_data *data,
					    unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	int gpio_index = irqd_to_hwirq(data);
	u32 interrupt_type;
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	u32 gpio_cfg;
	unsigned long flags;

	if (gpio_index >= gc->ngpio)
		return -EINVAL;

	switch (type) {
	case IRQ_TYPE_EDGE_BOTH:
		interrupt_type = MPFS_GPIO_INTR_EDGE_BOTH_MASK;
		break;

	case IRQ_TYPE_EDGE_FALLING:
		interrupt_type = MPFS_GPIO_INTR_EDGE_NEGATIVE_MASK;
		break;

	case IRQ_TYPE_EDGE_RISING:
		interrupt_type = MPFS_GPIO_INTR_EDGE_POSITIVE_MASK;
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		interrupt_type = MPFS_GPIO_INTR_LEVEL_HIGH_MASK;
		break;

	case IRQ_TYPE_LEVEL_LOW:
	default:
		interrupt_type = MPFS_GPIO_INTR_LEVEL_LOW_MASK;
		break;
	}

	spin_lock_irqsave(&mpfs_gpio->lock, flags);

	gpio_cfg = readl(mpfs_gpio->base + (gpio_index << 2));
	gpio_cfg |= interrupt_type;
	writel(gpio_cfg, mpfs_gpio->base + (gpio_index << 2));

	spin_unlock_irqrestore(&mpfs_gpio->lock, flags);

	return 0;
}

/* chained_irq_{enter,exit} already mask the parent */
static void microchip_mpfs_gpio_irq_mask(struct irq_data *data)
{
}

static void microchip_mpfs_gpio_irq_unmask(struct irq_data *data)
{
}

static void microchip_mpfs_gpio_irq_enable(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	int gpio_index = irqd_to_hwirq(data) % NUM_GPIO;

	mpfs_gpio_direction_input(gc, gpio_index);
	mpfs_gpio_assign_bit(mpfs_gpio->base + IRQ_OFFSET, gpio_index, 1);
	mpfs_gpio_assign_bit(mpfs_gpio->base + (gpio_index << 2),
			     MPFS_GPIO_X_CFG_EN_INT, 1);
}

static void microchip_mpfs_gpio_irq_disable(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct mpfs_gpio_chip *mpfs_gpio = gpiochip_get_data(gc);
	int gpio_index = irqd_to_hwirq(data) % NUM_GPIO;

	mpfs_gpio_assign_bit(mpfs_gpio->base + IRQ_OFFSET, gpio_index, 1);
	mpfs_gpio_assign_bit(mpfs_gpio->base + (gpio_index << 2),
			     MPFS_GPIO_X_CFG_EN_INT, 0);
}

static struct irq_chip mpfs_gpio_irqchip = {
	.name = "microchip_mpfs_gpio",
	.irq_set_type = microchip_mpfs_gpio_irq_set_type,
	.irq_mask = microchip_mpfs_gpio_irq_mask,
	.irq_unmask = microchip_mpfs_gpio_irq_unmask,
	.irq_enable = microchip_mpfs_gpio_irq_enable,
	.irq_disable = microchip_mpfs_gpio_irq_disable,
	.flags = IRQCHIP_MASK_ON_SUSPEND,
};

static void microchip_mpfs_gpio_irq_handler(struct irq_desc *desc)
{
	struct mpfs_gpio_chip *mpfs_gpio =
	    gpiochip_get_data(irq_desc_get_handler_data(desc));
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned long status;
	int offset;

	chained_irq_enter(irqchip, desc);
	status = readl(mpfs_gpio->base + IRQ_OFFSET) & MPFS_GPIO_IRQ_MASK;
	for_each_set_bit(offset, &status, mpfs_gpio->gc.ngpio)
		generic_handle_irq(irq_find_mapping(mpfs_gpio->gc.irq.domain, offset));

	chained_irq_exit(irqchip, desc);
}

static irqreturn_t mpfs_gpio_irq_handler(int irq, void *mpfs_gpio_data)
{
	struct mpfs_gpio_chip *mpfs_gpio = mpfs_gpio_data;
	unsigned long status;
	int offset;

	status = readl(mpfs_gpio->base + IRQ_OFFSET) & MPFS_GPIO_IRQ_MASK;

	for_each_set_bit(offset, &status, mpfs_gpio->gc.ngpio) {
		mpfs_gpio_assign_bit(mpfs_gpio->base + IRQ_OFFSET, offset, 1);
		generic_handle_irq(irq_find_mapping(mpfs_gpio->gc.irq.domain, offset));
	}
	return IRQ_HANDLED;
}

static int mpfs_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = pdev->dev.of_node;
	struct mpfs_gpio_chip *mpfs_gpio;
	int gpio_index, irq, ret, ngpio;
	struct gpio_irq_chip *irq_c;
	struct clk *clk;
	int irq_base = 0;

	mpfs_gpio = devm_kzalloc(dev, sizeof(*mpfs_gpio), GFP_KERNEL);
	if (!mpfs_gpio)
		return -ENOMEM;

	mpfs_gpio->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mpfs_gpio->base)) {
		dev_err(dev, "failed to allocate device memory\n");
		return PTR_ERR(mpfs_gpio->base);
	}
	clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(clk)) {
		dev_err(&pdev->dev, "failed to get clock\n");
		return PTR_ERR(clk);
	}

	ret = clk_prepare_enable(clk);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "failed to enable clock\n");

	mpfs_gpio->clk = clk;
	ngpio = of_irq_count(node);
	if (ngpio > NUM_GPIO) {
		dev_err(dev, "too many interrupts\n");
		goto cleanup_clock;
	}

	spin_lock_init(&mpfs_gpio->lock);

	mpfs_gpio->gc.direction_input = mpfs_gpio_direction_input;
	mpfs_gpio->gc.direction_output = mpfs_gpio_direction_output;
	mpfs_gpio->gc.get_direction = mpfs_gpio_get_direction;
	mpfs_gpio->gc.get = mpfs_gpio_get_value;
	mpfs_gpio->gc.set = mpfs_gpio_set_value;
	mpfs_gpio->gc.base = -1;
	mpfs_gpio->gc.ngpio = ngpio;
	mpfs_gpio->gc.label = dev_name(dev);
	mpfs_gpio->gc.parent = dev;
	mpfs_gpio->gc.owner = THIS_MODULE;

	irq_c = &mpfs_gpio->gc.irq;
	irq_c->chip = &mpfs_gpio_irqchip;
	irq_c->chip->parent_device = dev;
	irq_c->handler = handle_simple_irq;
	irq_c->default_type = IRQ_TYPE_NONE;
	irq_c->num_parents = 0;
	irq_c->parents = devm_kcalloc(&pdev->dev, 1,
				      sizeof(*irq_c->parents), GFP_KERNEL);
	if (!irq_c->parents) {
		ret = -ENOMEM;
		goto cleanup_clock;
	}

	irq = platform_get_irq(pdev, 0);
	irq_c->parents[0] = irq;

	irq_base = devm_irq_alloc_descs(mpfs_gpio->gc.parent,
					-1, 0, ngpio, 0);
	if (irq_base < 0) {
		dev_err(mpfs_gpio->gc.parent, "Couldn't allocate IRQ numbers\n");
		ret = -ENODEV;
		goto cleanup_clock;
	}
	irq_c->first = irq_base;

	ret = gpiochip_add_data(&mpfs_gpio->gc, mpfs_gpio);
	if (ret)
		goto cleanup_clock;

	ret = devm_request_irq(mpfs_gpio->gc.parent, irq,
			       mpfs_gpio_irq_handler,
			       IRQF_SHARED, pdev->name, mpfs_gpio);
	if (ret) {
		dev_err(dev, "Microchip MPFS GPIO devm_request_irq failed\n");
		goto cleanup_gpiochip;
	}

	/* Disable all GPIO interrupts */
	for (gpio_index = 0; gpio_index < ngpio; gpio_index++) {
		u32 gpio_cfg;
		unsigned long flags;

		spin_lock_irqsave(&mpfs_gpio->lock, flags);

		gpio_cfg = readl(mpfs_gpio->base + (gpio_index << 2));
		gpio_cfg &= ~(BIT(MPFS_GPIO_X_CFG_EN_INT));
		writel(gpio_cfg, mpfs_gpio->base + (gpio_index << 2));

		spin_unlock_irqrestore(&mpfs_gpio->lock, flags);
	}

	platform_set_drvdata(pdev, mpfs_gpio);
	dev_info(dev, "Microchip MPFS GPIO registered %d GPIO%s\n", ngpio, ngpio ? "s" : "");

	return 0;
cleanup_gpiochip:
	gpiochip_remove(&mpfs_gpio->gc);

cleanup_clock:
	clk_disable_unprepare(mpfs_gpio->clk);
	return ret;
}

static int mpfs_gpio_remove(struct platform_device *pdev)
{
	struct mpfs_gpio_chip *mpfs_gpio = platform_get_drvdata(pdev);

	gpiochip_remove(&mpfs_gpio->gc);
	clk_disable_unprepare(mpfs_gpio->clk);

	return 0;
}

static const struct of_device_id mpfs_gpio_match[] = {
	{ .compatible = "microchip,mpfs-gpio", },
	{ /* end of list */ },
};

static struct platform_driver mpfs_gpio_driver = {
	.probe = mpfs_gpio_probe,
	.driver = {
		.name = "microchip,mpfs-gpio",
		.of_match_table = of_match_ptr(mpfs_gpio_match),
	},
	.remove = mpfs_gpio_remove,
};

builtin_platform_driver(mpfs_gpio_driver);
