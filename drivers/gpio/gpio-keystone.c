/*
 * Keystone GPIO support.
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 * Written by Murali Karicheri <m-karicheri2@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <asm-generic/bug.h>

#define	GPIO_MAX_BANKS		4
#define GPIOS_PER_HW_BANK	16
#define GPIOS_PER_BANK		(GPIOS_PER_HW_BANK * 2)
#define GPIO_NAME		"gpio-keystone"
#define GPIO_IRQ_EN		(BIT(1) | BIT(0))

struct gpio_regs {
	u32	dir;
	u32	out_data;
	u32	set_data;
	u32	clr_data;
	u32	in_data;
	u32	set_rise_trig;
	u32	clr_rise_trig;
	u32	set_fal_trig;
	u32	clr_fal_trig;
	u32	intstat;
};

static struct gpio_regs gpio_bank_hw_reg = {
	.dir		= 0x00,
	.out_data	= 0x04,
	.set_data	= 0x08,
	.clr_data	= 0x0c,
	.in_data	= 0x10,
	.set_rise_trig	= 0x14,
	.clr_rise_trig	= 0x18,
	.set_fal_trig	= 0x1c,
	.clr_fal_trig	= 0x20,
	.intstat	= 0x24,
};

struct gpio_bank {
	void __iomem *reg_base;
	spinlock_t lock;
	struct gpio_chip chip;
	struct device *dev;
	struct clk *clk;
	struct gpio_regs *regs;
	int id;
	/* GPIO base */
	int base;
	/* IRQ related */
	struct irq_domain *irqdomain;
	/* Host side hw irq array for each of gpio line */
	int hw_irqs[GPIOS_PER_BANK];
};

/* BINTEN is common to all banks */
#define BINTEN_OFFSET		0x8

static void __iomem *gpio_base;

static const struct of_device_id keystone_gpio_dt_ids[] = {
	{ .compatible = "ti,keystone-gpio", },
	{ },
};

static int keystone_direction(struct gpio_bank *bank,
			unsigned offset, bool out, int value)
{
	struct gpio_regs *regs = bank->regs;
	u32 temp, mask = 1 << offset;
	unsigned long flags;

	spin_lock_irqsave(&bank->lock, flags);
	temp = __raw_readl(bank->reg_base + regs->dir);
	if (out) {
		temp &= ~mask;
		__raw_writel(mask, value ? (bank->reg_base + regs->set_data) :
					(bank->reg_base + regs->clr_data));
	} else {
		temp |= mask;
	}
	__raw_writel(temp, bank->reg_base + regs->dir);
	spin_unlock_irqrestore(&bank->lock, flags);

	return 0;
}

static int keystone_direction_in(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;

	bank = container_of(chip, struct gpio_bank, chip);
	return keystone_direction(bank, offset, false, 0);
}

static int
keystone_direction_out(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_bank *bank;

	bank = container_of(chip, struct gpio_bank, chip);
	return keystone_direction(bank, offset, true, value);
}

static int keystone_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank;
	struct gpio_regs *regs;

	bank = container_of(chip, struct gpio_bank, chip);
	regs = bank->regs;

	return (1 << offset) & __raw_readl(bank->reg_base + regs->in_data);
}

static void
keystone_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct gpio_bank *bank;
	struct gpio_regs *regs;

	bank = container_of(chip, struct gpio_bank, chip);
	regs = bank->regs;
	__raw_writel((1 << offset), value ? (bank->reg_base + regs->set_data) :
				(bank->reg_base + regs->clr_data));
};

static inline int keystone_hwirq_to_gpio(struct gpio_bank *bank, int hwirq)
{
	int gpio;

	/* This may need change if Host side IRQs are not contiguous */
	if (hwirq < bank->hw_irqs[0])
		return -EINVAL;

	gpio = hwirq - bank->hw_irqs[0];
	if (gpio >= GPIOS_PER_BANK)
		return -EINVAL;

	return gpio;
}

static void keystone_irq_handler(struct irq_desc *desc)
{
	struct gpio_bank *bank = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned int irq = irq_desc_get_irq(desc);
	struct gpio_regs *regs = bank->regs;
	u32 mask = 1, status;
	int gpio, virq;

	gpio = keystone_hwirq_to_gpio(bank, irq);
	if (gpio < 0)
		return;

	mask <<= gpio;
	/* temporarily mask (level sensitive) parent IRQ */
	chip->irq_mask(&desc->irq_data);
	if (chip->irq_ack)
		chip->irq_ack(&desc->irq_data);
	if (chip->irq_eoi)
		chip->irq_eoi(&desc->irq_data);

	status = __raw_readl(bank->reg_base + regs->intstat) & mask;
	if (status) {
		__raw_writel(status, bank->reg_base + regs->intstat);
		virq = irq_linear_revmap(bank->irqdomain, gpio);
		generic_handle_irq(virq);
	}
	chip->irq_unmask(&desc->irq_data);
}

static int keystone_gpio_to_irq(struct gpio_chip *chip, unsigned offset)
{
	struct gpio_bank *bank = container_of(chip, struct gpio_bank, chip);

	if (bank->irqdomain && offset < GPIOS_PER_BANK)
		return irq_linear_revmap(bank->irqdomain, offset);
	else
		return -ENXIO;
}

static void keystone_setup_irq(struct gpio_bank *bank)
{
	u32 val = 3 << bank->id;
	int hw_irq;

	for (hw_irq = 0; hw_irq < GPIOS_PER_BANK; hw_irq++) {
		irq_set_handler_data(bank->hw_irqs[hw_irq], bank);
		irq_set_chained_handler(bank->hw_irqs[hw_irq],
					keystone_irq_handler);
	}
	__raw_writel(val, gpio_base + BINTEN_OFFSET);

}

static void gpio_irq_disable(struct irq_data *d)
{
	struct gpio_bank *bank = irq_data_get_irq_chip_data(d);
	struct gpio_regs *regs = bank->regs;
	u32 mask;
	int gpio;

	gpio = d->hwirq - bank->base;
	mask = 1 << gpio;
	__raw_writel(mask, bank->reg_base + regs->clr_fal_trig);
	__raw_writel(mask, bank->reg_base + regs->clr_rise_trig);
}

static void gpio_irq_enable(struct irq_data *d)
{
	struct gpio_bank *bank = irq_data_get_irq_chip_data(d);
	u32 mask, status = irqd_get_trigger_type(d);
	struct gpio_regs *regs = bank->regs;
	int gpio;

	gpio = d->hwirq - bank->base;
	mask = 1 << gpio;

	if (status & IRQ_TYPE_EDGE_FALLING)
		__raw_writel(mask, bank->reg_base + regs->set_fal_trig);
	if (status & IRQ_TYPE_EDGE_RISING)
		__raw_writel(mask, bank->reg_base + regs->set_rise_trig);
}

static int gpio_irq_type(struct irq_data *d, unsigned trigger)
{
	if (trigger & ~(IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING))
		return -EINVAL;

	return 0;
}

static struct irq_chip keystone_gpio_irqchip = {
	.name		= GPIO_NAME,
	.irq_enable	= gpio_irq_enable,
	.irq_disable	= gpio_irq_disable,
	.irq_set_type	= gpio_irq_type,
	.flags		= IRQCHIP_SET_TYPE_MASKED,
};

static int keystone_gpio_irq_map(struct irq_domain *h, unsigned int virq,
				irq_hw_number_t hw)
{
	struct gpio_bank *bank = h->host_data;

	irq_set_chip_data(virq, bank);
	irq_set_chip_and_handler(virq, &keystone_gpio_irqchip,
				handle_simple_irq);
	irq_set_probe(virq);
	irq_set_irq_type(virq, IRQ_TYPE_NONE);

	return 0;
}

static struct irq_domain_ops keystone_gpio_irq_ops = {
	.map	= keystone_gpio_irq_map,
	.xlate	= irq_domain_xlate_twocell,
};

static void keystone_gpio_setup(struct gpio_bank *bank)
{
	int i;

	bank->chip.label = GPIO_NAME;
	bank->chip.direction_input = keystone_direction_in;
	bank->chip.get = keystone_gpio_get;
	bank->chip.direction_output = keystone_direction_out;
	bank->chip.set = keystone_gpio_set;
	bank->chip.base = bank->base;
	bank->chip.ngpio = GPIOS_PER_BANK;
	spin_lock_init(&bank->lock);
	bank->chip.to_irq = keystone_gpio_to_irq;
	gpiochip_add(&bank->chip);

	/* Set up all GPIO lines to be input */
	for (i = 0; i < GPIOS_PER_BANK; i++)
		keystone_direction_in(&bank->chip, i);

	/* setup IRQ chip controller */
	keystone_setup_irq(bank);
}

static void __iomem *keystone_gpio_iomap(struct platform_device *pdev,
			unsigned int type, unsigned int index, int *ret)
{
	struct device *dev = &pdev->dev;
	struct resource *res;

	*ret = 0;
	/* Static mapping, never released */
	res = platform_get_resource(pdev, type, index);
	if (unlikely(!res)) {
		dev_err(dev, "Invalid mem resource\n");
		*ret = -ENODEV;
		return NULL;
	}

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     pdev->name)) {
		dev_err(dev, "Region already claimed\n");
		*ret = -EBUSY;
		return NULL;
	}

	return devm_ioremap(dev, res->start, resource_size(res));
}

static int keystone_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct gpio_bank *bank;
	int ret = 0, id, irq;

	bank = devm_kzalloc(&pdev->dev, sizeof(struct gpio_bank), GFP_KERNEL);
	if (!bank) {
		dev_err(dev, "Memory alloc failed\n");
		return -ENOMEM;
	}

	bank->clk = devm_clk_get(dev, "gpio");
	if (IS_ERR(bank->clk)) {
		dev_err(dev, "Error %ld getting gpio clock?\n",
		       PTR_ERR(bank->clk));
		return PTR_ERR(bank->clk);
	}
	clk_prepare_enable(bank->clk);

	/* bank id */
	id = of_alias_get_id(node, "gpio");
	if (id < 0) {
		ret = -EINVAL;
		goto err_enable;
	}

	if (id == 0) {
		/*
		 * gpio base address is mapped for BINTEN register address
		 * that is common across all banks
		 */
		gpio_base = keystone_gpio_iomap(pdev, IORESOURCE_MEM, 0, &ret);
		if (!gpio_base) {
			ret = -EINVAL;
			goto err_enable;
		}

		/* map the bank base */
		bank->reg_base = keystone_gpio_iomap(pdev, IORESOURCE_MEM, 1,
							 &ret);
		if (!bank->reg_base) {
			ret = -EINVAL;
			goto err_enable;
		}
	} else {
		/*
		 * bank0 should have been setup already and gpio_base to be
		 * iomapped. For subsequent banks, just iomap the bank base
		 */
		bank->reg_base = keystone_gpio_iomap(pdev, IORESOURCE_MEM, 0,
							&ret);
		if (!bank->reg_base) {
			ret = -EINVAL;
			goto err_enable;
		}
	}
	bank->id = id;
	bank->base = id * GPIOS_PER_BANK;
	bank->dev = dev;
#ifdef CONFIG_OF_GPIO
	bank->chip.of_node = of_node_get(node);
#endif
	platform_set_drvdata(pdev, bank);
	bank->regs = &gpio_bank_hw_reg;

	/* parse and map parent irqs directly mapped to gpio irqs */
	for (irq = 0; irq < GPIOS_PER_BANK; irq++) {
		bank->hw_irqs[irq] =  irq_of_parse_and_map(node, irq);
		if (bank->hw_irqs[irq] < 0) {
			ret = -EINVAL;
			dev_err(dev,
				"interrupt not defined for the irq index %d\n",
				irq);
			goto err_enable;
		}
	}

	bank->irqdomain = irq_domain_add_linear(node, GPIOS_PER_BANK,
					&keystone_gpio_irq_ops, bank);
	if (!bank->irqdomain) {
		dev_err(dev, "IRQ domain registration failed\n");
		ret = -ENODEV;
		goto err_enable;
	}

	/* Map the gpio hw irqs */
	for (irq = 0; irq < GPIOS_PER_BANK; irq++)
		irq_create_mapping(bank->irqdomain, irq);

	keystone_gpio_setup(bank);

	return ret;

err_enable:
	clk_disable_unprepare(bank->clk);

	return ret;
}

static struct platform_driver keystone_gpio_driver = {
	.driver		= {
		.name	= GPIO_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = keystone_gpio_dt_ids,
	},
	.probe		= keystone_gpio_probe,
};

static int __init gpio_keystone_init(void)
{
	return platform_driver_register(&keystone_gpio_driver);
}
postcore_initcall(gpio_keystone_init);

MODULE_AUTHOR("Murali Karicheri <m-karicheri2@ti.com>");
MODULE_DESCRIPTION("Texas Instruments Keystone GPIO");
MODULE_LICENSE("GPL");
