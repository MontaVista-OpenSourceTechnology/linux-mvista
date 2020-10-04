/*
 * Keystone IPC GPIO support.
 *
 * Copyright (C) 2012 Texas Instruments, Inc.
 * Written by Murali Karicheri <m-karicheri2@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* 28 bits in IPCGRx are treated as GPIO pins to generate interrupt */
#define GPIOS_PER_BANK		28
#define GPIO_OFFSET		4

struct gpio_bank {
	void __iomem		*reg_base;
	struct gpio_chip	 chip;
	struct device		*dev;
};
#define chip_to_bank(c)	container_of(c, struct gpio_bank, chip)

static int keystone_direction_out(struct gpio_chip *c, unsigned ofs, int val)
{
	return 0;
}

static int keystone_gpio_get(struct gpio_chip *c, unsigned ofs)
{
	struct gpio_bank *bank = chip_to_bank(c);
	int bit = ofs + GPIO_OFFSET;
	return (__raw_readl(bank->reg_base) >> bit) & 1;
}

static void keystone_gpio_set(struct gpio_chip *c, unsigned ofs, int val)
{
	struct gpio_bank *bank = chip_to_bank(c);
	int bit = ofs + GPIO_OFFSET;

	if (val)
		__raw_writel(BIT(bit) | 1, bank->reg_base);
}

static int keystone_ipc_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct gpio_bank *bank;
	struct resource *res;
	int error = 0;

	bank = devm_kzalloc(&pdev->dev, sizeof(struct gpio_bank), GFP_KERNEL);
	if (!bank)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (unlikely(!res)) {
		dev_err(dev, "invalid register resource\n");
		return -ENODEV;
	}

	if (!devm_request_mem_region(dev, res->start, resource_size(res),
				     pdev->name)) {
		dev_err(dev, "register region already claimed\n");
		return -EBUSY;
	}

	bank->reg_base = devm_ioremap(dev, res->start, resource_size(res));
	if (!bank->reg_base) {
		dev_err(dev, "unable to map registers\n");
		return -EINVAL;
	}

	bank->dev			= dev;
#ifdef CONFIG_OF_GPIO
	bank->chip.of_node		= of_node_get(node);
#endif
	bank->chip.label		= dev_name(dev);
	bank->chip.get			= keystone_gpio_get;
	bank->chip.set			= keystone_gpio_set;
	bank->chip.direction_output	= keystone_direction_out;
	bank->chip.base			= -1;
	bank->chip.ngpio		= GPIOS_PER_BANK;

	error = gpiochip_add(&bank->chip);
	if (error) {
		dev_err(dev, "gpio chip registration failed\n");
		return error;
	}

	dev_dbg(bank->dev, "registered %d gpios\n", bank->chip.ngpio);
	platform_set_drvdata(pdev, bank);

	return error;
}

static const struct of_device_id keystone_ipc_gpio_dt_ids[] = {
	{ .compatible = "ti,keystone-ipc-gpio", },
	{ },
};

static struct platform_driver keystone_ipc_gpio_driver = {
	.driver		= {
		.name	= "keystone-ipc-gpio",
		.owner	= THIS_MODULE,
		.of_match_table = keystone_ipc_gpio_dt_ids,
	},
	.probe		= keystone_ipc_gpio_probe,
};

module_platform_driver(keystone_ipc_gpio_driver);

MODULE_AUTHOR("Murali Karicheri <m-karicheri2@ti.com>");
MODULE_DESCRIPTION("Texas Instruments Keystone IPC GPIO");
MODULE_LICENSE("GPL");
