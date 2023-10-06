// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 ROHM Semiconductors
// gpio-tr8mcu.c ROHM TR8MCUMWV gpio driver

#include <linux/gpio/driver.h>
#include <linux/mfd/seco-trizeps8mcu.h>
#include <linux/module.h>
#include <linux/platform_device.h>

struct tr8mcu_gpio {
	struct tr8mcu		*parent;
	struct gpio_chip	chip;
	const int			*gpio_to_pin;

	//struct irq_chip		irqchip;
	//struct mutex		lock;		/* protect 'out' */
	//unsigned		out;		/* software latch */
	//unsigned		status;		/* current status */
	//unsigned int		irq_parent;
	//unsigned		irq_enabled;	/* enabled irqs */
};

#define NGPIOS 11
static const int default_gpio_to_pin[NGPIOS] = {
/*  0  1  2  3   4   5   6   7   8   9  10 */
	2, 4, 6, 8, 14, 16, 18, 20, 87, 97, 99
};

static int tr8mcu_get_pin(struct gpio_chip *chip, unsigned int offset)
{
	struct tr8mcu_gpio *tr8mcu_gpio = gpiochip_get_data(chip);
	struct device * dev = tr8mcu_gpio->chip.parent;
	int pin;

	if( offset > tr8mcu_gpio->chip.ngpio) goto err;

	// Map gpio 0-X to the MCU's pins
	pin = tr8mcu_gpio->gpio_to_pin[offset];
	if( pin == -1 ) goto err;

	dev_info(dev, "Mapped offset %d to pin %d\n", offset, pin);
	return pin;
err:
	dev_err(dev, "Offset %d is not mapped\n", offset);
	return -ENOENT;

}

static int tr8mcu_set_direction(struct gpio_chip *chip, unsigned int offset,
								int direction, int value)
{
	struct tr8mcu_gpio *tr8mcu_gpio = gpiochip_get_data(chip);
	struct tr8mcu *tr8mcu = tr8mcu_gpio->parent;
	struct device * dev = tr8mcu_gpio->chip.parent;
	int pin, ret;
	u32 data;

	dev_info(dev, "%s %d: dir %d offset %d value %d\n",
			 __func__, __LINE__, direction, offset, value);

	pin = tr8mcu_get_pin( chip, offset);
	if( pin < 0) return pin;

	data = pin;

	//  1; /* Alt1=Input  */
	// 21; /* Alt1-Output High */
	// 20; /* Alt1-Output Low  */
	if( direction == GPIO_LINE_DIRECTION_IN)
		data |= ( 1) << 8;
	else // GPIO_LINE_DIRECTION_OUT
		data |= ( 20 + ( !! value )) << 8;

	ret = tr8mcu_write(tr8mcu, TR8MCU_REG_PIN_CONFIG, 2, data);
	if(ret)
		dev_err(dev, "Failed to set direction %s for pin %d level %d\n",
				  direction == GPIO_LINE_DIRECTION_IN ? "input" : "output",
				  offset, value);

	return ret;
}

static int tr8mcu_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return tr8mcu_set_direction(chip, offset, GPIO_LINE_DIRECTION_IN, 0);
}

static int tr8mcu_direction_output(struct gpio_chip *chip, unsigned int offset,
				    int value)
{
	return tr8mcu_set_direction(chip, offset, GPIO_LINE_DIRECTION_OUT, value);
}


static void tr8mcu_gpio_set(struct gpio_chip *chip, unsigned int offset,
			     int value)
{
	struct tr8mcu_gpio *tr8mcu_gpio = gpiochip_get_data(chip);
	struct device * dev = tr8mcu_gpio->chip.parent;
	int ret;

	dev_info(dev, "%s %d: offset %d value %d\n",
			 __func__, __LINE__, offset, value);
	// From kuk_tr8mci_touch driver
	// There seems to be a bug in the firmware of the so direction output is
	// used instead
	ret = tr8mcu_direction_output(chip, offset, value);

}

static int tr8mcu_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct tr8mcu_gpio *tr8mcu_gpio = gpiochip_get_data(chip);
	struct tr8mcu *tr8mcu = tr8mcu_gpio->parent;
	struct device * dev = tr8mcu_gpio->chip.parent;
	int pin, ret;
	u32 data;

	dev_info(dev, "%s %d: offset %d \n",
			 __func__, __LINE__, offset);

	pin = tr8mcu_get_pin( chip, offset);
	if( pin < 0) return pin;

	ret = tr8mcu_read_with_param(tr8mcu, TR8MCU_REG_PIN_GET, pin, 1, &data);

	dev_info(dev, "%s %d: offset %d pin %d, level %d, ret %d\n",
			 __func__, __LINE__, offset, pin, data, ret);

	if (ret) {
		dev_err(dev, "Unable to fetch MCU-pin: %d err: %d\n", pin, ret);
		return 0;
	}
	return data;
}

static int tr8mcu_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tr8mcu_gpio *tr8mcu_gpio;
	struct tr8mcu *tr8mcu;
	const struct device_node * np = pdev->dev.of_node;
	int ret, count;
	int *gpio_map;

	dev_info(dev, "%s %d\n", __func__, __LINE__);

	tr8mcu = dev_get_drvdata(pdev->dev.parent);
	if (!tr8mcu) {
		dev_err(dev, "No MFD driver data\n");
		return -EINVAL;
	}

	tr8mcu_gpio = devm_kzalloc(dev, sizeof(*tr8mcu_gpio),
			      GFP_KERNEL);
	if (!tr8mcu_gpio)
		return -ENOMEM;

	tr8mcu_gpio->parent = tr8mcu;
	tr8mcu_gpio->chip.parent = dev;
	tr8mcu_gpio->chip.label = "tr8mcu-gpio";
	tr8mcu_gpio->chip.owner = THIS_MODULE;
	tr8mcu_gpio->chip.can_sleep = true;
	tr8mcu_gpio->chip.direction_input = tr8mcu_direction_input;
	tr8mcu_gpio->chip.direction_output = tr8mcu_direction_output;
	tr8mcu_gpio->chip.get = tr8mcu_gpio_get;
	tr8mcu_gpio->chip.set = tr8mcu_gpio_set;
	tr8mcu_gpio->chip.base = -1;

	count = of_property_count_u32_elems(np, "gpio-mapping");
	if(count > 0)
	{
		dev_info(dev, "%d gpios configured\n", count);
		gpio_map = devm_kzalloc(dev, count * sizeof(int), GFP_KERNEL);
		if(!gpio_map){
			dev_err(dev, "Failed to allocate memory.");
			return -ENOMEM;
		}

		if(of_property_read_u32_array(np, "gpio-mapping", gpio_map, count))
		{
			dev_err(dev, "Failed read devicetree property.");
			return -ENOENT;
		}

		tr8mcu_gpio->gpio_to_pin = gpio_map;
		tr8mcu_gpio->chip.ngpio = count;

	}
	else{
		dev_info(dev, "Mapping for gpios not configured, using default\n");
		tr8mcu_gpio->chip.ngpio = NGPIOS;
		tr8mcu_gpio->gpio_to_pin = default_gpio_to_pin;
	}

	ret = devm_gpiochip_add_data(dev, &tr8mcu_gpio->chip,
				     tr8mcu_gpio);
	if (ret)
		dev_err(dev, "gpio_init: Failed to add tr8mcu-gpios\n");

	return ret;
}

static const struct of_device_id tr8mcu_gpio_of_match[] = {
	{ .compatible = "seco,tr8mcu-gpio", },
	{ },
};

MODULE_DEVICE_TABLE(of, tr8mcu_gpio_of_match);

static struct platform_driver tr8mcu_gpio = {
	.driver = {
		.name = "tr8mcu-gpios",
		.of_match_table = tr8mcu_gpio_of_match,
	},
	.probe = tr8mcu_gpio_probe,
};

module_platform_driver(tr8mcu_gpio);

MODULE_AUTHOR("Jonas Höppner <jonas.hoeppner@seco.com>");
MODULE_DESCRIPTION("Trizeps8 MCU driver gpio");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tr8mcu-gpio");
