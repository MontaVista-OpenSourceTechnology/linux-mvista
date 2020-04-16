/* rtc-ds1683.c
 *
 * Driver for Maxim/Dallas ds1683, I2C Compatible
 * Real Time Clock
 *
 * Author : Venkat Prashanth B U <venkat.prashanth2498@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/i2c.h>
#include <linux/rtc.h>
#include <linux/module.h>

/* Registers */

#define ds1683_REG_CNT_BASE	0
#define ds1683_REG_EVENT	4

#define ds1683_REG_CONTROL_EOSC	0x80

static struct i2c_driver ds1683_driver;

static int ds1683_get_datetime(struct i2c_client *client, struct rtc_time *tm)
{
	unsigned long time;
	unsigned char addr = ds1683_REG_CNT_BASE;
	unsigned char buf[4];

	struct i2c_msg msgs[] = {
		{	/* setup read pointer */
			.addr = client->addr,
			.len = 1,
			.buf = &addr
		},
		{/* read date */
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 4,
			.buf = buf
		},
	};

	/* read date registers */
	if ((i2c_transfer(client->adapter, &msgs[0], 2)) != 2) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		return -EIO;
	}

	time = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

	rtc_time_to_tm(time, tm);

	return 0;
}

static int ds1683_set_mmss(struct i2c_client *client, unsigned long secs)
{
	int xfer;
	unsigned char buf[6];

	buf[0] = ds1683_REG_CNT_BASE;
	buf[1] = secs & 0x000000FF;
	buf[2] = (secs & 0x0000FF00) >> 8;
	buf[3] = (secs & 0x00FF0000) >> 16;
	buf[4] = (secs & 0xFF000000) >> 24;
	buf[5] = 0;		/* set control reg to enable counting */

	xfer = i2c_master_send(client, buf, 6);
	if (xfer != 6) {
		dev_err(&client->dev, "%s: send: %d\n", __func__, xfer);
		return -EIO;
	}

	return 0;
}

static int ds1683_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	return ds1683_get_datetime(to_i2c_client(dev), tm);
}

static int ds1683_rtc_set_time(struct device *dev, struct rtc_time *dt)
{
	u32 time = cpu_to_le32(rtc_tm_to_time64(dt));
	return ds1683_set_mmss(to_i2c_client(dev), time);
}

static int ds1683_get_control(struct i2c_client *client, u8 *status)
{
	unsigned char addr = ds1683_REG_CONTROL_EOSC;

	struct i2c_msg msgs[] = {
		{/* setup read pointer */
			.addr = client->addr,
			.len = 1,
			.buf = &addr
		},
		{/* read control */
			.addr = client->addr,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = status
		},
	};

	/* read control register */
	if ((i2c_transfer(client->adapter, &msgs[0], 2)) != 2) {
		dev_err(&client->dev, "%s: read error\n", __func__);
		return -EIO;
	}

	return 0;
}

/* sysfs callback functions */
static ssize_t show_control(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	u8 control;
	int err;

	err = ds1683_get_control(client, &control);
	if (err)
		return err;

	return sprintf(buf, "%s\n", (control & ds1683_REG_CONTROL_EOSC)
		       ? "disabled" : "enabled");
}

static DEVICE_ATTR(control, 0444, show_control, NULL);

static const struct rtc_class_ops ds1683_rtc_ops = {
	.read_time = ds1683_rtc_read_time,
	.set_time = ds1683_rtc_set_time,
};

static int ds1683_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	u8 control;
	struct rtc_device *rtc;

	dev_dbg(&client->dev, "%s\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	rtc = devm_rtc_device_register(&client->dev, ds1683_driver.driver.name,
			&ds1683_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	i2c_set_clientdata(client, rtc);

	/* read control register */
	err = ds1683_get_control(client, &control);
	if (err)
		dev_warn(&client->dev, "Unable to read the control register\n");

	if (control & ds1683_REG_CONTROL_EOSC)
		dev_warn(&client->dev, "Oscillator not enabled Set time to enable.\n");

	/* Register sysfs hooks */
	err = device_create_file(&client->dev, &dev_attr_control);
	if (err)
		dev_err(&client->dev, "Unable to create sysfs entry: %s\n",
			dev_attr_control.attr.name);

	return 0;
}

static const struct i2c_device_id ds1683_id[] = {
	{ "ds1683", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ds1683_id);

static const struct of_device_id ds1683_of_match[] = {
	{ "dallas,ds1683", 0 },
	{ }
};
MODULE_DEVICE_TABLE(of, ds1683_of_match);

static struct i2c_driver ds1683_driver = {
	.driver = {
		   .name = "rtc-ds1683",
		   .of_match_table = of_match_ptr(ds1683_of_match),
	},
	.probe = &ds1683_probe,
	.id_table = ds1683_id,
};

module_i2c_driver(ds1683_driver);

MODULE_AUTHOR("Venkat Prashanth B U <venkat.prashanth2498@gmail.com>");
MODULE_DESCRIPTION("Dallas/Maxim ds1683 I2C driver");
MODULE_LICENSE("GPL");
