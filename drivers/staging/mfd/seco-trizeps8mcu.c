// SPDX-License-Identifier: GPL-2.0-or-later
//
// Copyright (C) 2023 SECO Nothern Europe
//
//
// Multifunction device driver for Trizeps8 MCU

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <linux/mfd/seco-trizeps8mcu.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/types.h>

struct tr8mcu {
	struct i2c_client * client;
	struct mfd_cell * mfd_cells;
};

/*== Read Write =======================================================*/

static int tr8mcu_readwrite(struct i2c_client *client,
				   u16 wr_len, u8 *wr_buf,
				   u16 rd_len, u8 *rd_buf)
{
	//struct device dev = client->dev;
	struct i2c_msg wrmsg[2];
	int i = 0;
	int ret;
	// TODO do we need a mutex here 

	if (wr_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = 0;
		wrmsg[i].len = wr_len;
		wrmsg[i].buf = wr_buf;
		i++;
	}
	if (rd_len) {
		wrmsg[i].addr  = client->addr;
		wrmsg[i].flags = I2C_M_RD;
		wrmsg[i].len = rd_len;
		wrmsg[i].buf = rd_buf;
		i++;
	}

	dev_info(&client->dev, "%s %d: write buf: 0x%02X, 0x%02X, 0x%02X\n",
			 __func__, __LINE__, wr_buf[0], wr_buf[1], wr_buf[2]);

	ret = i2c_transfer(client->adapter, wrmsg, i);
	if (ret < 0)
		return ret;
	if (ret != i)
		return -EIO;
	if(rd_buf)
			dev_info(&client->dev, "%s %d: read buf: 0x%02X, 0x%02X, 0x%02X\n",
			 __func__, __LINE__, rd_buf[0], rd_buf[1], rd_buf[2]);

	return 0;
}

int tr8mcu_read_with_param(struct tr8mcu * tr8mcu, u8 reg, u8 param, int len,
						   u32 * data)
{
	struct i2c_client *client = tr8mcu->client;
	u32 readdata = 0, mask;
	u8 writedata[2];
	int error;

	dev_info(&client->dev, "%s %d: 0x%02X, len %d\n",
			 __func__, __LINE__, reg, len);

	if( len == 0 || len > 4 )
	{
		dev_err_ratelimited(&client->dev,
				"tr8mcu_write: reading %d bytes is not supported\n", len);
		return -EPERM;
	}

	writedata[0] = reg;
	writedata[1] = param;

	error = tr8mcu_readwrite( client, 2, writedata, len, (u8*)&readdata);
	if (error) {
		dev_err_ratelimited(&client->dev,
				"tr8mcu_read: Unable to fetch %d bytes, error: %d\n",
				len, error);
		return error;
	}
	dev_info(&client->dev, "%s %d: 0x%02X, len %d: 0x%02X\n",
			 __func__, __LINE__, reg, len, readdata);

	// Mask out unread bits
	mask = 0xFFFFFFFF >> ( (4 - len) * 8 );
	readdata &= mask;

	dev_info(&client->dev, "%s %d: 0x%02X, len %d: 0x%02X, 0x%08X\n",
			 __func__, __LINE__, reg, len, readdata, mask);
	*data = readdata;
	return 0;
}

int tr8mcu_read(struct tr8mcu * tr8mcu, u8 reg, int len, u32 * data)
{
	struct i2c_client *client = tr8mcu->client;
	u32 readdata, mask;
	int error;

	dev_info(&client->dev, "%s %d: 0x%02X, len %d\n",
			 __func__, __LINE__, reg, len);

	if( len == 0 || len > 4 )
	{
		dev_err_ratelimited(&client->dev,
				"tr8mcu_write: reading %d bytes is not supported\n", len);
		return -EPERM;
	}

	error = tr8mcu_readwrite( client,
					sizeof(reg), &reg,
					len, (u8*)&readdata);
	if (error) {
		dev_err_ratelimited(&client->dev,
				"tr8mcu_read: Unable to fetch %d bytes, error: %d\n",
				len, error);
		return error;
	}
	dev_info(&client->dev, "%s %d: 0x%02X, len %d: 0x%02X\n",
			 __func__, __LINE__, reg, len, readdata);

	// Mask out unread bits
	mask = 0xFFFFFFFF >> ( (4 - len) * 8 );
	readdata &= mask;

	dev_info(&client->dev, "%s %d: 0x%02X, len %d: 0x%02X, 0x%08X\n",
			 __func__, __LINE__, reg, len, readdata, mask);
	*data = readdata;
	return 0;
}

int tr8mcu_write(struct tr8mcu * tr8mcu, u8 reg, int len, u32 data)
{
	struct i2c_client *client = tr8mcu->client;
	u8 buf[5];
	u32 mask;
	int i, error;

	dev_info(&client->dev, "%s %d: Reg: 0x%02X, len %d, data 0x%02x\n",
			 __func__, __LINE__, reg, len, data);
	if( len == 0 || len > 4 )
	{
		dev_err_ratelimited(&client->dev,
				"tr8mcu_write: writing %d bytes is not supported\n", len);
		return EPERM;
	}

	buf[0] = reg;

	// Mask out unread bits
	mask = 0xFFFFFFFF >> ( (4 - len) * 8 );
	data &= mask;

	for( i = 0; i < 4; i++)
		buf[1+i] = ( data >> ( 8 * i)) & 0xFF;

	error = tr8mcu_readwrite( client, len + 1, buf, 0, NULL);
	if (error) {
		dev_err_ratelimited(&client->dev,
				"tr8mcu_write: Unable to write %d bytes, error: %d\n",
				len + 1, error);
		return error;
	}
	return 0;
}

/*== Read Write =======================================================*/

static int tr8mcu_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct tr8mcu *tr8mcu;
	//struct regmap_irq_chip_data *irq_data;
	int ret = 0;
	u32 data;
	int child_count = 0;
	struct device_node *np, *child;

	if (!client->irq) {
		dev_err(&client->dev, "No IRQ configured\n");
		return -EINVAL;
	}

	tr8mcu = devm_kzalloc(&client->dev, sizeof(struct tr8mcu), GFP_KERNEL);
	if (!tr8mcu)
		return -ENOMEM;

	tr8mcu->client = client;
	dev_set_drvdata(&client->dev, tr8mcu);

	// Try to read out the ID to check if we find an MCU
	if(tr8mcu_read(tr8mcu, TR8MCU_REG_ID, 1, &data) || data != 0x61)
	{
		dev_err(&client->dev, "Trizeps8 MCU: probe No MCU found!\n");
		return -ENODEV;
	}

	np = client->dev.of_node;
	child_count = of_get_available_child_count(np);
	tr8mcu->mfd_cells = devm_kzalloc(&client->dev,
				child_count * sizeof(struct mfd_cell), GFP_KERNEL);
	dev_info(&client->dev, "%d child nodes found.\n", child_count);

	for_each_available_child_of_node(np, child)
	{
		struct property *prop;
		const char *cp = child->name, *compatible;
		dev_info(&client->dev, "%s %d: Found child: %s\n",
			 __func__, __LINE__, child->name);

		/* TODO use compatible here instead of the name, currently breaks
		 * everything */
		prop = of_find_property(child, "compatible", NULL);
		if(prop){
			compatible = of_prop_next_string(prop, NULL);
			dev_info(&client->dev, "%s %d: child: %s\n",
			 __func__, __LINE__, compatible);
			if(compatible) cp = compatible;
			tr8mcu->mfd_cells->of_compatible = devm_kstrdup(&client->dev,
						compatible, GFP_KERNEL);
		}

		tr8mcu->mfd_cells->name = devm_kstrdup(&client->dev, cp, GFP_KERNEL);
	}

	if(child_count)
	{
		ret = devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				   tr8mcu->mfd_cells, child_count, NULL, 0, NULL);
		if (ret)
			dev_err(&client->dev, "Failed to create subdevices\n");
	}


	return ret;
}

static const struct of_device_id tr8mcu_of_match[] = {
	{ .compatible = "seco,tr8mcu", },
	{ },
};
MODULE_DEVICE_TABLE(of, tr8mcu_of_match);

static struct i2c_driver tr8mcu_drv = {
	.driver = {
		.name = "seco-tr8mcu",
		.of_match_table = tr8mcu_of_match,
	},
	.probe = &tr8mcu_i2c_probe,
};

module_i2c_driver(tr8mcu_drv);

MODULE_AUTHOR("Jonas Höppner <jonas.hoeppner@seco.com>");
MODULE_DESCRIPTION("Trizeps8 MCU driver");
MODULE_LICENSE("GPL");
