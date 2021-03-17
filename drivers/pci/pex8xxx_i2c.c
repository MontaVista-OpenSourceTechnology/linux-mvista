/*
 * Driver for the PEX8xxx I2C slave interface
 *
 * Rajat Jain <rajatjain@juniper.net>
 * Copyright 2014 Juniper Networks
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License version 2 as published
 * by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/i2c/pex8xxx_i2c.h>

#define PCI_DEVICE_ID_PLX_8614	    0x8614
#define PCI_DEVICE_ID_PLX_8618	    0x8618
#define PCI_DEVICE_ID_PLX_8713	    0x8713

#define PEX8XXX_PORT_REG_SPACE	    4096

/* Supported devices */
enum chips { pex8614, pex8618, pex8713 };

#define MAXSTN			    2
#define MAXMODE			    4

/* Common Register defines */
#define PEX8XXX_CMD(val)	    (((val) & 7) << 24)
#define PEX8XXX_CMD_WR		    0x03
#define PEX8XXX_CMD_RD		    0x04

#define PEX8XXX_BYTE_ENA(val)	    (((val) & 0xF) << 10)
#define PEX8XXX_REG(val)	    (((val) >> 2) & 0x3FF)

/* PEX8614/8618 Device specific register defines */
#define PEX861X_PORT(val)	    (((val) & 0x1F) << 15)

#define PEX861X_I2C_CMD(cmd, port, mode, stn, reg, byte_mask)	\
	(PEX8XXX_CMD(cmd) |					\
	 PEX861X_PORT(port) |					\
	 PEX8XXX_BYTE_ENA(byte_mask) |				\
	 PEX8XXX_REG(reg))

/* PEX8713 Device specific register defines */
#define PEX8713_MODE(val)	    (((val) & 3) << 20)
#define PEX8713_STN(val)	    (((val) & 3) << 18)
#define PEX8713_PORT(val)	    (((val) & 7) << 15)

#define PEX8713_I2C_CMD(cmd, port, mode, stn, reg, byte_mask)	\
	(PEX8XXX_CMD(cmd) |					\
	 PEX8713_MODE(mode) |					\
	 PEX8713_STN(stn) |					\
	 PEX8713_PORT(port) |					\
	 PEX8XXX_BYTE_ENA(byte_mask) |				\
	 PEX8XXX_REG(reg))

struct pex8xxx_dev {
	enum chips	        devtype;
	u32			reg_addr;
	u8                      port_num;
	u8                      port_mode;      /* PEX8713 only */
	u8                      port_stn;       /* PEX8713 only */
	struct attribute_group  attr_group;
	bool (*port_is_valid)(u8 port);
};

static inline struct pex8xxx_dev *pex8xxx_get_drvdata(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	return i2c_get_clientdata(client);
}

/**
 * pex8xxx_read() - Read a (32 bit) register from the PEX8xxx device.
 * @client:	struct i2c_client*, representing the pex8xxx device.
 * @stn:	Station number (Used on some PLX switches such as PEX8713 that
 *		support multi stations. Ignored on switches that don't support
 *		it)
 * @mode:	Port mode (Transparent / Non-transparent etc)
 * @byte_mask:	Byte enable mask.
 * @port:	Port number
 * @reg:	Register offset to read.
 * @val:	Pointer where the result is to be written.
 *
 * Return: 0 on Success, Error value otherwise.
 */
int pex8xxx_read(struct i2c_client *client, u8 stn, u8 mode, u8 byte_mask,
		 u8 port, u32 reg, u32 *val)
{
	struct pex8xxx_dev *pex8xxx = i2c_get_clientdata(client);
	__be32 cmd, data;
	int ret;

	struct i2c_msg msgs[2] = {
		{
			.addr = client->addr,
			.len = 4,
			.flags = 0,
			.buf = (u8 *) &cmd,
		},
		{
			.addr = client->addr,
			.len = 4,
			.flags = I2C_M_RD,
			.buf = (u8 *) &data,
		},
	};

	switch (pex8xxx->devtype) {
	case pex8614:
	case pex8618:
		cmd = cpu_to_be32(PEX861X_I2C_CMD(PEX8XXX_CMD_RD, port, mode,
						  stn, reg, byte_mask));
		break;
	case pex8713:
		cmd = cpu_to_be32(PEX8713_I2C_CMD(PEX8XXX_CMD_RD, port, mode,
						  stn, reg, byte_mask));
		break;
	default: /* Unknown device */
		return -ENODEV;
	}

	ret = i2c_transfer(client->adapter, msgs, 2);
	*val = be32_to_cpu(data);

	if (ret < 0)
		return ret;
	else if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(pex8xxx_read);

/**
 * pex8xxx_write() - Write a (32 bit) register to the PEX8xxx device.
 * @client:	struct i2c_client*, representing the pex8xxx device.
 * @stn:	Station number (Used on some PLX switches such as PEX8713 that
 *		support multi stations. Ignored on switches that don't support
 *		it)
 * @mode:	Port mode (Transparent / Non-transparent etc)
 * @byte_mask:	Byte enable mask.
 * @port:	Port number
 * @reg:	Register offset to write.
 * @val:	Value to be written.
 *
 * Return: 0 on Success, Error value otherwise.
 */
int pex8xxx_write(struct i2c_client *client, u8 stn, u8 mode, u8 byte_mask,
		  u8 port, u32 reg, u32 val)
{
	struct pex8xxx_dev *pex8xxx = i2c_get_clientdata(client);
	__be32 msgbuf[2];
	int ret;

	struct i2c_msg msg = {
		.addr = client->addr,
		.len = 8,
		.flags = 0,
		.buf = (u8 *) msgbuf,
	};

	switch (pex8xxx->devtype) {
	case pex8614:
	case pex8618:
		msgbuf[0] = cpu_to_be32(PEX861X_I2C_CMD(PEX8XXX_CMD_WR, port,
							mode, stn, reg,
							byte_mask));
		break;
	case pex8713:
		msgbuf[0] = cpu_to_be32(PEX8713_I2C_CMD(PEX8XXX_CMD_WR, port,
							mode, stn, reg,
							byte_mask));
		break;
	default: /* Unknown device */
		return -ENODEV;
	}
	msgbuf[1] = cpu_to_be32(val);

	ret = i2c_transfer(client->adapter, &msg, 1);

	if (ret < 0)
		return ret;
	else if (ret != 1)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL(pex8xxx_write);

/*
 * Different PCIe switch can have different port validators.
 * Also, some switches have discontinuous port number configurations.
 */
static bool pex8618_port_is_valid(u8 port)
{
	return port <= 15;
}

static bool pex8713_port_is_valid(u8 port)
{
	return port <= 5 || (port >= 8 && port <= 13);
}

static bool pex8614_port_is_valid(u8 port)
{
	return port <= 2 ||
	    (port >= 4 && port <= 10) ||
	    port == 12 ||
	    port == 14;
}

static ssize_t port_num_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct pex8xxx_dev *pex8xxx = pex8xxx_get_drvdata(dev);

	return sprintf(buf, "%d\n", pex8xxx->port_num);
}
static ssize_t port_num_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct pex8xxx_dev *pex8xxx = pex8xxx_get_drvdata(dev);
	u8 port_num;

	if (kstrtou8(buf, 0, &port_num))
		return -EINVAL;

	if (!pex8xxx->port_is_valid(port_num))
		return -EINVAL;

	pex8xxx->port_num = port_num;
	return count;
}
static DEVICE_ATTR_RW(port_num);

static ssize_t port_mode_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct pex8xxx_dev *pex8xxx = pex8xxx_get_drvdata(dev);
	char *str;

	switch (pex8xxx->port_mode) {
	case MODE_TRANSPARENT:
		str = "transparent";
		break;
	case MODE_NT_LINK:
		str = "nt-link";
		break;
	case MODE_NT_VIRT:
		str = "nt-virtual";
		break;
	case MODE_DMA:
		str = "dma";
		break;
	default:
		str = "unknown";
		break;
	}
	return sprintf(buf, "%s\n", str);
}
static ssize_t port_mode_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct pex8xxx_dev *pex8xxx = pex8xxx_get_drvdata(dev);

	if (!strcmp(buf, "transparent\n"))
		pex8xxx->port_mode = MODE_TRANSPARENT;
	else if (!strcmp(buf, "nt-link\n"))
		pex8xxx->port_mode = MODE_NT_LINK;
	else if (!strcmp(buf, "nt-virtual\n"))
		pex8xxx->port_mode = MODE_NT_VIRT;
	else if (!strcmp(buf, "dma\n"))
		pex8xxx->port_mode = MODE_DMA;
	else
		return -EINVAL;

	return count;
}
static DEVICE_ATTR_RW(port_mode);

static ssize_t port_stn_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct pex8xxx_dev *pex8xxx = pex8xxx_get_drvdata(dev);

	return sprintf(buf, "%d\n", pex8xxx->port_stn);
}
static ssize_t port_stn_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct pex8xxx_dev *pex8xxx = pex8xxx_get_drvdata(dev);
	u8 stn;

	if (kstrtou8(buf, 0, &stn) || (stn >= MAXSTN))
		return -EINVAL;

	pex8xxx->port_stn = stn;

	return count;
}
static DEVICE_ATTR_RW(port_stn);

static ssize_t reg_addr_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct pex8xxx_dev *pex8xxx = pex8xxx_get_drvdata(dev);

	return sprintf(buf, "0x%X\n", pex8xxx->reg_addr);
}
static ssize_t reg_addr_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct pex8xxx_dev *pex8xxx = pex8xxx_get_drvdata(dev);
	unsigned long reg_addr;

	/* PEX8xxx devices support 4K memory per port */
	if (kstrtoul(buf, 0, &reg_addr) ||
	    reg_addr >= PEX8XXX_PORT_REG_SPACE ||
	    reg_addr % 4)
		return -EINVAL;

	pex8xxx->reg_addr = reg_addr;

	return count;
}
static DEVICE_ATTR_RW(reg_addr);

static ssize_t reg_value_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct pex8xxx_dev *pex8xxx;
	struct i2c_client *client;
	u32 regval = 0;
	int ret;

	client = to_i2c_client(dev);
	pex8xxx = i2c_get_clientdata(client);

	ret = pex8xxx_read(client, pex8xxx->port_stn, pex8xxx->port_mode,
			   MASK_BYTE_ALL, pex8xxx->port_num, pex8xxx->reg_addr,
			   &regval);
	if (ret)
		return ret;

	return sprintf(buf, "0x%08X\n", regval);
}

static ssize_t reg_value_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct pex8xxx_dev *pex8xxx;
	struct i2c_client *client;
	unsigned long reg_val;
	int retval;

	client = to_i2c_client(dev);
	pex8xxx = i2c_get_clientdata(client);

	if (kstrtoul(buf, 0, &reg_val))
		return -EINVAL;

	retval = pex8xxx_write(client, pex8xxx->port_stn, pex8xxx->port_mode,
			       MASK_BYTE_ALL, pex8xxx->port_num,
			       pex8xxx->reg_addr, reg_val);
	if (retval)
		return retval;

	return count;
}
static DEVICE_ATTR_RW(reg_value);

/*
 * Dump the 4096 byte binary configuration space
 */
static ssize_t
pex8xxx_read_full_config(struct file *filp, struct kobject *kobj,
			 struct bin_attribute *bin_attr,
			 char *buf, loff_t off, size_t count)
{
	unsigned int size = PEX8XXX_PORT_REG_SPACE;
	struct pex8xxx_dev *pex8xxx;
	struct i2c_client *client;
	struct device *dev;
	loff_t init_off = off;
	u32 *buf32 = (u32 *)buf;
	u32 regval;
	int ret;

	dev = container_of(kobj, struct device, kobj);
	client = to_i2c_client(dev);
	pex8xxx = i2c_get_clientdata(client);

	if (off > size || off & 3)
		return 0;
	if (off + count > size) {
		size -= off;
		count = size;
	} else {
		size = count;
	}

	while (size) {
		ret = pex8xxx_read(client, pex8xxx->port_stn,
				   pex8xxx->port_mode, MASK_BYTE_ALL,
				   pex8xxx->port_num, off, &regval);
		if (ret)
			regval = 0xDEADBEEF;

		buf32[(off - init_off)/4] = regval;
		off += 4;
		size -= 4;
	}

	return count;
}
static BIN_ATTR(port_config_regs, S_IRUGO, pex8xxx_read_full_config,
		NULL, PEX8XXX_PORT_REG_SPACE);

static struct attribute *pex861x_attrs[] = {
	&dev_attr_port_num.attr,
	&dev_attr_reg_addr.attr,
	&dev_attr_reg_value.attr,
	NULL,
};
static struct attribute *pex8713_attrs[] = {
	&dev_attr_port_num.attr,
	&dev_attr_port_mode.attr,
	&dev_attr_port_stn.attr,
	&dev_attr_reg_addr.attr,
	&dev_attr_reg_value.attr,
	NULL,
};

static struct bin_attribute *pex8xxx_bin_attrs[] = {
	&bin_attr_port_config_regs,
	NULL,
};

static int pex8xxx_verify_device(struct pex8xxx_dev *pex8xxx,
				 struct i2c_client *client)
{
	u8 stn, mode;
	bool found = false;
	u32 data = 0;

	for (stn = 0; stn < MAXSTN; stn++) {
		for (mode = 0; mode < MAXMODE; mode++) {
			if (!pex8xxx_read(client, stn, mode, MASK_BYTE_ALL, 0,
					  PCI_VENDOR_ID, &data)) {
				found = true;
				break;
			}
		}
	}

	if (!found || (data & 0xFFFF) != PCI_VENDOR_ID_PLX)
		return -ENODEV;

	switch (data >> 16) {
	case PCI_DEVICE_ID_PLX_8614:
		pex8xxx->devtype = pex8614;
		pex8xxx->port_is_valid = pex8614_port_is_valid;
		pex8xxx->attr_group.attrs = pex861x_attrs;
		break;
	case PCI_DEVICE_ID_PLX_8618:
		pex8xxx->devtype = pex8618;
		pex8xxx->port_is_valid = pex8618_port_is_valid;
		pex8xxx->attr_group.attrs = pex861x_attrs;
		break;
	case PCI_DEVICE_ID_PLX_8713:
		pex8xxx->devtype = pex8713;
		pex8xxx->port_is_valid = pex8713_port_is_valid;
		pex8xxx->attr_group.attrs = pex8713_attrs;
		break;
	default:    /* Unsupported PLX device */
		return -ENODEV;
	}

	return 0;
}

static int pex8xxx_probe(struct i2c_client *client,
			 const struct i2c_device_id *dev_id)
{
	struct pex8xxx_dev *pex8xxx;
	int retval;

	pex8xxx = devm_kzalloc(&client->dev, sizeof(*pex8xxx), GFP_KERNEL);
	if (!pex8xxx)
		return -ENOMEM;

	i2c_set_clientdata(client, pex8xxx);

	if (pex8xxx_verify_device(pex8xxx, client))
		return -ENODEV;

	pex8xxx->attr_group.bin_attrs = pex8xxx_bin_attrs;

	retval =  sysfs_create_group(&client->dev.kobj, &pex8xxx->attr_group);

	return retval;
}

static int pex8xxx_remove(struct i2c_client *client)
{
	struct pex8xxx_dev *pex8xxx = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &pex8xxx->attr_group);
	return 0;
}

static const struct i2c_device_id pex8xxx_id[] = {
	{ "pex8614", pex8614 },
	{ "pex8618", pex8618 },
	{ "pex8713", pex8713 },
	{}
};
MODULE_DEVICE_TABLE(i2c, pex8xxx_id);

static struct i2c_driver pex8xxx_driver = {
	.driver = {
		.name	= "pex8xxx",
	},
	.probe	= pex8xxx_probe,
	.remove	= pex8xxx_remove,
	.id_table = pex8xxx_id,
};

module_i2c_driver(pex8xxx_driver);

MODULE_DESCRIPTION("PLX PEX8xxx switch I2C interface driver");
MODULE_AUTHOR("Rajat Jain <rajatjain@juniper.net>");
MODULE_LICENSE("GPL v2");
