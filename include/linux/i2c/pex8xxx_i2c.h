/*
 * Interface for the PEX8xxx I2C slave interface
 *
 * Rajat Jain <rajatjain@juniper.net>
 * Copyright 2014 Juniper Networks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 */

#ifndef __PEX8XXX_I2C_H__
#define __PEX8XXX_I2C_H__

#include <linux/i2c.h>

/* Values for "mode" argument */
#define MODE_TRANSPARENT	0x00
#define MODE_NT_LINK		0x01
#define MODE_NT_VIRT		0x02
#define MODE_DMA		0x03

/* Values for "byte_mask" argument */
#define MASK_BYTE0		0x01
#define MASK_BYTE1		0x02
#define MASK_BYTE2		0x04
#define MASK_BYTE3		0x08
#define MASK_BYTE_ALL		(MASK_BYTE0 | MASK_BYTE1 |\
				 MASK_BYTE2 | MASK_BYTE3)

int pex8xxx_read(struct i2c_client *client, u8 stn, u8 mode, u8 byte_mask,
		 u8 port, u32 reg, u32 *val);
int pex8xxx_write(struct i2c_client *client, u8 stn, u8 mode, u8 byte_mask,
		  u8 port, u32 reg, u32 val);

#endif /* __PEX8XXX_I2C_H__ */
