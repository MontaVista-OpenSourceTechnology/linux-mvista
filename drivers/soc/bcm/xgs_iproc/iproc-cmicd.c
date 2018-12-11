/*
 * Copyright (C) 2016 Broadcom
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

#include <linux/module.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/soc/bcm/iproc-cmic.h>

#define CMICD_SBUS_RING_MAP_0_7(base)			(base + 0x10098)
#define CMICD_SBUS_RING_MAP_8_15(base)			(base + 0x1009C)
#define CMICD_SBUS_RING_MAP_16_23(base)			(base + 0x100A0)
#define CMICD_SBUS_RING_MAP_24_31(base)			(base + 0x100A4)
#define CMICD_SCHAN_CH0_CTRL(base)				(base + 0x10000)
#define CMICD_SCHAN_CH0_MESSAGE(base)			(base + 0x1000c)

#define READ_MEMORY_CMD_MSG				0x07
#define READ_MEMORY_ACK_MSG				0x08
#define WRITE_MEMORY_CMD_MSG			0x09
#define WRITE_MEMORY_ACK_MSG			0x0a
#define READ_REGISTER_CMD_MSG			0x0b
#define READ_REGISTER_ACK_MSG			0x0c
#define WRITE_REGISTER_CMD_MSG			0x0d
#define WRITE_REGISTER_ACK_MSG			0x0e
#define SBUSV4_REGTYPE_SHIFT			25
#define SBUSV4_REGADDR_SHIFT			8
#define SBUSV4_OPCODE_SHIFT				26
#define SBUSV4_BLOCKID_SHIFT			19
#define SBUSV4_DLEN_SHIFT				7

#define CMICD_XLPORT_WC_UCMEM_DATA		0x0
#define REG32_DATA_LEN					4
#define REG64_DATA_LEN					8
#define UCMEM_DATA_LEN					16

#define CMICD_BLOCK_ID_TOP				16


#define CMICD_CMD(mode, blk, len)		((mode << SBUSV4_OPCODE_SHIFT) | \
										 (blk << SBUSV4_BLOCKID_SHIFT) | \
										 (len << SBUSV4_DLEN_SHIFT))

static int __cmicd_schan_write(struct iproc_cmic *cmic,
						u32 cmd, u32 addr, u32 *val, int len)
{
	u32 read = 0x0;
	void __iomem *msg_addr;
	int i;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	msg_addr = CMICD_SCHAN_CH0_MESSAGE(cmic->base);
	writel(cmd, msg_addr);

	msg_addr += 4;
	writel(addr, msg_addr);

	for (i = 0; i < len; i++) {
		msg_addr += 4;
		writel(val[i], msg_addr);
	}

    writel(0x1, CMICD_SCHAN_CH0_CTRL(cmic->base));

	// FIXME, should set timeout
    while (read != 0x2) {
        read = readl(CMICD_SCHAN_CH0_CTRL(cmic->base));
    }
    return read;
}

static int __cmicd_schan_read(struct iproc_cmic *cmic,
						u32 cmd, u32 addr, u32 *val, int len)
{
	u32 read = 0x0;
	void __iomem *msg_addr;
	int i;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	read = __cmicd_schan_write(cmic, cmd, addr, NULL, 0);
	if (read != 0x02) {
		return read;
	}

	msg_addr = CMICD_SCHAN_CH0_MESSAGE(cmic->base);
    for (i = 0; i < len; i++) {
		msg_addr += 4;
        val[i] = readl(msg_addr);
    }
    return val[0];
}

static int iproc_cmicd_schan_reg32_write(struct iproc_cmic *cmic,
						u32 blk_type, u32 addr, u32 val)
{
	u32 cmd, block;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	if (blk_type == CMIC_BLOCK_TYPE_TOP) {
		block = CMICD_BLOCK_ID_TOP;
	} else {
		return -EINVAL;
	}

	cmd = CMICD_CMD(WRITE_REGISTER_CMD_MSG, block, REG32_DATA_LEN);
	return __cmicd_schan_write(cmic, cmd, addr, &val, 1);
}

static u32 iproc_cmicd_schan_reg32_read(struct iproc_cmic *cmic,
						u32 blk_type, u32 addr)
{
	u32 cmd, block, val;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	if (blk_type == CMIC_BLOCK_TYPE_TOP) {
		block = CMICD_BLOCK_ID_TOP;
	} else {
		return -EINVAL;
	}

	cmd = CMICD_CMD(READ_REGISTER_CMD_MSG, block, REG32_DATA_LEN);
	return __cmicd_schan_read(cmic, cmd, addr, &val, 1);
}

static int iproc_cmicd_init(struct iproc_cmic *cmic)
{
	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	/* Configure SBUS Ring Map for TOP, block id = 16, ring number = 4 */
	writel(0x11112200, CMICD_SBUS_RING_MAP_0_7(cmic->base));
	writel(0x00430001, CMICD_SBUS_RING_MAP_8_15(cmic->base));
	writel(0x00005064, CMICD_SBUS_RING_MAP_16_23(cmic->base));
	writel(0x00000000, CMICD_SBUS_RING_MAP_24_31(cmic->base));

	return 0;
}

const struct sbus_ops cmicd_sbus_ops = {
	.init			= iproc_cmicd_init,
	.reg32_write	= iproc_cmicd_schan_reg32_write,
	.reg32_read		= iproc_cmicd_schan_reg32_read,
};
