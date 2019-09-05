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

#define CMIC_COMMON_POOL_SCHAN_NUM				(4)

#define CMICX_TOP_SBUS_TIMEOUT(base)			(base + 0x00000)
#define CMICX_TOP_SBUS_RING_MAP_0_7(base)		(base + 0x0000c)
#define CMIC_TOP_SBUS_RING_MAP_24_31(base)		(base + 0x00018)
#define CMICX_TOP_SBUS_RING_MAP_32_39(base)		(base + 0x0001c)
#define CMICX_SCHAN_CH0_CTRL(base)				(base + 0x10000 + (CMIC_COMMON_POOL_SCHAN_NUM << 8))
#define CMICX_SCHAN_CH0_MESSAGE(base)			(base + 0x1000c + (CMIC_COMMON_POOL_SCHAN_NUM << 8))

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

#define CMICX_XLPORT_WC_UCMEM_DATA		0x0
#define REG32_DATA_LEN					4
#define REG64_DATA_LEN					8
#define UCMEM_DATA_LEN					16

#define CMICX_BLOCK_ID_TOP				7
#define CMICX_BLOCK_ID_XLPORT			((cmic->device == IPROC_DEVICE_HX5) ? 38 : 31)


#define CMICX_CMD(mode, blk, len)		((mode << SBUSV4_OPCODE_SHIFT) | \
										 (blk << SBUSV4_BLOCKID_SHIFT) | \
										 (len << SBUSV4_DLEN_SHIFT))

static int __cmicx_schan_write(struct iproc_cmic *cmic,
						u32 cmd, u32 addr, u32 *val, int len)
{
	u32 read = 0x0;
	void __iomem *msg_addr;
	int i;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	msg_addr = CMICX_SCHAN_CH0_MESSAGE(cmic->base);
	writel(cmd, msg_addr);

	msg_addr += 4;
	writel(addr, msg_addr);

	for (i = 0; i < len; i++) {
		msg_addr += 4;
		writel(val[i], msg_addr);
	}

    writel(0x1, CMICX_SCHAN_CH0_CTRL(cmic->base));

	// FIXME, should set timeout
    while (read != 0x2) {
        read = readl(CMICX_SCHAN_CH0_CTRL(cmic->base));
    }
    return read;
}

static int __cmicx_schan_read(struct iproc_cmic *cmic,
						u32 cmd, u32 addr, u32 *val, int len)
{
	u32 read = 0;
	void __iomem *msg_addr;
	int i;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	msg_addr = CMICX_SCHAN_CH0_MESSAGE(cmic->base);
	writel(cmd, msg_addr);

	msg_addr += 4;
	writel(addr, msg_addr);

	writel(0x1, CMICX_SCHAN_CH0_CTRL(cmic->base));

	// FIXME, should set timeout
	while (read != 0x2) {
		read = readl(CMICX_SCHAN_CH0_CTRL(cmic->base));
	}

	msg_addr = CMICX_SCHAN_CH0_MESSAGE(cmic->base);
    for (i = 0; i < len; i++) {
		msg_addr += 4;
        val[i] = readl(msg_addr);
    }

	if (len == 1)
		return val[0];
	else
		return val[1];
}

static int iproc_cmicx_schan_reg32_write(struct iproc_cmic *cmic,
						u32 blk_type, u32 addr, u32 val)
{
	u32 cmd, block;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	if (blk_type == CMIC_BLOCK_TYPE_TOP) {
		block = CMICX_BLOCK_ID_TOP;
	} else if (blk_type == CMIC_BLOCK_TYPE_APM) {
		block = CMICX_BLOCK_ID_XLPORT;
	} else {
		return -EINVAL;
	}

	cmd = CMICX_CMD(WRITE_REGISTER_CMD_MSG, block, REG32_DATA_LEN);
	return __cmicx_schan_write(cmic, cmd, addr, &val, 1);
}

static u32 iproc_cmicx_schan_reg32_read(struct iproc_cmic *cmic,
						u32 blk_type, u32 addr)
{
	u32 cmd, block, val;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	if (blk_type == CMIC_BLOCK_TYPE_TOP) {
		block = CMICX_BLOCK_ID_TOP;
	} else if (blk_type == CMIC_BLOCK_TYPE_APM) {
		block = CMICX_BLOCK_ID_XLPORT;
	} else {
		return -EINVAL;
	}

	cmd = CMICX_CMD(READ_REGISTER_CMD_MSG, block, REG32_DATA_LEN);
	return __cmicx_schan_read(cmic, cmd, addr, &val, 1);
}

static int iproc_cmicx_schan_reg64_write(struct iproc_cmic *cmic,
						u32 blk_type, u32 addr, u64 val)
{
	u32 cmd, block, val32[2];

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	if (blk_type == CMIC_BLOCK_TYPE_TOP) {
		block = CMICX_BLOCK_ID_TOP;
	} else if (blk_type == CMIC_BLOCK_TYPE_APM) {
		block = CMICX_BLOCK_ID_XLPORT;
	} else {
		return -EINVAL;
	}

	cmd = CMICX_CMD(WRITE_REGISTER_CMD_MSG, block, REG64_DATA_LEN);
	val32[0] = (u32)val;
	val32[1] = (u32)(val >> 32);
	return __cmicx_schan_write(cmic, cmd, addr, val32, 2);
}

static u64 iproc_cmicx_schan_reg64_read(struct iproc_cmic *cmic,
						u32 blk_type, u32 addr)
{
	u32 cmd, block, val32[2];
	u64 val;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	if (blk_type == CMIC_BLOCK_TYPE_TOP) {
		block = CMICX_BLOCK_ID_TOP;
	} else if (blk_type == CMIC_BLOCK_TYPE_APM) {
		block = CMICX_BLOCK_ID_XLPORT;
	} else {
		return -EINVAL;
	}

	cmd = CMICX_CMD(READ_REGISTER_CMD_MSG, block, REG64_DATA_LEN);
	__cmicx_schan_read(cmic, cmd, addr, val32, 2);
	val = (((u64)val32[1] << 32) | val32[0]);
	return val;
}

static int iproc_cmicx_schan_ucmem_write(struct iproc_cmic *cmic,
						u32 blk_type, u32 *mem)
{
	u32 cmd, block;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	if (blk_type == CMIC_BLOCK_TYPE_TOP) {
		block = CMICX_BLOCK_ID_TOP;
	} else if (blk_type == CMIC_BLOCK_TYPE_APM) {
		block = CMICX_BLOCK_ID_XLPORT;
	} else {
		return -EINVAL;
	}

	cmd = CMICX_CMD(WRITE_MEMORY_CMD_MSG, block, UCMEM_DATA_LEN);
	return __cmicx_schan_write(cmic, cmd, CMICX_XLPORT_WC_UCMEM_DATA, mem, 4);
}

static int iproc_cmicx_schan_ucmem_read(struct iproc_cmic *cmic,
						u32 blk_type, u32 *mem)
{
	u32 cmd, block;

	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	if (blk_type == CMIC_BLOCK_TYPE_TOP) {
		block = CMICX_BLOCK_ID_TOP;
	} else if (blk_type == CMIC_BLOCK_TYPE_APM) {
		block = CMICX_BLOCK_ID_XLPORT;
	} else {
		return -EINVAL;
	}

	cmd = CMICX_CMD(READ_MEMORY_CMD_MSG, block, UCMEM_DATA_LEN);
	return __cmicx_schan_read(cmic, cmd, CMICX_XLPORT_WC_UCMEM_DATA, mem, 2);
}

static int iproc_cmicx_init(struct iproc_cmic *cmic)
{
	if (!cmic || !cmic->base) {
		return -EINVAL;
	}

	if (cmic->device == IPROC_DEVICE_HX5) {
		/*
		 * SBUS ring and block number:
		 * ring 5: TOP(7)
		 * ring 7: XLPORT7(38)
		 */
		writel(0x52222100, CMICX_TOP_SBUS_RING_MAP_0_7(cmic->base));
		writel(0x07500066, CMICX_TOP_SBUS_RING_MAP_32_39(cmic->base));
		writel(0x5000, CMICX_TOP_SBUS_TIMEOUT(cmic->base));
	} else if (cmic->device == IPROC_DEVICE_HR4) {
		/*
		 * SBUS ring and block number:
		 * ring 5: TOP(7)
		 * ring 7: XLPORT3(31)
		 */
		writel(0x52222100, CMICX_TOP_SBUS_RING_MAP_0_7(cmic->base));
		writel(0x70003666, CMIC_TOP_SBUS_RING_MAP_24_31(cmic->base));
		writel(0x5000, CMICX_TOP_SBUS_TIMEOUT(cmic->base));
	}

	return 0;
}

const struct sbus_ops cmicx_sbus_ops = {
	.init			= iproc_cmicx_init,
	.reg32_write	= iproc_cmicx_schan_reg32_write,
	.reg32_read		= iproc_cmicx_schan_reg32_read,
	.reg64_write	= iproc_cmicx_schan_reg64_write,
	.reg64_read		= iproc_cmicx_schan_reg64_read,
	.ucmem_write	= iproc_cmicx_schan_ucmem_write,
	.ucmem_read		= iproc_cmicx_schan_ucmem_read,
};
