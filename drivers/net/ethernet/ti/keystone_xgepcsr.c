/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Author: WingMan Kwok <w-kwok2@ti.com>
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
#include <linux/io.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/random.h>

#include "keystone_net.h"

/*
 * Keystone2 XGE SERDES SS
 */
#define XGE_SERDES_BASE		0x0231e000
#define XGE_SERDES_SIZE		0x2000

/* SERDES registers */
/* 0x1fc0 - 0x1fff */
#define K2SERDES_SS_OFFSET	0x1fc0
/* 0x1fe0, 0x1fe4 */
#define LANE_CTRL_STS_REG(x)	(K2SERDES_SS_OFFSET + 0x20 + (x * 0x04))

/*
 * XGE PCS-R registers
 */
#define PCSR_OFFSET(x)		(0x600 + (x * 0x80))
#define PCSR_RX_STATUS(x)	(PCSR_OFFSET(x) + 0x0C)

static inline u32 k2serdes_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

/* Check xge link status
 * returns TRUE for link up,
 * FALSE otherwise
 * */
int k2serdes_check_xge_link_status(void __iomem *serdes,
			      void __iomem *sw_regs,
			      u32 lane, int print)
{
	u32 pcsr_rx_stat, blk_lock, blk_errs, link10g;
	int loss, status = 1;

	/* Rx Signal Loss bit in serdes lane control and status reg*/
	loss = (k2serdes_readl(serdes, LANE_CTRL_STS_REG(lane))) & 0x01;

	/* 10G linked up in serdes lane control and status register */
	link10g = (k2serdes_readl(serdes, LANE_CTRL_STS_REG(lane)) >> 20) & 0x1;

	/* Block Errors and Block Lock bits in PCSR rx status reg */
	pcsr_rx_stat = k2serdes_readl(sw_regs, PCSR_RX_STATUS(lane));
	blk_lock = (pcsr_rx_stat >> 30) & 0x1;
	blk_errs = (pcsr_rx_stat >> 16) & 0x0ff;

	if (print) {
		pr_info("%s: loss: %d, link10g %u, pc_rxstat 0x%x\n", __func__, loss, link10g, pcsr_rx_stat);
	}

	if (loss) {
		return 0;
	} else if (!link10g) {
		return 0;
	} else if (blk_errs) {
		return 0;
	} else if (!blk_lock) {
		return 0;
	}

	return status;
}

int keystone_pcsr_config(void __iomem *pcsr_ofs, int port, u32 interface)
{
	return 0;
}
