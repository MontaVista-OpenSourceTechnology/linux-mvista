/*
 * Copyright (C) 2012 - 2014 Texas Instruments Incorporated
 * Authors: Sandeep Paulraj <s-paulraj@ti.com>
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

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>

#include "keystone_net.h"

#define SGMII_SRESET_RESET		0x1
#define SGMII_SRESET_RTRESET		0x2
#define SGMII_CTL_AUTONEG		0x01
#define SGMII_CTL_LOOPBACK		0x10
#define SGMII_CTL_MASTER		0x20
#define SGMII_REG_STATUS_LOCK		BIT(4)
#define	SGMII_REG_STATUS_LINK		BIT(0)
#define SGMII_REG_STATUS_AUTONEG	BIT(2)
#define SGMII_REG_CONTROL_AUTONEG	BIT(0)

#define SGMII23_OFFSET(x)	((x - 2) * SGMII_REGS_SIZE)
#define SGMII_OFFSET(x)		((x <= 1) ? (x * SGMII_REGS_SIZE) : \
					    (SGMII23_OFFSET(x)))
/*
 * SGMII registers
 */
#define SGMII_IDVER_REG(x)    (SGMII_OFFSET(x) + 0x000)
#define SGMII_SRESET_REG(x)   (SGMII_OFFSET(x) + 0x004)
#define SGMII_CTL_REG(x)      (SGMII_OFFSET(x) + 0x010)
#define SGMII_STATUS_REG(x)   (SGMII_OFFSET(x) + 0x014)
#define SGMII_MRADV_REG(x)    (SGMII_OFFSET(x) + 0x018)
#define SGMII_LPADV_REG(x)    (SGMII_OFFSET(x) + 0x020)
#define SGMII_TXCFG_REG(x)    (SGMII_OFFSET(x) + 0x030)
#define SGMII_RXCFG_REG(x)    (SGMII_OFFSET(x) + 0x034)
#define SGMII_AUXCFG_REG(x)   (SGMII_OFFSET(x) + 0x038)

static inline void sgmii_write_reg(void __iomem *base, int reg, u32 val)
{
	__iowmb();
	__raw_writel(val, base + reg);
}

static inline u32 sgmii_read_reg(void __iomem *base, int reg)
{
	u32 v;

	v = __raw_readl(base + reg);
	__iormb();
	return v;
}

/* port is 0 based */
int keystone_sgmii_reset(void __iomem *sgmii_ofs, int port)
{
	u32 reg;

	/* Initiate a soft reset */
	reg = sgmii_read_reg(sgmii_ofs, SGMII_SRESET_REG(port));
	reg |= SGMII_SRESET_RESET;
	sgmii_write_reg(sgmii_ofs, SGMII_SRESET_REG(port), reg);

	/* Wait for the bit to auto-clear */
	do
		reg = sgmii_read_reg(sgmii_ofs, SGMII_SRESET_REG(port));
	while ((reg & SGMII_SRESET_RESET) != 0x0);

	return 0;
}

/* port is 0 based */
bool keystone_sgmii_rtreset(void __iomem *sgmii_ofs, int port, bool set)
{
	u32 reg;
	bool oldval;

	/* Initiate a soft reset */
	reg = sgmii_read_reg(sgmii_ofs, SGMII_SRESET_REG(port));
	oldval = (reg & SGMII_SRESET_RTRESET) != 0x0;
	if (set)
		reg |= SGMII_SRESET_RTRESET;
	else
		reg &= ~SGMII_SRESET_RTRESET;
	sgmii_write_reg(sgmii_ofs, SGMII_SRESET_REG(port), reg);
	__iowmb();

	return oldval;
}

/* assumes ports <= 2 */
int keystone_sgmii_link_status(void __iomem *sgmii_ofs, int ports)
{
	u32 status = 0, link = 0;
	u32 i;

	for (i = 0; i < ports; i++) {
		status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(i));
		if ((status & SGMII_REG_STATUS_LINK) != 0)
			link |= BIT(i);
		else
			link &= ~BIT(i);
	}

	return link;
}

int keystone_sgmii_get_port_link(void __iomem *sgmii_ofs, int port)
{
	u32 status = 0, link = 0;

	status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(port));
	if ((status & SGMII_REG_STATUS_LINK) != 0)
		link |= BIT(port);
	else
		link &= ~BIT(port);

	return link;
}


int keystone_sgmii_config(void __iomem *sgmii_ofs,
			  int port, u32 interface)
{
	unsigned int i, status, mask;
	u32 mr_adv_ability;
	u32 control;

	switch (interface) {
	case SGMII_LINK_MAC_PHY_MASTER:
	case SGMII_LINK_MAC_PHY_MASTER_NO_MDIO:
	case SGMII_LINK_MAC_MAC_AUTONEG:
		mr_adv_ability	= 0x9801;
		control		= 0x21;
		break;

	case SGMII_LINK_MAC_PHY:
	case SGMII_LINK_MAC_PHY_NO_MDIO:
	case SGMII_LINK_MAC_MAC_AN_SLAVE:
		mr_adv_ability	= 1;
		control		= 1;
		break;

	case SGMII_LINK_MAC_MAC_FORCED:
		mr_adv_ability	= 0x9801;
		control		= 0x20;
		break;

	case SGMII_LINK_MAC_FIBER:
		mr_adv_ability	= 0x20;
		control		= 0x1;
		break;

	default:
		WARN_ONCE(1, "Invalid sgmii interface: %d\n", interface);
		return -EINVAL;
	}

	sgmii_write_reg(sgmii_ofs, SGMII_CTL_REG(port), 0);

	/*
	 * Wait for the SerDes pll to lock,
	 * but don't trap if lock is never read
	 */
	for (i = 0; i < 1000; i++)  {
		udelay(2000);
		status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(port));
		if ((status & SGMII_REG_STATUS_LOCK) != 0)
			break;
	}

	sgmii_write_reg(sgmii_ofs, SGMII_MRADV_REG(port), mr_adv_ability);
	sgmii_write_reg(sgmii_ofs, SGMII_CTL_REG(port), control);


	mask = SGMII_REG_STATUS_LINK;

	if (control & SGMII_REG_CONTROL_AUTONEG)
		mask |= SGMII_REG_STATUS_AUTONEG;

	for (i = 0; i < 1000; i++)  {
		status = sgmii_read_reg(sgmii_ofs, SGMII_STATUS_REG(port));
		if ((status & mask) == mask)
			break;
	}

	return 0;
}

