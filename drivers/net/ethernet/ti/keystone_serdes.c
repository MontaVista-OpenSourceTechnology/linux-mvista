/*
 * Texas Instruments Keystone SerDes driver
 * Authors: Hao Zhang <hzhang@ti.com>
 *
 * Copyright (C) 2013 Texas Instruments
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

#include "keystone_serdes.h"

#define SERDES_CMU_REGS(x)		(0x0000 + (0x0c00 * x))
#define SERDES_LANE_REGS(x)		(0x0200 + (0x200 * x))
#define SERDES_COMLANE_REGS		0x0a00
#define SERDES_WIZ_REGS			0x1fc0

#define SERDES_CMU_REG_000(x)		(SERDES_CMU_REGS(x) + 0x000)
#define SERDES_CMU_REG_010(x)		(SERDES_CMU_REGS(x) + 0x010)
#define SERDES_COMLANE_REG_000		(SERDES_COMLANE_REGS + 0x000)
#define SERDES_LANE_REG_000(x)		(SERDES_LANE_REGS(x) + 0x000)
#define SERDES_LANE_REG_028(x)		(SERDES_LANE_REGS(x) + 0x028)
#define SERDES_LANE_CTL_STATUS_REG(x)	(SERDES_WIZ_REGS + 0x0020 + (0x4 * x))
#define SERDES_PLL_CTL_REG		(SERDES_WIZ_REGS + 0x0034)


#define SERDES_RESET			BIT(28)
#define SERDES_LANE_RESET		BIT(29)
#define SERDES_LANE_LOOPBACK		BIT(30)

#define SERDES_LANE_DISABLE_MASK	0x60006000

#define SERDES_LANE_EN_VAL(x, y, z)	(x[y] | (z << 26) | (z << 10))

#define reg_rmw(addr, value, mask) \
	__raw_writel(((__raw_readl(addr) & (~(mask))) | \
			(value & (mask))), (addr))

struct serdes_cfg {
	u32 ofs;
	u32 val;
	u32 mask;
};

/* SERDES PHY lane enable configuration value, indexed by PHY interface */
static u32 serdes_cfg_lane_enable[] = {
	0xf000f0c0,	/* SGMII */
	0xf0e9f038,	/* PCSR */
};

/* SERDES PHY PLL enable configuration value, indexed by PHY interface  */
static u32 serdes_cfg_pll_enable[] = {
	0xe0000000,	/* SGMII */
	0xee000000,	/* PCSR */
};

static struct serdes_cfg cfg_cmu_156p25m_5g[] = {
	{0x0000, 0x00800000, 0xffff0000},
	{0x0014, 0x00008282, 0x0000ffff},
	{0x0060, 0x00142438, 0x00ffffff},
	{0x0064, 0x00c3c700, 0x00ffff00},
	{0x0078, 0x0000c000, 0x0000ff00}
};

static struct serdes_cfg cfg_comlane_156p25m_5g[] = {
	{0x0a00, 0x00000800, 0x0000ff00},
	{0x0a08, 0x38a20000, 0xffff0000},
	{0x0a30, 0x008a8a00, 0x00ffff00},
	{0x0a84, 0x00000600, 0x0000ff00},
	{0x0a94, 0x10000000, 0xff000000},
	{0x0aa0, 0x81000000, 0xff000000},
	{0x0abc, 0xff000000, 0xff000000},
	{0x0ac0, 0x0000008b, 0x000000ff},
	{0x0b08, 0x583f0000, 0xffff0000},
	{0x0b0c, 0x0000004e, 0x000000ff}
};

static struct serdes_cfg cfg_lane_156p25mhz_5g[] = {
	{0x0004, 0x38000080, 0xff0000ff},
	{0x0008, 0x00000000, 0x000000ff},
	{0x000c, 0x02000000, 0xff000000},
	{0x0010, 0x1b000000, 0xff000000},
	{0x0014, 0x00006fb8, 0x0000ffff},
	{0x0018, 0x758000e4, 0xffff00ff},
	{0x00ac, 0x00004400, 0x0000ff00},
	{0x002c, 0x00100800, 0x00ffff00},
	{0x0080, 0x00820082, 0x00ff00ff},
	{0x0084, 0x1d0f0385, 0xffffffff}

};

static void serdes_cfg_setup(void __iomem *serdes_regs,
			     struct serdes_cfg *cfg,
			     u32 size)
{
	u32 i;

	for (i = 0; i < size; i++) {
		reg_rmw(serdes_regs + cfg[i].ofs,
			cfg[i].val,
			cfg[i].mask);
	}
}

static void serdes_cmu_setup(void __iomem *serdes_regs,
			     struct serdes_cfg *cfg_cmu,
			     u32 size)
{
	serdes_cfg_setup(serdes_regs, cfg_cmu, size);
}

static void serdes_comlane_setup(void __iomem *serdes_regs,
				 struct serdes_cfg *cfg_comlane,
				 u32 size)
{
	serdes_cfg_setup(serdes_regs, cfg_comlane, size);
}

static void serdes_lane_config(void __iomem *serdes_regs,
			       struct serdes_cfg *cfg_lane,
			       u32 size,
			       u32 lane)
{
	u32 i;

	for (i = 0; i < size; i++) {
		reg_rmw(serdes_regs + cfg_lane[i].ofs + SERDES_LANE_REGS(lane),
			cfg_lane[i].val, cfg_lane[i].mask);
	}
}

static int serdes_init_156p25m_5g(void __iomem *serdes_regs, u32 num_lanes)
{
	u32 i;

	serdes_cmu_setup(serdes_regs, cfg_cmu_156p25m_5g,
			 ARRAY_SIZE(cfg_cmu_156p25m_5g));
	serdes_comlane_setup(serdes_regs, cfg_comlane_156p25m_5g,
			     ARRAY_SIZE(cfg_comlane_156p25m_5g));
	for (i = 0; i < num_lanes; i++)
		serdes_lane_config(serdes_regs, cfg_lane_156p25mhz_5g,
				   ARRAY_SIZE(cfg_lane_156p25mhz_5g), i);
	return 0;
}

static int serdes_init_156p25m_10p3125g(void __iomem *serdes_regs,
					u32 num_lanes)
{
	/* FIXME: add support for 10G init */
	return 0;
}

static void serdes_cmu_comlane_enable(void __iomem *serdes_regs,
				      struct serdes *serdes)
{
	/* Bring SerDes out of Reset */
	reg_rmw(serdes_regs + SERDES_CMU_REG_010(0), 0x0, SERDES_RESET);
	if (serdes->intf == SERDES_PHY_PCSR)
		reg_rmw(serdes_regs + SERDES_CMU_REG_010(1), 0x0,
			SERDES_RESET);

	/* Enable CMU and COMLANE */
	reg_rmw(serdes_regs + SERDES_CMU_REG_000(0), 0x03, 0x000000ff);
	if (serdes->intf == SERDES_PHY_PCSR)
		reg_rmw(serdes_regs + SERDES_CMU_REG_000(1), 0x03, 0x000000ff);
	reg_rmw(serdes_regs + SERDES_COMLANE_REG_000, 0x5f, 0x000000ff);
}

static void serdes_pll_enable(void __iomem *serdes_regs, struct serdes *serdes)
{
	__raw_writel(serdes_cfg_pll_enable[serdes->intf],
			serdes_regs + SERDES_PLL_CTL_REG);
}

void serdes_lane_reset(void __iomem *serdes_regs, bool reset, u32 lane)
{
	if (reset)
		reg_rmw(serdes_regs + SERDES_LANE_REG_028(lane), 0x1,
			SERDES_LANE_RESET);
	else
		reg_rmw(serdes_regs + SERDES_LANE_REG_028(lane), 0x0,
			SERDES_LANE_RESET);
}

void serdes_lane_enable(void __iomem *serdes_regs, struct serdes *serdes,
			u32 lane)
{
	/* Bring lane out of reset */
	serdes_lane_reset(serdes_regs, false, lane);

	__raw_writel(SERDES_LANE_EN_VAL(serdes_cfg_lane_enable, serdes->intf,
					serdes->rate_mode),
		     serdes_regs + SERDES_LANE_CTL_STATUS_REG(lane));

	/* Set NES bit if Loopback Enabled */
	if (serdes->loopback)
		reg_rmw(serdes_regs + SERDES_LANE_REG_000(lane), 0x1,
				SERDES_LANE_LOOPBACK);
}

void serdes_lane_disable(void __iomem *serdes_regs, u32 lane)
{
	reg_rmw(serdes_regs + SERDES_LANE_CTL_STATUS_REG(lane), 0x0,
				SERDES_LANE_DISABLE_MASK);
}

void serdes_reset(void __iomem *serdes_regs, u32 num_lanes)
{
	int i;

	reg_rmw(serdes_regs + SERDES_CMU_REG_010(0), 0x1, SERDES_RESET);
	for (i = 0; i < num_lanes; i++)
		serdes_lane_reset(serdes_regs, true, i);
}

int serdes_init(void __iomem *serdes_regs, struct serdes *serdes, u32 num_lanes)
{
	int i, ret = 0;

	/* The driver currently supports 5GBaud rate with ref clock 156.25MHz */
	if (serdes->clk == SERDES_CLOCK_156P25M)
		if (serdes->rate == SERDES_RATE_5G)
			ret = serdes_init_156p25m_5g(serdes_regs, num_lanes);
		else if (serdes->rate == SERDES_RATE_10p3125g)
			ret = serdes_init_156p25m_10p3125g(serdes_regs,
							   num_lanes);
		else
			return -EINVAL;
	else
		return -EINVAL;
	serdes_cmu_comlane_enable(serdes_regs, serdes);
	for (i = 0; i < num_lanes; i++)
		serdes_lane_enable(serdes_regs, serdes, i);
	serdes_pll_enable(serdes_regs, serdes);

	return ret;
}


