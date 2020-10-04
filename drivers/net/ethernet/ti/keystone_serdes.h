/*
 * Texas Instruments Keystone SerDes APIs
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
#ifndef __TI_KEYSTONE_SERDES_H__
#define __TI_KEYSTONE_SERDES_H__

/* SERDES Reference clock */
enum SERDES_CLOCK {
	SERDES_CLOCK_100M,		/* 100 MHz */
	SERDES_CLOCK_122P88M,		/* 122.88 MHz */
	SERDES_CLOCK_125M,		/* 125 MHz */
	SERDES_CLOCK_156P25M,		/* 156.25 MHz */
	SERDES_CLOCK_312P5M,		/* 312.5 MHz */
};

/* SERDES Lane Baud Rate */
enum SERDES_RATE {
	SERDES_RATE_4P9152G,		/* 4.9152 GBaud */
	SERDES_RATE_5G,			/* 5 GBaud */
	SERDES_RATE_6P144G,		/* 6.144 GBaud */
	SERDES_RATE_6P25G,		/* 6.25 GBaud */
	SERDES_RATE_10p3125g,		/* 10.3215 GBaud */
	SERDES_RATE_12p5g,		/* 12.5 GBaud */
};

/* SERDES Lane Rate Mode */
enum SERDES_RATE_MODE {
	SERDES_FULL_RATE,
	SERDES_HALF_RATE,
	SERDES_QUARTER_RATE,
};

/* SERDES PHY TYPE */
enum SERDES_INTERFACE {
	SERDES_PHY_SGMII,
	SERDES_PHY_PCSR,		/* XGE SERDES */
};

struct serdes {
	enum SERDES_CLOCK	clk;
	enum SERDES_RATE	rate;
	enum SERDES_RATE_MODE	rate_mode;
	enum SERDES_INTERFACE	intf;
	u32			loopback;
};

void serdes_reset(void __iomem *serdes_regs, u32 num_lanes);
void serdes_lane_reset(void __iomem *serdes_regs, bool reset, u32 lane);
void serdes_lane_enable(void __iomem *serdes_regs, struct serdes *serdes,
			u32 lane);
void serdes_lane_disable(void __iomem *serdes_regs, u32 lane);
int serdes_init(void __iomem *serdes_regs, struct serdes *serdes,
		 u32 num_lanes);

#endif

