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

/* SERDES Reference clock KHz */
enum KSERDES_CLOCK_RATE {
	KSERDES_CLOCK_RATE_100M		= 100000,
	KSERDES_CLOCK_RATE_122P88M	= 122880,
	KSERDES_CLOCK_RATE_125M		= 125000,
	KSERDES_CLOCK_RATE_153P6M	= 153600,
	KSERDES_CLOCK_RATE_156P25M	= 156250,
	KSERDES_CLOCK_RATE_312P5M	= 312500,
};

/* SERDES Link Rate Kbps */
enum KSERDES_LINK_RATE {
	KSERDES_LINK_RATE_1P25G		=  1250000,
	KSERDES_LINK_RATE_3P125G	=  3125000,
	KSERDES_LINK_RATE_4P9152G	=  4915200,
	KSERDES_LINK_RATE_5G		=  5000000,
	KSERDES_LINK_RATE_6P144G	=  6144000,
	KSERDES_LINK_RATE_6P25G		=  6250000,
	KSERDES_LINK_RATE_7P3728G	=  7372800,
	KSERDES_LINK_RATE_9P8304G	=  9830400,
	KSERDES_LINK_RATE_10G		= 10000000,
	KSERDES_LINK_RATE_10P3125G	= 10312500,
	KSERDES_LINK_RATE_12P5G		= 12500000,
};

/* SERDES Lane Control Rate */
enum KSERDES_LANE_CTRL_RATE {
	KSERDES_FULL_RATE,
	KSERDES_HALF_RATE,
	KSERDES_QUARTER_RATE,
};

enum KSERDES_PHY_TYPE {
	KSERDES_PHY_XGE,
	KSERDES_PHY_SGMII,
	KSERDES_PHY_PCIE,
	KSERDES_PHY_HYPERLINK,
};

#define KSERDES_FLAG_ENABLE	0x1

struct kserdes_tx_coeff {
	u32	c1;
	u32	c2;
	u32	cm;
	u32	att;
	u32	vreg;
};

struct kserdes_equalizer {
	u32	att;
	u32	boost;
};

struct kserdes_lane_config {
	u32				enable;
	u32				ctrl_rate;
	struct kserdes_tx_coeff		tx_coeff;
	struct kserdes_equalizer	rx_start;
	struct kserdes_equalizer	rx_force;
	u32				loopback;
};

#define KSERDES_MAX_LANES		4

struct kserdes_fw_config {
	bool				on;
	u32				rate;
	u32				link_loss_wait;
	u32				lane_seeds;
	u32				fast_train;
	u32				active_lane;
	u32				c1, c2, cm, attn, boost, dlpf, cdrcal;
	u32				lane_config[KSERDES_MAX_LANES];
};

#define MAX_COEFS		5
struct kserdes_lane_dlev_out {
	u32 delay;
	int coef_vals[MAX_COEFS];
};

struct kserdes_dlev_out {
	struct kserdes_lane_dlev_out lane_dlev_out[KSERDES_MAX_LANES];
};

struct kserdes_cmp_coef_ofs {
	u32 cmp;
	u32 coef1;
	u32 coef2;
	u32 coef3;
	u32 coef4;
	u32 coef5;
};

#define MAX_CMP			5
struct kserdes_lane_ofs {
	struct kserdes_cmp_coef_ofs ct_ofs[MAX_CMP];
};

struct kserdes_ofs {
	struct kserdes_lane_ofs lane_ofs[KSERDES_MAX_LANES];
};

struct kserdes_config {
	struct device			*dev;
	enum KSERDES_CLOCK_RATE		clk_rate;
	enum KSERDES_PHY_TYPE		phy_type;
	u32				lanes;
	bool				debug;
	void __iomem			*regs;
	void __iomem			*sw_regs;

	void __iomem			*peripheral_regmap;
	void __iomem			*pcsr_regmap;

	const char			*init_fw;
	struct serdes_cfg		*init_cfg;
	int				init_cfg_len;

	/* non-fw specific */
	enum KSERDES_LINK_RATE		link_rate;
	u32				rx_force_enable;
	struct kserdes_lane_config	lane[KSERDES_MAX_LANES];
	struct kserdes_ofs		sofs;
	spinlock_t 			*tbus_lock;

	u32				prev_cpu_ctrl_reg;

	/* fw specific */
	bool				firmware;
	struct kserdes_fw_config	fw;
};

int kserdes_init(struct kserdes_config *sc);
int kserdes_lanes_enable(struct kserdes_config *sc);
int kserdes_get_serdes_bindings(const char *dev,
				struct device_node *np,
				struct kserdes_config *sc);

int kserdes_phy_enable_rx(struct kserdes_config *sc, u32 lane);
int kserdes_phy_reset(struct kserdes_config *sc, u32 lane);
int kserdes_of_parse(struct device *dev, struct kserdes_config *sc,
			void __iomem *sw_ss_regs,
			void __iomem *pcsr_port_regs,
			struct device_node *np);
int kserdes_provider_init(struct kserdes_config *sc);
#endif
