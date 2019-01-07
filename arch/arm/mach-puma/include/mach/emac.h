/*
 * Copyright (C) 2014 - 2019 MontaVista, Software, LLC.
 * Copyright (C) 2010 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_EMAC_H
#define __ASM_ARCH_EMAC_H

#include <linux/if_ether.h>	/* ETH_ALEN */

#define PUMA_EMAC_MOD_REG_OFFSET  (0x0000000)
#define PUMA_EMAC_CTRL_REG_OFFSET (0x0100000)
#define PUMA_MDIO_REG_OFFSET      (0x0200000)
#define PUMA_EMAC_RAM_OFFSET      (0x0300000)
#define PUMA_EMAC_CTRL_RAM_SIZE   (SZ_8K)

struct emac_platform_data {
	char mac_addr[ETH_ALEN];
	u32 ctrl_reg_offset;
	u32 ctrl_mod_reg_offset;
	u32 ctrl_ram_offset;
	u32 hw_ram_addr;
	u32 mdio_reg_offset;
	u32 ctrl_ram_size;
	u32 phy_mask;
	u32 mdio_max_freq;
	u8 rmii_en;
	u8 version;
	void (*interrupt_enable)(void);
	void (*interrupt_disable)(void);
};

enum {
	EMAC_VERSION_1,
	EMAC_VERSION_2,
};

#endif
