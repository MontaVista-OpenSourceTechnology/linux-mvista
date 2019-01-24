/*
 * Board setup routines for Ericsson PUMA-1 Board
 *
 * Copyright (C) 2010 Wind River Systems, Inc.
 * Copyright (C) 2014 - 2019 MontaVista Software, LLC.
 * (Modified by Niyas Ahamed Mydeen <nmydeen@mvista.com>
 * for MontaVista Software, LLC.)
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

#include <linux/mtd/physmap.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/memory.h>
#include <linux/platform_data/at24.h>
#include <linux/i2c.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <mach/core.h>
#include <mach/io.h>
#include <mach/emac.h>
#include <mach/emif.h>
#include <mach/nand.h>
#include <linux/mtd/rawnand.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/irqchip.h>
#include <linux/of_irq.h>
#include <linux/clocksource.h>

extern void __init PUMA_timer_init(void);
extern int __init cp_intc_of_init(struct device_node *node,
					struct device_node *parent);
/*-------------------------------------------------------------------------*/
/*-                           MTD                                         -*/
/*-------------------------------------------------------------------------*/

/* SCMB FLASH Memory Map */

/* NOR Flash */

/* FLASH 1 NOR for MTD driver (MCS_EXT_CS0 - EMIF 2.5#1 CS2-CS3) */
#define SCMB_NOR_FLASH_1_START (0x00000000)
#define SCMB_NOR_FLASH_1_SIZE  (0x01000000) /* 16 MByte */
#define SCMB_NOR_FLASH_1_END   (SCMB_NOR_FLASH_1_START + SCMB_NOR_FLASH_1_SIZE - 1)

static struct physmap_flash_data scmb_flash_data = {
	.width    = 2, /* bank width in octets */
};

static struct resource scmb_nor_flash_resource[] = {
	[0] = {
		.start = SCMB_NOR_FLASH_1_START,
		.end   = SCMB_NOR_FLASH_1_END,
		.flags = IORESOURCE_MEM
	}
};

static struct platform_device scmb_nor_mtd_device = {
	.name          = "physmap-flash",
	.id            = 0,
	.dev           = {
		.platform_data = &scmb_flash_data
	},

	.num_resources = ARRAY_SIZE(scmb_nor_flash_resource),
	.resource      = &(scmb_nor_flash_resource[0])
};

/* NAND Flash */

/* FLASH 1 NAND for MTD driver (MCS_EXT_CS2 - EMIF 2.5#2 CS2) */
#define SCMB_NAND_FLASH_1_START (0x10000000)
#define SCMB_NAND_FLASH_1_SIZE  (0x20000000) /* 512 MByte */
#define SCMB_NAND_FLASH_1_END   (SCMB_NAND_FLASH_1_START + SCMB_NAND_FLASH_1_SIZE - 1)

/* FLASH NAND memory space opened only with 256 bytes because NAND interface uses address up to 0x80 */
#define SCMB_NAND_FLASH_MEMORY_SPACE_SIZE (0x80) /* 256 Bytes */

static struct puma_nand_pdata scmb_nand_flash_data = {
	.mask_ale	= ALE_TRIGGER_ADDR,
	.mask_cle	= CLE_TRIGGER_ADDR,
	.ecc_mode	= NAND_ECC_HW_OOB_FIRST,
	.ecc_bits	= 4,
	.options	= (NAND_SAMSUNG_LP_OPTIONS |
					NAND_BUSWIDTH_16 |
					NAND_NO_SUBPAGE_WRITE),
	.bbt_options	= NAND_BBT_USE_FLASH,
	.bbt_td		= NULL, /* use default bbt */
	.bbt_md		= NULL, /* use default mirror bbt */

};

static struct resource scmb_nand_flash_resources[] = {
	/* First memory resource is EMIF 2.5#1 base address */
	[0] = {
		.start = PUMA_EMIF2_5_1_BASE,
		.end   = PUMA_EMIF2_5_1_BASE + PUMA_EMIF2_5_SIZE - 1,
		.flags = IORESOURCE_MEM
	},

	/* Second memory resource is NAND chip select base address */
	[1] = {
		.start = SCMB_NAND_FLASH_1_START,
		.end   = SCMB_NAND_FLASH_1_START + SCMB_NAND_FLASH_MEMORY_SPACE_SIZE - 1,
		.flags = IORESOURCE_MEM
	}
};

static struct platform_device scmb_nand_mtd_device = {
	.name          = "puma_nand",
	.id            = 0,
	.dev           = {
		.platform_data = &scmb_nand_flash_data
	},

	.num_resources = ARRAY_SIZE(scmb_nand_flash_resources),
	.resource      = scmb_nand_flash_resources
};
/*-------------------------------------------------------------------------*/
/*-                           I2C                                         -*/
/*-------------------------------------------------------------------------*/

#ifdef SCMB_I2C_E2P
extern void read_inventory_mac(struct memory_accessor *a, void *context);

static struct at24_platform_data inventory_eeprom = {
	.byte_len  = 65536,
	.page_size = 128,
	.flags     = (AT24_FLAG_ADDR16 | AT24_FLAG_IRUGO),
	.setup     = read_inventory_mac,
	.context   = (void *)0x7F00
};
#endif

static struct i2c_board_info scmb_i2c_info[] __initdata = {
	{
		I2C_BOARD_INFO("lm75", 0x49)
	},

	{
		I2C_BOARD_INFO("ds1341", 0x68)
	},
#ifdef SCMB_I2C_E2P
	{
		I2C_BOARD_INFO("24c512", 0x57),
		.platform_data = &inventory_eeprom
	}
#endif
};

void __init board_init_i2c(void)
{
	i2c_register_board_info(1, scmb_i2c_info, ARRAY_SIZE(scmb_i2c_info));
}

#define SCMB_EMAC_PHY_MASK (0x0<<0x10)

char *PUMA_mac_addr;

void board_init_phy(struct emac_platform_data *pdata)
{
	pdata->phy_mask = SCMB_EMAC_PHY_MASK;
	PUMA_mac_addr = pdata->mac_addr;
}

extern struct sys_timer PUMA_timer;

static void __init __maybe_unused board_init(void)
{
	PUMA_init();
	platform_device_register(&scmb_nor_mtd_device);
	platform_device_register(&scmb_nand_mtd_device);
}

#if 0
static void PUMA_restart(char mode, const char *cmd)
{
	PUMA_watchdog_reset();
}
#endif

#ifdef CONFIG_MACH_SCMB_DT
static const char *puma1_boards_compat_dt[] __initconst = {
		"ericsson,puma1",
		NULL,
};
static struct of_dev_auxdata puma1_auxdata_lookup[] __initdata = {
	OF_DEV_AUXDATA("puma,i2c_PUMA", 0x6a000000, "i2c", NULL),
    /* End */
};
static const struct of_device_id puma1_local_bus_nodes[] = {
		{ .compatible = "simple-bus",},
		{ .compatible = "arm,amba-bus", },
		{} /* Empty terminated list */
};

static void __init puma1_init_machine(void)
{
		/* probe child nodes of puma2 device */
		of_platform_populate(NULL, puma1_local_bus_nodes,
				puma1_auxdata_lookup, NULL);
}

static struct of_device_id puma1_irq_match[] __initdata = {
		{ .compatible = "ti,cp-intc", .data = cp_intc_of_init, },
		{ }
};


static void __init puma1_init_irq(void)
{
		if (of_have_populated_dt())
			of_irq_init(puma1_irq_match);
}
DT_MACHINE_START(SCMB_DT, "iPT1.5 SCMB with PUMA1 SoC (Flattened Device Tree)")
		.nr_irqs        = NR_IRQS_LEGACY,
		.map_io         = PUMA_map_io,
		.init_irq       = puma1_init_irq,
		.init_time      = PUMA_timer_init,
		.init_machine   = puma1_init_machine,
		.init_late      = NULL,
		.dt_compat      = puma1_boards_compat_dt,
		.restart        = PUMA_restart,
MACHINE_END
#endif
MACHINE_START(SCMB, "PT1.5 SCMB with PUMA1 SoC")
		.atag_offset    = 0x100,
		.map_io         = PUMA_map_io,
		.init_irq       = PUMA_init_irq,
		.init_time      = PUMA_timer_init,
		.init_machine   = board_init,
		.restart        = PUMA_restart,
MACHINE_END
