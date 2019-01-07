/*
 * mach-PUMA/include/mach/nand.h
 *
 * Copyright (C) 2014 - 2019 MontaVista, Software, LLC.
 * Copyright © 2006 Texas Instruments.
 *
 * Ported to 2.6.23 Copyright © 2008 by
 *   Sander Huijsen <Shuijsen@optelecom-nkf.com>
 *   Troy Kisky <troy.kisky@boundarydevices.com>
 *   Dirk Behme <Dirk.Behme@gmail.com>
 *
 * --------------------------------------------------------------------------
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This is the porting for PUMA SoC hardware.
 */

#ifndef __ARCH_ARM_PUMA_NAND_H
#define __ARCH_ARM_PUMA_NAND_H

#include <mach/emif.h>
#include <linux/mtd/rawnand.h>

/* EMIF NAND Register */

/* NAND Flash Control Register */
#define NANDFCR_OFFSET     (0x60)

/* NAND Flash Status Register */
#define NANDFSR_OFFSET     (0x64)

/* NAND Flash 1-Bit ECC Registers */
#define NANDF1ECC_OFFSET   (0x70)
#define NANDF2ECC_OFFSET   (0x74)
#define NANDF3ECC_OFFSET   (0x78)
#define NANDF4ECC_OFFSET   (0x7C)

/* NAND 4-bit ECC syndrome registers */
#define NAND_4BIT_ECC_LOAD_OFFSET (0xBC)
#define NAND_4BIT_ECC1_OFFSET     (0xC0)
#define NAND_4BIT_ECC2_OFFSET     (0xC4)
#define NAND_4BIT_ECC3_OFFSET     (0xC8)
#define NAND_4BIT_ECC4_OFFSET     (0xCC)
#define NAND_ERR_ADD1_OFFSET      (0xD0)
#define NAND_ERR_ADD2_OFFSET      (0xD4)
#define NAND_ERR_ERRVAL1_OFFSET   (0xD8)
#define NAND_ERR_ERRVAL2_OFFSET   (0xDC)

#define NANDFCR_NAND_ENABLE(cs)         (1 << (cs-2))
#define NANDFCR_4BIT_ECC_SEL_MASK       (3 << 4)
#define NANDFCR_4BIT_ECC_SEL(cs)        ((cs-2) << 4)
#define NANDFCR_1BIT_ECC_START(cs)      (1 << (8 + (cs-2)))
#define NANDFCR_4BIT_ECC_START          (1 << 12)
#define NANDFCR_4BIT_CALC_START         (1 << 13)
#define NANDFCR_CS2NAND                 (1 << 0)

/*
 * In PUMA SoC, the NAND latch enable signal aren't connected to GPI/O pins
 * but are associated to a specific Address bit of the EMIF Chip select connected
 * to the NAND device.
 * In order to move the related signal, it's needed an access to the proper address
 * that is the EMIF base address with the correct offset
 */
#define CLE_TRIGGER_ADDR (0x40)
#define ALE_TRIGGER_ADDR (0x80)

/* NAND platform_data structure */
struct puma_nand_pdata {
	uint32_t mask_ale;
	uint32_t mask_cle;

	/* for packages using two chipselects */
	uint32_t mask_chipsel;

	/* board's default static partition info */
	struct mtd_partition *parts;
	unsigned int nr_parts;

	/*
	 * none    == NAND_ECC_NONE (strongly *not* advised!!)
	 * soft    == NAND_ECC_SOFT
	 * hw 1bit == NAND_ECC_HW, according to ecc_bits
	 * hw 4bit == NAND_ECC_HW_OOB_FIRST
	 *
	 * All PUMA-family chips support 1-bit & 4-bit hardware ECC.
	 */
	nand_ecc_modes_t ecc_mode;
	u8 ecc_bits;

	/* e.g. NAND_BUSWIDTH_16 */
	unsigned int options;
	/* e.g. NAND_BBT_USE_FLASH */
	unsigned int               bbt_options;

	/* Main and mirror bbt descriptor overrides */
	struct nand_bbt_descr *bbt_td;
	struct nand_bbt_descr *bbt_md;
};
extern struct mtd_info *__mtd_next_device(int i);
extern int add_mtd_device(struct mtd_info *mtd);
extern int del_mtd_device(struct mtd_info *mtd);
extern int add_mtd_partitions(struct mtd_info *, const struct mtd_partition *, int);
extern int del_mtd_partitions(struct mtd_info *);
extern int parse_mtd_partitions(struct mtd_info *master, const char * const *types,
			struct mtd_partition **pparts,
			struct mtd_part_parser_data *data);
#endif /* __ARCH_ARM_PUMA_NAND_H */
