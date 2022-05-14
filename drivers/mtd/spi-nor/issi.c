// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2005, Intec Automation Inc.
 * Copyright (C) 2014, Freescale Semiconductor, Inc.
 */

#include <linux/mtd/spi-nor.h>

#include "core.h"

static int
is25lp256_post_bfpt_fixups(struct spi_nor *nor,
			   const struct sfdp_parameter_header *bfpt_header,
			   const struct sfdp_bfpt *bfpt,
			   struct spi_nor_flash_parameter *params)
{
	/*
	 * IS25LP256 supports 4B opcodes, but the BFPT advertises a
	 * BFPT_DWORD1_ADDRESS_BYTES_3_ONLY address width.
	 * Overwrite the address width advertised by the BFPT.
	 */
	if ((bfpt->dwords[BFPT_DWORD(1)] & BFPT_DWORD1_ADDRESS_BYTES_MASK) ==
		BFPT_DWORD1_ADDRESS_BYTES_3_ONLY)
		nor->addr_width = 4;

	return 0;
}

static struct spi_nor_fixups is25lp256_fixups = {
	.post_bfpt = is25lp256_post_bfpt_fixups,
};

/**
 * is25wx256_set_4byte_addr_mode() - Set 4-byte address mode for Octal SPI
 * ISSI flashes.
 * @nor:        pointer to 'struct spi_nor'.
 * @enable:     true to enter the 4-byte address mode, false to exit the 4-byte
 *              address mode.
 *
 * Return: 0 on success, -errno otherwise.
 */
static int is25wx256_set_4byte_addr_mode(struct spi_nor *nor, bool enable)
{
	int ret;

	ret = spi_nor_write_enable(nor);
	if (ret)
		return ret;

	ret = spi_nor_set_4byte_addr_mode(nor, enable);
	if (ret)
		return ret;

	return spi_nor_write_disable(nor);
}

static void is25wx256_default_init(struct spi_nor *nor)
{
	/*
	 * Some manufacturer like is25wx256(Octal SPI) may use
	 * Enter/Exit 4-Byte Address Mode, we need
	 * to set it in the default_init fixup hook.
	 */
	nor->params->set_4byte_addr_mode = is25wx256_set_4byte_addr_mode;
}

static struct spi_nor_fixups is25wx256_fixups = {
	.default_init = is25wx256_default_init,
};

static struct flash_info issi_parts[] = {
	/* ISSI */
	{ "is25wp080d", INFO(0x9d7014, 0, 64 * 1024, 32, SECT_4K |
		SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_HAS_LOCK) },
	{ "is25wp016d", INFO(0x9d7015, 0, 64 * 1024, 32, SECT_4K |
		SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ | SPI_NOR_HAS_LOCK) },
	{ "is25cd512",  INFO(0x7f9d20, 0, 32 * 1024,   2, SECT_4K) },
	{ "is25lq040b", INFO(0x9d4013, 0, 64 * 1024,   8,
			     SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25lp016d", INFO(0x9d6015, 0, 64 * 1024,  32,
			     SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25lp080d", INFO(0x9d6014, 0, 64 * 1024,  16,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK) },
	{ "is25lp032",  INFO(0x9d6016, 0, 64 * 1024,  64,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_HAS_LOCK) },
	{ "is25lp064",  INFO(0x9d6017, 0, 64 * 1024, 128,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_HAS_LOCK) },
	{ "is25lp128",  INFO(0x9d6018, 0, 64 * 1024, 256,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_HAS_LOCK) },
	{ "is25lp256",  INFO(0x9d6019, 0, 64 * 1024, 512,
			     SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_4B_OPCODES | SPI_NOR_HAS_LOCK)
		.fixups = &is25lp256_fixups },
	{ "is25wp256d", INFO(0x9d7019, 0, 64 * 1024, 512,
			SECT_4K | SPI_NOR_DUAL_READ |
			SPI_NOR_QUAD_READ | SPI_NOR_HAS_LOCK |
			SPI_NOR_4B_OPCODES) },
	{ "is25wp032",  INFO(0x9d7016, 0, 64 * 1024,  64,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			SPI_NOR_HAS_LOCK) },
	{ "is25wp064",  INFO(0x9d7017, 0, 64 * 1024, 128,
			SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ
			 | SPI_NOR_HAS_LOCK) },
	{ "is25wp128",  INFO(0x9d7018, 0, 64 * 1024, 256,
			     SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ) },
	{ "is25wp256", INFO(0x9d7019, 0, 64 * 1024, 512,
			    SECT_4K | SPI_NOR_DUAL_READ | SPI_NOR_QUAD_READ |
			    SPI_NOR_4B_OPCODES)
		.fixups = &is25lp256_fixups },
	{ "is25lp512m", INFO(0x9d601a, 0, 64 * 1024, 1024,
			SECT_4K | SPI_NOR_DUAL_READ |
			SPI_NOR_QUAD_READ | SPI_NOR_HAS_LOCK) },
	{ "is25wp512m", INFO(0x9d701a, 0, 64 * 1024, 1024,
			SECT_4K | SPI_NOR_DUAL_READ |
			SPI_NOR_QUAD_READ | SPI_NOR_HAS_LOCK |
			SPI_NOR_4B_OPCODES) },
        { "is25lp01g", INFO(0x9d601b, 0, 64 * 1024, 2048,
                        SECT_4K | SPI_NOR_DUAL_READ |
                        SPI_NOR_QUAD_READ | SPI_NOR_HAS_LOCK |
                        SPI_NOR_4B_OPCODES) },
        { "is25wp01g", INFO(0x9d701b, 0, 64 * 1024, 2048,
                        SECT_4K | SPI_NOR_DUAL_READ |
                        SPI_NOR_QUAD_READ | SPI_NOR_HAS_LOCK |
                        SPI_NOR_4B_OPCODES) },
	{ "is25wx256",  INFO(0x9d5b19, 0, 128 * 1024, 256,
			     SECT_4K | USE_FSR | SPI_NOR_OCTAL_READ |
			     SPI_NOR_OCTAL_WRITE | SPI_NOR_4B_OPCODES)
		.fixups = &is25wx256_fixups },

	/* PMC */
	{ "pm25lv512",   INFO(0,        0, 32 * 1024,    2, SECT_4K_PMC) },
	{ "pm25lv010",   INFO(0,        0, 32 * 1024,    4, SECT_4K_PMC) },
	{ "pm25lq032",   INFO(0x7f9d46, 0, 64 * 1024,   64, SECT_4K) },
};

static void issi_default_init(struct spi_nor *nor)
{
	nor->params->quad_enable = spi_nor_sr1_bit6_quad_enable;
}

static const struct spi_nor_fixups issi_fixups = {
	.default_init = issi_default_init,
};

const struct spi_nor_manufacturer spi_nor_issi = {
	.name = "issi",
	.parts = issi_parts,
	.nparts = ARRAY_SIZE(issi_parts),
	.fixups = &issi_fixups,
};
