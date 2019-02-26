/*
 * puma_nand.c - NAND Flash Driver for PUMA SoC family chips
 *
 * Copyright © 2006 Texas Instruments.
 *
 * Port to 2.6.23 Copyright © 2008 by:
 *   Sander Huijsen <Shuijsen@optelecom-nkf.com>
 *   Troy Kisky <troy.kisky@boundarydevices.com>
 *   Dirk Behme <Dirk.Behme@gmail.com>
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
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/mtd/rawnand.h>
#include <linux/mtd/partitions.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/of_device.h>

#include <mach/nand.h>

#include <asm/mach-types.h>
#define CONFIG_MTD_DEBUG
#ifdef CONFIG_MTD_DEBUG
#define DEBUG(args...)                  \
	do {                                \
		if (1)                          \
			printk(KERN_INFO args);     \
	} while (0)
#else /* CONFIG_MTD_DEBUG */
#define DEBUG(args...)                  \
	do {                                \
		if (0)                          \
			printk(KERN_INFO args);     \
	} while (0)
#endif /* CONFIG_MTD_DEBUG */

/*
 * This is a device driver for the NAND flash controller found on the
 * various DaVinci family chips. It handles up to four SoC chipselects,
 * and some flavors of secondary chipselect (e.g. based on A12) as used
 * with multichip packages.
 *
 * The 1-bit ECC hardware is supported, as well as the newer 4-bit ECC
 * available on chips like the DM355 and OMAP-L137 and needed with the
 * more error-prone MLC NAND chips.
 *
 * This is the driver porting for PUMA SoC hardware.
 */
struct puma_nand_info {
	struct nand_chip      chip;

	struct device         *dev;

	bool                  is_readmode;

	void __iomem          *emif_base;
	void __iomem          *nand_base;

	uint32_t              ioaddr;
	uint32_t              current_cs;

	uint32_t              mask_chipsel;
	uint32_t              mask_ale;
	uint32_t              mask_cle;

	uint32_t              core_chipsel;
};

static DEFINE_SPINLOCK(puma_nand_lock);

static inline struct puma_nand_info *to_puma_nand(struct mtd_info *mtd)
{
	return container_of(mtd_to_nand(mtd), struct puma_nand_info, chip);
}

static inline unsigned int puma_emif_readl(struct puma_nand_info *info, int offset)
{
	return __raw_readl(info->emif_base + offset);
}

static inline unsigned int puma_nand_readb(struct puma_nand_info *info, int offset)
{
	return __raw_readb(info->nand_base + offset);
}

static inline void puma_emif_writel(struct puma_nand_info *info, int offset, unsigned long value)
{
	__raw_writel(value, info->emif_base + offset);
}

static inline void puma_nand_writeb(struct puma_nand_info *info, int offset, unsigned long value)
{
	__raw_writeb(value, info->nand_base + offset);
}

/* Access to hardware control lines: ALE, CLE. */
static void nand_puma_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct puma_nand_info *info = to_puma_nand(mtd);
	uint32_t               addr = info->current_cs;
	struct nand_chip      *nand = mtd_to_nand(mtd);

	/* Did the control lines change? */
	if (ctrl & NAND_CTRL_CHANGE) {
		if ((ctrl & NAND_CTRL_CLE) == NAND_CTRL_CLE)
			addr |= info->mask_cle;
		else if ((ctrl & NAND_CTRL_ALE) == NAND_CTRL_ALE)
			addr |= info->mask_ale;

		nand->IO_ADDR_W = (void __iomem __force *)addr;
	}

	if (cmd != NAND_CMD_NONE) {
		iowrite8(cmd, nand->IO_ADDR_W);
		udelay(nand->chip_delay);
	}
}

static void nand_puma_select_chip(struct mtd_info *mtd, int chip)
{
	struct puma_nand_info *info = to_puma_nand(mtd);
	uint32_t              nand_base = info->ioaddr;

	/* maybe kick in a second chipselect */
	if (chip > 0)
		nand_base |= info->mask_chipsel;
	info->current_cs = nand_base;

	info->chip.IO_ADDR_W = (void __iomem __force *)nand_base;
	info->chip.IO_ADDR_R = info->chip.IO_ADDR_W;
}

/* 1-bit hardware ECC calculated through the EMIF */
static inline uint32_t nand_puma_readecc_1bit(struct mtd_info *mtd)
{
	struct puma_nand_info *info = to_puma_nand(mtd);
	uint32_t nand_ecc = 0;

	/*
	 * NAND Flash is on EMIF 2.5#1 Chip Select 4,
	 * so use NAND Flash CS4 1-Bit ECC Register
	 * (for pad_cs_o_n[4]) = NANDF3ECC
	 */
	nand_ecc = puma_emif_readl(info, NANDF3ECC_OFFSET);

	return nand_ecc;
}

static void nand_puma_hwctl_1bit(struct mtd_info *mtd, int mode)
{
	struct puma_nand_info *info;
	uint32_t nandcfr = 0;
	unsigned long flags;
	uint32_t dummy_read;

	info = to_puma_nand(mtd);

	/* Reset ECC hardware */
	dummy_read = puma_emif_readl(info, NANDF1ECC_OFFSET);
	dummy_read = puma_emif_readl(info, NANDF2ECC_OFFSET);
	dummy_read = puma_emif_readl(info, NANDF3ECC_OFFSET);
	dummy_read = puma_emif_readl(info, NANDF4ECC_OFFSET);

	spin_lock_irqsave(&puma_nand_lock, flags);

	/* Restart ECC hardware */
	nandcfr = puma_emif_readl(info, NANDFCR_OFFSET);
	/* NAND Flash 1-bit ECC start for chip select 4: cs4_ecc_start (Bit 10) = 1 */
	nandcfr |= (1 << 10);
	puma_emif_writel(info, NANDFCR_OFFSET, nandcfr);

	spin_unlock_irqrestore(&puma_nand_lock, flags);
}

/*
 * Read hardware ECC value and pack into three bytes
 */
static int nand_puma_calculate_1bit(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	unsigned int ecc_val = nand_puma_readecc_1bit(mtd);

	/* squeeze 4 bytes ECC into 3 bytes by removing RESERVED bits
	 * and shifting. RESERVED bits are 31 to 28 and 15 to 12.
	 */
	unsigned int ecc24 = (ecc_val & 0x0fff) | ((ecc_val & 0x0fff0000) >> 4);

	/* invert so that erased block ecc is correct */
	ecc24 = ~ecc24;
	ecc_code[0] = (u_char)(ecc24);
	ecc_code[1] = (u_char)(ecc24 >> 8);
	ecc_code[2] = (u_char)(ecc24 >> 16);

	return 0;
}

static int nand_puma_correct_1bit(struct mtd_info *mtd, u_char *dat, u_char *read_ecc, u_char *calc_ecc)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	uint32_t eccNand = read_ecc[0] | (read_ecc[1] << 8) | (read_ecc[2] << 16);
	uint32_t eccCalc = calc_ecc[0] | (calc_ecc[1] << 8) | (calc_ecc[2] << 16);
	uint32_t diff = eccCalc ^ eccNand;

	if (diff) {
		if ((((diff >> 12) ^ diff) & 0xfff) == 0xfff) {
			/* Correctable error */
			if ((diff >> (12 + 3)) < chip->ecc.size) {
				dat[diff >> (12 + 3)] ^= BIT((diff >> 12) & 7);
				return 1;
			} else {
				return -1;
			}
		} else if (!(diff & (diff - 1))) {
			/* Single bit ECC error in the ECC itself, nothing to fix */
			return 1;
		} else {
			/* Uncorrectable error */
			return -1;
		}
	}
	return 0;
}

/*----------------------------------------------------------------------*/

/*
 * 4-bit hardware ECC ... context maintained over entire AEMIF
 *
 * This is a syndrome engine, but we avoid NAND_ECC_HW_SYNDROME
 * since that forces use of a problematic "infix OOB" layout.
 * Among other things, it trashes manufacturer bad block markers.
 * Also, and specific to this hardware, it ECC-protects the "prepad"
 * in the OOB ... while having ECC protection for parts of OOB would
 * seem useful, the current MTD stack sometimes wants to update the
 * OOB without recomputing ECC.
 */

#define NAND_4BITECC_MASK               0x03FF03FF
#define NAND_FSR_ECC_STATE_MASK         0x00000F00

#define ECC_STATE_NO_ERR                0x0
#define ECC_STATE_TOO_MANY_ERRS         0x1
#define ECC_STATE_ERR_CORR_COMP_P       0x2
#define ECC_STATE_ERR_CORR_COMP_N       0x3

#define CONFIG_SYS_NAND_CS              4   /* This should probably be set somewhere else.... */

static void nand_puma_hwctl_4bit(struct mtd_info *mtd, int mode)
{
	u32 val;
	struct puma_nand_info *info = to_puma_nand(mtd);

	unsigned long flags;

	spin_lock_irqsave(&puma_nand_lock, flags);
	/*
	 * Start a new ECC calculation for reading or writing 512 bytes
	 * of data.
	 */
	val = puma_emif_readl(info, NANDFCR_OFFSET);
	val &= ~NANDFCR_4BIT_ECC_SEL_MASK;
	val |= NANDFCR_NAND_ENABLE(CONFIG_SYS_NAND_CS);
	val |= NANDFCR_4BIT_ECC_SEL(CONFIG_SYS_NAND_CS);
	val |= NANDFCR_4BIT_ECC_START;
	puma_emif_writel(info, NANDFCR_OFFSET, val);

	info->is_readmode = (mode == NAND_ECC_READ);

	spin_unlock_irqrestore(&puma_nand_lock, flags);
}

/* Read raw ECC code after writing to NAND. */
static u32 nand_puma_readecc_4bit(struct puma_nand_info *info, unsigned int ecc[4])
{
	int i;

	for (i = 0; i < 4; i++) {
		ecc[i] = puma_emif_readl(info, NAND_4BIT_ECC1_OFFSET + (i*4)) & NAND_4BITECC_MASK;
	}

	return 0;
}

/* Terminate read ECC; or return ECC (as bytes) of data written to NAND. */
static int nand_puma_calculate_4bit(struct mtd_info *mtd, const uint8_t *dat, uint8_t *ecc_code)
{
	struct puma_nand_info *info = to_puma_nand(mtd);
	unsigned int hw_4ecc[4];
	unsigned int i;

	nand_puma_readecc_4bit(info, hw_4ecc);

	/*Convert 10 bit ecc value to 8 bit */
	for (i = 0; i < 2; i++) {
		unsigned int hw_ecc_low = hw_4ecc[i * 2];
		unsigned int hw_ecc_hi  = hw_4ecc[(i * 2) + 1];

		/* Take first 8 bits from val1 (count1=0) or val5 (count1=1) */
		*ecc_code++ = hw_ecc_low & 0xFF;

		/*
		 * Take 2 bits as LSB bits from val1 (count1=0) or val5
		 * (count1=1) and 6 bits from val2 (count1=0) or
		 * val5 (count1=1)
		 */
		*ecc_code++ = ((hw_ecc_low >> 8) & 0x3) | ((hw_ecc_low >> 14) & 0xFC);

		/*
		 * Take 4 bits from val2 (count1=0) or val5 (count1=1) and
		 * 4 bits from val3 (count1=0) or val6 (count1=1)
		 */
		*ecc_code++ = ((hw_ecc_low >> 22) & 0xF) | ((hw_ecc_hi << 4) & 0xF0);

		/*
		 * Take 6 bits from val3(count1=0) or val6 (count1=1) and
		 * 2 bits from val4 (count1=0) or  val7 (count1=1)
		 */
		*ecc_code++ = ((hw_ecc_hi >> 4) & 0x3F) | ((hw_ecc_hi >> 10) & 0xC0);

		/* Take 8 bits from val4 (count1=0) or val7 (count1=1) */
		*ecc_code++ = (hw_ecc_hi >> 18) & 0xFF;
	}

	return 0;
}

/* Correct up to 4 bits in data we just read, using state left in the
 * hardware plus the ecc_code computed when it was first written.
 */
static int nand_puma_correct_4bit(struct mtd_info *mtd, uint8_t *data, uint8_t *ecc_code, uint8_t *calc_ecc)
{
	struct puma_nand_info *info = to_puma_nand(mtd);
	int i;
	unsigned int hw_4ecc[4];
	unsigned int iserror;
	unsigned short *ecc16;
	unsigned int numerrors, erroraddress, errorvalue;
	u32 val;
	unsigned long timeo;

	/*
	 * Check for an ECC where all bytes are 0xFF.  If this is the case, we
	 * will assume we are looking at an erased page and we should ignore
	 * the ECC.
	 */
	for (i = 0; i < 10; i++) {
		if (ecc_code[i] != 0xFF)
			break;
	}
	if (i == 10)
		return 0;

	/* Convert 8 bit in to 10 bit */
	ecc16 = (unsigned short *)&ecc_code[0];

	/*
	 * Write the parity values in the NAND Flash 4-bit ECC Load register.
	 * Write each parity value one at a time starting from 4bit_ecc_val8
	 * to 4bit_ecc_val1.
	 */

	/*Take 2 bits from 8th byte and 8 bits from 9th byte */
	puma_emif_writel(info, NAND_4BIT_ECC_LOAD_OFFSET, ((ecc16[4]) >> 6) & 0x3FF);

	/* Take 4 bits from 7th byte and 6 bits from 8th byte */
	puma_emif_writel(info, NAND_4BIT_ECC_LOAD_OFFSET, (((ecc16[3]) >> 12) & 0xF) | ((((ecc16[4])) << 4) & 0x3F0));

	/* Take 6 bits from 6th byte and 4 bits from 7th byte */
	puma_emif_writel(info, NAND_4BIT_ECC_LOAD_OFFSET, ((ecc16[3]) >> 2) & 0x3FF);

	/* Take 8 bits from 5th byte and 2 bits from 6th byte */
	puma_emif_writel(info, NAND_4BIT_ECC_LOAD_OFFSET, ((((ecc16[2]) >> 8) | ((((ecc16[3])) << 8) & 0x300))));

	/*Take 2 bits from 3rd byte and 8 bits from 4th byte */
	puma_emif_writel(info, NAND_4BIT_ECC_LOAD_OFFSET, (((ecc16[1]) >> 14) & 0x3) | ((((ecc16[2])) << 2) & 0x3FC));

	/* Take 4 bits form 2nd bytes and 6 bits from 3rd bytes */
	puma_emif_writel(info, NAND_4BIT_ECC_LOAD_OFFSET, ((ecc16[1]) >> 4) & 0x3FF);

	/* Take 6 bits from 1st byte and 4 bits from 2nd byte */
	puma_emif_writel(info, NAND_4BIT_ECC_LOAD_OFFSET, (((ecc16[0]) >> 10) & 0x3F) | (((ecc16[1]) << 6) & 0x3C0));

	/* Take 10 bits from 0th and 1st bytes */
	puma_emif_writel(info, NAND_4BIT_ECC_LOAD_OFFSET, (ecc16[0]) & 0x3FF);

	/*
	 * Perform a dummy read to the EMIF Revision Code and Status register.
	 * This is required to ensure time for syndrome calculation after
	 * writing the ECC values in previous step.
	 */

	val = puma_emif_readl(info, NANDFSR_OFFSET);

	/*
	 * Read the syndrome from the NAND Flash 4-Bit ECC 1-4 registers.
	 * A syndrome value of 0 means no bit errors. If the syndrome is
	 * non-zero then go further otherwise return.
	 */
	nand_puma_readecc_4bit(info, hw_4ecc);

	if (!(hw_4ecc[0] | hw_4ecc[1] | hw_4ecc[2] | hw_4ecc[3]))
		return 0;

	/*
	 * Clear any previous address calculation by doing a dummy read of an
	 * error address register.
	 */
	val = puma_emif_readl(info, NAND_ERR_ADD1_OFFSET);

	/*
	 * Set the addr_calc_st bit(bit no 13) in the NAND Flash Control
	 * register to 1.
	 */
	puma_emif_writel(info, NANDFCR_OFFSET, puma_emif_readl(info, NANDFCR_OFFSET) | NANDFCR_4BIT_CALC_START);

	/*
	 * Wait for the corr_state field (bits 8 to 11) in the
	 * NAND Flash Status register to be not equal to 0x0, 0x1, 0x2, or 0x3.
	 * Otherwise ECC calculation has not even begun and the next loop might
	 * fail because of a false positive!
	 */
	timeo = jiffies + usecs_to_jiffies(100);
	do {
		val = puma_emif_readl(info, NANDFSR_OFFSET);
		val &= 0xc00;
		cpu_relax();
	} while (time_before(jiffies, timeo) && !val);

	/*
	 * Wait for the corr_state field (bits 8 to 11) in the
	 * NAND Flash Status register to be equal to 0x0, 0x1, 0x2, or 0x3.
	 */
	timeo = jiffies + usecs_to_jiffies(100);
	do {
		val = puma_emif_readl(info, NANDFSR_OFFSET);
		val &= 0xc00;
		cpu_relax();
	} while (time_before(jiffies, timeo) && val);

	iserror = puma_emif_readl(info, NANDFSR_OFFSET);
	iserror &= NAND_FSR_ECC_STATE_MASK;
	iserror = iserror >> 8;

	/*
	 * ECC_STATE_TOO_MANY_ERRS (0x1) means errors cannot be
	 * corrected (five or more errors).  The number of errors
	 * calculated (err_num field) differs from the number of errors
	 * searched.  ECC_STATE_ERR_CORR_COMP_P (0x2) means error
	 * correction complete (errors on bit 8 or 9).
	 * ECC_STATE_ERR_CORR_COMP_N (0x3) means error correction
	 * complete (error exists).
	 */

	if (iserror == ECC_STATE_NO_ERR) {
		val = puma_emif_readl(info, NAND_ERR_ERRVAL1_OFFSET);
		return 0;
	} else if (iserror == ECC_STATE_TOO_MANY_ERRS) {
		val = puma_emif_readl(info, NAND_ERR_ERRVAL1_OFFSET);

		DEBUG(KERN_WARNING "NAND : TO MANY BIT ECC ERRORS FOUND, ABORTING\n");
		return -EIO;
	}

	numerrors = ((puma_emif_readl(info, NANDFSR_OFFSET) >> 16) & 0x3) + 1;

	/* Read the error address, error value and correct */
	for (i = 0; i < numerrors; i++) {
		if (i > 1) {
			erroraddress =
			((puma_emif_readl(info, NAND_ERR_ADD2_OFFSET) >>
			(16 * (i & 1))) & 0x3FF);
			erroraddress = ((512 + 7) - erroraddress);
			errorvalue =
			((puma_emif_readl(info, NAND_ERR_ERRVAL2_OFFSET) >>
			(16 * (i & 1))) & 0xFF);
		} else {
			erroraddress =
			((puma_emif_readl(info, NAND_ERR_ADD1_OFFSET) >>
			(16 * (i & 1))) & 0x3FF);
			erroraddress = ((512 + 7) - erroraddress);
			errorvalue =
			((puma_emif_readl(info, NAND_ERR_ERRVAL1_OFFSET) >>
			(16 * (i & 1))) & 0xFF);
		}
		/* xor the corrupt data with error value */
		if (erroraddress < 512)
			data[erroraddress] ^= errorvalue;

		DEBUG(KERN_INFO "NAND : CORRECTING ECC BIT ERROR AT ADDRESS: %d\n", erroraddress);
	}

	return numerrors;
}


static void nand_puma_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if ((0x03 & ((unsigned)buf)) == 0 && (0x03 & len) == 0)
		ioread32_rep(chip->IO_ADDR_R, buf, len >> 2);
	else if ((0x01 & ((unsigned)buf)) == 0 && (0x01 & len) == 0)
		ioread16_rep(chip->IO_ADDR_R, buf, len >> 1);
	else
		ioread8_rep(chip->IO_ADDR_R, buf, len);
}

static void nand_puma_write_buf(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd_to_nand(mtd);

	if ((0x03 & ((unsigned)buf)) == 0 && (0x03 & len) == 0)
		iowrite32_rep(chip->IO_ADDR_R, buf, len >> 2);
	else if ((0x01 & ((unsigned)buf)) == 0 && (0x01 & len) == 0)
		iowrite16_rep(chip->IO_ADDR_R, buf, len >> 1);
	else
		iowrite8_rep(chip->IO_ADDR_R, buf, len);
}


/* An ECC layout for using 4-bit ECC with small-page flash, storing
 * ten ECC bytes plus the manufacturer's bad block marker byte, and
 * and not overlapping the default BBT markers.
 */
static int hwecc4_ooblayout_small_ecc(struct mtd_info *mtd, int section,
				      struct mtd_oob_region *oobregion)
{
	if (section > 2)
		return -ERANGE;

	if (!section) {
		oobregion->offset = 0;
		oobregion->length = 5;
	} else if (section == 1) {
		oobregion->offset = 6;
		oobregion->length = 2;
	} else {
		oobregion->offset = 13;
		oobregion->length = 3;
	}

	return 0;
}

static int hwecc4_ooblayout_small_free(struct mtd_info *mtd, int section,
				       struct mtd_oob_region *oobregion)
{
	if (section > 1)
		return -ERANGE;

	if (!section) {
		oobregion->offset = 8;
		oobregion->length = 5;
	} else {
		oobregion->offset = 16;
		oobregion->length = mtd->oobsize - 16;
	}

	return 0;
}

static const struct mtd_ooblayout_ops hwecc4_small_ooblayout_ops = {
	.ecc = hwecc4_ooblayout_small_ecc,
	.free = hwecc4_ooblayout_small_free,
};


static int nand_puma_waitfunc(struct mtd_info *mtd, struct nand_chip *this)
{
	struct puma_nand_info *info = to_puma_nand(mtd);
	uint8_t nand_status = 0;

	/* Issue a "READ STATUS" command */
	puma_nand_writeb(info, CLE_TRIGGER_ADDR, NAND_CMD_STATUS);

	/* Since the NAND R/B# signal is not connected and it's not possible
	 * to check R/B# status reading it in STATUS REGISTER because this
	 * operation will require issuing a READ STATUS command (0x70)
	 * breaking the command sequence of the ongoing operation, we will wait
	 * for the ready in the waitfunc, i.e. at the end of the operation.
	 * NOTE: waiting for the ready is crucial in operations that touch data
	 * inside NAND block because,
	 *       these operations MUST be ended before proceeding with further
	 *	 operations and, moreover, they require a not predictable
	 *	 amount of time so it's not possible to use a
	 *	 predefined timeout.
	 */

	/* Check NAND STATUS Register RDY field (Bit 6: 0 = Busy, 1 = Ready; NAND R/B# signal follows bit 6) */
	while ((nand_status & NAND_STATUS_READY) != NAND_STATUS_READY) {
		nand_status = puma_nand_readb(info, 0x0);
	}

	return nand_status;
}

#ifdef CONFIG_OF
static const struct of_device_id puma_nand_of_match[] = {
	{.compatible = "ericsson,puma-nand", },
	{ /* End of List */ },
};
MODULE_DEVICE_TABLE(of, puma_nand_of_match);

static struct puma_nand_pdata
	*nand_puma_get_pdata(struct platform_device *pdev)
{
	if (!pdev->dev.platform_data && pdev->dev.of_node) {
		struct puma_nand_pdata *pdata;
		const char *mode;
		u32 prop;
		int len;

		pdata =  devm_kzalloc(&pdev->dev,
					sizeof(struct puma_nand_pdata),
							GFP_KERNEL);
		pdev->dev.platform_data = pdata;
		if (!pdata)
			return NULL;
		if (!of_property_read_u32(pdev->dev.of_node,
				"puma1,nand-chipselect", &prop))
			pdev->id = prop;
		if (!of_property_read_u32(pdev->dev.of_node,
				"puma1,nand-mask-ale", &prop))
			pdata->mask_ale = prop;
		if (!of_property_read_u32(pdev->dev.of_node,
				"puma1,nand-mask-cle", &prop))
			pdata->mask_cle = prop;
		if (!of_property_read_u32(pdev->dev.of_node,
				"puma1,nand-mask-chipsel", &prop))
			pdata->mask_chipsel = prop;
		if (!of_property_read_string(pdev->dev.of_node,
				"puma1,nand-ecc-mode", &mode)) {
			if (!strncmp("none", mode, 4))
				pdata->ecc_mode = NAND_ECC_NONE;
			if (!strncmp("soft", mode, 4))
				pdata->ecc_mode = NAND_ECC_SOFT;
			if (!strncmp("hw", mode, 2))
				pdata->ecc_mode = NAND_ECC_HW;
			if (!strncmp("hw_syndrome", mode, 11))
				pdata->ecc_mode = NAND_ECC_HW_SYNDROME;
			if (!strncmp("hw_oob_first", mode, 12))
				pdata->ecc_mode = NAND_ECC_HW_OOB_FIRST;
			if (!strncmp("soft_bch", mode, 8))
				pdata->ecc_mode = NAND_ECC_BCH;
		}
		if (!of_property_read_u32(pdev->dev.of_node,
			"puma1,nand-ecc-bits", &prop))
			pdata->ecc_bits = prop;
		if (of_find_property(pdev->dev.of_node,
			"puma1,nand-use-bbt", &len))
			pdata->bbt_options = NAND_BBT_USE_FLASH;

		pdata->options |= NAND_BUSWIDTH_16;
	}

	return pdev->dev.platform_data;
}
#else
static struct puma_nand_pdata
	*nand_puma_get_pdata(struct platform_device *pdev)
{
	return pdev->dev.platform_data;
}
#endif

static int __init nand_puma_probe(struct platform_device *pdev)
{
	struct puma_nand_pdata *pdata;
	struct puma_nand_info  *info;
	struct resource        *res1;
	struct resource        *res2;
	void __iomem           *emif_base;
	void __iomem           *nand_base;
	int                    ret;
	nand_ecc_modes_t       ecc_mode;
	struct mtd_info			*mtd;
	struct device_node __maybe_unused *np = pdev->dev.of_node;
	const char *mtd_name = NULL;

	pdata = nand_puma_get_pdata(pdev);
	/* insist on board-specific configuration */
	if (!pdata)
		return -ENODEV;

	/* which external chipselect will we be managing? */
	if (pdev->id < 0 || pdev->id > 3)
		return -ENODEV;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "unable to allocate memory\n");
		ret = -ENOMEM;
		goto err_nomem;
	}
	platform_set_drvdata(pdev, info);

	res1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	res2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res1 || !res2) {
		dev_err(&pdev->dev, "resource missing\n");
		ret = -EINVAL;
		goto err_nomem;
	}

	emif_base = ioremap(res1->start, resource_size(res1));
	nand_base = ioremap(res2->start, resource_size(res2));
	if (!emif_base || !nand_base) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -EINVAL;
		goto err_ioremap;
	}

	info->dev               = &pdev->dev;
	info->emif_base         = emif_base;
	info->nand_base         = nand_base;

#ifdef CONFIG_OF
	of_property_read_string(np, "linux,mtd-name", &mtd_name);
#endif

	mtd			= nand_to_mtd(&info->chip);
	mtd->name		= mtd_name ? : dev_name(&pdev->dev);
	mtd->dev.parent		= &pdev->dev;
	nand_set_flash_node(&info->chip, pdev->dev.of_node);

	info->chip.IO_ADDR_R    = nand_base;
	info->chip.IO_ADDR_W    = nand_base;
	info->chip.chip_delay   = 50;
	info->chip.select_chip  = nand_puma_select_chip;

	/* options such as NAND_BBT_USE_FLASH */
	info->chip.bbt_options  = pdata->bbt_options;
	/* options such as 16-bit widths */
	info->chip.options      = pdata->options;
	info->chip.bbt_td       = pdata->bbt_td;
	info->chip.bbt_md       = pdata->bbt_md;

	info->ioaddr            = (uint32_t __force) nand_base;

	info->current_cs        = info->ioaddr;
	info->core_chipsel      = pdev->id;
	info->mask_chipsel      = pdata->mask_chipsel;

	info->mask_ale          = pdata->mask_ale;
	info->mask_cle          = pdata->mask_cle;

	/* Set address of hardware control function */
	info->chip.cmd_ctrl     = nand_puma_hwcontrol;
	info->chip.waitfunc     = nand_puma_waitfunc;

	/* Speed up buffer I/O */
	info->chip.read_buf     = nand_puma_read_buf;
	info->chip.write_buf    = nand_puma_write_buf;

	/* Use board-specific ECC config */
	ecc_mode                = pdata->ecc_mode;

	ret = -EINVAL;

	/* NOTE on ECC layout
	 * For 1-bit HW ECC, the default is OK, but it allocates 6 bytes
	 * when only 3 are needed (for each 512 bytes).
	 */
	switch (ecc_mode) {
	case NAND_ECC_NONE:
	case NAND_ECC_SOFT:
		pdata->ecc_bits = 0;
		dev_info(&pdev->dev, "NAND ECC : WARNING NO HW ECC ENABLED\n");
		break;

	case NAND_ECC_HW:
		dev_info(&pdev->dev, "NAND ECC : 1-BIT HW ECC ENABLED\n");
		pdata->ecc_bits = 1;
		info->chip.ecc.calculate = nand_puma_calculate_1bit;
		info->chip.ecc.correct = nand_puma_correct_1bit;
		info->chip.ecc.hwctl = nand_puma_hwctl_1bit;
		info->chip.ecc.bytes = 3;
		info->chip.ecc.size = 512;
		break;

	case NAND_ECC_HW_OOB_FIRST:
		dev_info(&pdev->dev, "NAND ECC : 4-BIT HW ECC ENABLED\n");
		pdata->ecc_bits = 4;
		info->chip.ecc.calculate = nand_puma_calculate_4bit;
		info->chip.ecc.correct = nand_puma_correct_4bit;
		info->chip.ecc.hwctl = nand_puma_hwctl_4bit;
		info->chip.ecc.bytes = 10;
		info->chip.ecc.size = 512;
		info->chip.ecc.strength = 4;
		info->chip.options |= NAND_NO_SUBPAGE_WRITE;
		break;

	default:
		ret = -EINVAL;
		goto err_ecc;
	}
	info->chip.ecc.mode = ecc_mode;
	/* Scan to find existence of the device(s) */
	ret = nand_scan_ident(mtd, 1, NULL);
	if (ret < 0) {
		dev_dbg(&pdev->dev, "no NAND chip(s) found\n");
		goto err_scan;
	}
	/* Update ECC layout if needed ... for 1-bit HW ECC, the default
	 * is OK, but it allocates 6 bytes when only 3 are needed (for
	 * each 512 bytes).  For the 4-bit HW ECC, that default is not
	 * usable:  10 bytes are needed, not 6.
	 */
	if (pdata->ecc_bits == 4) {
		int chunks = mtd->writesize / 512;

		if (!chunks || mtd->oobsize < 16) {
			dev_dbg(&pdev->dev, "too small\n");
			ret = -EINVAL;
			goto err_scan;
		}
		/* For small page chips, preserve the manufacturer's
		 * badblock marking data ... and make sure a flash BBT
		 * table marker fits in the free bytes.
		 */
		if (chunks == 1) {
			mtd_set_ooblayout(mtd, &hwecc4_small_ooblayout_ops);
		} else if (chunks == 4 || chunks == 8) {
			mtd_set_ooblayout(mtd, &nand_ooblayout_lp_ops);
			info->chip.ecc.mode = NAND_ECC_HW_OOB_FIRST;
		} else {
			ret = -EIO;
			goto err_scan;
		}
	}

	ret = nand_scan_tail(mtd);
	if (ret < 0)
		goto err_scan;
	ret = mtd_device_register(mtd, NULL, 0);
	if (ret < 0)
		goto err_scan;

	return 0;

err_scan:
err_ecc:
err_ioremap:
	if (emif_base)
		iounmap(emif_base);
	if (nand_base)
		iounmap(nand_base);

err_nomem:
	kfree(info);
	return ret;
}

static int __exit nand_puma_remove(struct platform_device *pdev)
{
	struct puma_nand_info *info = platform_get_drvdata(pdev);

	iounmap(info->emif_base);
	iounmap(info->nand_base);

	nand_release(nand_to_mtd(&info->chip));

	kfree(info);

	return 0;
}

static struct platform_driver nand_puma_driver = {
	.remove = __exit_p(nand_puma_remove),
	.driver = {
		.name   = "puma_nand",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(puma_nand_of_match),
#endif
	},
};
MODULE_ALIAS("platform:puma_nand");

module_platform_driver_probe(nand_puma_driver, nand_puma_probe);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("TI/Ericsson");
MODULE_DESCRIPTION("PUMA SoC NAND flash driver");
