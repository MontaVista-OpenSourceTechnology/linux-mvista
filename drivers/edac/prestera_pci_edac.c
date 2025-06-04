// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Marvell Prestera new generation of integrated CPUs PCIe EDAC driver.
 * Supports AC5, AC5X and IronMan.
 *
 * Copyright (c) 2023, Marvell Technology, Inc.
 * Author: Elad Nachman <enachman@marvell.com>
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/edac.h>
#include <linux/of.h>

#include "edac_module.h"
#include "dbgfs_edac_macros.h"

#define PRESTERA_PCI_STAT_CMD_REG		0x4
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG 	0x104
#define PRESTERA_PCI_UNCORR_ERR_SEVERITY_REG	0x10C
#define PRESTERA_PCI_CORR_ERR_STAT_REG 	0x110
#define PRESTERA_PCI_ROOT_ERR_STAT_REG	0x130
#define PRESTERA_PCI_ERR_SRC_ID_REG		0x134

#define PRESTERA_PCI_ERR_HDR_LOG_REG		0x11C
#define PRESTERA_PCI_TLP_PREFIX_LOG_REG	0x138
#define PRESTERA_PCI_NUM_HDR_REGS		4

/* error injection control registers: */
#define PRESTERA_PCI_ERR_INJ_ENABLE_REG	0x188
#define PRESTERA_PCI_ERR_INJ_CTRL_CRC_REG	0x18C
#define PRESTERA_PCI_ERR_INJ_CTRL_SEQ_REG	0x190
#define PRESTERA_PCI_ERR_INJ_CTRL_DLLP_REG	0x194
#define PRESTERA_PCI_ERR_INJ_CTRL_SYMBOL_REG	0x198
#define PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG	0x19C
#define PRESTERA_PCI_ERR_INJ_CTRL_TLP_REG	0x1A0

/* All following 4 registers groups have 4 registers from base: */
#define PRESTERA_PCI_ERR_INJ_CTRL_COMP_PNT	0x1A4
#define PRESTERA_PCI_ERR_INJ_CTRL_COMP_VAL	0x1B4
#define PRESTERA_PCI_ERR_INJ_CTRL_CHG_PNT	0x1C4
#define PRESTERA_PCI_ERR_INJ_CTRL_CHG_VAL	0x1D4

#define PRESTERA_PCI_ERR_INJ_CTRL_PKT_ERR	0x1E4

/* register bits shifts/masks (post shift) definitions: */
#define PRESTERA_PCI_STAT_CMD_REG_PAR_ERR BIT(31)
#define PRESTERA_PCI_STAT_CMD_REG_MAS_DPE BIT(24)

#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_INTRNL 		BIT(22)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_UNSUP_REQ 		BIT(20)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_ECRC_ERR		BIT(19)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_MALF_TLP_ERR 	BIT(18)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_RX_OVERFLOW_ERR 	BIT(17)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_UNEXPECT_COMPL_ERR 	BIT(16)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_COMPL_ABORT_ERR 	BIT(15)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_COMPL_TIMEOUT_ERR 	BIT(14)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_FC_PROT_ERR 	BIT(13)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_POISON_TLP_ERR 	BIT(12)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_SURPRISE_DOWN_ERR 	BIT(5)
#define PRESTERA_PCI_UNCORR_ERR_STAT_REG_DATA_LINK_PROT_ERR 	BIT(4)

#define PRESTERA_PCI_CORR_ERR_STAT_REG_HDR_LOG_OVFLW_ERR	BIT(15)
#define PRESTERA_PCI_CORR_ERR_STAT_REG_INTERNAL_ERR		BIT(14)
#define PRESTERA_PCI_CORR_ERR_STAT_REG_ADVISORY_NO_FATAL_ERR	BIT(13)
#define PRESTERA_PCI_CORR_ERR_STAT_REG_REPLAY_TIMEOUT_ERR	BIT(12)
#define PRESTERA_PCI_CORR_ERR_STAT_REG_REPLAY_NUM_ROLLOVER	BIT(8)
#define PRESTERA_PCI_CORR_ERR_STAT_REG_BAD_DLLP_STATUS	BIT(7)
#define PRESTERA_PCI_CORR_ERR_STAT_REG_BAD_TLP_STATUS	BIT(6)
#define PRESTERA_PCI_CORR_ERR_STAT_REG_RX_ERR_STATUS		BIT(0)

#define PRESTERA_PCI_ROOT_ERR_STAT_REG_FATAL_ERR_MSG		BIT(6)
#define PRESTERA_PCI_ROOT_ERR_STAT_REG_NON_FATAL_ERR_MSG	BIT(5)
#define PRESTERA_PCI_ROOT_ERR_STAT_REG_FIRST_UNCOR_FATAL	BIT(4)
#define PRESTERA_PCI_ROOT_ERR_STAT_REG_MULT_ERRS_BOTH	BIT(3)
#define PRESTERA_PCI_ROOT_ERR_STAT_REG_ERRS_BOTH_FATAL_NON	BIT(2)
#define PRESTERA_PCI_ROOT_ERR_STAT_REG_MULT_CORR_ERRS_RX	BIT(1)
#define PRESTERA_PCI_ROOT_ERR_STAT_REG_CORR_ERR_RECEIVED	BIT(0)

#define PRESTERA_PCI_ROOT_ERR_STAT_REG_UNCORR_MASK		GENMASK(6,2)
#define PRESTERA_PCI_ROOT_ERR_STAT_REG_CORR_MASK		GENMASK(1,0)

#define PRESTERA_PCI_ERR_SRC_ID_REG_UNCOR_BSHIFT		16
#define PRESTERA_PCI_ERR_SRC_ID_REG_REQ_MASK			0xFFFF

#define PRESTERA_PCI_ERR_INJ_COUNT_MASK			0xFF
#define PRESTERA_PCI_ERR_INJ_CTRL_SEQ_REG_BAD_SEQNUM_BSHIFT	16
#define PRESTERA_PCI_ERR_INJ_CTRL_SEQ_REG_BAD_SEQNUM_MASK	0x1FFF
#define PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG_BAD_VAL_BSHIFT	16
#define PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG_BAD_VAL_MASK	0x1FFF
#define PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG_VC_NUM_BSHIFT	12
#define PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG_VC_NUM_MASK	0x7

struct prestera_dbgfs_entry {
	const struct file_operations *fops;
	const char *name;
};

struct prestera_edac_pci_pvt {
	void __iomem *base;
	struct dentry *debugfs_dir;
	unsigned err_inj_update;
	unsigned err_inj_en_bitmap;
	unsigned err_inj_crc_err_count;
	unsigned err_inj_bad_seq;
	unsigned err_inj_bad_seq_err_cnt;
	unsigned err_inj_dllp_err_cnt;
	unsigned err_inj_symbol_err_cnt;
	unsigned err_inj_credit_bad_val;
	unsigned err_inj_credit_vc_num;
	unsigned err_inj_credit_err_cnt;
	unsigned err_inj_tlp_err_cnt;
	unsigned err_inj_custom_err_cnt;
	unsigned err_inj_comp_mask[PRESTERA_PCI_NUM_HDR_REGS];
	unsigned err_inj_comp_values[PRESTERA_PCI_NUM_HDR_REGS];
	unsigned err_inj_change_mask[PRESTERA_PCI_NUM_HDR_REGS];
	unsigned err_inj_change_values[PRESTERA_PCI_NUM_HDR_REGS];
};

struct prestera_edac_error_strings {
	u32 code;
	char *str;
};

struct prestera_edac_error_strings prestera_pci_edac_error_strings_uncor_ar[] = {
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_INTRNL, "Uncorrectable Internal Error Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_UNSUP_REQ, "Unsupported Request Error Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_ECRC_ERR, "ECRC Error Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_MALF_TLP_ERR, "Malformed TLP Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_RX_OVERFLOW_ERR, "Receiver Overflow Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_UNEXPECT_COMPL_ERR, "Unexpected Completion Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_COMPL_ABORT_ERR, "Completer Abort Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_COMPL_TIMEOUT_ERR, "Completion Timeout Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_FC_PROT_ERR, "Flow Control Protocol Error Status."},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_POISON_TLP_ERR, "Poisoned TLP Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_SURPRISE_DOWN_ERR, "Surprise Down Error Status"},
	{PRESTERA_PCI_UNCORR_ERR_STAT_REG_DATA_LINK_PROT_ERR, "Data Link Protocol Error Status"},
	{0xff, NULL}
};

struct prestera_edac_error_strings prestera_pci_edac_error_strings_corr_ar[] = {
	{PRESTERA_PCI_CORR_ERR_STAT_REG_HDR_LOG_OVFLW_ERR, "Header Log Overflow Error Status"},
	{PRESTERA_PCI_CORR_ERR_STAT_REG_INTERNAL_ERR, "Corrected Internal Error Status"},
	{PRESTERA_PCI_CORR_ERR_STAT_REG_ADVISORY_NO_FATAL_ERR, "Advisory Non-Fatal Error Status"},
	{PRESTERA_PCI_CORR_ERR_STAT_REG_REPLAY_TIMEOUT_ERR, "Replay Timer Timeout Status"},
	{PRESTERA_PCI_CORR_ERR_STAT_REG_REPLAY_NUM_ROLLOVER, "REPLAY_NUM Rollover Status"},
	{PRESTERA_PCI_CORR_ERR_STAT_REG_BAD_DLLP_STATUS, "Bad DLLP Status"},
	{PRESTERA_PCI_CORR_ERR_STAT_REG_BAD_TLP_STATUS, "Bad TLP Status"},
	{PRESTERA_PCI_CORR_ERR_STAT_REG_RX_ERR_STATUS, "Receiver Error Status"},
	{0xff, NULL}
};

static const char *prestera_pci_edac_get_errmsg(u8 code, bool correctable)
{
	u8 i = 0;
	struct prestera_edac_error_strings *ptr;

	ptr = correctable ? prestera_pci_edac_error_strings_corr_ar :
			    prestera_pci_edac_error_strings_uncor_ar;

	while (ptr[i].str) {
		if (ptr[i].code == code)
			return ptr[i].str;
		i++;
	}

	return NULL;
}


static void prestera_pci_poll(struct edac_pci_ctl_info *pci)
{
	struct prestera_edac_pci_pvt *drvdata;
	u32 i, val, severity, masked_val, req_id, err_cnt = 0;
	char msg[128];
	const char *errmsg;

	drvdata = pci->pvt_info;

	val = readl(drvdata->base + PRESTERA_PCI_STAT_CMD_REG);
	masked_val = val & (PRESTERA_PCI_STAT_CMD_REG_PAR_ERR |
			    PRESTERA_PCI_STAT_CMD_REG_MAS_DPE);
	if (masked_val) {
		snprintf(msg, sizeof(msg), "%sPCIe parity error detected",
			 val & PRESTERA_PCI_STAT_CMD_REG_MAS_DPE ? "Master " : "");
		edac_pci_handle_pe(pci, msg);
		/* write 1 to error bits to clear error: */
		writel(masked_val, drvdata->base + PRESTERA_PCI_STAT_CMD_REG);
	}

	val = readl(drvdata->base + PRESTERA_PCI_UNCORR_ERR_STAT_REG);
	severity = readl(drvdata->base + PRESTERA_PCI_UNCORR_ERR_SEVERITY_REG);

	for (i = 0; i < 32; i++) {
		errmsg = prestera_pci_edac_get_errmsg(val & BIT(i), false);
		if (errmsg) {
			snprintf(msg, sizeof(msg), "%s (%s)",
				 errmsg, severity & BIT(i) ?
				 "Fatal" : "Non-Fatal");
			edac_pci_handle_npe(pci, msg);
			err_cnt++;
		}
	}

	/* write 1 to clear error status bits: */
	writel(val, drvdata->base + PRESTERA_PCI_UNCORR_ERR_STAT_REG);

	val = readl(drvdata->base + PRESTERA_PCI_CORR_ERR_STAT_REG);

	for (i = 0; i < 32; i++) {
		errmsg = prestera_pci_edac_get_errmsg(val & BIT(i), true);
		if (errmsg) {
			edac_pci_handle_npe(pci, errmsg);
			err_cnt++;
		}
	}

	/* write 1 to clear error status bits: */
	writel(val, drvdata->base + PRESTERA_PCI_CORR_ERR_STAT_REG);

	val = readl(drvdata->base + PRESTERA_PCI_ROOT_ERR_STAT_REG);
	req_id = readl(drvdata->base + PRESTERA_PCI_ERR_SRC_ID_REG);

	if (val & PRESTERA_PCI_ROOT_ERR_STAT_REG_UNCORR_MASK) {
		snprintf(msg, sizeof(msg),
			 "Uncorrectable, %s %s requester id %x",
			 val & PRESTERA_PCI_ROOT_ERR_STAT_REG_FATAL_ERR_MSG ?
			 "Fatal," : "",
			 val & PRESTERA_PCI_ROOT_ERR_STAT_REG_MULT_ERRS_BOTH ?
			 "Multiple Errors" : "",
			 req_id >> PRESTERA_PCI_ERR_SRC_ID_REG_UNCOR_BSHIFT);
		edac_pci_handle_npe(pci, msg);
		err_cnt++;
	}

	if (val & PRESTERA_PCI_ROOT_ERR_STAT_REG_CORR_MASK) {
		snprintf(msg, sizeof(msg),
			 "Correctable, %s requester id %x",
			 val & PRESTERA_PCI_ROOT_ERR_STAT_REG_MULT_CORR_ERRS_RX ?
			 "Multiple Errors" : "",
			 req_id & PRESTERA_PCI_ERR_SRC_ID_REG_REQ_MASK);
		edac_pci_handle_npe(pci, msg);
		err_cnt++;
	}

	/* write 1 to clear error status bits: */
	writel(val, drvdata->base + PRESTERA_PCI_ROOT_ERR_STAT_REG);

	if (err_cnt > 0) {
		snprintf(msg, sizeof(msg),
			 "TLP header: %x %x %x %x prefix: %x %x %x %x",
			 readl(drvdata->base + PRESTERA_PCI_ERR_HDR_LOG_REG),
			 readl(drvdata->base + PRESTERA_PCI_ERR_HDR_LOG_REG + 0x4),
			 readl(drvdata->base + PRESTERA_PCI_ERR_HDR_LOG_REG + 0x8),
			 readl(drvdata->base + PRESTERA_PCI_ERR_HDR_LOG_REG + 0xC),
			 readl(drvdata->base + PRESTERA_PCI_TLP_PREFIX_LOG_REG),
			 readl(drvdata->base + PRESTERA_PCI_TLP_PREFIX_LOG_REG + 0x4),
			 readl(drvdata->base + PRESTERA_PCI_TLP_PREFIX_LOG_REG + 0x8),
			 readl(drvdata->base + PRESTERA_PCI_TLP_PREFIX_LOG_REG + 0xC));
			 edac_pci_handle_npe(pci, msg);
	}

	/* if update requested, update the PCIe error injection registers: */
	if (drvdata->err_inj_update) {
		pr_info("%s: Injecting error...\n", __func__);
		drvdata->err_inj_update = 0;

		/*
		 * Disable all injection error types (bitmap 0..6)
		 * before updating any other registers:
		 */
		writel(0,
		       drvdata->base + PRESTERA_PCI_ERR_INJ_ENABLE_REG);

		/* Now update all error types 0..6 configuration registers: */
		writel(drvdata->err_inj_crc_err_count &
		       PRESTERA_PCI_ERR_INJ_COUNT_MASK,
		       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_CRC_REG);

		writel(((drvdata->err_inj_bad_seq &
		        PRESTERA_PCI_ERR_INJ_CTRL_SEQ_REG_BAD_SEQNUM_MASK) <<
		        PRESTERA_PCI_ERR_INJ_CTRL_SEQ_REG_BAD_SEQNUM_BSHIFT) |
		        (drvdata->err_inj_bad_seq_err_cnt &
		         PRESTERA_PCI_ERR_INJ_COUNT_MASK),
		       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_SEQ_REG);

		writel(drvdata->err_inj_dllp_err_cnt &
		       PRESTERA_PCI_ERR_INJ_COUNT_MASK,
		       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_DLLP_REG);

		writel(drvdata->err_inj_symbol_err_cnt &
		       PRESTERA_PCI_ERR_INJ_COUNT_MASK,
		       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_SYMBOL_REG);

		writel(((drvdata->err_inj_credit_bad_val &
		        PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG_BAD_VAL_MASK) <<
		        PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG_BAD_VAL_BSHIFT) |
		        ((drvdata->err_inj_credit_vc_num &
		         PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG_VC_NUM_MASK) <<
		         PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG_VC_NUM_BSHIFT) |
		        (drvdata->err_inj_credit_err_cnt &
		         PRESTERA_PCI_ERR_INJ_COUNT_MASK),
		       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_CREDIT_REG);

		writel(drvdata->err_inj_tlp_err_cnt &
		       PRESTERA_PCI_ERR_INJ_COUNT_MASK,
		       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_TLP_REG);

		for (i = 0; i < PRESTERA_PCI_NUM_HDR_REGS; i++) {
			writel(drvdata->err_inj_comp_mask[i],
			       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_COMP_PNT + (i * sizeof(int)));

			writel(drvdata->err_inj_comp_values[i],
			       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_COMP_VAL + (i * sizeof(int)));

			writel(drvdata->err_inj_change_mask[i],
			       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_CHG_PNT + (i * sizeof(int)));

			writel(drvdata->err_inj_change_values[i],
			       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_CHG_VAL + (i * sizeof(int)));

		}
		writel(drvdata->err_inj_custom_err_cnt &
		       PRESTERA_PCI_ERR_INJ_COUNT_MASK,
		       drvdata->base + PRESTERA_PCI_ERR_INJ_CTRL_PKT_ERR);

		/* this is bitmap 0..6 per injection error type to enable: */
		writel(drvdata->err_inj_en_bitmap,
		       drvdata->base + PRESTERA_PCI_ERR_INJ_ENABLE_REG);

	}
}

PCI_DBGFS_TEMPLATE_WRITE(err_inj_update)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_en_bitmap)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_crc_err_count)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_bad_seq)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_bad_seq_err_cnt)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_dllp_err_cnt)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_symbol_err_cnt)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_credit_bad_val)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_credit_vc_num)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_credit_err_cnt)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_tlp_err_cnt)
PCI_DBGFS_TEMPLATE_WRITE(err_inj_custom_err_cnt)

PCI_DBGFS_TEMPLATE_READ(err_inj_update)
PCI_DBGFS_TEMPLATE_READ(err_inj_en_bitmap)
PCI_DBGFS_TEMPLATE_READ(err_inj_crc_err_count)
PCI_DBGFS_TEMPLATE_READ(err_inj_bad_seq)
PCI_DBGFS_TEMPLATE_READ(err_inj_bad_seq_err_cnt)
PCI_DBGFS_TEMPLATE_READ(err_inj_dllp_err_cnt)
PCI_DBGFS_TEMPLATE_READ(err_inj_symbol_err_cnt)
PCI_DBGFS_TEMPLATE_READ(err_inj_credit_bad_val)
PCI_DBGFS_TEMPLATE_READ(err_inj_credit_vc_num)
PCI_DBGFS_TEMPLATE_READ(err_inj_credit_err_cnt)
PCI_DBGFS_TEMPLATE_READ(err_inj_tlp_err_cnt)
PCI_DBGFS_TEMPLATE_READ(err_inj_custom_err_cnt)

PCI_DBGFS_TEMPLATE_WRITE4(err_inj_comp_mask)
PCI_DBGFS_TEMPLATE_WRITE4(err_inj_comp_values)
PCI_DBGFS_TEMPLATE_WRITE4(err_inj_change_mask)
PCI_DBGFS_TEMPLATE_WRITE4(err_inj_change_values)

PCI_DBGFS_TEMPLATE_READ4(err_inj_comp_mask)
PCI_DBGFS_TEMPLATE_READ4(err_inj_comp_values)
PCI_DBGFS_TEMPLATE_READ4(err_inj_change_mask)
PCI_DBGFS_TEMPLATE_READ4(err_inj_change_values)

/* file operations structures per injection variable: */
PCI_TEMPLATE_FOPS(err_inj_update)
PCI_TEMPLATE_FOPS(err_inj_en_bitmap)
PCI_TEMPLATE_FOPS(err_inj_crc_err_count)
PCI_TEMPLATE_FOPS(err_inj_bad_seq)
PCI_TEMPLATE_FOPS(err_inj_bad_seq_err_cnt)
PCI_TEMPLATE_FOPS(err_inj_dllp_err_cnt)
PCI_TEMPLATE_FOPS(err_inj_symbol_err_cnt)
PCI_TEMPLATE_FOPS(err_inj_credit_bad_val)
PCI_TEMPLATE_FOPS(err_inj_credit_vc_num)
PCI_TEMPLATE_FOPS(err_inj_credit_err_cnt)
PCI_TEMPLATE_FOPS(err_inj_tlp_err_cnt)
PCI_TEMPLATE_FOPS(err_inj_custom_err_cnt)
PCI_TEMPLATE_FOPS(err_inj_comp_mask)
PCI_TEMPLATE_FOPS(err_inj_comp_values)
PCI_TEMPLATE_FOPS(err_inj_change_mask)
PCI_TEMPLATE_FOPS(err_inj_change_values)

static const struct prestera_dbgfs_entry prestera_pci_dev_fops[] = {
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_update),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_en_bitmap),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_crc_err_count),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_bad_seq),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_bad_seq_err_cnt),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_dllp_err_cnt),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_symbol_err_cnt),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_credit_bad_val),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_credit_vc_num),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_credit_err_cnt),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_tlp_err_cnt),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_custom_err_cnt),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_comp_mask),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_comp_values),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_change_mask),
	PCI_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_change_values),
	{ NULL, NULL }
};

static void prestera_edac_pci_create_dbgfs(struct edac_pci_ctl_info *pci,
				      struct platform_device *pdev)
{
	int i = 0;
	struct prestera_edac_pci_pvt *drvdata = pci->pvt_info;

	if (!IS_ENABLED(CONFIG_EDAC_DEBUG))
		return;

	drvdata->debugfs_dir = edac_debugfs_create_dir(pdev->dev.kobj.name);
	if (!drvdata->debugfs_dir)
		return;

	while (prestera_pci_dev_fops[i].name) {
		if (!edac_debugfs_create_file(prestera_pci_dev_fops[i].name, S_IWUSR,
					      drvdata->debugfs_dir, pci,
					      prestera_pci_dev_fops[i].fops)) {
			pr_info("failed to added debugfs file %s\n", prestera_pci_dev_fops[i].name);
			debugfs_remove_recursive(drvdata->debugfs_dir);
		}
		i++;
	}
}

static int prestera_edac_pci_probe(struct platform_device *pdev)
{
	struct resource *r;
	void __iomem *base;
	struct prestera_edac_pci_pvt *drvdata;
	struct edac_pci_ctl_info *pci;
	int res = 0;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "Unable to get mem resource\n");
		return -ENODEV;
	}

	/*base = devm_ioremap_resource(&pdev->dev, r);*/
	base = devm_ioremap(&pdev->dev, r->start, resource_size(r));
	if (IS_ERR(base)) {
		dev_err(&pdev->dev, "Unable to map regs\n");
		return PTR_ERR(base);
	}

	pci = edac_pci_alloc_ctl_info(sizeof(struct prestera_edac_pci_pvt), "prestera_pci_err");
	if (!pci) {
		dev_err(&pdev->dev, "Unable to allocate memory!\n");
		return -ENOMEM;
	}

	drvdata = pci->pvt_info;
	drvdata->base = base;

	pci->dev = &pdev->dev;
	platform_set_drvdata(pdev, pci);
	pci->dev_name = dev_name(&pdev->dev);

	pci->mod_name = "prestera-pci";
	pci->ctl_name = "prestera_pci_err";
	pci->edac_check = prestera_pci_poll;

	prestera_edac_pci_create_dbgfs(pci, pdev);

	if (edac_pci_add_device(pci, 0) > 0) {
		pr_err("%s: edac_pci_add_device() failed\n", __func__);
		goto err;
	}

	return 0;

err:
	edac_pci_free_ctl_info(pci);

	return res;
}

static int prestera_edac_pci_remove(struct platform_device *pdev)
{
	struct edac_pci_ctl_info *pci = platform_get_drvdata(pdev);
	struct prestera_edac_pci_pvt *drvdata = pci->pvt_info;

	debugfs_remove_recursive(drvdata->debugfs_dir);
	drvdata = pci->pvt_info;
	edac_pci_del_device(&pdev->dev);
	edac_pci_free_ctl_info(pci);

	return 0;
}

static const struct of_device_id prestera_pci_of_match[] = {
	{.compatible = "marvell,prestera-edac-pci",},
	{},
};
MODULE_DEVICE_TABLE(of, prestera_pci_of_match);

static struct platform_driver prestera_edac_pci_driver = {
	.probe = prestera_edac_pci_probe,
	.remove = prestera_edac_pci_remove,
	.driver = {
		   .name = "prestera_edac_pci",
		   .of_match_table = of_match_ptr(prestera_pci_of_match)
	}
};
module_platform_driver(prestera_edac_pci_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elad Nachman <enachman@marvell.com>");
