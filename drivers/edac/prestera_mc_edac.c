// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Marvell Prestera AC5/AC5X/IronMan family of SoCs DDR controller EDAC
 * (error detection and correction)
 *
 * Copyright (c) 2023, Marvell Technology, Inc.
 * Author: Elad Nachman <enachman@marvell.com>
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/edac.h>
#include <linux/ctype.h>
#include <linux/of.h>
#include <linux/mm.h>

#include "edac_module.h"
#include "dbgfs_edac_macros.h"

#define PRESTERA_MAX_MC 1

/* register offsets: */
#define PRESTERA_DDR_MSTR 0x0
#define PRESTERA_DDR_MRSTAT 0x18
#define PRESTERA_DDR_MRCTRL2 0x1C
#define PRESTERA_DDR_ECCCFG0 0x70
#define PRESTERA_DDR_ECCCFG1 0x74
#define PRESTERA_DDR_ECCSTAT 0x78
#define PRESTERA_DDR_ECCCTL 0x7C
#define PRESTERA_DDR_ECCERRCNT 0x80
#define PRESTERA_DDR_ECCCADDR0 0x84
#define PRESTERA_DDR_ECCCADDR1 0x88
#define PRESTERA_DDR_ECCCSYN0  0x8c
#define PRESTERA_DDR_ECCCSYN1  0x90
#define PRESTERA_DDR_ECCCSYN2  0x94
#define PRESTERA_DDR_ECCBITMASK0 0x98
#define PRESTERA_DDR_ECCBITMASK1 0x9c
#define PRESTERA_DDR_ECCBITMASK2 0xa0
#define PRESTERA_DDR_ECCUADDR0   0xa4
#define PRESTERA_DDR_ECCUADDR1   0xa8
#define PRESTERA_DDR_ECCUSYN0    0xac
#define PRESTERA_DDR_ECCUSYN1    0xb0
#define PRESTERA_DDR_ECCUSYN2    0xb4
#define PRESTERA_DDR_ECCPOISONADDR0 0xb8
#define PRESTERA_DDR_ECCPOISONADDR1 0xbc
#define PRESTERA_DDR_SWCTL	       0x320
#define PRESTERA_DDR_SWSTAT	       0x324
#define PRESTERA_DDR_SWCTLSTATIC    0x328
#define PRESTERA_DDR_OCPARCFG0      0x330
#define PRESTERA_DDR_OCPARCFG1      0x334
#define PRESTERA_DDR_OCPARSTAT0     0x338
#define PRESTERA_DDR_OCPARSTAT1     0x33C
#define PRESTERA_DDR_OCPARSTAT2     0x340
#define PRESTERA_DDR_POISONCFG      0x36C
#define PRESTERA_DDR_POISONSTAT     0x370
#define PRESTERA_DDR_ADVECCINDEX    0x374
#define PRESTERA_DDR_ECCPOISONPAT0  0x37C

/* register bits shifts/masks (post shift) definitions: */
#define PRESTERA_DDR_MSTR_DCFG_BSHIFT 30
#define PRESTERA_DDR_MSTR_ACT_RANK_BSHIFT 24
#define PRESTERA_DDR_MSTR_ACT_RANK_MASK 0x3
#define PRESTERA_DDR_X4 0x0
#define PRESTERA_DDR_X8 0x1
#define PRESTERA_DDR_X16 0x2
#define PRESTERA_DDR_X32 0x3
#define PRESTERA_DDR_TYPE_DDR4 0x10
#define PRESTERA_DDR_ECCCFG0_ECC_TYPE_BSHIFT 5
#define PRESTERA_DDR_ECCCFG0_ECC_TYPE_MASK 0x1
#define PRESTERA_DDR_ECCCFG0_ECC_REGION_PARITY_LOCK_BSHIFT 4
#define PRESTERA_DDR_ECCCFG0_ECC_REGION_PARITY_LOCK_MASK 0x1
#define PRESTERA_DDR_ECCCFG0_ECC_MODE_BITS 0x7
#define PRESTERA_DDR_ECCCFG1_DATA_POISON_BIT_BSHIFT 1
#define PRESTERA_DDR_ECCCFG1_DATA_POISON_EN 0x1
#define PRESTERA_DDR_ECCCTL_CLR_CTRS_MASK 0xF
#define PRESTERA_DDR_ECCERRCNT_UC_BSHIFT 16
#define PRESTERA_DDR_ECCERRCNT_CE_MASK 0xFFFF
#define PRESTERA_DDR_ECCCADDR0_RANK_BSHIFT 24
#define PRESTERA_DDR_RANK_MASK 0x1
#define PRESTERA_DDR_ROW_MASK 0x3FFFF
#define PRESTERA_DDR_ECCCADDR1_BG_BSHIFT 24
#define PRESTERA_DDR_BG_MASK 0x3
#define PRESTERA_DDR_ECCCADDR1_BANK_BSHIFT 16
#define PRESTERA_DDR_BANK_MASK 0x7
#define PRESTERA_DDR_COL_MASK 0xFFF
#define PRESTERA_DDR_SWCTL_SWDONE 0x1
#define PRESTERA_DDR_SWCTLSTATIC_UNLOCK 0x1
#define PRESTERA_DDR_ECCPOISONADDR0_BG_BSHIFT 28
#define PRESTERA_DDR_ECCPOISONADDR0_BANK_BSHIFT 24

/* macros to start/stop writing registers defined as static or quasi dynamic: */
#define PRESTERA_DDR_STATIC_PROG_START(drvdata) \
	writel(PRESTERA_DDR_SWCTLSTATIC_UNLOCK, drvdata->base + PRESTERA_DDR_SWCTLSTATIC)

#define PRESTERA_DDR_STATIC_PROG_STOP(drvdata) \
	writel(0, drvdata->base + PRESTERA_DDR_SWCTLSTATIC)

#define PRESTERA_DDR_QUASI_PROG_START(drvdata) \
	writel(0x0, drvdata->base + PRESTERA_DDR_SWCTL)

#define PRESTERA_DDR_QUASI_PROG_STOP(drvdata) \
	writel(0x1, drvdata->base + PRESTERA_DDR_SWCTL); \
	while (!readl(drvdata->base + PRESTERA_DDR_SWSTAT))

#define to_mci(k) container_of(k, struct mem_ctl_info, dev)

struct prestera_lmc_pvt {
	void __iomem *base;
	/* error injection control: */
	bool inline_ecc;
	unsigned long inline_ecc_ofs;
	unsigned long data_update;
	unsigned long pattern;
	/* These fields are only for sideband ECC: */
	unsigned long error_type;
	unsigned long bankgroup;
	unsigned long rank;
	unsigned long bank;
	unsigned long row;
	unsigned long col;
};

static void prestera_lmc_edac_poll(struct mem_ctl_info *mci)
{
	struct prestera_lmc_pvt *drvdata = mci->pvt_info;
	bool do_clear = false;
	char msg[256], details[128];
	struct sysinfo si;
	phys_addr_t pa;
	unsigned long pfn;
	u8 *p8;
	bool require_unmap = false;

	u32 stat, errcnt, ce_addr[2], ce_syndrome[3], ue_syndrome[3],
	    bitmasks[3], ue_addr[2], axi_parity[3], pstat, ce_cnt,
	    ue_cnt, ecc_cfg1, val32;
	volatile u32 *p32;

	if (drvdata->data_update) {
		pr_info("Poisoning %s error Injection...\n",
			drvdata->inline_ecc ?
			"Inline ECC by Software" :
			"Sideband ECC by Hardware");
		drvdata->data_update = 0;
		if (drvdata->inline_ecc) {
			/* Inline ECC does not support HW error injection /
			 * poisoning. What we can do here is to unlock
			 * quasi programming, then unlock the register:
			 * ECCCFG1.ecc_region_parity_lock,
			 * then write to the parity bits (above physical
			 * memory detected by Linux) then lock again
			 * ECCCFG1.ecc_region_parity_lock and then
			 * read from actual memory, ECC error should
			 * be caught:
			 */
			si_meminfo(&si);
			/* Start of ECC parity = End of Linux accessible RAM.
			 * 64/8 SECDED is used, so divide physical address by
			 * 8 to get parity offset in ECC parity region:
			 */
			pa = memstart_addr + si.totalram * si.mem_unit;
			pfn = __phys_to_pfn(pa);
			/* go to the actual end of RAM
			 * (si.totalram excludes reserved memory)
			 */
			while (page_is_ram(pfn))
				pfn++;
			pa = __pfn_to_phys(pfn);
			pa += drvdata->inline_ecc_ofs;
			p8 = ioremap(pa, si.mem_unit);
			PRESTERA_DDR_QUASI_PROG_START(drvdata);
			/* unlock parity region for access: */
			ecc_cfg1 = readl(drvdata->base + PRESTERA_DDR_ECCCFG1);
			ecc_cfg1 &= ~(1 <<
				      PRESTERA_DDR_ECCCFG0_ECC_REGION_PARITY_LOCK_BSHIFT);
			writel(ecc_cfg1, drvdata->base + PRESTERA_DDR_ECCCFG1);
			/*page = phys_to_page(pa);
			p8 = page_address(page);*/
			/* Damage a single parity bit to create correctable ECC error: */
			*p8 ^= 0x1;
			/* lock parity region again: */
			ecc_cfg1 |= (1 <<
				     PRESTERA_DDR_ECCCFG0_ECC_REGION_PARITY_LOCK_BSHIFT);
			writel(ecc_cfg1, drvdata->base + PRESTERA_DDR_ECCCFG1);
			PRESTERA_DDR_QUASI_PROG_STOP(drvdata);
			iounmap(p8);
			pa = memstart_addr + drvdata->inline_ecc_ofs;
			p32 = phys_to_virt(pa);
			if (!virt_addr_valid(p32)) {
				p32 = ioremap(pa, si.mem_unit);
				require_unmap = true;
			}
			/* read address to generate ECC error: */
			val32 = *p32;
			if (require_unmap)
				iounmap(p32);
		} else {
#ifdef PRESTERA_SIDEBAND_ECC
			/*
			 * Sidebank ECC has HW error injection / poisoning,
			 * so we set the address for poisoning below.
			 * Actual poisoning will occur once a write to this address
			 * will be done normally into the memory system.
			 */
			u32 paddr[2];

			paddr[0] = ((drvdata->rank & PRESTERA_DDR_RANK_MASK)
				    << PRESTERA_DDR_ECCCADDR0_RANK_BSHIFT) |
				   (drvdata->col & PRESTERA_DDR_COL_MASK);
			paddr[1] = ((drvdata->bankgroup & PRESTERA_DDR_BG_MASK)
				    << PRESTERA_DDR_ECCPOISONADDR0_BG_BSHIFT) |
				   ((drvdata->bank & PRESTERA_DDR_BANK_MASK)
				    << PRESTERA_DDR_ECCPOISONADDR0_BANK_BSHIFT) |
				    (drvdata->row & PRESTERA_DDR_ROW_MASK);

			PRESTERA_DDR_STATIC_PROG_START(drvdata);
			writel(paddr[0], drvdata->base + PRESTERA_DDR_ECCPOISONADDR0);
			writel(paddr[1], drvdata->base + PRESTERA_DDR_ECCPOISONADDR1);
			PRESTERA_DDR_STATIC_PROG_STOP(drvdata);

			/* Set the poison data pattern, and enable poison / type: */
			PRESTERA_DDR_QUASI_PROG_START(drvdata);
			writel(drvdata->pattern & 0xffffffff, drvdata->base + PRESTERA_DDR_ECCPOISONPAT0);
			ecc_cfg1 = readl(drvdata->base + PRESTERA_DDR_ECCCFG1);
			ecc_cfg1 |= ((drvdata->error_type << PRESTERA_DDR_ECCCFG1_DATA_POISON_BIT_BSHIFT) | PRESTERA_DDR_ECCCFG1_DATA_POISON_EN);
			writel(ecc_cfg1, drvdata->base + PRESTERA_DDR_ECCCFG1);
			PRESTERA_DDR_QUASI_PROG_STOP(drvdata);
#endif
		}
	}

	/* common stats: */
	stat = readl(drvdata->base + PRESTERA_DDR_ECCSTAT);
	errcnt = readl(drvdata->base + PRESTERA_DDR_ECCERRCNT);

	/* corrected errors stats: */
	ce_addr[0] = readl(drvdata->base + PRESTERA_DDR_ECCCADDR0);
	ce_addr[1] = readl(drvdata->base + PRESTERA_DDR_ECCCADDR1);
	ce_syndrome[0] = readl(drvdata->base + PRESTERA_DDR_ECCCSYN0);
	ce_syndrome[1] = readl(drvdata->base + PRESTERA_DDR_ECCCSYN1);
	ce_syndrome[2] = readl(drvdata->base + PRESTERA_DDR_ECCCSYN2);
	bitmasks[0] = readl(drvdata->base + PRESTERA_DDR_ECCBITMASK0);
	bitmasks[1] = readl(drvdata->base + PRESTERA_DDR_ECCBITMASK1);
	bitmasks[2] = readl(drvdata->base + PRESTERA_DDR_ECCBITMASK2);

	/* uncorrected errors stats: */
	ue_addr[0] = readl(drvdata->base + PRESTERA_DDR_ECCUADDR0);
	ue_addr[1] = readl(drvdata->base + PRESTERA_DDR_ECCUADDR1);
	ue_syndrome[0] = readl(drvdata->base + PRESTERA_DDR_ECCUSYN0);
	ue_syndrome[1] = readl(drvdata->base + PRESTERA_DDR_ECCUSYN1);
	ue_syndrome[2] = readl(drvdata->base + PRESTERA_DDR_ECCUSYN2);

	/* AXI parity errors */
	axi_parity[0] = readl(drvdata->base + PRESTERA_DDR_OCPARSTAT0);
	axi_parity[1] = readl(drvdata->base + PRESTERA_DDR_OCPARSTAT1);
	axi_parity[2] = readl(drvdata->base + PRESTERA_DDR_OCPARSTAT2);

	pstat = readl(drvdata->base + PRESTERA_DDR_POISONSTAT);
	ue_cnt = errcnt >> PRESTERA_DDR_ECCERRCNT_UC_BSHIFT;
	ce_cnt = errcnt & PRESTERA_DDR_ECCERRCNT_CE_MASK;

	/* report correctable error count: */
	if (ce_cnt) {
		snprintf(msg, sizeof(msg),
			 "rank %u bankgroup %u bank %u row %u col %u accum. bitmasks: %08x%08x%08x",
			 (ce_addr[0] >> PRESTERA_DDR_ECCCADDR0_RANK_BSHIFT) & PRESTERA_DDR_RANK_MASK,
			 (ce_addr[1] >> PRESTERA_DDR_ECCCADDR1_BG_BSHIFT) & PRESTERA_DDR_BG_MASK,
			 (ce_addr[1] >> PRESTERA_DDR_ECCCADDR1_BANK_BSHIFT) & PRESTERA_DDR_BANK_MASK,
			 ce_addr[0] & PRESTERA_DDR_ROW_MASK,
			 ce_addr[1] & PRESTERA_DDR_COL_MASK,
			 bitmasks[2], bitmasks[1], bitmasks[0]);

		snprintf(details, sizeof(details),
			 "stat %x axi parity %x %x %x poison stat %x",
			 stat, axi_parity[0], axi_parity[1], axi_parity[2],
			 pstat);

		edac_mc_handle_error(HW_EVENT_ERR_CORRECTED, mci, ce_cnt,
				     ce_addr[0], ce_addr[1],
				     (unsigned long)ce_syndrome[0] | (unsigned long)ce_syndrome[1] << 32,
				     -1, -1, -1, msg, details);
		do_clear = true;
	}

	/* report uncorrectable error count: */
	if (ue_cnt) {
		/* bit offsets and masks are the same for correctable and uncorrectable registers */
		snprintf(msg, sizeof(msg),
			 "rank %u bankgroup %u bank %u row %u col %u",
			 (ue_addr[0] >> PRESTERA_DDR_ECCCADDR0_RANK_BSHIFT) & PRESTERA_DDR_RANK_MASK,
			 (ue_addr[1] >> PRESTERA_DDR_ECCCADDR1_BG_BSHIFT) & PRESTERA_DDR_BG_MASK,
			 (ue_addr[1] >> PRESTERA_DDR_ECCCADDR1_BANK_BSHIFT) & PRESTERA_DDR_BANK_MASK,
			 ue_addr[0] & PRESTERA_DDR_ROW_MASK,
			 ue_addr[1] & PRESTERA_DDR_COL_MASK);

		snprintf(details, sizeof(details),
			 "stat %x axi parity %x %x %x poison stat %x",
			 stat, axi_parity[0], axi_parity[1], axi_parity[2],
			 pstat);

		edac_mc_handle_error(HW_EVENT_ERR_UNCORRECTED, mci, ue_cnt,
				     ue_addr[0], ue_addr[1],
				     ue_syndrome[0] | ue_syndrome[1],
				     -1, -1, -1, msg, details);
		do_clear = true;
	}

	if (do_clear) {
		/* clear counters: */
		writel(PRESTERA_DDR_ECCCTL_CLR_CTRS_MASK, drvdata->base + PRESTERA_DDR_ECCCTL);
	}
}

SYSFS_TEMPLATE_SHOW(data_update);
SYSFS_TEMPLATE_STORE(data_update);
SYSFS_TEMPLATE_SHOW(pattern);
SYSFS_TEMPLATE_STORE(pattern);
SYSFS_TEMPLATE_SHOW(bankgroup);
SYSFS_TEMPLATE_STORE(bankgroup);
SYSFS_TEMPLATE_SHOW(bank);
SYSFS_TEMPLATE_STORE(bank);
SYSFS_TEMPLATE_SHOW(rank);
SYSFS_TEMPLATE_STORE(rank);
SYSFS_TEMPLATE_SHOW(row);
SYSFS_TEMPLATE_STORE(row);
SYSFS_TEMPLATE_SHOW(col);
SYSFS_TEMPLATE_STORE(col);
SYSFS_TEMPLATE_SHOW(inline_ecc_ofs);
SYSFS_TEMPLATE_STORE(inline_ecc_ofs);

static ssize_t prestera_mc_inject_error_type_store(struct device *dev,
					      struct device_attribute *attr,
					      const char *data,
					      size_t count)
{
	struct mem_ctl_info *mci = to_mci(dev);
	struct prestera_lmc_pvt *pvt = mci->pvt_info;

	if (!strncmp(data, "single", 6))
		pvt->error_type = 1;
	else if (!strncmp(data, "double", 6))
		pvt->error_type = 0;

	return count;
}

static ssize_t prestera_mc_inject_error_type_show(struct device *dev,
					     struct device_attribute *attr,
					     char *data)
{
	struct mem_ctl_info *mci = to_mci(dev);
	struct prestera_lmc_pvt *pvt = mci->pvt_info;
	if (pvt->error_type == 1)
		return sprintf(data, "single");
	else if (pvt->error_type == 0)
		return sprintf(data, "double");

	return 0;
}


static DEVICE_ATTR(inject_data_update, S_IRUGO | S_IWUSR,
		   prestera_mc_inject_data_update_show, prestera_mc_inject_data_update_store);
static DEVICE_ATTR(error_type, S_IRUGO | S_IWUSR,
		   prestera_mc_inject_error_type_show, prestera_mc_inject_error_type_store);
static DEVICE_ATTR(pattern, S_IRUGO | S_IWUSR,
		   prestera_mc_inject_pattern_show, prestera_mc_inject_pattern_store);
static DEVICE_ATTR(inline_ecc_ofs, S_IRUGO | S_IWUSR,
		   prestera_mc_inject_inline_ecc_ofs_show, prestera_mc_inject_inline_ecc_ofs_store);
static DEVICE_ATTR(bankgroup, S_IRUGO | S_IWUSR,
		   prestera_mc_inject_bankgroup_show, prestera_mc_inject_bankgroup_store);
static DEVICE_ATTR(rank, S_IRUGO | S_IWUSR,
		   prestera_mc_inject_rank_show, prestera_mc_inject_rank_store);
static DEVICE_ATTR(bank, S_IRUGO | S_IWUSR,
		   prestera_mc_inject_bank_show, prestera_mc_inject_bank_store);
static DEVICE_ATTR(row, S_IRUGO | S_IWUSR,
		   prestera_mc_inject_row_show, prestera_mc_inject_row_store);
static DEVICE_ATTR(col, S_IRUGO | S_IWUSR,
		   prestera_mc_inject_col_show, prestera_mc_inject_col_store);

static struct attribute *prestera_mc_dev_attrs[] = {
	&dev_attr_inject_data_update.attr,
	&dev_attr_error_type.attr,
	&dev_attr_pattern.attr,
	&dev_attr_inline_ecc_ofs.attr,
	&dev_attr_bankgroup.attr,
	&dev_attr_rank.attr,
	&dev_attr_bank.attr,
	&dev_attr_row.attr,
	&dev_attr_col.attr,
	NULL
};

ATTRIBUTE_GROUPS(prestera_mc_dev);

static void prestera_mc_read_config(struct mem_ctl_info *mci)
{
	struct prestera_lmc_pvt *drvdata = mci->pvt_info;
	uint32_t config, rank_ctrl, dev_cfg;
	unsigned int i, cs_cnt, ddr_size_regs[2];
	unsigned int ddr_size_pages;
	struct dimm_info *dimm;
	enum dev_type dtype;	/* memory device type */

	config = readl(drvdata->base + PRESTERA_DDR_MSTR);
	dev_cfg = config >> PRESTERA_DDR_MSTR_DCFG_BSHIFT;
	switch (dev_cfg) {
	case PRESTERA_DDR_X4:
			dtype = DEV_X4;
			break;

	case PRESTERA_DDR_X8:
			dtype = DEV_X8;
			break;

	case PRESTERA_DDR_X16:
			dtype = DEV_X16;
			break;

	case PRESTERA_DDR_X32:
			dtype = DEV_X32;
			break;
	}

	rank_ctrl = (config >> PRESTERA_DDR_MSTR_ACT_RANK_BSHIFT) & PRESTERA_DDR_MSTR_ACT_RANK_MASK;
	/* 0x1 for single rank/CS, 0x11 for dual rank/CS */
	cs_cnt = rank_ctrl > 1 ? 2 : 1;

	/*
	 * MRSTAT + MRCTRL2 actually contain the DDR size written by the
	 * DDR Firmware. Shift into 512 bytes page size.
	 */
	ddr_size_regs[0] = readl(drvdata->base + PRESTERA_DDR_MRSTAT) >> 9;
	ddr_size_regs[1] = readl(drvdata->base + PRESTERA_DDR_MRCTRL2) << 23;
	ddr_size_pages = ddr_size_regs[0] + ddr_size_regs[1];

	for (i = 0; i < cs_cnt; i++) {
		dimm = mci->dimms[i];

		/* assume all DDR ranks are equal in size: */
		dimm->nr_pages = ddr_size_pages / cs_cnt;
		dimm->grain = 8;
		dimm->dtype = dtype;
		dimm->mtype = (config & PRESTERA_DDR_TYPE_DDR4) ?
			MEM_DDR4 : MEM_DDR3;
		dimm->edac_mode = EDAC_SECDED;
	}
}

static int prestera_lmc_edac_probe(struct platform_device *pdev)
{
	struct prestera_lmc_pvt *drvdata;
	struct mem_ctl_info *mci;
	struct edac_mc_layer layers[1];
	struct resource *r;
	void __iomem *base;
	uint32_t config;
	int mc = pdev->id;

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "Unable to get mem resource\n");
		return -ENODEV;
	}

	base = devm_ioremap_resource(&pdev->dev, r);
	if (IS_ERR(base)) {
		dev_err(&pdev->dev, "Unable to map regs\n");
		return PTR_ERR(base);
	}

	layers[0].type = EDAC_MC_LAYER_CHANNEL;
	layers[0].size = 2;
	layers[0].is_virt_csrow = false;

	/*
	 * Note: ECC and/or parity checking cannot be enabled from Linux kernel.
	 * It must be enabled from the DDR firmware in the bootloading stage.
	 * So here we only verify that the DDR firmware has enabled it.
	 */
	config = readl(base + PRESTERA_DDR_ECCCFG0);
	if (!(config & PRESTERA_DDR_ECCCFG0_ECC_MODE_BITS)) {
		dev_err(&pdev->dev, "DDR RAM ECC is not enabled\n");
		return -ENXIO;
	}

	mci = edac_mc_alloc(mc, ARRAY_SIZE(layers), layers, sizeof(struct prestera_lmc_pvt));
	if (!mci)
		return -ENXIO;

	drvdata = mci->pvt_info;
	drvdata->base = base;
	drvdata->inline_ecc = (config >> PRESTERA_DDR_ECCCFG0_ECC_TYPE_BSHIFT) & PRESTERA_DDR_ECCCFG0_ECC_TYPE_MASK;
	mci->pdev = &pdev->dev;
	mci->dev_name = dev_name(&pdev->dev);

	mci->mod_name = "prestera-lmc";
	mci->ctl_name = "prestera_lmc_err";
	mci->edac_check = prestera_lmc_edac_poll;
	mci->mtype_cap = MEM_FLAG_DDR4;
	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->scrub_mode = SCRUB_HW_TUNABLE;

	prestera_mc_read_config(mci);

	if (edac_mc_add_mc_with_groups(mci, prestera_mc_dev_groups)) {
		dev_err(&pdev->dev, "edac_mc_add_mc() failed\n");
		edac_mc_free(mci);
		return -ENXIO;
	}

	/* disable interrupts (poll mode is used), clear counters: */
	writel(PRESTERA_DDR_ECCCTL_CLR_CTRS_MASK, drvdata->base + PRESTERA_DDR_ECCCTL);

	platform_set_drvdata(pdev, mci);
	opstate_init();

	return 0;
}

static int prestera_lmc_edac_remove(struct platform_device *pdev)
{
	struct mem_ctl_info *mci = platform_get_drvdata(pdev);

	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(mci);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id prestera_lmc_of_match[] = {
	{.compatible = "marvell,prestera-edac-mc",},
	{},
};
MODULE_DEVICE_TABLE(of, prestera_lmc_of_match);

static struct platform_driver prestera_lmc_edac_driver = {
	.probe = prestera_lmc_edac_probe,
	.remove = prestera_lmc_edac_remove,
	.driver = {
		   .name = "prestera_lmc_edac",
		   .of_match_table = of_match_ptr(prestera_lmc_of_match)
	}
};
module_platform_driver(prestera_lmc_edac_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elad Nachman <enachman@marvell.com>");
MODULE_DESCRIPTION("EDAC drivers for Marvell PRESTERA SOC memory controller");
