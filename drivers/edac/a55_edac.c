// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ARM Cortex A55 EDAC (error detection and correction)
 * Should be partially compatible with most ARMv8.2 Cortex A series CPUs
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

/* binutils definitions do not match ARM Cortex A55 definitions: */
#define SYS_ERXPFGCDNR_EL1			sys_reg(3, 0, 15, 2, 2)
#define SYS_ERXPFGCTLR_EL1			sys_reg(3, 0, 15, 2, 1)


#define ERRIDR_MASK 0xFFFF
#define ERXADDR_MASK 0x00FFFFFFFFFFFFFFLLU
#define ERRCTRL_MASK 0xFFF

/* uncorrectable write, combined read/write reporting enable: */
#define ERXCTLR_WUE BIT(7)
#define ERXCTLR_UE BIT(4)

/* error reporting and logging enable bit: */
#define ERXCTLR_ED BIT(0)

/* register bits shifts/masks (post shift) definitions: */

/* nonspecific error count: */
#define ERXMISC0_CECO_BSHIFT 40
#define ERXMISC0_CECO_MASK 0x7F

/* Specific errors matching the location specified in this register: */
#define ERXMISC0_CECR_BSHIFT 32
#define ERXMISC0_CECR_MASK 0x7F

/* way or bank where above errors have occured: */
#define ERXMISC0_WAY_BSHIFT 28
#define ERXMISC0_WAY_MASK 0x7F

/* index where above errors have occured: */
#define ERXMISC0_IDX_BSHIFT 6
#define ERXMISC0_IDX_MASK 0x1FFF

/* Level where above errors have occured: */
#define ERXMISC0_LVL_BSHIFT 1
#define ERXMISC0_LVL_MASK 0x7

/* Type of cache containing the error: */
#define ERXMISC0_IND_BIT BIT(0)

#define ERXSTATUS_ADDR_VALID 	BIT(31)
#define ERXSTATUS_STATUS_VALID 	BIT(30)
#define ERXSTATUS_UE 		BIT(29)
/* inband error (external abort) : */
#define ERXSTATUS_ER 		BIT(28)
/* overflow (new error occured when previous error was not cleared): */
#define ERXSTATUS_OF 		BIT(27)
#define ERXSTATUS_MISC_VALID	BIT(26)
#define ERXSTATUS_CE		BIT(25)

/* deferred error (by poisoning) */
#define ERXSTATUS_DE		BIT(23)
/* unused in A55: */
#define ERXSTATUS_PN		BIT(22)
/* error on L1 dirty RAM, bits [15:9] not used for A55: */
#define ERXSTATUS_IERR		BIT(8)
#define ERXSTATUS_SERR_MASK	0xFF

/* Enable countdown using the value from ERXPFGCDNR_EL1 */
#define ERXPFGCTLR_CDNEN	BIT(31)
/* Restartable: when reaching zero, reload using the ERXPFGCDNR_EL1 value */
#define ERXPFGCTLR_R		BIT(30)
/* Corrected error generation enable: */
#define ERXPFGCTLR_CE		BIT(6)
/* Deferred error generation enable: */
#define ERXPFGCTLR_DE		BIT(5)
/* Signaled or recoverable error generation enable: */
#define ERXPFGCTLR_UER		BIT(3)
/* Uncontainable error generation enable: */
#define ERXPFGCTLR_UC		BIT(1)

struct a55_dbgfs_entry {
	const struct file_operations *fops;
	const char *name;
};

struct a55_edac_private {
	int num_blocks;
	struct dentry *debugfs_dir;
	unsigned err_inj_update;
	unsigned err_inj_cntdown_val;
	unsigned err_inj_auto_restart_cntdown;
	unsigned err_inj_correctable_err;
	unsigned err_inj_deferred_err;
	unsigned err_inj_signalled_recoverable_err;
	unsigned err_inj_uncontainable_err;
};

struct a55_edac_error_strings {
	u8 code;
	char *str;
};

struct a55_edac_error_strings a55_edac_error_strings_ar[] = {
	{0x0,	"No error"},
	{0x2,	"ECC error from internal data buffer"},
	{0x6,	"ECC error on cache data RAM"},
	{0x7,	"ECC error on cache tag or dirty RAM"},
	{0x8,	"Parity error on TLB data RAM"},
	{0x9,	"Parity error on TLB tag RAM"},
/*
 * For example, poisoned data received from a slave by a master that cannot
 * defer the error further:
 */
	{0x15,	"Deferred error from slave not supported at the consumer"},
	{0xff, NULL}
};

static const char *a55_cpu_edac_get_errmsg(u8 code)
{
	u8 i = 0;

	while (a55_edac_error_strings_ar[i].str) {
		if (a55_edac_error_strings_ar[i].code == code)
			return a55_edac_error_strings_ar[i].str;
		i++;
	}

	return NULL;
}

/* Return the number of error record entries in the ARM core: */
static u64 erridr_read(void)
{
	asm(".arch armv8.2-a+ras");
	return read_sysreg(erridr_el1) & ERRIDR_MASK;
}

/* Selects the record entry to access: */
static void errselr_write(u16 idx)
{
	write_sysreg(idx & ERRIDR_MASK, errselr_el1);
}

/* Return the address in the error record entry in the ARM core: */
static u64 erxaddr_read(void)
{
	return read_sysreg(erxaddr_el1) & ERXADDR_MASK;
}

/* Writes the record entry control register: */
static void erxctlr_write(u16 val)
{
	write_sysreg(val & ERRCTRL_MASK, erxctlr_el1);
}

/* Return the misc0 register in the error record entry in the ARM core: */
static u64 erxmisc0_read(void)
{
	return read_sysreg(erxmisc0_el1);
}

/* Return the status register in the error record entry in the ARM core: */
static u64 erxstatus_read(void)
{
	return read_sysreg(erxstatus_el1);
}

/* Writes the status register in the error record entry in the ARM core: */
static void erxstatus_write(u64 val)
{
	write_sysreg(val, erxstatus_el1);
}

/* Writes the Selected Error Pseudo Fault Generation Count Down Register: */
static void erxpfgcdn_write(u32 val)
{
	write_sysreg_s(val, SYS_ERXPFGCDNR_EL1);
}

/* Writes the Selected Error Pseudo Fault Generation Control Register: */
static void erxpfgctl_write(u32 val)
{
	write_sysreg_s(val, SYS_ERXPFGCTLR_EL1);
}

static void a55_cpu_edac_poll(struct edac_device_ctl_info *cpu)
{
	int i;
	struct a55_edac_private *drvdata;
	u64 status, misc = 0, addr = 0;
	char msg[256];
	const char *errmsg;

	drvdata = cpu->pvt_info;

	for (i = 0; i < drvdata->num_blocks; i++) {
		errselr_write(i);

		/*pr_err("erx %d st %llx addr %llx misc %llx\n", i, erxstatus_read(), erxaddr_read(), erxmisc0_read() );*/
		status = erxstatus_read();
		if (!(status & ERXSTATUS_STATUS_VALID))
			continue;

		if (status & ERXSTATUS_ADDR_VALID)
			addr = erxaddr_read();

		if (status & ERXSTATUS_MISC_VALID)
			misc = erxmisc0_read();

		errmsg = a55_cpu_edac_get_errmsg(status & ERXSTATUS_SERR_MASK);

		snprintf(msg, sizeof(msg),
			 "%s %s %s %s addr %llx corr count other %llu specific %llu way %llu idx %llu lvl %llu type: %s",
			 status & ERXSTATUS_ER ? "ext abort, " : "",
			 status & ERXSTATUS_DE ? "deferred, " : "",
			 status & ERXSTATUS_IERR ?
			 "err on L1 dirty RAM, " : "err on other RAM",
			 errmsg ? errmsg : "",
			 addr,
			 (misc >> ERXMISC0_CECO_BSHIFT) & ERXMISC0_CECO_MASK,
			 (misc >> ERXMISC0_CECR_BSHIFT) & ERXMISC0_CECR_MASK,
			 (misc >> ERXMISC0_WAY_BSHIFT) & ERXMISC0_WAY_MASK,
			 (misc >> ERXMISC0_IDX_BSHIFT) & ERXMISC0_IDX_MASK,
			 (misc >> ERXMISC0_LVL_BSHIFT) & ERXMISC0_LVL_MASK,
			 ERXMISC0_IND_BIT ? "Lvl 1 instr cache" : "data cache");

		if (status & (ERXSTATUS_UE | ERXSTATUS_DE))
			edac_device_handle_ue(cpu, 0, i, msg);

		if (status & ERXSTATUS_CE)
			edac_device_handle_ce(cpu, 0, i, msg);

		erxstatus_write(status & (~(ERXSTATUS_STATUS_VALID | ERXSTATUS_ADDR_VALID | ERXSTATUS_MISC_VALID)));
	}

	/* if update requested, update the ARM core error injection registers: */
	if (drvdata->err_inj_update) {
		drvdata->err_inj_update = 0;

		erxpfgcdn_write(drvdata->err_inj_cntdown_val);

		erxpfgctl_write(ERXPFGCTLR_CDNEN |
				(drvdata->err_inj_auto_restart_cntdown ?
				ERXPFGCTLR_R : 0) |
				(drvdata->err_inj_correctable_err ?
				ERXPFGCTLR_CE : 0) |
				(drvdata->err_inj_deferred_err ?
				ERXPFGCTLR_DE : 0) |
				(drvdata->err_inj_signalled_recoverable_err ?
				ERXPFGCTLR_UER : 0) |
				(drvdata->err_inj_uncontainable_err ?
				ERXPFGCTLR_UC : 0));

	}

}

A55_DBGFS_TEMPLATE_WRITE(err_inj_update)
A55_DBGFS_TEMPLATE_WRITE(err_inj_cntdown_val)
A55_DBGFS_TEMPLATE_WRITE(err_inj_auto_restart_cntdown)
A55_DBGFS_TEMPLATE_WRITE(err_inj_correctable_err)
A55_DBGFS_TEMPLATE_WRITE(err_inj_deferred_err)
A55_DBGFS_TEMPLATE_WRITE(err_inj_signalled_recoverable_err)
A55_DBGFS_TEMPLATE_WRITE(err_inj_uncontainable_err)

A55_DBGFS_TEMPLATE_READ(err_inj_update)
A55_DBGFS_TEMPLATE_READ(err_inj_cntdown_val)
A55_DBGFS_TEMPLATE_READ(err_inj_auto_restart_cntdown)
A55_DBGFS_TEMPLATE_READ(err_inj_correctable_err)
A55_DBGFS_TEMPLATE_READ(err_inj_deferred_err)
A55_DBGFS_TEMPLATE_READ(err_inj_signalled_recoverable_err)
A55_DBGFS_TEMPLATE_READ(err_inj_uncontainable_err)

/* file operations structures per injection variable: */
A55_DBGFS_TEMPLATE_FOPS(err_inj_update)
A55_DBGFS_TEMPLATE_FOPS(err_inj_cntdown_val)
A55_DBGFS_TEMPLATE_FOPS(err_inj_auto_restart_cntdown)
A55_DBGFS_TEMPLATE_FOPS(err_inj_correctable_err)
A55_DBGFS_TEMPLATE_FOPS(err_inj_deferred_err)
A55_DBGFS_TEMPLATE_FOPS(err_inj_signalled_recoverable_err)
A55_DBGFS_TEMPLATE_FOPS(err_inj_uncontainable_err)

static const struct a55_dbgfs_entry a55_cpu_dev_fops[] = {
	A55_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_update),
	A55_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_cntdown_val),
	A55_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_auto_restart_cntdown),
	A55_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_correctable_err),
	A55_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_deferred_err),
	A55_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_signalled_recoverable_err),
	A55_TEMPLATE_FOPS_DBGFS_ENTRY(err_inj_uncontainable_err),
	{ NULL, NULL }
};

static void a55_edac_cpu_create_dbgfs(struct edac_device_ctl_info *cpu,
				      struct platform_device *pdev)
{
	int i = 0;
	struct a55_edac_private *drvdata = cpu->pvt_info;

	if (!IS_ENABLED(CONFIG_EDAC_DEBUG))
		return;

	drvdata->debugfs_dir = edac_debugfs_create_dir(pdev->dev.kobj.name);
	if (!drvdata->debugfs_dir)
		return;

	while (a55_cpu_dev_fops[i].name) {
		if (!edac_debugfs_create_file(a55_cpu_dev_fops[i].name, S_IWUSR,
					      drvdata->debugfs_dir, cpu,
					      a55_cpu_dev_fops[i].fops))
			debugfs_remove_recursive(drvdata->debugfs_dir);
		i++;
	}
}


static int a55_cpu_edac_probe(struct platform_device *pdev)
{
	struct edac_device_ctl_info *cpu;
	struct a55_edac_private *pvt;
	u32 i, num_entries;

	/* Read number of error database entries available in the CPU: */
	num_entries = erridr_read();
	/* enable error reporting for all entries: */

	for (i = 0; i < num_entries; i++) {
		errselr_write(i);
		erxctlr_write(ERXCTLR_ED);
	}

	/* block #0 is the ARM core error record, #1 is for the DSU error record: */
	cpu = edac_device_alloc_ctl_info(sizeof(struct a55_edac_private), "a55-cpu", 1, "core_dsu",
					 num_entries, 0,
					 NULL, 0, edac_device_alloc_index());
	if (!cpu) {
		pr_err("%s: edac_device_alloc_ctl_info() failed\n", __func__);
		return -ENOMEM;
	}

	/* setup private EDAC area with number of blocks read from CPU: */
	pvt = cpu->pvt_info;

	pvt->num_blocks = num_entries;

	cpu->dev = &pdev->dev;

	platform_set_drvdata(pdev, cpu);

	/* Setup EDAC generic structure with names: */
	cpu->dev_name = dev_name(&pdev->dev);

	cpu->mod_name = "a55-cpu";
	cpu->ctl_name = "a55-cpu_err";

	/* store poll function to be called by kernel: */
	cpu->edac_check = a55_cpu_edac_poll;

	/* 
	 * Finally, create debugfs entries used to inject errors and add device
	 * to EDAC subsystem:
	 */

	a55_edac_cpu_create_dbgfs(cpu, pdev);

	if (edac_device_add_device(cpu) > 0) {
		pr_err("%s: edac_device_add_device() failed\n", __func__);
		goto err;
	}

	return 0;

err:
	edac_device_free_ctl_info(cpu);

	return -ENXIO;
}

static int a55_cpu_edac_remove(struct platform_device *pdev)
{
	struct edac_device_ctl_info *cpu = platform_get_drvdata(pdev);
	struct a55_edac_private *drvdata = cpu->pvt_info;

	debugfs_remove_recursive(drvdata->debugfs_dir);
	edac_device_del_device(&pdev->dev);
	edac_device_free_ctl_info(cpu);

	return 0;
}

static const struct of_device_id a55_cpu_of_match[] = {
	{.compatible = "arm,armv8-a55-edac",},
	{},
};
MODULE_DEVICE_TABLE(of, a55_cpu_of_match);

static struct platform_driver a55_cpu_edac_driver = {
	.probe = a55_cpu_edac_probe,
	.remove = a55_cpu_edac_remove,
	.driver = {
		   .name = "edac_a55",
		   .of_match_table = of_match_ptr(a55_cpu_of_match)
	}
};

module_platform_driver(a55_cpu_edac_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Elad Nachman <enachman@marvell.com>");
