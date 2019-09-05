/*
 * Copyright (C) 2016 Broadcom
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

/* Currently, this driver is only support for Helix5. As for the other
 * XGS IProc chips, the initial code and reset handler are defined in
 * mach-iproc/board_bu.c
 */

#include <asm/proc-fns.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/soc/bcm/xgs-iproc-misc-setup.h>
#include <linux/soc/bcm/xgs-iproc-idm.h>
#include <linux/soc/bcm/iproc-cmic.h>

#define DMU_CRU_RESET_OFFSET		0x200

static int xgs_iproc_restart(struct notifier_block *nb,
							unsigned long action, void *data)
{
	void * __iomem reg_addr;
	u32 val;

	/* CRU_RESET register */
	reg_addr = get_iproc_dmu_pcu_base() + DMU_CRU_RESET_OFFSET;

	/* set iproc_reset_n to 0 */
	val = readl(reg_addr);
	val &= ~((u32) 1 << 1);

	writel(val, reg_addr);

	/* Wait for reset */
	while (1) {
		cpu_do_idle();
	}

	return 0;
}

static struct notifier_block xgs_iproc_nb = {
	.notifier_call  = xgs_iproc_restart,
	.priority       = 192,
};

#define XGS_IPROC_HX5_COMPATIBLE 	"brcm,helix5"
#define XGS_IPROC_HR4_COMPATIBLE 	"brcm,hurricane4"

static int __init xgs_iproc_init(void)
{
	int ret;
	int dev_id = 0;

	if (of_machine_is_compatible(XGS_IPROC_HX5_COMPATIBLE)) {
		dev_id = IPROC_DEVICE_HX5;
	} else if (of_machine_is_compatible(XGS_IPROC_HR4_COMPATIBLE)) {
		dev_id = IPROC_DEVICE_HR4;
	}

	if (!dev_id) {
		return -EINVAL;
	}

	ret = xgs_iproc_misc_setup();
	if (ret < 0) {
		return ret;
	}

	/* Init idm and setup idm timeout handler for debug purpose */
	/* xgs_iproc_idm_init should be init before reset dmac */
	ret = xgs_iproc_idm_init();
	if (ret < 0) {
		return ret;
	}

	ret = xgs_iproc_cmic_init(dev_id);
	if (ret < 0) {
		return ret;
	}

	/* FIXME, need confirm whether we need reset the DMAC or not */
	xgs_iproc_idm_dmac_reset();

	/* Populate platform devices */
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);

	if (register_reboot_notifier(&xgs_iproc_nb)) {
		printk("Register reboot handler failed\n");
	}

	return 0;
}
arch_initcall(xgs_iproc_init);
