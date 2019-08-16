/*
 * Copyright (C) 2016 Broadcom Corporation
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

#include <asm/mach/arch.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>
#include <linux/soc/bcm/xgs-iproc-misc-setup.h>
#include <linux/soc/bcm/xgs-iproc-idm.h>

#define DMU_CRU_RESET_BASE 0x200

#define FSR_EXTERNAL           (1 << 12)
#define FSR_READ               (0 << 10)
#define FSR_IMPRECISE          0x0406

enum xgs_iproc_dev_id {
	XGS_IPROC_HX4=0,
	XGS_IPROC_KT2,
	XGS_IPROC_HR2,
	XGS_IPROC_GH,
	XGS_IPROC_SB2,
	XGS_IPROC_HR3,
	XGS_IPROC_GH2,
	XGS_IPROC_WH2,
	XGS_IPROC_GENERIC,
};

const char *const xgs_iproc_dt_compat[] = {
	"brcm,helix4",
	"brcm,katana2",
	"brcm,hurricane2",
	"brcm,greyhound",
	"brcm,saber2",
	"brcm,hurricane3",
	"brcm,greyhound2",
	"brcm,wolfhound2",
	"brcm,xgs-iproc",
	NULL,
};

#ifdef CONFIG_ML66_NPU_IPROC_PLATFORM

static int xgs_iproc_abort_handler(unsigned long addr, unsigned int fsr,
		struct pt_regs *regs)
{
	/*
	 * We want to ignore aborts forwarded from the PCIe bus that are
	 * expected and shouldn't really be passed by the PCIe controller.
	 * The biggest disadvantage is the same FSR code may be reported when
	 * reading non-existing APB register and we shouldn't ignore that.
	 */
	if (fsr == (FSR_EXTERNAL | FSR_READ | FSR_IMPRECISE))
		return 0;

	return 1;
}

static int (*fn_pci_fault)(unsigned long, unsigned int, struct pt_regs *);

static int xgs_iproc_pci_abort_handler(unsigned long addr, unsigned int fsr,
		struct pt_regs *regs)
{
	if (fn_pci_fault)
		return fn_pci_fault(addr, fsr, regs);
	return 1;
}

void hook_pci_fault_code(int (*fn)(unsigned long, unsigned int, struct pt_regs *))
{
	fn_pci_fault = fn;
}
EXPORT_SYMBOL(hook_pci_fault_code);

#endif /* CONFIG_ML66_NPU_IPROC_PLATFORM */

void __init xgs_iproc_init_early(void)
{
	/*
	 * SDK allocates coherent buffers from atomic context.
	 * Increase size of atomic coherent pool to make sure such
	 * allocations won't fail.
	 */
	/* can be overrided by "coherent_pool" in kernel boot argument */
	if (IS_ENABLED(CONFIG_DMA_CMA))
		init_dma_coherent_pool_size(SZ_1M * 16);
	hook_fault_code(16 + 6, xgs_iproc_abort_handler, SIGBUS, BUS_OBJERR,
			"imprecise external abort");
	hook_fault_code(8, xgs_iproc_pci_abort_handler, SIGBUS, 0,
			"external abort on non-linefetch");
}

static void __init xgs_iproc_init(void)
{
	int ret;

	ret = xgs_iproc_misc_setup();
	if (ret < 0)
		return;

	/* Init idm and setup idm timeout handler for debug purpose */
	/* xgs_iproc_idm_init should be init before reset dmac */
	ret = xgs_iproc_idm_init();
	if (ret < 0)
		return;

	xgs_iproc_idm_dmac_reset();

	/* Populate platform devices */
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
}


static void xgs_iproc_restart(enum reboot_mode mode, const char *cmd)
{
	void * __iomem reg_addr;
	u32 reg;

	/* CRU_RESET register */
	reg_addr = (void * __iomem)(get_iproc_dmu_pcu_base() +
					DMU_CRU_RESET_BASE);
	/* set iproc_reset_n to 0 */
	reg = readl(reg_addr);
	reg &= ~((u32) 1 << 1);

	writel(reg, reg_addr);

	/* Wait for reset */
	while (1)
		cpu_do_idle();
}

DT_MACHINE_START(XGS_iProc_DT, "BRCM XGS iProc")
	.init_early = xgs_iproc_init_early,
	.init_machine = xgs_iproc_init,
	.dt_compat = xgs_iproc_dt_compat,
	.restart = xgs_iproc_restart,
	.l2c_aux_val    = 0,
	.l2c_aux_mask	= ~0,
MACHINE_END
