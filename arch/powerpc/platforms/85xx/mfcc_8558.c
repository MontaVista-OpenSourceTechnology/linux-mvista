/*
 * MFCC-8558 board setup
 *
 * Derived from corenet_generic.c
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <asm/ppc-pci.h>
#include <asm/udbg.h>
#include <asm/mpic.h>

#include <sysdev/fsl_pci.h>

void __init corenet_gen_setup_arch(void);
void __init corenet_gen_pic_init(void);
int __init corenet_gen_publish_devices(void);

static int __init mfcc_8558_probe(void)
{
		unsigned long root = of_get_flat_dt_root();

		if (of_flat_dt_is_compatible(root, "mfcc_8558"))
			return 1;

		return 0;
}

define_machine(mfcc_8558) {
	.name                   = "MFCC-8558",
	.probe                  = mfcc_8558_probe,
	.setup_arch             = corenet_gen_setup_arch,
	.init_IRQ               = corenet_gen_pic_init,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus      = fsl_pcibios_fixup_bus,
	.pcibios_fixup_phb      = fsl_pcibios_fixup_phb,
#endif
/*
 * Core reset may cause issue if using the proxy mode of MPIC.
 * Use the mixed mode of MPIC if enabling CPU hotplug.
 */
#ifdef CONFIG_HOTPLUG_CPU
	.get_irq                = mpic_get_irq,
#else
	.get_irq                = mpic_get_coreint_irq,
#endif
	.calibrate_decr         = generic_calibrate_decr,
	.progress               = udbg_progress,
#ifdef CONFIG_PPC64
	.power_save             = book3e_idle,
#else
	.power_save             = e500_idle,
#endif
};

machine_arch_initcall(mfcc_8558, corenet_gen_publish_devices);
#ifdef CONFIG_SWIOTLB
machine_arch_initcall(mfcc_8558, swiotlb_setup_bus_notifier);
#endif
