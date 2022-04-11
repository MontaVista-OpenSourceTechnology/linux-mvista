/*
 * e6500 secondary cores release
 *
 * Initalize all secondary cores and keep them in the spin loop.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/kernel.h>
#include <asm/io.h>
#include <asm/cacheflush.h>
#include <linux/printk.h>
#include "e6500_release_sec.h"

#define BOOT_PAGE_SIZE 4096

extern u32 __e6500_secondary_start;
extern u32 __e6500_spin_table[];

#define E6500_THREAD_BY_CPU           2
#define E6500_NUM_LAW_ENTRY           32

#define E6500_DCFG_BASE_OFFSET        0x000E0000UL
#define E6500_MPIC_BASE_ADDR          0x00040000UL
#define E6500_LCC_BASE_ADDR           0x00000000UL
#define E6500_RCPM_BASE_ADDR          0x000e2000UL

#define E6500_RSTCR_REG_OFFSET        (E6500_DCFG_BASE_OFFSET + 0xB0UL)
#define E6500_BRR_REG_OFFSET          (E6500_DCFG_BASE_OFFSET + 0xe4UL)

#define E6500_FRR_REG_OFFSET          (E6500_MPIC_BASE_ADDR + 0x1000UL)
#define E6500_WHOAMI_REG_OFFSET       (E6500_MPIC_BASE_ADDR + 0x00000090UL)
#define E6500_PIR_REG_OFFSET          (E6500_MPIC_BASE_ADDR + 0x00001090UL) /* sreset */
#define E6500_FRR_NCPU_SHIFT          8
#define E6500_FRR_NCPU_MASK           0x1F00

#define E6500_BSTRH_REG_OFFSET        (E6500_LCC_BASE_ADDR + 0x20UL)
#define E6500_BSTRL_REG_OFFSET        (E6500_LCC_BASE_ADDR + 0x24UL)
#define E6500_BSTAR_REG_OFFSET        (E6500_LCC_BASE_ADDR + 0x28UL)
#define E6500_BSTAR_ENABLE            0x80000000UL
#define E6500_BSTAR_TARGET_SHIFT      20
#define E6500_BSTAR_WINDOW_SIZE_4k    0x0000000bUL

#define E6500_CTBENRL_REG_OFFSET      (E6500_RCPM_BASE_ADDR + 0x1a0UL)

#define E6500_RESET_MAX_INSTANCE      12UL

#define E6500_LAW_OFFSET              0x00000C00UL
#define E6500_LAW_NUM_OFFSET          0x00000010UL
#define E6500_LAW_HIGH_OFFSET         0x00000000UL
#define E6500_LAW_LOW_OFFSET          0x00000004UL
#define E6500_LAW_ATTR_OFFSET         0x00000008UL
#define E6500_LAW_ATTR_SIZE_MASK      0x0000003FUL
#define E6500_LAW_TARGET_MASK         0x0FF00000UL
#define E6500_LAW_TARGET_SHIFT        20
#define E6500_LAW_ENABLE              31

static u32 find_law(void *ccsr, phys_addr_t phys_addr)
{
	u32 attr, ret = 0;
	void *addr;
	u64 physAddr, physAddrUp;
	u32 i;

	for (i = 0; i < E6500_NUM_LAW_ENTRY; i++) {
		addr = ccsr + E6500_LAW_OFFSET + i * E6500_LAW_NUM_OFFSET;
		attr = in_be32(addr + E6500_LAW_ATTR_OFFSET);

		physAddr = ((u64)in_be32(addr + E6500_LAW_HIGH_OFFSET) << 32) |
			in_be32(addr + E6500_LAW_LOW_OFFSET);

		physAddrUp = physAddr + (2ULL << (attr & E6500_LAW_ATTR_SIZE_MASK));

		if (!(attr & BIT(E6500_LAW_ENABLE)) || (phys_addr < physAddr) ||
		    (phys_addr >= physAddrUp))
			continue;

		ret = ((attr & E6500_LAW_TARGET_MASK) >> E6500_LAW_TARGET_SHIFT);
		break;
	}
	return ret;
}

static u64 get_spin_address(void)
{
	return BOOT_PAGE + (u64)&__e6500_spin_table - (u64)&__e6500_secondary_start;
}

int release_sec(void)
{
	volatile u32 *table = phys_to_virt(get_spin_address());
	void __iomem *ccsr;
	u32 *fixup = (u32 *)&__e6500_secondary_start;
	u32 *bootpage = phys_to_virt(BOOT_PAGE);

	u32 whoami, cpu_up, cpu_up_mask = 0, law_target, nr_cpu;
	u32 timeout = 100; /* loop */
	u32 bstar, time = 0, i;
	int err = 0;

	ccsr = ioremap(CCSRBAR, E6500_RCPM_BASE_ADDR + 0x1000);
	if (!ccsr)
		return -ENOMEM;

	/* NCPU contain the number of threads */
	nr_cpu = ((in_be32(ccsr + E6500_FRR_REG_OFFSET) & E6500_FRR_NCPU_MASK)
		  >> E6500_FRR_NCPU_SHIFT) + 1;

	nr_cpu /= E6500_THREAD_BY_CPU;
	cpu_up = (1 << nr_cpu) - 1;

	whoami = in_be32(ccsr + E6500_WHOAMI_REG_OFFSET);
	cpu_up_mask = in_be32(ccsr + E6500_BRR_REG_OFFSET);

	pr_debug("nr_cpu %d, whoami %d, cpu_up_mask %x\n", nr_cpu, whoami,
		 cpu_up_mask);

	if (cpu_up_mask != 0) {
	    /* CPUs are already up from firmware, just go on. */
	    pr_info("CPUs are already up, use standard release loop.\n");
	    err = -EAGAIN;
	    goto out;
	}
	pr_info("CPUs are not up, bringing them up.\n");
	cpu_up_mask = 1 << whoami;

	law_target = find_law(ccsr, BOOT_PAGE);
	if (!law_target) {
		pr_err("Couldn't find LAW window for bootpage address...\n");
		err = -EINVAL;
		goto out;
	}
	pr_debug("found law with target %d\n", law_target);

	for (i = 0; i < BOOT_PAGE_SIZE / sizeof(u32); i++)
		bootpage[i] = fixup[i];

	flush_dcache_range((unsigned long)bootpage,
			   (unsigned long)bootpage + BOOT_PAGE_SIZE);

	/* set boot space translation attribute */
	out_be32(ccsr + E6500_BSTRH_REG_OFFSET, 0);
	out_be32(ccsr + E6500_BSTRL_REG_OFFSET, BOOT_PAGE);
	out_be32(ccsr + E6500_BSTAR_REG_OFFSET,
		 E6500_BSTAR_ENABLE | (law_target << E6500_BSTAR_TARGET_SHIFT) |
		 E6500_BSTAR_WINDOW_SIZE_4k);

	bstar = in_be32(ccsr + E6500_BSTAR_REG_OFFSET);
	pr_debug("BSTAR 0x%x\n", bstar);

	/* timebase_disable */
	out_be32(ccsr + E6500_CTBENRL_REG_OFFSET, 0);

	/* set my local time base to 0 */
	asm volatile("  mtspr 284,%[gpr]\n\t" /* Time Base Lower */
		     "  mtspr 285,%[gpr]\n\t" /* Time Base Upper */
		     "  mtspr 284,%[gpr]\n\t"
		     : /* output */
		     : [gpr] "r" (time)   /* input */);

	/* release secondary CPUs */
	out_be32(ccsr + E6500_BRR_REG_OFFSET, cpu_up);

	/* wait that all entry addr lower in spin_table are different than zero */
	while (timeout) {
		u32 i, v;
		volatile u32 *p = table + BOOT_ENTRY_ADDR_LOWER / sizeof(u32);

		flush_dcache_range((unsigned long)table,
				   (unsigned long)table + E6500_CPU_NUM * BOOT_ENTRY_SIZE);

		for (i = 0; i < nr_cpu; i++) {
			v = *p;
			if (v)
				cpu_up_mask |= (1 << i);

			p += BOOT_ENTRY_SIZE / sizeof(u32);
		}

		if ((cpu_up_mask & cpu_up) == cpu_up)
			break; /* done */

		timeout--;
	}

	/* timebase_enable */
	out_be32(ccsr + E6500_CTBENRL_REG_OFFSET, cpu_up);

	pr_debug("CPU release timeout %d, ready mask 0x%x\n", timeout, cpu_up_mask);

	if (timeout == 0) {
	    pr_err("Unable to release all CPUs: ready mask 0x%x\n",
		   cpu_up_mask);
	    err = -EINVAL;
	}

 out:
	iounmap(ccsr);

	return err;
}
