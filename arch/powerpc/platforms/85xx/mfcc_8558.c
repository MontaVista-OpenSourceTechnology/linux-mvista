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
#include <linux/of_address.h>

#ifdef CONFIG_PCI
#include <sysdev/fsl_pci.h>
#endif

#include "mpc85xx.h"

void __init corenet_gen_pic_init(void);
int __init corenet_gen_publish_devices(void);

#define LAWAR_EN		0x80000000
#define LAWAR_TARGET_MASK	0x0FF00000
#define LAWAR_TARGET_SHIFT	20
#define LAWAR_SIZE_MASK		0x0000003F
#define LAWAR_CSDID_MASK	0x000FF000
#define LAWAR_CSDID_SHIFT	12

#define LAW_SIZE_4K		0xb

struct ccsr_law {
	u32	lawbarh;	/* LAWn base address high */
	u32	lawbarl;	/* LAWn base address low */
	u32	lawar;		/* LAWn attributes */
	u32	reserved;
};

#ifdef CONFIG_PCI
#define TARGET_ID_PCIE1 0x00
#define TARGET_ID_PCIE2 0x01
#define TARGET_ID_PCIE3 0x02
#define TARGET_ID_PCIE4 0x03
#endif
#define TARGET_ID_BMAN  0x18
#define TARGET_ID_QMAN  0x3C

struct property_range {
	uint32_t reserved;
	uint32_t hi;
	uint32_t lo;
	uint32_t size;
};

#define LAW_OFFSET              0x00000C00UL
#define LAW_NUM_OFFSET          0x00000010UL

#define LAWAR_TARGET_ID(_x) (((_x) & LAWAR_TARGET_MASK) >> LAWAR_TARGET_SHIFT)

static int law_enable(const char *name, struct ccsr_law __iomem *law, int num_law,
		      uint8_t target_id, uint64_t addr, uint64_t size)
{
	int i;

	/* Find a free LAW */
	i = num_law;
	while (law[--i].lawar & LAWAR_EN) {
		if (i == 0) {
			/* No higher priority LAW slots available */
			pr_err("Failed to setup 0x%llx - 0x%llx for target 0x%x\n",
			       addr, addr + size - 1, target_id);
			return -EIO;
		}
	}

	law[i].lawbarh = upper_32_bits(addr);
	law[i].lawbarl = lower_32_bits(addr);
	wmb();

	law[i].lawar = 0
	  | LAWAR_EN
	  | (target_id << LAWAR_TARGET_SHIFT)
	  | 0
	  | (LAW_SIZE_4K + get_order(size));
	wmb();

	pr_info("%s law %d target %x bar 0x%x 0x%08x ar 0x%x\n", name, i, target_id,
		law[i].lawbarh, law[i].lawbarl, law[i].lawar);
	return 0;
}

static int law_setup_range(struct ccsr_law __iomem *law, int num_law,
			   uint8_t target_id, const char *name)
{
	struct device_node *np;
	int len;
	struct property_range range;
	uint64_t addr;

	np = of_find_node_by_name(NULL, name);
	if (!np) {
		pr_warn("No %s node found.\n", name);
		return -ENODEV;
	}

	len = of_property_read_variable_u32_array(
		np, "ranges", (uint32_t *)&range, 0, sizeof(range) / sizeof(uint32_t));
	if (len != 4) {
		pr_warn("No valid ranges defined for %s.\n", name);
		return -EINVAL;
	}

	addr = ((uint64_t) range.hi << 32) | range.lo;
	return law_enable(name, law, num_law, target_id, addr, range.size);
}

#ifdef CONFIG_PCI
static int law_setup_pcie(struct ccsr_law __iomem *law,
				   int num_law, uint8_t target_id) {
	struct device_node *np;
	char name[32];
	struct of_pci_range_parser parser;
	struct of_pci_range range;
	int ret;

	snprintf(name, sizeof(name), "pcie@ffe2%c0000", '4' + target_id);

	np = NULL;
	while ((np = of_find_compatible_node(np, NULL, "fsl,qoriq-pcie")) != NULL)
		if (strncmp(name, of_node_full_name(np), strlen(name)) == 0)
			break;

	if (!np) {
		pr_err("No %s node found.\n", name);
		return 0;
	}

	/* Do nothing if controller is disabled */
	if (!of_device_is_available(np)) {
		pr_debug("%s disabled\n", name);
		return 0;
	}

	/* Check for ranges property */
	if (of_pci_range_parser_init(&parser, np)) {
		pr_err("No ranges defined for %s.\n", name);
		return -EINVAL;
	}

	for_each_of_pci_range(&parser, &range) {
		switch (range.flags & IORESOURCE_TYPE_BITS) {
		case IORESOURCE_IO:
		case IORESOURCE_MEM:
			ret = law_enable(name, law, num_law, target_id,
					 range.cpu_addr, range.size);
			if (ret)
				return ret;
			break;
		default:
			break;
		}
	}
	return 0;
}

static void mfcc_8558_fixup_pci_phb(struct pci_controller *hose)
{
    /*
     * If this undocumented register at least has the low bit set to
     * 1, the PCI device will not work.  Zero it.  It's coming from
     * mmsi boot with the bit set, but not PPCMON boot.
     */
    early_write_config_dword(hose, 0, 0, 0x8bc, 0);

    fsl_pcibios_fixup_phb(hose);
}
#endif

static int law_setup_bman(struct ccsr_law __iomem *law, int num_law, uint8_t target_id)
{
	return law_setup_range(law, num_law, target_id, "bman-portals");
}

static int law_setup_qman(struct ccsr_law __iomem *law, int num_law, uint8_t target_id)
{
	return law_setup_range(law, num_law, target_id, "qman-portals");
}

struct ccsr_law_setup {
	uint8_t target_id;
	int (*setup)(struct ccsr_law __iomem *law, int num_law, uint8_t target_id);
};

struct ccsr_law_setup law_setup[] = {
#ifdef CONFIG_PCI
	{ TARGET_ID_PCIE1, law_setup_pcie },
	{ TARGET_ID_PCIE2, law_setup_pcie },
	{ TARGET_ID_PCIE3, law_setup_pcie },
	{ TARGET_ID_PCIE4, law_setup_pcie },
#endif
	{ TARGET_ID_BMAN, law_setup_bman },
	{ TARGET_ID_QMAN, law_setup_qman },
};

static int law_config(void)
{
	struct device_node *np;
	const __be32 *iprop;
	void __iomem *lac = NULL;
	struct ccsr_law __iomem *law;
	unsigned int i, j, num_laws;
	int ret = 0;

	np = of_find_compatible_node(NULL, NULL, "fsl,corenet-law");
	if (!np)
		return -ENODEV;

	iprop = of_get_property(np, "fsl,num-laws", NULL);
	if (!iprop) {
		ret = -EINVAL;
		goto error;
	}

	num_laws = be32_to_cpup(iprop);
	if (!num_laws) {
		ret = -EINVAL;
		goto error;
	}

	lac = of_iomap(np, 0);
	if (!lac) {
		ret = -ENODEV;
		goto error;
	}

	law = lac + LAW_OFFSET;

	/* Clear current LAW configuration */
	for (i = 0; i < num_laws; i++) {
		for (j = 0; j < ARRAY_SIZE(law_setup); j++) {
			if (law_setup[j].target_id == LAWAR_TARGET_ID(law[i].lawar)) {
				law[i].lawar = 0;
				law[i].lawbarh = 0;
				law[i].lawbarl = 0;
			}
		}
	}

	/* Setup new LAW configuration */
	for (j = 0; j < ARRAY_SIZE(law_setup); j++) {
		ret = law_setup[j].setup(law, num_laws, law_setup[j].target_id);
		if (ret)
			break;
	}

	iounmap(lac);
error:
	of_node_put(np);

	return ret;
}

#define CPCCSR0                    0x0
#define   CPCCSR0_E                0x80000000 /* CPC flash enable */
#define   CPCCSR0_PE               0x40000000 /* Enable ECC */
#define   CPCCSR0_FI               0x00200000 /* CPC flash invalidate */
#define   CPCCSR0_FC               0x00000400 /* CPC flash clear */
#define CPCCFG0                    0x8
#define   CPCCFG0_SZ_MASK          0x00003fff
#define   CPCCFG0_SZ_K(x)          (((x) & CPCCFG0_SZ_MASK) << 6)
#define CPCSRCR0                   0x100
#define   CPCSRCR0_SRAMEN          0x1
#define CPCHDBCR0                  0xf00
#define   CPCHDBCR0_SPLRU_LEVEL_EN 0x001e0000

/* Modifying CPC Control and Status Registers as described in T2080RM */
static inline void write_cpcsr0(volatile u32 *base, u32 val)
{
	__asm__ volatile(".equ CPCCSR0, 0 \n"
			 "mbar \n"
			 "isync \n"
			 "stw %1, CPCCSR0(%0) \n"
			 "lwz %1, CPCCSR0(%0) \n"
			 "mbar \n"
			 :
			 : "r" (base), "r" (val));
}

/* The CPC is disabled when starting from bootloader but is enabled when
 * starting from ppcmon.
 */
static int cpc_config(void)
{
	struct device_node *np;
	void __iomem *base;
	int ret = 0;
	volatile u32 *cpcsr0, *cpchdbcr0, *cpccfg0, *cpcsrcr0;

	np = of_find_compatible_node(NULL, NULL, "fsl,t2080-l3-cache-controller");
	if (!np)
		return -ENODEV;

	base = (struct cpc_corenet *)of_iomap(np, 0);
	if (!base) {
		ret = -ENODEV;
		goto error;
	}
	cpcsr0 = (volatile u32 *)(base + CPCCSR0);
	cpccfg0 = (volatile u32 *)(base + CPCCFG0);
	cpcsrcr0 = (volatile u32 *)(base + CPCSRCR0);
	cpchdbcr0 = (volatile u32 *)(base + CPCHDBCR0);

	if (*cpcsr0 & CPCCSR0_E)
		goto skip_config;

	/* clear and invalidate the cache */
	write_cpcsr0(base, CPCCSR0_FI | CPCCSR0_FC);

	/* wait clear and invalidate is done */
	while (*cpcsr0 & (CPCCSR0_FI | CPCCSR0_FC));

	/* errata A-006379 : CoreNet Platform Cache (CPC) CPCHDBCR0 register bit
	 *                   field [10:14] has incorrect default value after POR
	 *
	 * Set CPCHDBCR0 at offset 0xF00, bitfield [10:14] to 0x0F before
	 * enabling CPC or else mask CPCHDBCR0 register with 0x001E_0000
	 * before enabling CPC.
	 */
	*cpchdbcr0 |= CPCHDBCR0_SPLRU_LEVEL_EN;

	/* errata A-006593 : Atomic store may report failure but still
	 *                   allow the store data to be visible
	 *
	 * Set CoreNet Platform Cache register CPCHDBCR0 bit 21 to 1'b1.
	 * This may have a small impact on synthetic write bandwidth
	 * benchmarks but should have a negligible impact on real code.
	 */
	*cpchdbcr0 |= 1 << (31 - 21);

	/* enable CPC */
	write_cpcsr0(base, CPCCSR0_E | CPCCSR0_PE);

skip_config:
	pr_info("cpc %s with size %d kB, sram %d\n",
		(*cpcsr0 & CPCCSR0_E) ? "enabled" : "disabled",
		CPCCFG0_SZ_K(*cpccfg0), *cpcsrcr0 & CPCSRCR0_SRAMEN);
	iounmap(base);
error:
	of_node_put(np);
	return ret;
}

#include "smp.h"
#include "mpc85xx.h"
#include <asm/swiotlb.h>
#include <asm/machdep.h>
#include <linux/of_fdt.h>

void __init mfcc_8558_setup_arch(void)
{
	mpc85xx_smp_init();

	swiotlb_detect_4g();

	pr_info("%s board\n", ppc_md.name);

	if (law_config())
		pr_err("law_config failed\n");

	if (cpc_config())
		pr_err("cpc_config failed\n");

	mpc85xx_qe_init();
}



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
	.setup_arch             = mfcc_8558_setup_arch,
	.init_IRQ               = corenet_gen_pic_init,
#ifdef CONFIG_PCI
	.pcibios_fixup_bus      = fsl_pcibios_fixup_bus,
	.pcibios_fixup_phb      = mfcc_8558_fixup_pci_phb,
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
machine_arch_initcall(mfcc_8558, mpc85xx_common_publish_devices);
#endif
