/*
 * Board setup GIC routines for Ericsson PUMA-1 Board
 *
 * Copyright (C) 2010 Wind River Systems, Inc.
 * Copyright (C) 2014 - 2019 MontaVista Software, LLC.
 * (Modified by Niyas Ahamed Mydeen <nmydeen@mvista.com>
 * for MontaVista Software, LLC.)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/export.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <mach/cp_intc.h>

static void __iomem *cp_intc_base;

#define pr_regs(cond, irq, str)						\
	do {								\
		if (cond) {						\
			pr_devel("%lu: %s(%lu): " str "\n"		\
				 "raw       %#08x\n"			\
				 "st        %#08x\n"			\
				 "en        %#08x\n",			\
				 jiffies,				\
				 __func__,				\
				 irq,					\
				 cp_intc_read(				\
					 CP_INTC_SYS_RAW_STAT(		\
						 BIT_WORD(irq))),	\
				 cp_intc_read(				\
					 CP_INTC_SYS_STAT_CLR(		\
						 BIT_WORD(irq))),	\
				 cp_intc_read(				\
					 CP_INTC_SYS_ENABLE_SET(	\
						 BIT_WORD(irq))));	\
		}							\
	} while (false)

static inline uint cp_intc_read(size_t offset)
{
	return ioread32(cp_intc_base + offset);
}

static inline void cp_intc_write(uint value, size_t offset)
{
	iowrite32(value, cp_intc_base + offset);
}

static void cp_intc_ack(struct irq_data *d)
{
	cp_intc_write(d->hwirq, CP_INTC_SYS_STAT_IDX_CLR);
}

static void cp_intc_mask(struct irq_data *d)
{
	cp_intc_write(d->hwirq, CP_INTC_SYS_ENABLE_IDX_CLR);
}

static void cp_intc_unmask(struct irq_data *d)
{
	cp_intc_write(d->hwirq, CP_INTC_SYS_ENABLE_IDX_SET);
}

static int cp_intc_set_irq_type(struct irq_data *d, unsigned int flow_type)
{
	unsigned long reg, mask, polarity, type;

	mask = BIT_MASK(d->hwirq);
	reg = BIT_WORD(d->hwirq);

	polarity	= cp_intc_read(CP_INTC_SYS_POLARITY(reg));
	type		= cp_intc_read(CP_INTC_SYS_TYPE(reg));

	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
		polarity |= mask;
		type |= mask;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		polarity &= ~mask;
		type |= mask;
		break;
	case IRQ_TYPE_LEVEL_HIGH:
		polarity |= mask;
		type &= ~mask;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		polarity &= ~mask;
		type &= ~mask;
		break;
	default:
		return -EINVAL;
	}

	cp_intc_write(polarity, CP_INTC_SYS_POLARITY(reg));
	cp_intc_write(type, CP_INTC_SYS_TYPE(reg));

	cp_intc_ack(d);

	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
		irq_set_handler(d->irq, handle_edge_irq);
		break;
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		/* the status register holds the pending status so we can use
		 * the fasteoi handler that issues only a single
		 * end-of-interrupt call after the irq has been serviced to
		 * acknowledge the level interrupts */
		irq_set_handler(d->irq, handle_fasteoi_irq);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct irq_chip cp_intc_irq_chip = {
	.name		= "cp_intc",
	.irq_eoi	= cp_intc_ack,
	.irq_ack	= cp_intc_ack,
	.irq_mask	= cp_intc_mask,
	.irq_unmask	= cp_intc_unmask,
	.irq_set_type	= cp_intc_set_irq_type,
	.flags		= IRQCHIP_SKIP_SET_WAKE,
};


static struct irq_domain *cp_intc_domain;

static int cp_intc_host_map(struct irq_domain *h, unsigned int virq,
					irq_hw_number_t hw)
{
	pr_devel("cp_intc_host_map(%d, 0x%lx)\n", virq, hw);

	irq_set_chip(virq, &cp_intc_irq_chip);
	irq_set_probe(virq);

	return 0;
}

static const struct irq_domain_ops cp_intc_host_ops = {
	.map = cp_intc_host_map,
	.xlate = irq_domain_xlate_onetwocell,
};

int __init cp_intc_init(void __iomem *base, unsigned short num_irq,
					struct device_node *node)
{
	unsigned int num_reg	= BITS_TO_LONGS(num_irq);
	int i;

	cp_intc_base = base;

	cp_intc_write(0, CP_INTC_GLOBAL_ENABLE);

	/* Disable all host interrupts */
	cp_intc_write(0, CP_INTC_HOST_ENABLE(0));

	/* Disable system interrupts */
	for (i = 0; i < num_reg; i++)
		cp_intc_write(~0, CP_INTC_SYS_ENABLE_CLR(i));

	/* Set to normal mode, no nesting, no priority hold */
	cp_intc_write(0, CP_INTC_CTRL);
	cp_intc_write(0, CP_INTC_HOST_CTRL);

	/* Clear system interrupt status */
	for (i = 0; i < num_reg; i++)
		cp_intc_write(~0, CP_INTC_SYS_STAT_CLR(i));

	/* Enable nIRQ (what about nFIQ?) */
	cp_intc_write(1, CP_INTC_HOST_ENABLE_IDX_SET);

	/*
	 * Priority is determined by host channel: lower channel number has
	 * higher priority i.e. channel 0 has highest priority and channel 31
	 * had the lowest priority.
	 */
	num_reg = (num_irq + 3) >> 2;	/* 4 channels per register */

	/*
	 * Configure Host Interrupt Map Register
	 * Map all channel interrupts to IRQ of CPU
	 */
	for (i = 0; i < num_reg; i++)
		cp_intc_write(0x1, CP_INTC_HOST_MAP(i));

	cp_intc_domain = irq_domain_add_legacy(node, num_irq, PUMA_IRQ_SHIFT, 0,
					       &cp_intc_host_ops, NULL);
	if (!cp_intc_domain) {
		pr_err("cp_intc: failed to allocate irq host!\n");
		return -EINVAL;
	}

	/* Enable global interrupt */
	cp_intc_write(1, CP_INTC_GLOBAL_ENABLE);

	return 0;
}
#ifdef CONFIG_MACH_SCMB_DT
int __init cp_intc_of_init(struct device_node *node,
		struct device_node *parent)
{
	u32 num_irq;

	if (node) {
		cp_intc_base = of_iomap(node, 0);
		if (of_property_read_u32(node,
				"ti,intc-size", &num_irq))
			pr_warn("no intc-size, default	to %d\n", num_irq);
	}
	if (WARN_ON(!cp_intc_base))
		return -EINVAL;
	return cp_intc_init(cp_intc_base, num_irq, node);
}
#endif
