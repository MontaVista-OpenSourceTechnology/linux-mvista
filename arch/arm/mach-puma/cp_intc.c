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

/* Assert this define to force enable function (cp_intc_enable_irq) to include IRQ status clearing */
#undef CLEAR_SPURIOUS_IRQ_ON_ENABLE

static void __iomem *cp_intc_base;
#if defined(CLEAR_SPURIOUS_IRQ_ON_ENABLE)
static unsigned int irq_type[NR_IRQS] = {IRQ_TYPE_NONE};
#endif
static int irq_status[NR_IRQS] = {0};

static inline unsigned int cp_intc_read(unsigned int offset)
{
	return __raw_readl(cp_intc_base + offset);
}

static inline void cp_intc_write(unsigned long value, unsigned int offset)
{
	__raw_writel(value, cp_intc_base + offset);
}

static void cp_intc_ack_irq(struct irq_data *d)
{
	unsigned long reg, id, hwirq;

	/*
	 * The entire hardware IRQ window is shifted in software
	 * by PUMA_IRQ_SHIFT. So when talking to the hardware
	 * substract PUMA_IRQ_SHIFT from hwirq to get the
	 * actual hardware irq
	 */
	hwirq = d->hwirq - PUMA_IRQ_SHIFT;

	reg = BIT_WORD(hwirq);
	id  = hwirq;

	cp_intc_write(hwirq, CP_INTC_SYS_STAT_IDX_CLR);
	while (id >= BITS_PER_LONG)
		id -= BITS_PER_LONG;
	/* Write a 1 in a bit position to clear the status of the system interrupt. */
	/* Writing a 0 has no effect: so mask is useless */
	cp_intc_write((1 << id), CP_INTC_SYS_STAT_CLR(reg));
}

/* Disable interrupt */
static void cp_intc_mask_irq(struct irq_data *d)
{
	unsigned long hwirq;

	hwirq = d->hwirq - PUMA_IRQ_SHIFT;

	/* XXX don't know why we need to disable nIRQ here... */
	cp_intc_write(1, CP_INTC_HOST_ENABLE_IDX_CLR);
	cp_intc_write(hwirq, CP_INTC_SYS_ENABLE_IDX_CLR);
	cp_intc_write(1, CP_INTC_HOST_ENABLE_IDX_SET);
}

/* Enable interrupt */
static void cp_intc_unmask_irq(struct irq_data *d)
{
	unsigned long hwirq;

	hwirq = d->hwirq - PUMA_IRQ_SHIFT;

	if (!(irq_status[hwirq]))
		cp_intc_write(hwirq, CP_INTC_SYS_ENABLE_IDX_SET);
}

static void cp_intc_disable_irq(struct irq_data *d)
{
	unsigned long reg, id, hwirq;

	hwirq = d->hwirq - PUMA_IRQ_SHIFT;

	reg = BIT_WORD(hwirq);
	id  = hwirq;

	while (id >= BITS_PER_LONG)
		id -= BITS_PER_LONG;
	/* Write a 1 in a bit position to clear that enable. */
	/* Writing a 0 has no effect: so mask is useless */
	cp_intc_write((1 << id), CP_INTC_SYS_ENABLE_CLR(reg));

	cp_intc_mask_irq(d);

	/* Align IRQ status in order to match [disable / enable] couples */
	irq_status[hwirq]++;
}

static void cp_intc_enable_irq(struct irq_data *d)
{
	unsigned long reg, id, hwirq;

	hwirq = d->hwirq - PUMA_IRQ_SHIFT;

	reg = BIT_WORD(hwirq);
	id  = hwirq;
	/* No disable left: enable this IRQ */
	if ((!(irq_status[hwirq])) || ((irq_status[hwirq] > 0) &&
		(!(--irq_status[hwirq])))) {
		while (id >= BITS_PER_LONG)
			id -= BITS_PER_LONG;

#if defined(CLEAR_SPURIOUS_IRQ_ON_ENABLE)
		if (irq_type[irq] == (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW)) {
			/* Clear the irq spurious status */
			cp_intc_write((1 << id), CP_INTC_SYS_STAT_CLR(reg));
		}
#endif

		/* Write a 1 in a bit position to set that enable. */
		/* Writing a 0 has no effect: so mask is useless */
		cp_intc_write((1 << id), CP_INTC_SYS_ENABLE_SET(reg));

		cp_intc_unmask_irq(d);
	}
}

static int cp_intc_set_irq_type(struct irq_data *d, unsigned int flow_type)
{
	unsigned long reg, hwirq, mask, polarity, type, status;

	hwirq = d->hwirq - PUMA_IRQ_SHIFT;

	mask = BIT_MASK(hwirq);
	reg = BIT_WORD(hwirq);

	polarity	= cp_intc_read(CP_INTC_SYS_POLARITY(reg));
	type		= cp_intc_read(CP_INTC_SYS_TYPE(reg));
	status		= cp_intc_read(CP_INTC_SYS_RAW_STAT(reg));

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

	switch (flow_type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_EDGE_FALLING:
		irq_set_handler(d->hwirq, handle_edge_irq);
#if defined(CLEAR_SPURIOUS_IRQ_ON_ENABLE)
		irq_type[hwirq] = (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING);
#endif
		break;
	case IRQ_TYPE_LEVEL_HIGH:
	case IRQ_TYPE_LEVEL_LOW:
		irq_set_handler(d->hwirq, handle_level_irq);
#if defined(CLEAR_SPURIOUS_IRQ_ON_ENABLE)
		irq_type[hwirq] = (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW);
#endif
		break;
	}

	/* Check if the irq of which the type has been modified was previously already set */
	if (status & mask) {
		unsigned int id     = hwirq;

		/* Clear the irq spurious status */
		while (id >= BITS_PER_LONG)
			id -= BITS_PER_LONG;
		cp_intc_write((1 << id), CP_INTC_SYS_STAT_CLR(reg));
	}

	return 0;
}

/*
 * Faking this allows us to to work with suspend functions of
 * generic drivers which call {enable|disable}_irq_wake for
 * wake up interrupt sources (eg RTC on DA850).
 */
static int cp_intc_set_wake(struct irq_data *d, unsigned int on)
{
	return 0;
}

static struct irq_chip cp_intc_irq_chip = {
	.name		= "cp_intc",
	.irq_enable	= cp_intc_enable_irq,
	.irq_disable	= cp_intc_disable_irq,
	.irq_ack	= cp_intc_ack_irq,
	.irq_mask	= cp_intc_mask_irq,
	.irq_unmask	= cp_intc_unmask_irq,
	.irq_set_type	= cp_intc_set_irq_type,
	.irq_set_wake	= cp_intc_set_wake,
};


static struct irq_domain *cp_intc_domain;

static int cp_intc_host_map(struct irq_domain *h, unsigned int virq,
					irq_hw_number_t hw)
{
	pr_debug("cp_intc_host_map(%d, 0x%lx)\n", virq, hw);

	/* Set up genirq dispatching for cp_intc */
	irq_set_chip(virq, &cp_intc_irq_chip);
	irq_set_probe(virq);

	if ((hw == 35) || (hw == 36) || (hw == 37) || (hw == 38)) {
		irq_set_handler(hw, handle_level_irq);
#if defined(CLEAR_SPURIOUS_IRQ_ON_ENABLE)
		irq_type[i] = (IRQ_TYPE_LEVEL_HIGH | IRQ_TYPE_LEVEL_LOW);
#endif
	} else {
		irq_set_handler(hw, handle_edge_irq);
#if defined(CLEAR_SPURIOUS_IRQ_ON_ENABLE)
		irq_type[i] = (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING);
#endif
	}

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
	int i, irq_base;

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
	irq_base = irq_alloc_descs(-1, 0, num_irq, 0);
	if (irq_base < 0) {
		pr_warn("Couldn't allocate IRQ numbers\n");
		irq_base = 0;
	}

	/* create a legacy host */
	cp_intc_domain = irq_domain_add_legacy(node, num_irq,
				irq_base, 0, &cp_intc_host_ops, NULL);
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
