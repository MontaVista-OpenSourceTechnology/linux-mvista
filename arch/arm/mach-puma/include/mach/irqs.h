/*
 * Copyright (C) 2014 - 2019 MontaVista, Software, LLC.
 * Copyright (C) 2010 Wind River Systems, Inc.
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


#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

/*
 * puma1 top irq controller (cpintc) uses legacy irq domain that ignores kernel
 * irq number 0 (which means no irq). Therefore, we use PUMA_IRQ_SHIFT as a
 * fixed offset from hardware irq numbers to kernel irq numbers for cpintc.
 */
#define PUMA_IRQ_SHIFT 1

/* we don't use CONFIG_SPARSE_IRQ so all irq_descs have to be allocated
 * statically */
#define CPINTC_IRQS 56
#define TOP_IRQ_IRQS 32
#define FREJA2_IRQ1_IRQS 16
/* number of statically allocated irq_descs */
#define NR_IRQS (PUMA_IRQ_SHIFT + CPINTC_IRQS + TOP_IRQ_IRQS + FREJA2_IRQ1_IRQS)

#define IRQ_PUMA_C0_RX_PULSE            0
#define IRQ_PUMA_C0_TX_PULSE            1
#define IRQ_PUMA_CCINTG                 2
#define IRQ_PUMA_CCERRINT               7
#define TIMER1_INT_                     12
#define IRQ_TINT0_TINT12                13
#define IRQ_TINT0_TINT34                14
#define IRQ_TINT1_TINT12                15
#define IRQ_TINT1_TINT34                16
#define UART0_INT                       17
#define IRQ_I2C                         18
#define IRQ_PUMA_C0_RX_THRESH_PULSE     53
#define IRQ_PUMA_C0_MISC_PULSE          54

#endif
