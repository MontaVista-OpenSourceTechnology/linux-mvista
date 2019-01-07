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
 * Shift the entire hardware IRQ window by PUMA_IRQ_SHIFT. In Puma
 * the hardware irq window starts at 0; but linux don't
 * recognize hardware IRQ0. So lifting it.
 */
#define PUMA_IRQ_SHIFT	1
#define PUMA_IRQ(x)      (PUMA_IRQ_SHIFT + (x))
#define NR_IRQS	56

#define IRQ_PUMA_C0_RX_PULSE		PUMA_IRQ(0)
#define IRQ_PUMA_C0_TX_PULSE		PUMA_IRQ(1)
#define IRQ_PUMA_CCINTG			PUMA_IRQ(2)
#define IRQ_PUMA_CCERRINT		PUMA_IRQ(7)
#define TIMER1_INT_			PUMA_IRQ(12)
#define IRQ_TINT0_TINT12		PUMA_IRQ(13)
#define IRQ_TINT0_TINT34		PUMA_IRQ(14)
#define IRQ_TINT1_TINT12		PUMA_IRQ(15)
#define IRQ_TINT1_TINT34		PUMA_IRQ(16)
#define UART0_INT			PUMA_IRQ(17)
#define IRQ_I2C				PUMA_IRQ(18)
#define IRQ_PUMA_C0_RX_THRESH_PULSE	PUMA_IRQ(53)
#define IRQ_PUMA_C0_MISC_PULSE		PUMA_IRQ(54)

#endif
