/*
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

#include <mach/io.h>
#include <linux/serial_reg.h>
#define PUMA1_UART_REG_PHYS_BASE PUMA_UART0_BASE
#define PUMA1_UART_REG_VIRT_BASE  PUMA_UART0_VIRT

		.macro  addruart, rp, rv, tmp
		ldr     \rp, =PUMA1_UART_REG_PHYS_BASE
		ldr     \rv, =PUMA1_UART_REG_VIRT_BASE
		.endm

#define UART_SHIFT      2
                .macro  senduart,rd,rx
                strb    \rd, [\rx, #UART_TX << UART_SHIFT]
                .endm

                .macro  busyuart,rd,rx
1002:           ldrb    \rd, [\rx, #UART_LSR << UART_SHIFT]
                and     \rd, \rd, #UART_LSR_TEMT | UART_LSR_THRE
                teq     \rd, #UART_LSR_TEMT | UART_LSR_THRE
                bne     1002b
                .endm

                .macro  waituart,rd,rx
#ifdef FLOW_CONTROL
1001:           ldrb    \rd, [\rx, #UART_MSR << UART_SHIFT]
                tst     \rd, #UART_MSR_CTS
                beq     1001b
#endif
                .endm
