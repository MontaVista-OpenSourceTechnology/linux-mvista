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

#include "io.h"
#include "irqs.h"

                .macro  disable_fiq
                .endm

                .macro  get_irqnr_preamble, base, tmp
                ldr \base, =PUMA_INTC_VIRT
                .endm

                .macro  arch_ret_to_user, tmp1, tmp2
                .endm

                .macro  get_irqnr_and_base, irqnr, irqstat, base, tmp

1001:           ldr \irqnr, [\base, #0x80] /* get irq number */
                and \irqnr, \irqnr, #0xff  /* irq is in bits 0-9 */
                mov \tmp, \irqnr, lsr #3
                and \tmp, \tmp, #0xfc
                add \tmp, \tmp, #0x280 /* get the register offset */
                ldr \irqstat, [\base, \tmp] /* get the intc status */
                add \irqnr, \irqnr, #1
                cmp \irqstat, #0x0

1002:
                .endm

                .macro  irq_prio_table
                .endm
