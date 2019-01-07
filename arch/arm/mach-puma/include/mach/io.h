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


#ifndef __ASM_ARM_ARCH_PUMA_IO_H
#define __ASM_ARM_ARCH_PUMA_IO_H

#define IO_PHYS                 0x60000000

#define PUMA_INTC_BASE         (IO_PHYS + 0x08000000)
#define PUMA_UART0_BASE        (IO_PHYS + 0x09000000)
#define PUMA_TIMER1_BASE       (IO_PHYS + 0x05000000)
#define PUMA_TIMER2_BASE       (IO_PHYS + 0x06000000)
#define PUMA_TIMERWD_BASE      (IO_PHYS + 0x07000000)
#define PUMA_CPGEMACSS_BASE    (IO_PHYS + 0x03000000)
#define PUMA_CPGEMAC_BASE      (IO_PHYS + 0x03100000)
#define PUMA_MDIO_BASE         (IO_PHYS + 0x03200000)
#define PUMA_CPPI_BASE         (IO_PHYS + 0x03300000)
#define PUMA_USIM_BASE         (IO_PHYS + 0x0c000000)
#define PUMA_I2C_BASE          (IO_PHYS + 0x0a000000)
#define PUMA_DMA_3PCC_BASE     (IO_PHYS + 0x01000000)
#define PUMA_DMA_3PTC_BASE     (IO_PHYS + 0x02000000)

#define IO_OFFSET               0x90000000
#define IO_VIRT                 (IO_PHYS + IO_OFFSET)

#define io_p2v(pa)              ((pa) + IO_OFFSET)
#define io_v2p(va)              ((va) - IO_OFFSET)

#define __IO_ADDRESS(x)         ((x) + IO_OFFSET)
#define IO_ADDRESS(pa)          IOMEM(__IO_ADDRESS(pa))


#define PUMA_INTC_VIRT         IO_ADDRESS(PUMA_INTC_BASE)
#define PUMA_TIMER1_VIRT       IO_ADDRESS(PUMA_TIMER1_BASE)
#define PUMA_TIMER2_VIRT       IO_ADDRESS(PUMA_TIMER2_BASE)
#define PUMA_TIMERWD_VIRT      IO_ADDRESS(PUMA_TIMERWD_BASE)

#define PUMA_CPGEMAC_VIRT      IO_ADDRESS(PUMA_CPGEMAC_BASE)
#define PUMA_CPGEMACSS_VIRT    IO_ADDRESS(PUMA_CPGEMACSS_BASE)
#define PUMA_MDIO_VIRT         IO_ADDRESS(PUMA_MDIO_BASE)
#define PUMA_CPPI_VIRT         IO_ADDRESS(PUMA_CPPI_BASE)

#define PUMA_UART0_VIRT        IO_ADDRESS(PUMA_UART0_BASE)
#define PUMA_USIM_VIRT         IO_ADDRESS(PUMA_USIM_BASE)

#define PUMA_DMA_3PCC_VIRT     IO_ADDRESS(PUMA_DMA_3PCC_BASE)
#define PUMA_DMA_3PTC_VIRT     IO_ADDRESS(PUMA_DMA_3PTC_BASE)


/*
 * We don't actually have real ISA nor PCI buses, but there is so many
 * drivers out there that might just work if we fake them...
 */
#define __mem_pci(a)            (a)
#define __mem_isa(a)            (a)

#endif
