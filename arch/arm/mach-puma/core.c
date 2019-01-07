/*
 * Board setup routines for Ericsson PUMA-1 Board
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

#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/serial_core.h>

#include <asm/mach/map.h>
#include <asm/memory.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <asm/tlb.h>

#include <mach/io.h>
#include <mach/emac.h>
#include <mach/irqs.h>
#include <mach/i2c.h>
#include <mach/edma.h>
#include <mach/board.h>
#include <mach/timex.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>

extern void __init PUMA_timer_init(void);
/*-------------------------------------------------------------------------*/
/*-                           io mapping                                  -*/
/*-------------------------------------------------------------------------*/

static struct map_desc PUMA_io_desc[] __initdata = {
	{
		.virtual = (unsigned long)IO_ADDRESS(PUMA_UART0_BASE),
		.pfn     = __phys_to_pfn(PUMA_UART0_BASE),
		.length  = 0x2000,
		.type    = MT_DEVICE
	},
	{
		.virtual = (unsigned long)PUMA_INTC_VIRT,
		.pfn     = __phys_to_pfn(PUMA_INTC_BASE),
		.length  = 0x2000,
		.type    = MT_DEVICE
	},

	{
		.virtual = (unsigned long)PUMA_TIMER1_VIRT,
		.pfn     = __phys_to_pfn(PUMA_TIMER1_BASE),
		.length  = 0x1000,
		.type    = MT_DEVICE
	},

	{
		.virtual = (unsigned long)PUMA_TIMER2_VIRT,
		.pfn     = __phys_to_pfn(PUMA_TIMER2_BASE),
		.length  = 0x1000,
		.type    = MT_DEVICE
	},

	{
		.virtual = (unsigned long)PUMA_DMA_3PCC_VIRT,
		.pfn     = __phys_to_pfn(PUMA_DMA_3PCC_BASE),
		.length  = 0x10000,
		.type    = MT_DEVICE
	},

	{
		.virtual = (unsigned long)PUMA_DMA_3PTC_VIRT,
		.pfn     = __phys_to_pfn(PUMA_DMA_3PTC_BASE),
		.length  = 0x10000,
		.type    = MT_DEVICE
	}
};

void __init PUMA_map_io(void)
{
	/*
	 * Map the UARTs early so that the DEBUG_LL stuff continues to work.
	 */
	debug_ll_io_init();
	iotable_init(PUMA_io_desc, ARRAY_SIZE(PUMA_io_desc));
}

/*-------------------------------------------------------------------------*/
/*-                           network                                     -*/
/*-------------------------------------------------------------------------*/

struct resource PUMA_emac_resources[] = {
	{
		.start  = PUMA_CPGEMACSS_BASE,
		.end    = (PUMA_CPPI_BASE + SZ_8K),
		.flags  = IORESOURCE_MEM
	},

	{
		.start  = IRQ_PUMA_C0_RX_THRESH_PULSE,
		.end    = IRQ_PUMA_C0_RX_THRESH_PULSE,
		.flags  = IORESOURCE_IRQ
	},

	{
		.start  = IRQ_PUMA_C0_RX_PULSE,
		.end    = IRQ_PUMA_C0_RX_PULSE,
		.flags  = IORESOURCE_IRQ
	},

	{
		.start  = IRQ_PUMA_C0_TX_PULSE,
		.end    = IRQ_PUMA_C0_TX_PULSE,
		.flags  = IORESOURCE_IRQ
	},

	{
		.start  = IRQ_PUMA_C0_MISC_PULSE,
		.end    = IRQ_PUMA_C0_MISC_PULSE,
		.flags  = IORESOURCE_IRQ
	}
};

struct emac_platform_data PUMA_emac_pdata = {
	.ctrl_reg_offset     = PUMA_EMAC_CTRL_REG_OFFSET,
	.ctrl_mod_reg_offset = PUMA_EMAC_MOD_REG_OFFSET,
	.mdio_reg_offset     = PUMA_MDIO_REG_OFFSET,
	.ctrl_ram_size       = PUMA_EMAC_CTRL_RAM_SIZE,
	.ctrl_ram_offset     = PUMA_EMAC_RAM_OFFSET,
	.rmii_en             = 1,
	.version             = EMAC_VERSION_2
};

struct platform_device PUMA_emac_device = {
	.name           = "CPGMAC",
	.id             = 1,
	.dev = {
		.platform_data  = &PUMA_emac_pdata
	},

	.num_resources  = ARRAY_SIZE(PUMA_emac_resources),
	.resource       = PUMA_emac_resources
};

void __init PUMA_init_emac(struct emac_platform_data *pdata)
{
	board_init_phy(pdata);

	PUMA_emac_device.dev.platform_data = pdata;

	(void)platform_device_register(&PUMA_emac_device);
}

/*-------------------------------------------------------------------------*/
/*-                           i2c bus                                     -*/
/*-------------------------------------------------------------------------*/

static struct resource i2c_resources[] = {
	{
		.start = PUMA_I2C_BASE,
		.end   = (PUMA_I2C_BASE + 0x40),
		.flags = IORESOURCE_MEM
	},

	{
		.start = IRQ_I2C,
		.flags = IORESOURCE_IRQ
	}
};

static struct platform_device PUMA_i2c_device = {
	.name          = "i2c_PUMA",
	.id            = 1,
	.num_resources = ARRAY_SIZE(i2c_resources),
	.resource      = i2c_resources
};

static struct PUMA_i2c_platform_data i2c_pdata = {
	.bus_freq   = 150 /* kHz */,
	.bus_delay  = 20 /* usec */
};

void __init PUMA_init_i2c(struct PUMA_i2c_platform_data *pdata)
{
	PUMA_i2c_device.dev.platform_data = pdata;

	(void)platform_device_register(&PUMA_i2c_device);

	board_init_i2c();
}

/*-------------------------------------------------------------------------*/
/*-                           EDMA3                                       -*/
/*-------------------------------------------------------------------------*/

static struct resource PUMA_edma_resources[] = {
	{
		.name  = "edma_cc0",
		.start = PUMA_DMA_3PCC_BASE,
		.end   = (PUMA_DMA_3PCC_BASE + SZ_64K - 1),
		.flags = IORESOURCE_MEM
	},

	{
		.name  = "edma_tc0",
		.start = PUMA_DMA_3PTC_BASE,
		.end   = (PUMA_DMA_3PTC_BASE + SZ_1K - 1),
		.flags = IORESOURCE_MEM
	},

	{
		.name  = "edma0",
		.start = IRQ_PUMA_CCINTG,
		.flags = IORESOURCE_IRQ
	},

	{
	.name  = "edma0_err",
	.start = IRQ_PUMA_CCERRINT,
	.flags = IORESOURCE_IRQ
	}
};

static const s8 PUMA_queue_tc_mapping[][2] = {
	/* {event queue no, TC no} */
	{0, 0}
};

static const s8 PUMA_queue_priority_mapping[][2] = {
	/* {event queue no, Priority} */
	{0, 3}
};

static struct edma_soc_info PUMA_edma_info[] = {
	{
		.n_channel              = 8,
		.n_region               = 4,
		.n_slot                 = 16,
		.n_tc                   = 1,
		.n_cc                   = 1,
		.queue_tc_mapping       = PUMA_queue_tc_mapping,
		.queue_priority_mapping = PUMA_queue_priority_mapping
	}
};

static struct platform_device PUMA_edma_device = {
	.name           = "edma",
	.id             = 0,
	.dev            = {
		.platform_data = PUMA_edma_info
	},

	.num_resources  = ARRAY_SIZE(PUMA_edma_resources),
	.resource       = PUMA_edma_resources
};

/*-------------------------------------------------------------------------*/
/*-                           Watchdog                                    -*/
/*-------------------------------------------------------------------------*/

static struct resource wdt_resources[] = {
	{
		.start = PUMA_TIMERWD_BASE,
		.end   = (PUMA_TIMERWD_BASE + SZ_1K - 1),
		.flags = IORESOURCE_MEM
	}
};

struct platform_device PUMA_wdt_device = {
	.name          = "watchdog",
	.id            = -1,
	.num_resources = ARRAY_SIZE(wdt_resources),
	.resource      = wdt_resources
};

/*-------------------------------------------------------------------------*/
/*-                           SERIAL                                      -*/
/*-------------------------------------------------------------------------*/

static struct plat_serial8250_port PUMA_uart0_data[2];
static struct resource PUMA_uart0_resources[2];
static struct platform_device PUMA_uart0;
extern u32 get_puma_clock_tick_rate(void);

static void __init PUMA_init_serial(void)
{
	PUMA_uart0_data[0].mapbase    = PUMA_UART0_BASE;
	PUMA_uart0_data[0].membase    = (char *)PUMA_UART0_VIRT;
	PUMA_uart0_data[0].irq        = UART0_INT;
	PUMA_uart0_data[0].flags      = UPF_BOOT_AUTOCONF;
	PUMA_uart0_data[0].iotype     = UPIO_MEM;
	PUMA_uart0_data[0].regshift   = 2;
	PUMA_uart0_data[0].uartclk    = CLOCK_TICK_RATE;

	PUMA_uart0_resources[0].start = PUMA_UART0_BASE;
	PUMA_uart0_resources[0].end   = (PUMA_UART0_BASE + 0xFFFFF);
	PUMA_uart0_resources[0].flags = IORESOURCE_MEM;

	PUMA_uart0_resources[1].start = UART0_INT;
	PUMA_uart0_resources[1].end   = UART0_INT;
	PUMA_uart0_resources[1].flags = IORESOURCE_IRQ;

	PUMA_uart0.name               = "serial8250";
	PUMA_uart0.id                 = PLAT8250_DEV_PLATFORM;
	PUMA_uart0.dev.platform_data  = PUMA_uart0_data;
	PUMA_uart0.resource           = PUMA_uart0_resources;
	PUMA_uart0.num_resources      = ARRAY_SIZE(PUMA_uart0_resources);
}

/*-------------------------------------------------------------------------*/
/*-                           MAIN                                        -*/
/*-------------------------------------------------------------------------*/

extern u32 get_clock_tick_rate(void);
extern void PUMA_init_clock_tick_rate(void);
extern void __init cp_intc_init(void __iomem *base,
		unsigned short num_irq, struct device_node *node);

void __init PUMA_init_irq(void)
{
	cp_intc_init((void __iomem *)PUMA_INTC_VIRT, NR_IRQS, NULL);
}

void __init PUMA_init(void)
{
	local_flush_tlb_all();
	flush_cache_all();

	PUMA_init_clock_tick_rate();

	PUMA_init_i2c(&i2c_pdata);

	PUMA_init_serial();

	platform_device_register(&PUMA_uart0);

	printk(KERN_DEBUG "PUMA ASIC SPEED: %d (expected %d)\n", get_clock_tick_rate(), CLOCK_TICK_RATE);

	platform_device_register(&PUMA_wdt_device);

	platform_device_register(&PUMA_edma_device);

	PUMA_init_emac(&PUMA_emac_pdata);
}
