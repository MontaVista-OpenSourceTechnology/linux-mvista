/*
 * Copyright (C) 2014 - 2019 MontaVista, Software, LLC.
 * Copyright (C) 2011 Ericsson
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

#include <mach/timex.h>
#include <mach/top_ctrl.h>
#include <mach/emif.h>
#include <asm/io.h>
#include <linux/string.h>
#include <linux/kernel.h>

u32 puma_clk_tick_rate;
char clk_str[8] = "UNKNOWN";

void PUMA_init_clock_tick_rate(void)
{
	u32 *emif4_async3;
	u16 *top_ctrl_status;

	/* init top cs */
	emif4_async3 = (u32 *) ioremap(PUMA_EMIF4_BASE + REG_EMIF_ASYNC3, sizeof(u32));

	*emif4_async3 = TOP_CTRL_ASYNC3_CFG;

	iounmap(emif4_async3);

	/* read status from top */
	top_ctrl_status = (u16 *) ioremap(PUMA_TOP_CTRL_BASE + REG_TOP_TOPSTATUS, sizeof(u16));

	if (((*top_ctrl_status & BIT_TOPSTATUS_SPEED_M) >> BIT_TOPSTATUS_SPEED_S) == BIT_TOPSTATUS_SPEED_600) {
		puma_clk_tick_rate = PUMA_CLK_LOW;
		strcpy(clk_str, "LOW");
	} else {
		puma_clk_tick_rate = PUMA_CLK_HIGH;
		strcpy(clk_str, "HIGH");
	}

	iounmap(top_ctrl_status);
}

u32 get_clock_tick_rate(void)
{
	return puma_clk_tick_rate;
}

inline char *get_puma_clock_string(void)
{
	return clk_str;
}
