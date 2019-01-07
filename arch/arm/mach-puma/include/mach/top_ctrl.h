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

#ifndef __ASM_ARCH_TOP_CTRL_H
#define __ASM_ARCH_TOP_CTRL_H

#define TOP_CTRL_ASYNC3_CFG     0x00600305

#define PUMA_TOP_CTRL_BASE      0x6F000000
#define REG_TOP_TOPSTATUS       0x48

#define BIT_TOPSTATUS_SPEED_S   3
#define BIT_TOPSTATUS_SPEED_M   0x0008
#define BIT_TOPSTATUS_SPEED_800 1
#define BIT_TOPSTATUS_SPEED_600 0

#endif /* __ASM_ARCH_TOP_CTRL_H */
