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

#ifndef __ASM_ARCH_EMIF_H
#define __ASM_ARCH_EMIF_H
#include <linux/types.h>

/* EMIF Base Addresses */
#define PUMA_EMIF2_5_1_BASE  (0x81000000)
#define PUMA_EMIF2_5_2_BASE  (0x82000000)
#define PUMA_EMIF2_5_3_BASE  (0x83000000)
#define PUMA_EMIF2_5_4_BASE  (0x84000000)

#define PUMA_EMIF2_5_SIZE    (0x80000)

/* Pre-existing defines */
#define PUMA_EMIF4_BASE (0x84000000)
#define REG_EMIF_ASYNC3 (0x18)

#endif /* __ASM_ARCH_EMIF_H */
