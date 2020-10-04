/*
 * Helper module for board specific RAPIDIO bus registration
 *
 * Copyright (C) 2014 LSI Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */
#ifndef __ASM__ARCH_AXXIA_RAPIDIO_H
#define __ASM__ARCH_AXXIA_RAPIDIO_H
#include <linux/io.h>

#define IN_SRIO8(a, v, ec)      do {            \
	v = ioread8(a); ec = 0;  \
	} while (0)
#define IN_SRIO16(a, v, ec)     do {            \
	v = ioread16be(a); ec = 0;  \
	} while (0)
#define IN_SRIO32(a, v, ec)     do {            \
	v = ioread32be(a); ec = 0;  \
	} while (0)

#define OUT_SRIO8(a, v)         iowrite8(v, a)
#define OUT_SRIO16(a, v)        iowrite16be(v, a)
#define OUT_SRIO32(a, v)        iowrite32be(v, a)

int axxia_rapidio_board_init(struct platform_device *dev, int devnum,
							int *portndx);

#endif /* __ASM__ARCH_AXXIA_RAPIDIO_H */
