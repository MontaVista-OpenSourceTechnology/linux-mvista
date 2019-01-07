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

#ifndef __ASM_ARCH_I2C_H
#define __ASM_ARCH_I2C_H

#include "io.h"
#include "irqs.h"

/* All frequencies are expressed in kHz */
struct PUMA_i2c_platform_data {
	unsigned int	bus_freq;	/* standard bus frequency */
	unsigned int	bus_delay;	/* transaction delay */
};

#endif /* __ASM_ARCH_I2C_H */
