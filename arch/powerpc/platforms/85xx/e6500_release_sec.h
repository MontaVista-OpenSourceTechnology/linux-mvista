/*
 * e6500 secondary cores release
 *
 * Initalize all secondary cores and keep them in the spin loop.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
#ifndef __E6500_RELEASE_SEC_H__
#define __E6500_RELEASE_SEC_H__

#define E6500_CPU_NUM                 12

#define CCSRBAR                       0xffe000000UL

#define BOOT_PAGE                     0xfffff000UL

#define BOOT_ENTRY_ADDR_UPPER         0
#define BOOT_ENTRY_ADDR_LOWER         4
#define BOOT_ENTRY_R3_UPPER           8
#define BOOT_ENTRY_R3_LOWER           12
#define BOOT_ENTRY_RESV               16
#define BOOT_ENTRY_PIR	              20
#define BOOT_ENTRY_R6_UPPER           24
#define BOOT_ENTRY_R6_LOWER           28

#define BOOT_ENTRY_SIZE               64

#endif
