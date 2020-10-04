/*
 * Syscom ethernet type
 *
 * It must be defined by the kernel, because it's used by the RIO driver.
 *
 * Author: Petr Malat
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef IF_SYSCOM_ETHER_H
#define IF_SYSCOM_ETHER_H

#define ETH_P_SYSCOM      0x0F00
#define ETH_P_SYSCOM_FRAG 0x0F01

#endif // IF_SYSCOM_ETHER_H
