/*
 * Copyright (C) 2017 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _XGS_IPROC_SERDES_H_
#define _XGS_IPROC_SERDES_H_

/* Specify the used lane number of SERDES */
typedef struct xgs_serdes_info_s {
	u32 lane;
} xgs_serdes_info_t;

extern bool xgs_serdes_hx4_amac(struct phy_device *);
extern bool xgs_serdes_kt2_amac(struct phy_device *);
extern void xgs_serdes_set_lane(struct phy_device *phy_dev, u32 lane);

#endif /* _XGS_IPROC_SERDES_H_ */
