/*
 * Copyright (C) 2016 Broadcom Corporation
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

#ifndef XGS_IPROC_MISC_SETUP_H
#define XGS_IPROC_MISC_SETUP_H

extern int xgs_iproc_misc_setup(void);
extern void __iomem *get_iproc_wrap_ctrl_base(void);
extern void __iomem *get_iproc_dmu_pcu_base(void);
extern int is_wh2_amac_sgmii(void);

#endif /* XGS_IPROC_MISC_SETUP_H */
