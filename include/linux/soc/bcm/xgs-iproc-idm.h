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

#ifndef XGS_IPROC_IDM_H
#define XGS_IPROC_IDM_H

extern void inline __iomem *get_iproc_idm_base(int idx);
extern int xgs_iproc_idm_dmac_reset(void);
#ifdef CONFIG_ML66_NPU_IPROC_PLATFORM
extern int xgs_iproc_idm_pcie_reset(void);
#endif
extern int xgs_iproc_idm_init(void);

#endif /* XGS_IPROC_IDM_H */
