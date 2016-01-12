/* SPDX-License-Identifier: GPL-2.0-only */
/*
 *
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 */

#ifndef __FSL_PAMU_STASH_H
#define __FSL_PAMU_STASH_H

/* Define operation mapping indexes */
enum omap_index {
	OMI_QMAN,
	OMI_FMAN,
	OMI_QMAN_PRIV,
	OMI_CAAM,
	OMI_PMAN,
	OMI_DSP,
	OMI_MAX,
};

/* cache stash targets */
enum pamu_stash_target {
	PAMU_ATTR_CACHE_L1 = 1,
	PAMU_ATTR_CACHE_L2,
	PAMU_ATTR_CACHE_L3,
	PAMU_ATTR_CACHE_DSP_L2,
};

/*
 * This attribute allows configuring stashig specific parameters
 * in the PAMU hardware.
 */

struct pamu_stash_attribute {
	u32	cpu;	/* cpu number */
	u32	cache;	/* cache to stash to: L1,L2,L3 */
	u32	window; /* ~0 indicates all windows */
};

struct pamu_omi_attribute {
	u32	omi;	/* index in the operation mapping table */
	u32	window;	/* ~0 indicates all windows */
};

#endif  /* __FSL_PAMU_STASH_H */
