/*
 * Xylon logiCVC frame buffer driver internal data structures
 *
 * Copyright (C) 2016 Xylon d.o.o.
 * Author: Davor Joja <davor.joja@logicbricks.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef __XYLONFB_CORE_H__
#define __XYLONFB_CORE_H__

#include <linux/fb.h>
#include <linux/mutex.h>
#include <linux/wait.h>

#if defined(CONFIG_FB_XYLON_MISC)
#include "xylonfb_misc.h"
#endif

#define XYLONFB_DRIVER_NAME "xylonfb"
#define XYLONFB_DEVICE_NAME "logicvc"
#define XYLONFB_DRIVER_DESCRIPTION "Xylon logiCVC frame buffer driver"
#define XYLONFB_DRIVER_VERSION "4.3"

#define INFO		1
#define CORE		2
#define DEBUG_LEVEL	CORE

#ifdef DEBUG
#define XYLONFB_DBG(level, format, ...)				\
	do {							\
		if (level >= DEBUG_LEVEL)			\
			pr_info(format "\n", ## __VA_ARGS__);	\
	} while (0)
#else
#define XYLONFB_DBG(format, ...) do { } while (0)
#endif

#define LOGICVC_MAX_LAYERS	5

#define XYLONFB_EDID_SIZE	256
#define XYLONFB_EDID_WAIT_TOUT	60

/* Xylon FB driver flags */
#define XYLONFB_FLAGS_READABLE_REGS		(1 << 0)
#define XYLONFB_FLAGS_SIZE_POSITION		(1 << 1)
#define XYLONFB_FLAGS_BACKGROUND_LAYER		(1 << 2)
#define XYLONFB_FLAGS_BACKGROUND_LAYER_RGB	(1 << 3)
#define XYLONFB_FLAGS_BACKGROUND_LAYER_YUV	(1 << 4)
#define XYLONFB_FLAGS_DISPLAY_INTERFACE_ITU656	(1 << 5)
#define XYLONFB_FLAGS_DYNAMIC_LAYER_ADDRESS	(1 << 6)
#define XYLONFB_FLAGS_CHECK_CONSOLE_LAYER	(1 << 7)
#define XYLONFB_FLAGS_PIXCLK_VALID		(1 << 8)
#define XYLONFB_FLAGS_DMA_BUFFER		(1 << 9)
#define XYLONFB_FLAGS_EDID_VMODE		(1 << 10)
#define XYLONFB_FLAGS_EDID_PRINT		(1 << 11)
#define XYLONFB_FLAGS_EDID_READY		(1 << 12)
#define XYLONFB_FLAGS_VSYNC_IRQ			(1 << 13)
#define XYLONFB_FLAGS_VMODE_CUSTOM		(1 << 14)
#define XYLONFB_FLAGS_VMODE_INIT		(1 << 15)
#define XYLONFB_FLAGS_VMODE_DEFAULT		(1 << 16)
#define XYLONFB_FLAGS_VMODE_SET			(1 << 17)
#define XYLONFB_FLAGS_LAYER_ENABLED		(1 << 18)
#define XYLONFB_FLAGS_MISC_ADV7511		(1 << 19)
#define XYLONFB_FLAGS_ADV7511_SKIP		(1 << 20)
#define XYLONFB_FLAGS_ACTIVATE_NEXT_OPEN	(1 << 21)
#define XYLONFB_FLAGS_PUT_VSCREENINFO_EXACT	(1 << 22)

/* Xylon FB driver color formats */
enum xylonfb_color_format {
	XYLONFB_FORMAT_A8,
/* CLUT color formats */
	XYLONFB_FORMAT_C8,
	XYLONFB_FORMAT_CLUT_ARGB6565,
	XYLONFB_FORMAT_CLUT_ARGB8888,
	XYLONFB_FORMAT_CLUT_AYUV8888,
/* RGB color forrmats */
	XYLONFB_FORMAT_RGB332,
	XYLONFB_FORMAT_BGR233,
	XYLONFB_FORMAT_ARGB3332,
	XYLONFB_FORMAT_ABGR3233,
	XYLONFB_FORMAT_RGB565,
	XYLONFB_FORMAT_BGR565,
	XYLONFB_FORMAT_ARGB565,
	XYLONFB_FORMAT_ABGR565,
	XYLONFB_FORMAT_XRGB8888,
	XYLONFB_FORMAT_XBGR8888,
	XYLONFB_FORMAT_ARGB8888,
	XYLONFB_FORMAT_ABGR8888,
	XYLONFB_FORMAT_XRGB2101010,
	XYLONFB_FORMAT_XBGR2101010,
/* YUV color formats */
/* packed YCbCr 4:2:2, 32bit for 2 pixels. 8bit color component */
	XYLONFB_FORMAT_YUYV,
	XYLONFB_FORMAT_UYVY,
/* packed YCbCr 4:2:2, 64bit for 2 pixels, 10bit color component */
	XYLONFB_FORMAT_YUYV_121010,
	XYLONFB_FORMAT_UYVY_121010,
/* packed YCbCr 4:4:4, 32bit for 1 pixel, 8bit color component */
	XYLONFB_FORMAT_AYUV,
	XYLONFB_FORMAT_AVUY,
	XYLONFB_FORMAT_XYUV,
	XYLONFB_FORMAT_XVUY,
/* packed YCbCr 4:4:4, 32bit for 1 pixel, 10bit color component */
	XYLONFB_FORMAT_XYUV_2101010,
	XYLONFB_FORMAT_XVUY_2101010
};

struct xylonfb_layer_data;
struct xylonfb_data;

#define VMODE_NAME_SIZE	21
#define VMODE_OPTS_SIZE	3

struct xylonfb_vmode {
	u32 ctrl;
	struct fb_videomode vmode;
	char name[VMODE_NAME_SIZE];
	char opts_cvt[VMODE_OPTS_SIZE];
	char opts_ext[VMODE_OPTS_SIZE];
};

struct xylonfb_registers {
	u32 ctrl;
	u32 dtype;
	u32 bg;
	u32 unused[3];
	u32 int_mask;
};

union xylonfb_layer_reg_0 {
	u32 addr;
	u32 hoff;
};

union xylonfb_layer_reg_1 {
	u32 unused;
	u32 voff;
};

struct xylonfb_layer_registers {
	union xylonfb_layer_reg_0 reg_0;
	union xylonfb_layer_reg_1 reg_1;
	u32 hpos;
	u32 vpos;
	u32 hsize;
	u32 vsize;
	u32 alpha;
	u32 ctrl;
	u32 transp;
};

struct xylonfb_register_access {
	u32 (*get_reg_val)(void __iomem *dev_base, unsigned int offset,
			   struct xylonfb_layer_data *layer_data);
	void (*set_reg_val)(u32 value, void __iomem *dev_base,
			    unsigned int offset,
			    struct xylonfb_layer_data *layer_data);
};

struct xylonfb_layer_fix_data {
	unsigned int id;
	u32 address;
	u32 address_range;
	u32 buffer_offset;
	u32 bpp;
	u32 width;
	u32 height;
	u32 format;
	u32 format_clut;
	u32 transparency;
	u32 type;
	bool component_swap;
};

struct xylonfb_layer_data {
	struct mutex mutex;

	atomic_t refcount;

	struct xylonfb_data *data;
	struct xylonfb_layer_fix_data *fd;
	struct xylonfb_layer_registers regs;

	dma_addr_t pbase;
	void __iomem *base;
	void __iomem *clut_base;

	dma_addr_t fb_pbase;
	void *fb_base;
	u32 fb_size;

	dma_addr_t fb_pbase_active;

	u32 flags;
};

struct xylonfb_rgb2yuv_coeff {
	s32 cy;
	s32 cyr;
	s32 cyg;
	s32 cyb;
	s32 cur;
	s32 cug;
	s32 cub;
	s32 cvr;
	s32 cvg;
	s32 cvb;
};

struct xylonfb_sync {
	wait_queue_head_t wait;
	unsigned int count;
};

struct xylonfb_data {
	struct platform_device *pdev;

	struct device_node *device;
	struct device_node *pixel_clock;

	struct resource resource_mem;
	struct resource resource_irq;

	void __iomem *dev_base;

	struct mutex irq_mutex;

	struct xylonfb_register_access reg_access;
	struct xylonfb_sync vsync;
	struct xylonfb_vmode vm;
	struct xylonfb_vmode vm_active;
	struct xylonfb_rgb2yuv_coeff coeff;

	struct xylonfb_layer_fix_data *fd[LOGICVC_MAX_LAYERS];
	struct xylonfb_registers regs;
#if defined(CONFIG_FB_XYLON_MISC)
	struct xylonfb_misc_data misc;
#endif

	u32 bg_layer_bpp;
	u32 console_layer;
	u32 pixel_stride;

	atomic_t refcount;

	u32 flags;
	int irq;
	u8 layers;
	/*
	 * Delay after applying display power and
	 * before applying display signals
	 */
	u32 pwr_delay;
	/*
	 * Delay after applying display signal and
	 * before applying display backlight power supply
	 */
	u32 sig_delay;
	/* IP version */
	u8 major;
	u8 minor;
	u8 patch;
	u32 max_h_res;
	u32 max_v_res;
};

/* Xylon FB video mode options */
extern char *xylonfb_mode_option;

/* Xylon FB core pixel clock interface functions */
extern bool xylonfb_hw_pixclk_supported(struct device *dev,
					struct device_node *dn);
extern void xylonfb_hw_pixclk_unload(struct device_node *dn);
extern int xylonfb_hw_pixclk_set(struct device *dev, struct device_node *dn,
				 unsigned long pixclk_khz);

/* Xylon FB core V sync wait function */
extern int xylonfb_vsync_wait(u32 crt, struct fb_info *fbi);

/* Xylon FB core interface functions */
extern int xylonfb_init_core(struct xylonfb_data *data);
extern int xylonfb_deinit_core(struct platform_device *pdev);
extern int xylonfb_ioctl(struct fb_info *fbi, unsigned int cmd,
			 unsigned long arg);

#endif /* __XYLONFB_CORE_H__ */
