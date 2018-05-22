/*
 * Xylon logiCVC frame buffer driver core functions
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

#include <linux/console.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/videodev2.h>

#include "xylonfb_core.h"
#include "logicvc.h"

#define LOGICVC_PIX_FMT_AYUV			v4l2_fourcc('A', 'Y', 'U', 'V')
#define LOGICVC_PIX_FMT_AVUY			v4l2_fourcc('A', 'V', 'U', 'Y')
#define LOGICVC_PIX_FMT_ALPHA			v4l2_fourcc('A', '8', ' ', ' ')
/* Framebuffer v4 added YUV formats */
#define LOGICVC_PIX_FMT_XYUV			v4l2_fourcc('X', 'Y', 'U', 'V')
#define LOGICVC_PIX_FMT_XVUY			v4l2_fourcc('X', 'V', 'U', 'Y')
#define LOGICVC_PIX_FMT_YUYV_121010		v4l2_fourcc('Y', 'U', '2', '0')
#define LOGICVC_PIX_FMT_UYVY_121010		v4l2_fourcc('U', 'Y', '2', '0')
#define LOGICVC_PIX_FMT_XYUV_2101010	v4l2_fourcc('X', 'Y', '3', '0')
#define LOGICVC_PIX_FMT_XVUY_2101010	v4l2_fourcc('X', 'V', '3', '0')

#define XYLONFB_PSEUDO_PALETTE_SIZE	256
#define XYLONFB_VRES_DEFAULT		1080

#define LOGICVC_COLOR_RGB_BLACK		0
#define LOGICVC_COLOR_RGB_WHITE		0xFFFFFF
#define LOGICVC_COLOR_YUV888_BLACK	0x8080
#define LOGICVC_COLOR_YUV888_WHITE	0xFF8080

char *xylonfb_mode_option;

static const struct xylonfb_vmode xylonfb_vm = {
	.vmode = {
		.refresh = 60,
		.xres = 1024,
		.yres = 768,
		.pixclock = KHZ2PICOS(65000),
		.left_margin = 160,
		.right_margin = 24,
		.upper_margin = 29,
		.lower_margin = 3,
		.hsync_len = 136,
		.vsync_len = 6,
		.vmode = FB_VMODE_NONINTERLACED
	},
	.name = "1024x768"
};

static int xylonfb_set_timings(struct fb_info *fbi, int bpp);
static void xylonfb_logicvc_disp_ctrl(struct fb_info *fbi, bool enable);
static void xylonfb_enable_logicvc_output(struct fb_info *fbi);
static void xylonfb_disable_logicvc_output(struct fb_info *fbi);
static void xylonfb_logicvc_layer_enable(struct fb_info *fbi, bool enable);
static void xylonfb_fbi_update(struct fb_info *fbi);

static unsigned long xylonfb_get_reg_mem_addr(void __iomem *base,
					      unsigned int offset,
					      struct xylonfb_layer_data *ld)
{
	unsigned int ordinal = offset / LOGICVC_REG_STRIDE;

	if ((unsigned long)base - (unsigned long)ld->data->dev_base)
	{
		return (unsigned long)(&ld->regs) + (ordinal * sizeof(u32));
	}
	else
	{
		ordinal -= (LOGICVC_CTRL_ROFF / LOGICVC_REG_STRIDE);
		return (unsigned long)(&ld->data->regs) +
				       (ordinal * sizeof(u32));
	}
}

static u32 xylonfb_get_reg(void __iomem *base, unsigned int offset,
			   struct xylonfb_layer_data *ld)
{
	u32 value =  readl(base + offset);

	unsigned long *reg_mem_addr = (unsigned long *)xylonfb_get_reg_mem_addr(base, offset, ld);
	*reg_mem_addr = value;

	return value;
}

static void xylonfb_set_reg(u32 value, void __iomem *base, unsigned int offset,
			    struct xylonfb_layer_data *ld)
{
	unsigned long *reg_mem_addr = (unsigned long *)xylonfb_get_reg_mem_addr(base, offset, ld);
	*reg_mem_addr = value;
	writel(value, (base + offset));
}

static u32 xylonfb_get_reg_mem(void __iomem *base, unsigned int offset,
			       struct xylonfb_layer_data *ld)
{
	return *((unsigned long *)xylonfb_get_reg_mem_addr(base, offset, ld));
}

static void xylonfb_set_reg_mem(u32 value, void __iomem *base,
				unsigned int offset,
				struct xylonfb_layer_data *ld)
{
	unsigned long *reg_mem_addr =
		(unsigned long *)xylonfb_get_reg_mem_addr(base, offset, ld);
	*reg_mem_addr = value;
	writel((*reg_mem_addr), (base + offset));
}

static irqreturn_t xylonfb_isr(int irq, void *dev_id)
{
	struct fb_info **afbi = dev_get_drvdata(dev_id);
	struct fb_info *fbi = afbi[0];
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	void __iomem *dev_base = data->dev_base;
	u32 isr;

	isr = readl(dev_base + LOGICVC_INT_STAT_ROFF);
	if (isr & LOGICVC_INT_V_SYNC) {
		writel(LOGICVC_INT_V_SYNC, dev_base + LOGICVC_INT_STAT_ROFF);

		data->vsync.count++;

		if (waitqueue_active(&data->vsync.wait))
			wake_up_interruptible(&data->vsync.wait);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int xylonfb_open(struct fb_info *fbi, int user)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	int ret;
	bool enable = false;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (atomic_read(&ld->refcount) == 0) {
		if (ld->flags & XYLONFB_FLAGS_ACTIVATE_NEXT_OPEN) {
			ld->flags &= ~XYLONFB_FLAGS_ACTIVATE_NEXT_OPEN;
			enable = true;
		} else {
			if (fbi->var.activate == FB_ACTIVATE_NOW) {
				enable = true;
			} else if (fbi->var.activate == FB_ACTIVATE_NXTOPEN) {
				ld->flags |= XYLONFB_FLAGS_ACTIVATE_NEXT_OPEN;
				return 0;
			} else if (fbi->var.activate == FB_ACTIVATE_VBL) {
				ret = xylonfb_vsync_wait(0, fbi);
				if (ret > 0)
					enable = true;
				else
					return ret;
			}
		}

		if (enable) {
			xylonfb_logicvc_layer_enable(fbi, true);
			atomic_inc(&data->refcount);
		}
	}

	atomic_inc(&ld->refcount);

	return 0;
}

static int xylonfb_release(struct fb_info *fbi, int user)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (atomic_read(&ld->refcount) > 0) {
		atomic_dec(&ld->refcount);

		if (atomic_read(&ld->refcount) == 0) {
			xylonfb_logicvc_layer_enable(fbi, false);
			atomic_dec(&data->refcount);
		}
	}

	return 0;
}

static int xylonfb_check_var(struct fb_var_screeninfo *var,
			     struct fb_info *fbi)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	struct xylonfb_data *data = ld->data;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (var->xres < LOGICVC_MIN_XRES)
		var->xres = LOGICVC_MIN_XRES;
	if (var->xres > data->max_h_res)
		var->xres = data->max_h_res;
	if (var->yres < LOGICVC_MIN_VRES)
		var->yres = LOGICVC_MIN_VRES;
	if (var->yres > data->max_v_res)
		var->yres = data->max_v_res;
	if (fd->buffer_offset) {
		if (var->yres > fd->buffer_offset)
			return -EINVAL;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->xres_virtual > fd->width)
		var->xres_virtual = fd->width;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;
	if (var->yres_virtual > fd->height)
		var->yres_virtual = fd->height;

	/* YUV 4:2:2 layer type can only have even layer xoffset */
	if (fd->format == XYLONFB_FORMAT_YUYV ||
		fd->format == XYLONFB_FORMAT_UYVY ||
		fd->format == XYLONFB_FORMAT_YUYV_121010 ||
		fd->format == XYLONFB_FORMAT_UYVY_121010)
		var->xoffset &= ~((unsigned long) + 1);

	if ((var->xoffset + var->xres) >= var->xres_virtual)
		var->xoffset = var->xres_virtual - var->xres - 1;
	if ((var->yoffset + var->yres) >= var->yres_virtual)
		var->yoffset = var->yres_virtual - var->yres - 1;

	if (var->bits_per_pixel != fbi->var.bits_per_pixel) {
		if (var->bits_per_pixel == 24)
			var->bits_per_pixel = 32;
		else
			var->bits_per_pixel = fbi->var.bits_per_pixel;
	}

	var->grayscale = fbi->var.grayscale;

	var->transp.offset = fbi->var.transp.offset;
	var->transp.length = fbi->var.transp.length;
	var->transp.msb_right = fbi->var.transp.msb_right;
	var->red.offset = fbi->var.red.offset;
	var->red.length = fbi->var.red.length;
	var->red.msb_right = fbi->var.red.msb_right;
	var->green.offset = fbi->var.green.offset;
	var->green.length = fbi->var.green.length;
	var->green.msb_right = fbi->var.green.msb_right;
	var->blue.offset = fbi->var.blue.offset;
	var->blue.length = fbi->var.blue.length;
	var->blue.msb_right = fbi->var.blue.msb_right;
	var->height = fbi->var.height;
	var->width = fbi->var.width;
	var->sync = fbi->var.sync;
	var->rotate = fbi->var.rotate;

	return 0;
}

static int xylonfb_set_par(struct fb_info *fbi)
{
	struct device *dev = fbi->dev;
	struct fb_info **afbi = NULL;
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	unsigned long f;
	int i, bpp;
	int ret = 0;
	char vmode_opt[VMODE_NAME_SIZE];
	bool resolution_change, layer_on[LOGICVC_MAX_LAYERS];

	XYLONFB_DBG(INFO, "%s", __func__);

	XYLONFB_DBG(INFO, "%s var mode: [%dx%d]&%d, (%d %d)(%d %d)(%d %d)",
		__func__,
		fbi->var.xres, fbi->var.yres, fbi->var.pixclock,
		fbi->var.left_margin, fbi->var.right_margin,
		fbi->var.upper_margin, fbi->var.lower_margin,
		fbi->var.hsync_len, fbi->var.vsync_len);

	if (data->flags & XYLONFB_FLAGS_VMODE_SET)
		return 0;

	/* check if params changed */
		resolution_change = true;
	if (!(data->flags & XYLONFB_FLAGS_EDID_VMODE)) {
		struct fb_videomode *vmact = &data->vm_active.vmode;
		if (data->flags & XYLONFB_FLAGS_PUT_VSCREENINFO_EXACT) {
			if ((fbi->var.xres == vmact->xres) &&
				(fbi->var.yres == vmact->yres) &&
				(fbi->var.left_margin == vmact->left_margin) &&
				(fbi->var.right_margin == vmact->right_margin) &&
				(fbi->var.upper_margin == vmact->upper_margin) &&
				(fbi->var.lower_margin == vmact->lower_margin) &&
				(fbi->var.hsync_len == vmact->hsync_len) &&
				(fbi->var.vsync_len == vmact->vsync_len) &&
				(fbi->var.pixclock == vmact->pixclock))
			{
				resolution_change = false;  
			}
		}
		else {
			if ((fbi->var.xres == vmact->xres) &&
				(fbi->var.yres == vmact->yres))
				resolution_change = false;  
		}
	}

	if (resolution_change || (data->flags & XYLONFB_FLAGS_VMODE_INIT)) {
		if (!(data->flags & XYLONFB_FLAGS_VMODE_INIT)) {
			struct xylonfb_layer_data *ld;

			afbi = dev_get_drvdata(fbi->device);
			for (i = 0; i < data->layers; i++) {
				ld = afbi[i]->par;
				if (ld->flags & XYLONFB_FLAGS_LAYER_ENABLED)
					layer_on[i] = true;
				else
					layer_on[i] = false;
			}
		}

		xylonfb_disable_logicvc_output(fbi);
		xylonfb_logicvc_disp_ctrl(fbi, false);

		if (!(data->flags & XYLONFB_FLAGS_VMODE_INIT)) {
			data->vm_active.vmode.refresh = 60;
			sprintf(vmode_opt, "%dx%d%s-%d@%d%s",
				fbi->var.xres, fbi->var.yres,
				data->vm_active.opts_cvt,
				fbi->var.bits_per_pixel,
				data->vm_active.vmode.refresh,
				data->vm_active.opts_ext);
			if (!strcmp(data->vm.name, vmode_opt)) {
				data->vm_active = data->vm;
			} else {
				bpp = fbi->var.bits_per_pixel;
				xylonfb_mode_option = vmode_opt;
				ret = xylonfb_set_timings(fbi, bpp);
				xylonfb_mode_option = NULL;
			}
		}
		if (!ret) {
			f = PICOS2KHZ(data->vm_active.vmode.pixclock);
			if (data->flags & XYLONFB_FLAGS_PIXCLK_VALID)
				if (xylonfb_hw_pixclk_set(&data->pdev->dev,
							  data->pixel_clock, f))
					dev_err(dev,
						"failed set pixel clock\n");

			xylonfb_fbi_update(fbi);
			XYLONFB_DBG(INFO, "video mode: %dx%d%s-%d@%d%s\n",
				    fbi->var.xres, fbi->var.yres,
				    data->vm_active.opts_cvt,
				    fbi->var.bits_per_pixel,
				    data->vm_active.vmode.refresh,
				    data->vm_active.opts_ext);
		}

		xylonfb_enable_logicvc_output(fbi);
		xylonfb_logicvc_disp_ctrl(fbi, true);

		if (data->flags & XYLONFB_FLAGS_VMODE_INIT)
			data->flags |= XYLONFB_FLAGS_VMODE_SET;

		if (!(data->flags & XYLONFB_FLAGS_VMODE_SET)) {
			if (!afbi) {
				xylonfb_logicvc_layer_enable(fbi, true);
				return ret;
			}

			for (i = 0; i < data->layers; i++) {
				if (layer_on[i])
					xylonfb_logicvc_layer_enable(afbi[i],
								     true);
			}
		}
	}

	return ret;
}

static void xylonfb_set_color_hw_rgb2yuv(u16 t, u16 r, u16 g, u16 b, u32 *yuv,
					 struct xylonfb_layer_data *ld)
{
	struct xylonfb_data *data = ld->data;
	u32 y, u, v;

	XYLONFB_DBG(INFO, "%s", __func__);

	y = ((data->coeff.cyr * (r & 0xFF)) + (data->coeff.cyg * (g & 0xFF)) +
	     (data->coeff.cyb * (b & 0xFF)) + data->coeff.cy) /
	     LOGICVC_YUV_NORM;
	u = ((-data->coeff.cur * (r & 0xFF)) - (data->coeff.cug * (g & 0xFF)) +
	     (data->coeff.cub * (b & 0xFF)) + LOGICVC_COEFF_U) /
	     LOGICVC_YUV_NORM;
	v = ((data->coeff.cvr * (r & 0xFF)) - (data->coeff.cvg * (g & 0xFF)) -
	     (data->coeff.cvb * (b & 0xFF)) + LOGICVC_COEFF_V) /
	     LOGICVC_YUV_NORM;

	*yuv = ((t & 0xFF) << 24) | (y << 16) | (u << 8) | v;
}

static int xylonfb_set_color_hw(u16 *t, u16 *r, u16 *g, u16 *b,
				int len, int id, struct fb_info *fbi)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	u32 pixel, pixel_clut;
	u16 a = 0xFF;
	int bpp, to, ro, go, bo;

	XYLONFB_DBG(INFO, "%s", __func__);

	bpp = fd->bpp;

	to = fbi->var.transp.offset;
	ro = fbi->var.red.offset;
	go = fbi->var.green.offset;
	bo = fbi->var.blue.offset;

	switch (fbi->fix.visual) {
	case FB_VISUAL_PSEUDOCOLOR:
		if ((id > (LOGICVC_CLUT_SIZE - 1)) || (len > LOGICVC_CLUT_SIZE))
			return -EINVAL;

		switch (fd->format_clut) {
		case XYLONFB_FORMAT_CLUT_ARGB6565:
			while (len > 0) {
				if (t)
					a = t[id];
				pixel_clut = ((((a & 0xFC) >> 2) << to) |
					      (((r[id] & 0xF8) >> 3) << ro) |
					      (((g[id] & 0xFC) >> 2) << go) |
					      (((b[id] & 0xF8) >> 3) << bo));
				writel(pixel_clut, ld->clut_base +
				       (id*LOGICVC_CLUT_REGISTER_SIZE));
				len--;
				id++;
			}
			break;
		case XYLONFB_FORMAT_CLUT_ARGB8888:
			while (len > 0) {
				if (t)
					a = t[id];
				pixel_clut = (((a & 0xFF) << to) |
					      ((r[id] & 0xFF) << ro) |
					      ((g[id] & 0xFF) << go) |
					      ((b[id] & 0xFF) << bo));
				writel(pixel_clut, ld->clut_base +
				       (id*LOGICVC_CLUT_REGISTER_SIZE));
				len--;
				id++;
			}
			break;
		case XYLONFB_FORMAT_CLUT_AYUV8888:
			while (len > 0) {
				if (t)
					a = t[id];
				xylonfb_set_color_hw_rgb2yuv(a, r[id],
							     g[id], b[id],
					 		     &pixel_clut,
					 		     ld);
				writel(pixel_clut, ld->clut_base +
				       (id*LOGICVC_CLUT_REGISTER_SIZE));
				len--;
				id++;
			}
			break;
		}
		break;
	case FB_VISUAL_TRUECOLOR:
		switch (fd->format) {
		case XYLONFB_FORMAT_RGB332:
		case XYLONFB_FORMAT_BGR233:
			while (len > 0) {
				pixel = ((((r[id] & 0xE0) >> 5) << ro) |
					(((g[id] & 0xE0) >> 5) << go) |
					(((b[id] & 0xC0) >> 6) << bo));
				((u32 *)(fbi->pseudo_palette))[id] =
					(pixel << 24) | (pixel << 16) |
					(pixel << 8) | pixel;
				len--;
				id++;
			}
			break;
		case XYLONFB_FORMAT_RGB565:
		case XYLONFB_FORMAT_BGR565:
			while (len > 0) {
				pixel = ((((r[id] & 0xF8) >> 3) << ro) |
					(((g[id] & 0xFC) >> 2) << go) |
					(((b[id] & 0xF8) >> 3) << bo));
				((u32 *)(fbi->pseudo_palette))[id] =
					(pixel << 16) | pixel;
				len--;
				id++;
			}
			break;
		case XYLONFB_FORMAT_XRGB8888:
		case XYLONFB_FORMAT_XBGR8888:
			while (len > 0) {
				((u32 *)(fbi->pseudo_palette))[id] =
					(((r[id] & 0xFF) << ro) |
					((g[id] & 0xFF) << go) |
					((b[id] & 0xFF) << bo));
				len--;
				id++;
			}
			break;
		case XYLONFB_FORMAT_ARGB8888:
		case XYLONFB_FORMAT_ABGR8888:
			while (len > 0) {
				if (t)
					a = t[id];
				((u32 *)(fbi->pseudo_palette))[id] =
					(((a & 0xFF) << to) |
					((r[id] & 0xFF) << ro) |
					((g[id] & 0xFF) << go) |
					((b[id] & 0xFF) << bo));
				len--;
				id++;
			}
			break;
		case XYLONFB_FORMAT_ARGB3332:
		case XYLONFB_FORMAT_ABGR3233:
			while (len > 0) {
				if (t)
					a = t[id];
				pixel = ((((a & 0xE0) >> 5) << to) |
				(((r[id] & 0xE0) >> 5) << ro) |
				(((g[id] & 0xE0) >> 5) << go) |
				(((b[id] & 0xC0) >> 6) << bo));
				((u32 *)(fbi->pseudo_palette))[id] =
					(pixel << 16) | pixel;
				len--;
				id++;
			} break;
		case XYLONFB_FORMAT_ARGB565:
		case XYLONFB_FORMAT_ABGR565:
			while (len > 0) {
				if (t)
					a = t[id];
				((u32 *)(fbi->pseudo_palette))[id] =
					((((a & 0xFC) >> 2) << to) |
					(((r[id] & 0xF8) >> 3) << ro) |
					(((g[id] & 0xFC) >> 2) << go) |
					(((b[id] & 0xF8) >> 3) << bo));
				len--;
				id++;
			} break;
		case XYLONFB_FORMAT_XRGB2101010:
		case XYLONFB_FORMAT_XBGR2101010:
			while (len > 0) {
				((u32 *)(fbi->pseudo_palette))[id] =
					(((r[id] & 0x3FF) << ro) |
					((g[id] & 0x3FF) << go) |
					((b[id] & 0x3FF) << bo));
				len--;
				id++;
			}
			break;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int xylonfb_set_color(unsigned regno,
			     unsigned red, unsigned green, unsigned blue,
			     unsigned transp, struct fb_info *fbi)
{
	XYLONFB_DBG(INFO, "%s", __func__);

	return xylonfb_set_color_hw((u16 *)&transp,
				    (u16 *)&red, (u16 *)&green, (u16 *)&blue,
				    1, regno, fbi);
}

static int xylonfb_set_cmap(struct fb_cmap *cmap, struct fb_info *fbi)
{
	XYLONFB_DBG(INFO, "%s", __func__);

	return xylonfb_set_color_hw(cmap->transp,
				    cmap->red, cmap->green, cmap->blue,
				    cmap->len, cmap->start, fbi);
}

static void xylonfb_set_pixels(struct fb_info *fbi,
			       struct xylonfb_layer_data *ld,
			       int bpp, unsigned int pix)
{
	u32 *vmem;
	u8 *vmem8;
	u16 *vmem16;
	u32 *vmem32;
	int x, y, pixoffset;

	XYLONFB_DBG(INFO, "%s", __func__);

	vmem = ld->fb_base + (fbi->var.xoffset * (fbi->var.bits_per_pixel/4)) +
	       (fbi->var.yoffset * fbi->var.xres_virtual *
	       (fbi->var.bits_per_pixel/4));

	switch (bpp) {
	case 8:
		vmem8 = (u8 *)vmem;
		for (y = fbi->var.yoffset; y < fbi->var.yres; y++) {
			pixoffset = (y * fbi->var.xres_virtual);
			for (x = fbi->var.xoffset; x < fbi->var.xres; x++)
				vmem8[pixoffset + x] = pix;
		}
		break;
	case 16:
		vmem16 = (u16 *)vmem;
		for (y = fbi->var.yoffset; y < fbi->var.yres; y++) {
			pixoffset = (y * fbi->var.xres_virtual);
			for (x = fbi->var.xoffset; x < fbi->var.xres; x++)
				vmem16[pixoffset + x] = pix;
		}
		break;
	case 32:
		vmem32 = (u32 *)vmem;
		for (y = fbi->var.yoffset; y < fbi->var.yres; y++) {
			pixoffset = (y * fbi->var.xres_virtual);
			for (x = fbi->var.xoffset; x < fbi->var.xres; x++)
				vmem32[pixoffset + x] = pix;
		}
		break;
	}
}

static int xylonfb_blank(int blank_mode, struct fb_info *fbi)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	void __iomem *dev_base = data->dev_base;
	u32 ctrl = data->reg_access.get_reg_val(dev_base,
						LOGICVC_CTRL_ROFF,
						ld);
	u32 power = readl(dev_base + LOGICVC_POWER_CTRL_ROFF);

	XYLONFB_DBG(INFO, "%s", __func__);

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		XYLONFB_DBG(INFO, "FB_BLANK_UNBLANK");
		ctrl |= (LOGICVC_CTRL_HSYNC | LOGICVC_CTRL_VSYNC |
			LOGICVC_CTRL_DATA_ENABLE);
		data->reg_access.set_reg_val(ctrl, dev_base,
					     LOGICVC_CTRL_ROFF, ld);

		power |= LOGICVC_V_EN_MSK;
		writel(power, dev_base + LOGICVC_POWER_CTRL_ROFF);

		mdelay(50);
		break;

	case FB_BLANK_NORMAL:
		XYLONFB_DBG(INFO, "FB_BLANK_NORMAL");
		switch (fd->format) {
		case XYLONFB_FORMAT_C8:
			xylonfb_set_color(0, 0, 0, 0, 0xFF, fbi);
			xylonfb_set_pixels(fbi, ld, 8, 0);
			break;
		case XYLONFB_FORMAT_RGB332:
		case XYLONFB_FORMAT_BGR233:
			xylonfb_set_pixels(fbi, ld, 8, 0x00);
			break;
		case XYLONFB_FORMAT_ARGB3332:
		case XYLONFB_FORMAT_ABGR3233:
			xylonfb_set_pixels(fbi, ld, 16, 0xFF00);
			break;
		case XYLONFB_FORMAT_RGB565:
		case XYLONFB_FORMAT_ARGB565:
			xylonfb_set_pixels(fbi, ld, 16, 0x0000);
			break;
		case XYLONFB_FORMAT_ABGR565:
		case XYLONFB_FORMAT_BGR565:
			xylonfb_set_pixels(fbi, ld, 32, 0xFFFF0000);
			break;
		case XYLONFB_FORMAT_XRGB8888:
		case XYLONFB_FORMAT_ARGB8888:
		case XYLONFB_FORMAT_XBGR8888:
		case XYLONFB_FORMAT_ABGR8888:
			xylonfb_set_pixels(fbi, ld, 32, 0xFF000000);
			break;
		case XYLONFB_FORMAT_XRGB2101010:
		case XYLONFB_FORMAT_XBGR2101010:
			xylonfb_set_pixels(fbi, ld, 32, 0xC0000000);
			break;
		}
		break;

	case FB_BLANK_POWERDOWN:
		XYLONFB_DBG(INFO, "FB_BLANK_POWERDOWN");
		ctrl &= ~(LOGICVC_CTRL_HSYNC | LOGICVC_CTRL_VSYNC |
			LOGICVC_CTRL_DATA_ENABLE);
		data->reg_access.set_reg_val(ctrl, dev_base,
					     LOGICVC_CTRL_ROFF, ld);

		power &= ~LOGICVC_V_EN_MSK;
		writel(power, dev_base + LOGICVC_POWER_CTRL_ROFF);

		mdelay(50);
		break;

	case FB_BLANK_VSYNC_SUSPEND:
		XYLONFB_DBG(INFO, "FB_BLANK_VSYNC_SUSPEND");
		ctrl &= ~LOGICVC_CTRL_VSYNC;
		data->reg_access.set_reg_val(ctrl, dev_base,
					     LOGICVC_CTRL_ROFF, ld);
		break;

	case FB_BLANK_HSYNC_SUSPEND:
		XYLONFB_DBG(INFO, "FB_BLANK_HSYNC_SUSPEND");
		ctrl &= ~LOGICVC_CTRL_HSYNC;
		data->reg_access.set_reg_val(ctrl, dev_base,
					     LOGICVC_CTRL_ROFF, ld);
		break;
	}

	return 0;
}

static int xylonfb_pan_display(struct fb_var_screeninfo *var,
			       struct fb_info *fbi)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (!(data->flags & XYLONFB_FLAGS_SIZE_POSITION))
		return -EINVAL;

	if ((fbi->var.xoffset == var->xoffset) &&
	    (fbi->var.yoffset == var->yoffset))
		return 0;

	if (fbi->var.vmode & FB_VMODE_YWRAP) {
		return -EINVAL;
	} else {
		if (((var->xoffset + fbi->var.xres) > fbi->var.xres_virtual) ||
		    ((var->yoffset + fbi->var.yres) > fbi->var.yres_virtual))
			return -EINVAL;
	}

	if (fd->format == XYLONFB_FORMAT_YUYV ||
		fd->format == XYLONFB_FORMAT_UYVY ||
		fd->format == XYLONFB_FORMAT_YUYV_121010 ||
		fd->format == XYLONFB_FORMAT_UYVY_121010)
		var->xoffset &= ~((unsigned long) + 1);

	fbi->var.xoffset = var->xoffset;
	fbi->var.yoffset = var->yoffset;

	if (!(data->flags & XYLONFB_FLAGS_DYNAMIC_LAYER_ADDRESS)) {
		data->reg_access.set_reg_val(var->xoffset, ld->base,
					     LOGICVC_LAYER_HOFF_ROFF, ld);
		data->reg_access.set_reg_val(var->yoffset, ld->base,
					     LOGICVC_LAYER_VOFF_ROFF, ld);
	}

	if (data->flags & XYLONFB_FLAGS_DYNAMIC_LAYER_ADDRESS) {
		ld->fb_pbase_active = ld->fb_pbase +
				      ((var->xoffset * (fd->bpp / 8)) +
				      (var->yoffset * fd->width *
				      (fd->bpp / 8)));
		data->reg_access.set_reg_val(ld->fb_pbase_active, ld->base,
					     LOGICVC_LAYER_ADDR_ROFF, ld);
	}

	return 0;
}

static struct fb_ops xylonfb_ops = {
	.owner = THIS_MODULE,
	.fb_open = xylonfb_open,
	.fb_release = xylonfb_release,
	.fb_check_var = xylonfb_check_var,
	.fb_set_par = xylonfb_set_par,
	.fb_setcolreg = xylonfb_set_color,
	.fb_setcmap = xylonfb_set_cmap,
	.fb_blank = xylonfb_blank,
	.fb_pan_display = xylonfb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_ioctl = xylonfb_ioctl,
};

static int xylonfb_find_next_layer(struct xylonfb_data *data, int layers,
				   int id)
{
	dma_addr_t address = data->fd[id]->address;
	dma_addr_t temp_address = ((unsigned long) - 1);
	dma_addr_t loop_address;
	int next = -1;
	int i;

	XYLONFB_DBG(INFO, "%s", __func__);

	for (i = 0; i < layers; i++) {
		loop_address = data->fd[i]->address;
		if ((address < loop_address) && (loop_address < temp_address)) {
			next = i;
			temp_address = loop_address;
		}
	}

	return next;
}

static void xylonfb_get_vmem_height(struct xylonfb_data *data, int layers,
				    int id)
{
	struct xylonfb_layer_fix_data *fd = data->fd[id];
	dma_addr_t vmem_start = fd->address;
	dma_addr_t vmem_end;
	int next;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (fd->address_range && (id < (layers - 1))) {
		fd->height = fd->address_range / (fd->width * (fd->bpp / 8));
		return;
	}

	vmem_start = fd->address;

	next = xylonfb_find_next_layer(data, layers, id);
	if (next == -1) {
		if (fd->address_range) {
			fd->height = fd->address_range /
				     (fd->width * (fd->bpp / 8));
		} else {
			if (fd->buffer_offset)
				fd->height = fd->buffer_offset *
					     LOGICVC_MAX_LAYER_BUFFERS;
			else
				fd->height = XYLONFB_VRES_DEFAULT;
		}
	} else {
		vmem_end = data->fd[next]->address;
		fd->height = (vmem_end - vmem_start) /
			     (fd->width * (fd->bpp / 8));
	}

	if (fd->height > (data->max_v_res * LOGICVC_MAX_LAYER_BUFFERS))
		fd->height = data->max_v_res * LOGICVC_MAX_LAYER_BUFFERS;
}

static void xylonfb_set_fbi_var_screeninfo(struct fb_var_screeninfo *var,
					   struct xylonfb_data *data)
{
	XYLONFB_DBG(INFO, "%s", __func__);

	var->xres = data->vm_active.vmode.xres;
	var->yres = data->vm_active.vmode.yres;
	var->pixclock = data->vm_active.vmode.pixclock;
	var->left_margin = data->vm_active.vmode.left_margin;
	var->right_margin = data->vm_active.vmode.right_margin;
	var->upper_margin = data->vm_active.vmode.upper_margin;
	var->lower_margin = data->vm_active.vmode.lower_margin;
	var->hsync_len = data->vm_active.vmode.hsync_len;
	var->vsync_len = data->vm_active.vmode.vsync_len;
	var->sync = data->vm_active.vmode.sync;
	var->vmode = data->vm_active.vmode.vmode;
}

static void xylonfb_fbi_update(struct fb_info *fbi)
{
	struct fb_info **afbi = dev_get_drvdata(fbi->device);
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	int i, layers, id;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (!(data->flags & XYLONFB_FLAGS_EDID_VMODE) ||
	    !(data->flags & XYLONFB_FLAGS_EDID_READY) || !afbi)
		return;

	layers = data->layers;
	id = ld->fd->id;

	for (i = 0; i < layers; i++) {
		if (i == id)
			continue;

		xylonfb_set_fbi_var_screeninfo(&afbi[i]->var, data);
		afbi[i]->monspecs = afbi[id]->monspecs;
	}
}

static void xylonfb_set_hw_specifics(struct fb_info *fbi,
				     struct xylonfb_layer_data *ld,
				     struct xylonfb_layer_fix_data *fd)
{
	struct xylonfb_data *data = ld->data;

	XYLONFB_DBG(INFO, "%s", __func__);

	fbi->fix.smem_start = ld->fb_pbase;
	fbi->fix.smem_len = ld->fb_size;
	if (fd->type == LOGICVC_LAYER_RGB) {
		fbi->fix.type = FB_TYPE_PACKED_PIXELS;
	} else if (fd->type == LOGICVC_LAYER_YUV) {
		if (fd->format == XYLONFB_FORMAT_C8)
			fbi->fix.type = FB_TYPE_PACKED_PIXELS;
		else
			fbi->fix.type = FB_TYPE_FOURCC;
	}
	if ((fd->type == LOGICVC_LAYER_YUV) ||
	    (fd->type == LOGICVC_LAYER_ALPHA)) {
		if (fd->format == XYLONFB_FORMAT_C8)
			fbi->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		else
			fbi->fix.visual = FB_VISUAL_FOURCC;
	} else if (fd->format == XYLONFB_FORMAT_C8) {
		fbi->fix.visual = FB_VISUAL_PSEUDOCOLOR;
	} else {
		/*
		 * Other logiCVC layer pixel formats:
		 * - 8 bpp: LAYER or PIXEL alpha
		 *   It is not true color, RGB triplet is stored in 8 bits.
		 * - 16 bpp:
		 *   LAYER or PIXEL alpha: RGB triplet is stored in 16 bits
		 * - 32 bpp: LAYER or PIXEL alpha
		 *   True color, RGB triplet or ARGB quadriplet
		 *   is stored in 32 bits.
		 */
		fbi->fix.visual = FB_VISUAL_TRUECOLOR;
	}

	fbi->fix.xpanstep = 1;
	fbi->fix.ypanstep = 1;
	fbi->fix.ywrapstep = 0;
	fbi->fix.line_length = fd->width * (fd->bpp / 8);
	fbi->fix.mmio_start = ld->pbase;
	fbi->fix.mmio_len = LOGICVC_LAYER_REGISTERS_RANGE;
	fbi->fix.accel = FB_ACCEL_NONE;

	fbi->var.xres_virtual = fd->width;
	fbi->var.yres_virtual = fd->height;

	fbi->var.bits_per_pixel = fd->bpp;

	switch (fd->type) {
	case LOGICVC_LAYER_ALPHA:
		fbi->var.grayscale = LOGICVC_PIX_FMT_ALPHA;
		break;
	case LOGICVC_LAYER_RGB:
		fbi->var.grayscale = 0;
		break;
	case LOGICVC_LAYER_YUV:
		switch (fd->format){
		case XYLONFB_FORMAT_C8:
			fbi->var.grayscale = LOGICVC_PIX_FMT_AYUV;
			break;
		case XYLONFB_FORMAT_YUYV:
			fbi->var.grayscale = V4L2_PIX_FMT_VYUY;
			break;
		case XYLONFB_FORMAT_UYVY:
			fbi->var.grayscale = V4L2_PIX_FMT_VYUY;
			break;
		case XYLONFB_FORMAT_AYUV:
			fbi->var.grayscale = LOGICVC_PIX_FMT_AYUV;
			break;
		case XYLONFB_FORMAT_AVUY:
			fbi->var.grayscale = LOGICVC_PIX_FMT_AVUY;
			break;
		case XYLONFB_FORMAT_XYUV:
			fbi->var.grayscale = LOGICVC_PIX_FMT_XYUV;
			break;
		case XYLONFB_FORMAT_XVUY:
			fbi->var.grayscale = LOGICVC_PIX_FMT_XVUY;
			break;
		case XYLONFB_FORMAT_YUYV_121010:
			fbi->var.grayscale = LOGICVC_PIX_FMT_YUYV_121010;
		break;
		case XYLONFB_FORMAT_UYVY_121010:
			fbi->var.grayscale = LOGICVC_PIX_FMT_UYVY_121010;
			break;
		case XYLONFB_FORMAT_XYUV_2101010:
			fbi->var.grayscale = LOGICVC_PIX_FMT_XYUV_2101010;
		break;
		case XYLONFB_FORMAT_XVUY_2101010:
			fbi->var.grayscale = LOGICVC_PIX_FMT_XYUV_2101010;
			break;
		}
		break;
	}

	/*
	 * Set values according to logiCVC layer data width configuration:
	 * layer data width can be 1, 2, 4 bytes
	 */
	if (fd->transparency == LOGICVC_ALPHA_LAYER) {
		fbi->var.transp.offset = 0;
		fbi->var.transp.length = 0;
	}

	switch (fd->format) {
	case XYLONFB_FORMAT_A8:
		fbi->var.transp.offset = 0;
		fbi->var.transp.length = 8;
		break;
	case XYLONFB_FORMAT_C8:
		switch (fd->format_clut) {
		case XYLONFB_FORMAT_CLUT_ARGB6565:
			fbi->var.transp.offset = 24;
			fbi->var.transp.length = 6;
			fbi->var.red.offset = 19;
			fbi->var.red.length = 5;
			fbi->var.green.offset = 10;
			fbi->var.green.length = 6;
			fbi->var.blue.offset = 3;
			fbi->var.blue.length = 5;
			break;
		case XYLONFB_FORMAT_CLUT_ARGB8888:
			fbi->var.transp.offset = 24;
			fbi->var.transp.length = 8;
			fbi->var.red.offset = 16;
			fbi->var.red.length = 8;
			fbi->var.green.offset = 8;
			fbi->var.green.length = 8;
			fbi->var.blue.offset = 0;
			fbi->var.blue.length = 8;
			break;
		case XYLONFB_FORMAT_CLUT_AYUV8888:
			fbi->var.transp.offset = 24;
			fbi->var.transp.length = 8;
			fbi->var.red.offset = 16;
			fbi->var.red.length = 8;
			fbi->var.green.offset = 8;
			fbi->var.green.length = 8;
			fbi->var.blue.offset = 0;
			fbi->var.blue.length = 8;
			break;
		}
		break;
	case XYLONFB_FORMAT_ARGB3332:
		fbi->var.transp.offset = 8;
		fbi->var.transp.length = 3;
	case XYLONFB_FORMAT_RGB332:
		fbi->var.red.offset = 5;
		fbi->var.red.length = 3;
		fbi->var.green.offset = 2;
		fbi->var.green.length = 3;
		fbi->var.blue.offset = 0;
		fbi->var.blue.length = 2;
		break;
	case XYLONFB_FORMAT_ARGB565:
		fbi->var.transp.offset = 24;
		fbi->var.transp.length = 6;
	case XYLONFB_FORMAT_RGB565:
		fbi->var.red.offset = 11;
		fbi->var.red.length = 5;
		fbi->var.green.offset = 5;
		fbi->var.green.length = 6;
		fbi->var.blue.offset = 0;
		fbi->var.blue.length = 5;
		break;
	case XYLONFB_FORMAT_ARGB8888:
	case XYLONFB_FORMAT_AYUV:
		fbi->var.transp.offset = 24;
		fbi->var.transp.length = 8;
	case XYLONFB_FORMAT_XRGB8888:
		fbi->var.red.offset = 16;
		fbi->var.red.length = 8;
		fbi->var.green.offset = 8;
		fbi->var.green.length = 8;
		fbi->var.blue.offset = 0;
		fbi->var.blue.length = 8;
		break;
	case XYLONFB_FORMAT_ABGR3233:
		fbi->var.transp.offset = 8;
		fbi->var.transp.length = 3;
	case XYLONFB_FORMAT_BGR233:
		fbi->var.red.offset = 0;
		fbi->var.red.length = 3;
		fbi->var.green.offset = 3;
		fbi->var.green.length = 3;
		fbi->var.blue.offset = 6;
		fbi->var.blue.length = 2;
		break;
	case XYLONFB_FORMAT_ABGR565:
		fbi->var.transp.offset = 24;
		fbi->var.transp.length = 6;
	case XYLONFB_FORMAT_BGR565:
		fbi->var.red.offset = 0;
		fbi->var.red.length = 5;
		fbi->var.green.offset = 5;
		fbi->var.green.length = 6;
		fbi->var.blue.offset = 11;
		fbi->var.blue.length = 5;
		break;
	case XYLONFB_FORMAT_ABGR8888:
		fbi->var.transp.offset = 24;
		fbi->var.transp.length = 8;
	case XYLONFB_FORMAT_XBGR8888:
		fbi->var.red.offset = 0;
		fbi->var.red.length = 8;
		fbi->var.green.offset = 8;
		fbi->var.green.length = 8;
		fbi->var.blue.offset = 16;
		fbi->var.blue.length = 8;
		break;
	case XYLONFB_FORMAT_XRGB2101010:
		fbi->var.red.offset = 20;
		fbi->var.red.length = 10;
		fbi->var.green.offset = 10;
		fbi->var.green.length = 10;
		fbi->var.blue.offset = 0;
		fbi->var.blue.length = 10;
		break;
	case XYLONFB_FORMAT_XBGR2101010:
		fbi->var.red.offset = 0;
		fbi->var.red.length = 10;
		fbi->var.green.offset = 10;
		fbi->var.green.length = 10;
		fbi->var.blue.offset = 20;
		fbi->var.blue.length = 10;
		break;
	case XYLONFB_FORMAT_YUYV:
		fbi->var.transp.offset = 16;
		fbi->var.transp.length = 8;
		fbi->var.red.offset = 0;
		fbi->var.red.length = 8;
		fbi->var.green.offset = 8;
		fbi->var.green.length = 8;
		fbi->var.blue.offset = 24;
		fbi->var.blue.length = 8;
		break;
	case XYLONFB_FORMAT_UYVY:
		fbi->var.transp.offset = 24;
		fbi->var.transp.length = 8;
		fbi->var.red.offset = 8;
		fbi->var.red.length = 8;
		fbi->var.green.offset = 0;
		fbi->var.green.length = 8;
		fbi->var.blue.offset = 16;
		fbi->var.blue.length = 8;
		break;
	case XYLONFB_FORMAT_YUYV_121010:
		fbi->var.transp.offset = 0;
		fbi->var.transp.length = 10;
		fbi->var.red.offset = 0;
		fbi->var.red.length = 10;
		fbi->var.green.offset = 10;
		fbi->var.green.length = 10;
		fbi->var.blue.offset = 10;
		fbi->var.blue.length = 10;
		break;
	case XYLONFB_FORMAT_UYVY_121010:
		fbi->var.transp.offset = 10;
		fbi->var.transp.length = 10;
		fbi->var.red.offset = 10;
		fbi->var.red.length = 10;
		fbi->var.green.offset = 0;
		fbi->var.green.length = 10;
		fbi->var.blue.offset = 0;
		fbi->var.blue.length = 10;
		break;
	case XYLONFB_FORMAT_XYUV:
		fbi->var.red.offset = 16;
		fbi->var.red.length = 8;
		fbi->var.green.offset = 8;
		fbi->var.green.length = 8;
		fbi->var.blue.offset = 0;
		fbi->var.blue.length = 8;
		break;
	case XYLONFB_FORMAT_AVUY:
		fbi->var.transp.offset = 24;
		fbi->var.transp.length = 8;
	case XYLONFB_FORMAT_XVUY:
		fbi->var.red.offset = 0;
		fbi->var.red.length = 8;
		fbi->var.green.offset = 8;
		fbi->var.green.length = 8;
		fbi->var.blue.offset = 16;
		fbi->var.blue.length = 8;
		break;
	case XYLONFB_FORMAT_XYUV_2101010:
		fbi->var.red.offset = 20;
		fbi->var.red.length = 10;
		fbi->var.green.offset = 10;
		fbi->var.green.length = 10;
		fbi->var.blue.offset = 0;
		fbi->var.blue.length = 10;
		break;
	case XYLONFB_FORMAT_XVUY_2101010:
		fbi->var.red.offset = 0;
		fbi->var.red.length = 10;
		fbi->var.green.offset = 10;
		fbi->var.green.length = 10;
		fbi->var.blue.offset = 20;
		fbi->var.blue.length = 10;
		break;
	}
	fbi->var.transp.msb_right = 0;
	fbi->var.red.msb_right = 0;
	fbi->var.green.msb_right = 0;
	fbi->var.blue.msb_right = 0;
	fbi->var.activate = FB_ACTIVATE_NOW;
	fbi->var.height = 0;
	fbi->var.width = 0;
	fbi->var.sync = 0;
	if (!(data->vm_active.ctrl & LOGICVC_CTRL_HSYNC_INVERT))
		fbi->var.sync |= FB_SYNC_HOR_HIGH_ACT;
	if (!(data->vm_active.ctrl & LOGICVC_CTRL_VSYNC_INVERT))
		fbi->var.sync |= FB_SYNC_VERT_HIGH_ACT;
	fbi->var.rotate = 0;
}

static int xylonfb_set_timings(struct fb_info *fbi, int bpp)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	struct fb_var_screeninfo fb_var;
	struct fb_videomode *vm;
	int rc;
#if defined(CONFIG_FB_XYLON_MISC)
	u32 xres, yres;
#endif

	XYLONFB_DBG(INFO, "%s flags 0x%x", __func__, data->flags);

	if ((data->flags & XYLONFB_FLAGS_VMODE_INIT) &&
	    (data->flags & XYLONFB_FLAGS_VMODE_CUSTOM) &&
	    memchr(data->vm.name, 'x', 10)) {
		data->vm_active = data->vm;
		vm = &data->vm.vmode;
		data->vm_active.vmode.refresh =
			DIV_ROUND_CLOSEST((PICOS2KHZ(vm->pixclock) * 1000),
					  ((vm->xres + vm->left_margin +
					  vm->right_margin + vm->hsync_len) *
					  (vm->yres + vm->upper_margin +
					  vm->lower_margin + vm->vsync_len)));
		return 0;
	}

	rc = 255;
	if ((data->flags & XYLONFB_FLAGS_EDID_VMODE) &&
	    (data->flags & XYLONFB_FLAGS_EDID_READY)) {
		if (data->flags & XYLONFB_FLAGS_VMODE_INIT) {
			rc = fb_find_mode(&fb_var, fbi, xylonfb_mode_option,
					  fbi->monspecs.modedb,
					  fbi->monspecs.modedb_len,
					  &xylonfb_vm.vmode, bpp);
			if (!rc)
				return -EINVAL;
		} else {
			rc = fb_find_mode(&fb_var, fbi, xylonfb_mode_option,
					  fbi->monspecs.modedb,
					  fbi->monspecs.modedb_len,
					  &xylonfb_vm.vmode, bpp);
			if ((rc != 1) && (rc != 2))
				return -EINVAL;
#if defined(CONFIG_FB_XYLON_MISC)
			if ((fbi->monspecs.modedb) &&
			    (data->misc.monspecs->misc & FB_MISC_1ST_DETAIL)) {
				xres = fbi->monspecs.modedb[0].xres;
				yres = fbi->monspecs.modedb[0].yres;
			} else {
				xres = 0;
				yres = 0;
			}
			if ((fbi->var.xres == xres) && (fbi->var.yres == yres))
				fb_videomode_to_var(&fb_var,
						    &fbi->monspecs.modedb[0]);
#endif
		}
	}
	/* get video mode directly from fb_var_screeninfo */
	else if ( data->flags & XYLONFB_FLAGS_PUT_VSCREENINFO_EXACT) {
		fb_var = fbi->var;
		XYLONFB_DBG(INFO, "%s exact mode", __func__);
	}
	else {
		XYLONFB_DBG(INFO, "%s fb_find_mode %s ", __func__, xylonfb_mode_option);           
		rc = fb_find_mode(&fb_var, fbi, xylonfb_mode_option, NULL, 0,
				  &xylonfb_vm.vmode, bpp);
	}
	switch (rc) {
	case 0:
		dev_err(fbi->dev, "failed find video mode\n"
			"using driver default mode %dx%dM-%d@%d\n",
			xylonfb_vm.vmode.xres,
			xylonfb_vm.vmode.yres,
			bpp,
			xylonfb_vm.vmode.refresh);
		break;
	case 1:
		dev_dbg(fbi->dev, "video mode %s", xylonfb_mode_option);
		break;
	case 2:
		dev_warn(fbi->dev, "video mode %s with ignored refresh rate\n",
			 xylonfb_mode_option);
		break;
	case 3:
		dev_warn(fbi->dev, "default video mode %dx%dM-%d@%d\n",
			 xylonfb_vm.vmode.xres,
			 xylonfb_vm.vmode.yres,
			 bpp,
			 xylonfb_vm.vmode.refresh);
		break;
	case 4:
		dev_warn(fbi->dev, "video mode fallback\n");
		break;
	default:
		break;
	}

	data->vm_active.ctrl = data->vm.ctrl;
	data->vm_active.vmode.xres = fb_var.xres;
	data->vm_active.vmode.yres = fb_var.yres;
	data->vm_active.vmode.pixclock = fb_var.pixclock;
	data->vm_active.vmode.left_margin = fb_var.left_margin;
	data->vm_active.vmode.right_margin = fb_var.right_margin;
	data->vm_active.vmode.upper_margin = fb_var.upper_margin;
	data->vm_active.vmode.lower_margin = fb_var.lower_margin;
	data->vm_active.vmode.hsync_len = fb_var.hsync_len;
	data->vm_active.vmode.vsync_len = fb_var.vsync_len;
	data->vm_active.vmode.sync = fb_var.sync;
	data->vm_active.vmode.vmode = fb_var.vmode;
	data->vm_active.vmode.refresh =
		DIV_ROUND_CLOSEST((PICOS2KHZ(fb_var.pixclock) * 1000),
				  ((fb_var.xres + fb_var.left_margin +
				  fb_var.right_margin + fb_var.hsync_len) *
				  (fb_var.yres + fb_var.upper_margin +
				  fb_var.lower_margin + fb_var.vsync_len)));
	strcpy(data->vm_active.opts_cvt, data->vm.opts_cvt);
	strcpy(data->vm_active.opts_ext, data->vm.opts_ext);
	sprintf(data->vm_active.name, "%dx%d%s-%d@%d%s",
		fb_var.xres, fb_var.yres,
		data->vm_active.opts_cvt,
		fb_var.bits_per_pixel,
		data->vm_active.vmode.refresh,
		data->vm_active.opts_ext);

	if ((data->flags & XYLONFB_FLAGS_EDID_READY) ||
	    !memchr(data->vm.name, 'x', 10))
		data->vm = data->vm_active;

	return 0;
}

static int xylonfb_register_fb(struct fb_info *fbi,
			       struct xylonfb_layer_data *ld, int id,
			       int *regfb)
{
	struct device *dev = fbi->dev;
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	int transp;

	XYLONFB_DBG(INFO, "%s", __func__);

	fbi->flags = FBINFO_DEFAULT;
	fbi->screen_base = (char __iomem *)ld->fb_base;
	fbi->screen_size = ld->fb_size;
	fbi->pseudo_palette = kzalloc(sizeof(u32) * XYLONFB_PSEUDO_PALETTE_SIZE,
				      GFP_KERNEL);
	fbi->fbops = &xylonfb_ops;

	sprintf(fbi->fix.id, "Xylon FB%d", id);
	xylonfb_set_hw_specifics(fbi, ld, fd);

	if (!(data->flags & XYLONFB_FLAGS_VMODE_DEFAULT)) {
		if (!xylonfb_set_timings(fbi, fbi->var.bits_per_pixel))
			data->flags |= XYLONFB_FLAGS_VMODE_DEFAULT;
		else
			dev_err(dev, "videomode not set\n");
	}
	xylonfb_set_fbi_var_screeninfo(&fbi->var, data);
	fbi->mode = &data->vm_active.vmode;
	fbi->mode->name = data->vm_active.name;

	if (fd->transparency == LOGICVC_ALPHA_LAYER)
		transp = 0;
	else
		transp = 1;
	if (fb_alloc_cmap(&fbi->cmap, XYLONFB_PSEUDO_PALETTE_SIZE, transp))
		return -ENOMEM;

	/*
	 * After fb driver registration, values in struct fb_info
	 * must not be changed anywhere else in driver except in
	 * xylonfb_set_par() function
	 */
	*regfb = register_framebuffer(fbi);
	if (*regfb) {
		dev_err(dev, "failed register fb %d\n", id);
		return -EINVAL;
	}

	return 0;
}

static void xylonfb_layer_initialize(struct xylonfb_layer_data *ld)
{
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	u32 reg = ld->data->reg_access.get_reg_val(ld->base,
						   LOGICVC_LAYER_CTRL_ROFF,
						   ld);

	XYLONFB_DBG(INFO, "%s", __func__);

	reg |= LOGICVC_LAYER_CTRL_COLOR_TRANSPARENCY_DISABLE;
	if (fd->component_swap)
		reg |= LOGICVC_LAYER_CTRL_PIXEL_FORMAT_ABGR;
	ld->data->reg_access.set_reg_val(reg, ld->base,
					 LOGICVC_LAYER_CTRL_ROFF,
					 ld);

	if (data->flags & XYLONFB_FLAGS_DYNAMIC_LAYER_ADDRESS)
		data->reg_access.set_reg_val(ld->fb_pbase, ld->base,
					     LOGICVC_LAYER_ADDR_ROFF,
					     ld);
}

static int xylonfb_vmem_init(struct xylonfb_layer_data *ld, int id, bool *mmap)
{
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	struct device *dev = &data->pdev->dev;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (fd->address) {
		ld->fb_pbase = fd->address;

		xylonfb_get_vmem_height(data, data->layers, id);
		ld->fb_size = fd->width * (fd->bpp / 8) * fd->height;

		if (*mmap) {
			ld->fb_base = (__force void *)ioremap_wc(ld->fb_pbase,
								 ld->fb_size);
			if (!ld->fb_base) {
				dev_err(dev, "failed map video memory\n");
				return -EINVAL;
			}
		}
	} else {
		if (fd->buffer_offset)
			fd->height = fd->buffer_offset *
				     LOGICVC_MAX_LAYER_BUFFERS;
		else
			fd->height = XYLONFB_VRES_DEFAULT *
				     LOGICVC_MAX_LAYER_BUFFERS;
		ld->fb_size = fd->width * (fd->bpp / 8) * fd->height;

		ld->fb_base = dma_alloc_coherent(&data->pdev->dev,
						 PAGE_ALIGN(ld->fb_size),
						 &ld->fb_pbase, GFP_KERNEL);
		if (!ld->fb_base) {
			dev_err(dev, "failed allocate video buffer ID%d\n", id);
			return -ENOMEM;
		}

		data->flags |= XYLONFB_FLAGS_DMA_BUFFER;
	}

	ld->fb_pbase_active = ld->fb_pbase;

	*mmap = false;

	return 0;
}

static void xylonfb_logicvc_disp_ctrl(struct fb_info *fbi, bool enable)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	void __iomem *dev_base = data->dev_base;
	u32 val;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (enable) {
		val = LOGICVC_EN_VDD_MSK;
		writel(val, dev_base + LOGICVC_POWER_CTRL_ROFF);
		mdelay(data->pwr_delay);
		val |= LOGICVC_V_EN_MSK;
		writel(val, dev_base + LOGICVC_POWER_CTRL_ROFF);
		mdelay(data->sig_delay);
		val |= LOGICVC_EN_BLIGHT_MSK;
		writel(val, dev_base + LOGICVC_POWER_CTRL_ROFF);
	} else {
		writel(0, dev_base + LOGICVC_POWER_CTRL_ROFF);
	}
}

static void xylonfb_logicvc_layer_enable(struct fb_info *fbi, bool enable)
{
	struct xylonfb_layer_data *ld = fbi->par;
	u32 reg;

	XYLONFB_DBG(INFO, "%s", __func__);

	reg = ld->data->reg_access.get_reg_val(ld->base,
					       LOGICVC_LAYER_CTRL_ROFF,
					       ld);

	if (enable) {
		reg |= LOGICVC_LAYER_CTRL_ENABLE;
		ld->flags |= XYLONFB_FLAGS_LAYER_ENABLED;
	} else {
		reg &= ~LOGICVC_LAYER_CTRL_ENABLE;
		ld->flags &= ~XYLONFB_FLAGS_LAYER_ENABLED;
	}

	ld->data->reg_access.set_reg_val(reg, ld->base,
					 LOGICVC_LAYER_CTRL_ROFF,
					 ld);
}

static void xylonfb_enable_logicvc_output(struct fb_info *fbi)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	void __iomem *dev_base = data->dev_base;
	struct fb_videomode *vm = &data->vm_active.vmode;

	XYLONFB_DBG(INFO, "%s", __func__);

	writel(vm->right_margin - 1, dev_base + LOGICVC_HSYNC_FRONT_PORCH_ROFF);
	writel(vm->hsync_len - 1, dev_base + LOGICVC_HSYNC_ROFF);
	writel(vm->left_margin - 1, dev_base + LOGICVC_HSYNC_BACK_PORCH_ROFF);
	writel(vm->xres - 1, dev_base + LOGICVC_HRES_ROFF);
	writel(vm->lower_margin - 1, dev_base + LOGICVC_VSYNC_FRONT_PORCH_ROFF);
	writel(vm->vsync_len - 1, dev_base + LOGICVC_VSYNC_ROFF);
	writel(vm->upper_margin - 1, dev_base + LOGICVC_VSYNC_BACK_PORCH_ROFF);
	writel(vm->yres - 1, dev_base + LOGICVC_VRES_ROFF);
	data->reg_access.set_reg_val(data->vm_active.ctrl, dev_base,
				     LOGICVC_CTRL_ROFF, ld);

	if (data->flags & XYLONFB_FLAGS_BACKGROUND_LAYER_YUV)
		data->reg_access.set_reg_val(LOGICVC_COLOR_YUV888_BLACK,
					     dev_base,
					     LOGICVC_BACKGROUND_COLOR_ROFF,
					     ld);
	else
		data->reg_access.set_reg_val(LOGICVC_COLOR_RGB_BLACK,
					     dev_base,
					     LOGICVC_BACKGROUND_COLOR_ROFF,
					     ld);

	writel(LOGICVC_DTYPE_REG_INIT, dev_base + LOGICVC_DTYPE_ROFF);

	XYLONFB_DBG(INFO, "logiCVC HW parameters:\n" \
		"    Horizontal Front Porch: %d pixclks\n" \
		"    Horizontal Sync:        %d pixclks\n" \
		"    Horizontal Back Porch:  %d pixclks\n" \
		"    Vertical Front Porch:   %d pixclks\n" \
		"    Vertical Sync:          %d pixclks\n" \
		"    Vertical Back Porch:    %d pixclks\n" \
		"    Pixel Clock:            %d ps\n" \
		"    Horizontal Resolution:  %d pixels\n" \
		"    Vertical Resolution:    %d lines\n", \
		vm->right_margin, vm->hsync_len, vm->left_margin,
		vm->lower_margin, vm->vsync_len, vm->upper_margin,
		vm->pixclock, vm->xres, vm->yres);
}

static void xylonfb_disable_logicvc_output(struct fb_info *fbi)
{
	struct fb_info **afbi = dev_get_drvdata(fbi->device);
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	int i;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (afbi)
		for (i = 0; i < data->layers; i++)
			xylonfb_logicvc_layer_enable(afbi[i], false);
}

static void xylonfb_start(struct fb_info **afbi, int layers)
{
	struct fb_info *fbi = afbi[0];
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	int i;

	XYLONFB_DBG(INFO, "%s", __func__);

	for (i = 0; i < layers; i++) {
		ld = afbi[i]->par;
		if (ld->flags & XYLONFB_FLAGS_LAYER_ENABLED)
			continue;

		xylonfb_logicvc_layer_enable(afbi[i], false);
	}

	if (data->flags & XYLONFB_FLAGS_VSYNC_IRQ) {
		writel(LOGICVC_INT_V_SYNC,
		       data->dev_base + LOGICVC_INT_STAT_ROFF);
		data->reg_access.set_reg_val(~LOGICVC_INT_V_SYNC,
					     data->dev_base,
					     LOGICVC_INT_MASK_ROFF, ld);
	}

	for (i = 0; i < layers; i++) {
		ld = afbi[i]->par;
		XYLONFB_DBG(INFO, "logiCVC layer %d\n" \
			"    Registers Base Address:     0x%lX\n" \
			"    Layer Video Memory Address: 0x%lX\n" \
			"    X resolution:               %d\n" \
			"    Y resolution:               %d\n" \
			"    X resolution (virtual):     %d\n" \
			"    Y resolution (virtual):     %d\n" \
			"    Line length (bytes):        %d\n" \
			"    Bits per Pixel:             %d\n" \
			"\n", \
			i,
			(unsigned long)ld->pbase,
			(unsigned long)ld->fb_pbase,
			afbi[i]->var.xres,
			afbi[i]->var.yres,
			afbi[i]->var.xres_virtual,
			afbi[i]->var.yres_virtual,
			afbi[i]->fix.line_length,
			afbi[i]->var.bits_per_pixel);
	}
}

static void xylonfb_get_vmode_opts(struct xylonfb_data *data)
{
	char *s, *opt, *ext, *c;

	XYLONFB_DBG(INFO, "%s", __func__);

	if ((data->flags & XYLONFB_FLAGS_EDID_VMODE) &&
	    (data->flags & XYLONFB_FLAGS_EDID_READY))
		return;

	s = data->vm.name;
	opt = data->vm.opts_cvt;
	ext = data->vm.opts_ext;

	data->vm.vmode.vmode = 0;

	c = strchr(s, 'M');
	if (c)
		*opt++ = *c;
	c = strchr(s, 'R');
	if (c)
		*opt = *c;
	c = strchr(s, 'i');
	if (c) {
		*ext++ = *c;
		data->vm.vmode.vmode |= FB_VMODE_INTERLACED;
	}
	c = strchr(s, 'm');
	if (c)
		*ext = *c;
}

static bool xylonfb_allow_console(struct xylonfb_layer_fix_data *fd)
{
	XYLONFB_DBG(INFO, "%s", __func__);

	switch (fd->format) {
	case XYLONFB_FORMAT_A8:
	case XYLONFB_FORMAT_C8:
	case XYLONFB_FORMAT_RGB332:
	case XYLONFB_FORMAT_RGB565:
	case XYLONFB_FORMAT_XRGB8888:
	case XYLONFB_FORMAT_ARGB8888:
	case XYLONFB_FORMAT_BGR233:
	case XYLONFB_FORMAT_ARGB3332:
	case XYLONFB_FORMAT_ABGR3233:
	case XYLONFB_FORMAT_ARGB565:
	case XYLONFB_FORMAT_ABGR565:
	case XYLONFB_FORMAT_BGR565:
	case XYLONFB_FORMAT_XBGR8888:
	case XYLONFB_FORMAT_ABGR8888:
	case XYLONFB_FORMAT_XRGB2101010:
	case XYLONFB_FORMAT_XBGR2101010:
/* packed YCbCr 4:2:2, 32bit for 2 pixels. 8bit color component */
	case XYLONFB_FORMAT_YUYV:
	case XYLONFB_FORMAT_UYVY:
/* packed YCbCr 4:2:2, 64bit for 2 pixels, 10bit color component */
	case XYLONFB_FORMAT_YUYV_121010:
	case XYLONFB_FORMAT_UYVY_121010:
/* packed YCbCr 4:4:4, 32bit for 1 pixel, 8bit color component */
	case XYLONFB_FORMAT_AYUV:
	case XYLONFB_FORMAT_AVUY:
	case XYLONFB_FORMAT_XYUV:
	case XYLONFB_FORMAT_XVUY:
/* packed YCbCr 4:4:4, 32bit for 1 pixel, 10bit color component */
	case XYLONFB_FORMAT_XYUV_2101010:
	case XYLONFB_FORMAT_XVUY_2101010:
		return true;
	default:
		return false;
	}
}

int xylonfb_init_core(struct xylonfb_data *data)
{
	struct device *dev = &data->pdev->dev;
	struct fb_info **afbi, *fbi;
	struct xylonfb_layer_data *ld;
	void __iomem *dev_base;
	u32 ip_ver;
	int i, ret, layers, console_layer;
	int regfb[LOGICVC_MAX_LAYERS];
	size_t size;
	unsigned short layer_base_off[] = {
		(LOGICVC_LAYER_BASE_OFFSET + LOGICVC_LAYER_0_OFFSET),
		(LOGICVC_LAYER_BASE_OFFSET + LOGICVC_LAYER_1_OFFSET),
		(LOGICVC_LAYER_BASE_OFFSET + LOGICVC_LAYER_2_OFFSET),
		(LOGICVC_LAYER_BASE_OFFSET + LOGICVC_LAYER_3_OFFSET),
		(LOGICVC_LAYER_BASE_OFFSET + LOGICVC_LAYER_4_OFFSET)
	};
	unsigned short clut_base_off[] = {
		(LOGICVC_CLUT_BASE_OFFSET + LOGICVC_CLUT_L0_CLUT_0_OFFSET),
		(LOGICVC_CLUT_BASE_OFFSET + LOGICVC_CLUT_L1_CLUT_0_OFFSET),
		(LOGICVC_CLUT_BASE_OFFSET + LOGICVC_CLUT_L2_CLUT_0_OFFSET),
		(LOGICVC_CLUT_BASE_OFFSET + LOGICVC_CLUT_L3_CLUT_0_OFFSET),
		(LOGICVC_CLUT_BASE_OFFSET + LOGICVC_CLUT_L4_CLUT_0_OFFSET),
	};
	bool memmap;

	XYLONFB_DBG(INFO, "%s", __func__);

	dev_base = devm_ioremap_resource(dev, &data->resource_mem);
	if (IS_ERR(dev_base)) {
		dev_err(dev, "failed ioremap mem resource\n");
		return PTR_ERR(dev_base);
	}
	data->dev_base = dev_base;

	data->irq = data->resource_irq.start;
	ret = devm_request_irq(dev, data->irq, xylonfb_isr, IRQF_TRIGGER_HIGH,
			       XYLONFB_DEVICE_NAME, dev);
	if (ret)
		return ret;

	ip_ver = readl(dev_base + LOGICVC_IP_VERSION_ROFF);
	data->major = (ip_ver >> LOGICVC_MAJOR_REVISION_SHIFT) &
		      LOGICVC_MAJOR_REVISION_MASK;
	data->minor = (ip_ver >> LOGICVC_MINOR_REVISION_SHIFT) &
		      LOGICVC_MINOR_REVISION_MASK;
	data->patch = ip_ver & LOGICVC_PATCH_LEVEL_MASK;
	dev_info(dev, "logiCVC IP core %d.%02d.%c\n",
		 data->major, data->minor, ('a' + data->patch));

	if (data->major >= 4)
		data->flags |= XYLONFB_FLAGS_DYNAMIC_LAYER_ADDRESS;
	if (data->major >= 5){
		data->max_h_res = 8192;
		data->max_v_res = 8192;
	}
	else{
		data->max_h_res = 2048;
		data->max_v_res = 2048;
	}
	layers = data->layers;
	if (layers == 0) {
		dev_err(dev, "no available layers\n");
		return -ENODEV;
	}
	console_layer = data->console_layer;
	if (console_layer >= layers) {
		dev_err(dev, "invalid console layer ID\n");
		console_layer = 0;
	}

	if (data->flags & XYLONFB_FLAGS_CHECK_CONSOLE_LAYER) {
		if (!xylonfb_allow_console(data->fd[console_layer])) {
			dev_err(dev, "invalid console layer format\n");
			return -EINVAL;
		}
		data->flags &= ~XYLONFB_FLAGS_CHECK_CONSOLE_LAYER;
	}

	size = sizeof(struct fb_info *);
	afbi = devm_kzalloc(dev, (size * layers), GFP_KERNEL);
	if (!afbi) {
		dev_err(dev, "failed allocate internal data\n");
		return -ENOMEM;
	}

	if (data->flags & XYLONFB_FLAGS_READABLE_REGS) {
		data->reg_access.get_reg_val = xylonfb_get_reg;
		data->reg_access.set_reg_val = xylonfb_set_reg;
	} else {
		size = sizeof(struct xylonfb_registers);
		data->reg_access.get_reg_val = xylonfb_get_reg_mem;
		data->reg_access.set_reg_val = xylonfb_set_reg_mem;
	}

	data->coeff.cyr = LOGICVC_COEFF_Y_R;
	data->coeff.cyg = LOGICVC_COEFF_Y_G;
	data->coeff.cyb = LOGICVC_COEFF_Y_B;
	if (data->flags & XYLONFB_FLAGS_DISPLAY_INTERFACE_ITU656) {
		data->coeff.cy = LOGICVC_COEFF_ITU656_Y;
		data->coeff.cur = LOGICVC_COEFF_ITU656_U_R;
		data->coeff.cug = LOGICVC_COEFF_ITU656_U_G;
		data->coeff.cub = LOGICVC_COEFF_ITU656_U_B;
		data->coeff.cvr = LOGICVC_COEFF_ITU656_V_R;
		data->coeff.cvg = LOGICVC_COEFF_ITU656_V_G;
		data->coeff.cvb = LOGICVC_COEFF_ITU656_V_B;
	} else {
		data->coeff.cy = LOGICVC_COEFF_Y;
		data->coeff.cur = LOGICVC_COEFF_U_R;
		data->coeff.cug = LOGICVC_COEFF_U_G;
		data->coeff.cub = LOGICVC_COEFF_U_B;
		data->coeff.cvr = LOGICVC_COEFF_V_R;
		data->coeff.cvg = LOGICVC_COEFF_V_G;
		data->coeff.cvb = LOGICVC_COEFF_V_B;
	}

	atomic_set(&data->refcount, 0);

	data->flags |= XYLONFB_FLAGS_VMODE_INIT;

	sprintf(data->vm.name, "%s-%d@%d",
		data->vm.name, data->fd[console_layer]->bpp,
		data->vm.vmode.refresh);
	if (!(data->flags & XYLONFB_FLAGS_VMODE_CUSTOM))
		xylonfb_mode_option = data->vm.name;
	xylonfb_get_vmode_opts(data);

	if (data->pixel_clock) {
		if (xylonfb_hw_pixclk_supported(dev, data->pixel_clock)) {
			data->flags |= XYLONFB_FLAGS_PIXCLK_VALID;
		} else {
			dev_warn(dev, "pixel clock not supported\n");
			ret = -EPROBE_DEFER;
			goto err_probe;
		}
	} else {
		dev_info(dev, "external pixel clock\n");
	}

	ld = NULL;

	for (i = 0; i < layers; i++)
		regfb[i] = -1;
	memmap = true;

	/*
	 * /dev/fb0 will be default console layer,
	 * no matter how logiCVC layers are sorted in memory
	 */
	for (i = console_layer; i < layers; i++) {
		if (regfb[i] != -1)
			continue;

		size = sizeof(struct xylonfb_layer_data);
		fbi = framebuffer_alloc(size, dev);
		if (!fbi) {
			dev_err(dev, "failed allocate fb info\n");
			ret = -ENOMEM;
			goto err_probe;
		}
		fbi->dev = dev;
		afbi[i] = fbi;

		ld = fbi->par;
		ld->data = data;
		ld->fd = data->fd[i];

		atomic_set(&ld->refcount, 0);

		ld->pbase = data->resource_mem.start + layer_base_off[i];
		ld->base = dev_base + layer_base_off[i];
		ld->clut_base = dev_base + clut_base_off[i];

#if defined(CONFIG_FB_XYLON_MISC)
		xylonfb_misc_init(fbi);
#endif

		ret = xylonfb_vmem_init(ld, i, &memmap);
		if (ret)
			goto err_probe;

		xylonfb_layer_initialize(ld);

		ret = xylonfb_register_fb(fbi, ld, i, &regfb[i]);
		if (ret)
			goto err_probe;

		if (console_layer >= 0)
			fbi->monspecs = afbi[console_layer]->monspecs;

		mutex_init(&ld->mutex);

		XYLONFB_DBG(INFO, "Layer parameters\n" \
			    "    ID %d\n" \
			    "    Width %d pixels\n" \
			    "    Height %d lines\n" \
			    "    Bits per pixel %d\n" \
			    "    Buffer size %d bytes\n", \
			    ld->fd->id,
			    ld->fd->width,
			    ld->fd->height,
			    ld->fd->bpp,
			    ld->fb_size);

		if (console_layer > 0) {
			i = -1;
			console_layer = -1;
		}
	}

	if (ld) {
		if (!(data->flags & XYLONFB_FLAGS_READABLE_REGS))
			data->reg_access.set_reg_val(0xFFFF, dev_base,
						     LOGICVC_INT_MASK_ROFF,
						     ld);
	} else {
		dev_warn(dev, "initialization not completed\n");
	}

	if (data->flags & XYLONFB_FLAGS_BACKGROUND_LAYER)
		dev_info(dev, "BG layer: %s@%dbpp",
			 data->flags & XYLONFB_FLAGS_BACKGROUND_LAYER_RGB ? \
			 "RGB" : "YUV", data->bg_layer_bpp);

	mutex_init(&data->irq_mutex);
	init_waitqueue_head(&data->vsync.wait);
	atomic_set(&data->refcount, 0);

	dev_set_drvdata(dev, (void *)afbi);

	data->flags &= ~(XYLONFB_FLAGS_VMODE_INIT |
			 XYLONFB_FLAGS_VMODE_DEFAULT | XYLONFB_FLAGS_VMODE_SET);
	xylonfb_mode_option = NULL;

	xylonfb_start(afbi, layers);

	return 0;

err_probe:
	for (i = layers - 1; i >= 0; i--) {
		fbi = afbi[i];
		if (!fbi)
			continue;
		ld = fbi->par;
		if (regfb[i] == 0)
			unregister_framebuffer(fbi);
		else
			regfb[i] = 0;
		if (fbi->cmap.red)
			fb_dealloc_cmap(&fbi->cmap);
		if (ld) {
			if (data->flags & XYLONFB_FLAGS_DMA_BUFFER) {
				if (ld->fb_base)
					dma_free_coherent(dev,
						PAGE_ALIGN(ld->fb_size),
						ld->fb_base, ld->fb_pbase);
			} else {
				if (ld->fb_base)
					iounmap((void __iomem *)ld->fb_base);
			}
			kfree(fbi->pseudo_palette);
			framebuffer_release(fbi);
		}
	}

	return ret;
}

int xylonfb_deinit_core(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fb_info **afbi = dev_get_drvdata(dev);
	struct fb_info *fbi = afbi[0];
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	int i;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (atomic_read(&data->refcount) != 0) {
		dev_err(dev, "driver in use\n");
		return -EINVAL;
	}

	xylonfb_disable_logicvc_output(fbi);

#if defined(CONFIG_FB_XYLON_MISC)
	xylonfb_misc_deinit(fbi);
#endif

	xylonfb_hw_pixclk_unload(data->pixel_clock);

	for (i = data->layers - 1; i >= 0; i--) {
		fbi = afbi[i];
		ld = fbi->par;

		unregister_framebuffer(fbi);
		fb_dealloc_cmap(&fbi->cmap);
		if (data->flags & XYLONFB_FLAGS_DMA_BUFFER) {
			dma_free_coherent(dev,
					  PAGE_ALIGN(ld->fb_size),
					  ld->fb_base, ld->fb_pbase);
		}
		kfree(fbi->pseudo_palette);
		framebuffer_release(fbi);
	}

	return 0;
}
