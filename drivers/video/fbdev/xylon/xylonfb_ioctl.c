/*
 * Xylon logiCVC frame buffer driver IOCTL functionality
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

#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <uapi/linux/xylonfb.h>

#include "logicvc.h"
#include "xylonfb_core.h"
#if defined(CONFIG_FB_XYLON_MISC)
#include "xylonfb_misc.h"
#endif

static int xylonfb_get_vblank(struct fb_vblank *vblank, struct fb_info *fbi)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;

	vblank->flags |= (FB_VBLANK_HAVE_VSYNC | FB_VBLANK_HAVE_COUNT);
	vblank->count = data->vsync.count;
	return 0;
}

static void xylonfb_vsync_ctrl(struct fb_info *fbi, bool enable)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	u32 imr;

	mutex_lock(&data->irq_mutex);

	imr = data->reg_access.get_reg_val(data->dev_base,
					   LOGICVC_INT_MASK_ROFF, ld);
	if (enable) {
		imr &= (~LOGICVC_INT_V_SYNC);
		writel(LOGICVC_INT_V_SYNC,
		       data->dev_base + LOGICVC_INT_STAT_ROFF);
	} else {
		imr |= LOGICVC_INT_V_SYNC;
	}

	data->reg_access.set_reg_val(imr, data->dev_base,
				     LOGICVC_INT_MASK_ROFF, ld);

	mutex_unlock(&data->irq_mutex);
}

int xylonfb_vsync_wait(u32 crt, struct fb_info *fbi)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	int ret, count;

	mutex_lock(&data->irq_mutex);

	count = data->vsync.count;

	ret = wait_event_interruptible_timeout(data->vsync.wait,
					       (count != data->vsync.count),
					       HZ/10);

	mutex_unlock(&data->irq_mutex);

	if (ret < 0)
		return ret;
	else if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

static unsigned int alpha_normalized(unsigned int alpha, unsigned int used_bits,
				     bool set)
{
	unsigned int val, x = 0;
	if (set) {
		return alpha / (1023 / ((1 << used_bits) - 1));
	}
	else {
		if(used_bits == 6)
			x = 1;
		val = ((((1023 << 16) / ((1 << used_bits) - 1))+x) * alpha) >> 16;
		return val;
	}
}

static int xylonfb_layer_alpha(struct xylonfb_layer_data *ld, u16 *alpha,
			       bool set)
{
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	unsigned int used_bits;
	u32 val;

	if (fd->transparency != LOGICVC_ALPHA_LAYER)
		return -EPERM;

	switch (fd->type) {
	case LOGICVC_LAYER_YUV:
		switch (fd->format){
		case XYLONFB_FORMAT_YUYV:
		case XYLONFB_FORMAT_UYVY:
		case XYLONFB_FORMAT_XYUV:
		case XYLONFB_FORMAT_XVUY:
			used_bits = 8;
			break;
		case XYLONFB_FORMAT_YUYV_121010:
		case XYLONFB_FORMAT_UYVY_121010:
		case XYLONFB_FORMAT_XYUV_2101010:
		case XYLONFB_FORMAT_XVUY_2101010:
			used_bits = 10;
			break;
		}
		break;
	case LOGICVC_LAYER_RGB:
		switch (fd->format){
		case XYLONFB_FORMAT_RGB332:
		case XYLONFB_FORMAT_BGR233:
			used_bits = 3;
			break;
		case XYLONFB_FORMAT_RGB565:
		case XYLONFB_FORMAT_BGR565:
			used_bits = 6;
			break;
		case XYLONFB_FORMAT_XRGB8888:
		case XYLONFB_FORMAT_XBGR8888:
			used_bits = 8;
			break;
		case XYLONFB_FORMAT_XRGB2101010:
		case XYLONFB_FORMAT_XBGR2101010:
			used_bits = 10;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (!set) {
		val = data->reg_access.get_reg_val(ld->base,
						   LOGICVC_LAYER_ALPHA_ROFF,
						   ld);
		*alpha = (u16)(val & (0x03FF >> (10 - used_bits)));
	}

	/* get/set normalized alpha value */
	*alpha = alpha_normalized(*alpha, used_bits, set);

	if (set)
		data->reg_access.set_reg_val(*alpha, ld->base,
					     LOGICVC_LAYER_ALPHA_ROFF,
					     ld);

	return 0;
}

static int xylonfb_layer_buff(struct fb_info *fbi,
			      struct xylonfb_layer_buffer *layer_buff,
			      bool set)
{
	struct xylonfb_layer_data *ld = fbi->par;
	unsigned int layer_id = ld->fd->id;
	u32 reg;

	if (set) {
		if (layer_buff->id >= LOGICVC_MAX_LAYER_BUFFERS)
			return -EINVAL;

		reg = readl(ld->data->dev_base + LOGICVC_VBUFF_SELECT_ROFF);
		reg |= (1 << (10 + layer_id));
		reg &= ~(0x03 << (layer_id << 1));
		reg |= (layer_buff->id << (layer_id << 1));
		writel(reg, ld->data->dev_base + LOGICVC_VBUFF_SELECT_ROFF);

		xylonfb_vsync_wait(0, fbi);
	} else {
		reg = readl(ld->data->dev_base + LOGICVC_VBUFF_SELECT_ROFF);
		reg >>= ((layer_id << 1));
		layer_buff->id = reg & 0x03;
	}

	return 0;
}

static void xylonfb_rgb_yuv(u32 c1, u32 c2, u32 c3, u32 *pixel,
			    struct xylonfb_layer_data *ld, bool rgb2yuv)
{
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	u32 r, g, b, y, u, v;

	if (rgb2yuv) {
		switch (fd->format){
		case XYLONFB_FORMAT_C8:
		case XYLONFB_FORMAT_YUYV:
		case XYLONFB_FORMAT_UYVY:
		case XYLONFB_FORMAT_AYUV:
		case XYLONFB_FORMAT_AVUY:
		case XYLONFB_FORMAT_XYUV:
		case XYLONFB_FORMAT_XVUY:
			y = ((data->coeff.cyr * c1) + (data->coeff.cyg * c2) +
				 (data->coeff.cyb * c3) + data->coeff.cy) /
				 LOGICVC_YUV_NORM;
			u = ((-data->coeff.cur * c1) - (data->coeff.cug * c2) +
				 (data->coeff.cub * c3) + LOGICVC_COEFF_U) /
				 LOGICVC_YUV_NORM;
			v = ((data->coeff.cvr * c1) - (data->coeff.cvg * c2) -
				 (data->coeff.cvb * c3) + LOGICVC_COEFF_V) /
				 LOGICVC_YUV_NORM;

			*pixel = (0xFF << 24) | (y << 16) | (u << 8) | v;
			break;
		case XYLONFB_FORMAT_XYUV_2101010:
		case XYLONFB_FORMAT_XVUY_2101010:
		case XYLONFB_FORMAT_YUYV_121010:
		case XYLONFB_FORMAT_UYVY_121010:
			y = ((data->coeff.cyr * c1) + (data->coeff.cyg * c2) +
				 (data->coeff.cyb * c3) + data->coeff.cy) /
				 LOGICVC_YUV_NORM;
			u = ((-data->coeff.cur * c1) - (data->coeff.cug * c2) +
				 (data->coeff.cub * c3) + LOGICVC_COEFF_U) /
				 LOGICVC_YUV_NORM;
			v = ((data->coeff.cvr * c1) - (data->coeff.cvg * c2) -
				 (data->coeff.cvb * c3) + LOGICVC_COEFF_V) /
				 LOGICVC_YUV_NORM;

			*pixel = (0xFF <<30) | (y << 20) | (u << 10) | v;
			break;
		}
	} else {
		r = ((c1 * LOGICVC_RGB_NORM) + (LOGICVC_COEFF_R_U * c2) -
		     LOGICVC_COEFF_R) / LOGICVC_RGB_NORM;
		g = ((c1 * LOGICVC_RGB_NORM) - (LOGICVC_COEFF_G_U * c2) -
		     (LOGICVC_COEFF_G_V * c3) + LOGICVC_COEFF_G) /
		     LOGICVC_RGB_NORM;
		b = ((c1 * LOGICVC_RGB_NORM) - (LOGICVC_COEFF_B_V * c3) -
		     LOGICVC_COEFF_B) / LOGICVC_RGB_NORM;

		*pixel = (0xFF << 24) | (r << 16) | (g << 8) | b;
	}
}

static int xylonfb_layer_color_rgb(struct xylonfb_layer_data *ld,
				   struct xylonfb_layer_color *layer_color,
				   unsigned int reg_offset, bool set)
{
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	void __iomem *base;
	u32 r = 0, g = 0, b = 0;
	u32 raw_rgb, y, u, v;
	int bpp, format, format_clut;

	if (reg_offset == LOGICVC_LAYER_TRANSP_COLOR_ROFF) {
		base = ld->base;
		bpp = fd->bpp;
		format_clut = fd->format_clut;
		format = fd->format;
	} else /* if (reg_offset == LOGICVC_BACKGROUND_COLOR_ROFF) */ {
		base = data->dev_base;
		bpp = data->bg_layer_bpp;
		format_clut = fd->format_clut;
		format = fd->format;
	}

	if (set) {
		if (layer_color->use_raw) {
			raw_rgb = layer_color->raw_rgb;
		} else if (data->flags & XYLONFB_FLAGS_BACKGROUND_LAYER_YUV) {
			r = layer_color->r;
			g = layer_color->g;
			b = layer_color->b;
			xylonfb_rgb_yuv(r, g, b, &raw_rgb, ld, true);
		} else {
			r = layer_color->r;
			g = layer_color->g;
			b = layer_color->b;
check_format_set:
			switch (format) {
			case XYLONFB_FORMAT_C8:
				switch (format_clut) {
				case XYLONFB_FORMAT_CLUT_ARGB6565:
					format = XYLONFB_FORMAT_ARGB565;
					goto check_format_get;
					break;
				case XYLONFB_FORMAT_CLUT_ARGB8888:
					format = XYLONFB_FORMAT_ABGR8888;
					goto check_format_get;
					break;
				case XYLONFB_FORMAT_CLUT_AYUV8888:
					format = XYLONFB_FORMAT_AYUV;
					goto check_format_set;
					break;
				default:
					return -EINVAL;
				}
				break;
			case XYLONFB_FORMAT_RGB332:
			case XYLONFB_FORMAT_BGR233:
				raw_rgb = (r & 0x0380) |
					   ((g & 0x0380) >> 3) |
					   ((b & 0x0300) >> 6);
				break;
			case XYLONFB_FORMAT_ARGB3332:
			case XYLONFB_FORMAT_ABGR3233:
				raw_rgb = (r & 0xE0) |
				   ((g & 0xE0) >> 3) |
				   ((b & 0xC0) >> 6);
				break;
			case XYLONFB_FORMAT_RGB565:
			case XYLONFB_FORMAT_BGR565:
				raw_rgb = ((r & 0xF8) << 8) |
				   ((g & 0xFC) << 3) |
				   ((b & 0xF8) >> 3);
			   break;
			case XYLONFB_FORMAT_XRGB8888:
			case XYLONFB_FORMAT_XBGR8888:
			case XYLONFB_FORMAT_ARGB8888:
			case XYLONFB_FORMAT_ABGR8888:
			case XYLONFB_FORMAT_YUYV:
			case XYLONFB_FORMAT_UYVY:
			case XYLONFB_FORMAT_XYUV:
			case XYLONFB_FORMAT_XVUY:
			case XYLONFB_FORMAT_AYUV:
			case XYLONFB_FORMAT_AVUY:
				raw_rgb = (r << 16) | (g << 8) | b;
				break;
			case XYLONFB_FORMAT_ARGB565:
			case XYLONFB_FORMAT_ABGR565:
				raw_rgb = ((r & 0xF8) << 8) |
					((g & 0xFC) << 3) |
					((b & 0xF8) >> 3);
				break;
			case XYLONFB_FORMAT_XRGB2101010:
			case XYLONFB_FORMAT_XBGR2101010:
			case XYLONFB_FORMAT_YUYV_121010:
			case XYLONFB_FORMAT_UYVY_121010:
			case XYLONFB_FORMAT_XYUV_2101010:
			case XYLONFB_FORMAT_XVUY_2101010:
				raw_rgb = ((r & 0x3FF) << 20) | ((r & 0x3FF) << 10) | (r & 0x3FF);
				break;
			default:
				raw_rgb = 0;
			}
		}
		data->reg_access.set_reg_val(raw_rgb, base, reg_offset, ld);
	} else {
		raw_rgb = data->reg_access.get_reg_val(base, reg_offset, ld);
check_format_get:
		if (data->flags & XYLONFB_FLAGS_BACKGROUND_LAYER_YUV) {
			y = (raw_rgb >> 16) & 0xFF;
			u = (raw_rgb >> 8) & 0xFF;
			v = raw_rgb & 0xFF;
			xylonfb_rgb_yuv(y, u, v, &raw_rgb, ld, false);
		} else {
			switch (format) {
			case XYLONFB_FORMAT_C8:
				switch (fd->format_clut) {
				case XYLONFB_FORMAT_CLUT_ARGB6565:
					format = XYLONFB_FORMAT_ARGB565;
					goto check_format_get;
					break;
				case XYLONFB_FORMAT_CLUT_ARGB8888:
					format = XYLONFB_FORMAT_ABGR8888;
					goto check_format_get;
					break;
				case XYLONFB_FORMAT_CLUT_AYUV8888:
					format = XYLONFB_FORMAT_AYUV;
					goto check_format_set;
					break;
				}
				break;
			case XYLONFB_FORMAT_RGB332:
			case XYLONFB_FORMAT_BGR233:
				r = raw_rgb >> 5;
				r = (((r << 3) | r) << 2) | (r >> 1);
				g = (raw_rgb >> 2) & 0x07;
				g = (((g << 3) | g) << 2) | (g >> 1);
				b = raw_rgb & 0x03;
				b = (b << 6) | (b << 4) | (b << 2) | b;
				break;
			case XYLONFB_FORMAT_ARGB3332:
			case XYLONFB_FORMAT_ABGR3233:
				r = raw_rgb >> 5;
				r = (((r << 3) | r) << 2) | (r >> 1);
				g = (raw_rgb >> 2) & 0x07;
				g = (((g << 3) | g) << 2) | (g >> 1);
				b = raw_rgb & 0x03;
				b = (b << 6) | (b << 4) | (b << 2) | b;
				break;
			case XYLONFB_FORMAT_RGB565:
			case XYLONFB_FORMAT_BGR565:
				r = raw_rgb >> 11;
				r = (r << 3) | (r >> 2);
				g = (raw_rgb >> 5) & 0x3F;
				g = (g << 2) | (g >> 4);
				b = raw_rgb & 0x1F;
				b = (b << 3) | (b >> 2);
				break;
			case XYLONFB_FORMAT_XRGB8888:
			case XYLONFB_FORMAT_XBGR8888:
			case XYLONFB_FORMAT_ARGB8888:
			case XYLONFB_FORMAT_ABGR8888:
			case XYLONFB_FORMAT_YUYV:
			case XYLONFB_FORMAT_UYVY:
			case XYLONFB_FORMAT_XYUV:
			case XYLONFB_FORMAT_XVUY:
			case XYLONFB_FORMAT_AYUV:
			case XYLONFB_FORMAT_AVUY:
				r = raw_rgb >> 16;
				g = (raw_rgb >> 8) & 0xFF;
				b = raw_rgb & 0xFF;
				break;
			case XYLONFB_FORMAT_ARGB565:
			case XYLONFB_FORMAT_ABGR565:
				r = raw_rgb >> 11;
				r = (r << 3) | (r >> 2);
				g = (raw_rgb >> 5) & 0x3F;
				g = (g << 2) | (g >> 4);
				b = raw_rgb & 0x1F;
				b = (b << 3) | (b >> 2);
				break;
			case XYLONFB_FORMAT_XRGB2101010:
			case XYLONFB_FORMAT_XBGR2101010:
			case XYLONFB_FORMAT_YUYV_121010:
			case XYLONFB_FORMAT_UYVY_121010:
			case XYLONFB_FORMAT_XYUV_2101010:
			case XYLONFB_FORMAT_XVUY_2101010:
				r = raw_rgb >> 20;
				g = (raw_rgb >> 10) & 0x3FF;
				b = raw_rgb & 0x3FF;
				break;
			default:
				raw_rgb = r = g = b = 0;
			}
		}
		layer_color->raw_rgb = raw_rgb;
		layer_color->r = r;
		layer_color->g = g;
		layer_color->b = b;
	}

	return 0;
}

static int xylonfb_layer_geometry(struct fb_info *fbi,
				  struct xylonfb_layer_geometry *layer_geometry,
				  bool set)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	u32 x, y, width, height, xoff, yoff, xres, yres;

	xres = fbi->var.xres;
	yres = fbi->var.yres;

	if (set) {
		x = layer_geometry->x;
		y = layer_geometry->y;
		width = layer_geometry->width;
		height = layer_geometry->height;

		if ((x > xres) || (y > yres))
			return -EINVAL;

		if ((width == 0) || (height == 0))
			return -EINVAL;

		if ((x + width) > xres) {
			width = xres - x;
			layer_geometry->width = width;
		}
		if ((y + height) > yres) {
			height = yres - y;
			layer_geometry->height = height;
		}
		/* YUV 4:2:2 layer type can only have even layer width */
		if ((width > 2) && (fd->format == XYLONFB_FORMAT_YUYV ||
			fd->format == XYLONFB_FORMAT_UYVY ||
			fd->format == XYLONFB_FORMAT_YUYV_121010 ||
			fd->format == XYLONFB_FORMAT_UYVY_121010))
			width &= ~((unsigned long) + 1);

		/*
		 * logiCVC 3.x registers write sequence:
		 * offset, size, position with implicit last write to
		 * LOGICVC_LAYER_VPOS_ROFF
		 * logiCVC 4.x registers write sequence:
		 * size, position with implicit last write to
		 * LOGICVC_LAYER_ADDR_ROFF
		 */
		if (!(data->flags & XYLONFB_FLAGS_DYNAMIC_LAYER_ADDRESS)) {
			data->reg_access.set_reg_val(layer_geometry->x_offset,
						     ld->base,
						     LOGICVC_LAYER_HOFF_ROFF,
						     ld);
			data->reg_access.set_reg_val(layer_geometry->y_offset,
						     ld->base,
						     LOGICVC_LAYER_VOFF_ROFF,
						     ld);
		}
		data->reg_access.set_reg_val((width - 1), ld->base,
					     LOGICVC_LAYER_HSIZE_ROFF,
					     ld);
		data->reg_access.set_reg_val((height - 1), ld->base,
					     LOGICVC_LAYER_VSIZE_ROFF,
					     ld);
		data->reg_access.set_reg_val((xres - (x + 1)), ld->base,
					     LOGICVC_LAYER_HPOS_ROFF,
					     ld);
		data->reg_access.set_reg_val((yres - (y + 1)), ld->base,
					     LOGICVC_LAYER_VPOS_ROFF,
					     ld);
		if (data->flags & XYLONFB_FLAGS_DYNAMIC_LAYER_ADDRESS) {
			xoff = layer_geometry->x_offset * (ld->fd->bpp / 8);
			yoff = layer_geometry->y_offset * ld->fd->width *
			       (ld->fd->bpp / 8);

			ld->fb_pbase_active = ld->fb_pbase + xoff + yoff;

			data->reg_access.set_reg_val(ld->fb_pbase_active,
						     ld->base,
						     LOGICVC_LAYER_ADDR_ROFF,
						     ld);
		}
	} else {
		x = data->reg_access.get_reg_val(ld->base,
						 LOGICVC_LAYER_HPOS_ROFF,
						 ld);
		layer_geometry->x = xres - (x + 1);
		y = data->reg_access.get_reg_val(ld->base,
						 LOGICVC_LAYER_VPOS_ROFF,
						 ld);
		layer_geometry->y = yres - (y + 1);
		layer_geometry->width =
			data->reg_access.get_reg_val(ld->base,
						     LOGICVC_LAYER_HSIZE_ROFF,
						     ld);
		layer_geometry->width += 1;
		layer_geometry->height =
			data->reg_access.get_reg_val(ld->base,
						     LOGICVC_LAYER_VSIZE_ROFF,
						     ld);
		layer_geometry->height += 1;
	}

	return 0;
}

static int xylonfb_layer_reg_access(struct xylonfb_layer_data *ld,
				    struct xylonfb_hw_access *hw_access,
				    bool set)
{
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_fix_data *fd = ld->fd;
	u32 offset;
	u32 rel_offset;

	if ((hw_access->offset < LOGICVC_LAYER_BASE_OFFSET) ||
	    (hw_access->offset > LOGICVC_LAYER_BASE_END))
		return -EPERM;

	if (data->flags & XYLONFB_FLAGS_READABLE_REGS) {
		offset = hw_access->offset;
		if (set)
			data->reg_access.set_reg_val(hw_access->value,
						     data->dev_base,
						     offset,
						     ld);
		else
			hw_access->value =
				data->reg_access.get_reg_val(data->dev_base,
							     offset,
							     ld);
		return 0;
	}

	rel_offset = hw_access->offset - (fd->id * 0x80) -
		     LOGICVC_LAYER_BASE_OFFSET;

	if (rel_offset > LOGICVC_LAYER_BASE_END)
		return -EINVAL;

	if (set)
		data->reg_access.set_reg_val(hw_access->value, ld->base,
					     rel_offset, ld);
	else
		hw_access->value = data->reg_access.get_reg_val(ld->base,
								rel_offset, ld);

	return 0;
}

static int xylonfb_reload_registers(struct fb_info *fbi)
{
	struct fb_info **afbi = NULL;
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	struct xylonfb_layer_registers regs;
	void __iomem *dev_base = data->dev_base;
	int i;

	/* Reload layer registers */
	afbi = dev_get_drvdata(fbi->device);
	for (i = 0; i < data->layers; i++) {
		ld = afbi[i]->par;
		regs = ld->regs;
		ld->data->reg_access.set_reg_val(ld->fb_pbase, ld->base,
					     LOGICVC_LAYER_ADDR_ROFF,
					     ld);
		ld->data->reg_access.set_reg_val(regs.hpos, ld->base,
						 LOGICVC_LAYER_HPOS_ROFF,
						 ld);
		ld->data->reg_access.set_reg_val(regs.vpos, ld->base,
						 LOGICVC_LAYER_VPOS_ROFF,
						 ld);
		ld->data->reg_access.set_reg_val(regs.hsize, ld->base,
						 LOGICVC_LAYER_HSIZE_ROFF,
						 ld);
		ld->data->reg_access.set_reg_val(regs.vsize, ld->base,
						 LOGICVC_LAYER_VSIZE_ROFF,
						 ld);
		ld->data->reg_access.set_reg_val(regs.alpha, ld->base,
						 LOGICVC_LAYER_ALPHA_ROFF,
						 ld);
		ld->data->reg_access.set_reg_val(regs.ctrl, ld->base,
						 LOGICVC_LAYER_CTRL_ROFF,
						 ld);
		ld->data->reg_access.set_reg_val(regs.transp, ld->base,
						 LOGICVC_LAYER_TRANSP_COLOR_ROFF,
						 ld);
	}

	/* Reload common registers */
	writel(LOGICVC_INT_V_SYNC, dev_base + LOGICVC_INT_STAT_ROFF);
	data->reg_access.set_reg_val(data->regs.int_mask, dev_base,
					     LOGICVC_INT_MASK_ROFF, ld);

	/* Reload resolution */
	data->flags=0x11A34F;
	if(fbi->fbops->fb_set_par(fbi))
		return -EFAULT;

	return 0;
}

static int xylonfb_ctrl_reg_access(struct xylonfb_layer_data *ld,
				    struct xylonfb_hw_access *hw_access,
				    bool set)
{
	struct xylonfb_data *data = ld->data;
	u32 offset;

	if (hw_access->offset != LOGICVC_CTRL_ROFF)
		return -EPERM;

	offset = hw_access->offset;
	if (set)
		data->reg_access.set_reg_val(hw_access->value,
					     data->dev_base,
					     offset,
					     ld);
	else
		hw_access->value =
			data->reg_access.get_reg_val(data->dev_base,
						     offset,
						     ld);

	return 0;
}

static int xylonfb_int_stat_reg_access(struct xylonfb_layer_data *ld,
				    struct xylonfb_hw_access *hw_access,
				    bool set)
{
	struct xylonfb_data *data = ld->data;
	u32 offset;

	if (hw_access->offset != LOGICVC_INT_STAT_ROFF)
		return -EPERM;

	offset = hw_access->offset;
	if (set)
		writel(hw_access->value, data->dev_base + offset);
	else
		hw_access->value = readl(data->dev_base + offset);

	return 0;
}

int xylonfb_ioctl(struct fb_info *fbi, unsigned int cmd, unsigned long arg)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	union {
		struct fb_vblank vblank;
		struct xylonfb_hw_access hw_access;
		struct xylonfb_layer_buffer layer_buff;
		struct xylonfb_layer_color layer_color;
		struct xylonfb_layer_geometry layer_geometry;
		struct xylonfb_layer_transparency layer_transp;
	} ioctl;
	void __user *argp = (void __user *)arg;
	unsigned long val;
	u32 var32;
	int ret = 0;
	bool flag;

	switch (cmd) {
	case FBIOGET_VBLANK:
		if (copy_from_user(&ioctl.vblank, argp, sizeof(ioctl.vblank)))
			return -EFAULT;

		ret = xylonfb_get_vblank(&ioctl.vblank, fbi);
		if (!ret &&
		    copy_to_user(argp, &ioctl.vblank, sizeof(ioctl.vblank)))
			ret = -EFAULT;
		break;

	case FBIO_WAITFORVSYNC:
		if (get_user(var32, (u32 __user *)arg))
			return -EFAULT;

		ret = xylonfb_vsync_wait(var32, fbi);
		break;

	case XYLONFB_VSYNC_CTRL:
		if (get_user(flag, (u8 __user *)arg))
			return -EFAULT;

		xylonfb_vsync_ctrl(fbi, flag);
		break;

	case XYLONFB_LAYER_IDX:
		var32 = ld->fd->id;
		put_user(var32, (u32 __user *)arg);
		break;

	case XYLONFB_LAYER_ALPHA:
		if (copy_from_user(&ioctl.layer_transp, argp,
				   sizeof(ioctl.layer_transp)))
			return -EFAULT;

		mutex_lock(&ld->mutex);
		ret = xylonfb_layer_alpha(ld, &ioctl.layer_transp.alpha,
					  ioctl.layer_transp.set);
		if (!ret && !ioctl.layer_transp.set)
			if (copy_to_user(argp, &ioctl.layer_transp,
					 sizeof(ioctl.layer_transp)))
				ret = -EFAULT;
		mutex_unlock(&ld->mutex);
		break;

	case XYLONFB_LAYER_COLOR_TRANSP_CTRL:
		if (get_user(flag, (u8 __user *)arg))
			return -EFAULT;

		mutex_lock(&ld->mutex);
		var32 = data->reg_access.get_reg_val(ld->base,
						     LOGICVC_LAYER_CTRL_ROFF,
						     ld);
		if (flag)
			var32 |= LOGICVC_LAYER_CTRL_COLOR_TRANSPARENCY_DISABLE;
		else
			var32 &= ~LOGICVC_LAYER_CTRL_COLOR_TRANSPARENCY_DISABLE;
		data->reg_access.set_reg_val(var32, ld->base,
					     LOGICVC_LAYER_CTRL_ROFF,
					     ld);
		mutex_unlock(&ld->mutex);
		break;

	case XYLONFB_LAYER_COLOR_TRANSP:
		if (copy_from_user(&ioctl.layer_color, argp,
				   sizeof(ioctl.layer_color)))
			return -EFAULT;

		mutex_lock(&ld->mutex);
		ret = xylonfb_layer_color_rgb(ld, &ioctl.layer_color,
					      LOGICVC_LAYER_TRANSP_COLOR_ROFF,
					      ioctl.layer_color.set);
		if (!ret && !ioctl.layer_color.set)
			if (copy_to_user(argp, &ioctl.layer_color,
					 sizeof(ioctl.layer_color)))
				ret = -EFAULT;
		mutex_unlock(&ld->mutex);
		break;

	case XYLONFB_LAYER_GEOMETRY:
		if (!(data->flags & XYLONFB_FLAGS_SIZE_POSITION))
			return -EINVAL;

		if (copy_from_user(&ioctl.layer_geometry, argp,
				   sizeof(ioctl.layer_geometry)))
			return -EFAULT;

		mutex_lock(&ld->mutex);
		ret = xylonfb_layer_geometry(fbi, &ioctl.layer_geometry,
					     ioctl.layer_geometry.set);
		if (!ret && !ioctl.layer_geometry.set)
			if (copy_to_user(argp, &ioctl.layer_geometry,
					 sizeof(ioctl.layer_geometry)))
				ret = -EFAULT;
		mutex_unlock(&ld->mutex);
		break;

	case XYLONFB_LAYER_BUFFER:
		if (data->major >= 4)
			return -EPERM;

		if (copy_from_user(&ioctl.layer_buff, argp,
				   sizeof(ioctl.layer_buff)))
			return -EFAULT;

		mutex_lock(&ld->mutex);
		ret = xylonfb_layer_buff(fbi, &ioctl.layer_buff,
					 ioctl.layer_buff.set);
		if (!ret && !ioctl.layer_buff.set)
			if (copy_to_user(argp, &ioctl.layer_buff,
					 sizeof(ioctl.layer_buff)))
				ret = -EFAULT;
		mutex_unlock(&ld->mutex);
		break;

	case XYLONFB_LAYER_BUFFER_OFFSET:
		if (data->major < 4) {
			var32 = readl(ld->data->dev_base +
				      LOGICVC_VBUFF_SELECT_ROFF);
			var32 >>= (ld->fd->id << 1);
			var32 &= 0x03;
			val = ld->fd->buffer_offset;
			val *= var32;
		} else {
			val = ld->fd->buffer_offset;
		}
		put_user(val, (unsigned long __user *)arg);
		break;

	case XYLONFB_BACKGROUND_COLOR:
		if (data->bg_layer_bpp == 0)
			return -EPERM;

		if (copy_from_user(&ioctl.layer_color, argp,
				   sizeof(ioctl.layer_color)))
			return -EFAULT;

		mutex_lock(&ld->mutex);
		ret = xylonfb_layer_color_rgb(ld, &ioctl.layer_color,
					      LOGICVC_BACKGROUND_COLOR_ROFF,
					      ioctl.layer_color.set);
		if (!ret && !ioctl.layer_color.set)
			if (copy_to_user(argp, &ioctl.layer_color,
					 sizeof(ioctl.layer_color)))
				ret = -EFAULT;
		mutex_unlock(&ld->mutex);
		break;

	case XYLONFB_LAYER_EXT_BUFF_SWITCH:
		if (get_user(flag, (u8 __user *)arg))
			return -EFAULT;

		mutex_lock(&ld->mutex);
		var32 = data->reg_access.get_reg_val(ld->base,
						     LOGICVC_LAYER_CTRL_ROFF,
						     ld);
		if (flag)
			var32 |= LOGICVC_LAYER_CTRL_EXTERNAL_BUFFER_SWITCH;
		else
			var32 &= ~LOGICVC_LAYER_CTRL_EXTERNAL_BUFFER_SWITCH;
		data->reg_access.set_reg_val(var32, ld->base,
					     LOGICVC_LAYER_CTRL_ROFF,
					     ld);
		mutex_unlock(&ld->mutex);
		break;

	case XYLONFB_HW_ACCESS:
		if (copy_from_user(&ioctl.hw_access, argp,
				   sizeof(ioctl.hw_access)))
			return -EFAULT;

		ret = xylonfb_layer_reg_access(ld, &ioctl.hw_access,
					       ioctl.hw_access.set);
		if (!ret && !ioctl.hw_access.set)
			if (copy_to_user(argp, &ioctl.hw_access,
					 sizeof(ioctl.hw_access)))
				ret = -EFAULT;
		break;

	case XYLONFB_IP_CORE_VERSION:
		var32 = (data->major << 16) | (data->minor << 8) | data->patch;
		if (copy_to_user(argp, &var32, sizeof(u32)))
			ret = -EFAULT;
		break;

	case XYLONFB_RELOAD_REGISTERS:
		if (xylonfb_reload_registers(fbi))
		{
			ret = -EFAULT;
		}
		break;

	case XYLONFB_WAIT_EDID:
#if defined(CONFIG_FB_XYLON_MISC)
		if (data->flags & XYLONFB_FLAGS_EDID_READY)
			break;
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if ((val == 0) || (val < 0))
			val = XYLONFB_EDID_WAIT_TOUT;
		ret = wait_event_interruptible_timeout(data->misc.wait,
						       (data->flags & XYLONFB_FLAGS_EDID_READY),
						       (val * HZ));
		if (ret == 0)
			return -ETIMEDOUT;
		else
			ret = 0;
#else
			return -EPERM;
#endif
		break;

	case XYLONFB_GET_EDID:
#if defined(CONFIG_FB_XYLON_MISC)
		if (data->flags & XYLONFB_FLAGS_EDID_READY) {
			if (data->misc.edid) {
				if (copy_to_user(argp, data->misc.edid,
						 XYLONFB_EDID_SIZE))
					ret = -EFAULT;
			} else {
				return -EPERM;
			}
		} else {
			return -EPERM;
		}
#else
		return -EPERM;
#endif
		break;
		
	case XYLONFB_HW_ACCESS_CTRL_REG:
		if (copy_from_user(&ioctl.hw_access, argp,
				   sizeof(ioctl.hw_access)))
			return -EFAULT;

		ret = xylonfb_ctrl_reg_access(ld, &ioctl.hw_access,
					       ioctl.hw_access.set);
		if (!ret && !ioctl.hw_access.set)
			if (copy_to_user(argp, &ioctl.hw_access,
					 sizeof(ioctl.hw_access)))
				ret = -EFAULT;
		break;
		
	case XYLONFB_HW_ACCESS_INT_STAT_REG:
		if (copy_from_user(&ioctl.hw_access, argp,
				   sizeof(ioctl.hw_access)))
			return -EFAULT;

		ret = xylonfb_int_stat_reg_access(ld, &ioctl.hw_access,
					       ioctl.hw_access.set);
		if (!ret && !ioctl.hw_access.set)
			if (copy_to_user(argp, &ioctl.hw_access,
					 sizeof(ioctl.hw_access)))
				ret = -EFAULT;
		break;

	default:
		dev_err(&data->pdev->dev, "unknown ioctl");
		ret = -EINVAL;
	}

	return ret;
}
