/*
 * Xylon logiCVC frame buffer Open Firmware driver
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

#include <linux/errno.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <video/of_display_timing.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include "xylonfb_core.h"
#include "logicvc.h"

static void xylonfb_init_ctrl(struct device_node *dn, enum display_flags flags,
			      u32 *ctrl)
{
	u32 ctrl_reg = (LOGICVC_CTRL_HSYNC | LOGICVC_CTRL_VSYNC |
			LOGICVC_CTRL_DATA_ENABLE);

	XYLONFB_DBG(INFO, "%s", __func__);

	if (of_property_read_bool(dn, "hsync-active-low") ||
	    (flags & DISPLAY_FLAGS_HSYNC_LOW))
		ctrl_reg |= LOGICVC_CTRL_HSYNC_INVERT;
	if (of_property_read_bool(dn, "vsync-active-low") ||
	    (flags & DISPLAY_FLAGS_VSYNC_LOW))
		ctrl_reg |= LOGICVC_CTRL_VSYNC_INVERT;
	if (of_property_read_bool(dn, "data-enable-active-low") ||
	    (flags & DISPLAY_FLAGS_DE_LOW))
		ctrl_reg |= LOGICVC_CTRL_DATA_ENABLE_INVERT;
	if (of_property_read_bool(dn, "pixel-data-invert"))
		ctrl_reg |= LOGICVC_CTRL_PIXEL_DATA_INVERT;
	if (of_property_read_bool(dn, "pixel-data-output-trigger-high") ||
	    (flags & DISPLAY_FLAGS_PIXDATA_POSEDGE))
		ctrl_reg |= LOGICVC_CTRL_PIXEL_DATA_TRIGGER_INVERT;

	*ctrl = ctrl_reg;
}

static int xylonfb_layer_set_format(struct xylonfb_layer_fix_data *fd,
				    struct device *dev)
{
	XYLONFB_DBG(INFO, "%s", __func__);

	switch (fd->type) {
	case LOGICVC_LAYER_ALPHA:
		fd->format = XYLONFB_FORMAT_A8;
		break;

	case LOGICVC_LAYER_RGB:
		switch (fd->bpp) {
		case 8:
			switch (fd->transparency) {
			case LOGICVC_ALPHA_CLUT_16BPP:
				fd->format = XYLONFB_FORMAT_C8;
				fd->format_clut = XYLONFB_FORMAT_CLUT_ARGB6565;
				break;
			case LOGICVC_ALPHA_CLUT_32BPP:
				fd->format = XYLONFB_FORMAT_C8;
				fd->format_clut = XYLONFB_FORMAT_CLUT_ARGB8888;
				break;
			case LOGICVC_ALPHA_LAYER:
				if (fd->component_swap)
					fd->format = XYLONFB_FORMAT_BGR233;
				else
					fd->format = XYLONFB_FORMAT_RGB332;
				break;
			case LOGICVC_ALPHA_PIXEL:
				if (fd->component_swap)
					fd->format = XYLONFB_FORMAT_ABGR3233;
				else
					fd->format = XYLONFB_FORMAT_ARGB3332;
				break;
			default:
				return -EINVAL;
			}
			break;
		case 16:
			switch (fd->transparency) {
			case LOGICVC_ALPHA_LAYER:
				if (fd->component_swap)
					fd->format = XYLONFB_FORMAT_BGR565;
				else
					fd->format = XYLONFB_FORMAT_RGB565;
				break;
			case LOGICVC_ALPHA_PIXEL:
				if (fd->component_swap)
					fd->format = XYLONFB_FORMAT_ABGR565;
				else
					fd->format = XYLONFB_FORMAT_ARGB565;
				break;
			}
			break;
		case 30:
			if (fd->transparency != LOGICVC_ALPHA_LAYER)
				return -EINVAL;
			if (fd->component_swap)
				fd->format = XYLONFB_FORMAT_XBGR2101010;
			else
				fd->format = XYLONFB_FORMAT_XRGB2101010;
			break;
		case 24:
		case 32:
			switch (fd->transparency) {
			case LOGICVC_ALPHA_LAYER:
				if (fd->component_swap)
					fd->format = XYLONFB_FORMAT_XBGR8888;
				else
					fd->format = XYLONFB_FORMAT_XRGB8888;
				break;
			case LOGICVC_ALPHA_PIXEL:
				if (fd->component_swap)
					fd->format = XYLONFB_FORMAT_ABGR8888;
				else
					fd->format = XYLONFB_FORMAT_ARGB8888;
				break;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
		break;

	case LOGICVC_LAYER_YUV:
		switch (fd->bpp) {
		case 8:
			if (fd->transparency != LOGICVC_ALPHA_CLUT_32BPP)
				return -EINVAL;

			fd->format = XYLONFB_FORMAT_C8;
			fd->format_clut = XYLONFB_FORMAT_CLUT_AYUV8888;
			break;
		case 16:
			if (fd->transparency != LOGICVC_ALPHA_LAYER)
				return -EINVAL;
			if (fd->component_swap)
				fd->format = XYLONFB_FORMAT_UYVY;
			else
				fd->format = XYLONFB_FORMAT_YUYV;
			break;
		case 20:
			if (fd->transparency != LOGICVC_ALPHA_LAYER)
				return -EINVAL;
			if (fd->component_swap)
				fd->format = XYLONFB_FORMAT_UYVY_121010;
			else
				fd->format = XYLONFB_FORMAT_YUYV_121010;
			break;
		case 30:
			if (fd->transparency != LOGICVC_ALPHA_LAYER)
				return -EINVAL;
			if (fd->component_swap)
				fd->format = XYLONFB_FORMAT_XVUY_2101010;
			else
				fd->format = XYLONFB_FORMAT_XYUV_2101010;
			break;
		case 24:
		case 32:
			switch (fd->transparency) {
				case LOGICVC_ALPHA_LAYER:
					if (fd->component_swap)
						fd->format = XYLONFB_FORMAT_XVUY;
					else
						fd->format = XYLONFB_FORMAT_XYUV;
					break;
				case LOGICVC_ALPHA_PIXEL:
					if (fd->component_swap)
						fd->format = XYLONFB_FORMAT_AVUY;
					else
						fd->format = XYLONFB_FORMAT_AYUV;
					break;
				default:
					return -EINVAL;
			}
			break;
		}
		break;
	default:
		dev_err(dev, "unsupported layer type\n");
		return -EINVAL;
	}
	switch (fd->format) {
	case XYLONFB_FORMAT_A8:
	case XYLONFB_FORMAT_C8:
	case XYLONFB_FORMAT_RGB332:
	case XYLONFB_FORMAT_BGR233:
		fd->bpp = 8;
		break;
	case XYLONFB_FORMAT_ARGB3332:
	case XYLONFB_FORMAT_ABGR3233:
	case XYLONFB_FORMAT_RGB565:
	case XYLONFB_FORMAT_BGR565:
	case XYLONFB_FORMAT_YUYV:
	case XYLONFB_FORMAT_UYVY:
		fd->bpp = 16;
		break;
	case XYLONFB_FORMAT_ARGB565:
	case XYLONFB_FORMAT_ABGR565:
	case XYLONFB_FORMAT_XRGB8888:
	case XYLONFB_FORMAT_XBGR8888:
	case XYLONFB_FORMAT_ARGB8888:
	case XYLONFB_FORMAT_ABGR8888:
	case XYLONFB_FORMAT_XRGB2101010:
	case XYLONFB_FORMAT_XBGR2101010:
	case XYLONFB_FORMAT_YUYV_121010:
	case XYLONFB_FORMAT_UYVY_121010:
	case XYLONFB_FORMAT_AYUV:
	case XYLONFB_FORMAT_AVUY:
	case XYLONFB_FORMAT_XYUV:
	case XYLONFB_FORMAT_XVUY:
	case XYLONFB_FORMAT_XYUV_2101010:
	case XYLONFB_FORMAT_XVUY_2101010:
		fd->bpp = 32;
		break;
	}

	return 0;
}

static int xylonfb_parse_layer_info(struct device_node *parent_dn,
				    struct xylonfb_data *data, int id)
{
	struct device *dev = &data->pdev->dev;
	struct device_node *dn;
	struct xylonfb_layer_fix_data *fd;
	int ret;
	char layer_name[10];
	const char *string;

	XYLONFB_DBG(INFO, "%s", __func__);

	snprintf(layer_name, sizeof(layer_name), "layer_%d", id);
	dn = of_get_child_by_name(parent_dn, layer_name);
	if (!dn)
		return 0;

	data->layers++;

	fd = devm_kzalloc(&data->pdev->dev,
			  sizeof(struct xylonfb_layer_fix_data), GFP_KERNEL);
	if (!fd) {
		dev_err(dev, "failed allocate layer fix data (%d)\n", id);
		return -ENOMEM;
	}

	data->fd[id] = fd;

	fd->id = id;

	ret = of_property_read_u32(dn, "address", &fd->address);
	if (ret && (ret != -EINVAL)) {
		dev_err(dev, "failed get address\n");
		return ret;
	}
	ret = of_property_read_u32_index(dn, "address", 1, &fd->address_range);

	ret = of_property_read_u32(dn, "buffer-offset", &fd->buffer_offset);
	if (ret && (ret != -EINVAL)) {
		dev_err(dev, "failed get buffer-offset\n");
		return ret;
	}

	ret = of_property_read_u32(dn, "bits-per-pixel", &fd->bpp);
	if (ret) {
		dev_err(dev, "failed get bits-per-pixel\n");
		return ret;
	}
	switch (fd->bpp) {
	case 8:
	case 16:
	case 20:
	case 30:
	case 24:
	case 32:
		break;
	default:
		dev_err(dev, "invalid bits-per-pixel value\n");
		return -EINVAL;
	}

	ret = of_property_read_string(dn, "type", &string);
	if (ret) {
		dev_err(dev, "failed get type\n");
		return ret;
	}
	if (!strcmp(string, "alpha")) {
		fd->type = LOGICVC_LAYER_ALPHA;
	} else if (!strcmp(string, "rgb")) {
		fd->type = LOGICVC_LAYER_RGB;
	} else if (!strcmp(string, "yuv")) {
		fd->type = LOGICVC_LAYER_YUV;
	} else {
		dev_err(dev, "unsupported layer type\n");
		return -EINVAL;
	}

	if (fd->type != LOGICVC_LAYER_ALPHA) {
		ret = of_property_read_string(dn, "transparency", &string);
		if (ret) {
			dev_err(dev, "failed get transparency\n");
			return ret;
		}
		if (!strcmp(string, "clut16")) {
			fd->transparency = LOGICVC_ALPHA_CLUT_16BPP;
		} else if (!strcmp(string, "clut32")) {
			fd->transparency = LOGICVC_ALPHA_CLUT_32BPP;
		} else if (!strcmp(string, "layer")) {
			fd->transparency = LOGICVC_ALPHA_LAYER;
		} else if (!strcmp(string, "pixel")) {
			fd->transparency = LOGICVC_ALPHA_PIXEL;
		} else {
			dev_err(dev, "unsupported layer transparency\n");
			return -EINVAL;
		}
	}

	if (of_property_read_bool(dn, "component-swap"))
		fd->component_swap = true;

	fd->width = data->pixel_stride;

	ret = xylonfb_layer_set_format(fd, dev);
	if (ret) {
		dev_err(dev, "failed set layer format\n");
		return ret;
	}

	of_node_put(dn);

	return id + 1;
}

static int xylon_parse_hw_info(struct device_node *dn,
			       struct xylonfb_data *data)
{
	struct device *dev = &data->pdev->dev;
	int ret;
	const char *string;

	XYLONFB_DBG(INFO, "%s", __func__);

	ret = of_property_read_u32(dn, "background-layer-bits-per-pixel",
				   &data->bg_layer_bpp);
	if (ret && (ret != -EINVAL)) {
		dev_err(dev, "failed get bg-layer-bits-per-pixel\n");
		return ret;
	} else if (ret == 0) {
		data->flags |= XYLONFB_FLAGS_BACKGROUND_LAYER;

		ret = of_property_read_string(dn, "background-layer-type",
					      &string);
		if (ret) {
			dev_err(dev, "failed get bg-layer-type\n");
			return ret;
		}
		if (!strcmp(string, "rgb")) {
			data->flags |= XYLONFB_FLAGS_BACKGROUND_LAYER_RGB;
		} else if (!strcmp(string, "yuv")) {
			data->flags |= XYLONFB_FLAGS_BACKGROUND_LAYER_YUV;
		} else {
			dev_err(dev, "unsupported bg layer type\n");
			return -EINVAL;
		}
	}

	if (of_property_read_bool(dn, "display-interface-itu656"))
		data->flags |= XYLONFB_FLAGS_DISPLAY_INTERFACE_ITU656;

	if (of_property_read_bool(dn, "readable-regs"))
	{
		data->flags |= XYLONFB_FLAGS_READABLE_REGS;
		dev_warn(dev, "logicvc registers readable\n");
	}
	else
		dev_warn(dev, "logicvc registers not readable\n");

	if (of_property_read_bool(dn, "size-position"))
		data->flags |= XYLONFB_FLAGS_SIZE_POSITION;
	else
		dev_warn(dev, "logicvc size-position disabled\n");

	ret = of_property_read_u32(dn, "pixel-stride", &data->pixel_stride);
	if (ret) {
		dev_err(dev, "failed get pixel-stride\n");
		return ret;
	}

	ret = of_property_read_u32(dn, "power-delay", &data->pwr_delay);
	if (ret && (ret != -EINVAL)) {
		dev_err(dev, "failed get power-delay\n");
		return ret;
	}

	ret = of_property_read_u32(dn, "signal-delay", &data->sig_delay);
	if (ret && (ret != -EINVAL)) {
		dev_err(dev, "failed get signal\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id logicvc_of_match[] = {
	{ .compatible = "xylon,logicvc-3.00.a" },
	{ .compatible = "xylon,logicvc-4.00.a" },
	{ .compatible = "xylon,logicvc-5.00.a" },
	{/* end of table */}
};

static int xylonfb_get_logicvc_configuration(struct xylonfb_data *data)
{
	struct device *dev = &data->pdev->dev;
	struct device_node *dn = data->device;
	const struct of_device_id *match;
	struct videomode vm;
	int i, ret;

	XYLONFB_DBG(INFO, "%s", __func__);

	match = of_match_node(logicvc_of_match, dn);
	if (!match) {
		dev_err(dev, "failed match logicvc\n");
		return -ENODEV;
	}

	ret = of_address_to_resource(dn, 0, &data->resource_mem);
	if (ret) {
		dev_err(dev, "failed get mem resource\n");
		return ret;
	}
	data->irq = of_irq_to_resource(dn, 0, &data->resource_irq);
	if (data->irq == 0) {
		dev_err(dev, "failed get irq resource\n");
		return ret;
	}

	ret = xylon_parse_hw_info(dn, data);
	if (ret)
		return ret;

	for (i = 0; i < LOGICVC_MAX_LAYERS; i++) {
		ret = xylonfb_parse_layer_info(dn, data, i);
		if (ret < 0)
			return ret;
		if (ret == 0)
			break;
	}

	if (data->flags & XYLONFB_FLAGS_BACKGROUND_LAYER &&
	    data->layers == LOGICVC_MAX_LAYERS) {
		data->flags &= ~XYLONFB_FLAGS_BACKGROUND_LAYER;
		data->layers--;
		if (data->console_layer == data->layers)
			data->console_layer--;

		dev_warn(dev, "invalid last layer configuration\n");
	}

	memset(&vm, 0, sizeof(vm));

	if (!(data->flags & XYLONFB_FLAGS_EDID_VMODE) &&
	    (data->vm.name[0] == 0)) {
		ret = of_get_videomode(dn, &vm, OF_USE_NATIVE_MODE);
		if (!ret) {
			fb_videomode_from_videomode(&vm, &data->vm.vmode);

			sprintf(data->vm.name, "%dx%d",
				data->vm.vmode.xres, data->vm.vmode.yres);
			XYLONFB_DBG(INFO, "%s native mode %s ", __func__, data->vm.name);

			data->flags |= XYLONFB_FLAGS_VMODE_CUSTOM;
		}
	}

	xylonfb_init_ctrl(dn, vm.flags, &data->vm.ctrl);

	return 0;
}

static int xylonfb_get_driver_configuration(struct xylonfb_data *data)
{
	struct device *dev = &data->pdev->dev;
	struct device_node *dn = data->pdev->dev.of_node;
	int ret;
	const char *string;

	XYLONFB_DBG(INFO, "%s", __func__);

	data->device = of_parse_phandle(dn, "device", 0);
	if (!data->device) {
		dev_err(dev, "failed get device\n");
		return -ENODEV;
	}

	data->pixel_clock = of_parse_phandle(dn, "clocks", 0);

	ret = of_property_read_u32(dn, "console-layer", &data->console_layer);
	if (ret && (ret != -EINVAL)) {
		dev_err(dev, "failed get console-layer\n");
		return ret;
	} else if (ret == 0) {
		data->flags |= XYLONFB_FLAGS_CHECK_CONSOLE_LAYER;
	}

	if (of_property_read_bool(dn, "vsync-irq"))
		data->flags |= XYLONFB_FLAGS_VSYNC_IRQ;

	if (of_property_read_bool(dn, "edid-video-mode")) {
		data->flags |= XYLONFB_FLAGS_EDID_VMODE;
		if (of_property_read_bool(dn, "edid-print"))
			data->flags |= XYLONFB_FLAGS_EDID_PRINT;
	} else {
		data->flags |= XYLONFB_FLAGS_ADV7511_SKIP;
	}

	ret = of_property_read_string(dn, "video-mode", &string);
	if (ret && (ret != -EINVAL) && !(data->flags & XYLONFB_FLAGS_EDID_VMODE)) {
		dev_err(dev, "failed get video-mode\n");
		return ret;
	} else if (ret == 0) {
		strcpy(data->vm.name, string);
		return 0;
	}

	if (of_property_read_bool(dn, "put-vscreeninfo-exact"))
		data->flags |= XYLONFB_FLAGS_PUT_VSCREENINFO_EXACT;

	return 0;
}

static int xylonfb_probe(struct platform_device *pdev)
{
	struct xylonfb_data *data;
	int ret;

	XYLONFB_DBG(INFO, "%s", __func__);

	data = devm_kzalloc(&pdev->dev, sizeof(struct xylonfb_data),
			     GFP_KERNEL);
	if (!data) {
		dev_err(&pdev->dev, "failed allocate init data\n");
		return -ENOMEM;
	}

	data->pdev = pdev;

	ret = xylonfb_get_driver_configuration(data);
	if (ret)
		goto xylonfb_probe_error;

	ret = xylonfb_get_logicvc_configuration(data);
	if (ret)
		goto xylonfb_probe_error;

	ret = xylonfb_init_core(data);

xylonfb_probe_error:
	return ret;
}

static int xylonfb_remove(struct platform_device *pdev)
{
	XYLONFB_DBG(INFO, "%s", __func__);

	return xylonfb_deinit_core(pdev);
}

static const struct of_device_id xylonfb_of_match[] = {
	{ .compatible = "xylon,fb-3.00.a" },
	{ .compatible = "xylon,fb-4.00.a" },
	{ .compatible = "xylon,fb-4.03" },
	{/* end of table */},
};
MODULE_DEVICE_TABLE(of, xylonfb_of_match);

static struct platform_driver xylonfb_driver = {
	.probe = xylonfb_probe,
	.remove = xylonfb_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = XYLONFB_DEVICE_NAME,
		.of_match_table = xylonfb_of_match,
	},
};

static int xylonfb_get_params(char *options)
{
	char *this_opt;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (!options || !*options)
		return 0;

	while ((this_opt = strsep(&options, ",")) != NULL) {
		if (!*this_opt)
			continue;
		xylonfb_mode_option = this_opt;
	}
	return 0;
}

static int xylonfb_init(void)
{
	char *option = NULL;
	/*
	 *  Kernel boot options (in 'video=xxxfb:<options>' format)
	 */
	if (fb_get_options(XYLONFB_DRIVER_NAME, &option))
		return -ENODEV;
	/* Set internal module parameters */
	xylonfb_get_params(option);

	if (platform_driver_register(&xylonfb_driver)) {
		pr_err("failed %s driver registration\n", XYLONFB_DRIVER_NAME);
		return -ENODEV;
	}

	return 0;
}

static void __exit xylonfb_exit(void)
{
	platform_driver_unregister(&xylonfb_driver);
}

module_init(xylonfb_init);
module_exit(xylonfb_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION(XYLONFB_DRIVER_DESCRIPTION);
MODULE_VERSION(XYLONFB_DRIVER_VERSION);
