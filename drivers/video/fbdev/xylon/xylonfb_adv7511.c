/*
 * Xylon logiCVC frame buffer driver miscellaneous ADV7511 functionality
 * interface for V4L2 adv7511 driver (Copyright 2012 Cisco Systems, Inc.
 * and/or its affiliates)
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

#include <linux/atomic.h>
#include <linux/completion.h>
#include <linux/console.h>
#include <linux/interrupt.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include "adv7511.h"
#include "xylonfb_adv7511.h"
#include "xylonfb_core.h"
#include "xylonfb_misc.h"

#define ADV7511_NAME "adv7511"

struct xylonfb_adv7511 {
	atomic_t edid_lock;
	struct completion edid_done;
	struct v4l2_device dev;
	struct v4l2_subdev *sd;
	struct v4l2_subdev_edid sd_edid;
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;
	struct fb_info *fbi;
	struct fb_var_screeninfo *var_screeninfo;
	struct fb_monspecs *monspecs;
	wait_queue_head_t *wait;
	u32 *xylonfb_flags;
	unsigned long timeout;
	u8 edid[256];
	bool init;
};

static struct xylonfb_adv7511 *adv7511;

static void xylonfb_adv7511_get_monspecs(u8 *edid, struct fb_monspecs *monspecs,
					 struct fb_var_screeninfo *var)
{
	XYLONFB_DBG(INFO, "%s", __func__);

	fb_edid_to_monspecs(edid, monspecs);

	if (*(adv7511->xylonfb_flags) & XYLONFB_FLAGS_EDID_PRINT) {
		pr_info("========================================\n");
		pr_info("Display Information (EDID)\n");
		pr_info("========================================\n");
		pr_info("Monitor: %s\n", monspecs->monitor);
		pr_info("Serial Number: %s\n", monspecs->serial_no);
		pr_info("EDID Version %d.%d\n",
			(int)monspecs->version, (int)monspecs->revision);
		pr_info("Manufacturer: %s\n", monspecs->manufacturer);
		pr_info("Model: %x\n", monspecs->model);
		pr_info("Serial Number: %u\n", monspecs->serial);
		pr_info("Year: %u Week %u\n", monspecs->year, monspecs->week);
		pr_info("Display Characteristics:\n");
		pr_info("   Monitor Operating Limits from EDID\n");
		pr_info("   H: %d-%dKHz V: %d-%dHz DCLK: %dMHz\n",
			monspecs->hfmin/1000, monspecs->hfmax/1000,
			monspecs->vfmin, monspecs->vfmax,
			monspecs->dclkmax/1000000);
		if (monspecs->input & FB_DISP_DDI) {
			pr_info("   Digital Display Input\n");
		} else {
			pr_info("   Analog Display Input:\n");
			pr_info("   Input Voltage:\n");
			if (monspecs->input & FB_DISP_ANA_700_300)
				pr_info("      0.700V/0.300V");
			else if (monspecs->input & FB_DISP_ANA_714_286)
				pr_info("      0.714V/0.286V");
			else if (monspecs->input & FB_DISP_ANA_1000_400)
				pr_info("      1.000V/0.400V");
			else if (monspecs->input & FB_DISP_ANA_700_000)
				pr_info("      0.700V/0.000V");
		}
		if (monspecs->signal) {
			pr_info("   Synchronization:\n");
			if (monspecs->signal & FB_SIGNAL_BLANK_BLANK)
				pr_info("      Blank to Blank\n");
			if (monspecs->signal & FB_SIGNAL_SEPARATE)
				pr_info("      Separate\n");
			if (monspecs->signal & FB_SIGNAL_COMPOSITE)
				pr_info("      Composite\n");
			if (monspecs->signal & FB_SIGNAL_SYNC_ON_GREEN)
				pr_info("      Sync on Green\n");
			if (monspecs->signal & FB_SIGNAL_SERRATION_ON)
				pr_info("      Serration on\n");
		}
		if (monspecs->max_x)
			pr_info("   Max H-size %dcm\n", monspecs->max_x);
		else
			pr_info("   Variable H-size\n");
		if (monspecs->max_y)
			pr_info("   Max V-size %dcm\n", monspecs->max_y);
		else
			pr_info("   Variable V-size\n");
		pr_info("   Display Gamma %d.%d\n",
			monspecs->gamma/100, monspecs->gamma % 100);
		pr_info("   DPMS: Active %s, Suspend %s, Standby %s\n",
			(monspecs->dpms & FB_DPMS_ACTIVE_OFF) ? "yes" : "no",
			(monspecs->dpms & FB_DPMS_SUSPEND)    ? "yes" : "no",
			(monspecs->dpms & FB_DPMS_STANDBY)    ? "yes" : "no");
		if (monspecs->input & FB_DISP_MONO)
			pr_info("   Monochrome/Grayscale\n");
		else if (monspecs->input & FB_DISP_RGB)
			pr_info("   RGB Color Display\n");
		else if (monspecs->input & FB_DISP_MULTI)
			pr_info("   Non-RGB Multicolor Display\n");
		else if (monspecs->input & FB_DISP_UNKNOWN)
			pr_info("   Unknown\n");
		pr_info("   Chromaticity coordinates:\n");
		pr_info("      RedX:   0.%03d\n", monspecs->chroma.redx);
		pr_info("      RedY:   0.%03d\n", monspecs->chroma.redy);
		pr_info("      GreenX: 0.%03d\n", monspecs->chroma.greenx);
		pr_info("      GreenY: 0.%03d\n", monspecs->chroma.greeny);
		pr_info("      BlueX:  0.%03d\n", monspecs->chroma.bluex);
		pr_info("      BlueY:  0.%03d\n", monspecs->chroma.bluey);
		pr_info("      WhiteX: 0.%03d\n", monspecs->chroma.whitex);
		pr_info("      WhiteY: 0.%03d\n", monspecs->chroma.whitey);
		if (monspecs->misc) {
			if (monspecs->misc & FB_MISC_PRIM_COLOR)
				pr_info("   Default primary color format\n");
			if (monspecs->misc & FB_MISC_1ST_DETAIL)
				pr_info("   First DETAILED Timing preferred\n");
			if (monspecs->gtf == 1)
				pr_info("   Display GTF capable\n");
		}
		pr_info("Monitor Timings\n");
		pr_info("   Resolution %dx%d\n", var->xres, var->yres);
		pr_info("   Pixel Clock %d MHz ",
			(int)PICOS2KHZ(var->pixclock)/1000);
		pr_info("   H sync:\n");
		pr_info("      Front porch %d Length %d Back porch %d\n",
			var->right_margin, var->hsync_len, var->left_margin);
		pr_info("   V sync:\n");
		pr_info("      Front porch %d Length %d Back porch %d\n",
			var->lower_margin, var->vsync_len, var->upper_margin);
		pr_info("   %sHSync %sVSync\n",
			(var->sync & FB_SYNC_HOR_HIGH_ACT) ? "+" : "-",
			(var->sync & FB_SYNC_VERT_HIGH_ACT) ? "+" : "-");
		pr_info("========================================\n");
	}
}

static void xylonfb_adv7511_set_v4l2_timings(struct v4l2_subdev *sd,
					     struct fb_var_screeninfo *var)
{
	struct v4l2_dv_timings dv_timings;

	XYLONFB_DBG(INFO, "%s", __func__);

	dv_timings.type = V4L2_DV_BT_656_1120;

	dv_timings.bt.width = var->xres;
	dv_timings.bt.height = var->yres;
	dv_timings.bt.interlaced = 0;
	dv_timings.bt.polarities = 0;
	if (var->sync & FB_SYNC_VERT_HIGH_ACT)
		dv_timings.bt.polarities |= V4L2_DV_VSYNC_POS_POL;
	if (var->sync & FB_SYNC_HOR_HIGH_ACT)
		dv_timings.bt.polarities |= V4L2_DV_HSYNC_POS_POL;
	dv_timings.bt.pixelclock = (__u64)PICOS2KHZ(var->pixclock) * 1000;
	dv_timings.bt.hfrontporch = var->right_margin;
	dv_timings.bt.hsync = var->hsync_len;
	dv_timings.bt.hbackporch = var->left_margin;
	dv_timings.bt.vfrontporch = var->lower_margin;
	dv_timings.bt.vsync = var->vsync_len;
	dv_timings.bt.vbackporch = var->upper_margin;
	dv_timings.bt.il_vfrontporch = 0;
	dv_timings.bt.il_vsync = 0;
	dv_timings.bt.il_vbackporch = 0;
	dv_timings.bt.standards = 0;
	dv_timings.bt.standards = V4L2_DV_BT_STD_DMT | V4L2_DV_BT_STD_CEA861;
	dv_timings.bt.flags = 0;

	sd->ops->video->s_dv_timings(sd, &dv_timings);
}

static int xylonfb_adv7511_update(struct fb_info *fbi)
{
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_misc_data *misc = &ld->data->misc;
	int ret;

	XYLONFB_DBG(INFO, "%s", __func__);

	fbi->monspecs = *(misc->monspecs);

	console_lock();

	misc->var_screeninfo->xres_virtual = fbi->var.xres_virtual;
	misc->var_screeninfo->yres_virtual = fbi->var.yres_virtual;
	misc->var_screeninfo->xoffset = fbi->var.xoffset;
	misc->var_screeninfo->yoffset = fbi->var.yoffset;
	misc->var_screeninfo->bits_per_pixel = fbi->var.bits_per_pixel;

	fbi->flags |= FBINFO_MISC_USEREVENT;

	misc->var_screeninfo->activate |= FB_ACTIVATE_ALL;
	ret = fb_set_var(fbi, misc->var_screeninfo);
	misc->var_screeninfo->activate &= ~FB_ACTIVATE_ALL;

	console_unlock();

	return ret;
}

static irqreturn_t xylonfb_adv7511_isr(int irq, void *dev_id)
{
	struct xylonfb_adv7511 *adv7511 = dev_id;

	XYLONFB_DBG(INFO, "%s", __func__);

	queue_work(adv7511->irq_work_queue, &adv7511->irq_work);

	return IRQ_HANDLED;
}

static void xylonfb_adv7511_handler(struct work_struct *work)
{
	struct xylonfb_adv7511 *adv7511 =
		container_of(work, struct xylonfb_adv7511, irq_work);

	XYLONFB_DBG(INFO, "%s", __func__);

	adv7511->sd->ops->core->interrupt_service_routine(adv7511->sd, 0, NULL);
}

static void xylonfb_adv7511_notify(struct v4l2_subdev *sd,
				   unsigned int notification, void *arg)
{
	union notify_data {
		struct adv7511_monitor_detect *md;
		struct adv7511_edid_detect *ed;
	} nd;
	int ret;

	XYLONFB_DBG(INFO, "%s", __func__);

	switch (notification) {
	case ADV7511_MONITOR_DETECT:
		nd.md = arg;
		XYLONFB_DBG(INFO, "ADV7511 monitor%sdetected",
			    nd.md->present ? " " : " not ");
		if (nd.md->present) {
			adv7511->timeout = HZ;
		} else {
			adv7511->timeout = 0;
			*(adv7511->xylonfb_flags) &= ~XYLONFB_FLAGS_EDID_READY;
			atomic_set(&adv7511->edid_lock, 0);
		}
		break;
	case ADV7511_EDID_DETECT:
		if (!(*(adv7511->xylonfb_flags) & XYLONFB_FLAGS_EDID_VMODE)) {
			*(adv7511->xylonfb_flags) |= XYLONFB_FLAGS_EDID_READY;
			wake_up_interruptible(adv7511->wait);
		}

		if (atomic_read(&adv7511->edid_lock))
			break;

		nd.ed = arg;
		XYLONFB_DBG(INFO, "ADV7511 EDID%sread",
			    nd.ed->present ? " " : " not ");
		if (!nd.ed->present)
			break;

		atomic_set(&adv7511->edid_lock, 1);
		pr_debug("EDID segment: %d\n", nd.ed->segment);

		memset(adv7511->edid, 0, XYLONFB_EDID_SIZE);

		adv7511->sd_edid.pad = 0;
		adv7511->sd_edid.start_block = 0;
		adv7511->sd_edid.blocks = 2;
		adv7511->sd_edid.edid = (__u8 __user *)adv7511->edid;
		ret = adv7511->sd->ops->pad->get_edid(sd, &adv7511->sd_edid);
		if (ret) {
			pr_err("failed get EDID\n");
			break;
		}

		fb_parse_edid(adv7511->edid, adv7511->var_screeninfo);
		xylonfb_adv7511_get_monspecs(adv7511->edid, adv7511->monspecs,
					     adv7511->var_screeninfo);
		xylonfb_adv7511_set_v4l2_timings(adv7511->sd,
						 adv7511->var_screeninfo);

		*(adv7511->xylonfb_flags) |= XYLONFB_FLAGS_EDID_READY;

		wake_up_interruptible(adv7511->wait);

		if (adv7511->init)
			complete(&adv7511->edid_done);
		else
			xylonfb_adv7511_update(adv7511->fbi);
		break;
	default:
		pr_warn("ADV7511 false notify (%d)\n", notification);
		break;
	}
}

int xylonfb_adv7511_register(struct fb_info *fbi)
{
	struct v4l2_subdev *sd;
	struct i2c_client *client;
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	struct xylonfb_misc_data *misc = &data->misc;
	int ret;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (adv7511)
		return -EEXIST;

	adv7511 = kzalloc(sizeof(struct xylonfb_adv7511), GFP_KERNEL);
	if (!adv7511) {
		pr_err("ADV7511 error allocating data\n");
		return -ENOMEM;
	}

	strlcpy(adv7511->dev.name, XYLONFB_DRIVER_NAME,
		sizeof(adv7511->dev.name));
	ret = v4l2_device_register(NULL, &adv7511->dev);
	if (ret) {
		pr_err("ADV7511 registering V4L2 device error\n");
		return ret;
	}

	adv7511->init = true;
	adv7511->dev.notify = xylonfb_adv7511_notify;

	init_completion(&adv7511->edid_done);

	adv7511->var_screeninfo = kzalloc(sizeof(struct fb_var_screeninfo),
					  GFP_KERNEL);
	adv7511->monspecs = kzalloc(sizeof(struct fb_monspecs),
				    GFP_KERNEL);
	adv7511->xylonfb_flags = &data->flags;
	adv7511->fbi = fbi;

	misc->var_screeninfo = adv7511->var_screeninfo;
	misc->monspecs = adv7511->monspecs;
	misc->edid = adv7511->edid;

	adv7511->wait = &misc->wait;

	sd = adv7511_sd(NULL);
	if (!sd) {
		pr_err("ADV7511 getting V4L2 subdevice error %s\n",
			ADV7511_NAME);
		ret = -ENODEV;
		goto error_subdev;
	}
	sd->v4l2_dev = &adv7511->dev;
	adv7511->sd = sd;

	client = v4l2_get_subdevdata(sd);
	if (!client) {
		pr_err("ADV7511 getting V4L2 subdevice client error\n");
		ret = -ENODEV;
		goto error_subdev;
	}

	adv7511->irq_work_queue = create_singlethread_workqueue(ADV7511_NAME);
	if (adv7511->irq_work_queue == NULL) {
		pr_err("ADV7511 workqueue error\n");
		goto error_subdev;
	}
	INIT_WORK(&adv7511->irq_work, xylonfb_adv7511_handler);

	if (client->irq > 0) {
		ret = request_irq(client->irq, xylonfb_adv7511_isr,
				  IRQF_TRIGGER_RISING, ADV7511_NAME, adv7511);
		if (ret) {
			pr_err("ADV7511 registering interrupt error %d at %d\n",
				ret, client->irq);
			goto error_irq;
		}
	} else {
		pr_err("ADV7511 error no IRQ registered\n");
	}

	sd->ops->core->interrupt_service_routine(sd, 0, NULL);

	if (*(adv7511->xylonfb_flags) & XYLONFB_FLAGS_EDID_VMODE) {
		if (adv7511->timeout) {
			ret = wait_for_completion_timeout(&adv7511->edid_done,
							  adv7511->timeout);
		}
		adv7511->init = false;
		if ((ret == 0) && (adv7511->timeout)) {
			pr_err("ADV7511 EDID error\n");
			return -ETIMEDOUT;
		}
	}

	return 0;

error_irq:
	flush_work(&adv7511->irq_work);
	flush_workqueue(adv7511->irq_work_queue);
	destroy_workqueue(adv7511->irq_work_queue);
error_subdev:
	v4l2_device_unregister(&adv7511->dev);

	kfree(adv7511->monspecs);
	kfree(adv7511->var_screeninfo);
	misc->edid = NULL;
	misc->monspecs = NULL;
	misc->var_screeninfo = NULL;

	kfree(adv7511);

	return ret;
}

void xylonfb_adv7511_unregister(struct fb_info *fbi)
{
	struct i2c_client *client = v4l2_get_subdevdata(adv7511->sd);
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	struct xylonfb_misc_data *misc = &data->misc;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (!adv7511)
		return;

	adv7511->sd->ops->video->s_stream(adv7511->sd, 0);
	adv7511->sd->ops->core->s_power(adv7511->sd, 0);

	free_irq(client->irq, adv7511);
	flush_work(&adv7511->irq_work);
	flush_workqueue(adv7511->irq_work_queue);
	destroy_workqueue(adv7511->irq_work_queue);

	kfree(adv7511->monspecs);
	kfree(adv7511->var_screeninfo);
	misc->edid = NULL;
	misc->monspecs = NULL;
	misc->var_screeninfo = NULL;

	v4l2_device_unregister(&adv7511->dev);

	kfree(adv7511);
	adv7511 = NULL;
}
