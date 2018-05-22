/*
 * Xylon logiCVC frame buffer driver miscellaneous interface functionality
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

#include "xylonfb_core.h"
#include "xylonfb_misc.h"

#if defined(CONFIG_FB_XYLON_MISC_ADV7511)

#include "xylonfb_adv7511.h"

static void xylonfb_misc_adv7511(struct fb_info *fbi, bool init)
{
	struct device *dev = fbi->dev;
	struct xylonfb_layer_data *ld = fbi->par;
	struct xylonfb_data *data = ld->data;
	struct xylonfb_misc_data *misc = &data->misc;
	int ret;

	XYLONFB_DBG(INFO, "%s", __func__);

	if (data->flags & XYLONFB_FLAGS_ADV7511_SKIP)
		return;

	if (init) {
		if (data->flags & XYLONFB_FLAGS_MISC_ADV7511)
			return;

		ret = xylonfb_adv7511_register(fbi);
		if (!ret) {
			fbi->monspecs = *(misc->monspecs);
			data->flags |= XYLONFB_FLAGS_MISC_ADV7511;
		} else {
			if (ret == -EEXIST)
				dev_warn(dev, "ADV7511 already initialized\n");
			else
				dev_err(dev, "ADV7511 initialization error\n");
		}
	} else {
		xylonfb_adv7511_unregister(fbi);
		data->flags &= ~XYLONFB_FLAGS_MISC_ADV7511;
	}
}
#endif

static void xylonfb_misc_init_wait(struct fb_info *fbi)
{
	struct xylonfb_layer_data *ld = fbi->par;

	XYLONFB_DBG(INFO, "%s", __func__);

	init_waitqueue_head(&ld->data->misc.wait);
}

void xylonfb_misc_init(struct fb_info *fbi)
{
	XYLONFB_DBG(INFO, "%s", __func__);

	xylonfb_misc_init_wait(fbi);
#if defined(CONFIG_FB_XYLON_MISC_ADV7511)
	xylonfb_misc_adv7511(fbi, true);
#endif
}

void xylonfb_misc_deinit(struct fb_info *fbi)
{
	XYLONFB_DBG(INFO, "%s", __func__);

#if defined(CONFIG_FB_XYLON_MISC_ADV7511)
	xylonfb_misc_adv7511(fbi, false);
#endif
}
