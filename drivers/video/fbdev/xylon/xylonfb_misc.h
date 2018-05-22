/*
 * Xylon logiCVC frame buffer driver miscellaneous interface functionality
 * header file
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

#ifndef __XYLONFB_MISC__
#define __XYLONFB_MISC__

#include <linux/fb.h>

struct xylonfb_misc_data {
	wait_queue_head_t wait;
	struct fb_var_screeninfo *var_screeninfo;
	struct fb_monspecs *monspecs;
	u8 *edid;
};

void xylonfb_misc_init(struct fb_info *fbi);
void xylonfb_misc_deinit(struct fb_info *fbi);

#endif /* #ifndef __XYLONFB_MISC__ */
