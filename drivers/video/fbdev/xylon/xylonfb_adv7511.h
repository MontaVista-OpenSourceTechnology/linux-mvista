/*
 * Xylon logiCVC frame buffer driver miscellaneous ADV7511 functionality
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

#ifndef __XYLONFB_MISC_ADV7511_H__
#define __XYLONFB_MISC_ADV7511_H__

#include <linux/fb.h>

int xylonfb_adv7511_register(struct fb_info *fbi);
void xylonfb_adv7511_unregister(struct fb_info *fbi);

#endif /* #ifndef __XYLONFB_MISC_ADV7511_H__ */
