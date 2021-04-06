/* VGEM driver. Implements MCU-DSP interface for data exchange
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DSP_MCU_PROTOCOL_H__
#define __DSP_MCU_PROTOCOL_H__

#include <asm/types.h>
#include <net/sock.h>
#include <linux/cdev.h>
#include <linux/wait.h>

#define MAGIC_WORD		0xE374

struct gem_header {
	u16 magic_word;
	u16 receiver;
	u16 sender;
	u16 size;
	u16 msg_id;
	u16 ref_number;
	u32 response_addr;
	u16 bs_instance;
	u16 reserved;
};


struct dsp_mcu_buffer {
	u32 dm_buf[0x40];
};

struct dsp_mcu_mi {
	u32	write_lock;
	u32	read_lock;
	u32	write_index;
	u32	read_index;
	u32	unused[0x3C];
	struct	dsp_mcu_buffer buffers[6];
};

#endif
