/*
 * SYSCOM protocol stack for the Linux kernel
 * Author: Petr Malat
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef SYSCOM_H
#define SYSCOM_H

#include <uapi/linux/if_syscom.h>

#define SKB_GSO_SYSCOM SKB_GSO_DODGY
#define NETIF_F_GSO_SYSCOM (SKB_GSO_SYSCOM << NETIF_F_GSO_SHIFT)

enum {
	/** Request peer to enable AXM workaround */
	SYSCOM_FRAG_HDR_FLAG_AXM_WA = 1,
};

/** Fragmentation header structure. Yes, the layout is really that sick. */
struct syscom_frag_hdr {
	/** Transaction ID */
        uint8_t tid;
	/** Reserved byte */
        uint8_t reserved;
	/** Source ID, e.g. NID/CPID */
        __be16 src;
	/** Total length of the message */
        __be16 total_length;
#ifdef __LITTLE_ENDIAN_BITFIELD
	/** Highest 4 bits of the fragment length */
        uint16_t frag_len1 :4;
	/** Fragment number */
        uint16_t frag :4;
	/** Lower 8 bits of the fragment length */
        uint16_t frag_len2 :8;
#endif
#ifdef __BIG_ENDIAN_BITFIELD
	// uint16_t is used, which gives GCC a hint to join
	// frag_len1 << 8 | frag_len2 access
	/** Fragment number */
        uint16_t frag :4;
	/** Highest 4 bits of the fragment length */
        uint16_t frag_len1 :4;
	/** Lower 8 bits of the fragment length */
        uint16_t frag_len2 :8;
#endif
} __attribute__((__packed__));

#endif // SYSCOM_H
