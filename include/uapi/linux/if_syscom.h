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

#ifndef _UAPI_LINUX_IF_SYSCOM_H
#define _UAPI_LINUX_IF_SYSCOM_H

#include <linux/types.h>
#include <asm/byteorder.h>
#ifdef __KERNEL__
#include <linux/if.h>
#else
#include <net/if.h>
#include <stdint.h>
#endif

/** Socket option to configure back pressure */
#define	SYSCOM_SO_BACKPRESSURE 1
/** Socket option to configure time stamping */
#define	SYSCOM_SO_QUEUETSTAMP  2
/** Socket option to read statistics */
#define	SYSCOM_SO_STATS        3
/** Receive full syscom header */
#define	SYSCOM_SO_RECV_FULLHDR 4
/** Send with full syscom header */
#define	SYSCOM_SO_SEND_FULLHDR 5
/** CMSG type for specifying NID for use with sockets bound to ORPHAN or ANY */
#define SYSCOM_SO_ADDR_NID     6
/** CMSG type for specifying CPID for use with sockets bound to ORPHAN or ANY */
#define SYSCOM_SO_ADDR_CPID    7
/** Controls what delivery notifications should be queued to the error queue */
#define SYSCOM_SO_TX_REPORT    8
/** Weakly bound socket unbinds itself if normal socket binds the same address */
#define SYSCOM_SO_WEAK_BIND    9


/** Flags for SYSCOM_SO_TX_REPORT */
enum {
	/** Report when delivery of reliable message fails */
	SYSCOM_SO_TX_REPORT_RELIABLE_FAIL = 1,
	/** Report when delivery of reliable message succeeds */
	SYSCOM_SO_TX_REPORT_RELIABLE_DONE = 2,
	/** Report when delivery of unreliable message fails */
	SYSCOM_SO_TX_REPORT_UNRELIABLE_FAIL = 4,
	/** Report when delivery of unreliable message succeeds */
	SYSCOM_SO_TX_REPORT_UNRELIABLE_DONE = 8,
};

/** Socket statistics - optval for SYSCOM_SO_STATS */
struct syscom_stats {
	/** Packets dropped while receiving (e.g socket queue was full) */
	uint32_t rx_dropped;
	/** Packets dropped during sending */
	uint32_t tx_dropped;
	/** Packets received */
	uint64_t rx_count;
	/** Packets send */
	uint64_t tx_count;
};

/** Syscom header flags */
enum syscom_hdr_flags {
	SYSCOM_HDR_FLAG_BE32 = 0x0,
	/** Payload contains 32-bit word data in the little endian format */
	SYSCOM_HDR_FLAG_LE32 = 0x4,
	/** Payload contains 8-bit byte data (exceptional longer fields are in
	 * the big endian format) */
	SYSCOM_HDR_FLAG_BE8  = 0x8,
	/** Payload contains 8-bit byte data (exceptional longer fields are in
	 * the little endian format) */
	SYSCOM_HDR_FLAG_LE8  = 0xc,
	/** Reliable transport is not required */
	SYSCOM_HDR_FLAG_UNRELIABLE = 0x10,
};

/** SYSCOM header for use with raw sockets */
struct syscom_hdr {
#if defined(__BIG_ENDIAN_BITFIELD)
	/** Protocol version */
	uint8_t version:4;
	/** Message priority or dscp_val_3 */
	uint8_t priority:2;
	/** Middle 2 bits of DSCP value */
	uint8_t dscp_val_2:2;
	/** Lowest 2 bits of DSCP value */
	uint8_t dscp_val_1:2;
	/** Indicates if DSCP value is present */
	uint8_t dscp:1;
	uint8_t :1;
	/** Deliver this message in order in respect to other messages with
	 * the same addresses. */
	uint8_t ordered:1;
	/** Time to live */
	uint8_t ttl:3;
#elif defined(__LITTLE_ENDIAN_BITFIELD)
	/** Middle 2 bits of DSCP value */
	uint8_t dscp_val_2:2;
	/** Message priority or dscp_val_3 */
	uint8_t priority:2;
	/** Protocol version */
	uint8_t version:4;
	/** Time to live */
	uint8_t ttl:3;
	/** Deliver this message in order in respect to other messages with
	 * the same addresses. */
	uint8_t ordered:1;
	uint8_t :1;
	/** Indicates if DSCP value is present */
	uint8_t dscp:1;
	/** Lowest 2 bits of DSCP value */
	uint8_t dscp_val_1:2;
#else
#error "Broken asm/byteorder.h"
#endif
	/** The message type */
	__be16 msg_id;
	union {
		/** Destination address */
		uint32_t destination;
		/** Broken down destination address */
		struct {
			/** Destination NID */
			__be16 nid;
			/** Destination CPID */
			__be16 cpid;
		} dst;
	};
	union {
		/** Source address */
		uint32_t source;
		/** Broken down source address */
		struct {
			/** Source NID */
			__be16 nid;
			/** Source CPID */
			__be16 cpid;
		} src;
	};
	/** Length of the message including this header */
	__be16 length;
	/** Message flags (see #syscom_hdr_flags) */
	__be16 flags;
} __attribute__ ((packed));

/** Short header used by DGRAM sockets */
struct syscom_dgram_hdr {
	/** The message type */
	__be16 msg_id;
	/** Message flags */
	__be16 flags;
} __attribute__ ((packed));

/** SYSCOM address */
struct sockaddr_syscom {
	/** Address family, shall be AF_SYSCOM */
	sa_family_t sun_family;
	/** NID */
	__be16 nid;
	/** CPID */
	__be16 cpid;
};

/*** Address family *************************************************/

/** SYSCOM address family */
#define AF_SYSCOM  19
/** SYSCOM protocol family */
#define PF_SYSCOM  AF_SYSCOM
/** SYSCOM sockets API level, for use with getsockopt() and setsockopt() */
#define SOL_SYSCOM AF_SYSCOM

/** Automatically assign available CPID */
#define	SOCKADDR_SYSCOM_CPID_AUTO 0x0000
/** CPID used by the syscom service socket */
#define SOCKADDR_SYSCOM_CPID_SERVICE 0xFF00
/** CPID where syscom notifications are send to */
#define SOCKADDR_SYSCOM_CPID_NOTIFY 0xFF01
/** Wild-card value representing all unassigned NIDs or CPIDs */
#define SOCKADDR_SYSCOM_ORPHANS 0xFFFE
/** Wild-card value representing all NIDs or CPIDs */
#define SOCKADDR_SYSCOM_ANY 0xFFFF

#endif // _UAPI_LINUX_IF_SYSCOM_H
