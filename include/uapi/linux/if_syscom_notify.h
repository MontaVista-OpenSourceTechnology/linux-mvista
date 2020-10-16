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

#ifndef _UAPI_LINUX_IF_SYSCOM_NOTIFY_H
#define _UAPI_LINUX_IF_SYSCOM_NOTIFY_H

#include <linux/if_syscom_service.h>
#include <linux/types.h>
#ifdef __KERNEL__
#include <linux/if.h>
#else
#include <net/if.h>
#include <stdint.h>
#endif

/** Syscom message IDs for notify messages */
enum syscom_notify_msg_id_e {
	/** ID of a message send after a route is added */
	SYSCOM_NOTIFY_MSG_ID_ROUTE_ADDED = 0x80,
	/** ID of a message send after a route is removed */
	SYSCOM_NOTIFY_MSG_ID_ROUTE_REMOVED,

	/** ID of a message, send after a gateway is added */
	SYSCOM_NOTIFY_MSG_ID_GW_ADDED,
	/** ID of a message send after a gateway is bound to a local address */
	SYSCOM_NOTIFY_MSG_ID_GW_BOUND,
	/** ID of a message send after a gateway is connected */
	SYSCOM_NOTIFY_MSG_ID_GW_CONNECTED,
	/** ID of a message send after a gateway is removed */
	SYSCOM_NOTIFY_MSG_ID_GW_REMOVED,
};

/** Notification header, which start all notification messages. The actual
 *  message can grow in the future by adding new elements to the end. */
struct syscom_notify_msg_hdr_s {
	/** Sequence number */
	__be32 seq;
	/** Reserved */
	__be32 reserved[3];
} __attribute__((__packed__));

/** Flags for "GW added" notification */
enum syscom_notify_msg_gw_add_flags_e {
	/** Gateway was automatically created after accepting a remote
	 * connection request. */
	SYSCOM_NOTIFY_MSG_GW_ADDED_FLAGS_ACCEPT = 1,
	/** Another gateway was renamed to this one */
	SYSCOM_NOTIFY_MSG_GW_ADDED_FLAGS_RENAME = 2,
};

/** Message send after a GW has been added */
struct syscom_notify_msg_gw_added_s {
	/** Notification header */
	struct syscom_notify_msg_hdr_s hdr;
	/** Added GW name */
	char name[SYSCOM_GW_NAME_MAX];
	/** Previous name of the GW if #SYSCOM_NOTIFY_MSG_GW_ADDED_FLAGS_RENAME
	 *  is set */
	char old_name[SYSCOM_GW_NAME_MAX];
	/** Combination of syscom_notify_msg_gw_add_flags_e */
	__be32 flags;
} __attribute__((__packed__));

/** Message send after a GW has been bound */
struct syscom_notify_msg_gw_bound_s {
	/** Notification header */
	struct syscom_notify_msg_hdr_s hdr;
	/** Bound GW name */
	char name[SYSCOM_GW_NAME_MAX];
	/** Number of addresses following the message. Note that sa_family is
	 *  in the network order. */
	__be16 addrcnt;
} __attribute__((__packed__));

/** Message send after a GW has been connected */
struct syscom_notify_msg_gw_connected_s {
	/** Notification header */
	struct syscom_notify_msg_hdr_s hdr;
	/** Connected GW name */
	char name[SYSCOM_GW_NAME_MAX];
	/** Number of addresses following the message. Note that sa_family is
	 *  in the network order. */
	__be16 addrcnt;
} __attribute__((__packed__));

/** Message send after a GW has been removed */
struct syscom_notify_msg_gw_removed_s {
	/** Notification header */
	struct syscom_notify_msg_hdr_s hdr;
	/** Removed GW name */
	char name[SYSCOM_GW_NAME_MAX];
	/** Reason for removal - can be a negative errno value or 0 for
	 * directly requested removal */
	__be32 reason;
} __attribute__((__packed__));

#endif // _UAPI_LINUX_IF_SYSCOM_NOTIFY_H
