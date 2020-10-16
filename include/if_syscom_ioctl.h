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

#ifndef _UAPI_LINUX_IF_SYSCOM_ROUTE_H
#define _UAPI_LINUX_IF_SYSCOM_ROUTE_H

#ifdef __KERNEL__
#include <linux/if.h>
#else
#include <net/if.h>
#include <stdint.h>
#endif
#include <asm/ioctl.h>
#include <linux/if_syscom_service.h>

/** Type for removing routes */
#define SYSCOM_ROUTE_DELETE 0
/** Local route type for use in syscom_route.type */
#define SYSCOM_ROUTE_LOCAL  1
/** RIO route type for use in syscom_route.type */
#define SYSCOM_ROUTE_RIO    2
/** Socket route type for use in syscom_route.type */
#define SYSCOM_ROUTE_SOCKET 3

/** RIO route specific settings */
struct syscom_route_rio {
	/** RIO address of the destination */
	__be16 dst_addr;
	/** Messages bigger than this will be fragmented */
	uint16_t frag_threshold;
	/** Name of an interface used for sending */
	char src_if[IFNAMSIZ];
} __attribute__ ((__packed__, __aligned__(4)));

/** RIO route specific settings */
struct syscom_route_socket {
	/** Socket domain/family */
	int family;
	/** Socket type */
	int type;
	/** Socket protocol */
	int protocol;
	/** Source address length */
	int src_addrlen;
	/** Destination address length */
	int dst_addrlen;
	/** Source address */
	struct sockaddr src_addr;
	/** Destination address */
	struct sockaddr dst_addr;
} __attribute__ ((__packed__, __aligned__(4)));

/** Parameter for SYSCOM_ROUTE_ADD and SYSCOM_ROUTE_GET ioctls */
struct syscom_route {
	/** Destination NID */
	__be16 dst_nid;
	/** Number of valid bits in dst_nid */
	uint8_t dst_nid_len;
	/** Route type (e.g. #SYSCOM_ROUTE_LOCAL) */
	uint8_t type;
	union {
		/** RIO specific arguments */
		struct syscom_route_rio rio;
		/** Socket specific arguments */
		struct syscom_route_socket socket;
	};
} __attribute__ ((__packed__, __aligned__(4)));

struct syscom_socket_gw {
	int family;
	int protocol;
	int type;
	int addrlen;
	struct sockaddr addr;
} __attribute__ ((__packed__, __aligned__(4)));

/** Open the specified gateway socket */
struct syscom_gw_open_socket {
	/** Gateway name */
	char gw[SYSCOM_GW_NAME_MAX];
	/** Socket flags, e.g. O_CLOEXEC */
	int flags;
} __attribute__ ((__packed__, __aligned__(4)));

/** ioctl() request to add a new route */
#define SYSCOM_ROUTE_ADD _IOW('S', 1, struct syscom_route)
/** ioctl() request to get a configured route information */
#define SYSCOM_ROUTE_GET _IORW('S', 2, struct syscom_route)
/** ioctl() request to delete a route */
#define SYSCOM_ROUTE_DEL _IOW('S', 3, struct syscom_route)
/** ioctl() request to create a socket gateway */
#define SYSCOM_SOCKET_GW_ADD _IOW('S', 4, struct syscom_socket_gw)
/** ioctl() request to delete a socket gateway */
#define SYSCOM_SOCKET_GW_DEL _IOW('S', 5, struct syscom_socket_gw)
/** ioctl() to open a socket used by a gateway. Returns the file descriptor. */
#define SYSCOM_GW_OPEN_SOCKET _IOW('S', 6, struct syscom_gw_open_socket)

#endif // _UAPI_LINUX_IF_SYSCOM_ROUTE_H
