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

#ifndef _UAPI_LINUX_IF_SYSCOM_SERVICE_H
#define _UAPI_LINUX_IF_SYSCOM_SERVICE_H

#include <linux/if_syscom.h>
#include <linux/types.h>
#ifdef __KERNEL__
#include <linux/if.h>
#else
#include <net/if.h>
#include <stdint.h>
#endif

/** Maximum length of base gateway name (base gateway name can create child gateways)*/
#define SYSCOM_GW_BASENAME_MAX 10
/** Maximum length of gateway name */
#define SYSCOM_GW_NAME_MAX 20
/** Maximum length of the header */
#define SYSCOM_ROUTE_DEV_HDR_MAX 18

/** Maximum timeout for service operations in ms (24 days).
 *
 * Timeouts equal or longer than this are treated as infinite timeout
 */
#define SYSCOM_SERVICE_MAX_TIMEOUT (24 * 86400 * 1000 + 1)

/** Syscom message IDs for service messages */
enum syscom_service_msg_id_e {
	/** Message ID of echo request */
	SYSCOM_SERVICE_MSG_ID_ECHO_REQUEST = 1,
	/** Message ID of echo reply */
	SYSCOM_SERVICE_MSG_ID_ECHO_REPLY = 2,
	/** Message ID of configuration request */
	SYSCOM_SERVICE_MSG_ID_CONF_REQUEST = 3,
	/** Message ID of configuration reply */
	SYSCOM_SERVICE_MSG_ID_CONF_REPLY = 4,
};

/** Ping request and reply message commonly send to the echo service.
 *
 * The echo service doesn't interpret the data and just sends back, what was
 * received. The structure is defined here to have a common definition for
 * different users of the ping service.
 */
struct syscom_service_msg_ping_s {
	/** Sequence number */
	__be64 seq;
	/** Time stamp */
	struct {
		/** Time stamp seconds */
		__be64 sec;
		/** Time stamp nanoseconds */
		__be32 nsec;
	} __attribute__((__packed__)) ts;
	/** Arbitrary payload */
	char data[];
} __attribute__((__packed__));

/** Configuration message request and reply.
 *
 * After this header there can be any number of syscom_service_msg_conf_op_*
 * structures describing requested configuration operations. These structures
 * can grow in the future by appending elements to the end, so one has always
 * initialize the whole structure to 0.
 */
struct syscom_service_msg_conf_s {
	/** Sequence number to make pairing of the reply with the request
	 * easier. */
	__be64 seq;
	/** Maximum time for the processing in milliseconds. */
	__be64 timeout;
	/** Reserved (must be zeroed) */
	__be32 reserved[4];
} __attribute__((__packed__));

/** Different configuration requests */
enum syscom_service_msg_conf_op_id_e {
	/** Delete a route */
	syscom_service_msg_conf_op_id_route_del = 1,
	/** Add a loopback route */
	syscom_service_msg_conf_op_id_route_add_local,
	/** Add a netdevice route */
	syscom_service_msg_conf_op_id_route_add_dev,
	/** Add a gateway route */
	syscom_service_msg_conf_op_id_route_add_gw,

	/** Put a destination down */
	syscom_service_msg_conf_op_id_route_dev_down = 0xa,
	/** Put a destination up */
	syscom_service_msg_conf_op_id_route_dev_up,

	/** Delete a gateway */
	syscom_service_msg_conf_op_id_gw_del = 0x10,
	/** Add a gateway */
	syscom_service_msg_conf_op_id_gw_add,
	/** Rename a gateway */
	syscom_service_msg_conf_op_id_gw_rename,
	/** Bind a gateway to an address or addresses */
	syscom_service_msg_conf_op_id_gw_bind,
	/** Start listening for incoming connections */
	syscom_service_msg_conf_op_id_gw_listen,
	/** Connect to a remote gateway and spawn a child gateway
	 * representing the newly created connection */
	syscom_service_msg_conf_op_id_gw_connect,
	/** Syscom header used for non-syscom connections */
	syscom_service_msg_conf_op_id_gw_hdr,
};

/** Configuration operations */
struct syscom_service_msg_conf_op_hdr_s {
	/** [in] Operation ID, one of ::syscom_service_msg_conf_op_id_e */
	__be16 op_id;
	/** [in] Request length including this header */
	__be16 len;
	/** [out] Request return value */
	__be32 rtn;
	/** Reserved (must be zeroed) */
	__be32 reserved[2];
} __attribute__((__packed__));

#define syscom_service_msg_conf_op(type, request, reply) \
	struct syscom_service_msg_conf_op_##type##_s { \
		struct syscom_service_msg_conf_op_hdr_s hdr; \
		union { \
			struct request __attribute__((__packed__)) req;\
			struct reply __attribute__((__packed__)) rep;\
		};\
	}

/** Delete syscom route */
syscom_service_msg_conf_op(route_del, {
		/** Destination NID */
		__be16 dst_nid;
		/** Number of valid bits in dst_nid */
		__u8 dst_nid_len;
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
	}, {
		char none[0];
	});

/** Add local route */
syscom_service_msg_conf_op(route_add_local, {
		/** Source NID used if not provided by the sender */
		__be16 src_nid;
		/** Destination NID */
		__be16 dst_nid;
		/** Number of valid bits in dst_nid */
		__u8 dst_nid_len;
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
	}, {
		char none[0];
	});

enum syscom_service_msg_conf_op_route_add_dev_flags_e {
	/** Fail with EMEDIUMTYPE when sending a message with ordered bit set */
	SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_DEV_FLAGS_REJECT_ORDERED = 1 << 3,
	/** Fail with EMEDIUMTYPE when sending a message without
	 * SYSCOM_HDR_FLAG_UNRELIABLE flag set */
	SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_DEV_FLAGS_REJECT_RELIABLE = 1 << 4,
};

/** Add netdev route */
syscom_service_msg_conf_op(route_add_dev, {
		/** Source NID used if not provided by the sender */
		__be16 src_nid;
		/** Destination NID */
		__be16 dst_nid;
		/** Fragmentation threshold */
		__be16 frag;
		/** Number of valid bits in dst_nid */
		__u8 dst_nid_len;
		/** Underlaying transport header length */
		__u8 header_len;
		/** Underlaying transport header */
		__u8 header[SYSCOM_ROUTE_DEV_HDR_MAX];
		/** Source interface name */
		char src_if[16];
		/** Combination of syscom_service_msg_conf_op_route_add_dev_flags_e */
		__be32 flags;
		/** Reserved (must be zeroed) */
		__be32 reserved[1];
	}, {
		char none[0];
	});

/** Flags for route gateway */
enum syscom_service_msg_conf_op_route_add_gw_flags_e {
	/** Create the route even if the gateway doesn't exist. User must
	 *  specify the behaviour by #SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_FAIL
	 *  or #SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_BLOCK */
	SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW = 1 << 0,
	/** Sending operation fails with ENOTCONN, if the gateway doesn't
	 *  exist. */
	SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_FAIL = 1 << 1,
	/** Sending operation blocks (or fails with EAGAIN if it's non-blocking),
	 *  if the gateway doesn't exist. */
	SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_NOGW_BLOCK = 1 << 2,
	/** Fail with EMEDIUMTYPE when sending a message with ordered bit set */
	SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_REJECT_ORDERED =
			SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_DEV_FLAGS_REJECT_ORDERED,
	/** Fail with EMEDIUMTYPE when sending a message without
	 * SYSCOM_HDR_FLAG_UNRELIABLE flag set */
	SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_GW_FLAGS_REJECT_RELIABLE =
			SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_ADD_DEV_FLAGS_REJECT_RELIABLE,
};

/** Add gateway route */
syscom_service_msg_conf_op(route_add_gw, {
		/** Source NID used if not provided by the sender */
		__be16 src_nid;
		/** Destination NID */
		__be16 dst_nid;
		/** Fragmentation threshold */
		__be16 frag;
		/** Number of valid bits in dst_nid */
		__u8 dst_nid_len;
		/** Combination of syscom_service_msg_conf_op_route_add_gw_flags_e */
		__be32 flags;
		/** Target gateway name */
		char gw[SYSCOM_GW_NAME_MAX];
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
	}, {
		char none[0];
	});

/** Flags for putting device routes down */
enum syscom_service_msg_conf_op_route_dev_down_flags_e {
	/** Automatically put down matching newly created routes as well until
	 *  this flag is cleared or matching route_dev_up is called */
	SYSCOM_SERVICE_MSG_CONF_OP_ROUTE_DEV_DOWN_AUTO = 1,
};

/** Put all matching device routes down */
syscom_service_msg_conf_op(route_dev_down, {
		/** Underlaying transport header length */
		__u8 header_len;
		/** Underlaying transport header */
		__u8 header[SYSCOM_ROUTE_DEV_HDR_MAX];
		/** Source interface name */
		char src_if[16];
		/** Combination of syscom_service_msg_conf_op_route_dev_down_flags_e */
		__be32 flags;
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
	}, {
		char none[0];
	});

/** Put all matching device routes up and clear matching ROUTE_DEV_DOWN_AUTO */
syscom_service_msg_conf_op(route_dev_up, {
		/** Underlaying transport header length */
		__u8 header_len;
		/** Underlaying transport header */
		__u8 header[SYSCOM_ROUTE_DEV_HDR_MAX];
		/** Source interface name */
		char src_if[16];
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
	}, {
		char none[0];
	});

/** Gateways
 *
 * Base gateway is a gateway, which has a local address only. It can receive
 * messages only if the underlying protocol is connectionless and
 *
 * Child gateway can be instantiated by connecting the base gateway to a peer or
 * by accepting a remote connection on a listening gateway. Flags, header etc. are
 * inherited from the base gateway.
 *
 * Base gateway name specified by the user matches
 *    [a-zA-Z][a-zA-Z0-9_]{0,#SYSCOM_GW_BASENAME_MAX - 2}
 * base name created by the kernel has the form
 *    _[a-zA-Z0-9_.]{8}
 *
 * User supplied child gateway name is created from the base gateway name by
 * appending colon ":" and child gateway identifier in the form
 *    [a-zA-Z][a-zA-Z0-9_]{0,#SYSCOM_GW_NAME_MAX - #SYSCOM_GW_BASENAME_MAX - 2}
 * The kernel provided child gateway name first character is always
 * the underscore "_", similarly to a base gateway name.
 */

/** Flags for gateway delete operation */
enum syscom_service_msg_conf_op_gw_del_flags_e {
	/** Unbind from routes pointing to this gateway */
	SYSCOM_SERVICE_MSG_CONF_OP_GW_DEL_ROUTES = 1 << 0,
	/** Automatically delete children routes */
	SYSCOM_SERVICE_MSG_CONF_OP_GW_DEL_CHILDREN = 1 << 1,
	/** Asynchronous GW delete */
	SYSCOM_SERVICE_MSG_CONF_OP_GW_DEL_NONBLOCK = 1 << 2,
};

/** Delete a gateway */
syscom_service_msg_conf_op(gw_del, {
		/** Deleted gateway name */
		char name[SYSCOM_GW_NAME_MAX];
		/** Combination of syscom_service_msg_conf_op_gw_del_flags_e */
		__be32 flags;
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
	}, {
		char none[0];
	});

/** Rename a gateway */
syscom_service_msg_conf_op(gw_rename, {
		/** Current name */
		char old_name[SYSCOM_GW_NAME_MAX];
		/** New name */
		char new_name[SYSCOM_GW_NAME_MAX];
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
	}, {
		char none[0];
	});

/** Flags for gateway add operation */
enum syscom_service_msg_conf_op_gw_add_flags_e {
	/** Strip syscom header on TX and add one on RX */
	SYSCOM_SERVICE_MSG_CONF_OP_GW_ADD_NOHDR = 1 << 0,
	/** Emulate a many to many style socket if connect is called
	 *  on a listening gateway. It works by allocating a socket
	 *  with the same address, but a different port. */
	SYSCOM_SERVICE_MSG_CONF_OP_GW_ADD_M2M_EMULATION = 1 << 1,
};

/** Add a socket gateway */
syscom_service_msg_conf_op(gw_add, {
		/** Requested name or empty for automatically generated name */
		char name[SYSCOM_GW_BASENAME_MAX];
		/** Net namespace ID, where to create the socket (-1 for init_net) */
		__be32 net;
		/** Socket domain (e.g. PF_INET) */
		__be32 domain;
		/** Socket type (e.g. SOCK_DGRAM) */
		__be32 type;
		/** Socket protocol (e.g. IPPROTO_UDP) */
		__be32 protocol;
		/** Combination of syscom_service_msg_conf_op_gw_add_flags_e */
		__be32 flags;
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
		/** Number of addresses for immediate bind. These addresses are
		 *  of the message. Member sa_family must be in the network
		 *  order. */
		__be16 addr_cnt;
	}, {
		/** Name of the new gateway */
		char name[SYSCOM_GW_BASENAME_MAX];
	});

/** Bind a socket gateway */
syscom_service_msg_conf_op(gw_bind, {
		/** Gateway name */
		char name[SYSCOM_GW_BASENAME_MAX];
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
		/** Number of addresses. These addresses are appended to the end
		 *  of the message. Member sa_family must be in the network
		 *  order. */
		__be16 addr_cnt;
	}, {
		char none[0];
	});

/** Flags for gateway connect */
enum syscom_service_msg_conf_op_gw_connect_flags_e {
	/** Initiate the connection, but do not wait for completion */
	SYSCOM_SERVICE_MSG_CONF_OP_GW_CONNECT_NONBLOCK = 1,
};

/** Connect a socket gateway */
syscom_service_msg_conf_op(gw_connect, {
		/** Gateway name */
		char name[SYSCOM_GW_BASENAME_MAX];
		/** Desired gateway name or empty string for automatically
		 * generated name */
		char childname[SYSCOM_GW_NAME_MAX];
		/** Combination of syscom_service_msg_conf_op_gw_connect_flags_e */
		__be32 flags;
		/** Connection timeout in milliseconds. */
		__be32 timeout;
		/** Number of addresses. These addresses are appended to the end
		 *  of the message. Member sa_family must be in the network
		 *  order. */
		__be16 addr_cnt;
	}, {
		/** New gateway name */
		char childname[SYSCOM_GW_NAME_MAX];
	});

/** Listen on a socket gateway */
syscom_service_msg_conf_op(gw_listen, {
		/** Gateway name */
		char name[SYSCOM_GW_NAME_MAX];
		/** Maximum number of connection waiting for being accepted */
		__be32 backlog;
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
	}, {
		char none[0];
	});

/** Specify a syscom header prepended to received messages */
syscom_service_msg_conf_op(gw_hdr, {
		/** Gateway name */
		char name[SYSCOM_GW_NAME_MAX];
		/** Syscom header prepended to incoming messages */
		struct syscom_hdr syscom_hdr;
		/** Reserved (must be zeroed) */
		__be32 reserved[2];
	}, {
		char none[0];
	});

#endif // _UAPI_LINUX_IF_SYSCOM_SERVICE_H
