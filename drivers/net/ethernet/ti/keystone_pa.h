/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Author: Sandeep Paulraj <s-paulraj@ti.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef KEYSTONE_PA_H
#define KEYSTONE_PA_H

#ifdef __KERNEL__


struct pa_pdsp_config {

	u32 pdsp[6];

	char *pdsp_fw[6];
};

#define PAFRM_MAX_CMD_SET_SIZE		124

#define	PA_DEST_DISCARD  3  /**< Packet is discarded */

/** 
 *  @def  PA_DEST_CONTINUE_PARSE_LUT1
 *        packet remains in PA sub-system for more parsing and LUT1 classification
 */
#define PA_DEST_CONTINUE_PARSE_LUT1  4 /**< Packet remains in PA sub-system for more parsing and LUT1 classification */

/** 
 *  @def  PA_DEST_CONTINUE_PARSE_LUT2
 *        packet remains in PA sub-system for more parsing and LUT2 classification. 
 */
#define PA_DEST_CONTINUE_PARSE_LUT2  5  /**< Packet remains in PA sub-system for more parsing and LUT2 classification */

/**
 *  @def  PA_DEST_HOST
 *        host thread 
 */
#define PA_DEST_HOST   6   /**< Packet is routed to host */

/** 
 *  @def  PA_DEST_EMAC
 *        ethernet mac port (of the switch)
 */
#define PA_DEST_EMAC   7   /**< Packet is routed to  EMAC */

/** 
 *  @def  PA_DEST_SASS
 *        security accelerator destination 
 */
#define PA_DEST_SASS   8   /**< Packet is routed to SA */

#define PA_DEST_SRIO   9

#define PA_NO_MULTI_ROUTE		-1
#define PA_MAX_MULTI_ROUTE_SETS		32
#define PA_MAX_MULTI_ROUTE_ENTRIES	8
#define PA_MULTI_ROUTE_DESCRIPTOR_ONLY	0x01

#define PA_EMAC_CTRL_PORT_MASK		0x0F 
#define PA_EMAC_CTRL_CRC_DISABLE	0x80 
#define PA_CUSTOM_TYPE_NONE		0   
#define PA_CUSTOM_TYPE_LUT1		1   
#define PA_CUSTOM_TYPE_LUT2		2   
#define PA_MAX_CUSTOM_TYPES_LUT1	4
#define PA_MAX_CUSTOM_TYPES_LUT2	4

#define PA_CMD_TX_DEST_0	0  /* Packet is sent to PDSP0 */
#define PA_CMD_TX_DEST_1	1  /* Packet is sent to PDSP1 */
#define PA_CMD_TX_DEST_2	2  /* Packet is sent to PDSP2 */
#define PA_CMD_TX_DEST_3	3  /* Packet is sent to PDSP3 */
#define PA_CMD_TX_DEST_4	4  /* Packet is sent to PDSP4 */
#define PA_CMD_TX_DEST_5	5  /* Packet is sent to PDSP5 */

#define PA_CMD_NONE			0   
#define PA_CMD_NEXT_ROUTE		1   
#define PA_CMD_CRC_OP			2   
#define PA_CMD_COPY_DATA_TO_PSINFO	3
#define PA_CMD_PATCH_DATA		4  
#define PA_CMD_TX_CHECKSUM		5 
#define PA_CMD_MULTI_ROUTE		6  
#define PA_CMD_REPORT_TX_TIMESTAMP	7 
#define PA_CMD_REMOVE_HEADER		8 
#define PA_CMD_REMOVE_TAIL		9 
#define PA_CMD_CMDSET			10   
#define PA_CMD_SA_PAYLOAD		11
#define PA_CMD_IP_FRAGMENT		12
#define PA_CMD_USR_STATS		13
#define PA_CMD_CMDSET_AND_USR_STATS	14

/* Interface based routing modes */
#define PA_ROUTE_INTF_NONE	0 /* No interface based routing */
#define PA_ROUTE_INTF_QUEUE	1 /* Route by interface number as dest queue
				     offset */
#define PA_ROUTE_INTF_FLOW	2 /* Route by interface number as both dest
				     queue & CPPI flow offset */

struct pa_frm_forward_host {

	u32	context; /* Context returned as swInfo0 for matched packet */
	/*  Control bitmap, 1 for enable, 0 for disable
	 *  /-----------------------------------------------------\
	 *  | 7           |       2     |      1      |     0     |
	 *  | Selection   |             |Flow IF Dest |           |
	 *  | 0: Priority |             |    OR       |           |
	 *  | 1: IF dest  |DSCP priority|VLAN priority| multiRoute|
	 *  \-----------------------------------------------------/
	 */
	u8	ctrl_bm;
	u8	multi_idx;	/* Index of the multiple destination set */
	u8	pa_pdsp_router; /* PA PDSP number used as multi-route router */
	u8	ps_flags;	/* use the bits 7:4.
				   bit 7: Disable CRC,
				   bit 6:4 port number (0/1/2),
				   bit 3:0 errflags = 0
				   psFlags may be required when the packet is
				   forwarded through QoS queue */
	u8	cmd[4];		/* optional simple command:0 means no command */
}; /* 12 bytes */
#define PAFRM_MULTIROUTE_ENABLE			0x1
#define PAFRM_ROUTING_PRIORITY_DSCP_ENABLE	0x2
#define PAFRM_ROUTING_PRIORITY_VLAN_ENABLE	0x4
#define PAFRM_ROUTING_FLOW_IF_BASE_ENABLE	0x2 /* 0: queue-based only
						       1: queue & flow-based */
#define PAFRM_ROUTING_IF_DEST_SELECT_ENABLE	0x80


/* Routing information used to forward packets to the SA (via PKTDMA) */
struct pa_frm_forward_sa {

	u32	sw_info_0;	/* Packet descriptor swInfo0 required by SA
				   operation */
	u32	sw_info_1;	/* Packet descriptor swInfo1 required by SA
				   operation */
	u8	cmd[4];		/* optional simple command:0 means no command */
};

/* Routing information used to forward packets to the SRIO (via PKTDMA) */
struct pa_frm_forward_srio {

	u32  ps_info0;		/* 8-byte protocol-specific information
				   required by SRIO  */
	u32  ps_info1;		/* routing */
	u8   pkt_type;		/* Packet type specified for SRIO operation */
	u8   rsv4[3];
};

/* Routing information used to forward packets to the Ethernet port */
struct pa_frm_forward_eth {
	u8	ps_flags;  /* use the bit 7:4
			      bit 7: Disable CRC,
			      bit 6:4 port number (0/1/2)
			      bit 3:0 errflags = 0*/
	u8	rsvd1;
	u16	rsvd2;
	u32	rsvd3;
	u32	rsvd4;
};

#define PAFRM_ETH_PS_FLAGS_DISABLE_CRC          0x80
#define PAFRM_ETH_PS_FLAGS_PORT_MASK            0x70
#define PAFRM_ETH_PS_FLAGS_PORT_SHIFT              4


/* Routing information used to forward packets within PA */
struct pa_frm_forward_pa {

	u8	pa_dest;      /* PDSP destination */
	u8	custom_type;  /* None, LUT1, LUT2 */
	u8	custom_idx;   /* Index of the custom type if LUT1 or LUT2
				 custom */
	u8	rsvd2;
	u32	rsvd3;
	u32	rsvd4;
};

#define PAFRM_CUSTOM_TYPE_NONE PA_CUSTOM_TYPE_NONE    /* 0 */
#define PAFRM_CUSTOM_TYPE_LUT1 PA_CUSTOM_TYPE_LUT1    /* 1 */
#define PAFRM_CUSTOM_TYPE_LUT2 PA_CUSTOM_TYPE_LUT2    /* 2 */

/* Routing information used to forward packets fromm PA sub-system to various
 * destinations
 */
struct pa_frm_forward  {

	u8 forward_type;	/* Forwarding type as defined below */
	u8 flow_id;		/* PKTDMA flow Id, valid if forwarding via
				   PKTDMA */
	u16 queue;		/* Destination queue number, valid if
				   forwarding via PKTDMA */
  
	union {
		struct pa_frm_forward_host	host; /* Host specific routing
							 information */
		struct pa_frm_forward_sa	sa;   /* SA specific routing
							 information */
		struct pa_frm_forward_srio	srio; /* SRIO specific routing
							 information */
		struct pa_frm_forward_eth	eth;  /* Ethernet specific
							 routing information */
		struct pa_frm_forward_pa	pa;   /* PA internal routing
							 information */
	} u;
};

enum {
	PAFRM_FORWARD_TYPE_HOST = 0,	/* use PAFRM_DEST_CDMA */
	PAFRM_FORWARD_TYPE_SA,		/* use PAFRM_DEST_CDMA */
	PAFRM_FORWARD_TYPE_PA,		/* use pa.paDest */
	PAFRM_FORWARD_TYPE_ETH,		/* use PAFRM_DEST_ETH */
	PAFRM_FORWARD_TYPE_SRIO,	/* use PAFRM_DEST_CDMA */
	PAFRM_FORWARD_TYPE_DISCARD
};

/* Custom match flag bits */
#define	PAFRM_LUT1_CUSTOM_MATCH_ETYPE	(1 << 2)
#define	PAFRM_LUT1_CUSTOM_MATCH_VLAN	(1 << 3)
#define	PAFRM_LUT1_CUSTOM_MATCH_MATCH	(3 << 4)  /* Ipv6 source and dest entries */
#define	PAFRM_LUT1_CUSTOM_MATCH_KEY	(1 << 13)
#define	PAFRM_LUT1_CUSTOM_MATCH_VALID	(1 << 15)

/* Key values. The PDSP will set these bits as it parses the SRIO header */
#define PAFRM_LUT1_CUSTOM_KEY_CUSTOM		PAFRM_LUT1_KEY_CUSTOM
#define PAFRM_LUT1_CUSTOM_KEY_INDEX(index)	((index) << 0)  /* Vaild if custom type is set */

/* Add entry to LUT1 */
/* if PA_LUT1_INDEX_LAST_FREE is used then when the command returns, the value of index
 * will be replaced with the actual index used */
#define PAFRM_HW_LUT1_ENTRIES		64
#define PAFRM_LUT1_INDEX_LAST_FREE	PAFRM_HW_LUT1_ENTRIES

/* Standard match flag bits */
#define PAFRM_LUT1_MATCH_DMAC		(1 << 0)
#define PAFRM_LUT1_MATCH_SMAC		(1 << 1)
#define PAFRM_LUT1_MATCH_ETYPE		(1 << 2)
#define PAFRM_LUT1_MATCH_VLAN		(1 << 3)
#define PAFRM_LUT1_MATCH_SIP		(1 << 4)
#define PAFRM_LUT1_MATCH_DIP		(1 << 5)
#define PAFRM_LUT1_MATCH_SPI_GRE_SCTP	(1 << 6)
#define PAFRM_LUT1_MATCH_FLOW		(1 << 7)
#define PAFRM_LUT1_MATCH_SPORT		(1 << 8)
#define PAFRM_LUT1_MATCH_DPORT		(1 << 9)
#define PAFRM_LUT1_MATCH_PROTO		(1 << 10)
#define PAFRM_LUT1_MATCH_TOS		(1 << 11)
#define PAFRM_LUT1_MATCH_PORT		(1 << 12)
#define PAFRM_LUT1_MATCH_KEY		(1 << 13)
#define PAFRM_LUT1_MATCH_VALID		(1 << 15)

#define PAFRM_LUT1_MATCH_MPLS		(PAFRM_LUT1_MATCH_SPORT | PAFRM_LUT1_MATCH_DPORT)

/* Key values. The PDSP will set these bits as it parses the headers. */
/* LUT1_1 and LUT1_2 (L3): The following bit fields are used */
#define PAFRM_LUT1_KEY_SPI	(1 << 0)
#define PAFRM_LUT1_KEY_GRE	(1 << 1)
#define PAFRM_LUT1_KEY_MPLS	(1 << 2)
#define PAFRM_LUT1_KEY_IPV4	(1 << 3)
#define PAFRM_LUT1_KEY_IPV6	(1 << 4)
#define PAFRM_LUT1_KEY_SCTP	(1 << 5)

/* LUT1: Custom  (L3) */
#define PAFRM_LUT1_KEY_CUSTOM	(1 << 7)     

/* LUT1_0: MAC and SRIO (L0-l2): The following bit fields are used */
#define PAFRM_LUT1_KEY_SRIO	(1 << 7)

#define PAFRM_LUT1_KEY_MAC    (1 << 0)

struct pa_frm_com_l1_standard {

	/* LUT1 view 1 */
	u8	dmac[6];	/* Destination mac */
	u8	smac[6];	/* Source mac */
	u16	etype;		/* Ethernrt type, the field is also used for the previous match PDSP number */
	u16	vlan;		/* VLAN tag, the field is also used for the previous match LUT1 index */
  
	/* LUT1 view 2 */
	u8	src_ip[16];	/* Source IP address */
	u8	dst_ip[16];	/* Destination IP address */
  
	/* LUT1 view 3 */
	u32	spi;		/* ESP or AH header Security Parameters Index */
				/* The field is also used for GRE protocol or SCTP destination port */
	u32	flow;		/* IPv6 flow label in 20 lsbs */
  
	union {
		u16	ports[2];   /* UDP/TCP Source port (0), destination port (1) */
		u32	mpls;       /* mpls label in 20 Lsbs */
	} pm;
  
	u8	proto_next;	/* Ipv4 Protocol fields, IPv6 next */
	u8	tos_tclass;	/* Ipv4 TOS, Ipv6 traffic class */
	u8	inport;		/* reserved field: not used */
	u8	key;		/* IP: Distinguishs spi/gre and mpls and ports
					* LUT1_0: MAC/SRIO, 
					* LUT1_1/LUT1_2: custom or standard 
					*/
	/* end LUT1 view 3 */
  
	/* Lookup cares/don't cares */
	u16	match_flags;	/* lookup matching valid flags as defined below */
	u16	rsvd;		/* reserved for alignment */
};

struct pa_frm_com_l1_srio {

	/* LUT1 view 1 */
	u8	rsvd1[4];	/* unused field: All zero's */
	u16	src_id;	/* Source ID */
	u16	dest_id;	/* Destination ID */
	u8	rsvd2[4];	/* unused field: All zero's */
	u16	etype;	/* upper link (previous match PDSP number) */
	u16	vlan;	/* upper link (previous match LUT1 index) */
  
	/* LUT1 view 2 */
	u8	rsvd3[16];		/* unused field: All zero's */
	u8	rsvd4[14];		/* unused field: All zero's */
	u16	type_param1;	/* stream ID or mailbox */
  
	/* LUT1 view 3 */
	u32	spi;	/* unused field: All zero's */
	u32	flow;	/* unused field: All zero's */
  
	u16	next_hdr_offset;	/* unused field: All zero's */
	u8	next_hdr;		/* place holder for nextHdr and nextOffset */
	u8	rsvd5;			/* unused field: All zero's */
	u8	pri;			/* 3-bit Priority */
	u8	type_param2;		/* cos or letter */
	u8	inport;			/* unused field: All zero's */
	u8	key;			/* IP: Distinguishs spi/gre and mpls and ports
					 * LUT1_0: MAC/SRIO, 
					 * LUT1_1/LUT1_2: custom or standard 
					 */
	/* end LUT1 view 3 */
	/* Lookup cares/don't cares */
	u16	match_flags;		/* lookup matching valid flags as defined below */
	u16	rsvd;			/* reserved for alignment */
};

struct pa_frm_com_l1_custom{

	/* LUT1 view 1 */
	u8	dmac[6];	/* unused field: All zero's */
	u8	smac[6];	/* unused field: All zero's */
	u16	etype;		/* upper link (previous match PDSP number) */
	u16	vlan;		/* upper link (previous match LUT1 index) */
  
	/* LUT1 view 2 */
	u8	match_values[32];	/* 32 bytes to match   */
  
	/* LUT1 view 3 - offset from start */
	u32	rsvd0;		/* unused field: All zero's */
	u32	rsvd1;		/* unused field: All zero's */
	u32	rsvd2;		/* unused field: All zero's */
  
	u8	rsvd3;		/* unused field: All zero's */
	u8	rsvd4;		/* unused field: All zero's */
	u8	inport;		/* unused field: All zero's */
	u8	key;		/* IP: Distinguishs spi/gre and mpls and ports
				 * LUT1_0: MAC/SRIO, 
				 * LUT1_1/LUT1_2: custom or standard 
				 */
  
	/* Lookup cares/dont cares */
	u16	match_flags;	/* lookup matching valid flags as defined below */
	u16	rsvd5;		/* reserved for alignment */ 
};

enum {
	PAFRM_CONFIG_COMMAND_RSVD	= 0,
	PAFRM_CONFIG_COMMAND_ADDREP_LUT1,
	PAFRM_CONFIG_COMMAND_DEL_LUT1,
	PAFRM_CONFIG_COMMAND_ADDREP_LUT2,
	PAFRM_CONFIG_COMMAND_DEL_LUT2,
	PAFRM_CONFIG_COMMAND_CONFIG_PA,
	PAFRM_CONFIG_COMMAND_REQ_STATS,
	PAFRM_CONFIG_COMMAND_REQ_VERSION,
	PAFRM_CONFIG_COMMAND_MULTI_ROUTE,
	PAFRM_CONFIG_COMMAND_CRC_ENGINE,
	PAFRM_CONFIG_COMMAND_CMD_SET,
	PAFRM_CONFIG_COMMAND_USR_STATS,
	PAFRM_CONFIG_COMMAND_SYS_CONFIG
};

/* Command magic value */
#define PAFRM_CONFIG_COMMAND_SEC_BYTE  0xce

/* Command return values */
enum {

	PAFRM_COMMAND_RESULT_SUCCESS = 0,              /* Must be 0 */
	PAFRM_COMMAND_RESULT_NO_COMMAND_MAGIC,         /* Command magic value not found */
  
	PAFRM_COMMAND_RESULT_INVALID_CMD,              /* Invalid command identifier */
  
	/* Add entry to LUT1 fails */
	PAFRM_COMMAND_RESULT_LUT1_TYPE_INVALID,        /* Invalid type, custom or standard IP/ethernet */
	PAFRM_COMMAND_RESULT_LUT1_INDEX_INVALID,       /* Invalid LUT1 index (0-63) or no free indices available */
	PAFRM_COMMAND_RESULT_LUT1_MATCH_DEST_INVALID,  /* Sent a match packet to q0 on c1 or c2 - this is illegal. */
	PAFRM_COMMAND_RESULT_LUT1_NMATCH_INVALID,      /* Previous match forward info was somewhere in chunk domain */
	PAFRM_COMMAND_RESULT_LUT1_INVALID_KEYS,        /* Invalid combination found in the key value */
  
	/* Lut 2 entry warnings since the lut can be configured without pdsp */
	PAFRM_COMMAND_RESULT_WARN_OVER_MAX_ENTRIES,
	PAFRM_COMMAND_RESULT_WARN_NEGATIVE_ENTRY_COUNT,
  
	/* Lut 2 entry failures */
	PAFRM_COMMAND_RESULT_LUT2_ADD_BUSY,            /* LUT2 had a lookup and pending config */
  
	/* Not enough room in stats request packet for the reply */
	PAFRM_COMMAND_RESULT_WARN_STATS_REPLY_SIZE,
  
	/* Command sent to PDSP which couldn't handle it */
	PAFRM_COMMAND_RESULT_INVALID_DESTINATION,
  
	/* Add/Delete/Read entries to multi route table */
	PAFRM_COMMAND_RESULT_MULTI_ROUTE_NO_FREE_ENTRIES,    /* Asked to use a free entry, but none found */
	PAFRM_COMMAND_RESULT_MULTI_ROUTE_INVALID_IDX,        /* Illegal index value used */
	PAFRM_COMMAND_RESULT_MULTI_ROUTE_INVALID_MODE,       /* Illegal multi route mode used */
  
	/* Packet size didn't match command */
	PAFRM_COMMAND_RESULT_INVALID_PKT_SIZE,
  
	/* Coustom and Command set index */
	PAFRM_COMMAND_RESULT_INVALID_C1_CUSTOM_IDX,          /* Illegal Custom LUT1 index value used */
	PAFRM_COMMAND_RESULT_INVALID_C2_CUSTOM_IDX,          /* Illegal Custom LUT2 index value used */
	PAFRM_COMMAND_RESULT_INVALID_CMDSET_IDX              /* Illegal Custom Command Set index value used */
};

#define PA_SS_TIMER_CNTRL_REG_GO		0x00000001u
#define PA_SS_TIMER_CNTRL_REG_MODE		0x00000002u
#define PA_SS_TIMER_CNTRL_REG_PSE		0x00008000u
#define PA_SS_TIMER_CNTRL_REG_PRESCALE_SHIFT	0x00000002u

/* Destination (route) values */
#define PAFRM_DEST_PDSP0	0
#define PAFRM_DEST_PDSP1	1
#define PAFRM_DEST_PDSP2	2
#define PAFRM_DEST_PDSP3	3
#define PAFRM_DEST_PDSP4	4
#define PAFRM_DEST_PDSP5	5
#define PAFRM_DEST_PKTDMA	6   
#define PAFRM_DEST_ETH		7

#define PAFRM_DEST_DISCARD	10

/* Assigning names based on PDSP functions */
#define PAFRM_DEST_PA_C1_0	PAFRM_DEST_PDSP0
#define PAFRM_DEST_PA_C1_1	PAFRM_DEST_PDSP1
#define PAFRM_DEST_PA_C1_2	PAFRM_DEST_PDSP2 
#define PAFRM_DEST_PA_C2	PAFRM_DEST_PDSP3
#define PAFRM_DEST_PA_M_0	PAFRM_DEST_PDSP4
#define PAFRM_DEST_PA_M_1	PAFRM_DEST_PDSP5

/* The default queue for packets that arrive at the PA and don't match in
 * classify1 (right at init time) */
#define PAFRM_DEFAULT_INIT_Q	0x100

/* Ethertypes recognized by the firmware. */
#define PAFRM_ETHERTYPE_IP		0x0800
#define PAFRM_ETHERTYPE_IPV6		0x86dd
#define PAFRM_ETHERTYPE_VLAN		0x8100
#define PAFRM_ETHERTYPE_SPVLAN		0x88a8
#define PAFRM_ETHERTYPE_MPLS		0x8847
#define PAFRM_ETHERTYPE_MPLS_MULTI	0x8848

/* Next header type values  */
#define PAFRM_HDR_MAC			0
#define PAFRM_HDR_VLAN			1
#define PAFRM_HDR_MPLS			2
#define PAFRM_HDR_IPv4			3
#define PAFRM_HDR_IPv6			4
#define PAFRM_HDR_IPv6_EXT_HOP		5
#define PAFRM_HDR_IPv6_EXT_ROUTE	6
#define PAFRM_HDR_IPv6_EXT_FRAG		7
#define PAFRM_HDR_IPv6_EXT_DEST		8
#define PAFRM_HDR_GRE			9
#define PAFRM_HDR_ESP			10
#define PAFRM_HDR_ESP_DECODED		11
#define PAFRM_HDR_AUTH			12
#define PAFRM_HDR_CUSTOM_C1		13
#define PAFRM_HDR_FORCE_LOOKUP		14   /* A contrived header type used with custom SRIO to force
                                           a parse after looking at only the RIO L0-L2 */
#define PAFRM_HDR_SCTP			15
#define PAFRM_HDR_UNKNOWN		16
#define PAFRM_HDR_UDP			17
#define PAFRM_HDR_UDP_LITE		18
#define PAFRM_HDR_TCP			19
#define PAFRM_HDR_GTPU			20
#define PAFRM_HDR_ESP_DECODED_C2	21
#define PAFRM_HDR_CUSTOM_C2		22

/* Command related definitions */
#define PAFRM_CRC_FLAG_CRC_OFFSET_VALID		0x01
#define PAFRM_CRC_FLAG_CRC_OFFSET_FROM_DESC	0x02
#define PAFRM_CHKSUM_FALG_NEGATIVE		0x01

#define PA_NEXT_ROUTE_PARAM_PRESENT		0x0001
#define PA_NEXT_ROUTE_PROC_NEXT_CMD		0x0002
#define PA_NEXT_ROUTE_PROC_MULTI_ROUTE		0x0004

/* PAFRM receive commands related definitions */

/* 
 * There are the following two groups of PAFRM receive commands:
 * PAFRM short commands which can be used as part of the routing info 
 * PAFRM commands which can be used within a command set
 */
 
#define PAFRM_RX_CMD_NONE		0           /* Dummy command */

/* short commands */
#define PAFRM_RX_CMD_CMDSET		1           /* Execute a command set */
#define PAFRM_RX_CMD_INSERT		2           /* Insert up to two types at the current location */

/* command set commands */
#define PAFRM_RX_CMD_NEXT_ROUTE		3           /* Specify the next route */
#define PAFRM_RX_CMD_CRC_OP		4           /* CRC generation or verification */
#define PAFRM_RX_CMD_COPY_DATA		5           /* Copy data to the PS Info section */
#define PAFRM_RX_CMD_PATCH_DATA		6           /* Insert or pacth packet data at the specific location */
#define PAFRM_RX_CMD_REMOVE_HDR		7           /* Remove the parsed packet header */
#define PAFRM_RX_CMD_REMOVE_TAIL	8           /* Remove the parsed packet tail */
#define PAFRM_RX_CMD_MULTI_ROUTE	9           /* Duplicate packet to multiple destinations */

/*
 * PASS command ID formatting
 * Bit 15 is used to distinguish the L2 table from
 * the L3 table in the command comId field
 */
#define PA_COMID_L2		(0 << 15)
#define PA_COMID_L3		(1 << 15)
#define PA_COMID_L_MASK		(1 << 15)
#define PA_COMID_IDX_MASK	(~(1 << 15))

/* define LUT1 entry types */
#define PAFRM_COM_ADD_LUT1_STANDARD	0	/* MAC/IP */
#define PAFRM_COM_ADD_LUT1_SRIO		1	/* SRIO */
#define PAFRM_COM_ADD_LUT1_CUSTOM	2   /* Custom LUT1 */

struct pa_frm_cmd_add_lut1 {

	u8	index;		/* LUT1 index. */
	u8	type;		/* Custom or standard */
	u8	rsvd;		/* reserved for alignment */
	u8	cust_index;     /* Vaild only if type is custom */
	
	union {
		struct	pa_frm_com_l1_standard	eth_ip;   /* matching information for MAC/IP entry */
		struct	pa_frm_com_l1_srio	srio;
		struct	pa_frm_com_l1_custom	custom;
	} u;

	struct	pa_frm_forward match;	/* Routing information when a match is found */
  
	/*
	 * Routing information when subsequent match fails - a fragmented
	 * packet orinner route
	 */
	struct	pa_frm_forward next_fail;
};

/* CRC Engine Configuration */
#define PARAM_CRC_TABLE_SIZE    16

struct pa_frm_config_crc {
	u8	ctrl_bitmap;			/* Control bit maps as defined below */
#define PARAM_CRC_SIZE_8         0
#define PARAM_CRC_SIZE_16        1
#define PARAM_CRC_SIZE_24        2
#define PARAM_CRC_SIZE_32        3

#define PARAM_CRC_CTRL_CRC_SIZE_MASK    0x3
#define PARAM_CRC_CTRL_LEFT_SHIFT       0x0
#define PARAM_CRC_CTRL_RIGHT_SHIFT      0x4
#define PARAM_CRC_CTRL_INV_RESULT       0x8

	u8	rsvd1;				/* reserved for alignment */
	u16	rsvd2;				/* reserved for alignment */
	u32	init_val;			/* Initial value to use in the CRC calcualtion */
	u32	crc_tbl[PARAM_CRC_TABLE_SIZE];	/* CRC table */
};

/* Commands to PA */
struct pa_frm_command {

	u32	command_result; /* Returned to the host, ignored on entry to the PASS */
	u8	command;	/* Command value */
	u8	magic;		/* Magic value */
	u16	com_id;		/* Used by the host to identify command results */
	u32	ret_context;	/* Returned in swInfo to identify packet as a command */
	u16	reply_queue;	/* Specifies the queue number for the message reply. 0xffff to toss the reply */
	u8	reply_dest;	/* Reply destination (host0, host1, discard are the only valid values) */
	u8	flow_id;	/* Flow ID used to assign packet at reply */
	u32	cmd;		/* First word of the command */
};

struct pa_cmd_next_route {
	u16	ctrl_bit_field;		/* Routing control information as defined at @ref routeCtrlInfo */	
	int	dest;			/* Packet destination as defined at @ref pktDest */
	u8	pkt_type_emac_ctrl;	/*  For destination SRIO, specify the 5-bit packet type toward SRIO 
                                     For destination EMAC, specify the EMAC control @ref emcOutputCtrlBits to the network */
	u8	flow_id;	/* For host, SA or SRIO destinations, specifies return free descriptor setup */
	u16	queue;		/*For host, SA or SRIO destinations, specifies the dest queue */
	u32	sw_info_0;	/* Placed in SwInfo0 for packets to host or SA */
	u32	sw_info_1;         /* Placed in SwInfo1 for packets to the SA */
	u16	multi_route_index; /* Multi-route index. It is valid in the from-network direction only */
};

struct pa_cmd_crcOp {
	u16	ctrl_bit_field;    /* CRC operation control information as defined at @ref crcOpCtrlInfo */
	u16	start_offset;     /* Byte location, from SOP/Protocol Header, where the CRC computation begins 
                                    if frame type is not specified
                                    Byte location, from SOP/Protocol header, where the specific frame header begins
                                    if frame type is specified
                                    In to-network direction: offset from SOP
                                    In from-network direction: offset from the current parsed header 
                                    */
	u16	len;             /* Number of bytes covered by the CRC computation 
                                    valid only if pa_CRC_OP_PAYLOAD_LENGTH_IN_HEADER is clear */
	u16	len_offset;       /* Payload length field offset in the custom header */
	u16	len_mask;         /* Payload length field mask */
	u16	len_adjust;       /* Payload length adjustment: valid only if PA_CRC_OP_PAYLOAD_LENGTH_IN_HEADER is set */
	u16	crc_offset;       /* Offset from SOP/Protocol Header to the CRC field 
                                    In to-network direction: offset from SOP
                                    In from-network direction: offset from the current parsed header */
	u16	frame_yype;       /* Frame type @ref crcFrameTypes, vaild if
			    PA_CRC_OP_CRC_FRAME_TYPE is set */
};

/**
 *  @ingroup palld_api_structures
 *  @brief  Transmit checksum configuration
 *
 *  @details  paTxChksum_t is used in the call to @ref Pa_formatTxRoute or @ref Pa_formatTxCmd to create a tx 
 *            command header that instructs the packet accelerator sub-system to generate ones' complement
 *             checksums into network packets. The checksums are typically used for TCP and UDP payload checksums as
 *            well as IPv4 header checksums. In the case of TCP and UDP payload checksums the psuedo header
 *            checksum must be pre-calculated and provided, the sub-system does not calculate it.
 */
struct pa_tx_chksum {
	u16	start_offset;   /* Byte location, from SOP, where the checksum calculation begins */
	u16	length_bytes;   /* Number of bytes covered by the checksum. Must be even */
	u16	result_offset;  /* Byte offset, from startOffset, to place the resulting checksum */
	u16	initial_sum;    /* Initial value of the checksum */
	u16	negative_0;     /* If TRUE, a computed value of 0 is written as -0 */
};

struct pa_cmd_copy {
	u16	ctrl_bitfield;    /* Copy operation control information as defined at @ref copyCtrlInfo */
	u16	src_offset;       /* Offset from the start of current protocol header for the data copy to begin */
	u16	dest_offset;      /* Offset from the top of the PSInfo for the data to be copied to */
	u16	num_bytes;        /* Number of bytes to be copied */   
};

struct pa_patch_info{
	unsigned int	n_patch_bytes;              /**<  The number of bytes to be patched */
	unsigned int	total_patch_size;           /**<  The number of patch bytes in the patch command, must be >= to nPatchBytes and a multiple of 4 bytes */
	unsigned int	offset;                   /**<  Offset from the start of the packet for the patch to begin in the to-network direction 
                                                 Offset from the start of the current header for the patch to begin in the from-network direction */
	u16		overwrite;                /**<  If TRUE the patch data replaces existing packet data. If false the data is added */
	u8		*patch_data;                /**<  Pointer to the patch data */
};


/**
 *  @ingroup palld_api_structures
 *  @brief  paPayloadInfo_t defines the packet payload information in the short format.
 *          It is required by the Security Accelerator sub-system (SASS)
 *
 *  @details paPayloadInfo_t defines the packet parsing information in terms of
 *           payload offset and payload length as described below
 *  @li      SRTP:      offset to the RTP header; RTP payload length including ICV
 *  @li      IPSEC AH:  offset to the Outer IP; IP payload length
 *  @li      IPSEC ESP: offset to the ESP header; ESP papload length including ICV
 */

struct pa_payload_info  {
	u16	offset;	/* The offset to where the SA packet parsing starts */
	u16	len;	/* The total length of the protocal payload to be processed by SA */
};

struct pa_cmd_multi_route {
	u16	index;        /*  Multi-route set Index */
};

/**
 *   @def  PA_MAX_CMD_SETS
 *         The maximum number of command sets supported
 */
#define PA_MAX_CMD_SETS     8

#define PA_OK					0
#define PA_ERR_CONFIG				-10
#define PA_INSUFFICIENT_CMD_BUFFER_SIZE		-11
#define PA_INVALID_CMD_REPLY_DEST		-12

/**
 *  @ingroup palld_api_structures
 *  @brief  Command Set Command
 *
 *  @details paCmdSet_t is used to specify the desired PA command set. The command set command 
 *           instructs the PASS to execute a list of commands after a LUT1 or LUT2 match occurs. 
 *           It is one of the command which can be embedded within the @ref paRouteInfo_t. 
 */
struct pa_cmd_set {
	u16	index;        /*Command Set Index */
};

struct pa_cmd_tx_timestamp {
	u16	dest_queue;	/* Host queue for the tx timestamp reporting packet */
	u16	flow_id;	/* CPPI flow */
	u32	sw_info0;	/* 32 bit value returned in the descriptor */
};

struct pa_cmd_ip_frag {
	u16	ip_offset;	/* Offset to the IP header. */
	u16	mtu_size;	/* Size of the maximum transmission unit (>= 68) */
};

struct pa_cmd_usr_stats {
	u16	index;		/* User-defined statistics index */
};

struct pa_cmd_set_usr_stats {
	u16	set_index;	/* Commad Set Index */
	u16	stats_index;    /* User-defined statistics index */
};

struct pa_cmd_info {
	u16	cmd;			/*Specify the PA command code as defined at @ref paCmdCode */
	union {
		struct pa_cmd_next_route route;	/* Specify nextRoute command specific parameters */
		struct pa_tx_chksum	chksum;	/* Specify Tx Checksum command specific parameters */
		struct pa_cmd_crcOp     crcOp;    /* Specify CRC operation command specific parameters */
		struct pa_cmd_copy	copy;     /* Specify Copy command specific parameters */
		struct pa_patch_info	patch;    /* Specify Patch command specific parameters */
		struct pa_payload_info	payload;  /* Specify the payload information required by SA */
		struct pa_cmd_set	cmd_set;   /* Specify Command Set command specific parameters */
		struct pa_cmd_multi_route m_route;   /* Specify Multi-route command specific parameters */
		struct pa_cmd_tx_timestamp tx_ts;     /*Specify Report Tx Timestamp command specific parameters */
		struct pa_cmd_ip_frag	ip_frag;   /* Specify IP fragmentation command specific parameters */
		struct pa_cmd_usr_stats usr_stats; /* Specify User-defined Statistics command specific parameters */
		struct pa_cmd_set_usr_stats cmd_set_usr_stats;  
	} params;
};

#define PAFRM_ROUTE_
struct pa_route_info {
	int	dest;
	u8	flow_id;
	u16	queue;
	int	m_route_index;
	u32	sw_info_0;
	u32	sw_info_1;
	int	custom_type;
	u8	custom_index;                                    
	u8	pkt_type_emac_ctrl;
	u8	route_type;
	struct pa_cmd_info *pcmd;
};

struct pa_cmd_reply {
	int	dest;		/* Packet destination, must be pa_DEST_HOST or PA_DEST_DISCARD, see @ref pktDest */
	u32	reply_id;	/*  Value placed in swinfo0 in reply packet */
	u16	queue;		/*  Destination queue for destination PA_DEST_HOST */
	u8	flow_id;	/*  Flow ID used on command reply from PASS */
};

/* Exception routing enumeration */
enum pa_eroutes {
	EROUTE_LUT1_FAIL = 0,  /* packet failed to match in LUT1 table */
	EROUTE_VLAN_MAX_DEPTH, /* packet exceeded maximum number of VLAN tags */
	EROUTE_IP_MAX_DEPTH,   /* packet exceeded maximum number of IP
				  headers */
	EROUTE_MPLS_MAX_DEPTH, /* packet exceeded maximum number of MPLS
				  headers */
	EROUTE_GRE_MAX_DEPTH,  /* packet exceeded maximum number of GRE
				  headers */
	EROUTE_PARSE_FAIL,     /* packet failed to parse */
	EROUTE_LUT2_FAIL,      /* packet failed to match in LUT2 table */
	EROUTE_IP_FRAG,        /* IP fragmented packet found in classify2
				  lookup */
	EROUTE_IPV6_OPT_FAIL,  /* Packet failed due to unsupported IPV6 option
				  header */
	EROUTE_UDP_LITE_FAIL,  /* Udp lite checksum coverage invalid */
	EROUTE_ROUTE_OPTION,   /* IPv4 strict source route or IPv6 routing
				  extension header */
	EROUTE_SYSTEM_FAIL,    /* Unknown system failure - should never
				  happen */
	EROUTE_MAC_BROADCAST,  /* MAC broadcast packet */
	EROUTE_MAC_MULTICAST,  /* MAC multicast packet */
	EROUTE_IP_BROADCAST,   /* IP broadcast packet */
	EROUTE_IP_MULTICAST,   /* IP multicast packet */
	EROUTE_GTPU_MESSAGE_TYPE_1,   /* GTP-U PING Request packet */
	EROUTE_GTPU_MESSAGE_TYPE_2,   /* GTP-U PING Response packet */
	EROUTE_GTPU_MESSAGE_TYPE_26,  /* GTP-U Error Indication packet */
	EROUTE_GTPU_MESSAGE_TYPE_31,  /* GTP-U Supported Header Notification
					 packet */
	EROUTE_GTPU_MESSAGE_TYPE_254, /* GTP-U End Markr packet */
	EROUTE_GTPU_FAIL,             /* packet failed due to GTPU parsing
					 error or unsupporte dmessage types */
	EROUTE_PPPOE_FAIL,            /* Packet failed due to PPPoE session
					 packet parsing error */
	EROUTE_PPPOE_CTRL,            /* PPPoE session stage non-IP packets */
	EROUTE_802_1ag,               /* 802.1ag Packet*/
	EROUTE_IP_FAIL,               /* Packet failed due to invalid IP
					 header */
	EROUTE_NAT_T_KEEPALIVE,       /* NAT-T Keep Alive packet where UDP
					 Length = 9, data = 0xFF */
	EROUTE_NAT_T_CTRL,            /* NAT-T control packet where UDP Length
					 > 12 and the first 4 payload bytes are
					 equal to 0 */
	EROUTE_NAT_T_DATA,            /* NAT-T IPSEC ESP data packet where UDP
					 Length > 12 and the first 4 payload
					 bytes are not equal to 0 */
	EROUTE_NAT_T_FAIL,            /* Invalid NAT-T packet */
	EROUTE_GTPU_MATCH_FAIL,       /* Packet failed to match GTPU */
	EROUTE_N_MAX                  /* Number of error routes */
};

/* exception route configuration */
struct pa_frm_com_eroute {
	/* Exception route vaild bitmap */
	u32			route_bitmap;
	/* Array of exception routing information */
	struct pa_frm_forward	eroute[EROUTE_N_MAX];
};

/* PA system configuration command */
struct pa_frm_command_sys_config_pa {
	u8	cfg_code; /* system configuration code as defined below */
	u8	rsvd1;
	u16	rsvd2;    /* reserved for alignment */

	union {
		/* Exception routes configuration */
		struct pa_frm_com_eroute eroute;
	} u;
};

/* PA system configuration codes */
#define PAFRM_SYSTEM_CONFIG_CODE_EROUTE         0
#define PAFRM_SYSTEM_CONFIG_CODE_CUSTOM_LUT1    1
#define PAFRM_SYSTEM_CONFIG_CODE_CUSTOM_LUT2    2
#define PAFRM_SYSTEM_CONFIG_CODE_802_1AG        3
#define PAFRM_SYSTEM_CONFIG_CODE_IPSEC_NAT_T    4
#define PAFRM_SYSTEM_CONFIG_CODE_GTPU           5

#endif /* __KERNEL__ */

#endif /* KEYSTONE_PA_H */
