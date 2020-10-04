/*
 * Copyright (C) 2013 Texas Instruments Incorporated
 * Author: Hao Zhang <hzhang@ti.com>
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

#ifndef KEYSTONE_PASAHOST2_H
#define KEYSTONE_PASAHOST2_H

#ifdef __KERNEL__

#define	PASAHO2_CONFIGURE		4
#define PASAHO2_PARX_PARSECMD		0
#define PASAHO2_PARX_MULTI_ROUTE		5
#define PASAHO2_PAMOD_CMPT_CHKSUM	0
#define PASAHO2_PAMOD_CMPT_CRC		1
#define PASAHO2_PAMOD_PATCH		2
#define PASAHO2_PAMOD_NROUTE		3
#define PASAHO2_PAMOD_EF_OP		5
#define PASAHO2_PAMOD_REPORT_TIMESTAMP	6
#define PASAHO2_PAMOD_GROUP_7		7
#define PASAHO2_PAMOD_DUMMY		PASAHO2_PAMOD_GROUP_7
#define PASAHO2_PAMOD_IP_FRAGMENT	PASAHO2_PAMOD_GROUP_7
#define PASAHO2_SA_LONG_INFO		0
#define PASAHO2_SA_SHORT_INFO		1
#define PASAHO2_SA_AIR_INFO		2

#define PASAHO2_READ_BITFIELD(a, b, c)	(((a) >> (b)) & ((1 << (c)) - 1))

#define PASAHO2_SET_BITFIELD(a, x, b, c) (a) &= ~(((1 << (c)) - 1) << (b)), \
					(a) |= (((x) & ((1 << (c)) - 1)) << (b))

#define PASAHO2_SET_CMDID(x, v)	PASAHO2_SET_BITFIELD((x)->word0, (v), 29, 3)

#define PASAHO2_PACFG_CMD	(((u32)PASAHO2_CONFIGURE << 5) << 24)

enum pasaho2_header_type {
	PASAHO2_HDR_MAC	= 0,		/* MAC */
	PASAHO2_HDR_VLAN,		/* VLAN */
	PASAHO2_HDR_MPLS,		/* MPLS */
	PASAHO2_HDR_IPv4,		/* IPv4 */
	PASAHO2_HDR_IPv6,		/* IPv6 */
	PASAHO2_HDR_IPv6_EXT_HOP,	/* IPv6 hop by hop extenstion header */
	PASAHO2_HDR_IPv6_EXT_ROUTE,	/* IPv6 routing extenstion header */
	PASAHO2_HDR_IPv6_EXT_FRAG,	/* IPv6 fragmentation extention hdr */
	PASAHO2_HDR_IPv6_EXT_DEST,	/* IPv6 destination options header */
	PASAHO2_HDR_GRE,		/* Generic Routing Encapsulation
					   header */
	PASAHO2_HDR_ESP,		/* Encapsulating Security Payload
					   header */
	PASAHO2_HDR_ESP_DECODED,	/* Decoded Encapsulating Security
					   Payload header */
	PASAHO2_HDR_AUTH,		/* Authentication header */
	PASAHO2_HDR_CUSTOM_C1,		/* Custom classify 1 header */
	PASAHO2_HDR_PPPoE,		/* PPPoE Header */
	PASAHO2_HDR_SCTP,		/* SCTP Header */
	PASAHO2_HDR_UNKNOWN,		/* Next header type is unknown */
	PASAHO2_HDR_UDP,		/* User Datagram Protocol header */
	PASAHO2_HDR_UDP_LITE,		/* Lightweight User Datagram Protocol
					   header */
	PASAHO2_HDR_TCP,		/* Transmission Control Protocol
					   header */
	PASAHO2_HDR_GTPU,		/* GTPU header */
	PASAHO2_HDR_ESP_DECODED_C2,	/* Decoded Encapsulating Security
					   Payload header at Classifyer2 */
	PASAHO2_HDR_CUSTOM_C2		/* Custom classify 2 header */
};

/* Definition of the 5-bit sub-command codes which is used to specify the group
   7 commands. */
enum pasaho2_sub_cmd_code {
	PASAHO2_SUB_CMD_DUMMY	= 0,	/* Dummy */
	PASAHO2_SUB_CMD_IP_FRAG,		/* IPv4 fragmentation */
	PASAHO2_SUB_CMD_PATCH_MSG_LEN	/* Message length Patching */
};

/* pasaho2_cmd_info defines the general short command information */
struct pasaho2_cmd_info {
	u32	word0;		/* Control block word 0 */
};

/* pasaho2_long_info defines the packet parsing information in the long format.
 * The information is structured as an array of 32 bit values. These values
 * are broken down through macros. This allows the representation to be
 * endian independent to the hardware which operates only on 32 bit values. */
struct pasaho2_long_info {
	u32   word0;	/* Control block word 0 */
	u32   word1;	/* Control block word 1 */
	u32   word2;	/* Control block word 2 */
	u32   word3;	/* Control block word 3 */
	u32   word4;	/* Control block word 4 */
};

/* Macros used by the PASAHO Long Info Command */

/* Extract the command ID defined at */
#define PASAHO2_LINFO_READ_CMDID(x)	PASAHO2_READ_BITFIELD((x)->word0, 29, 3)

/* Extract the block length */
#define PASAHO2_LINFO_READ_RECLEN(x)	PASAHO2_READ_BITFIELD((x)->word0, 24, 5)

/* Extract the next parse start offset */
#define PASAHO2_LINFO_READ_START_OFFSET(x) \
	PASAHO2_READ_BITFIELD((x)->word0, 0, 8)

/* Indicate whether it is a broadcast MAC packet */
#define PASAHO2_LINFO_IS_MAC_BROADCAST(x) \
	PASAHO2_READ_BITFIELD((x)->word0, 16, 1)

/* Indicate whether it is a multicast MAC packet */
#define PASAHO2_LINFO_IS_MAC_MULTICAST(x) \
	PASAHO2_READ_BITFIELD((x)->word0, 17, 1)

/* Extract the MAC packet type */
#define PASAHO2_LINFO_READ_MAC_PKTTYPE(x) \
	PASAHO2_READ_BITFIELD((x)->word0, 16, 2)

/* Indicate whether it is a broadcast IP packet */
#define PASAHO2_LINFO_IS_IP_BROADCAST(x) \
	PASAHO2_READ_BITFIELD((x)->word0, 16, 1)

/* Indicate whether it is a multicast IP packet */
#define PASAHO2_LINFO_IS_IP_MULTICAST(x) \
	PASAHO2_READ_BITFIELD((x)->word0, 17, 1)

/* Extract the IP packet type */
#define PASAHO2_LINFO_READ_IP_PKTTYPE(x) \
	PASAHO2_READ_BITFIELD((x)->word0, 16, 2)

/* Extract the previous match flag */
#define PASAHO2_LINFO_READ_PMATCH(x)	PASAHO2_READ_BITFIELD((x)->word0, 23, 1)

/* Extract the fragmentation found flag */
#define PASAHO2_LINFO_READ_FLAG_FRAG(x)	PASAHO2_READ_BITFIELD((x)->word0, 19, 1)

/* Extract the end of packet parse offset */
#define PASAHO2_LINFO_READ_END_OFFSET(x) \
	PASAHO2_READ_BITFIELD((x)->word1, 16, 16)

/* Extract the error index */
#define PASAHO2_LINFO_READ_EIDX(x)	PASAHO2_READ_BITFIELD((x)->word1, 10, 6)

/* Extract the next header to parse type */
#define PASAHO2_LINFO_READ_NXT_HDR_TYPE(x) \
	PASAHO2_READ_BITFIELD((x)->word1, 0, 6)

/* Extract the (1-based) input EMAC port number through CPSW */
#define PASAHO2_LINFO_READ_INPORT(x)	PASAHO2_READ_BITFIELD((x)->word1, 6, 4)

/* Extract the offset to the level 3 header */
#define PASAHO2_LINFO_READ_L3_OFFSET(x)	PASAHO2_READ_BITFIELD((x)->word2, 24, 8)

/* Extract the offset to the level 4 header */
#define PASAHO2_LINFO_READ_L4_OFFSET(x)	PASAHO2_READ_BITFIELD((x)->word2, 16, 8)

/* Extract the offset to the level 5 header */
#define PASAHO2_LINFO_READ_L5_OFFSET(x)	PASAHO2_READ_BITFIELD((x)->word2, 8, 8)

/* Extract the offset to the security header */
#define PASAHO2_LINFO_READ_ESP_AH_OFFSET(x) \
	PASAHO2_READ_BITFIELD((x)->word2, 0, 8)

/* Extract the first parse module ID */
#define PASAHO2_LINFO_READ_L1_PDSP_ID(x) \
	PASAHO2_READ_BITFIELD((x)->word3, 26, 6)

/* Extract the first parse module match index */
#define PASAHO2_LINFO_READ_L1_IDX(x) \
	PASAHO2_READ_BITFIELD((x)->word3, 16, 10)

/* Extract the bitmask of parsed header types */
#define PASAHO2_LINFO_READ_HDR_BITMASK(x) \
	PASAHO2_READ_BITFIELD((x)->word3, 0, 16)

/* Extract the number of VLAN tags found */
#define PASAHO2_LINFO_READ_VLAN_COUNT(x) PASAHO2_READ_BITFIELD((x)->word4, 6, 2)

/* Extract the number of IP headers found */
#define PASAHO2_LINFO_READ_IP_COUNT(x)	PASAHO2_READ_BITFIELD((x)->word4, 0, 3)

/* Extract the number of GRE headers found */
#define PASAHO2_LINFO_READ_GRE_COUNT(x)	PASAHO2_READ_BITFIELD((x)->word4, 3, 3)

/*  Extract the last pseudo-header checksum computed (depreciated) */
#define PASAHO2_LINFO_READ_PSEUDO_CHKSM(x) \
	PASAHO2_READ_BITFIELD((x)->word5, 16, 16)

/**< Extract the offset to the inner IP header */
#define PASAHO2_LINFO_READ_INNER_IP_OFFSET(x) \
	PASAHO2_READ_BITFIELD((x)->word5, 24, 8)

/* Extract the most significant 16-bit of the 48-bit timestamp */
#define PASAHO2_LINFO_READ_TSTAMP_MSB(x) \
	PASAHO2_READ_BITFIELD((x)->word5, 0, 16)


/* Indicate whether it is a MAC packet */
#define PASAHO2_LINFO_IS_MAC(x)		PASAHO2_READ_BITFIELD((x)->word3, 0, 1)

/* Indicate whether it is a MAC packet with VLAN */
#define PASAHO2_LINFO_IS_WITH_VLAN(x)	PASAHO2_LINFO_READ_VLAN_COUNT(x)

/* Indicate whether it is a MAC packet with MPLS */
#define PASAHO2_LINFO_IS_WITH_MPLS(x)	PASAHO2_READ_BITFIELD((x)->word3, 2, 1)

/* Indicate whether it is a 802.3 packet */
#define PASAHO2_LINFO_IS_802_3(x)	PASAHO2_READ_BITFIELD((x)->word3, 3, 1)

/* Indicate whether it is a PPPoE packet */
#define PASAHO2_LINFO_IS_PPPoE(x)	PASAHO2_READ_BITFIELD((x)->word3, 4, 1)

/* Indicate whether it is an IP packet */
#define PASAHO2_LINFO_IS_IP(x)		PASAHO2_LINFO_READ_IP_COUNT(x)

/* Indicate whether it is an IPv4 packet */
#define PASAHO2_LINFO_IS_IPv4(x)	PASAHO2_READ_BITFIELD((x)->word3, 5, 1)

/* Indicate whether it is an IPv4 packet */
#define PASAHO2_LINFO_IS_IPv6(x)	PASAHO2_READ_BITFIELD((x)->word3, 6, 1)

/* Indicate whether there are IPV4 options or IPv6 extention headers */
#define PASAHO2_LINFO_IS_IP_OPTIONS(x)	PASAHO2_READ_BITFIELD((x)->word3, 7, 1)

/* Indicate whether it is an IPSEC packet */
#define PASAHO2_LINFO_IS_IPSEC(x)	PASAHO2_READ_BITFIELD((x)->word3, 8, 2)

/* Indicate whether it is an IPSEC ESP packet */
#define PASAHO2_LINFO_IS_IPSEC_ESP(x)	PASAHO2_READ_BITFIELD((x)->word3, 8, 1)

/* Indicate whether it is an IPSEC AH packet */
#define PASAHO2_LINFO_IS_IPSEC_AH(x)	PASAHO2_READ_BITFIELD((x)->word3, 9, 1)

/* Indicate whether it is a SCTP packet */
#define PASAHO2_LINFO_IS_SCTP(x)	PASAHO2_READ_BITFIELD((x)->word3, 10, 1)

/* Indicate whether it is an UDP packet */
#define PASAHO2_LINFO_IS_UDP(x)		PASAHO2_READ_BITFIELD((x)->word3, 11, 1)

/* Indicate whether it is an UDP Lite packet */
#define PASAHO2_LINFO_IS_UDP_LITE(x)	PASAHO2_READ_BITFIELD((x)->word3, 11, 1)

/* Indicate whether it is a TCP packet */
#define PASAHO2_LINFO_IS_TCP(x)		PASAHO2_READ_BITFIELD((x)->word3, 12, 1)

/* Indicate whether it is a GRE packet */
#define PASAHO2_LINFO_IS_GRE(x)		PASAHO2_LINFO_READ_GRE_COUNT(x)

/* Indicate whether it is a GTPU packet */
#define PASAHO2_LINFO_IS_GTPU(x)	PASAHO2_READ_BITFIELD((x)->word3, 13, 1)

/* Indicate whether it is a Custom packet */
#define PASAHO2_LINFO_IS_CUSTOM(x)	PASAHO2_READ_BITFIELD((x)->word3, 14, 1)

/* Indicate whether it is an IPSEC NAT-T packet */
#define PASAHO2_LINFO_IS_IPSEC_NAT_T(x)	PASAHO2_READ_BITFIELD((x)->word3, 15, 1)


/* Extract the IP Reassembly Traffic Flow Index */
#define PASAHO2_LINFO_READ_TFINDEX(x)	PASAHO2_READ_BITFIELD((x)->word5, 24, 8)

/* Extract the IP Reassembly Fragment count */
#define PASAHO2_LINFO_READ_FRANCNT(x)	PASAHO2_READ_BITFIELD((x)->word5, 16, 8)

/* Set the IP Reassembly Traffic Flow Index */
#define PASAHO2_LINFO_SET_TFINDEX(x, v) \
	PASAHO2_SET_BITFIELD((x)->word5, (v), 24, 8)

/* Set the IP Reassembly Fragment count */
#define PASAHO2_LINFO_SET_FRANCNT(x, v) \
	PASAHO2_SET_BITFIELD((x)->word5, (v), 16, 8)


/* Clear IPSEC indication bits */
#define PASAHO2_LINFO_CLR_IPSEC(x) \
	PASAHO2_SET_BITFIELD((x)->word3, 0, 8, 2)

/* Clear IPSEC ESP indication bit */
#define PASAHO2_LINFO_CLR_IPSEC_ESP(x) \
	PASAHO2_SET_BITFIELD((x)->word3, 0, 8, 1)

/* Claer IPSEC AH indication bit */
#define PASAHO2_LINFO_CLR_IPSEC_AH(x) \
	PASAHO2_SET_BITFIELD((x)->word3, 0, 9, 1)

/* Clear the fragmentation found flag */
#define PASAHO2_LINFO_CLR_FLAG_FRAG(x) \
	PASAHO2_SET_BITFIELD((x)->word1, 0, 19, 1)

/* Update the next parse start offset */
#define PASAHO2_LINFO_SET_START_OFFSET(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 0, 8)

/* Update the end of packet parse offset */
#define PASAHO2_LINFO_SET_END_OFFSET(x, v) \
	PASAHO2_SET_BITFIELD((x)->word1, (v), 16, 16)

/* Update the next header to parse type */
#define PASAHO2_LINFO_SET_NXT_HDR_TYPE(x, v) \
	PASAHO2_SET_BITFIELD((x)->word1, (v), 0, 6)

/* Set the null packet flag which indicates that the packet should be dropped.
 * This flag should be set for the null packet to be delivered to PASS when the
 * reassembly timeout occurs */
#define PASAHO2_LINFO_SET_NULL_PKT_IND(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 21, 1)

/*
 * PA_INV_TF_INDEX
 * PASS-asssited IP reassembly traffic flow index to indicate
 * that no traffic flow is available
 */
#define PA_INV_TF_INDEX		0xFF

struct pasaho2_short_info {
	u32	word0;
	u32	word1;
};

/* Extract the command ID defined at */
#define PASAHO2_SINFO_READ_CMDID(x)	PASAHO2_READ_BITFIELD((x)->word0, 29, 3)

/* Extract the offset to the packet payload */
#define PASAHO2_SINFO_RESD_PAYLOAD_OFFSET(x) \
	PASAHO2_READ_BITFIELD((x)->word0, 16, 8)

/* Extract the byte length of the payload */
#define PASAHO2_SINFO_READ_PAYLOAD_LENGTH(x) \
	PASAHO2_READ_BITFIELD((x)->word0, 0, 16)

/* Set the offset to the payload */
#define PASAHO2_SINFO_SET_PAYLOAD_OFFSET(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Set the payload length */
#define PASAHO2_SINFO_SET_PAYLOAD_LENGTH(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 0,  16)

/* Format the entire short info command */
#define PASAHO2_SINFO_FORMAT_CMD(offset, len) \
	(((offset) << 16) | (len) | (PASAHO2_SA_SHORT_INFO << 29))

#define PASAHO2_HDR_BITMASK_MAC		(1 << 0)  /* MAC present */
#define PASAHO2_HDR_BITMASK_VLAN		(1 << 1)  /* VLAN present */
#define PASAHO2_HDR_BITMASK_MPLS		(1 << 2)  /* MPLS present */
#define PASAHO2_HDR_BITMASK_802_3	(1 << 3)  /* 802.3 present */
#define PASAHO2_HDR_BITMASK_PPPoE	(1 << 4)  /* PPPoE present */
#define PASAHO2_HDR_BITMASK_IPv4		(1 << 5)  /* IPv4 present */
#define PASAHO2_HDR_BITMASK_IPv6		(1 << 6)  /* IPv6 present */
#define PASAHO2_HDR_BITMASK_IP_OPTS	(1 << 7)  /* IPv4 options or IPv6
						     extension headers esent */
#define PASAHO2_HDR_BITMASK_ESP		(1 << 8)  /* IPSEC/ESP present */
#define PASAHO2_HDR_BITMASK_AH		(1 << 9)  /* IPSEC/AH present */
#define PASAHO2_HDR_BITMASK_SCTP		(1 << 10) /* SCTP present */
#define PASAHO2_HDR_BITMASK_UDP		(1 << 11) /* UDP present */
#define PASAHO2_HDR_BITMASK_UDPLITE	(1 << 11) /* UDPLITE present */
#define PASAHO2_HDR_BITMASK_TCP		(1 << 12) /* TCP present */
#define PASAHO2_HDR_BITMASK_GTPU		(1 << 13) /* GTPU present */
#define PASAHO2_HDR_BITMASK_CUSTOM	(1 << 14) /* Custom header present */
#define PASAHO2_HDR_BITMASK_IPSEC_NAT_T	(1 << 15) /* IPSEC NAT-T present */

struct pasaho2_next_route {
	u32  word0;
	u32  sw_info0;
	u32  sw_info1;
	u32  word1;
};

/*
 * Sets the N bit which indicates the next command
 * should be executed prior to the route command
 */
#define PASAHO2_SET_N(x, v)	PASAHO2_SET_BITFIELD((x)->word0, (v), 28, 1)

/*
 * Sets the E bit which indicates the extened
 * parameters (packet type) are present for SRIO
 */
#define PASAHO2_SET_E(x, v)	PASAHO2_SET_BITFIELD((x)->word0, (v), 27, 1)

/*
 * Sets the destination of the route defined */
#define PASAHO2_SET_DEST(x, v)	PASAHO2_SET_BITFIELD((x)->word0, (v), 24, 3)

/* Specifies the flow to use for packets sent to the host */
#define PASAHO2_SET_FLOW(x, v)	PASAHO2_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Specifies the queue to use for packets send to the host */
#define PASAHO2_SET_QUEUE(x, v)   PASAHO2_SET_BITFIELD((x)->word0, (v), 0,  16)

/* Specifies the packet type to use for packets send to the SRIO */
#define PASAHO2_SET_PKTTYPE(x, v) PASAHO2_SET_BITFIELD((x)->word1, (v), 24, 8)

struct pasaho2_com_chk_crc {
	u32	word0;		/* PASAHO2_chksum_command_macros */
	u32	word1;		/* PASAHO2_chksum_command_macros */
	u32	word2;		/* PASAHO2_chksum_command_macros */
};

/*
 * Sets the negative 0 flag - if set a
 * checksum computed as 0 will be sent as 0xffff
 */
#define PASAHO2_CHKCRC_SET_NEG0(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 23, 1)

/* Sets the optional flags of the CRC/Checksum command */
#define PASAHO2_CHKCRC_SET_CTRL(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 16, 4)

/* Sets the size of the crc in bytes */
#define PASAHO2_CHKCRC_SET_CRCSIZE(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 8,  8)

/* Sets the start offset of the checksum/crc */
#define PASAHO2_CHKCRC_SET_START(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 0,  8)

/* Sets the length of the checksum/crc */
#define PASAHO2_CHKCRC_SET_LEN(x, v) \
	PASAHO2_SET_BITFIELD((x)->word1, (v), 16, 16)

/* Sets the offset to where to paste the checksum/crc into the packet */
#define PASAHO2_CHKCRC_SET_RESULT_OFF(x, v) \
	PASAHO2_SET_BITFIELD((x)->word1, (v), 0,  16)

/* Sets the initial value of the checksum/crc */
#define PASAHO2_CHKCRC_SET_INITVAL(x, v) \
	PASAHO2_SET_BITFIELD((x)->word2, (v), 16, 16)

/* Sets the initial value of the 32-bit crc */
#define PASAHO2_CHKCRC_SET_INITVAL32(x, v)	((x)->word2) = (v)


#define PASAHO2_BPATCH_MAX_PATCH_WORDS	4

struct pasaho2_com_blind_patch {
	u32	word0;
	u32	patch[PASAHO2_BPATCH_MAX_PATCH_WORDS];
};


#define PASAHO2_BPATCH_SET_PATCH_NBYTES(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, v, 24,  5)

/* Sets the number of bytes to patch */
#define PASAHO2_BPATCH_SET_PATCH_CMDSIZE(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, v, 20, 4)

/* Sets the size of the command in 32 bit word units */
#define PASAHO2_BPATCH_SET_OVERWRITE(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, v, 19, 1)

/*
 * Sets the overwrite flag. If set the patch will
 * overwrite existing packet data, otherwise data is inserted
 */
#define PASAHO2_BPATCH_SET_OFFSET(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, v, 0,  16)

/* Sets the offset to the start of the patch */
#define PASAHO2_BPATCH_SET_PATCH_BYTE(x, byteNum, byte) \
	((x)->patch[(byteNum) >> 2]) = \
	PASAHO2_SET_BITFIELD((x)->patch[(byteNum) >> 2], \
	byte, ((3 - (byteNum & 0x3)) << 3), 8)


struct pasaho2_report_timestamp {
	u32	word0;
	u32	sw_info0;
};

/* Specifies the flow to use for report packets sent to the host */

#define PASAHO2_SET_REPORT_FLOW(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Specifies the queue to use for report packets send to the host */
#define PASAHO2_SET_REPORT_QUEUE(x, v) \
	PASAHO2_SET_BITFIELD((x)->word0, (v), 0,  16)

struct pasaho2_ip_frag {
	u32	word0;
};

/* Set sub-command code to indicate IP Fragmentation command */
#define PASAHO2_SET_SUB_CODE_IP_FRAG(x) \
	PASAHO2_SET_BITFIELD((x)->word0, PASAHO2_SUB_CMD_IP_FRAG, 24, 5)

/* Specifies the sub-command code */
#define PASAHO2_SET_SUB_CODE(x, v) PASAHO2_SET_BITFIELD((x)->word0, (v), 24, 5)

/* Specifies the offset to the IP header to be fragmented */
#define PASAHO2_SET_IP_OFFSET(x, v) PASAHO2_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Specifies the MTU size */
#define PASAHO2_SET_MTU_SIZE(x, v) PASAHO2_SET_BITFIELD((x)->word0, (v), 0,  16)

#endif /* __KERNEL__ */
#endif /* KEYSTONE_PASAHOST2_H */
