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

#ifndef KEYSTONE_PASAHOST_H
#define KEYSTONE_PASAHOST_H

#ifdef __KERNEL__

#define	PASAHO_CONFIGURE		4
#define PASAHO_PARX_PARSECMD		0
#define PASAHO_PARX_MULTI_ROUTE		5
#define PASAHO_PAMOD_CMPT_CHKSUM	0
#define PASAHO_PAMOD_CMPT_CRC		1
#define PASAHO_PAMOD_PATCH		2
#define PASAHO_PAMOD_NROUTE		3
#define PASAHO_PAMOD_MULTI_ROUTE	5
#define PASAHO_PAMOD_REPORT_TIMESTAMP	6   
#define PASAHO_PAMOD_GROUP_7		7   
#define PASAHO_PAMOD_DUMMY		PASAHO_PAMOD_GROUP_7
#define PASAHO_PAMOD_IP_FRAGMENT	PASAHO_PAMOD_GROUP_7
#define PASAHO_SA_LONG_INFO		0
#define PASAHO_SA_SHORT_INFO		1
#define PASAHO_SA_AIR_INFO		2

#define PASAHO_READ_BITFIELD(a,b,c)	(((a)>>(b)) & ((1UL<<(c))-1))

#define PASAHO_SET_BITFIELD(a,x,b,c)	(a) &= ~(((1UL<<(c))-1)<<(b)), \
					(a) |= (((x) & ((1UL<<(c))-1))<<(b))

#define PASAHO_SET_CMDID(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 29,3)

#define PASAHO_PACFG_CMD	(((u32)PASAHO_CONFIGURE << 5) << 24)

enum pasaho_header_type {
	PASAHO_HDR_MAC        = 0,        /* MAC */
	PASAHO_HDR_VLAN,                  /* VLAN */
	PASAHO_HDR_MPLS,                  /* MPLS */
	PASAHO_HDR_IPv4,                  /* IPv4 */
	PASAHO_HDR_IPv6,                  /* IPv6 */
	PASAHO_HDR_IPv6_EXT_HOP,          /* IPv6 hop by hop extenstion header */
	PASAHO_HDR_IPv6_EXT_ROUTE,        /* IPv6 routing extenstion header */
	PASAHO_HDR_IPv6_EXT_FRAG,         /* IPv6 fragmentation extention header */
	PASAHO_HDR_IPv6_EXT_DEST,         /* IPv6 destination options header */
	PASAHO_HDR_GRE,                   /* Generic Routing Encapsulation header */
	PASAHO_HDR_ESP,                   /* Encapsulating Security Payload header */
	PASAHO_HDR_ESP_DECODED,           /* Decoded Encapsulating Security Payload header */
	PASAHO_HDR_AUTH,                  /* Authentication header */
	PASAHO_HDR_CUSTOM_C1,             /* Custom classify 1 header */
	PASAHO_HDR_FORCE_LOOKUP,          /* A contrived header type used with custom SRIO to force
                                        a parse after looking at only the SRIO L0-L2 */
	PASAHO_HDR_UNKNOWN,               /* Next header type is unknown */
	PASAHO_HDR_UDP,                   /* User Datagram Protocol header */
	PASAHO_HDR_UDP_LITE,              /* Lightweight User Datagram Protocol header */
	PASAHO_HDR_TCP,                   /* Transmission Control Protocol header */
	PASAHO_HDR_CUSTOM_C2              /* Custom classify 2 header */
};

/**
 *  @defgroup pasahoSubCmdCode PASS Sub-Command Code
 *  @ingroup pasaho_if_constants
 *  @{
 *
 *  @name PASS Sub-Command Code
 *  Definition of the 5-bit sub-command codes which is used to specify the group 7 commands. 
 */ 

enum pasaho_sub_cmd_code {
	PASAHO_SUB_CMD_DUMMY	= 0,	/* Dummy */
	PASAHO_SUB_CMD_IP_FRAG		/* IPv4 fragmentation */
};

/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoCmdInfo_t defines the general short command information
 *
 */
struct pasaho_cmd_info {
	u32	word0;		/* Control block word 0 */
};

/**
 *  @ingroup pasaho_if_structures
 *  @brief  pasahoLongInfo_t defines the packet parsing information in the long format. 
 *          The information is structured as an array of 32 bit values. These values
 *          are broken down through macros. This allows the representation to be
 *          endian independent to the hardware which operates only on 32 bit values.
 *
 *  @details  
 */
struct pasaho_long_info {
	u32   word0;	/* Control block word 0 */
	u32   word1;	/* Control block word 1 */
	u32   word2;	/* Control block word 2 */
	u32   word3;	/* Control block word 3 */
	u32   word4;	/* Control block word 4 */
};

/** 
 *  @defgroup PASAHO_long_info_command_macros  PASAHO Long Info Command Macros
 *  @ingroup pasaho_if_macros
 *  @{
 *  @name PASAHO Long Info Command Macros
 *  Macros used by the PASAHO Long Info Command
 */

/* Extract the command ID defined at */
#define PASAHO_LINFO_READ_CMDID(x)		PASAHO_READ_BITFIELD((x)->word0,29,3)

/* Extract the block length */
#define PASAHO_LINFO_READ_RECLEN(x)		PASAHO_READ_BITFIELD((x)->word0,24,5)

/* Extract the next parse start offset */
#define PASAHO_LINFO_READ_START_OFFSET(x)	PASAHO_READ_BITFIELD((x)->word0,0,16)

/* Extract the end of packet parse offset */
#define PASAHO_LINFO_READ_END_OFFSET(x)		PASAHO_READ_BITFIELD((x)->word1,16,16)

/* Extract the error index */
#define PASAHO_LINFO_READ_EIDX(x)		PASAHO_READ_BITFIELD((x)->word1,11,5)

/* Extract the previous match flag */
#define PASAHO_LINFO_READ_PMATCH(x)		PASAHO_READ_BITFIELD((x)->word1,10,1)

/* Extract the custom classify flag */
#define PASAHO_LINFO_READ_C2C(x)		PASAHO_READ_BITFIELD((x)->word1,9,1)

/* Extract the first parse module ID */
#define PASAHO_LINFO_READ_L1_PDSP_ID(x)		PASAHO_READ_BITFIELD((x)->word1,6,3)

/* Extract the first parse module match index */
#define PASAHO_LINFO_READ_L1_IDX(x)		PASAHO_READ_BITFIELD((x)->word1,0,6)

/* Extract the offset to the level 3 header */
#define PASAHO_LINFO_READ_L3_OFFSET(x)		PASAHO_READ_BITFIELD((x)->word2,24,8)

/* Extract the offset to the level 4 header */
#define PASAHO_LINFO_READ_L4_OFFSET(x)		PASAHO_READ_BITFIELD((x)->word2,16,8)

/* Extract the offset to the level 5 header */
#define PASAHO_LINFO_READ_L5_OFFSET(x)		PASAHO_READ_BITFIELD((x)->word2,8,8)

/* Extract the offset to the security header */
#define PASAHO_LINFO_READ_ESP_AH_OFFSET(x)	PASAHO_READ_BITFIELD((x)->word2,0,8)

/* Extract the bitmask of parsed header types */
#define PASAHO_LINFO_READ_HDR_BITMASK(x)	PASAHO_READ_BITFIELD((x)->word3,21,11)

/* Extract the next header to parse type */
#define PASAHO_LINFO_READ_NXT_HDR_TYPE(x)	PASAHO_READ_BITFIELD((x)->word3,16,5)

/* Extract the number of VLAN tags found */
#define PASAHO_LINFO_READ_VLAN_COUNT(x)		PASAHO_READ_BITFIELD((x)->word3,12,4)

/* Extract the number of IP headers found */
#define PASAHO_LINFO_READ_IP_COUNT(x)		PASAHO_READ_BITFIELD((x)->word3,8,4)

/* Extract the number of GRE headers found */
#define PASAHO_LINFO_READ_GRE_COUNT(x)		PASAHO_READ_BITFIELD((x)->word3,4,4)

/* Extract the fragmentation found flag */
#define PASAHO_LINFO_READ_FLAG_FRAG(x)		PASAHO_READ_BITFIELD((x)->word3,3,1)

/* Extract the incomplete IP route flag */
#define PASAHO_LINFO_READ_FLAG_ROUTE(x)		PASAHO_READ_BITFIELD((x)->word3,2,1)

/* Extract the (1-based) input EMAC port number */
/*  0: Indicates that the packet does not enter PASS through CPSW */
#define PASAHO_LINFO_READ_INPORT(x)		PASAHO_READ_BITFIELD((x)->word3,0,3)

/* Extract the last pseudo-header checksum computed */
#define PASAHO_LINFO_READ_PSEUDO_CHKSM(x)	PASAHO_READ_BITFIELD((x)->word4,16,16)

#define PASAHO_LINFO_READ_TSTAMP_MSB(x)		PASAHO_READ_BITFIELD((x)->word4,0,16)

/* Extract the IP Reassembly Traffic Flow Index */
#define PASAHO_LINFO_READ_TFINDEX(x)		PASAHO_READ_BITFIELD((x)->word4,24,8)

/* Extract the IP Reassembly Fragment count */
#define PASAHO_LINFO_READ_FRANCNT(x)		PASAHO_READ_BITFIELD((x)->word4,16,8)

/* Set the IP Reassembly Traffic Flow Index */
#define PASAHO_LINFO_SET_TFINDEX(x, v)		PASAHO_SET_BITFIELD((x)->word4,(v),24,8)

/* Set the IP Reassembly Fragment count */
#define PASAHO_LINFO_SET_FRANCNT(x, v)		PASAHO_SET_BITFIELD((x)->word4,(v),16,8)

/* Indicate whether it is an IPSEC packet */
#define PASAHO_LINFO_IS_IPSEC(x)		PASAHO_READ_BITFIELD((x)->word3,25,2)

/* Indicate whether it is an IPSEC ESP packet */
#define PASAHO_LINFO_IS_IPSEC_ESP(x)		PASAHO_READ_BITFIELD((x)->word3,26,1)

/* Indicate whether it is an IPSEC AH packet */
#define PASAHO_LINFO_IS_IPSEC_AH(x)		PASAHO_READ_BITFIELD((x)->word3,25,1)

/* Clear IPSEC indication bits */
#define PASAHO_LINFO_CLR_IPSEC(x)		PASAHO_SET_BITFIELD((x)->word3,0,25,2)

/* Clear IPSEC ESP indication bit */
#define PASAHO_LINFO_CLR_IPSEC_ESP(x)		PASAHO_SET_BITFIELD((x)->word3,0,26,1)

/* Clear IPSEC AH indication bit */
#define PASAHO_LINFO_CLR_IPSEC_AH(x)		PASAHO_SET_BITFIELD((x)->word3,0,25,1)

/* Clear the fragmentation found flag */
#define PASAHO_LINFO_CLR_FLAG_FRAG(x)		PASAHO_SET_BITFIELD((x)->word3,0,3,1)

/* Update the next parse start offset */
#define PASAHO_LINFO_SET_START_OFFSET(x, v)	PASAHO_SET_BITFIELD((x)->word0,(v),0,16)

/* Update the end of packet parse offset */
#define PASAHO_LINFO_SET_END_OFFSET(x, v)	PASAHO_SET_BITFIELD((x)->word1,(v),16,16)


/*
 * Set the null packet flag which indicates that the packet should be dropped.
 * This flag should be set for the null packet to be delivered to PASS when
 * the reassembly timeout occurs
 */
#define PASAHO_LINFO_SET_NULL_PKT_IND(x, v)	PASAHO_SET_BITFIELD((x)->word0,(v),21,1)

/*
 * PA_INV_TF_INDEX
 * PASS-asssited IP reassembly traffic flow index to indicate
 * that no traffic flow is available
 */
#define PA_INV_TF_INDEX		0xFF

struct pasaho_short_info {
	u32	word0;
	u32	word1;
};

/* Extract the command ID defined at */
#define PASAHO_SINFO_READ_CMDID(x)		PASAHO_READ_BITFIELD((x)->word0,29,3)

/* Extract the offset to the packet payload */
#define PASAHO_SINFO_RESD_PAYLOAD_OFFSET(x)	PASAHO_READ_BITFIELD((x)->word0,16,8)

/* Extract the byte length of the payload */
#define PASAHO_SINFO_READ_PAYLOAD_LENGTH(x)	PASAHO_READ_BITFIELD((x)->word0,0,16)

/* Set the offset to the payload */
#define PASAHO_SINFO_SET_PAYLOAD_OFFSET(x, v)	PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Set the payload length */
#define PASAHO_SINFO_SET_PAYLOAD_LENGTH(x, v)	PASAHO_SET_BITFIELD((x)->word0, (v), 0,  16)

/* Format the entire short info command */
#define PASAHO_SINFO_FORMAT_CMD(offset, len)	(((offset) << 16) | (len) | (PASAHO_SA_SHORT_INFO << 29))

#define PASAHO_HDR_BITMASK_MAC		(1 << 0)	/* MAC present */
#define PASAHO_HDR_BITMASK_VLAN		(1 << 1)	/* VLAN present */
#define PASAHO_HDR_BITMASK_MPLS		(1 << 2)	/* MPLS present */
#define PASAHO_HDR_BITMASK_IP		(1 << 3)	/* IP present */
#define PASAHO_HDR_BITMASK_ESP		(1 << 4)	/* IPSEC/ESP present */
#define PASAHO_HDR_BITMASK_AH		(1 << 5)	/* IPSEC/AH present */
#define PASAHO_HDR_BITMASK_UDP		(1 << 6)	/* UDP present */
#define PASAHO_HDR_BITMASK_UDPLITE	(1 << 7)	/* UDPLITE present */
#define PASAHO_HDR_BITMASK_TCP		(1 << 8)	/* TCP present */
#define PASAHO_HDR_BITMASK_GRE		(1 << 9)	/* GRE present */
#define PASAHO_HDR_BITMASK_CUSTOM	(1 << 10)	/* Custom header */

struct pasaho_next_route {
	u32  word0;          
	u32  sw_info0;        
	u32  sw_info1;        
	u32  word1;          
};

/*
 * Sets the N bit which indicates the next command
 * should be executed prior to the route command
 */
#define PASAHO_SET_N(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 28, 1)

/*
 * Sets the E bit which indicates the extened
 * parameters (packet type) are present for SRIO
 */
#define PASAHO_SET_E(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 27, 1)

/*
 * Sets the destination of the route defined */
#define PASAHO_SET_DEST(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 24, 3)

/* Specifies the flow to use for packets sent to the host */
#define PASAHO_SET_FLOW(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Specifies the queue to use for packets send to the host */
#define PASAHO_SET_QUEUE(x,v)   PASAHO_SET_BITFIELD((x)->word0, (v), 0,  16)

/* Specifies the packet type to use for packets send to the SRIO */
#define PASAHO_SET_PKTTYPE(x,v) PASAHO_SET_BITFIELD((x)->word1, (v), 24, 8)

struct pasaho_com_chk_crc {
	u32	word0;		/* PASAHO_chksum_command_macros */
	u32	word1;		/* PASAHO_chksum_command_macros */
	u32	word2;		/* PASAHO_chksum_command_macros */
};

/*
 * Sets the negative 0 flag - if set a
 * checksum computed as 0 will be sent as 0xffff
 */
#define PASAHO_CHKCRC_SET_NEG0(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 23, 1)

/* Sets the optional flags of the CRC/Checksum command */
#define PASAHO_CHKCRC_SET_CTRL(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Sets the start offset of the checksum/crc */
#define PASAHO_CHKCRC_SET_START(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 0,  16)

/* Sets the length of the checksum/crc */
#define PASAHO_CHKCRC_SET_LEN(x,v)	PASAHO_SET_BITFIELD((x)->word1, (v), 16, 16)

/* Sets the offset to where to paste the checksum/crc into the packet */
#define PASAHO_CHKCRC_SET_RESULT_OFF(x,v)	PASAHO_SET_BITFIELD((x)->word1, (v), 0,  16)

/* Sets the initial value of the checksum/crc */
#define PASAHO_CHKCRC_SET_INITVAL(x,v)	PASAHO_SET_BITFIELD((x)->word2, (v), 16, 16)

#define PASAHO_BPATCH_MAX_PATCH_WORDS	4

struct pasaho_com_blind_patch {
	u32	word0;
	u32	patch[PASAHO_BPATCH_MAX_PATCH_WORDS];
};


#define PASAHO_BPATCH_SET_PATCH_NBYTES(x,v)	PASAHO_SET_BITFIELD((x)->word0, v, 24,  5)

/* Sets the number of bytes to patch */
#define PASAHO_BPATCH_SET_PATCH_CMDSIZE(x,v)	PASAHO_SET_BITFIELD((x)->word0, v, 20, 4)

/* Sets the size of the command in 32 bit word units */                        
#define PASAHO_BPATCH_SET_OVERWRITE(x,v)	PASAHO_SET_BITFIELD((x)->word0, v, 19, 1)

/*
 * Sets the overwrite flag. If set the patch will
 * overwrite existing packet data, otherwise data is inserted
 */                         
#define PASAHO_BPATCH_SET_OFFSET(x,v)		PASAHO_SET_BITFIELD((x)->word0, v, 0,  16)

/* Sets the offset to the start of the patch */
#define PASAHO_BPATCH_SET_PATCH_BYTE(x, byteNum, byte)  (x)->patch[(byteNum) >> 2] = \
		PASAHO_SET_BITFIELD((x)->patch[(byteNum) >> 2], byte, ((3 - (byteNum & 0x3)) << 3), 8)


struct pasaho_report_timestamp {
	u32	word0;
	u32	sw_info0;
};

/* Specifies the flow to use for report packets sent to the host */

#define PASAHO_SET_REPORT_FLOW(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Specifies the queue to use for report packets send to the host */
#define PASAHO_SET_REPORT_QUEUE(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 0,  16)

struct pasaho_ip_frag {
	u32	word0;
};

/* Set sub-command code to indicate IP Fragmentation command */
#define PASAHO_SET_SUB_CODE_IP_FRAG(x) PASAHO_SET_BITFIELD((x)->word0, PASAHO_SUB_CMD_IP_FRAG, 24, 5)

/* Specifies the sub-command code */
#define PASAHO_SET_SUB_CODE(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 24, 5)

/* Specifies the offset to the IP header to be fragmented */
#define PASAHO_SET_IP_OFFSET(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 16, 8)

/* Specifies the MTU size */
#define PASAHO_SET_MTU_SIZE(x,v)	PASAHO_SET_BITFIELD((x)->word0, (v), 0,  16)

#endif /* __KERNEL__ */
#endif /* KEYSTONE_PASAHOST_H */
