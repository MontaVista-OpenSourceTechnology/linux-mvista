/*
 * Copyright (C) 2010, 2011, 2012, 2013, 2014 Texas Instruments Incorporated
 * Authors: Aurelien Jacquiot <a-jacquiot@ti.com>
 * - Main driver implementation.
 * - Updated for support on TI KeyStone 2 platform.
 *
 * Copyright (C) 2012, 2013 Texas Instruments Incorporated
 * WingMan Kwok <w-kwok2@ti.com>
 * - Updated for support on TI KeyStone 1 platform.
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
#ifndef KEYSTONE_RIO_H
#define KEYSTONE_RIO_H

#include <asm/setup.h>
#include <linux/cache.h>
#include <linux/uaccess.h>
#include <asm/irq.h>
#include <linux/io.h>


/* Some NSN/keystone specific redefinitions for Macros of rio_regs.h. rio_regs.h must be included before keystone.h */
#undef   RIO_PORT_N_MNT_RSP_RVAL
#undef   RIO_PORT_N_MNT_RSP_ASTAT
#undef   RIO_PORT_N_MNT_RSP_LSTAT
#define  RIO_PORT_N_MNT_RSP_RVAL(x)	((x) & 0x80000000) /* Response Valid */
#define  RIO_PORT_N_MNT_RSP_ASTAT(x)	(((x) & 0x000007e0) >> 5) /* ackID Status */
#define  RIO_PORT_N_MNT_RSP_LSTAT(x)	((x) & 0x0000001f) /* Link Status */


#define KEYSTONE_RIO_MAP_FLAG_SEGMENT		BIT(0)
#define KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC	BIT(1)
#define KEYSTONE_RIO_MAP_FLAG_TT_16		BIT(13)
#define KEYSTONE_RIO_MAP_FLAG_DST_PROMISC	BIT(15)
#define KEYSTONE_RIO_DESC_FLAG_TT_16		BIT(9)

#define KEYSTONE_RIO_BOOT_COMPLETE		BIT(24)
#define KEYSTONE_RIO_PER_RESTORE		BIT(4)
#define KEYSTONE_RIO_PER_EN			BIT(2)
#define KEYSTONE_RIO_PER_FREE			BIT(0)
#define KEYSTONE_RIO_PEF_FLOW_CONTROL		BIT(7)

/*
 * Packet types
 */
#define KEYSTONE_RIO_PACKET_TYPE_NREAD    0x24
#define KEYSTONE_RIO_PACKET_TYPE_NWRITE   0x54
#define KEYSTONE_RIO_PACKET_TYPE_NWRITE_R 0x55
#define KEYSTONE_RIO_PACKET_TYPE_SWRITE   0x60
#define KEYSTONE_RIO_PACKET_TYPE_DBELL    0xa0
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_R  0x80
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_W  0x81
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_RR 0x82
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_WR 0x83
#define KEYSTONE_RIO_PACKET_TYPE_MAINT_PW 0x84

/*
 * LSU defines
 */
#define KEYSTONE_RIO_LSU_PRIO             0

#define KEYSTONE_RIO_LSU_BUSY_MASK        BIT(31)
#define KEYSTONE_RIO_LSU_FULL_MASK        BIT(30)

#define KEYSTONE_RIO_LSU_CC_MASK          0x0f
#define KEYSTONE_RIO_LSU_CC_TIMEOUT       0x01
#define KEYSTONE_RIO_LSU_CC_XOFF          0x02
#define KEYSTONE_RIO_LSU_CC_ERROR         0x03
#define KEYSTONE_RIO_LSU_CC_INVALID       0x04
#define KEYSTONE_RIO_LSU_CC_DMA           0x05
#define KEYSTONE_RIO_LSU_CC_RETRY         0x06
#define KEYSTONE_RIO_LSU_CC_CANCELED      0x07

/* Mask for receiving both error and good completion LSU interrupts */
#define KEYSTONE_RIO_ICSR_LSU0(src_id)    ((0x10001) << (src_id))

/* Keystone2 supported baud rates */
#define KEYSTONE_RIO_BAUD_1_250		0
#define KEYSTONE_RIO_BAUD_2_500		1
#define KEYSTONE_RIO_BAUD_3_125		2
#define KEYSTONE_RIO_BAUD_5_000		3

#define KEYSTONE_RIO_FULL_RATE		0
#define KEYSTONE_RIO_HALF_RATE		1
#define KEYSTONE_RIO_QUARTER_RATE	2

/* Max ports configuration per path modes */
#define KEYSTONE_MAX_PORTS_PATH_MODE_0  0xf /* 4 ports */
#define KEYSTONE_MAX_PORTS_PATH_MODE_1  0xd /* 3 ports */
#define KEYSTONE_MAX_PORTS_PATH_MODE_2  0x7 /* 3 ports */
#define KEYSTONE_MAX_PORTS_PATH_MODE_3  0x5 /* 2 ports */
#define KEYSTONE_MAX_PORTS_PATH_MODE_4  0x1 /* 1 ports */

#define SERDES_LANE(lane_num)					BIT(lane_num)
#define IS_SERDES_LANE_USED(lanes, lane_num)	(lanes & SERDES_LANE(lane_num))
#define KEYSTONE_MAX_SERDES_LINES	4

/*
 * Various RIO defines
 */
#define KEYSTONE_RIO_DBELL_NUMBER         4
#define KEYSTONE_RIO_DBELL_VALUE_MAX      (KEYSTONE_RIO_DBELL_NUMBER * 16)
#define KEYSTONE_RIO_DBELL_MASK           (KEYSTONE_RIO_DBELL_VALUE_MAX - 1)

#define KEYSTONE_RIO_TIMEOUT_CNT	  1000
#define KEYSTONE_RIO_TIMEOUT_MSEC         100
#define KEYSTONE_RIO_TIMEOUT_NSEC         1000
#define KEYSTONE_RIO_RETRY_CNT            4
#define KEYSTONE_RIO_REGISTER_DELAY	  (3*HZ)

#define KEYSTONE_RIO_GM_SCHED          200 /* ms */
#define KEYSTONE_RIO_QM_DESC_MASK      0x00003FFF
#define KEYSTONE_QMSS_PUPO_BASE_ADDR   (0x02a80000)
#define KEYSTONE_QMSS_PEEK_BASE_ADDR   (0x02a40000)
#define KEYSTONE_QMSS_QUEUE_FETCH(q)   (KEYSTONE_QMSS_PUPO_BASE_ADDR + 0x10*(q&(KEYSTONE_RIO_QM_DESC_MASK)) + 0xC)
#define KEYSTONE_QMSS_QUEUE_PEEK(q)    (KEYSTONE_QMSS_PEEK_BASE_ADDR + 0x10*(q&(KEYSTONE_RIO_QM_DESC_MASK)))

/* Descriptor memory setup region */
#define KEYSTONE_QMSS_MEM_REGION_BASE  (0x02a03000)
#define KEYSTONE_QMSS_MEM_REGION_SIZE  (0x400)
#define KEYSTONE_QMSS_MAX_REGIONS      (64)

#define KEYSTONE_RIO_DDR3A_LINUX_REGION (0xC0000000)

#define K2_PLL_LOCK_TIMEOUT	          100 /* 100ms timeout */

/*
 * RIO error, reset and special event interrupt defines
 */
#define KEYSTONE_RIO_PORT_ERROR_OUT_PKT_DROP	BIT(26)
#define KEYSTONE_RIO_PORT_ERROR_OUT_FAILED	BIT(25)
#define KEYSTONE_RIO_PORT_ERROR_OUT_DEGRADED	BIT(24)
#define KEYSTONE_RIO_PORT_ERROR_OUT_RETRY	BIT(20)
#define KEYSTONE_RIO_PORT_ERROR_OUT_ERROR	BIT(17)
#define KEYSTONE_RIO_PORT_ERROR_IN_ERROR	BIT(9)
#define KEYSTONE_RIO_PORT_ERROR_PW_PENDING	BIT(4)
#define KEYSTONE_RIO_PORT_ERROR_PORT_ERR	BIT(2)

#define KEYSTONE_RIO_PORT_ERROR_RX_RETRY_CS	BIT(19)
#define KEYSTONE_RIO_PORT_ERROR_RX_RT_OUT_RT	BIT(18)
#define KEYSTONE_RIO_PORT_ERROR_OUT_ERR_STOP	BIT(16)
#define KEYSTONE_RIO_PORT_ERROR_IN_RETRY_STOP	BIT(10)
#define KEYSTONE_RIO_PORT_ERROR_IN_ERR_STOP	BIT(8)
#define KEYSTONE_RIO_PORT_ERROR_PORT_OK		BIT(1)
#define KEYSTONE_RIO_PORT_ERROR_INITED		BIT(0)

#define KEYSTONE_RIO_PORT_ERROR_STATUS		 \
	(KEYSTONE_RIO_PORT_ERROR_OUT_PKT_DROP	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_FAILED	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_DEGRADED	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_RETRY	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_ERROR	|\
	KEYSTONE_RIO_PORT_ERROR_IN_ERROR	|\
	KEYSTONE_RIO_PORT_ERROR_PW_PENDING	|\
	KEYSTONE_RIO_PORT_ERROR_PORT_ERR	|\
	KEYSTONE_RIO_PORT_ERROR_RX_RETRY_CS	|\
	KEYSTONE_RIO_PORT_ERROR_RX_RT_OUT_RT	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_ERR_STOP	|\
	KEYSTONE_RIO_PORT_ERROR_IN_RETRY_STOP	|\
	KEYSTONE_RIO_PORT_ERROR_IN_ERR_STOP	|\
	KEYSTONE_RIO_PORT_ERROR_PORT_OK		|\
	KEYSTONE_RIO_PORT_ERROR_INITED)

/*
 * KeystoneII Local Error Detect Fields Descriptions
 */
#define KEYSTONE_RIO_LLERR_DET_LOCAL_ILL_ID	BIT(26)
#define KEYSTONE_RIO_LLERR_DET_LOCAL_ILL_TYPE	BIT(22)

#define KEYSTONE_RIO_PORT_ERROR_MASK		 \
	(KEYSTONE_RIO_PORT_ERROR_OUT_PKT_DROP	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_FAILED	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_DEGRADED	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_RETRY	|\
	KEYSTONE_RIO_PORT_ERROR_OUT_ERROR	|\
	KEYSTONE_RIO_PORT_ERROR_IN_ERROR	|\
	KEYSTONE_RIO_PORT_ERROR_PW_PENDING	|\
	KEYSTONE_RIO_PORT_ERROR_PORT_ERR)

#define KEYSTONE_RIO_PORT_ERRORS	 \
	(RIO_PORT_N_ERR_STS_OUT_ES	|\
	 RIO_PORT_N_ERR_STS_INP_ES	|\
	 RIO_PORT_N_ERR_STS_PW_PEND	|\
	 RIO_PORT_N_ERR_STS_PORT_ERR)

/* RIO PLM port event status defines */
#define KEYSTONE_RIO_PORT_PLM_STATUS_MAX_DENIAL		BIT(31)
#define KEYSTONE_RIO_PORT_PLM_STATUS_LINK_INIT		BIT(28)
#define KEYSTONE_RIO_PORT_PLM_STATUS_DLT		BIT(27)
#define KEYSTONE_RIO_PORT_PLM_STATUS_PORT_ERR		BIT(26)
#define KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_FAIL	BIT(25)
#define KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_DEGR	BIT(24)
#define KEYSTONE_RIO_PORT_PLM_STATUS_RST_REQ		BIT(16)
#define KEYSTONE_RIO_PORT_PLM_STATUS_PBM_PW		BIT(15)
#define KEYSTONE_RIO_PORT_PLM_STATUS_TLM_PW		BIT(14)
#define KEYSTONE_RIO_PORT_PLM_STATUS_MECS		BIT(12)
#define KEYSTONE_RIO_PORT_PLM_STATUS_PBM_INT		BIT(11)
#define KEYSTONE_RIO_PORT_PLM_STATUS_TLM_INT		BIT(10)

#define KEYSTONE_RIO_PORT_PLM_STATUS_ERRORS		 \
	(KEYSTONE_RIO_PORT_PLM_STATUS_MAX_DENIAL	|\
	 KEYSTONE_RIO_PORT_PLM_STATUS_PORT_ERR		|\
	 KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_FAIL	|\
	 KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_DEGR)

#define KEYSTONE_RIO_PORT_PLM_PORT_STATUS		 \
	 (KEYSTONE_RIO_PORT_PLM_STATUS_MAX_DENIAL       |\
	 KEYSTONE_RIO_PORT_PLM_STATUS_LINK_INIT         |\
	 KEYSTONE_RIO_PORT_PLM_STATUS_DLT               |\
	 KEYSTONE_RIO_PORT_PLM_STATUS_PORT_ERR          |\
	 KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_FAIL       |\
	 KEYSTONE_RIO_PORT_PLM_STATUS_OUTPUT_DEGR       |\
	 KEYSTONE_RIO_PORT_PLM_STATUS_RST_REQ           |\
	 KEYSTONE_RIO_PORT_PLM_STATUS_MECS)

#define KEYSTONE_RIO_SP_HDR_NEXT_BLK_PTR	0x1000
#define KEYSTONE_RIO_SP_HDR_EP_REC_ID		0x0002
#define KEYSTONE_RIO_ERR_HDR_NEXT_BLK_PTR	0x3000
#define KEYSTONE_RIO_ERR_EXT_FEAT_ID		0x0007

/* RIO TLM port event status defines */
#define KEYSTONE_RIO_PORT_TLM_STATUS_IG_BAD_VC		BIT(31)
#define KEYSTONE_RIO_PORT_TLM_STATUS_IG_BRR_FILTER	BIT(20)

#define KEYSTONE_RIO_PORT_TLM_STATUS		 \
	(KEYSTONE_RIO_PORT_TLM_STATUS_IG_BAD_VC	|\
	KEYSTONE_RIO_PORT_TLM_STATUS_IG_BRR_FILTER)

/* RIO EM block event status defines */
#define KEYSTONE_RIO_BLOCK_EM_STATUS_PORT	BIT(29)
#define KEYSTONE_RIO_BLOCK_EM_STATUS_LOG	BIT(28)
#define KEYSTONE_RIO_BLOCK_EM_STATUS_RCS	BIT(27)
#define KEYSTONE_RIO_BLOCK_EM_STATUS_MECS	BIT(26)
#define KEYSTONE_RIO_BLOCK_EM_STATUS_PW_RX	BIT(16)
#define KEYSTONE_RIO_BLOCK_EM_STATUS_LOCALLOG	BIT(8)

#define KEYSTONE_RIO_BLOCK_EM_STATUS			 \
	(KEYSTONE_RIO_BLOCK_EM_STATUS_PORT		|\
	KEYSTONE_RIO_BLOCK_EM_STATUS_LOG		|\
	KEYSTONE_RIO_BLOCK_EM_STATUS_RCS		|\
	KEYSTONE_RIO_BLOCK_EM_STATUS_MECS		|\
	KEYSTONE_RIO_BLOCK_EM_STATUS_PW_RX		|\
	KEYSTONE_RIO_BLOCK_EM_STATUS_LOCALLOG)

#define KEYSTONE_RIO_BLOCK_EM_INT_ENABLE		 \
	(KEYSTONE_RIO_BLOCK_EM_STATUS_LOCALLOG		|\
	KEYSTONE_RIO_BLOCK_EM_STATUS_PW_RX		|\
	KEYSTONE_RIO_BLOCK_EM_STATUS_MECS		|\
	KEYSTONE_RIO_BLOCK_EM_STATUS_LOG)
#define KEYSTONE_RIO_BLOCK_EM_DEV_INT_ENABLE	BIT(0)

/* RIO PBM port event status defines */
#define KEYSTONE_RIO_PORT_PBM_STATUS_IG_EMPTY		BIT(16)
#define KEYSTONE_RIO_PORT_PBM_STATUS_EG_EMPTY		BIT(15)
#define KEYSTONE_RIO_PORT_PBM_STATUS_EG_DATA_OVERFLOW	BIT(4)
#define KEYSTONE_RIO_PORT_PBM_STATUS_EG_CRQ_OVERFLOW	BIT(3)
#define KEYSTONE_RIO_PORT_PBM_STATUS_EG_BAD_CHANNEL	BIT(1)
#define KEYSTONE_RIO_PORT_PBM_STATUS_EG_BABBLE_PACKET	BIT(0)

#define KEYSTONE_RIO_PORT_PBM_STATUS			 \
	(KEYSTONE_RIO_PORT_PBM_STATUS_IG_EMPTY		|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_EMPTY		|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_DATA_OVERFLOW	|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_CRQ_OVERFLOW	|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_BAD_CHANNEL	|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_BABBLE_PACKET)

#define KEYSTONE_RIO_PORT_PBM_STATUS_CHECK		 \
	(KEYSTONE_RIO_PORT_PBM_STATUS_EG_DATA_OVERFLOW	|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_CRQ_OVERFLOW	|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_BAD_CHANNEL	|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_BABBLE_PACKET)

#define KEYSTONE_RIO_PORT_PBM_INT_STATUS		 \
	(KEYSTONE_RIO_PORT_PBM_STATUS_EG_DATA_OVERFLOW	|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_CRQ_OVERFLOW	|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_BAD_CHANNEL	|\
	 KEYSTONE_RIO_PORT_PBM_STATUS_EG_BABBLE_PACKET)

/*
 * RapidIO global definitions
 */
#define KEYSTONE_RIO_MAX_PORT		4
enum keystone_rio_blk {
	KEYSTONE_RIO_BLK0_MMRS	= 0,
	KEYSTONE_RIO_BLK1_LSU	= 1,
	KEYSTONE_RIO_BLK2_MAU	= 2,
	KEYSTONE_RIO_BLK3_TXU	= 3,
	KEYSTONE_RIO_BLK4_RXU	= 4,
	KEYSTONE_RIO_BLK5_PORT0	= 5,
	KEYSTONE_RIO_BLK6_PORT1	= 6,
	KEYSTONE_RIO_BLK7_PORT2	= 7,
	KEYSTONE_RIO_BLK8_PORT3	= 8,
	KEYSTONE_RIO_BLK9		= 9,
	KEYSTONE_RIO_BLK_NUM	= 10
};
#define KEYSTONE_RIO_RXU_MAP_MIN	0	/* Max RXU_MAP range */
#define KEYSTONE_RIO_RXU_MAP_MAX	63
#define KEYSTONE_RIO_MAX_MBOX		4	/* 4 in multi-segment,
						   64 in single-seg */
#define KEYSTONE_RIO_MAX_LETTER		4	/* 4 letters per mailbox */
#define KEYSTONE_RIO_MAX_GARBAGE_QUEUES 6
#define KEYSTONE_RIO_MAX_PKT_FW_ENTRIES 8	/* max of packet forwarding
						   mapping entries */
#define KEYSTONE_RIO_MAINT_BUF_SIZE	64
#define KEYSTONE_RIO_MSG_SSIZE		0xe
#define KEYSTONE_RIO_SGLIST_SIZE	3

#define KEYSTONE_RIO_PKT_FW_BRR_NUM     1    /* BRR used for packet forwarding */

/*
 * Dev Id and dev revision
 */
#define KEYSTONE_RIO_DEV_ID_VAL	\
	((((__raw_readl(krio_priv->jtagid_reg)) << 4)  & 0xffff0000) | 0x30)

#define KEYSTONE_RIO_DEV_INFO_VAL \
	(((__raw_readl(krio_priv->jtagid_reg)) >> 28) & 0xf)

#define KEYSTONE_RIO_ID_TI		(0x00000030)
#define KEYSTONE_RIO_EXT_FEAT_PTR	(0x00000100)


#define KEYSTONE_RIO_RETRY_CNT          4
#define KEYSTONE_RIO_MAX_DIO_PKT_SIZE   0x100000 /* hardware support up to 1MB */

/*
 * RIO error, reset and special event interrupt defines
  */
#define KEYSTONE_RIO_ERR_RST_EVNT_MASK  0x00010f07

/* Refer to bits in KEYSTONE_RIO_ERR_RST_EVNT_MASK */
#define KEYSTONE_RIO_RESET_INT          16  /* device reset interrupt on any port */
#define KEYSTONE_RIO_PORT3_ERROR_INT    11  /* port 3 error */
#define KEYSTONE_RIO_PORT2_ERROR_INT    10  /* port 2 error */
#define KEYSTONE_RIO_PORT1_ERROR_INT    9   /* port 1 error */
#define KEYSTONE_RIO_PORT0_ERROR_INT    8   /* port 0 error */
#define KEYSTONE_RIO_EVT_CAP_ERROR_INT  2   /* logical layer error management event capture */
#define KEYSTONE_RIO_PORT_WRITEIN_INT   1   /* port-write-in request received on any port */
#define KEYSTONE_RIO_MCAST_EVT_INT      0   /* multi-cast event control symbol interrupt received on any port */

/*
 * Interrupt and DMA event mapping
 * MP RXU and TXU interrupts are provided in platform_data
 */
#define KEYSTONE_GEN_RIO_INT            0   /* RIO interrupt used for generic RIO events */
#define KEYSTONE_LSU_RIO_INT            1   /* RIO interrupt used for LSU */

/*
 * RapidIO_INT_CDMA_0: PKTDMA starvation interrupt line
 */
#define KEYSTONE_RIO_THRESH_STARVATION	4

/* Mask for receiving both error and good completion LSU interrupts */
#define KEYSTONE_RIO_ICSR_LSU0(src_id)  ((0x10001) << (src_id))

/*
 * Definition of the different RapidIO packet types according to the RapidIO
 * specification 2.0
 */
#define RIO_PACKET_TYPE_STREAM          9  /* Data Streaming */
#define RIO_PACKET_TYPE_MESSAGE         11 /* Message */

/*
 * SerDes configurations
 */
struct keystone_serdes_config {
	u32 cfg_cntl;                              /* setting control reg cfg */
	u16 serdes_cfg_pll;                        /* SerDes PLL cfg */
	u16 prescalar_srv_clk;                     /* prescalar fo ip_clk */
	u32 rx_chan_config[KEYSTONE_RIO_MAX_PORT]; /* SerDes rx channel cfg
						      (per-port) */
	u32 tx_chan_config[KEYSTONE_RIO_MAX_PORT]; /* SerDes tx channel cfg
						      (per-port) */
};

/*
 * Routing configuration for packet forwarding
 */
struct keystone_routing_config {
	u16 dev_id_low;
	u16 dev_id_high;
	u8  port;
};

/*
 * Per board RIO devices controller configuration
 */
struct keystone_rio_board_controller_info {
	u32		ports;	/* bitfield of port(s) to probe on this controller */
	u32		lanes;	/* Used SerDes lanes */
	u32		ports_remote[KEYSTONE_RIO_MAX_PORT];
	u32		id;		/* host id */
	u32		size;	/* RapidIO common transport system size.
					 * 0 - Small size. 256 devices.
					 * 1 - Large size, 65536 devices. */
	u16		keystone2_serdes;
	u32		serdes_baudrate;
	u32		path_mode;
	u32		port_register_timeout;
	u32		pkt_forwarding;

	int		rio_irq;
	int		lsu_irq;
	int		rio_starvation_irq;

	u32		comp_tag;

	u32		tx_garbage_queues[6];
	u32		free_fdq;

	u8		host;

	struct keystone_serdes_config serdes_config;
	struct keystone_routing_config routing_config[8];
};

struct keystone_rio_data;

struct keystone_rio_packet {
	struct scatterlist		sg[KEYSTONE_RIO_SGLIST_SIZE];
	int				sg_ents;
	u32				epib[4];
	u32				psdata[2];
	u32				mbox;
	u32				slot;
	void				*buff;
	void				*buff_cookie;
	struct keystone_rio_data	*priv;
	enum dma_status			status;
	dma_cookie_t			cookie;
};

struct keystone_rio_tx_chan_info {
	struct rio_mport *port;
	void		 *dev_id;
	struct dma_chan	 *tx_channel;
	u32		 running;
	u32		 entries;
	u32		 slot;
	const char	 *name;
	u32		 queue_depth;
};

struct keystone_rio_rx_chan_info {
	struct keystone_rio_data *priv;
	struct rio_mport	 *port;
	void			 *dev_id;
	struct dma_chan		 *dma_channel;
	u32			 running;
	u32			 entries;
	int			 rxu_map_id[2];
	const char		 *name;
	u32			 queue_depths[KEYSTONE_QUEUES_PER_CHAN];
	u32			 buffer_sizes[KEYSTONE_QUEUES_PER_CHAN];
	int			 queue_num;	/* rx complete queue */
	int			 flow_num;
	u32			 packet_type;
	u32			 starv_count;
	struct mutex		 dma_lock;
	union {
		int		 letter;	/* letter for type 11 packet */
		int		 stream_id;	/* stream id for type 9 packet */
	};
	void (*release_cb)(void *cookie);
#ifdef KEYSTONE_RIO_RX_OUTOFORDER_WA
	struct {
		struct keystone_rio_packet	*packet;
		u8				status;
	} *packets;
	int				packet_insert_ix;
	int				packet_rx_ix;
#endif
};

struct port_write_msg {
	union rio_pw_msg msg;
	u32              msg_count;
	u32              err_count;
	u32              discard_count;
};

/*
 * RapidIO Registers
 */

struct keystone_srio_serdes_regs {
	u32	pll;

	struct {
		u32	rx;
		u32	tx;
	} channel[4];
};

/* Keystone2 SerDes registers 0000 - 1fff */
struct keystone2_srio_serdes_regs {
	u32 __cmu0_rsvd0[2];		/* 0000 - 0004 */
	u32	cmu_008;				/* 0008 */
	u32 __cmu0_rsvd1[56];		/* 000c - 00e8 */
	u32 cmu_0EC;				/* 00ec */
	u32 __cmu0_rsvd2[3];		/* 00f0 - 00f8 */
	u32 cmu_0FC;				/* 00fc */
	u32 __cmu0_rsvd3[64];		/* 0100 - 01fc */

	struct {					/* 0200 * lane_no */
		u32 lane_000;			/* 0000 */
		u32 lane_004;			/* 0004 */
		u32 lane_008;			/* 0008 */
		u32 __lane_rsvd0[9];	/* 000c - 002c */
		u32 lane_030;			/* 0030 */
		u32 lane_034;			/* 0034 */
		u32 lane_038;			/* 0038 */
		u32 lane_03C;			/* 003c */
		u32 lane_040;			/* 0040 */
		u32 lane_044;			/* 0044 */
		u32 lane_048;			/* 0048 */
		u32 __lane_rsvd1[11];	/* 004c - 0074 */
		u32 lane_078;			/* 0078 */
		u32 __lane_rsvd2[2];	/* 007c - 0080 */
		u32 lane_084;			/* 0084 */
		u32 __lane_rsvd3;		/* 0088 */
		u32 lane_08C;			/* 008c */
		u32 __lane_rsvd4[4];	/* 0090 - 009c */
		u32 lane_0A0;			/* 00a0 */
		u32 __lane_rsvd5;		/* 00a4 */
		u32 lane_0A8;			/* 00a8 */
		u32 __lane_rsvd6[85];	/* 00ac - 01fc */
	} lane[4];

	u32 comlane_000;			/* 0a00 */
	u32 __comlane_rsvd0[4];		/* 0a04 - 0a10 */
	u32 comlane_014;			/* 0a14 */
	u32 __comlane_rsvd1[27];	/* 0a18 - 0a80 */
	u32 comlane_084;			/* 0a84 */
	u32 __comlane_rsvd2;		/* 0a88 */
	u32 comlane_08C;			/* 0a8c */
	u32 comlane_090;			/* 0a90 */
	u32 __comlane_rsvd3[89];	/* 0a94 - 0bf4 */
	u32 comlane_1F8;			/* 0bf8 */
	u32 __comlane_rsvd4[1265];	/* 0bfc - 1fbc */

	u32 __wiz_rsvd0[8];			/* 1fc0 - 1fdc */
	struct {
		u32 ctl_sts;			/* 1fe0 - 1fec */
	} wiz_lane[4];
	u32 __wiz_rsvd1;			/* 1ff0 */
	u32 wiz_pll_ctrl;			/* 1ff4 */
	u32 __wiz_rsvd2[2];			/* 1ff8 - 1ffc */
};

/* RIO Registers  0000 - 2fff */
struct keystone_rio_regs {
/* Required Peripheral Registers */
	u32	pid;			/* 0000 */
	u32	pcr;			/* 0004 */
	u32	__rsvd0[3];		/* 0008 - 0010 */

/* Peripheral Settting Control Registers */
	u32	per_set_cntl;		/* 0014 */
	u32	per_set_cntl1;		/* 0018 */

	u32	__rsvd1[2];		/* 001c - 0020 */

	u32	gbl_en;			/* 0024 */
	u32	gbl_en_stat;		/* 0028 */

	struct {
		u32 enable;		/* 002c */
		u32 status;		/* 0030 */
	} blk[10];			/* 002c - 0078 */

	/* ID Registers */
	u32	__rsvd2[17];		/* 007c - 00bc */
	u32	multiid_reg[8];		/* 00c0 - 00dc */

/* Hardware Packet Forwarding Registers */
	struct {
		u32	pf_16b;
		u32	pf_8b;
	} pkt_fwd_cntl[8];		/* 00e0 - 011c */

	u32	__rsvd3[24];		/* 0120 - 017c */

/* Interrupt Registers */
	struct {
		u32	status;
		u32	__rsvd0;
		u32	clear;
		u32	__rsvd1;
	} doorbell_int[4];		/* 0180 - 01bc */

	struct {
		u32	status;
		u32	__rsvd0;
		u32	clear;
		u32	__rsvd1;
	} lsu_int[2];			/* 01c0 - 01dc */

	u32	err_rst_evnt_int_stat;	/* 01e0 */
	u32	__rsvd4;
	u32	err_rst_evnt_int_clear;	/* 01e8 */
	u32	__rsvd5;

	u32	__rsvd6[4];		/* 01f0 - 01fc */

	struct {
		u32 route;		/* 0200 */
		u32 route2;		/* 0204 */
		u32 __rsvd;		/* 0208 */
	} doorbell_int_route[4];	/* 0200 - 022c */

	u32	lsu0_int_route[4];		/* 0230 - 023c */
	u32	lsu1_int_route1;		/* 0240 */

	u32	__rsvd7[3];		/* 0244 - 024c */

	u32	err_rst_evnt_int_route[3];	/* 0250 - 0258 */

	u32	__rsvd8[2];		/* 025c - 0260 */

	u32	interrupt_ctl;		/* 0264 */

	u32	__rsvd9[26];		/* 0268, 026c, 0270 - 02cc */

	u32	intdst_rate_cntl[16];	/* 02d0 - 030c */
	u32	intdst_rate_disable;	/* 0310 */

	u32	__rsvd10[59];		/* 0314 - 03fc */

/* RXU Registers */
	struct {
		u32	ltr_mbox_src;
		u32	dest_prom_seg;
		u32	flow_qid;
	} rxu_map[64];			/* 0400 - 06fc */

	struct {
		u32	cos_src;
		u32	dest_prom;
		u32	stream;
	} rxu_type9_map[64];		/* 0700 - 09fc */

	u32	__rsvd11[192];		/* 0a00 - 0cfc */

/* LSU/MAU Registers */
	struct {
		u32 addr_msb;		/* 0d00 */
		u32 addr_lsb_cfg_ofs;	/* 0d04 */
		u32 phys_addr;		/* 0d08 */
		u32 dbell_val_byte_cnt;	/* 0d0c */
		u32 destid;		/* 0d10 */
		u32 dbell_info_fttype;	/* 0d14 */
		u32 busy_full;		/* 0d18 */
	} lsu_reg[8];			/* 0d00 - 0ddc */

	u32	lsu_setup_reg[2];	/* 0de0 - 0de4 */
	u32	lsu_stat_reg[6];	/* 0de8 - 0dfc */
	u32	lsu_flow_masks[4];	/* 0e00 - 0e0c */

	u32	__rsvd12[16];		/* 0e10 - 0e4c */

/* Flow Control Registers */
	u32	flow_cntl[16];		/* 0e50 - 0e8c */
	u32	__rsvd13[8];		/* 0e90 - 0eac */

/* TXU Registers 0eb0 - 0efc */
	u32	tx_cppi_flow_masks[8];	/* 0eb0 - 0ecc */
	u32	tx_queue_sch_info[4];	/* 0ed0 - 0edc */
	u32	garbage_coll_qid[3];	/* 0ee0 - 0ee8 */

	u32	__rsvd14[69];		/* 0eec, 0ef0 - 0ffc */

};

/* CDMAHP Registers 1000 - 2ffc */
struct keystone_rio_pktdma_regs {
	u32	__rsvd[2048];		/* 1000 - 2ffc */
};

/* CSR/CAR Registers  b000+ */
struct keystone_rio_car_csr_regs {
	u32	dev_id;			/* b000 */
	u32	dev_info;		/* b004 */
	u32	assembly_id;		/* b008 */
	u32	assembly_info;		/* b00c */
	u32	pe_feature;		/* b010 */

	u32	sw_port;		/* b014 */

	u32	src_op;			/* b018 */
	u32	dest_op;		/* b01c */

	u32	__rsvd1[7];		/* b020 - b038 */

	u32	data_stm_info;		/* b03c */

	u32	__rsvd2[2];		/* b040 - b044 */

	u32	data_stm_logical_ctl;	/* b048 */
	u32	pe_logical_ctl;		/* b04c */

	u32	__rsvd3[2];		/* b050 - b054 */

	u32	local_cfg_hbar;		/* b058 */
	u32	local_cfg_bar;		/* b05c */

	u32	base_dev_id;		/* b060 */
	u32	__rsvd4;
	u32	host_base_id_lock;	/* b068 */
	u32	component_tag;		/* b06c */
					/* b070 - b0fc */
};

struct keystone_rio_serial_port_regs {
	u32	sp_maint_blk_hdr;	/* b100 */
	u32	__rsvd6[7];		/* b104 - b11c */

	u32	sp_link_timeout_ctl;	/* b120 */
	u32	sp_rsp_timeout_ctl;	/* b124 */
	u32	__rsvd7[5];		/* b128 - b138 */
	u32	sp_gen_ctl;		/* b13c */

	struct {
		u32	link_maint_req;	/* b140 */
		u32	link_maint_resp;/* b144 */
		u32	ackid_stat;	/* b148 */
		u32	__rsvd[2];	/* b14c - b150 */
		u32	ctl2;		/* b154 */
		u32	err_stat;	/* b158 */
		u32	ctl;		/* b15c */
	} sp[4];			/* b140 - b1bc */

					/* b1c0 - bffc */
};

struct keystone_rio_err_mgmt_regs {
	u32	err_report_blk_hdr;	/* c000 */
	u32	__rsvd9;
	u32	err_det;		/* c008 */
	u32	err_en;			/* c00c */
	u32	h_addr_capt;		/* c010 */
	u32	addr_capt;		/* c014 */
	u32	id_capt;		/* c018 */
	u32	ctrl_capt;		/* c01c */
	u32	__rsvd10[2];		/* c020 - c024 */
	u32	port_write_tgt_id;	/* c028 */
	u32	__rsvd11[5];		/* c02c - c03c */

	struct {
		u32	det;		/* c040 */
		u32	rate_en;	/* c044 */
		u32	attr_capt_dbg0;	/* c048 */
		u32	capt_0_dbg1;	/* c04c */
		u32	capt_1_dbg2;	/* c050 */
		u32	capt_2_dbg3;	/* c054 */
		u32	capt_3_dbg4;	/* c058 */
		u32	__rsvd0[3];	/* c05c - c064 */
		u32	rate;		/* c068 */
		u32	thresh;		/* c06c */
		u32	__rsvd1[4];	/* c070 - c07c */
	} sp_err[4];			/* c040 - c13c */

	u32	__rsvd12[1972];		/* c140 - e00c */

	struct {
		u32	stat0;		/* e010 */
		u32	stat1;		/* e014 */
		u32	__rsvd[6];	/* e018 - e02c */
	} lane_stat[4];			/* e010 - e08c */

					/* e090 - 1affc */
};

struct keystone_rio_phy_layer_regs {
	u32	phy_blk_hdr;		/* 1b000 */
	u32	__rsvd14[31];		/* 1b004 - 1b07c */
	struct {
		u32	imp_spec_ctl;	/* 1b080 */
		u32	pwdn_ctl;	/* 1b084 */
		u32	__rsvd0[2];

		u32	status;		/* 1b090 */
		u32	int_enable;	/* 1b094 */
		u32	port_wr_enable;	/* 1b098 */
		u32	event_gen;	/* 1b09c */

		u32	all_int_en;	/* 1b0a0 */
		u32	all_port_wr_en;	/* 1b0a4 */
		u32	__rsvd1[2];

		u32	path_ctl;	/* 1b0b0 */
		u32	discovery_timer;/* 1b0b4 */
		u32	silence_timer;	/* 1b0b8 */
		u32	vmin_exp;	/* 1b0bc */

		u32	pol_ctl;	/* 1b0c0 */
		u32	__rsvd2;
		u32	denial_ctl;	/* 1b0c8 */
		u32	__rsvd3;

		u32	rcvd_mecs;	/* 1b0d0 */
		u32	__rsvd4;
		u32	mecs_fwd;	/* 1b0d8 */
		u32	__rsvd5;

		u32	long_cs_tx1;	/* 1b0e0 */
		u32	long_cs_tx2;	/* 1b0e4 */
		u32	__rsvd[6];	/* 1b0e8, 1b0ec, 1b0f0 - 1b0fc */
	} phy_sp[4];			/* 1b080 - 1b27c */

					/* 1b280 - 1b2fc */
};

struct keystone_rio_transport_layer_regs {
	u32	transport_blk_hdr;	/* 1b300 */
	u32	__rsvd16[31];		/* 1b304 - 1b37c */

	struct {
		u32	control;	/*1b380 */
		u32	__rsvd0[3];

		u32	status;		/* 1b390 */
		u32	int_enable;	/* 1b394 */
		u32	port_wr_enable;	/* 1b398 */
		u32	event_gen;	/* 1b39c */

		struct {
			u32	ctl;		/* 1b3a0 */
			u32	pattern_match;	/* 1b3a4 */
			u32	__rsvd[2];	/* 1b3a8 - 1b3ac */
		} base_route[4];		/* 1b3a0 - 1b3dc */

		u32	__rsvd1[8];		/* 1b3e0 - 1b3fc */

	} transport_sp[4];			/* 1b380 - 1b57c */

						/* 1b580 - 1b5fc */
};

struct keystone_rio_pkt_buf_regs {
	u32	pkt_buf_blk_hdr;	/* 1b600 */
	u32	__rsvd18[31];		/* 1b604 - 1b67c */

	struct {
		u32	control;	/* 1b680 */
		u32	__rsvd0[3];

		u32	status;		/* 1b690 */
		u32	int_enable;	/* 1b694 */
		u32	port_wr_enable;	/* 1b698 */
		u32	event_gen;	/* 1b69c */

		u32	ingress_rsc;	/* 1b6a0 */
		u32	egress_rsc;	/* 1b6a4 */
		u32	__rsvd1[2];

		u32	ingress_watermark[4];	/* 1b6b0 - 1b6bc */
		u32	__rsvd2[16];	/* 1b6c0 - 1b6fc */

	} pkt_buf_sp[4];		/* 1b680 - 1b87c */

					/* 1b880 - 1b8fc */
};

struct keystone_rio_evt_mgmt_regs {
	u32	evt_mgmt_blk_hdr;	/* 1b900 */
	u32	__rsvd20[3];

	u32	evt_mgmt_int_stat;	/* 1b910 */
	u32	evt_mgmt_int_enable;	/* 1b914 */
	u32	evt_mgmt_int_port_stat;	/* 1b918 */
	u32	__rsvd21;

	u32	evt_mgmt_port_wr_stat;	/* 1b920 */
	u32	evt_mgmt_port_wr_enable;/* 1b924 */
	u32	evt_mgmt_port_wr_port_stat;	/* 1b928 */
	u32	__rsvd22;

	u32	evt_mgmt_dev_int_en;	/* 1b930 */
	u32	evt_mgmt_dev_port_wr_en;	/* 1b934 */
	u32	__rsvd23;
	u32	evt_mgmt_mecs_stat;	/* 1b93c */

	u32	evt_mgmt_mecs_int_en;	/* 1b940 */
	u32	evt_mgmt_mecs_cap_en;	/* 1b944 */
	u32	evt_mgmt_mecs_trig_en;	/* 1b948 */
	u32	evt_mgmt_mecs_req;	/* 1b94c */

	u32	evt_mgmt_mecs_port_stat;/* 1b950 */
	u32	__rsvd24[2];
	u32	evt_mgmt_mecs_event_gen;/* 1b95c */

	u32	evt_mgmt_rst_port_stat;	/* 1b960 */
	u32	__rsvd25;
	u32	evt_mgmt_rst_int_en;	/* 1b968 */
	u32	__rsvd26;

	u32	evt_mgmt_rst_port_wr_en;/* 1b970 */
					/* 1b974 - 1b9fc */
};

struct keystone_rio_port_write_regs {
	u32	port_wr_blk_hdr;	/* 1ba00 */
	u32	port_wr_ctl;		/* 1ba04 */
	u32	port_wr_route;		/* 1ba08 */
	u32	__rsvd28;

	u32	port_wr_rx_stat;	/* 1ba10 */
	u32	port_wr_rx_event_gen;	/* 1ba14 */
	u32	__rsvd29[2];

	u32	port_wr_rx_capt[4];	/* 1ba20 - 1ba2c */
					/* 1ba30 - 1bcfc */
};

struct keystone_rio_link_layer_regs {
	u32	link_blk_hdr;		/* 1bd00 */
	u32	__rsvd31[8];		/* 1bd04 - 1bd20 */
	u32	whiteboard;		/* 1bd24 */
	u32	port_number;		/* 1bd28 */

	u32	__rsvd32;		/* 1bd2c */

	u32	prescalar_srv_clk;	/* 1bd30 */
	u32	reg_rst_ctl;		/* 1bd34 */
	u32	__rsvd33[4];		/* 1bd38, 1bd3c, 1bd40, 1bd44 */
	u32	local_err_det;		/* 1bd48 */
	u32	local_err_en;		/* 1bd4c */

	u32	local_h_addr_capt;	/* 1bd50 */
	u32	local_addr_capt;	/* 1bd54 */
	u32	local_id_capt;		/* 1bd58 */
	u32	local_ctrl_capt;	/* 1bd5c */

					/* 1bd60 - 1bdfc */
};

struct keystone_rio_fabric_regs {
	u32	fabric_hdr;		/* 1be00 */
	u32	__rsvd35[3];		/* 1be04 - 1be0c */

	u32	fabric_csr;		/* 1be10 */
	u32	__rsvd36[11];		/* 1be14 - 1be3c */

	u32	sp_fabric_status[4];	/* 1be40 - 1be4c */
};

#endif /* KEYSTONE_RIO_H */
