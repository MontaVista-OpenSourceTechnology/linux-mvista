/*
 * Copyright (C) 2016 Broadcom
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
#ifndef _APM_H
#define _APM_H

#include <linux/netdevice.h>

#define APM_DEV_CTL						0x000
#define  APM_DC_TSM						0x00000001
#define  APM_DC_ROCS					0x00000002
#define  APM_DC_CFCO					0x00000004
#define  APM_DC_FCM_MASK				0x00000060
#define  APM_DC_FCM_SHIFT				5

#define APM_DEV_STATUS					0x004		/* Configuration of the interface */
#define  APM_DS_RBF						0x00000001
#define  APM_DS_RDF						0x00000002
#define  APM_DS_RIF						0x00000004
#define  APM_DS_TDF						0x00000010
#define  APM_DS_PO						0x00000040
#define  APM_DS_RQS_MASK				0x000f0000
#define  APM_DS_RQS_SHIFT				16
#define  APM_DS_TQS_MASK				0x00f00000
#define  APM_DS_TQS_SHIFT				20
#define  APM_DS_TOC_MASK				0x3f000000
#define  APM_DS_TOC_SHIFT				24

#define APM_DATA_SWAP_CTL				0x010
#define  APM_DSC_TBSWD					0x00000001
#define  APM_DSC_TDS					0x00000002
#define  APM_DSC_RBSWD					0x00000004
#define  APM_DSC_RDS					0x00000008

#define APM_ERROR_STATIS				0x014
#define  APM_ES_DESC_READ_ERR_TC0		0x00000001
#define  APM_ES_DESC_READ_ERR_TC1		0x00000002
#define  APM_ES_DESC_READ_ERR_TC2		0x00000004
#define  APM_ES_DESC_READ_ERR_TC3		0x00000008
#define  APM_ES_DESC_READ_ERR_RC0		0x00000010
#define  APM_ES_DATA_ERR_TC0			0x00000100
#define  APM_ES_DATA_ERR_TC1			0x00000200
#define  APM_ES_DATA_ERR_TC2			0x00000400
#define  APM_ES_DATA_ERR_TC3			0x00000800
#define  APM_ES_DATA_ERR_RC0			0x00001000
#define  APM_ES_DESC_PROT_ERR_TC0		0x00010000
#define  APM_ES_DESC_PROT_ERR_TC1		0x00020000
#define  APM_ES_DESC_PROT_ERR_TC2		0x00040000
#define  APM_ES_DESC_PROT_ERR_TC3		0x00080000
#define  APM_ES_DESC_PROT_ERR_RC0		0x00100000

#define APM_INT_STATUS					0x020		/* Interrupt status */
#define  APM_IS_TO						0x00000080	/* Timeout */
#define  APM_IS_SL_SC					0x00000100  /* Software link status change */
#define  APM_IS_PM_LS					0x00000200  /* Port Macro link status */
#define  APM_IS_DESC_ERR				0x00000400	/* Descriptor error */
#define  APM_IS_DATA_ERR				0x00000800	/* Data error */
#define  APM_IS_DESC_PROT_ERR			0x00001000	/* Descriptor protocol error */
#define  APM_IS_RX_DESC_UNDERF			0x00002000	/* Receive descriptor underflow */
#define  APM_IS_RX_F_OVERF				0x00004000	/* Receive FIFO overflow */
#define  APM_IS_RX						0x00010000	/* Interrupt for RX queue 0 */
#define  APM_IS_RX_INFO_ECC_CORR		0x00100000	/* RXQ info memory corrected error */
#define  APM_IS_RX_INFO_ECC_UNCORR		0x00200000	/* RXQ info memory uncorrected error */
#define  APM_IS_AXI_SHARED_ECC_CORR		0x00400000	/* AXI share memory corrected error */
#define  APM_IS_AXI_SHARED_ECC_UNCORR	0x00800000	/* AXI share memory uncorrected error */
#define  APM_IS_TX0						0x01000000	/* Interrupt for TX queue 0 */
#define  APM_IS_TX1						0x02000000	/* Interrupt for TX queue 1 */
#define  APM_IS_TX2						0x04000000	/* Interrupt for TX queue 2 */
#define  APM_IS_TX3						0x08000000	/* Interrupt for TX queue 3 */
#define  APM_IS_ECC_MASK				0x00f00000
#define  APM_IS_TX_MASK					0x0f000000
#define  APM_IS_INTMASK					0x0ff17f80
#define  APM_IS_ERRMASK					0x00007c00

#define APM_INT_MASK							0x024		/* Interrupt mask */
#define APM_GP_TIMER							0x028

#define APM_TXQ_COMMON_CTL						0x040
#define  APM_TCCTL_FLUSH_CREDIT					0x00000001
#define  APM_TCCTL_FUNC_MODE_MASK				0x00000006
#define  APM_TCCTL_FUNC_MODE_SHIFT				1
#define   APM_FUNC_MODE_SINGLE					0
#define   APM_FUNC_MODE_DUAL					1
#define   APM_FUNC_MODE_QUAD					2
#define  APM_TCCTL_WR_FUNC_MODE					0x00000008
#define  APM_TCCTL_INTERRUPT_SEL				0x00000010

#define APM_TXQ_DATA_TXREQ_CTL0					0x044
#define APM_TXQ_DATA_TXREQ_CTL1					0x048
#define APM_TXQ_SHARED_BUF_DEPTH0				0x050
#define APM_TXQ_SHARED_BUF_DEPTH1				0x054
#define APM_RXQ_BUF_DEPTH						0x058
#define APM_DMA_TOTAL_OUTSTD_TRANS_LIMIT		0x080
#define APM_DMA_RD_PER_ID_OUTSTD_TRANS_LIMIT	0x084
#define APM_PKT_DMA_RD_AXI_MAP_CTRL				0x088
#define APM_RX_PKT_DMA_WR_AXI_MAP_CTRL			0x08c
#define APM_TX_PKT_DMA_RD_ARB_CTRL				0x090
#define APM_RX_PKT_DMA_RD_ARB_CTRL				0x094
#define APM_RX_PKT_DMA_WR_ARB_CTRL				0x098
#define APM_RCBUF_MAX_FLIST_ENTRIES				0x09c
#define APM_STAT_COUNTER_CTL					0x0a0
#define APM_STAT_RXQ_TRANSFERRED_PKT_CNT		0x0b0
#define APM_STAT_RXQ_COMPLETE_PKT_DROP_CNT		0x0b4
#define APM_STAT_RXQ_PARTIAL_PKT_DROP_CNT		0x0b8
#define APM_STAT_RXQ_TRUNCATED_PKT_CNT			0x0bc
#define APM_STAT_TXQ_CH0_GOOD_PKT_CNT			0x0c0
#define APM_STAT_TXQ_CH0_ERR_PKT_CNT			0x0c4
#define APM_STAT_TXQ_CH1_GOOD_PKT_CNT			0x0c8
#define APM_STAT_TXQ_CH1_ERR_PKT_CNT			0x0cc
#define APM_STAT_TXQ_CH2_GOOD_PKT_CNT			0x0d0
#define APM_STAT_TXQ_CH2_ERR_PKT_CNT			0x0d4
#define APM_STAT_TXQ_CH3_GOOD_PKT_CNT			0x0d8
#define APM_STAT_TXQ_CH3_ERR_PKT_CNT			0x0dc
#define APM_DBG_TXQ_CH0_STM						0x0e0
#define APM_DBG_TXQ_CH1_STM						0x0e4
#define APM_DBG_TXQ_CH2_STM						0x0e8
#define APM_DBG_TXQ_CH3_STM						0x0ec
#define APM_DBG_RXQ_STM							0x0f0
#define APM_DBG_DMA_HOSTRD_STM					0x0f8
#define APM_DBG_DMA_HOSTWR_STM					0x0fc

#define APM_INT_RECV_LAZY						0x100
#define  APM_IRL_TO_MASK						0x00ffffff
#define  APM_IRL_TO_SHIFT						0
#define  APM_IRL_FC_MASK						0xff000000
#define  APM_IRL_FC_SHIFT						24		/* Shift the number of interrupts triggered per received frame */
#define APM_FLOW_CTL_THRESH						0x104		/* Flow control thresholds */
#define APM_WRRTHRESH							0x108
#define APM_RXQ_CTL								0x190
#define  APM_RXQ_CTL_DBT_MASK					0x00000f7ff
#define  APM_RXQ_CTL_DBT_SHIFT					0
#define  APM_RXQ_CTL_RPT_EN						0x40000000

#define APM_MEM_ECC_CTL							0x1F0
#define APM_MEM_ECC_STAT						0x1F4

#define APM_DMA_BASE0							0x200	/* Tx and Rx controller */
#define APM_DMA_BASE1							0x240	/* Tx controller only */
#define APM_DMA_BASE2							0x280	/* Tx controller only */
#define APM_DMA_BASE3							0x2C0	/* Tx controller only */

#define APM_DMA_TX_CTL							0x00
#define  APM_DMA_TX_ENABLE						0x00000001
#define  APM_DMA_TX_SUSPEND						0x00000002
#define  APM_DMA_TX_SBAI						0x00002000
#define  APM_DMA_TX_BL_MASK						0x001C0000	/* BurstLen bits */
#define  APM_DMA_TX_BL_SHIFT					18
#define   APM_DMA_TX_BL_16						0
#define   APM_DMA_TX_BL_32						1
#define   APM_DMA_TX_BL_64						2
#define   APM_DMA_TX_BL_128						3
#define  APM_DMA_TX_PC_MASK						0x00E00000	/* Prefetch control */
#define  APM_DMA_TX_PC_SHIFT					21
#define   APM_DMA_TX_PC_0						0
#define   APM_DMA_TX_PC_4						1
#define   APM_DMA_TX_PC_8						2
#define   APM_DMA_TX_PC_16						3
#define  APM_DMA_TX_PT_MASK						0x03000000	/* Prefetch threshold */
#define  APM_DMA_TX_PT_SHIFT					24
#define   APM_DMA_TX_PT_1						0
#define   APM_DMA_TX_PT_2						1
#define   APM_DMA_TX_PT_4						2
#define   APM_DMA_TX_PT_8						3

#define APM_DMA_TX_INDEX						0x04
#define APM_DMA_TX_RINGLO						0x08
#define APM_DMA_TX_RINGHI						0x0C
#define APM_DMA_TX_STATUS						0x10
#define  APM_DMA_TX_STATDPTR					0x00001FFF
#define  APM_DMA_TX_STAT						0xF0000000
#define   APM_DMA_TX_STAT_DISABLED				0x00000000
#define   APM_DMA_TX_STAT_ACTIVE				0x10000000
#define   APM_DMA_TX_STAT_IDLEWAIT				0x20000000
#define   APM_DMA_TX_STAT_STOPPED				0x30000000
#define   APM_DMA_TX_STAT_SUSP					0x40000000
#define APM_DMA_TX_ERROR						0x14
#define  APM_DMA_TX_ERRDPTR						0x0001FFFF
#define  APM_DMA_TX_ERR							0xF0000000
#define   APM_DMA_TX_ERR_NOERR					0x00000000
#define   APM_DMA_TX_ERR_PROT					0x10000000
#define   APM_DMA_TX_ERR_TRANSFER				0x30000000
#define   APM_DMA_TX_ERR_DESCREAD				0x40000000
#define   APM_DMA_TX_ERR_CORE					0x50000000

#define APM_DMA_RX_CTL							0x20
#define  APM_DMA_RX_ENABLE						0x00000001
#define  APM_DMA_RX_FRAME_OFFSET_MASK			0x000000FE
#define  APM_DMA_RX_FRAME_OFFSET_SHIFT			1
#define  APM_DMA_RX_OVERFLOW_CONT				0x00000400
#define  APM_DMA_RX_BL_MASK						0x001C0000	/* BurstLen bits */
#define  APM_DMA_RX_BL_SHIFT					18
#define   APM_DMA_RX_BL_16						0
#define   APM_DMA_RX_BL_32						1
#define   APM_DMA_RX_BL_64						2
#define   APM_DMA_RX_BL_128						3
#define  APM_DMA_RX_PC_MASK						0x00E00000	/* Prefetch control */
#define  APM_DMA_RX_PC_SHIFT					21
#define   APM_DMA_RX_PC_0						0
#define   APM_DMA_RX_PC_4						1
#define   APM_DMA_RX_PC_8						2
#define   APM_DMA_RX_PC_16						3
#define  APM_DMA_RX_PT_MASK						0x03000000	/* Prefetch threshold */
#define  APM_DMA_RX_PT_SHIFT					24
#define   APM_DMA_RX_PT_1						0
#define   APM_DMA_RX_PT_2						1
#define   APM_DMA_RX_PT_4						2
#define   APM_DMA_RX_PT_8						3
#define APM_DMA_RX_INDEX						0x24
#define APM_DMA_RX_RINGLO						0x28
#define APM_DMA_RX_RINGHI						0x2C
#define APM_DMA_RX_STATUS						0x30
#define  APM_DMA_RX_STATDPTR					0x00001FFF
#define  APM_DMA_RX_STAT						0xF0000000
#define   APM_DMA_RX_STAT_DISABLED				0x00000000
#define   APM_DMA_RX_STAT_ACTIVE				0x10000000
#define   APM_DMA_RX_STAT_IDLEWAIT				0x20000000
#define   APM_DMA_RX_STAT_STOPPED				0x30000000
#define APM_DMA_RX_ERROR						0x34
#define  APM_DMA_RX_ERRDPTR						0x0001FFFF
#define  APM_DMA_RX_ERR							0xF0000000
#define   APM_DMA_RX_ERR_NOERR					0x00000000
#define   APM_DMA_RX_ERR_PROT					0x10000000
#define   APM_DMA_RX_ERR_UNDERRUN				0x20000000
#define   APM_DMA_RX_ERR_TRANSFER				0x30000000
#define   APM_DMA_RX_ERR_DESCREAD				0x40000000
#define   APM_DMA_RX_ERR_CORE					0x50000000


#define APM_DESC_CTL0_CRC						0x00300000  /* CRC mode */
#define APM_DESC_CRC_APPEND						0x00000000	/* CRC append mode */
#define APM_DESC_CRC_OVERWRITE					0x00100000	/* CRC overwrite mode */
#define APM_DESC_CRC_FORWARD					0x00200000	/* CRC forward mode */
#define APM_DESC_CTL0_EOT						0x10000000	/* End of ring */
#define APM_DESC_CTL0_IOC						0x20000000	/* IRQ on complete */
#define APM_DESC_CTL0_EOF						0x40000000	/* End of frame */
#define APM_DESC_CTL0_SOF						0x80000000	/* Start of frame */
#define APM_DESC_CTL1_LEN						0x00007FFF

#define APM_MAX_TX_RINGS						4
#define APM_MAX_RX_RINGS						1
#define APM_TX_MAX_DESCS						512
#define APM_RX_MAX_DESCS						512

#define APM_RX_HEADER_LEN						28		/* Last 24 bytes are unused. Well... */
#define APM_RX_FRAME_OFFSET						30		/* There are 2 unused bytes between header and real data */
#define APM_RX_BUF_OFFSET						(NET_SKB_PAD + NET_IP_ALIGN - APM_RX_FRAME_OFFSET)
#define APM_RX_MAX_FRAME_SIZE					1536	/* Copied from b44/tg3 */
#define APM_RX_BUF_SIZE			(APM_RX_FRAME_OFFSET + APM_RX_MAX_FRAME_SIZE)
#define APM_RX_ALLOC_SIZE		(SKB_DATA_ALIGN(APM_RX_BUF_SIZE + APM_RX_BUF_OFFSET) + \
								 SKB_DATA_ALIGN(sizeof(struct skb_shared_info)))

#define APM_WEIGHT	64
#define ETHER_MAX_LEN   1518

/* Feature flags */
#define APM_FEAT_TX_MASK_SETUP		BIT(0)
#define APM_FEAT_RX_MASK_SETUP		BIT(1)

/* Loopback flags */
#define APM_LOOPBACK_TYPE_NONE			0
#define APM_LOOPBACK_TYPE_MAC			1
#define APM_LOOPBACK_TYPE_PHY			2

struct apm_slot_info {
	union {
		struct sk_buff *skb;
		void *buf;
	};
	dma_addr_t dma_addr;
};

struct apm_dma_desc {
	__le32 ctl0;
	__le32 ctl1;
	__le32 addr_low;
	__le32 addr_high;
} __packed;

enum apm_dma_ring_type {
	APM_DMA_RING_TYPE_TX = 0,
	APM_DMA_RING_TYPE_RX,
	APM_DMA_RING_TYPE_NUM
};

/**
 * apm_dma_ring - contains info about DMA ring (either TX or RX one)
 * @start: index of the first slot containing data
 * @end: index of a slot that can *not* be read (yet)
 *
 * Be really aware of the specific @end meaning. It's an index of a slot *after*
 * the one containing data that can be read. If @start equals @end the ring is
 * empty.
 */
struct apm_dma_ring {
	u32 start;
	u32 end;

	int desc_num;
	struct apm_dma_desc *desc_base;
	dma_addr_t dma_base;
	u32 index_base; /* Used for unaligned rings only, otherwise 0 */
	u16 mmio_base;
	bool unaligned;

	struct apm_slot_info *slots;
};


struct apm_rx_header {
	__le16 len;
	__le16 flags;
	__le16 pad[12];
};

struct apm {
	union {
		struct {
			void *base;
			void *idm_base;
		} plat;
	};

	struct device *dev;
	struct device *dma_dev;

	struct iproc_pm_ops *pm_ops;
	u8 land_idx;

	u8 mac_addr[ETH_ALEN];
	u32 feature_flags;

	struct net_device *net_dev;
	struct napi_struct napi;
	struct mii_bus *mii_bus;

	/* DMA */
	struct apm_dma_desc *desc_buf;
	dma_addr_t dma_addr;
	struct apm_slot_info *slot_buf;

	struct apm_dma_ring tx_ring[APM_MAX_TX_RINGS];
	struct apm_dma_ring rx_ring[APM_MAX_RX_RINGS];

	/* QoS */
	u8 tx_channel;
	bool strict_mode;

	/* Int */
	int irq0;
	int irq1;
	int irq2;
	u32 int_mask;

	/* Current MAC state */
	int mac_speed;
	int mac_duplex;

	u8 phyaddr;
	bool loopback;
};

extern int apm_ethtool_init(struct net_device *net_dev);
#endif /* _APM_H */
