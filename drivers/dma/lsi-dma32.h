/*
 * Copyright (C) 2012 Ericsson AB. All rights reserved.
 *
 * Author:
 *   Kerstin Jonsson <kerstin.jonsson@ericsson.com>, Feb 2012
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */
#ifndef __LSI_DMA32_H
#define __LSI_DMA32_H

#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include "virt-dma.h"

#define MAX_GPDMA_CHANNELS      4
#define GPDMA_MAX_DESCRIPTORS  128
#define GPDMA_MAGIC            0xABCD1234UL

#define DMA_X_SRC_COUNT				0x00
#define DMA_Y_SRC_COUNT				0x04
#define DMA_X_MODIF_SRC				0x08
#define DMA_Y_MODIF_SRC				0x0c
#define DMA_SRC_CUR_ADDR			0x10
#define DMA_SRC_ACCESS				0x14
#define    DMA_SRC_ACCESS_BURST_TYPE		(1<<15)
#define    DMA_SRC_ACCESS_TAIL_LENGTH(x)	(((x) & 0xF) << 11)
#define    DMA_SRC_ACCESS_ROTATOR_LENGTH(x)	(((x) & 1F) << 6)
#define    DMA_SRC_ACCESS_SRC_SIZE(x)		(((x) & 7) << 3)
#define    DMA_SRC_ACCESS_SRC_BURST(x)		(((x) & 7) << 0)
#define DMA_SRC_MASK				0x18
#define DMA_X_DST_COUNT				0x1c
#define DMA_Y_DST_COUNT				0x20
#define DMA_X_MODIF_DST				0x24
#define DMA_Y_MODIF_DST				0x28
#define DMA_DST_CUR_ADDR			0x2C
#define DMA_DST_ACCESS				0x30
#define    DMA_DST_ACCESS_DST_SIZE(x)		(((x) & 7) << 3)
#define    DMA_DST_ACCESS_DST_BURST(x)		(((x) & 7) << 0)
#define DMA_NXT_DESCR				0x34
#define DMA_CHANNEL_CONFIG			0x38
#define    DMA_CONFIG_DST_SPACE(x)		(((x) & 7) << 26)
#define    DMA_CONFIG_SRC_SPACE(x)		(((x) & 7) << 23)
#define    DMA_CONFIG_PRIORITY_ROW		(1<<21)
#define    DMA_CONFIG_PRIORITY			(1<<20)
#define    DMA_CONFIG_LAST_BLOCK		(1<<15)
#define    DMA_CONFIG_CLEAR_FIFO		(1<<14)
#define    DMA_CONFIG_START_MEM_LOAD		(1<<13)
#define    DMA_CONFIG_STOP_DST_EOB		(1<<11)
#define    DMA_CONFIG_FULL_DESCR_ADDR		(1<<8)
#define    DMA_CONFIG_INT_DST_EOT		(1<<7)
#define    DMA_CONFIG_INT_DST_EOB		(1<<6)
#define    DMA_CONFIG_WAIT_FOR_TASK_CNT2	(1<<5)
#define    DMA_CONFIG_TASK_CNT2_RESET		(1<<4)
#define    DMA_CONFIG_WAIT_FOR_TASK_CNT1	(1<<3)
#define    DMA_CONFIG_TASK_CNT1_RESET		(1<<2)
#define    DMA_CONFIG_TX_EN			(1<<1)
#define    DMA_CONFIG_CHAN_EN			(1<<0)
#define DMA_STATUS				0x3C
#define    DMA_STATUS_WAIT_TASK_CNT2		(1<<20)
#define    DMA_STATUS_TASK_CNT2_OVERFLOW	(1<<19)
#define    DMA_STATUS_WAIT_TASK_CNT1		(1<<18)
#define    DMA_STATUS_TASK_CNT1_OVERFLOW	(1<<17)
#define    DMA_STATUS_CH_PAUS_WR_EN		(1<<16)
#define    DMA_STATUS_ERR_ACC_DESCR		(1<<14)
#define    DMA_STATUS_ERR_ACC_DST		(1<<13)
#define    DMA_STATUS_ERR_ACC_SRC		(1<<12)
#define    DMA_STATUS_ERR_OVERFLOW		(1<<9)
#define    DMA_STATUS_ERR_UNDERFLOW		(1<<8)
#define    DMA_STATUS_CH_PAUSE			(1<<7)
#define    DMA_STATUS_CH_WAITING		(1<<5)
#define    DMA_STATUS_CH_ACTIVE			(1<<4)
#define    DMA_STATUS_TR_COMPLETE		(1<<3)
#define    DMA_STATUS_BLK_COMPLETE		(1<<2)
#define    DMA_STATUS_UNALIGNED_READ		(1<<1)
#define    DMA_STATUS_UNALIGNED_WRITE		(1<<0)
#define    DMA_STATUS_UNALIGNED_ERR		(DMA_STATUS_UNALIGNED_READ | \
						 DMA_STATUS_UNALIGNED_WRITE)
#define DMA_TASK_CNT_1				0x40
#define DMA_TASK_CNT_2				0x44
#define DMA_MODE_CONFIG				0x48
#define DMA_CURR_DESCR				0x4c
#define DMA_PREV_DESCR				0x50
#define DMA_SRC_ADDR_SEG			0x54
#define DMA_DST_ADDR_SEG			0x58
#define DMA_DESCR_ADDR_SEG			0x5c

#define DMA_STATUS_ERROR		(DMA_STATUS_ERR_ACC_DESCR | \
					 DMA_STATUS_ERR_ACC_DST   | \
					 DMA_STATUS_ERR_ACC_SRC   | \
					 DMA_STATUS_ERR_OVERFLOW  | \
					 DMA_STATUS_ERR_UNDERFLOW | \
					 DMA_STATUS_UNALIGNED_ERR)

#define DMA_STATUS_CLEAR		(DMA_STATUS_CH_PAUS_WR_EN | \
					 DMA_STATUS_TR_COMPLETE   | \
					 DMA_STATUS_BLK_COMPLETE)

#define DMA_CONFIG_END			(DMA_CONFIG_LAST_BLOCK | \
					 DMA_CONFIG_INT_DST_EOT)

#define DMA_CONFIG_EOB			(DMA_CONFIG_STOP_DST_EOB | \
					 DMA_CONFIG_INT_DST_EOB)

#define DMA_CONFIG_ONE_SHOT(__ext)	(DMA_CONFIG_DST_SPACE((__ext)) | \
					 DMA_CONFIG_SRC_SPACE((__ext)) | \
					 DMA_CONFIG_TX_EN              | \
					 DMA_CONFIG_CHAN_EN)

#define DMA_CONFIG_DSC_LOAD		(DMA_CONFIG_START_MEM_LOAD  | \
					 DMA_CONFIG_FULL_DESCR_ADDR | \
					 DMA_CONFIG_CHAN_EN)

#define GEN_STAT       0x0
#define   GEN_STAT_CH0_ACTIVE (1<<0)
#define   GEN_STAT_CH1_ACTIVE (1<<2)
#define   GEN_STAT_CH1_ACTIVE (1<<2)
#define   GEN_STAT_CH0_ERROR  (1<<16)
#define   GEN_STAT_CH1_ERROR  (1<<17)
#define GEN_CONFIG     0x4
#define  GEN_CONFIG_EXT_MEM                     (1<<19)
#define  GEN_CONFIG_INT_EDGE(_ch)               (1<<(_ch))
#define SOFT_RESET     0x8

#define GPDMA_GEN_STAT(__p) ((__p)->gbase + GEN_STAT)
#define GPDMA_GEN_CONFIG(__p) ((__p)->gbase + GEN_CONFIG)
#define GPDMA_SOFT_RESET(__p) ((__p)->gbase + SOFT_RESET)


struct descriptor {
	u16 src_x_ctr;
	u16 src_y_ctr;
	s32 src_x_mod;
	s32 src_y_mod;
	u32 src_addr;
	u32 src_data_mask;
	u16 src_access;
	u16 dst_access;
	u32 ch_config;
	u32 next_ptr;
	u16 dst_x_ctr;
	u16 dst_y_ctr;
	s32 dst_x_mod;
	s32 dst_y_mod;
	u32 dst_addr;
} __aligned(32);

struct gpdma_engine;

struct gpdma_desc {
	struct descriptor               hw;
	struct gpdma_desc              *chain;
	struct gpdma_desc	       *head; /* track first desc because of cookie */
	struct gpdma_desc	       *ned;  /* next end-of-block descriptor */
	dma_addr_t                      src;
	dma_addr_t                      dst;
	struct gpdma_engine            *engine;
	struct virt_dma_desc	        vdesc;
} __aligned(32);

static struct gpdma_desc *to_gpdma_desc(struct virt_dma_desc *vdesc)
{
	return container_of(vdesc, struct gpdma_desc, vdesc);
}

struct gpdma_channel {
	/* Back reference to DMA engine */
	struct gpdma_engine		*engine;
	/* Channel registers */
	void __iomem			*base;
	/* Channel id */
	int			        id;
	/* IRQ number as passed to request_irq() */
	int				irq;
	/* Currently running descriptor */
	struct gpdma_desc               *active;
	/* Channel parameters (DMA engine framework) */
	struct virt_dma_chan		vc;
	/* Channel slave configuration */
	struct dma_slave_config		cfg;
};

static inline struct gpdma_channel *to_gpdma_chan(struct dma_chan *chan)
{
	return container_of(chan, struct gpdma_channel, vc.chan);
}

struct lsidma_hw {
	unsigned int num_channels;
	unsigned int chregs_offset;
	unsigned int genregs_offset;
	unsigned int flags;
#define LSIDMA_NEXT_FULL     (1<<0)
#define LSIDMA_SEG_REGS      (1<<1)
#define LSIDMA_EDGE_INT      (1<<2)
};

struct gpdma_engine {
	struct device			*dev;
	struct lsidma_hw		*chip;
	struct gpdma_channel		channel[MAX_GPDMA_CHANNELS];
	/** Bit mask where bit[n] == 1 if channel busy */
	unsigned long                   ch_busy;
	int                             err_irq;
	void __iomem			*iobase;
	void __iomem			*gbase;
	void __iomem			*gpreg;
	spinlock_t			lock;
	struct list_head                free_list;
	struct {
		u32                     order;
		dma_addr_t              phys;
		struct gpdma_desc       *va;
	} pool;
	struct dma_device		dma_device;
};

#define desc_to_engine(n) container_of(n, struct gpdma_engine, desc)

#endif
