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
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/hardirq.h>
#include <linux/kfifo.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/keystone-dma.h>
#include <linux/hwqueue.h>
#include <linux/bitops.h>
#include "keystone_rio.h"

#define DRIVER_VER	        "v1.2"

#define K2_SERDES(p)            ((p)->board_rio_cfg.keystone2_serdes)

#define reg_rmw(addr, value, mask) \
	__raw_writel(((__raw_readl(addr) & (~(mask))) | (value)), (addr))

/*
 * mport.host_deviceid is calculated as base_id + node_id_offset
 */
static unsigned int node_id_offset = 0xc;
module_param(node_id_offset, uint, 0644);
MODULE_PARM_DESC(node_id_offset,
		"Offset from base id used in the calculation of host_deviceid");


struct serdes_reg_value {
	unsigned int offset;
	unsigned int bit_mask;
	unsigned int value;
};

/*
 * Main KeyStone RapidIO driver data
 */
struct keystone_rio_data {
	struct device	       *dev;
	struct rio_mport       *mport[KEYSTONE_RIO_MAX_PORT];
	struct clk	       *clk;
	struct completion	lsu_completion;
	struct mutex		lsu_lock;
	u8                      lsu_dio;
	u8                      lsu_maint;
	u32			rio_pe_feat;

	struct port_write_msg	port_write_msg;

	struct work_struct	pe_work;
	u32			pe_ports;

	struct work_struct	reset_work;

	u32			ports_registering;
	struct delayed_work	port_chk_task;
	struct delayed_work	garbage_monitor_task;
	struct tasklet_struct	task;

	unsigned long		rxu_map_start;
	unsigned long		rxu_map_end;
	unsigned long		rxu_map_bitmap[2];

	u32                     num_mboxes;

	struct keystone_rio_rx_chan_info rx_channels[KEYSTONE_RIO_MAX_MBOX][KEYSTONE_RIO_MAX_LETTER];
	struct keystone_rio_tx_chan_info tx_channels[KEYSTONE_RIO_MAX_MBOX];

	u32 *__iomem gq_pop_push_reg[KEYSTONE_RIO_MAX_GARBAGE_QUEUES];
	u32 *__iomem gq_peek_reg[KEYSTONE_RIO_MAX_GARBAGE_QUEUES];

	u32 *__iomem qmss_mem_region;

	struct keystone_rio_regs __iomem	*regs;
	u32 __iomem				*jtagid_reg;
	u32 __iomem				*serdes_sts_reg;
	union {
		struct keystone_srio_serdes_regs __iomem	*serdes_regs;
		struct keystone2_srio_serdes_regs __iomem	*k2_serdes_regs;
	};

	struct keystone_rio_car_csr_regs __iomem	*car_csr_regs;
	struct keystone_rio_serial_port_regs __iomem	*serial_port_regs;
	struct keystone_rio_err_mgmt_regs __iomem	*err_mgmt_regs;
	struct keystone_rio_phy_layer_regs __iomem	*phy_regs;
	struct keystone_rio_transport_layer_regs __iomem *transport_regs;
	struct keystone_rio_pkt_buf_regs __iomem	*pkt_buf_regs;
	struct keystone_rio_evt_mgmt_regs __iomem	*evt_mgmt_regs;
	struct keystone_rio_port_write_regs __iomem	*port_write_regs;
	struct keystone_rio_link_layer_regs __iomem	*link_regs;
	struct keystone_rio_fabric_regs __iomem		*fabric_regs;

	struct keystone_rio_board_controller_info	 board_rio_cfg;
	u8						 isolate;
	struct serdes_reg_value				*serdes_values;
	int						 serdes_val_len;

#ifdef CONFIG_RAPIDIO_MEM_MAP
	struct rio_mem_map *mem_map;
#endif
};

static int keystone_rio_setup_controller(struct keystone_rio_data *krio_priv);
static void keystone_rio_shutdown_controller(struct keystone_rio_data *krio_priv);
static void keystone_rio_serdes_lane_disable(u32 lane, struct keystone_rio_data *krio_priv);
static void dbell_handler(struct keystone_rio_data *krio_priv);
static void keystone_rio_port_write_handler(struct keystone_rio_data *krio_priv);
static void keystone_rio_llerr_evt_handler(struct keystone_rio_data *krio_priv);
static void keystone_rio_mcast_evt_handler(struct keystone_rio_data *krio_priv);
static void keystone_rio_plerr_evt_handler(struct keystone_rio_data *krio_priv,
		u32 port);

struct keystone_lane_config {
	int start; /* lane start number of the port */
	int end;   /* lane end number of the port */
};

static struct keystone_lane_config keystone_lane_configs[5][KEYSTONE_RIO_MAX_PORT] = {
	{ {0, 1}, {1, 2},   {2, 3},   {3, 4}   }, /* path mode 0: 4 ports in 1x    */
	{ {0, 2}, {-1, -1}, {2, 3},   {3, 4}   }, /* path mode 1: 3 ports in 2x/1x */
	{ {0, 1}, {1, 2},   {2, 4},   {-1, -1} }, /* path mode 2: 3 ports in 1x/2x */
	{ {0, 2}, {-1, -1}, {2, 4},   {-1, -1} }, /* path mode 3: 2 ports in 2x    */
	{ {0, 4}, {-1, -1}, {-1, -1}, {-1, -1} }, /* path mode 4: 1 ports in 4x    */
};

#ifdef CONFIG_RAPIDIO_MEM_MAP

#define KEYSTONE_RIO_SYNC_BUF_LEN (0x8)
static unsigned char *sync_buf = NULL;

static void keystone_rio_sync_buf_alloc(struct keystone_rio_data *krio_priv)
{
  dma_addr_t dma;
  sync_buf = kmalloc(KEYSTONE_RIO_SYNC_BUF_LEN, GFP_KERNEL | GFP_DMA);
  if (!sync_buf) {
    pr_err("failed to allocate sync region\n");
    return;
  }
  dma = dma_map_single(krio_priv->dev, (void*)sync_buf, KEYSTONE_RIO_SYNC_BUF_LEN, DMA_BIDIRECTIONAL);
  pr_debug("%s sync_buf=%p dma=%pad\n", __func__, sync_buf, &dma);
}

void* keystone_rio_sync_buf_addr_get(void)
{
  pr_debug("%s sync_buf=%p content=0x%016x\n", __func__, sync_buf, *sync_buf);
  return sync_buf;
}

size_t keystone_rio_sync_buf_len_get(void)
{
  pr_debug("%s 0x%x\n", __func__, KEYSTONE_RIO_SYNC_BUF_LEN);
  return KEYSTONE_RIO_SYNC_BUF_LEN;
}

/*
 * Map a kernel logical address to a physical address in the lower
 * memory area (addressable on 32 bits). Note that on Keystone 2, DDR3A
 * is mapped in 2 different regions: one in the lower memory area and
 * one in the upper memory area (above address 2^32). All kernel
 * pointers (logical addresses) correspond to the lower area mapping.
 * However, virt_to_phys() simply returns the equivalent address in the
 * upper memory area. In RIO we prefer the lower memory area address,
 * because the high memory area address is larger than 34 bits and
 * cannot be represented on a 34-bit RIO address.
 *
 * Example: 0xc1001234 is the lower memory area equivalent of
 *          0x841001234.
 */
static inline unsigned long krio_map_low(void *p)
{
	return (unsigned long)p;
}

static u64 keystone_op_map_virt(struct rio_mem_map *map,
		void *addr, size_t size)
{
	return krio_map_low(addr);
}

static u64 keystone_op_map_phys(struct rio_mem_map *map,
		phys_addr_t addr, size_t size)
{
	return krio_map_low(phys_to_virt(addr));
}

static const struct rio_mem_map_ops keystone_rio_mem_map_ops = {
	.alloc_block	= rio_mem_map_op_alloc_block,
	.free_block	= rio_mem_map_op_free_block,
	.map_virt	= keystone_op_map_virt,
	.map_phys	= keystone_op_map_phys,
};
#endif

/*----------------------- Interrupt management -------------------------*/

static irqreturn_t lsu_interrupt_handler(int irq, void *data)
{
	struct keystone_rio_data *krio_priv = data;
	u32 pending_lsu_int =
		__raw_readl(&(krio_priv->regs->lsu_int[0].status));

	if (pending_lsu_int & KEYSTONE_RIO_ICSR_LSU0(0))
		complete(&krio_priv->lsu_completion);

	/* ACK the interrupt. ICS0-ICS7, ICS16-ICS23 SRCID for DSP cores, do not ack it  */
	pending_lsu_int &= 0xff00ff00;
	__raw_writel(pending_lsu_int,
		     &(krio_priv->regs->lsu_int[0].clear));

	return IRQ_HANDLED;
}

static void special_interrupt_handler(int ics, struct keystone_rio_data *krio_priv)
{
	u32 lanes = 0;
	u32 sp_gen_ctl;
	u32 fabric_csr;
	u32 fabric_stat;

	/* Acknowledge the interrupt */
	__raw_writel(BIT(ics), &krio_priv->regs->err_rst_evnt_int_clear);
	fabric_csr = __raw_readl(&krio_priv->fabric_regs->fabric_csr);
	fabric_stat = __raw_readl(krio_priv->fabric_regs->sp_fabric_status);

	dev_info_ratelimited(krio_priv->dev, "ics = %d fabric csr 0x%x fabric stat 0x%x\n",
			ics, fabric_csr, fabric_stat);

	switch(ics) {
	case KEYSTONE_RIO_MCAST_EVT_INT:
		/* Multi-cast event control symbol interrupt
		   received on any port */
		if (printk_ratelimit())
			dev_info(krio_priv->dev, "Multi-cast event control symbol interrupt"
									 " received on any port\n");
		keystone_rio_mcast_evt_handler(krio_priv);
		break;

	case KEYSTONE_RIO_PORT_WRITEIN_INT:
		/* Port-write-in request received on any port */
		if (printk_ratelimit())
			dev_info(krio_priv->dev, "Port-write-in request received on any port\n");
		keystone_rio_port_write_handler(krio_priv);
		break;

	case KEYSTONE_RIO_EVT_CAP_ERROR_INT:
		/* Logical layer error management event capture */
		keystone_rio_llerr_evt_handler(krio_priv);
		break;

	case KEYSTONE_RIO_PORT0_ERROR_INT:
	case KEYSTONE_RIO_PORT1_ERROR_INT:
	case KEYSTONE_RIO_PORT2_ERROR_INT:
	case KEYSTONE_RIO_PORT3_ERROR_INT:
		/* Port error */
		keystone_rio_plerr_evt_handler(krio_priv, (ics&0x00000F00)>>8);

		/* For host device
		 * add port to failed ports and schedule recovery */
		sp_gen_ctl =
			__raw_readl(&krio_priv->serial_port_regs->sp_gen_ctl);
		if (sp_gen_ctl & RIO_PORT_GEN_HOST) {
			krio_priv->pe_ports |=
				BIT(ics - KEYSTONE_RIO_PORT0_ERROR_INT);
			schedule_work(&krio_priv->pe_work);
		}
		break;

	case KEYSTONE_RIO_RESET_INT:
		/* Device reset interrupt on any port */
	        dev_info(krio_priv->dev, "Device reset request received on any port\n");

		/* Disable SerDes lanes asap to generate loss of link */
		lanes = krio_priv->board_rio_cfg.lanes;
		while (lanes) {
			u32 lane = __ffs(lanes);
			lanes &= ~(1 << lane);
			keystone_rio_serdes_lane_disable(lane, krio_priv);
		}

		/* Schedule SRIO peripheral reinitialization */
		schedule_work(&krio_priv->reset_work);
		break;
	}
}

static void event_management_handler(struct keystone_rio_data *krio_priv)
{
	u32 em_status;
	em_status = __raw_readl(&krio_priv->evt_mgmt_regs->evt_mgmt_int_stat);
	if (em_status & KEYSTONE_RIO_BLOCK_EM_STATUS)
		dev_info_ratelimited(krio_priv->dev, "Event management status 0x%08x\n", em_status);
}

static irqreturn_t rio_interrupt_handler(int irq, void *data)
{
	struct keystone_rio_data *krio_priv = data;

	u32 pending_err_rst_evnt_int =
		__raw_readl(&(krio_priv->regs->err_rst_evnt_int_stat)) &
		KEYSTONE_RIO_ERR_RST_EVNT_MASK;

    if (0 != pending_err_rst_evnt_int)
        return IRQ_WAKE_THREAD;
    else
        return IRQ_NONE;
}

static irqreturn_t rio_interrupt_thrd_handler(int irq, void *data)
{
	struct keystone_rio_data *krio_priv = data;
    u32 pending_err_rst_evnt_int =
		__raw_readl(&(krio_priv->regs->err_rst_evnt_int_stat)) &
		KEYSTONE_RIO_ERR_RST_EVNT_MASK;

    irq = irq;

    do {
        /* Handle special interrupts (error, reset, special event) */
        while (pending_err_rst_evnt_int) {
            u32 ics = __ffs(pending_err_rst_evnt_int);
            pending_err_rst_evnt_int &= ~(1 << ics);
            special_interrupt_handler(ics, krio_priv);
        }

        /* Call doorbell handler(s) */
        dbell_handler(krio_priv);

        event_management_handler(krio_priv);

        pending_err_rst_evnt_int =
                __raw_readl(&(krio_priv->regs->err_rst_evnt_int_stat)) &
                KEYSTONE_RIO_ERR_RST_EVNT_MASK;
    } while (pending_err_rst_evnt_int);

	return IRQ_HANDLED;
}

static int rio_starvation_peek(struct keystone_rio_data *krio_priv)
{
	static u32 count = 0;
	struct keystone_rio_rx_chan_info *krx_chan;
	struct hwqueue *q;
	u32 i, j, ret;
	u32 desc_cnt = 0;
	u32 cnt = 0;
	u32 fdq;
	u32 pops = 0, pushes = 0, notifies = 0, cpu = 0;
	u32 push_err = 0, pop_err = 0;
	struct hwqueue *compq;

	count++;

	for (i = 0; i < KEYSTONE_RIO_MAX_MBOX; i++) {
		for (j = 0; j < KEYSTONE_RIO_MAX_LETTER; j++) {
			krx_chan = &krio_priv->rx_channels[i][j];
			if (!krx_chan->queue_num)
				continue;
			if (!mutex_trylock(&krx_chan->dma_lock)) {
				dev_warn(krio_priv->dev,
#ifdef KEYSTONE_RIO_RX_OUTOFORDER_WA
					"pktdma-fdq starvation %d/%d. RXDMA-Q-%d channel stopped-%d m%dl%d rx:%d ins:%d\n",
					count, krx_chan->starv_count, krx_chan->queue_num, krx_chan->running, i, j,
					krx_chan->packet_rx_ix, krx_chan->packet_insert_ix);
#else
					"pktdma-fdq starvation %d/%d. RXDMA-Q-%d channel stopped-%d m%dl%d\n",
					count, krx_chan->starv_count, krx_chan->queue_num, krx_chan->running, i, j);
#endif
				continue;
			}
			if ((!krx_chan->running) || (!krx_chan->dma_channel)) {
				dev_warn_ratelimited(krio_priv->dev, "pktdma channel stopped m%dl%d\n", i, j);
				goto checknext;
			}
			ret = dma_get_rx_hw_fdq(krx_chan->dma_channel, 0);
			if (!ret) {
				dev_warn_ratelimited(krio_priv->dev, "pktdma no hw fdq m%dl%d\n", i, j);
				goto checknext;
			}
			q = (struct hwqueue*)((void*)(ret));
			desc_cnt = hwqueue_get_count(q);
			fdq = dma_get_rx_fdq(krx_chan->dma_channel, 0);
			if (!fdq) {
				dev_warn_ratelimited(krio_priv->dev, "pktdma no fdq m%dl%d\n", i, j);
				goto checknext;
			}
			if (desc_cnt > KEYSTONE_RIO_THRESH_STARVATION)
				goto checknext;

			krx_chan->starv_count++;
			cnt++;

			ret = dma_get_rx_hw_compq(krx_chan->dma_channel);
			if (!ret) {
				dev_warn_ratelimited(krio_priv->dev, "pktdma no access to hw complete queue m%dl%d\n", i, j);
				goto checknext;
			}
			compq = (struct hwqueue*)((void*)ret);
			for_each_possible_cpu(cpu) {
				pops += per_cpu_ptr(compq->stats, cpu)->pops;
				pushes  += per_cpu_ptr(q->stats, cpu)->pushes;
				notifies += per_cpu_ptr(compq->stats, cpu)->notifies;
				push_err += per_cpu_ptr(q->stats, cpu)->push_errors;
				pop_err += per_cpu_ptr(compq->stats, cpu)->pop_errors;
			}
			dev_warn(krio_priv->dev,
#ifdef KEYSTONE_RIO_RX_OUTOFORDER_WA
				"pktdma-fdq starvation %d/%d m%dl%d fdq-%d rx:%d ins:%d [%d %d %d %d %d %d]\n",
				count, krx_chan->starv_count, i, j, fdq,
				krx_chan->packet_rx_ix, krx_chan->packet_insert_ix,
				pushes, pops, desc_cnt, notifies, push_err, pop_err);
#else
				"pktdma-fdq starvation %d/%d m%dl%d fdq-%d [%d %d %d %d %d %d]\n",
				count, krx_chan->starv_count, i, j, fdq,
				pushes, pops, desc_cnt, notifies, push_err, pop_err);
#endif
checknext:
			mutex_unlock(&krx_chan->dma_lock);
		}
	}
	return cnt;
}

static irqreturn_t rio_starvation_interrupt_thread(int irq, void *data)
{
	struct keystone_rio_data *krio_priv = data;
	int ret = 0;

	ret = rio_starvation_peek(krio_priv);
	if (!ret)
		dev_warn_ratelimited(krio_priv->dev, "pktdma-fdq starvation happened %d lfs channels\n", ret);

	// Throttle threaded interrupt as a workaround for PR247252
	msleep(1000);

	return IRQ_HANDLED;
}

/*
 * Map a sRIO event to a sRIO interrupt
 */
static void keystone_rio_interrupt_map(u32 __iomem *reg, u32 mask, u32 rio_int)
{
	int i;
	u32 reg_val;

	reg_val = __raw_readl(reg);

	for (i = 0; i <= 32; i+= 4) {
		if ((mask >> i) & 0xf) {
			reg_val &= ~(0xf << i);
			reg_val |= (rio_int << i);
		}
	}
	__raw_writel(reg_val, reg);
}

/*
 * Setup RIO interrupts
 */
static void keystone_rio_interrupt_setup(struct keystone_rio_data *krio_priv)
{
	int res;

	/* Clear all pending interrupts */
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[0].clear));
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[1].clear));
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[2].clear));
	__raw_writel(0x0000ffff,
		     &(krio_priv->regs->doorbell_int[3].clear));
	__raw_writel(0xffffffff,
		     &(krio_priv->regs->lsu_int[krio_priv->lsu_dio].clear));
	__raw_writel(0x00010f07,
		     &(krio_priv->regs->err_rst_evnt_int_clear));

	/* LSU interrupts are routed to RIO interrupt dest 1 (LSU) */
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu0_int_route[0]),
				   0x88888888, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu0_int_route[1]),
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu0_int_route[2]),
				   0x88888888, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu0_int_route[3]),
				   0x11111111, KEYSTONE_LSU_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->lsu1_int_route1),
				   0x11111111, KEYSTONE_LSU_RIO_INT);

	/* Error, reset and special event interrupts are routed to RIO interrupt dest 0 (Rx/Tx) */
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[0]),
				   0x00000111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[1]),
				   0x00001111, KEYSTONE_GEN_RIO_INT);
	keystone_rio_interrupt_map(&(krio_priv->regs->err_rst_evnt_int_route[2]),
				   0x00000001, KEYSTONE_GEN_RIO_INT);

	/* The doorbell interrupts routing table is for the 16 general purpose interrupts */
	__raw_writel(0x1, &(krio_priv->regs->interrupt_ctl));

	/* Do not use pacing */
	__raw_writel(0x0000ffff, &krio_priv->regs->intdst_rate_disable);

	/* Attach interrupt handlers */
	res = request_threaded_irq(krio_priv->board_rio_cfg.rio_irq,
			  rio_interrupt_handler,
			  rio_interrupt_thrd_handler,
			  IRQF_TRIGGER_NONE | IRQF_ONESHOT,
			  "sRIO",
			  krio_priv);
	if (res)
		dev_err(krio_priv->dev,
			"Failed to request RIO irq (%d)\n",
			krio_priv->board_rio_cfg.rio_irq);

	res = request_irq(krio_priv->board_rio_cfg.lsu_irq,
			  lsu_interrupt_handler,
			  0,
			  "sRIO LSU",
			  krio_priv);
	if (res)
		dev_err(krio_priv->dev,
			"Failed to request LSU irq (%d)\n",
			krio_priv->board_rio_cfg.lsu_irq);
	res = request_threaded_irq(krio_priv->board_rio_cfg.rio_starvation_irq,
			NULL, rio_starvation_interrupt_thread,
			IRQF_ONESHOT,
			"sRIO-starvation",
			krio_priv);
	if (res)
		dev_err(krio_priv->dev, "Failed to request RIO starvation irq (%d)\n",
				krio_priv->board_rio_cfg.rio_starvation_irq);
}

static void keystone_rio_interrupt_release(struct keystone_rio_data *krio_priv)
{
	free_irq(krio_priv->board_rio_cfg.rio_irq, krio_priv);
	free_irq(krio_priv->board_rio_cfg.lsu_irq, krio_priv);
	free_irq(krio_priv->board_rio_cfg.rio_starvation_irq, krio_priv);
}

/*---------------------------- Direct I/O -------------------------------*/

static u32 keystone_rio_dio_get_lsu_cc(u32 lsu_id, u8 ltid, u8 *lcb,
				       struct keystone_rio_data *krio_priv)
{
	u32 idx;
	u32 shift;
	u32 value;
	u32 cc;
	/* lSU shadow register status mapping */
	u32 lsu_index[8] = { 0, 9, 15, 20, 24, 33, 39, 44 };

	/* Compute LSU stat index from LSU id and LTID */
	idx   = (lsu_index[lsu_id] + ltid) >> 3;
	shift = ((lsu_index[lsu_id] + ltid) & 0x7) << 2;

	/* Get completion code and context */
	value  = __raw_readl(&(krio_priv->regs->lsu_stat_reg[idx]));
	cc     = (value >> (shift + 1)) & 0x7;
	*lcb   = (value >> shift) & 0x1;

	return cc;
}

static u32 keystone_rio_dio_packet_type(int dio_mode)
{
	switch (dio_mode) {
	case RIO_DIO_MODE_READ:
		return KEYSTONE_RIO_PACKET_TYPE_NREAD;
	case RIO_DIO_MODE_WRITER:
		return KEYSTONE_RIO_PACKET_TYPE_NWRITE_R;
	case RIO_DIO_MODE_WRITE:
		return KEYSTONE_RIO_PACKET_TYPE_NWRITE;
	case RIO_DIO_MODE_SWRITE:
		return KEYSTONE_RIO_PACKET_TYPE_SWRITE;
	}
	return KEYSTONE_RIO_PACKET_TYPE_NREAD;
}

static const char * keystone_rio_lsu_status_str(u32 status)
{
	static const char * const lsu_cc_stat[] = {
		"transaction complete, No Errors",
		"transaction Timeout occurred on Non-posted transaction",
		"transaction complete, Packet not sent due to flow control blockade",
		"non-posted response packet (type 8 and 13) contained ERROR status",
		"packet not sent due to unsupported transaction type",
		"DMA data transfer error",
		"Retry DOORBELL response received",
		"no packets sent as the transaction was killed using the CBUSY bit",
	};
	return lsu_cc_stat[status & KEYSTONE_RIO_LSU_CC_MASK];
}

/*
 * DIO transfer using LSU directly
 */
static inline int keystone_rio_dio_raw_transfer(int port_id,
						u16 dest_id,
						dma_addr_t src_addr,
						u32 tgt_addr,
						int size_bytes,
						int size,
						int dio_mode,
						struct keystone_rio_data *krio_priv)
{
	unsigned int count;
	unsigned int status = 0;
	unsigned int res = 0;
	unsigned int retry_count = KEYSTONE_RIO_RETRY_CNT;
	u8           context;
	u8           ltid;
	u8           lsu = krio_priv->lsu_dio;

	size_bytes &= (KEYSTONE_RIO_MAX_DIO_PKT_SIZE - 1);

retry_transfer:

	mutex_lock(&krio_priv->lsu_lock);

	reinit_completion(&krio_priv->lsu_completion);

	/* Check if there is space in the LSU shadow reg and that it is free */
	count = 0;
	while(1)
        {
		status = __raw_readl(&(krio_priv->regs->lsu_reg[lsu].busy_full));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_err(krio_priv->dev, "DIO: cannot get a free LSU\n");
			res = -EIO;
			goto out;
		}
		ndelay(1000);
        }

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of destination */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	__raw_writel(tgt_addr, &(krio_priv->regs->lsu_reg[lsu].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	__raw_writel(src_addr, &(krio_priv->regs->lsu_reg[lsu].phys_addr));

	/* LSU Reg 3 - Byte count */
	__raw_writel(size_bytes,
		     &(krio_priv->regs->lsu_reg[lsu].dbell_val_byte_cnt));

	/* LSU Reg 4 -
	 * out port ID = rio.port
         * priority = LSU_PRIO
	 * XAM = 0
	 * ID size = 8 or 16 bit
	 * Dest ID specified as arg
	 * interrupt request = 1 */
	__raw_writel(((port_id << 8)
		      | (KEYSTONE_RIO_LSU_PRIO << 4)
		      | (size ? (1 << 10) : 0)
		      | ((u32) dest_id << 16)
		      | 1),
		     &(krio_priv->regs->lsu_reg[lsu].destid));

	/* LSU Reg 5 -
	 * doorbell info = 0 for this packet type
	 * hop count = 0 for this packet type
	 * Writing this register should initiate the transfer */
	__raw_writel(keystone_rio_dio_packet_type(dio_mode),
		     &(krio_priv->regs->lsu_reg[lsu].dbell_info_fttype));

        /* Wait for transfer to complete */
	res = wait_for_completion_timeout(&krio_priv->lsu_completion,
					  msecs_to_jiffies(KEYSTONE_RIO_TIMEOUT_MSEC));
	if (res == 0) {
		/* Timeout expired */
		res = -EIO;
		dev_warn_ratelimited(krio_priv->dev, "DIO not completed timeout expired irq miss\n");
		goto out;
	}

	dev_dbg(krio_priv->dev, "DIO: LSU interrupt received\n");

	/* Retrieve our completion code */
   	count = 0;
	res   = 0;
	while(1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(lsu, ltid, &lcb, krio_priv);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			dev_warn_ratelimited(krio_priv->dev, "DIO timeout retrieve CC\n");
			break;
		}
		ndelay(1000);
	}
out:
	mutex_unlock(&krio_priv->lsu_lock);

	if (res) {
		dev_err(krio_priv->dev, "DIO: LSU error = %d, dest_id=0x%x\n", res, dest_id);
		return res;
	}

	dev_dbg(krio_priv->dev, "DIO: status = 0x%x\n", status);

	switch (status & KEYSTONE_RIO_LSU_CC_MASK) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		/* LSU Reg 6 - Flush this transaction */
		__raw_writel(1, &(krio_priv->regs->lsu_reg[lsu].busy_full));
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EAGAIN;
		break;
	default:
		break;
	}

	/*
	 * Try to transfer again in case of retry doorbell receive
	 * or canceled LSU transfer.
	 */
	if ((res == -EAGAIN) && (retry_count-- > 0)) {
		ndelay(1000);
		goto retry_transfer;
	}

	if (status & KEYSTONE_RIO_LSU_CC_MASK)
		dev_warn_ratelimited(krio_priv->dev, "DIO %s dest_id=0x%x\n",
				keystone_rio_lsu_status_str(status), dest_id);

	return res;
}

/**
 * keystone_rio_dio_transfer - Transfer bytes data from/to physical address
 * to device ID's global address.
 * @mport: RapidIO master port info
 * @index: ID of the RapidIO interface
 * @dest_id: destination device id
 * @src_addr: source (host) address
 * @tgt_addr: target global address
 * @size_bytes: size in bytes
 * @dio_mode: DIO transfer mode (write, write_r, swrite , read)
 *
 * Return %0 on success. Return %-EIO, %-EBUSY or %-EAGAIN on failure.
 */
static int keystone_rio_dio_transfer(struct rio_mport *mport,
				     int index,
				     u16 dest_id,
				     u32 src_addr,
				     u32 tgt_addr,
				     int size_bytes,
				     int dio_mode)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct device *dev = ((struct keystone_rio_data *)(mport->priv))->dev;
	int count  = size_bytes;
	int length;
	int res    = 0;
	u32 s_addr = src_addr;
	u32 t_addr = tgt_addr;
	dma_addr_t dma;
	enum dma_data_direction dir;

	/* SWRITE implies double-word alignment for address and size */
	if ((dio_mode == RIO_DIO_MODE_SWRITE) &&
	    ((size_bytes % 8) || (tgt_addr % 8) || (src_addr % 8)))
		return -EINVAL;

	dev_dbg(krio_priv->dev,
		"DIO: transferring chunk addr = 0x%x, size = %d, tgt_addr=%08x, mode=%d\n",
		src_addr, size_bytes, tgt_addr, dio_mode);

	if (dio_mode == RIO_DIO_MODE_READ)
		dir = DMA_FROM_DEVICE;
	else
		dir = DMA_TO_DEVICE;

	/* Transfer packet by packet */
	while(count) {
		length = (count <= KEYSTONE_RIO_MAX_DIO_PKT_SIZE) ?
			count : KEYSTONE_RIO_MAX_DIO_PKT_SIZE;

		dma = dma_map_single(dev, (void *)s_addr, length, dir);

		res = keystone_rio_dio_raw_transfer(mport->index,
						    dest_id,
						    dma,
						    t_addr,
						    length,
						    mport->sys_size,
						    dio_mode,
						    krio_priv);

		dma_unmap_single(dev, dma, length, dir);

		if (res)
			break;

		s_addr += length;
		t_addr += length;

		count -= length;
	}
	return res;
}

/*------------------------------ Doorbell management --------------------------*/

static inline int dbell_get(u32* pending)
{
	if (*pending) {
		int n = __ffs(*pending);
		*pending &= ~(1 << n);
		return n;
	} else
		return -1;
}

static inline void dbell_call_handler(u32 dbell_num, struct keystone_rio_data *krio_priv)
{
	struct rio_dbell *dbell;
	int i;
	int found = 0;

	for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i ++) {
		if (krio_priv->mport[i]) {
			struct rio_mport *mport = krio_priv->mport[i];
			list_for_each_entry(dbell, &mport->dbells, node) {
				if ((dbell->res->start <= dbell_num) &&
				    (dbell->res->end   >= dbell_num)) {
					found = 1;
					break;
				}
			}

			if (found && dbell->dinb) {
				/* Call the registered handler if any */
				dbell->dinb(mport,
					    dbell->dev_id,
					    -1, /* we don't know the source Id */
					    mport->host_deviceid,
					    dbell_num);
				break;
			}
		}
	}

	if (!found) {
		dev_dbg(krio_priv->dev, "DBELL: received spurious doorbell %d\n", dbell_num);
	}
}

static void dbell_handler(struct keystone_rio_data *krio_priv)
{
	u32 pending_dbell;
	unsigned int i;

	for (i = 2; i < KEYSTONE_RIO_DBELL_NUMBER; i++) {
		pending_dbell =  __raw_readl(&(krio_priv->regs->doorbell_int[i].status));

		if (pending_dbell)
			/* Acknowledge the interrupts for these doorbells */
			__raw_writel(pending_dbell,
				     &(krio_priv->regs->doorbell_int[i].clear));

		while (pending_dbell) {
			u32 dbell_num = dbell_get(&pending_dbell) + (i << 4);

			/* Call the registered dbell handler(s) */
			dbell_call_handler(dbell_num, krio_priv);
		}
	}
}

/**
 * keystone_rio_doorbell_send - Send a KeyStone doorbell message
 * @mport: RapidIO master port info
 * @index: ID of the RapidIO interface
 * @destid: device ID of target device
 * @num: doorbell number
 *
 * Sends a KeyStone doorbell message. Returns %0 on success or
 * %-EINVAL, %-EIO, %-EBUSY or %-EAGAIN on failure.
 */
static int keystone_rio_dbell_send(struct rio_mport *mport,
				   int index,
				   u16 dest_id,
				   u16 num)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	unsigned int count;
	unsigned int status = 0;
	unsigned int res    = 0;
	/* Transform doorbell number into info field */
	u16          info   = (num & 0xf) | (((num >> 4) & 0x3) << 5);
	u8           port_id = mport->index;
	u8           context;
	u8           ltid;
	u8           lsu = krio_priv->lsu_dio;

	dev_dbg(krio_priv->dev, "DBELL: sending doorbell (info = %d) to %x\n",
		info & KEYSTONE_RIO_DBELL_MASK, dest_id);

	mutex_lock(&krio_priv->lsu_lock);

	reinit_completion(&krio_priv->lsu_completion);

	/* Check is there is space in the LSU shadow reg and that it is free */
	count = 0;
	while(1)
        {
		status = __raw_readl(&(krio_priv->regs->lsu_reg[lsu].busy_full));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			goto out;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
        }

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of destination */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].phys_addr));

	/* LSU Reg 3 - byte count */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].dbell_val_byte_cnt));

	/* LSU Reg 4 - */
	__raw_writel(((port_id << 8)
		      | (KEYSTONE_RIO_LSU_PRIO << 4)
		      | ((mport->sys_size) ? (1 << 10) : 0)
		      | ((u32) dest_id << 16)
		      | 1),
		     &(krio_priv->regs->lsu_reg[lsu].destid));

	/* LSU Reg 5
	 * doorbell info = info
	 * hop count = 0
	 * Packet type = 0xa0 ftype = 10, ttype = 0 */
	__raw_writel(((info & 0xffff) << 16) | (KEYSTONE_RIO_PACKET_TYPE_DBELL & 0xff),
		     &(krio_priv->regs->lsu_reg[lsu].dbell_info_fttype));

        /* Wait for transfer to complete */
	res = wait_for_completion_timeout(&krio_priv->lsu_completion,
					  msecs_to_jiffies(KEYSTONE_RIO_TIMEOUT_MSEC));
	if (res == 0) {
		/* Timeout expired */
		res = -EIO;
		dev_warn_ratelimited(krio_priv->dev, "DBELL: LSU interrupt timeout expired\n");
		goto out;
	}

	dev_dbg(krio_priv->dev, "DBELL: LSU interrupt received\n");

	/* Retrieve our completion code */
   	count = 0;
	res   = 0;
	while(1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(lsu, ltid, &lcb, krio_priv);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			res = -EIO;
			break;
		}
		ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
	}
out:
	mutex_unlock(&krio_priv->lsu_lock);

	if (res) {
		dev_dbg(krio_priv->dev, "DBELL: LSU error = %d\n", res);
		return res;
	}

	dev_dbg(krio_priv->dev, "DBELL: status = 0x%x\n", status);

	switch (status & KEYSTONE_RIO_LSU_CC_MASK) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		res = -EIO;
		/* LSU Reg 6 - Flush this transaction */
		__raw_writel(1, &(krio_priv->regs->lsu_reg[lsu].busy_full));
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		res = -EAGAIN;
		break;
	default:
		dev_dbg(krio_priv->dev, "DBELL: doorbell sent\n");
		break;
	}

	if (status & KEYSTONE_RIO_LSU_CC_MASK)
		dev_warn_ratelimited(krio_priv->dev, "Doorbell CC %s dest_id=0x%x\n",
				keystone_rio_lsu_status_str(status), dest_id);

	return res;
}

/*---------------------- Maintenance Request Management  ---------------------*/

/**
 * keystone_rio_maint_request - Perform a maintenance request
 * @port_id: output port ID
 * @dest_id: destination ID of target device
 * @hopcount: hopcount for this request
 * @offset: offset in the RapidIO configuration space
 * @buff: dma address of the data on the host
 * @buff_len: length of the data
 * @size: 1 for 16bit, 0 for 8bit ID size
 * @type: packet type
 *
 * Returns %0 on success or %-EINVAL, %-EIO, %-EAGAIN or %-EBUSY on failure.
 */
static inline int keystone_rio_maint_request(int port_id,
					     u32 dest_id,
					     u8  hopcount,
					     u32 offset,
					     dma_addr_t buff,
					     int buff_len,
					     u16 size,
					     u16 type,
					     struct keystone_rio_data *krio_priv)
{
	unsigned int count;
	unsigned int status = 0;
	unsigned int res    = 0;
	u8           context;
	u8           ltid;
	u8           lsu = krio_priv->lsu_maint;

	mutex_lock(&krio_priv->lsu_lock);

	/* Check is there is space in the LSU shadow reg and that it is free */
	count = 0;
	while (1) {
		status = __raw_readl(&(krio_priv->regs->lsu_reg[lsu].busy_full));
		if (((status & KEYSTONE_RIO_LSU_FULL_MASK) == 0x0)
		    && ((status & KEYSTONE_RIO_LSU_BUSY_MASK) == 0x0))
			break;
		count++;

		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_warn_ratelimited(krio_priv->dev,
				"no LSU available, status = 0x%x\n", status);
			res = -EIO;
			goto out;
		}
		ndelay(1000);
	}

	/* Get LCB and LTID, LSU reg 6 is already read */
	context = (status >> 4) & 0x1;
	ltid    = status & 0xf;

	/* LSU Reg 0 - MSB of RapidIO address */
	__raw_writel(0, &(krio_priv->regs->lsu_reg[lsu].addr_msb));

	/* LSU Reg 1 - LSB of destination */
	__raw_writel(offset, &(krio_priv->regs->lsu_reg[lsu].addr_lsb_cfg_ofs));

	/* LSU Reg 2 - source address */
	__raw_writel(buff, &(krio_priv->regs->lsu_reg[lsu].phys_addr));

	/* LSU Reg 3 - byte count */
	__raw_writel(buff_len, &(krio_priv->regs->lsu_reg[lsu].dbell_val_byte_cnt));

	/* LSU Reg 4 - */
	__raw_writel(((port_id << 8)
		      | (KEYSTONE_RIO_LSU_PRIO << 4)
		      | (size ? (1 << 10) : 0)
		      | ((u32) dest_id << 16)),
		     &(krio_priv->regs->lsu_reg[lsu].destid));

	/* LSU Reg 5 */
	__raw_writel(((hopcount & 0xff) << 8) | (type & 0xff),
		     &(krio_priv->regs->lsu_reg[lsu].dbell_info_fttype));

	/* Retrieve our completion code */
	count = 0;
	res   = 0;
	while (1) {
		u8 lcb;
		status = keystone_rio_dio_get_lsu_cc(lsu, ltid, &lcb, krio_priv);
		if (lcb == context)
			break;
		count++;
		if (count >= KEYSTONE_RIO_TIMEOUT_CNT) {
			dev_dbg(krio_priv->dev,
				"timeout %d, ltid = %d, context = %d, "
				"lcb = %d, cc = %d\n",
				count, ltid, context, lcb, status);
			res = -EIO;
			break;
		}
		ndelay(1000);
	}
out:
	mutex_unlock(&krio_priv->lsu_lock);

	if (res)
		return res;

	if (status & KEYSTONE_RIO_LSU_CC_MASK)
		dev_warn_ratelimited(krio_priv->dev, "Maint CC %s dest_id=0x%x\n",
				keystone_rio_lsu_status_str(status), dest_id);

	switch (status) {
	case KEYSTONE_RIO_LSU_CC_TIMEOUT:
	case KEYSTONE_RIO_LSU_CC_XOFF:
	case KEYSTONE_RIO_LSU_CC_ERROR:
	case KEYSTONE_RIO_LSU_CC_INVALID:
	case KEYSTONE_RIO_LSU_CC_DMA:
		return -EIO;
		break;
	case KEYSTONE_RIO_LSU_CC_RETRY:
		return -EBUSY;
		break;
	case KEYSTONE_RIO_LSU_CC_CANCELED:
		return -EAGAIN;
		break;
	default:
		break;
	}
	return 0;
}

static int keystone_rio_maint_read(struct keystone_rio_data *krio_priv,
				   int port_id,
				   u16 destid,
				   u16 size,
				   u8  hopcount,
				   u32 offset,
				   int len,
				   u32 *val)
{
	u32 *tbuf;
	int res;
	dma_addr_t dma;
	struct device *dev = krio_priv->dev;
	size_t align_len = L1_CACHE_ALIGN(len);

	tbuf = kzalloc(align_len, GFP_KERNEL | GFP_DMA);
	if (!tbuf)
		return -ENOMEM;

	dma = dma_map_single(dev, tbuf, len, DMA_FROM_DEVICE);

	res = keystone_rio_maint_request(port_id, destid, hopcount, offset, dma,
					 len, size, KEYSTONE_RIO_PACKET_TYPE_MAINT_R,
					 krio_priv);

	dma_unmap_single(dev, dma, len, DMA_FROM_DEVICE);

	/* Taking care of byteswap */
	switch (len) {
	case 1:
		*val = *((u8*)tbuf);
		break;
	case 2:
		*val = ntohs(*((u16*)tbuf));
		break;
	default:
		*val = ntohl(*((u32*)tbuf));
		break;
	}

	dev_dbg(dev,
		"maint_r: index %d destid 0x%04x hopcount %d offset 0x%x "
		"len %d val 0x%x res %d\n",
		port_id, destid, hopcount, offset, len, *val, res);

	kfree(tbuf);

	return res;
}

static int keystone_rio_maint_write(struct keystone_rio_data *krio_priv,
				    int port_id,
				    u16 destid,
				    u16 size,
				    u8  hopcount,
				    u32 offset,
				    int len,
				    u32 val)
{
	u32 *tbuf;
	int res;
	dma_addr_t dma;
	struct device *dev = krio_priv->dev;
	size_t align_len = L1_CACHE_ALIGN(len);

	tbuf = kzalloc(align_len, GFP_KERNEL | GFP_DMA);
	if (!tbuf)
		return -ENOMEM;

	/* Taking care of byteswap */
	switch (len) {
	case 1:
		*tbuf = ((u8) val);
		break;
	case 2:
		*tbuf = htons((u16) val);
		break;
	default:
		*tbuf = htonl((u32) val);
		break;
	}

	dma = dma_map_single(dev, tbuf, len, DMA_TO_DEVICE);

	res = keystone_rio_maint_request(port_id, destid, hopcount, offset, dma,
					 len, size, KEYSTONE_RIO_PACKET_TYPE_MAINT_W,
					 krio_priv);

	dma_unmap_single(dev, dma, len, DMA_TO_DEVICE);

	dev_dbg(dev,
		"maint_w: index %d destid 0x%04x hopcount %d offset 0x%x "
		"len %d val 0x%x res %d\n",
		port_id, destid, hopcount, offset, len, val, res);

	kfree(tbuf);

	return res;
}

/*------------------------- RapidIO hw controller setup ---------------------*/

/* Retrieve the corresponding lanes bitmask from ports bitmask and path_mode */
static int keystone_rio_get_lane_config(u32 ports, u32 path_mode)
{
	u32 lanes = 0;

	while (ports) {
		u32 lane;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		if (keystone_lane_configs[path_mode][port].start == -1)
			return -1;

		for (lane = keystone_lane_configs[path_mode][port].start;
		     lane < keystone_lane_configs[path_mode][port].end;
		     lane ++) {
			lanes |= (1 << lane);
		}
	}
	return (int) lanes;
}

static void keystone_rio_blocks_enable(u32 lanes, struct keystone_rio_data *krio_priv)
{
	u32 block;
	u32 val;
	int i;

	/* Enable blocks (only used port blocks are enabled) */
	__raw_writel(1, &krio_priv->regs->gbl_en);
	if (1 == krio_priv->isolate) {
		__raw_writel(1, &krio_priv->regs->blk[KEYSTONE_RIO_BLK0_MMRS].enable);
		__raw_writel(1, &krio_priv->regs->blk[KEYSTONE_RIO_BLK5_PORT0].enable);
		__raw_writel(1, &krio_priv->regs->blk[KEYSTONE_RIO_BLK6_PORT1].enable);
		__raw_writel(1, &krio_priv->regs->blk[KEYSTONE_RIO_BLK7_PORT2].enable);
		__raw_writel(1, &krio_priv->regs->blk[KEYSTONE_RIO_BLK8_PORT3].enable);
		__raw_writel(1, &krio_priv->regs->blk[KEYSTONE_RIO_BLK9].enable);
		__raw_writel(1, &krio_priv->regs->blk[KEYSTONE_RIO_BLK2_MAU].enable);
	} else {
		for (block = 0; block < KEYSTONE_RIO_BLK_NUM; block++) {
			if (block >= KEYSTONE_RIO_BLK5_PORT0 &&
				block <= KEYSTONE_RIO_BLK8_PORT3) {
				if (IS_SERDES_LANE_USED(lanes, block - KEYSTONE_RIO_BLK5_PORT0)) {
					__raw_writel(1, &krio_priv->regs->blk[block].enable);
					dev_dbg(krio_priv->dev, "enable srio block %d\n", block);
				}
			} else {
				__raw_writel(1, &krio_priv->regs->blk[block].enable);
				dev_dbg(krio_priv->dev, "enable srio block %d\n", block);
			}
		}
	}
	/* Wait for enabled status */
	i = 0;
	do {
		if (++i > 5) {
			dev_warn_ratelimited(krio_priv->dev, "Failed to enable SRIO blocks\n");
			break;
		}

		val = __raw_readl(&krio_priv->regs->gbl_en_stat);
		if (val == (0x043f | (lanes << (KEYSTONE_RIO_BLK5_PORT0 + 1))))
			break;

		udelay(1);
	} while (1);
}

static void keystone_rio_blocks_disable(struct keystone_rio_data *krio_priv)
{
	u32 block;
	u32 val;
	int i;

	/* Disable blocks */
	__raw_writel(0, &krio_priv->regs->gbl_en);
	for (block = 0; block < KEYSTONE_RIO_BLK_NUM; block++)
		__raw_writel(0, &krio_priv->regs->blk[block].enable);

	/* Wait for disabled status */
	i = 0;
	do {
		if (++i > 5) {
			dev_warn_ratelimited(krio_priv->dev, "Failed to disable SRIO blocks\n");
			break;
		}

		udelay(1);
		val = __raw_readl(&krio_priv->regs->gbl_en_stat);
	} while (val != 0);
}

static void keystone_rio_serdes_init(struct keystone_rio_data *krio_priv)
{
	struct keystone_serdes_config *serdes_config =
		&krio_priv->board_rio_cfg.serdes_config;
	u32 port;
	u32 val;

	/* K1 SerDes main configuration */
	__raw_writel(serdes_config->serdes_cfg_pll,
			 &krio_priv->serdes_regs->pll);

	/* Per-port SerDes configuration */
	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		__raw_writel(serdes_config->rx_chan_config[port],
				 &krio_priv->serdes_regs->channel[port].rx);
		__raw_writel(serdes_config->tx_chan_config[port],
				 &krio_priv->serdes_regs->channel[port].tx);
	}

	/* Check for RIO SerDes PLL lock */
	do {
		val = __raw_readl(krio_priv->serdes_sts_reg);
	} while (!(val & 0x1));
}

static void k2_rio_serdes_init_3g(u32 lanes, struct keystone_rio_data *krio_priv)
{
	void __iomem *regs = (void __iomem *) krio_priv->k2_serdes_regs;

	/* Uses Half Rate configuration */
	reg_rmw(regs + 0x000, 0x00000000, 0xff000000);
	reg_rmw(regs + 0x014, 0x00008282, 0x0000ffff);
	reg_rmw(regs + 0x060, 0x00132c48, 0x00ffffff);
	reg_rmw(regs + 0x064, 0x00c3c700, 0x00ffff00);
	reg_rmw(regs + 0x078, 0x0000c000, 0x0000ff00);

	if (krio_priv->serdes_values) {
		int i = 0;

		for (i = 0; i < krio_priv->serdes_val_len; i++)
			reg_rmw(regs + krio_priv->serdes_values[i].offset,
				krio_priv->serdes_values[i].value,
				krio_priv->serdes_values[i].bit_mask);
		goto done;

	}

	if (IS_SERDES_LANE_USED(lanes, 0)) {
		dev_dbg(krio_priv->dev, "setting lane 0 SerDes to 3GHz\n");
		reg_rmw(regs + 0x204, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x208, 0x00000024, 0x000000ff);
		reg_rmw(regs + 0x20c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x210, 0x1b000000, 0xff000000);
		reg_rmw(regs + 0x214, 0x00006e7c, 0x0000ffff);
		reg_rmw(regs + 0x218, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x22c, 0x00100800, 0x00ffff00);
		reg_rmw(regs + 0x280, 0x00700070, 0x00ff00ff);
		reg_rmw(regs + 0x284, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x28c, 0x00003b00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 1)) {
		dev_dbg(krio_priv->dev, "setting lane 1 SerDes to 3GHz\n");
		reg_rmw(regs + 0x404, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x408, 0x00000024, 0x000000ff);
		reg_rmw(regs + 0x40c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x410, 0x1b000000, 0xff000000);
		reg_rmw(regs + 0x414, 0x00006e7c, 0x0000ffff);
		reg_rmw(regs + 0x418, 0x758000e4, 0xffff0000);
		reg_rmw(regs + 0x42c, 0x00100800, 0x00ffff00);
		reg_rmw(regs + 0x480, 0x00700070, 0x00ff00ff);
		reg_rmw(regs + 0x484, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x48c, 0x00003b00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 2)) {
		dev_dbg(krio_priv->dev, "setting lane 2 SerDes to 3GHz\n");
		reg_rmw(regs + 0x604, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x608, 0x00000024, 0x000000ff);
		reg_rmw(regs + 0x60c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x610, 0x1b000000, 0xff000000);
		reg_rmw(regs + 0x614, 0x00006e7c, 0x0000ffff);
		reg_rmw(regs + 0x618, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x62c, 0x00100800, 0x00ffff00);
		reg_rmw(regs + 0x680, 0x00700070, 0x00ff00ff);
		reg_rmw(regs + 0x684, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x68c, 0x00003b00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 3)) {
		dev_dbg(krio_priv->dev, "setting lane 3 SerDes to 3GHz\n");
		reg_rmw(regs + 0x804, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x808, 0x00000024, 0x000000ff);
		reg_rmw(regs + 0x80c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x810, 0x1b000000, 0xff000000);
		reg_rmw(regs + 0x814, 0x00006e7c, 0x0000ffff);
		reg_rmw(regs + 0x818, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x82c, 0x00100800, 0x00ffff00);
		reg_rmw(regs + 0x880, 0x00700070, 0x00ff00ff);
		reg_rmw(regs + 0x884, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x88c, 0x00003b00, 0x0000ff00);
	}

done:
	reg_rmw(regs + 0xa00, 0x00000800, 0x0000ff00);
	reg_rmw(regs + 0xa08, 0x37720000, 0xffff0000);
	reg_rmw(regs + 0xa30, 0x00777700, 0x00ffff00);
	reg_rmw(regs + 0xa84, 0x00000600, 0x0000ff00);
	reg_rmw(regs + 0xa94, 0x10000000, 0xff000000);
	reg_rmw(regs + 0xaa0, 0x81000000, 0xff000000);
	reg_rmw(regs + 0xabc, 0xff000000, 0xff000000);
	reg_rmw(regs + 0xac0, 0x0000008b, 0x000000ff);

	reg_rmw(regs + 0x000, 0x00000003, 0x000000ff);
	reg_rmw(regs + 0xa00, 0x0000005f, 0x000000ff);
}

static void k2_rio_serdes_init_5g(u32 lanes, struct keystone_rio_data *krio_priv)
{
	void __iomem *regs = (void __iomem *) krio_priv->k2_serdes_regs;

	/* Uses Full Rate configuration by default */
	reg_rmw(regs + 0x000, 0x00000000, 0xff000000);
	reg_rmw(regs + 0x014, 0x00008282, 0x0000ffff);
	reg_rmw(regs + 0x060, 0x00142438, 0x00ffffff);
	reg_rmw(regs + 0x064, 0x00c3c700, 0x00ffff00);
	reg_rmw(regs + 0x078, 0x0000c000, 0x0000ff00);

	if (krio_priv->serdes_values) {
		int i = 0;

		for (i = 0; i < krio_priv->serdes_val_len; i++)
			reg_rmw(regs + krio_priv->serdes_values[i].offset,
				krio_priv->serdes_values[i].value,
				krio_priv->serdes_values[i].bit_mask);
		goto done;

	}

	if (IS_SERDES_LANE_USED(lanes, 0)) {
		dev_dbg(krio_priv->dev, "setting lane 0 SerDes to 5GHz\n");
		reg_rmw(regs + 0x204, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x208, 0x00000026, 0x000000ff);
		reg_rmw(regs + 0x20c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x214, 0x00006f38, 0x0000ffff);
		reg_rmw(regs + 0x218, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x22c, 0x00200800, 0x00ffff00);
		reg_rmw(regs + 0x280, 0x00860086, 0x00ff00ff);
		reg_rmw(regs + 0x284, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x28c, 0x00002c00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 1)) {
		dev_dbg(krio_priv->dev, "setting lane 1 SerDes to 5GHz\n");
		reg_rmw(regs + 0x404, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x408, 0x00000026, 0x000000ff);
		reg_rmw(regs + 0x40c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x414, 0x00006f38, 0x0000ffff);
		reg_rmw(regs + 0x418, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x42c, 0x00200800, 0x00ffff00);
		reg_rmw(regs + 0x480, 0x00860086, 0x00ff00ff);
		reg_rmw(regs + 0x484, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x48c, 0x00002c00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 2)) {
		dev_dbg(krio_priv->dev, "setting lane 2 SerDes to 5GHz\n");
		reg_rmw(regs + 0x604, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x608, 0x00000026, 0x000000ff);
		reg_rmw(regs + 0x60c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x614, 0x00006f38, 0x0000ffff);
		reg_rmw(regs + 0x618, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x62c, 0x00200800, 0x00ffff00);
		reg_rmw(regs + 0x680, 0x00860086, 0x00ff00ff);
		reg_rmw(regs + 0x684, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x68c, 0x00002c00, 0x0000ff00);
	}

	if (IS_SERDES_LANE_USED(lanes, 3)) {
		dev_dbg(krio_priv->dev, "setting lane 3 SerDes to 5GHz\n");
		reg_rmw(regs + 0x804, 0x78000080, 0xff0000ff);
		reg_rmw(regs + 0x808, 0x00000026, 0x000000ff);
		reg_rmw(regs + 0x80c, 0x02000000, 0xff000000);
		reg_rmw(regs + 0x814, 0x00006f38, 0x0000ffff);
		reg_rmw(regs + 0x818, 0x758000e4, 0xffff00ff);
		reg_rmw(regs + 0x82c, 0x00200800, 0x00ffff00);
		reg_rmw(regs + 0x880, 0x00860086, 0x00ff00ff);
		reg_rmw(regs + 0x884, 0x1d0f0085, 0xffff00ff);
		reg_rmw(regs + 0x88c, 0x00002c00, 0x0000ff00);
	}

done:
	reg_rmw(regs + 0xa00, 0x00008000, 0x0000ff00);
	reg_rmw(regs + 0xa08, 0x38d20000, 0xffff0000);
	reg_rmw(regs + 0xa30, 0x008d8d00, 0x00ffff00);
	reg_rmw(regs + 0xa84, 0x00000200, 0x0000ff00);
	reg_rmw(regs + 0xa94, 0x10000000, 0xff000000);
	reg_rmw(regs + 0xaa0, 0x81000000, 0xff000000);
	reg_rmw(regs + 0xabc, 0xff000000, 0xff000000);
	reg_rmw(regs + 0xac0, 0x0000008b, 0x000000ff);
	reg_rmw(regs + 0x000, 0x00000003, 0x000000ff);
	reg_rmw(regs + 0xa00, 0x0000005f, 0x000000ff);

	reg_rmw(regs + 0xa48, 0x00fd8c00, 0x00ffff00);
	reg_rmw(regs + 0xa54, 0x002fec72, 0x00ffffff);
	reg_rmw(regs + 0xa58, 0x00f92100, 0xffffff00);
	reg_rmw(regs + 0xa5c, 0x00040060, 0xffffffff);
	reg_rmw(regs + 0xa60, 0x00008000, 0xffffffff);
	reg_rmw(regs + 0xa64, 0x0c581220, 0xffffffff);
	reg_rmw(regs + 0xa68, 0xe13b0602, 0xffffffff);
	reg_rmw(regs + 0xa6c, 0xb8074cc1, 0xffffffff);
	reg_rmw(regs + 0xa70, 0x3f02e989, 0xffffffff);
	reg_rmw(regs + 0xa74, 0x00000001, 0x000000ff);
	reg_rmw(regs + 0xb20, 0x00370000, 0x00ff0000);
	reg_rmw(regs + 0xb1c, 0x37000000, 0xff000000);
	reg_rmw(regs + 0xb20, 0x0000005d, 0x000000ff);
}

static void k2_rio_serdes_lane_enable(u32 lane, u32 rate, struct keystone_rio_data *krio_priv)
{
	struct keystone2_srio_serdes_regs *serdes_regs =
		krio_priv->k2_serdes_regs;
	u32 val;

	/* Bring this lane out of reset by clearing override bit 29 */
	val = __raw_readl(&serdes_regs->lane[lane].__lane_rsvd0[8]);
	val &= ~BIT(29);
	__raw_writel(val, &serdes_regs->lane[lane].__lane_rsvd0[8]);

	dev_dbg(krio_priv->dev, "enable serdes line 0x%x\n", lane);
	/* Set Lane Control Rate */
	switch (rate) {
	case KEYSTONE_RIO_FULL_RATE:
		__raw_writel(0xF0C0F0F0, &serdes_regs->wiz_lane[lane].ctl_sts);
		break;
	case KEYSTONE_RIO_HALF_RATE:
		__raw_writel(0xF4C0F4F0, &serdes_regs->wiz_lane[lane].ctl_sts);
		break;
	case KEYSTONE_RIO_QUARTER_RATE:
		__raw_writel(0xF8C0F8F0, &serdes_regs->wiz_lane[lane].ctl_sts);
		break;
	default:
		return;
	}
}
static void k2_rio_serdes_lane_disable(u32 lane, struct keystone_rio_data *krio_priv)
{
	struct keystone2_srio_serdes_regs *serdes_regs =
		krio_priv->k2_serdes_regs;
	u32 val;

	dev_dbg(krio_priv->dev, "disable serdes line 0x%x\n", lane);
	val = __raw_readl(&serdes_regs->wiz_lane[lane].ctl_sts);
	val &= ~(BIT(29) | BIT(30) | BIT(13) | BIT(14));
	__raw_writel(val, &serdes_regs->wiz_lane[lane].ctl_sts);
}

static void keystone_rio_serdes_lane_disable(u32 lane, struct keystone_rio_data *krio_priv)
{
	if (!K2_SERDES(krio_priv)) {
		__raw_writel(0, &krio_priv->serdes_regs->channel[lane].rx);
		__raw_writel(0, &krio_priv->serdes_regs->channel[lane].tx);
	} else {
		k2_rio_serdes_lane_disable(lane, krio_priv);
	}
}

static int k2_rio_serdes_config(u32 lanes, u32 baud, struct keystone_rio_data *krio_priv)
{
	struct keystone2_srio_serdes_regs *serdes_regs =
		krio_priv->k2_serdes_regs;
	u32 rate;
	u32 val;
	u32 lane;

	/* Disable pll before configuring the SerDes registers */
	__raw_writel(0x00000000, &serdes_regs->wiz_pll_ctrl);

	switch (baud) {
	case KEYSTONE_RIO_BAUD_1_250:
		rate = KEYSTONE_RIO_QUARTER_RATE;
		k2_rio_serdes_init_5g(lanes, krio_priv);
		break;
	case KEYSTONE_RIO_BAUD_2_500:
		rate = KEYSTONE_RIO_HALF_RATE;
		k2_rio_serdes_init_5g(lanes, krio_priv);
		break;
	case KEYSTONE_RIO_BAUD_5_000:
		rate = KEYSTONE_RIO_FULL_RATE;
		k2_rio_serdes_init_5g(lanes, krio_priv);
		break;
	case KEYSTONE_RIO_BAUD_3_125:
		rate = KEYSTONE_RIO_HALF_RATE;
		k2_rio_serdes_init_3g(lanes, krio_priv);
		break;
	default:
		dev_warn(krio_priv->dev, "unsupported baud rate %d\n", baud);
		return -EINVAL;
	}

	/* Disable serdes for all not requested lines */
	lane = 0;
	while(lane < KEYSTONE_MAX_SERDES_LINES) {
		if (!IS_SERDES_LANE_USED(lanes, lane)) {
			k2_rio_serdes_lane_disable(lane, krio_priv);
		}
		++lane;
	}
	lane  = 0;

	/* Enable serdes for requested lanes */
	while(lanes) {
		lane = __ffs(lanes);
		lanes &= ~(1 << lane);
		k2_rio_serdes_lane_enable(lane, rate, krio_priv);
	}

	/* Enable pll via the pll_ctrl */
	__raw_writel(0xe0000000, &serdes_regs->wiz_pll_ctrl);

	/* Wait until CMU_OK bit is set */
	do {
		val = __raw_readl(&serdes_regs->comlane_1F8);
	} while (!(val & BIT(16)));

	return 0;
}

static int k2_rio_serdes_wait_lock(struct keystone_rio_data *krio_priv, u32 lanes)
{
	u32 val;
	unsigned long timeout;
	struct keystone2_srio_serdes_regs *regs = krio_priv->k2_serdes_regs;
	u32 val_mask;

	val_mask = lanes | (lanes << 8);

	/* Wait for the SerDes PLL lock */
	timeout = jiffies + msecs_to_jiffies(K2_PLL_LOCK_TIMEOUT);
	while (1) {
		/* read PLL_CTRL */
		val = __raw_readl(&regs->wiz_pll_ctrl);
		if ((val & val_mask) == val_mask)
			break;
		if (time_after(jiffies, timeout))
			return -1;
		udelay(10);
	}
	return 0;
}

static int keystone_rio_serdes_config(u32 lanes, struct keystone_rio_data *krio_priv)
{
	int res = 0;

	if (!K2_SERDES(krio_priv)) {
		keystone_rio_serdes_init(krio_priv);
	} else {
		res = k2_rio_serdes_config(lanes,
			krio_priv->board_rio_cfg.serdes_baudrate, krio_priv);
	}

	return res;
}

static int k2_rio_serdes_shutdown(struct keystone_rio_data *krio_priv, u32 lanes)
{
	struct keystone2_srio_serdes_regs *regs = krio_priv->k2_serdes_regs;
	u32 val;

	/* Disable SerDes for all lanes */
	while(lanes) {
		u32 lane = __ffs(lanes);
		lanes &= ~(1 << lane);
		k2_rio_serdes_lane_disable(lane, krio_priv);
	}

	/* Disable pll */
	__raw_writel(0x00000000, &regs->wiz_pll_ctrl);

	/* Reset CMU PLL for all lanes */
	val = __raw_readl(&regs->__cmu0_rsvd1[1]);
	val |= BIT(28);
	__raw_writel(val, &regs->__cmu0_rsvd1[1]);

	return 0;
}

static void keystone_rio_serdes_shutdown(struct keystone_rio_data *krio_priv)
{
	if (!K2_SERDES(krio_priv)) {
		__raw_writel(0, &krio_priv->serdes_regs->pll);
	} else {
		k2_rio_serdes_shutdown(krio_priv, krio_priv->board_rio_cfg.lanes);
	}
}

/**
 * keystone_rio_hw_init - Configure a RapidIO controller
 * @baud: serdes baudrate
 *
 * Returns %0 on success or %-EINVAL or %-EIO on failure.
 */
static int keystone_rio_hw_init(u32 baud, struct keystone_rio_data *krio_priv)
{
	u32 val;
	u32 port;
	int res = 0;
	int i;
	u32 assembly_id = KEYSTONE_RIO_ID_TI;
	u32 assembly_info = KEYSTONE_RIO_EXT_FEAT_PTR;

	/* Reset blocks */
	keystone_rio_blocks_disable(krio_priv);

	/* Set sRIO out of reset */
	__raw_writel(KEYSTONE_RIO_PER_RESTORE | KEYSTONE_RIO_PER_FREE,
		&krio_priv->regs->pcr);

	/* Clear BOOT_COMPLETE bit (allowing write) */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl);

	/* Enable blocks */
	keystone_rio_blocks_enable(krio_priv->board_rio_cfg.lanes, krio_priv);

	/* Set control register 1 configuration */
	__raw_writel(0x00000000, &krio_priv->regs->per_set_cntl1);

	/* Set control register */
	__raw_writel(krio_priv->board_rio_cfg.serdes_config.cfg_cntl,
		&krio_priv->regs->per_set_cntl);

	/* SerDes main configuration */
	res = keystone_rio_serdes_config(krio_priv->board_rio_cfg.lanes, krio_priv);
	if (res < 0) {
		dev_err(krio_priv->dev, "initialization of SerDes failed\n");
		return res;
	}

	/* Set prescalar for ip_clk */
	__raw_writel(krio_priv->board_rio_cfg.serdes_config.prescalar_srv_clk,
		&krio_priv->link_regs->prescalar_srv_clk);

	/* Peripheral-specific configuration and capabilities */
	__raw_writel(KEYSTONE_RIO_DEV_ID_VAL,
		     &krio_priv->car_csr_regs->dev_id);
	__raw_writel(KEYSTONE_RIO_DEV_INFO_VAL,
		     &krio_priv->car_csr_regs->dev_info);
#ifdef CONFIG_RAPIDIO_MEM_MAP
#if defined(CONFIG_RAPIDIO_OVERRIDE_ASSY_VID) && defined(CONFIG_RAPIDIO_ASSY_VENDOR_ID)
	assembly_id = CONFIG_RAPIDIO_ASSY_VENDOR_ID & 0xffff;
#endif
	if (krio_priv->mem_map) {
		unsigned long paddr =
			krio_map_low(krio_priv->mem_map->block) >> 12;
		assembly_id |= paddr & 0xffff0000;
		assembly_info |= paddr << 16;
	}
#endif
	__raw_writel(assembly_id, &krio_priv->car_csr_regs->assembly_id);
	__raw_writel(assembly_info, &krio_priv->car_csr_regs->assembly_info);

	__raw_writel(krio_priv->board_rio_cfg.comp_tag,
		     &krio_priv->car_csr_regs->component_tag);

	krio_priv->rio_pe_feat = RIO_PEF_PROCESSOR
		| RIO_PEF_CTLS
		| KEYSTONE_RIO_PEF_FLOW_CONTROL
		| RIO_PEF_EXT_FEATURES
		| RIO_PEF_ADDR_34
		| RIO_PEF_STD_RT
		| RIO_PEF_INB_DOORBELL
		| RIO_PEF_INB_MBOX;

	__raw_writel(krio_priv->rio_pe_feat,
		     &krio_priv->car_csr_regs->pe_feature);

	__raw_writel(KEYSTONE_RIO_MAX_PORT << 8,
		     &krio_priv->car_csr_regs->sw_port);

	__raw_writel((RIO_SRC_OPS_READ
		      | RIO_SRC_OPS_WRITE
		      | RIO_SRC_OPS_STREAM_WRITE
		      | RIO_SRC_OPS_WRITE_RESPONSE
		      | RIO_SRC_OPS_DATA_MSG
		      | RIO_SRC_OPS_DOORBELL
		      | RIO_SRC_OPS_ATOMIC_TST_SWP
		      | RIO_SRC_OPS_ATOMIC_INC
		      | RIO_SRC_OPS_ATOMIC_DEC
		      | RIO_SRC_OPS_ATOMIC_SET
		      | RIO_SRC_OPS_ATOMIC_CLR
		      | RIO_SRC_OPS_PORT_WRITE),
		     &krio_priv->car_csr_regs->src_op);

	__raw_writel((RIO_DST_OPS_READ
		      | RIO_DST_OPS_WRITE
		      | RIO_DST_OPS_STREAM_WRITE
		      | RIO_DST_OPS_WRITE_RESPONSE
		      | RIO_DST_OPS_DATA_MSG
		      | RIO_DST_OPS_DOORBELL
		      | RIO_DST_OPS_PORT_WRITE),
		     &krio_priv->car_csr_regs->dest_op);

	__raw_writel(RIO_PELL_ADDR_34,
		     &krio_priv->car_csr_regs->pe_logical_ctl);

	val = (((KEYSTONE_RIO_SP_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
	       KEYSTONE_RIO_SP_HDR_EP_REC_ID);
	__raw_writel(val, &krio_priv->serial_port_regs->sp_maint_blk_hdr);

	/* clear high bits of local config space base addr */
	__raw_writel(0x00000000, &krio_priv->car_csr_regs->local_cfg_hbar);

	/* set local config space base addr */
	__raw_writel(0x00520000, &krio_priv->car_csr_regs->local_cfg_bar);

	/* Enable MASTER_ENABLE BIT(30) bit */
	val = RIO_PORT_GEN_MASTER;
	if (krio_priv->board_rio_cfg.host)
		val |= RIO_PORT_GEN_HOST | RIO_PORT_GEN_DISCOVERED;
	__raw_writel(val, &krio_priv->serial_port_regs->sp_gen_ctl);

#if defined(CONFIG_RAPIDIO_PORT_LINK_TIMEOUT)
	/* set link timeout value */
	__raw_writel((CONFIG_RAPIDIO_PORT_LINK_TIMEOUT & 0xffffff) << 8,
		     &krio_priv->serial_port_regs->sp_link_timeout_ctl);
	dev_info(krio_priv->dev, "configure port link timeout to %d00ns\n",
			CONFIG_RAPIDIO_PORT_LINK_TIMEOUT * 3); /*approximately*/
#else
	/* set link timeout value */
	__raw_writel(0x000FFF00,
		     &krio_priv->serial_port_regs->sp_link_timeout_ctl);
#endif

#if defined(CONFIG_RAPIDIO_PORT_RESPONSE_TIMEOUT)
	/* set response timeout value */
	__raw_writel((CONFIG_RAPIDIO_PORT_RESPONSE_TIMEOUT & 0xffffff) << 8,
		     &krio_priv->serial_port_regs->sp_rsp_timeout_ctl);
	dev_info(krio_priv->dev, "configure port response timeout to %dns\n",
			(15*(1+((krio_priv->board_rio_cfg.serdes_config.cfg_cntl>>4)&0xf))*214*CONFIG_RAPIDIO_PORT_RESPONSE_TIMEOUT)/100);
	/*
	 * with 1400MHz CPU clock
	 * with DMA clock 466.7MHz  (CPU clock / 3) --> 2.14ns period
	 * Timeout = 15 x (PER_SET_CNTL[PRESCALER_SELECT] + 1) x 2.14 ns x CONFIG_RAPIDIO_PORT_RESPONSE_TIMEOUT
	 */
#else
	/* set response timeout value */
	__raw_writel(0x000FFF00,
		     &krio_priv->serial_port_regs->sp_rsp_timeout_ctl);
#endif

	/* allows SELF_RESET and PWDN_PORT resets to clear stcky reg bits */
	__raw_writel(0x00000001, &krio_priv->link_regs->reg_rst_ctl);

	/* Local Logical/Transport Layer Error Enable */
	__raw_writel(0x4400000, &krio_priv->link_regs->local_err_en);

	/* Set error detection mode */
	/* clear all errors */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->err_det);

	/* enable all error detection */
	__raw_writel(0xFFC07EC0, &krio_priv->err_mgmt_regs->err_en);

	/* set err det block header */
	val = (((KEYSTONE_RIO_ERR_HDR_NEXT_BLK_PTR & 0xffff) << 16) |
	       KEYSTONE_RIO_ERR_EXT_FEAT_ID);
	__raw_writel(val, &krio_priv->err_mgmt_regs->err_report_blk_hdr);

	/* clear msb of err captured addr reg */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->h_addr_capt);

	/* clear lsb of err captured addr reg */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->addr_capt);

	/* clear err captured source and dest DevID reg */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->id_capt);

	/* clear err captured packet info */
	__raw_writel(0x00000000, &krio_priv->err_mgmt_regs->ctrl_capt);

	/* Set per port information */
	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		__raw_writel(0x41004141,
			     &krio_priv->phy_regs->phy_sp[port].__rsvd[3]);

		/* Set the baud rate to the port information */
		val = __raw_readl(&krio_priv->serial_port_regs->sp[port].ctl2);
		val |= BIT(24 - (baud << 1));
		__raw_writel(val, &krio_priv->serial_port_regs->sp[port].ctl2);
	}

	/* Set packet forwarding */
	for (i = 0; i < KEYSTONE_RIO_MAX_PKT_FW_ENTRIES; i++) {
		if ((krio_priv->board_rio_cfg.pkt_forwarding) && (i < 8)) {
			struct keystone_routing_config *routing = krio_priv->board_rio_cfg.routing_config;

			/* Enable packet forwarding DevId and port defined in DTS */
			__raw_writel(routing[i].dev_id_low
				     | (routing[i].dev_id_high << 16),
				     &(krio_priv->regs->pkt_fwd_cntl[i].pf_16b));
			__raw_writel((routing[i].dev_id_low & 0xff)
				     | ((routing[i].dev_id_high & 0xff ) << 8)
				     | (routing[i].port << 16),
				     &(krio_priv->regs->pkt_fwd_cntl[i].pf_8b));

			dev_info(krio_priv->dev,
				 "enabling packet forwarding to port %d for DestID 0x%04x - 0x%04x\n",
				 routing[i].port, routing[i].dev_id_low, routing[i].dev_id_high);
		} else {
			/* Disable packet forwarding */
			__raw_writel(0xffffffff, &(krio_priv->regs->pkt_fwd_cntl[i].pf_16b));
			__raw_writel(0x0003ffff, &(krio_priv->regs->pkt_fwd_cntl[i].pf_8b));
		}
	}
	if (!krio_priv->board_rio_cfg.pkt_forwarding)
		dev_info(krio_priv->dev, "packet forwarding disabled\n");

	/* Force all writes to finish */
	val = __raw_readl(&krio_priv->err_mgmt_regs->ctrl_capt);

	return res;
}

/**
 * keystone_rio_start - Start RapidIO controller
 */
static void keystone_rio_start(struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Set PEREN bit to enable logical layer data flow */
	val = (KEYSTONE_RIO_PER_EN | KEYSTONE_RIO_PER_FREE);
	__raw_writel(val, &krio_priv->regs->pcr);

	/* Set BOOT_COMPLETE bit */
	val = __raw_readl(&krio_priv->regs->per_set_cntl);
	__raw_writel(val | KEYSTONE_RIO_BOOT_COMPLETE,
		     &krio_priv->regs->per_set_cntl);
}

/**
 * keystone_rio_stop - Stop RapidIO controller
 */
static void keystone_rio_stop(struct keystone_rio_data *krio_priv)
{
	u32 val;

	/* Disable PEREN bit to stop all new logical layer transactions */
	val = __raw_readl(&krio_priv->regs->pcr);
	val &= ~KEYSTONE_RIO_PER_EN;
	__raw_writel(val, &krio_priv->regs->pcr);
}

static void keystone_rio_reset_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv = container_of(work,
		struct keystone_rio_data, reset_work);
	u32 ports_rst;
	u32 ports;
	u32 port;

	ports_rst = __raw_readl(&krio_priv->evt_mgmt_regs->evt_mgmt_rst_port_stat);
	dev_info(krio_priv->dev,
		"reset device request received on ports: 0x%x\n", ports_rst);

	/* Acknowledge reset */
	ports = ports_rst;
	while (ports) {
		port = __ffs(ports);
		ports &= ~(1 << port);
		__raw_writel(KEYSTONE_RIO_PORT_PLM_STATUS_RST_REQ,
			&krio_priv->phy_regs->phy_sp[port].status);
	}

	__raw_writel(ports_rst, &krio_priv->evt_mgmt_regs->evt_mgmt_rst_port_stat);

	/* Reinitialize SRIO peripheral */
	keystone_rio_shutdown_controller(krio_priv);
	krio_priv->isolate = 1;
	keystone_rio_setup_controller(krio_priv);
}

static int keystone_rio_test_link(u8 port, struct keystone_rio_data *krio_priv)
{
	int res = 0;
	u32 value;

	res = keystone_rio_maint_read(krio_priv, port, 0xffff,
				      krio_priv->board_rio_cfg.size,
				      0, 0, sizeof(value), &value);
	return res;
}

static int keystone_rio_port_error_recovery(u32 port, struct keystone_rio_data *krio_priv)
{
	int res;
	u32 err_stat;
	u32 err_det;
	u32 plm_status;
	int i;

	if (unlikely(port >= KEYSTONE_RIO_MAX_PORT))
		return -EINVAL;

	err_stat = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
	err_det = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].det);
	plm_status = __raw_readl(&krio_priv->phy_regs->phy_sp[port].status);
	dev_info(krio_priv->dev,
		"port %d: err_stat = 0x%08x, err_det = 0x%08x, plm_status = 0x%08x\n",
		port, err_stat, err_det, plm_status);

	/* Acknowledge errors on this port */
	__raw_writel(err_stat & KEYSTONE_RIO_PORT_ERROR_MASK,
		&krio_priv->serial_port_regs->sp[port].err_stat);
	__raw_writel(0, &krio_priv->err_mgmt_regs->sp_err[port].det);
	__raw_writel(plm_status & KEYSTONE_RIO_PORT_PLM_STATUS_ERRORS,
		&krio_priv->phy_regs->phy_sp[port].status);

	if (unlikely(!(err_stat & RIO_PORT_N_ERR_STS_PORT_OK)))
		return -EINVAL;

	if (err_stat & RIO_PORT_N_ERR_STS_OUT_ES) {
		u32 lm_resp;
		u32 ackid_stat;
		u32 l_ackid;
		u32 r_ackid;

		dev_info_ratelimited(krio_priv->dev,
			"port %d: Output Error-Stopped recovery\n", port);

		/*
		 * Clear valid bit in maintenance response register.
		 * Send both Input-Status Link-Request and PNA control symbols and
		 * wait for valid maintenance response
		 */
		__raw_readl(&krio_priv->serial_port_regs->sp[port].link_maint_resp);
		__raw_writel(0x2003f044,
			&krio_priv->phy_regs->phy_sp[port].long_cs_tx1);
		i = 0;
		do {
			if (++i > KEYSTONE_RIO_TIMEOUT_CNT) {
				dev_dbg(krio_priv->dev,
					"port %d: Input-Status response timeout\n", port);
				goto oes_rd_err;
			}

			ndelay(KEYSTONE_RIO_TIMEOUT_NSEC);
			lm_resp = __raw_readl(
				&krio_priv->serial_port_regs->sp[port].link_maint_resp);
		} while (!(RIO_PORT_N_MNT_RSP_RVAL(lm_resp)));
		dev_dbg(krio_priv->dev,
			"port %d: Input-Status response = 0x%08x\n", port, lm_resp);

		/* Set outbound ackID to the value expected by link partner */
		ackid_stat = __raw_readl(
			&krio_priv->serial_port_regs->sp[port].ackid_stat);
		dev_dbg(krio_priv->dev,
			"port %d: ackid_stat = 0x%08x\n", port, ackid_stat);
		l_ackid = (ackid_stat & RIO_PORT_N_ACK_INBOUND) >> 24;
		r_ackid = RIO_PORT_N_MNT_RSP_ASTAT(lm_resp);
		__raw_writel((l_ackid << 24) | r_ackid,
			&krio_priv->serial_port_regs->sp[port].ackid_stat);
		udelay(50);

		/*
		 * Reread outbound ackID as it may have changed as a result of
		 * outstanding unacknowledged packets retransmission
		 */
		ackid_stat = __raw_readl(
			&krio_priv->serial_port_regs->sp[port].ackid_stat);
		dev_dbg(krio_priv->dev,
			"port %d: ackid_stat = 0x%08x\n", port, ackid_stat);
		r_ackid = ackid_stat & RIO_PORT_N_ACK_OUTBOUND;

		/*
		 * Set link partner inbound ackID to outbound ackID + 1.
		 * Set link partner outbound and outstanding ackID to inbound ackID.
		 */
		res = keystone_rio_maint_write(krio_priv, port, 0xffff,
			krio_priv->board_rio_cfg.size,
			0, 0x100 + RIO_PORT_N_ACK_STS_CSR(
				krio_priv->board_rio_cfg.ports_remote[port]),
			sizeof(u32), ((++r_ackid << 24) & RIO_PORT_N_ACK_INBOUND) |
				(l_ackid << 8) | l_ackid);
		if (res < 0) {
			dev_dbg(krio_priv->dev,
				"port %d: failed to align ackIDs with link partner port %d\n",
				port, krio_priv->board_rio_cfg.ports_remote[port]);
		}

oes_rd_err:
		err_stat = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
		err_det = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].det);
		plm_status = __raw_readl(&krio_priv->phy_regs->phy_sp[port].status);
		dev_dbg(krio_priv->dev,
			"port %d: err_stat = 0x%08x, err_det = 0x%08x, plm_status = 0x%08x\n",
			port, err_stat, err_det, plm_status);
	}

	if (err_stat & RIO_PORT_N_ERR_STS_INP_ES) {
		dev_info_ratelimited(krio_priv->dev,
			"port %d: Input Error-Stopped recovery\n", port);

		res = keystone_rio_maint_write(krio_priv, port, 0xffff,
			krio_priv->board_rio_cfg.size,
			0, 0x100 + RIO_PORT_N_MNT_REQ_CSR(1,
				krio_priv->board_rio_cfg.ports_remote[port]),
			sizeof(u32), RIO_MNT_REQ_CMD_IS);
		if (res < 0) {
			dev_dbg(krio_priv->dev,
				"port %d: failed to issue Input-Status request from link partner port %d\n",
				port, krio_priv->board_rio_cfg.ports_remote[port]);
		}
		udelay(50);

		err_stat = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
		err_det = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].det);
		plm_status = __raw_readl(&krio_priv->phy_regs->phy_sp[port].status);
		dev_dbg(krio_priv->dev,
			"port %d: err_stat = 0x%08x, err_det = 0x%08x, plm_status = 0x%08x\n",
			port, err_stat, err_det, plm_status);
	}

	return err_stat & KEYSTONE_RIO_PORT_ERRORS;
}

static void keystone_rio_pe_dpc(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv = container_of(work,
		struct keystone_rio_data, pe_work);
	u32 port;

	dev_dbg(krio_priv->dev, "errors on ports: 0x%x\n", krio_priv->pe_ports);

	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		if (test_and_clear_bit(port, (void*)&krio_priv->pe_ports)) {
			/*
			 * Recover from port error state.
			 * Depending on the link partner state, two attempts may be needed.
			 */
			if (keystone_rio_port_error_recovery(port, krio_priv)) {
				if (keystone_rio_port_error_recovery(port, krio_priv)) {
					dev_err(krio_priv->dev,
						"port %d: failed to recover from errors\n",
						krio_priv->pe_ports);
						continue;
				}
			}

			/*
			 * Perform test read on successful recovery.
			 * This seems to be required to generate consecutive interrupts.
			 */
			keystone_rio_test_link(port, krio_priv);
		}
	}
}

/**
 * keystone_rio_port_status - Return if the port is OK or not
 * @port: index of the port
 *
 * Return %0 if the port is ready or %-EIO on failure.
 */
static int keystone_rio_port_status(int port, struct keystone_rio_data *krio_priv)
{
	unsigned int count = 0, value;
	int res = 0;

	if (port >= KEYSTONE_RIO_MAX_PORT)
		return -EINVAL;

	/* Check port status */
	for (count = 0; count < 100; count++) {
		value = __raw_readl(&(krio_priv->serial_port_regs->sp[port].err_stat));
		if ((value & RIO_PORT_N_ERR_STS_PORT_OK) != 0)
			break;
		udelay(10);
	}

	if ((value & RIO_PORT_N_ERR_STS_PORT_OK) != 0) {
		value = __raw_readl(&(krio_priv->serial_port_regs->sp_gen_ctl));
		if ((value & RIO_PORT_GEN_HOST) == 0 &&
		    (value & RIO_PORT_GEN_DISCOVERED) == 0) {
			dev_warn(krio_priv->dev,
				"port %d is not discovered\n",
				port);
			return -ENONET;
		}

		res = keystone_rio_test_link(port, krio_priv);
		if (res != 0) {
			dev_err(krio_priv->dev,
				"link test failed on port %d\n", port);

			res = keystone_rio_port_error_recovery(port, krio_priv);
			if (res != 0) {
				dev_err(krio_priv->dev,
					"link sync failed on port %d\n", port);
				return -EIO;
			}
		}
	} else {
		dev_err(krio_priv->dev,
			"port %d is not initialized - PORT_OK not set\n",
			port);
		return -EIO;
	}

	return 0; /* port must be solid OK */
}

/**
 * keystone_rio_port_enable - Enable a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_enable(u32 port, struct keystone_rio_data *krio_priv)
{
	/* Enable port */
	__raw_writel(0x600000, &(krio_priv->serial_port_regs->sp[port].ctl));
}

/**
 * keystone_rio_port_disable - Disable a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_disable(u32 port, struct keystone_rio_data *krio_priv)
{
	/* Disable port */
	__raw_writel(0x800000, &(krio_priv->serial_port_regs->sp[port].ctl));
}

/**
 * keystone_rio_port_init - Configure a RapidIO port
 * @port: index of the port to configure
 * @path_mode: serdes configuration
 */
static int keystone_rio_port_init(u32 port, u32 path_mode, struct keystone_rio_data *krio_priv)
{
	u32 val;

	if (unlikely(port >= KEYSTONE_RIO_MAX_PORT))
		return -EINVAL;

	/* Program channel allocation to ports (1x, 2x or 4x) */
	__raw_writel(path_mode, &(krio_priv->phy_regs->phy_sp[port].path_ctl));

	/* Silence and discovery timers */
	if ((port == 0)|| (port == 2)) {
		__raw_writel(0x20000000,
			     &(krio_priv->phy_regs->phy_sp[port].silence_timer));
		__raw_writel(0x20000000,
			     &(krio_priv->phy_regs->phy_sp[port].discovery_timer));
	}

	/* Set multicast and packet forwarding mode otherwise unicast mode */
	if (1 == krio_priv->isolate)
		val = 0x00309000;
	else
		val = krio_priv->board_rio_cfg.pkt_forwarding ? 0x00209000 : 0x00109000;
	__raw_writel(val, &(krio_priv->transport_regs->transport_sp[port].control));

	/* Errors detected by TLM to notify the system host by interrupt */
	__raw_writel(KEYSTONE_RIO_PORT_TLM_STATUS,
			&krio_priv->transport_regs->transport_sp[port].int_enable);

	/* Disable port-write request generation */
	__raw_writel(0, &(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));

	/* Sets the threshold for the dead link timer
	 * The duration of the dead link timer: 2^13 * DLT_THRESH * P_CLK period */
	/*__raw_writel(0x0000ffff, &krio_priv->phy_regs->phy_sp[port].imp_spec_ctl);*/

	/* Enable interrupt for reset request */
	val = __raw_readl(&(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));
	__raw_writel(val | BIT(port),
		     &(krio_priv->evt_mgmt_regs->evt_mgmt_rst_int_en));

	/* Event Management interrupts enable LLM events, MECS, PW, ImplSpec */
	__raw_writel(KEYSTONE_RIO_BLOCK_EM_INT_ENABLE,
			&krio_priv->evt_mgmt_regs->evt_mgmt_int_enable);
	__raw_writel(KEYSTONE_RIO_BLOCK_EM_DEV_INT_ENABLE,
			&krio_priv->evt_mgmt_regs->evt_mgmt_dev_int_en);

	/* Enable all PLM interrupts */
	__raw_writel(0xffffffff, &(krio_priv->phy_regs->phy_sp[port].int_enable));
	__raw_writel(1, &(krio_priv->phy_regs->phy_sp[port].all_int_en));

	/* Sets the threshold for reporting too many consecutive retries 180 */
	__raw_writel(0x100000B4, &krio_priv->phy_regs->phy_sp[port].denial_ctl);

	/* Enable all errors */
	__raw_writel(0xffffffff, &(krio_priv->err_mgmt_regs->sp_err[port].rate_en));

	/* Thresh set */
	__raw_writel(0x08020000, &(krio_priv->err_mgmt_regs->sp_err[port].rate));
	__raw_writel(0x01010000, &(krio_priv->err_mgmt_regs->sp_err[port].thresh));

	/* Cleanup port error status */
	__raw_writel(KEYSTONE_RIO_PORT_ERROR_MASK,
		     &(krio_priv->serial_port_regs->sp[port].err_stat));
	__raw_writel(0, &(krio_priv->err_mgmt_regs->sp_err[port].det));
	__raw_writel(KEYSTONE_RIO_PORT_PLM_PORT_STATUS,
			&krio_priv->phy_regs->phy_sp[port].status);

	/* Enable errors detected by PBM to notify the system host by interrupt */
	__raw_writel(KEYSTONE_RIO_PORT_PBM_INT_STATUS, &krio_priv->pkt_buf_regs->pkt_buf_sp[port].int_enable);

	return 0;
}

/**
 * keystone_rio_port_set_routing - Configure routing for a RapidIO port
 * @port: index of the port to configure
 */
static void keystone_rio_port_set_routing(u32 port, struct keystone_rio_data *krio_priv)
{
	u32 base_dev_id = krio_priv->board_rio_cfg.size ?
		__raw_readl(&krio_priv->car_csr_regs->base_dev_id) & 0xffff :
		(__raw_readl(&krio_priv->car_csr_regs->base_dev_id) >> 16) & 0xff;

	u32 brr = KEYSTONE_RIO_PKT_FW_BRR_NUM;

	/* Enable routing to LLM for this BRR and port */
	__raw_writel(0x84000000,
		     &(krio_priv->transport_regs->transport_sp[port].base_route[brr].ctl));

	/*
	 * Configure the Base Routing Register (BRR) to ensure that all packets
	 * matching our DevId are admitted.
	 */
	__raw_writel((base_dev_id << 16) |
		     (krio_priv->board_rio_cfg.size ? 0xffff : 0xff),
		     &(krio_priv->transport_regs->transport_sp[port].base_route[brr].pattern_match));

	dev_dbg(krio_priv->dev, "pattern_match = 0x%x for BRR %d\n",
		__raw_readl(&krio_priv->transport_regs->transport_sp[port].base_route[brr].pattern_match),
		brr);

	/* Enable routing to LLM for this BRR and port */
	brr += 1;
	__raw_writel(0x84000000,
		     &(krio_priv->transport_regs->transport_sp[port].base_route[brr].ctl));

	/*
	 * Configure the Base Routing Register (BRR) to ensure that all broadcast
	 * packets are admitted as well.
	 */
	__raw_writel((0xffff << 16) |
		     (krio_priv->board_rio_cfg.size ? 0xffff : 0xff),
		     &(krio_priv->transport_regs->transport_sp[port].base_route[brr].pattern_match));

	dev_dbg(krio_priv->dev, "pattern_match = 0x%x for BRR %d\n",
		__raw_readl(&krio_priv->transport_regs->transport_sp[port].base_route[brr].pattern_match),
		brr);
}

/*------------------------- Configuration space mngt  ----------------------*/
static void keystone_rio_set_host_deviceid(struct rio_mport *port)
{
	struct keystone_rio_data *krio_priv = port->priv;
	u16 id = rio_local_get_device_id(port);

	dev_info(krio_priv->dev, "rio[%d]: AR[%d] devid=0x%x hdid=0x%x offset=0x%x\n",
			port->index, __LINE__, port->host_deviceid, id, node_id_offset);
	id += node_id_offset;
	if ((id != 0xFF) && (port->sys_size == 0))
		port->host_deviceid = id;
	else if ((id != 0xFFFF) && (port->sys_size != 0))
		port->host_deviceid = id;
}

/**
 * keystone_local_config_read - Generate a KeyStone local config space read
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be read into
 *
 * Generates a KeyStone local configuration space read. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_read(struct rio_mport *mport,
				      int index, u32 offset, int len, u32 *data)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	/*
	 * Workaround for rionet: the processing element features must content
	 * RIO_PEF_INB_MBOX and RIO_PEF_INB_DOORBELL bits that cannot be set on
	 * KeyStone hardware. So cheat the read value in this case...
	 */
	if (unlikely(offset == RIO_PEF_CAR))
		*data = krio_priv->rio_pe_feat;
	else
		*data = __raw_readl(
			(void __iomem *)krio_priv->car_csr_regs + offset);

	dev_dbg(krio_priv->dev,
		"local_conf_r: index %d offset 0x%x data 0x%x\n",
		index, offset, *data);

	return 0;
}

/**
 * keystone_local_config_write - Generate a KeyStone local config space write
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @data: Value to be written
 *
 * Generates a KeyStone local configuration space write. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int keystone_local_config_write(struct rio_mport *mport,
				       int index, u32 offset, int len, u32 data)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	dev_dbg(krio_priv->dev,
		"local_conf_w: index %d offset 0x%x data 0x%x\n",
		index, offset, data);
	__raw_writel(data,
		(void __iomem *)krio_priv->car_csr_regs + offset);

	return 0;
}

/**
 * keystone_rio_config_read - Generate a KeyStone read maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Location to be read into
 *
 * Generates a KeyStone read maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_read(struct rio_mport *mport, int index, u16 destid,
			 u8 hopcount, u32 offset, int len, u32 *val)
{
	return keystone_rio_maint_read((struct keystone_rio_data *)mport->priv,
				       mport->index, destid, mport->sys_size,
				       hopcount, offset, len, val);
}

/**
 * keystone__rio_config_write - Generate a KeyStone write
 * maintenance transaction
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of transaction
 * @hopcount: Number of hops to target device
 * @offset: Offset into configuration space
 * @len: Length (in bytes) of the maintenance transaction
 * @val: Value to be written
 *
 * Generates an KeyStone write maintenance transaction. Returns %0 on
 * success or %-EINVAL on failure.
 */
static int
keystone_rio_config_write(struct rio_mport *mport, int index, u16 destid,
			  u8 hopcount, u32 offset, int len, u32 val)
{
	return keystone_rio_maint_write((struct keystone_rio_data *)mport->priv,
					mport->index, destid, mport->sys_size,
					hopcount, offset, len, val);
}

/*------------------------------- Port-Write management --------------------------*/

static int keystone_rio_port_write_enable(struct keystone_rio_data *krio_priv,
					  u32 port,
					  int enable)
{
	u32 val;

	/* Clear port-write reception capture */
	__raw_writel(0, &(krio_priv->port_write_regs->port_wr_rx_capt[port]));

	if (enable) {
		/*
		 * Enable generation of port-write requests
		 */
		__raw_writel(
			BIT(25) | BIT(26) | BIT(28),
			&(krio_priv->phy_regs->phy_sp[port].port_wr_enable));

		val = __raw_readl(
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));

		__raw_writel(
			val | BIT(0), /* PW_EN */
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));
	} else {
		/*
		 * Disable generation of port-write requests
		 */
		__raw_writel(
			0,
			&(krio_priv->phy_regs->phy_sp[port].port_wr_enable));

		val = __raw_readl(
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));

		__raw_writel(
			val & ~(BIT(0)), /* PW_EN */
			&(krio_priv->phy_regs->phy_sp[port].all_port_wr_en));
	}

	return 0;
}

/**
 *  keystone_rio_port_write_handler - KeyStone port write interrupt handler
 *
 * Handles port write interrupts. Parses a list of registered
 * port write event handlers and executes a matching event handler.
 */
static void keystone_rio_port_write_handler(struct keystone_rio_data *krio_priv)
{
	int pw, i;

	/* Check that we have a port-write-in case */
	pw = __raw_readl(&(krio_priv->port_write_regs->port_wr_rx_stat)) & 0x1;
	if (!pw)
		return;

	/* Retrieve PW message */
	krio_priv->port_write_msg.msg.em.comptag =
		__raw_readl(&(krio_priv->port_write_regs->port_wr_rx_capt[0]));
	krio_priv->port_write_msg.msg.em.errdetect =
		__raw_readl(&(krio_priv->port_write_regs->port_wr_rx_capt[1]));
	krio_priv->port_write_msg.msg.em.is_port =
		__raw_readl(&(krio_priv->port_write_regs->port_wr_rx_capt[2]));
	krio_priv->port_write_msg.msg.em.ltlerrdet =
		__raw_readl(&(krio_priv->port_write_regs->port_wr_rx_capt[3]));

	for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++) {
		if (!krio_priv->mport[i])
			continue;

#ifdef KEYSTONE_RIO_DEBUG_PW
		printk(KERN_DEBUG "rio[%d]: PortWrite(%d) %08x %08x %08x %08x\n",
				i, krio_priv->port_write_msg.msg_count,
				krio_priv->port_write_msg.msg.raw[0],
				krio_priv->port_write_msg.msg.raw[1],
				krio_priv->port_write_msg.msg.raw[2],
				krio_priv->port_write_msg.msg.raw[3]);
#endif
		krio_priv->port_write_msg.msg_count++;
		rio_inb_pwrite_handler(krio_priv->mport[i], &krio_priv->port_write_msg.msg);
	}

	/* Clear port write status pw_val, to receive next PW */
	__raw_writel(BIT(0), &(krio_priv->port_write_regs->port_wr_rx_stat));
}

/**
 * keystone_rio_port_write_init - KeyStone port write interface init
 * @mport: Master port implementing the port write unit
 *
 * Initializes port write unit hardware and buffer
 * ring. Called from keystone_rio_setup(). Returns %0 on success
 * or %-ENOMEM on failure.
 */
static int keystone_rio_port_write_init(struct keystone_rio_data *krio_priv)
{
	int port;

	for (port = 0; port < KEYSTONE_RIO_MAX_PORT; port++) {
		/* Disabling port write */
		keystone_rio_port_write_enable(krio_priv, port, 0);

		/* Clear port-write-in capture registers */
		__raw_writel(
			0,
			&(krio_priv->port_write_regs->port_wr_rx_capt[port]));
	}

	return 0;
}

/**
 * keystone_rio_pw_enable - enable/disable port-write interface init
 * @mport: Master port implementing the port write unit
 * @enable: 1=enable; 0=disable port-write message handling
 */
static int keystone_rio_pw_enable(struct rio_mport *mport, int enable)
{
	return keystone_rio_port_write_enable(mport->priv,
					      mport->index,
					      enable);
}

static void keystone_rio_mcast_evt_handler(struct keystone_rio_data *krio_priv)
{
	__raw_writel(0x1, &(krio_priv->evt_mgmt_regs->evt_mgmt_mecs_stat));

	__raw_writel(0x1, &(krio_priv->phy_regs->phy_sp[0].rcvd_mecs));
	__raw_writel(0x1, &(krio_priv->phy_regs->phy_sp[1].rcvd_mecs));
	__raw_writel(0x1, &(krio_priv->phy_regs->phy_sp[2].rcvd_mecs));
	__raw_writel(0x1, &(krio_priv->phy_regs->phy_sp[3].rcvd_mecs));
}

static void keystone_rio_plerr_evt_handler(struct keystone_rio_data *krio_priv,
		u32 port)
{
	u32 sp_err_status = 0, sps = 0;
	u32 err_det = 0, errs = 0;
	u32 plm_status = 0, plms = 0;
	u32 pbm_status = 0, pbms = 0;
	u32 tlm_status = 0;
	u32 lm_resp;
	u32 i = 0;
	u32 err_rate_cnt = 0;
	u32 lane_stat0[4], lane_stat1[4];
	u32 attr_capt_dbg0;
	u32 capt_0_dbg1;
	u32 capt_1_dbg2;
	u32 capt_2_dbg3;
	u32 capt_3_dbg4;

	static const char * const sp_err_stat_msg[] = {
	[26] = "output has discarded a packet",
	[25] = "output failed condition, error threshold reached",
	[24] = "output degraded condition, error threshold reached",
	[20] = "output encountered a retry condition",
	[19] = "output received retry ctrl, can not make forward progress",
	[18] = "output received retry ctrl, in 'output retry-stopped' state",
	[17] = "output has encountered a transmission error",
	[16] = "output is in the 'output error-stopped' state",
	[10] = "input in the 'input retry-stopped' state",
	[9] = "input encountered a transmission error.",
	[8] = "input in the 'input error-stopped' state",
	[4] = "required to initiate a Maintenance Port-write operation",
	[2] = "IB or OB encountered an error hardware was unable to recover",
	[1] = "IB and OB are inited and the port is exchanging error-free ctrl",
	[0] = "IB and OB are not initialized",
	[3] = "Reserved",
	[21 ... 23] = "Reserved",
	[11 ... 15] = "Reserved",
	[5 ... 7] = "Reserved",
	[27 ... 31] = "IDLE",
	};

	static const char * const sp_err_mgmt_msg[] = {
	[31] = "impl specific error OB packet dropped or too many retries",
	[22] = "received control signal with bad CRC",
	[21] = "received an ACK ctrl symbol with an unexpected ackID",
	[20] = "received packet-not-accepted ctrl symbol",
	[19] = "received packet with unexpected ackID",
	[18] = "received packet with bad CRC",
	[17] = "received packet exceeds 276 bytes",
	[14] = "ctrl symbol and packet data was scrambled before transmission",
	[5] = "link RESP received with an ackID that is not outstanding",
	[4] = "an unexpected packet or ctrl symbol was received",
	[2] = "received unaligned SC or PD or undefined code-group",
	[1] = "unexpected ACK ctrl symbol was received",
	[0] = "ACK or link RESP ctrl symbol not received TIMEOUT",
	[3] = "Reserved",
	[6 ... 13] = "Reserved",
	[15 ... 16] = "Reserved",
	[23 ... 30] = "Reserved",
	};

	static const char * const sp_plm_msg[] = {
	[31] = "maximum Denial Error",
	[28] = "link Initialization Notification",
	[27] = "dead Link Timer Event",
	[26] = "port error",
	[25] = "port output fail",
	[24] = "port output degraded",
	[16] = "IB reset request received",
	[15] = "physical buffer module detected Port-Write notification",
	[14] = "transport layer module detected Port-Write notification",
	[12] = "port received a Multicast-Event Control Symbol",
	[11] = "physical buffer module detected interrupt notification",
	[10] = "transport layer module detected interrupt notification",
	[13] = "Reserved",
	[29 ... 30] = "Reserved",
	[17 ... 23] = "Reserved",
	[0 ... 9] = "Reserved",
	};

	static const char * const sp_pbm_stat_msg[] = {
	[0] = "PBMe detected a packet that exceeded 276 bytes",
	[1] = "PBMe received a request to enqueue a packet on a wrong channel",
	[3] = "PBMe enqueue a packet for which it did not have a CRQ entry",
	[4] = "PBMe did not have enough data storage for request",
	[15] = "PBMe has not packets enqueued",
	[16] = "The PBMi has no packets enqueued",
	};

	if (unlikely(port >= KEYSTONE_RIO_MAX_PORT))
		return;

	err_rate_cnt = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].rate);

	for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++) {
		lane_stat0[i] = __raw_readl(&krio_priv->err_mgmt_regs->lane_stat[i].stat0);
		if (lane_stat0[i] & 0x00000F00) {
			lane_stat1[i] = __raw_readl(&krio_priv->err_mgmt_regs->lane_stat[i].stat1);
			dev_info_ratelimited(krio_priv->dev, "LANE%d_STAT0:0x%x 0x%x 8b/10b decoding errors %d\n",
					i, lane_stat0[i], lane_stat1[i], (lane_stat0[i] & 0x00000F00) >> 8);
		}
	}

	lm_resp = __raw_readl(&krio_priv->serial_port_regs->sp[port].link_maint_resp);
	sp_err_status = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
	sps = sp_err_status;
	while (sp_err_status & KEYSTONE_RIO_PORT_ERROR_STATUS) {
		i = __ffs(sp_err_status);
		dev_info_ratelimited(krio_priv->dev, "SP%d_ERR_STAT:0x%08x %s\n",
				port, sp_err_status, sp_err_stat_msg[i]);
		sp_err_status = 0; /*&= ~(1 << i);*/ /*show only first status*/
	}

	err_det = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].det);
	errs = err_det;
	while (err_det) {
		i = __ffs(err_det);
		dev_info_ratelimited(krio_priv->dev, "SP%d_ERR_DET:0x%08x %s\n",
				port, err_det, sp_err_mgmt_msg[i]);
		err_det &= ~(1 << i);
	}

	plm_status = __raw_readl(&krio_priv->phy_regs->phy_sp[port].status);
	plms = plm_status;
	while (plm_status & KEYSTONE_RIO_PORT_PLM_PORT_STATUS) {
		i = __ffs(plm_status);
		dev_info_ratelimited(krio_priv->dev, "PLM_SP%d_STATUS:0x%08x %s\n",
				port, plm_status, sp_plm_msg[i]);
		plm_status =0; /*&= ~(1 << i);*/ /*show only first status*/
	}

	pbm_status = __raw_readl(&krio_priv->pkt_buf_regs->pkt_buf_sp[port].status);
	pbms = pbm_status;
	while (pbm_status & KEYSTONE_RIO_PORT_PBM_STATUS_CHECK) {
		i = __ffs(pbm_status);
		dev_info_ratelimited(krio_priv->dev, "PBM_SP%d_STATUS:0x%08x %s\n",
						port, pbm_status, sp_pbm_stat_msg[i]);
		pbm_status &= ~(1 << i);
	}

	tlm_status = __raw_readl(&krio_priv->transport_regs->transport_sp[port].status);
	if (tlm_status & KEYSTONE_RIO_PORT_TLM_STATUS_IG_BRR_FILTER)
		dev_info_ratelimited(krio_priv->dev, "TLM_SP%d_STATUS:0x%08x %s\n",
				port, tlm_status,  "discarded based on the BRR route");
	if (tlm_status & KEYSTONE_RIO_PORT_TLM_STATUS_IG_BAD_VC)
		dev_info_ratelimited(krio_priv->dev, "TLM_SP%d_STATUS:0x%08x %s\n",
				port, tlm_status, "inbound packet with the VC bit set");

	attr_capt_dbg0 = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].attr_capt_dbg0);
	capt_0_dbg1    = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].capt_0_dbg1);
	capt_1_dbg2    = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].capt_1_dbg2);
	capt_2_dbg3    = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].capt_2_dbg3);
	capt_3_dbg4    = __raw_readl(&krio_priv->err_mgmt_regs->sp_err[port].capt_3_dbg4);

	dev_info(krio_priv->dev, "Error Detect SP%d "
			"ERR_RATE:0x%08x "
			"LANE_STAT0:[0x%08x 0x%08x 0x%08x 0x%08x] "
			"ERR_STAT:0x%08x "
			"ERR_DET:0x%08x "
			"LM_RESP:0x%08x "
			"PLM_STATUS:0x%08x "
			"PBM_STATUS:0x%08x "
			"TLM_STATUS:0x%08x "
			"CAPT:0x%08x 0x%08x 0x%x 0x%x 0x%x\n",
			port, err_rate_cnt,
			lane_stat0[0], lane_stat0[1], lane_stat0[2], lane_stat0[3],
			sps, errs, lm_resp,
			plms, pbms, tlm_status,
			attr_capt_dbg0, capt_0_dbg1, capt_1_dbg2, capt_2_dbg3, capt_3_dbg4);

	__raw_writel((err_rate_cnt & 0xFFFFFF00), &krio_priv->err_mgmt_regs->sp_err[port].rate);
	__raw_writel(sps & KEYSTONE_RIO_PORT_ERROR_MASK,
			&(krio_priv->serial_port_regs->sp[port].err_stat));
	__raw_writel(plms & KEYSTONE_RIO_PORT_PLM_PORT_STATUS,
			&krio_priv->phy_regs->phy_sp[port].status);
	__raw_writel(0, &krio_priv->err_mgmt_regs->sp_err[port].det);
	__raw_writel(pbms & KEYSTONE_RIO_PORT_PBM_STATUS,
			&krio_priv->pkt_buf_regs->pkt_buf_sp[port].status);
	__raw_writel(tlm_status & KEYSTONE_RIO_PORT_TLM_STATUS,
			&krio_priv->transport_regs->transport_sp[port].status);
}

static void keystone_rio_llerr_evt_handler(struct keystone_rio_data *krio_priv)
{
	u32 err_det = 0;
	u32 h_addr_capt, addr_capt, id_capt, ctrl_capt, local_err_det;
	u32 local_h_addr_capt, local_addr_capt, local_id_capt, local_ctrl_capt;
	u32 i = 0;

	static const char * const err_det_msg[] = {
		[31] = "LSU received an ERROR RESP to an I/O logical layer REQ",
		[30] = "RXU received an ERROR RESP to a message logical layer REQ",
		[29] = "received a RESP of ERROR for a GSM Logical Layer REQ",
		[28] = "RXU received message data payload with an invalid size or seg",
		[27] = "RX illegal fields in RESP/REQ packet for an IO/msg transaction",
		[26] = "received a packet bad dstID not for this end point",
		[25] = "timeout REQ has been detected by the RXU",
		[24] = "timeout RESP has been detected by an LSU or the TXU",
		[23] = "unsolicited RESP packet has been received by LSU or TXU",
		[22] = "MAU has received an unsupported transaction",
		[14] = "data streaming PDU length error",
		[13] = "short data streaming segment error",
		[12] = "long data streaming segment error",
		[11] = "open existing data streaming context error",
		[10] = "missing data streaming context error",
		[9]  = "no Context available for type9 drop packet",
		[7]  = "RX Security Violation access RX queues was blocked",
		[6]  = "DMA access to the MAU was blocked",
		[8] = "Reserved",
		[0 ... 5] = "Reserved",
		[15 ... 21] = "Reserved",
	};

	err_det = __raw_readl(&krio_priv->err_mgmt_regs->err_det);
	while (err_det) {
		i = __ffs(err_det);
		if (printk_ratelimit()) {
			h_addr_capt = __raw_readl(&krio_priv->err_mgmt_regs->h_addr_capt);
		    addr_capt = __raw_readl(&krio_priv->err_mgmt_regs->addr_capt);
		    id_capt = __raw_readl(&krio_priv->err_mgmt_regs->id_capt);
		    ctrl_capt = __raw_readl(&krio_priv->err_mgmt_regs->ctrl_capt);
            dev_info(krio_priv->dev, "LLEM:[0x%08x 0x%08x 0x%08x 0x%08x 0x%08x] %s\n",
                     err_det, h_addr_capt, addr_capt, id_capt, ctrl_capt,
                     err_det_msg[i]);
		}
		err_det &= ~(1 << i);
	}

	local_err_det = __raw_readl(&krio_priv->link_regs->local_err_det);
	if ((0 != local_err_det) && (printk_ratelimit())) {
		local_h_addr_capt = __raw_readl(&krio_priv->link_regs->local_h_addr_capt);
		local_addr_capt = __raw_readl(&krio_priv->link_regs->local_addr_capt);
		local_id_capt = __raw_readl(&krio_priv->link_regs->local_id_capt);
		local_ctrl_capt = __raw_readl(&krio_priv->link_regs->local_ctrl_capt);

		if (local_err_det & KEYSTONE_RIO_LLERR_DET_LOCAL_ILL_ID)
			dev_info(krio_priv->dev, "LINK: Illegal transaction target error\n");
		if (local_err_det & KEYSTONE_RIO_LLERR_DET_LOCAL_ILL_TYPE)
			dev_info(krio_priv->dev, "LINK: Unsupported Transaction is received\n");

		dev_info(krio_priv->dev, "LINK: High Addr Capt: 0x%x "
								 "Addr Capt: 0x%x DevID Capt: 0x%x Ctrl Capt: 0x%x\n",
				 local_h_addr_capt, local_addr_capt, local_id_capt, local_ctrl_capt);
	}

	__raw_writel(0x0, &(krio_priv->link_regs->local_err_det));
	__raw_writel(0x0, &(krio_priv->err_mgmt_regs->err_det));
}

/*------------------------- Message passing management  ----------------------*/

/*
 * Retrieve MP receive code completion
 */
static inline u32 keystone_rio_mp_get_cc(u32 psdata1, u32 packet_type)
{
	return (packet_type == RIO_PACKET_TYPE_MESSAGE ?
		psdata1 >> 15 : psdata1 >> 8) & 0x3;
}

/*
 * This function retrieves the packet type for a given mbox
 */
static inline u32 keystone_rio_mp_get_type(int mbox, int letter,
					   struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan = &(krio_priv->rx_channels[mbox][letter]);
	return (u32) ((krx_chan->packet_type != RIO_PACKET_TYPE_STREAM) &&
 		      (krx_chan->packet_type != RIO_PACKET_TYPE_MESSAGE)) ?
		RIO_PACKET_TYPE_MESSAGE : krx_chan->packet_type;
}

/*
 * This function retrieves the mapping from Linux RIO mailbox to stream id for type 9 packets
 */
static inline u32 keystone_rio_mbox_to_strmid(int mbox, int letter,
					      struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan = &(krio_priv->rx_channels[mbox][letter]);
	return (u32) krx_chan->stream_id;
}

/*
 * Release a free receive buffer
 */
static void keystone_rio_rxpool_free(void *arg, unsigned q_num, unsigned bufsize,
				     struct dma_async_tx_descriptor *desc)
{
       struct keystone_rio_rx_chan_info *krx_chan = arg;
       struct keystone_rio_data *krio_priv = krx_chan->priv;
       struct keystone_rio_packet *p_info = desc->callback_param;

       dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);
       p_info->buff = NULL;
       if (krx_chan->release_cb && p_info->buff_cookie)
	       krx_chan->release_cb(p_info->buff_cookie);
       kfree(p_info);

       return;
}

static void keystone_rio_chan_work_handler(unsigned long data)
{
	struct keystone_rio_data *krio_priv = (struct keystone_rio_data *)data;
	struct keystone_rio_rx_chan_info *krx_chan;
	int mbox, letter;

	for (mbox = 0; mbox < KEYSTONE_RIO_MAX_MBOX; mbox++) {
		for (letter = 0; letter < KEYSTONE_RIO_MAX_LETTER; letter++) {
			krx_chan = &(krio_priv->rx_channels[mbox][letter]);
			if (krx_chan->running) {
				krx_chan->port->inb_msg[mbox].mcback(krx_chan->port,
								     krx_chan->dev_id,
								     mbox);
				dmaengine_resume(krx_chan->dma_channel);
			}
		}
	}
}

/*
 * A dummy receive callback is needed for dmaengine teardown
 */
static void keystone_rio_rx_complete(void *data)
{
}

static void keystone_rio_rx_notify(struct dma_chan *chan, void *arg)
{
	struct keystone_rio_data *krio_priv = arg;

	dmaengine_pause(chan);

	tasklet_schedule(&krio_priv->task);

	return;
}

static void keystone_rio_mp_inb_exit(int mbox, int letter,
		struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan;

	krx_chan = &(krio_priv->rx_channels[mbox][letter]);

	if (!(krx_chan->dma_channel))
		return;

	mutex_lock(&krx_chan->dma_lock);
	dmaengine_pause(krx_chan->dma_channel);
	dma_release_channel(krx_chan->dma_channel);
	krx_chan->dma_channel = NULL;
	mutex_unlock(&krx_chan->dma_lock);

	return;
}

static int keystone_rio_mp_inb_init(int mbox, int letter,
				    struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan;
	struct dma_keystone_info config;
	dma_cap_mask_t mask;
	int err = -ENODEV;
	int i;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* DMA RX channel */
	krx_chan = &(krio_priv->rx_channels[mbox][letter]);
	krx_chan->priv = krio_priv;
	krx_chan->dma_channel = dma_request_channel_by_name(mask, krx_chan->name);
	if (IS_ERR_OR_NULL(krx_chan->dma_channel))
		goto fail;

	mutex_init(&krx_chan->dma_lock);

	memset(&config, 0, sizeof(config));
	config.direction	    = DMA_DEV_TO_MEM;
	config.scatterlist_size	    = KEYSTONE_RIO_SGLIST_SIZE;
	config.rxpool_destructor    = keystone_rio_rxpool_free;
	config.rxpool_param	    = krx_chan;
	config.rxpool_thresh_enable = DMA_THRESH_NONE;

	for (i = 0; i < KEYSTONE_QUEUES_PER_CHAN &&
		     krx_chan->queue_depths[i] &&
		     krx_chan->buffer_sizes[i]; ++i) {
		config.rxpools[i].pool_depth  = krx_chan->queue_depths[i];
		config.rxpools[i].buffer_size = krx_chan->buffer_sizes[i];
		dev_dbg(krio_priv->dev, "rx_pool[%d] depth %d, size %d\n", i,
			config.rxpools[i].pool_depth,
			config.rxpools[i].buffer_size);
	}
	config.rxpool_count = i;

	err = dma_keystone_config(krx_chan->dma_channel, &config);
	if (err) {
		dev_err(krio_priv->dev,
			"Error configuring RX channel, err %d\n", err);
		goto fail;
	}

	dma_set_notify(krx_chan->dma_channel,
		       keystone_rio_rx_notify,
		       krio_priv);

	krx_chan->flow_num = dma_get_rx_flow(krx_chan->dma_channel);
	krx_chan->queue_num = dma_get_rx_queue(krx_chan->dma_channel);

	dev_info(krio_priv->dev,
		 "Opened rx channel: %p (mbox=%d, letter=%d, flow=%d,"
		 " rx_q=%d, pkt_type=%d)\n",
		 krx_chan->dma_channel, mbox, letter, krx_chan->flow_num,
		 krx_chan->queue_num, krx_chan->packet_type);

	return 0;

fail:
	if (krx_chan->dma_channel) {
		dma_release_channel(krx_chan->dma_channel);
		krx_chan->dma_channel = NULL;
	}
	return err;
}

static int keystone_rio_get_rxu_map(struct keystone_rio_data *krio_priv)
{
	int id;
	unsigned long bit_sz = sizeof(krio_priv->rxu_map_bitmap) * 8;

	id = find_first_zero_bit(&(krio_priv->rxu_map_bitmap[0]), bit_sz);
	while (id < krio_priv->rxu_map_start)
		id = find_next_zero_bit(&(krio_priv->rxu_map_bitmap[0]), bit_sz, ++id);
	if (id > krio_priv->rxu_map_end)
		return -1;

	__set_bit(id, &(krio_priv->rxu_map_bitmap[0]));
	return id;
}

static void keystone_rio_free_rxu_map(int id, struct keystone_rio_data *krio_priv)
{
	clear_bit(id, &(krio_priv->rxu_map_bitmap[0]));
}

/**
 * keystone_rio_map_mbox - Map a mailbox to a given queue.
 * for both type 11 and type 9 packets.
 * @mbox: mailbox to map
 * @queue: associated queue number
 *
 * Returns %0 on success or %-ENOMEM on failure.
 */
static int keystone_rio_map_mbox(int mbox, int letter,
				 int queue, int flowid, int size,
				 struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan = &(krio_priv->rx_channels[mbox][letter]);
	u32 mapping_entry_low = 0;
	u32 mapping_entry_high = 0;
	u32 mapping_entry_qid;
	u32 mapping_t9_reg[3];
	u32 pkt_type;
	u32 deviceid;
	u32 port = 0;
	int i;

	/* Retrieve the packet type */
	pkt_type = keystone_rio_mp_get_type(mbox, letter, krio_priv);
	if ((port = krio_priv->ports_registering))
		port = __ffs(port);
	deviceid = krio_priv->mport[port]->host_deviceid;

	if (pkt_type == RIO_PACKET_TYPE_MESSAGE) {
		/* Map the multi-segment mailbox to the corresponding Rx queue for type 11 */
		if (krx_chan->letter == -1) {
			mapping_entry_low = ((mbox & 0x1f) << 16)
				| (0x3f000000); /* Given mailbox, all letters, srcid = 0 */
		} else {
			mapping_entry_low = ((mbox & 0x1f) << 16) | ((letter & 0x03) << 22)
				| (0xfff000000); /* Given mailbox and letter, srcid = 0 */
		}

		/* multi-segment messaging and promiscuous (don't care about src/dst id) */
		mapping_entry_high = (deviceid << 16) | KEYSTONE_RIO_MAP_FLAG_SEGMENT
			| KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC;
	} else {
		/* Map the multi-segment mailbox for type 9 */
		mapping_t9_reg[0] = 0; /* accept all COS and srcid = 0 */
		mapping_t9_reg[1] = KEYSTONE_RIO_MAP_FLAG_SRC_PROMISC
			| KEYSTONE_RIO_MAP_FLAG_DST_PROMISC; /* promiscuous (don't care about src/dst id) */
		mapping_t9_reg[2] = (0xffff << 16)
			| (keystone_rio_mbox_to_strmid(mbox, letter, krio_priv));
	}

	/* Set TT flag */
	if (size) {
		mapping_entry_high |= KEYSTONE_RIO_MAP_FLAG_TT_16;
		mapping_t9_reg[1]  |= KEYSTONE_RIO_MAP_FLAG_TT_16;
	}

	/* QMSS/PktDMA mapping (generic for both type 9 and 11) */
	mapping_entry_qid = (queue & 0x3fff) | (flowid << 16);

	i = keystone_rio_get_rxu_map(krio_priv);
	if (i < 0)
		return -ENOMEM;

	krx_chan->rxu_map_id[0] = i;

	dev_info(krio_priv->dev,
		"Using RXU map %d @ 0x%08x: mbox = %d, letter = %d,"
		" flowid = %d, queue = %d pkt_type = %d devid=0x%x\n",
		i, (u32)&(krio_priv->regs->rxu_map[i]), mbox, letter,
             flowid, queue, pkt_type, deviceid);

	if (pkt_type == RIO_PACKET_TYPE_MESSAGE) {
		/* Set packet type 11 rx mapping */
		__raw_writel(mapping_entry_low,
			     &(krio_priv->regs->rxu_map[i].ltr_mbox_src));
		__raw_writel(mapping_entry_high,
			     &(krio_priv->regs->rxu_map[i].dest_prom_seg));
	} else {
		/* Set packet type 9 rx mapping */
		__raw_writel(mapping_t9_reg[0],
			     &(krio_priv->regs->rxu_type9_map[i].cos_src));
      		__raw_writel(mapping_t9_reg[1],
			     &(krio_priv->regs->rxu_type9_map[i].dest_prom));
		__raw_writel(mapping_t9_reg[2],
			     &(krio_priv->regs->rxu_type9_map[i].stream));
	}

	__raw_writel(mapping_entry_qid,
		     &(krio_priv->regs->rxu_map[i].flow_qid));

	if (pkt_type == RIO_PACKET_TYPE_MESSAGE) {
		/*
		 *  The RapidIO peripheral looks at the incoming RapidIO msgs
		 *  and if there is only one segment (the whole msg fits into one
		 *  RapidIO msg), the peripheral uses the single segment mapping
		 *  table. Therefore we need to map the single-segment mailbox too.
		 *  The same Rx CPPI Queue is used (as for the multi-segment
		 *  mailbox).
		 */
		mapping_entry_high &= ~KEYSTONE_RIO_MAP_FLAG_SEGMENT;

		i = keystone_rio_get_rxu_map(krio_priv);
		if (i < 0)
			return -ENOMEM;

		krx_chan->rxu_map_id[1] = i;
	        dev_info(krio_priv->dev,
			"Using RXU map %d @ 0x%08x: mbox = %d, letter = %d,"
			" flowid = %d, queue = %d pkt_type = %d devid=0x%x\n",
			i, (u32)&(krio_priv->regs->rxu_map[i]), mbox, letter,
			flowid, queue, pkt_type, deviceid);

		__raw_writel(mapping_entry_low,
			     &(krio_priv->regs->rxu_map[i].ltr_mbox_src));
		__raw_writel(mapping_entry_high,
			     &(krio_priv->regs->rxu_map[i].dest_prom_seg));
		__raw_writel(mapping_entry_qid,
			     &(krio_priv->regs->rxu_map[i].flow_qid));
	}

	return 0;
}

static int keystone_rio_open_rx_mbox(struct rio_mport *mport, void *dev_id,
				     int mbox, int letter, int entries)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan = &krio_priv->rx_channels[mbox][letter];
	int res;

	if (mbox >= KEYSTONE_RIO_MAX_MBOX || letter >= KEYSTONE_RIO_MAX_LETTER)
		return -EINVAL;

	/* Check that number of entries is a power of two to ease ring management */
	if ((entries & (entries - 1)) != 0)
		return -EINVAL;

	/* Check if already initialized */
	if (krx_chan->port)
		return -EBUSY;

	/* Check if open was for a channel not specified in device tree */
	if (!krx_chan->name) {
		dev_info(krio_priv->dev, "open inb mbox: mport = 0x%x, dev_id = 0x%x, "
				"mbox = %d, letter = %d not supported\n",
				(u32)mport, (u32)dev_id, mbox, letter);
		return 0;
	}

	dev_info(krio_priv->dev,
		"open inb mbox: mport = 0x%x, dev_id = 0x%x,"
		" mbox = %d, letter = %d, entries = %d\n",
		(u32)mport, (u32)dev_id, mbox, letter, entries);

	/* Initialization of RapidIO inbound MP */
	if (!(krx_chan->dma_channel)) {
		res = keystone_rio_mp_inb_init(mbox, letter, krio_priv);
		if (res)
			return res;
	}

	krx_chan->dev_id  = dev_id;
	krx_chan->port    = mport;
	krx_chan->entries = entries;
	krx_chan->release_cb = NULL;

#ifdef KEYSTONE_RIO_RX_OUTOFORDER_WA
	krx_chan->packets =
		kzalloc(sizeof(*krx_chan->packets) * entries, GFP_ATOMIC);
	if (!krx_chan->packets) {
		dev_err(krio_priv->dev, "rx packet ring alloc failed\n");
		return -ENOMEM;
	}

	krx_chan->packet_insert_ix = 0;
	krx_chan->packet_rx_ix = 0;
#endif

	krx_chan->running = 1;

	if (krio_priv->board_rio_cfg.host)
		keystone_rio_set_host_deviceid(mport);

	/* Map the mailbox to queue/flow */
	res = keystone_rio_map_mbox(mbox,
				    letter,
				    krx_chan->queue_num,
				    krx_chan->flow_num,
				    mport->sys_size,
				    krio_priv);

	dmaengine_resume(krx_chan->dma_channel);

	return res;
}

static int keystone_rio_open_inb_mbox_let(struct rio_mport *mport, void *dev_id,
					  int mbox, int letter_mask,
					  int entries)
{
	int res, letter;

	for (letter = 0; letter < KEYSTONE_RIO_MAX_LETTER; letter++) {
		if (!(letter_mask & (1 << letter)))
			continue;

		res = keystone_rio_open_rx_mbox(mport, dev_id,
						mbox, letter, entries);
		if (res)
			return res;
	}

	return 0;
}

/**
 * keystone_rio_open_inb_mbox - Initialize KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring, request the inbound message interrupt,
 * and enables the inbound message unit. Returns %0 on success
 * and %-EINVAL, %-EBUSY or %-ENOMEM on failure.
 */
static int keystone_rio_open_inb_mbox(struct rio_mport *mport, void *dev_id,
				      int mbox, int entries)
{
	return keystone_rio_open_rx_mbox(mport, dev_id, mbox, 0, entries);
}

static void keystone_rio_close_rx_mbox(int mbox, int letter,
				       struct keystone_rio_data *krio_priv,
				       void (*release_cb)(void *cookie))
{
	struct keystone_rio_rx_chan_info *krx_chan = &krio_priv->rx_channels[mbox][letter];

	if (mbox >= KEYSTONE_RIO_MAX_MBOX || letter >= KEYSTONE_RIO_MAX_LETTER)
		return;

	krx_chan->running = 0;

	if (!krx_chan->port)
		return;

	krx_chan->port = NULL;
	krx_chan->release_cb = release_cb;

	/* Release associated resource */
	keystone_rio_free_rxu_map(krx_chan->rxu_map_id[0], krio_priv);
	keystone_rio_free_rxu_map(krx_chan->rxu_map_id[1], krio_priv);

	keystone_rio_mp_inb_exit(mbox, letter, krio_priv);

#ifdef KEYSTONE_RIO_RX_OUTOFORDER_WA
	kfree(krx_chan->packets);
#endif
}

static void keystone_rio_close_inb_mbox_let(struct rio_mport *mport,
					    int mbox, int letter_mask,
					    void (*release_cb)(void *cookie))
{
	struct keystone_rio_data *krio_priv = mport->priv;
	int letter;

	for (letter = 0; letter < KEYSTONE_RIO_MAX_LETTER; letter++) {
		if (!(letter_mask & (1 << letter)))
			continue;

		dev_info(krio_priv->dev,
			 "close inb mbox: mport = 0x%x,"
			 " mbox = %d, letter = %d\n",
			 (u32)mport, mbox, letter);

		keystone_rio_close_rx_mbox(mbox, letter, krio_priv, release_cb);
	}
}

/**
 * keystone_rio_close_inb_mbox - Shut down KeyStone inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
static void keystone_rio_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	dev_info(krio_priv->dev,
		 "close inb mbox: mport = 0x%x, mbox = %d, letter = %d\n",
		 (u32)mport, mbox, 0);

	keystone_rio_close_rx_mbox(mbox, 0, krio_priv, NULL);
}

/**
 * keystone_rio_hw_add_inb_buffer - Add buffer to the KeyStone
 *   inbound message queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Adds the @buf buffer to the KeyStone inbound message queue. Returns
 * %0 on success or %-EINVAL on failure.
 */
static int keystone_rio_hw_add_inb_buffer_let(struct rio_mport *mport,
					      int mbox, int letter,
					      void *buffer, void *cookie)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan = &krio_priv->rx_channels[mbox][letter];
	struct dma_async_tx_descriptor *desc = NULL;
	struct keystone_rio_packet *p_info;

	if (unlikely(!krx_chan->running))
		return -EINVAL;

	/* Allocate a primary receive queue entry */
	p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
	if (!p_info) {
		dev_err(krio_priv->dev, "packet alloc failed\n");
		return -ENOMEM;
	}
	p_info->priv = krio_priv;
	p_info->buff = buffer;
	p_info->buff_cookie = cookie;

	sg_init_table(p_info->sg, KEYSTONE_RIO_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], p_info->buff, krx_chan->buffer_sizes[0]);

	p_info->sg_ents = 2 + dma_map_sg(krio_priv->dev, &p_info->sg[2],
					 1, DMA_FROM_DEVICE);

	if (p_info->sg_ents != 3) {
		dev_err(krio_priv->dev, "dma map failed\n");
		p_info->buff = NULL;
		kfree(p_info);
		return -EINVAL;
	}

	desc = dmaengine_prep_slave_sg(krx_chan->dma_channel, p_info->sg,
				       p_info->sg_ents, DMA_DEV_TO_MEM,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);

	if (IS_ERR_OR_NULL(desc)) {
		u32 err = 0;
		dma_unmap_sg(krio_priv->dev, &p_info->sg[2],
			     1, DMA_FROM_DEVICE);
		p_info->buff = NULL;
		kfree(p_info);
		err = PTR_ERR(desc);
		if (err != -ENOMEM) {
			dev_err(krio_priv->dev,
				"dma prep failed, error %d\n", err);
		}
		return -EINVAL;
	}

	desc->callback_param = p_info;
	desc->callback = keystone_rio_rx_complete;
	p_info->cookie = desc->cookie;

#ifdef KEYSTONE_RIO_RX_OUTOFORDER_WA
	krx_chan->packets[krx_chan->packet_insert_ix].packet = p_info;
	krx_chan->packets[krx_chan->packet_insert_ix].status = 0;
	krx_chan->packet_insert_ix =
		(krx_chan->packet_insert_ix + 1) % krx_chan->entries;
#endif

	return dma_rxfree_refill_one(krx_chan->dma_channel, 0, desc);
}

static int keystone_rio_hw_add_inb_buffer(struct rio_mport *mport,
					  int mbox, void *buf, void *cookie)
{
	return keystone_rio_hw_add_inb_buffer_let(mport, mbox, 0, buf, cookie);
}

#ifdef KEYSTONE_RIO_RX_OUTOFORDER_WA
static struct keystone_rio_packet * keystone_rio_hw_get_inb_packet(
				struct keystone_rio_data *krio_priv,
				struct keystone_rio_rx_chan_info *krx_chan)
{
	struct keystone_rio_packet *p_info = NULL;
	int i;

	/* Check if there is already valid packet received/stored at rx index */
	i = krx_chan->packet_rx_ix;
	if (krx_chan->packets[i].status) {
		p_info = krx_chan->packets[i].packet;
		krx_chan->packets[i].status = 0;
		krx_chan->packet_rx_ix = (i + 1) % krx_chan->entries;

		dev_info_ratelimited(krio_priv->dev,
			 "out of order rx buffer get (0x%p), rx_ix: %d, %s:%d\n",
			 p_info->buff, i, krx_chan->name, krx_chan->queue_num);

		return p_info;
	}

	while (1) {
		p_info = (struct keystone_rio_packet *)dma_get_one(krx_chan->dma_channel);

		if (p_info == NULL)
			return NULL;

		/* Check if received packet is the one at current rx index */
		i = krx_chan->packet_rx_ix;
		if (p_info == krx_chan->packets[i].packet) {
			krx_chan->packet_rx_ix = (i + 1) % krx_chan->entries;
			break;
		}

		/* Store/mark out of order packet in the rx packets ring */
		while ((i = (i + 1) % krx_chan->entries) !=
			krx_chan->packet_insert_ix) {
			if (p_info == krx_chan->packets[i].packet) {
				krx_chan->packets[i].status = 1;

				dev_info_ratelimited(krio_priv->dev,
					 "out of order rx buffer put (0x%p)"
					 ", rx_ix: %d, ins_ix: %d, %s:%d\n",
					 p_info->buff, krx_chan->packet_rx_ix, i,
					 krx_chan->name, krx_chan->queue_num);

				break;
			}
		}
	}

	return p_info;
}
#endif

/**
 * keystone_rio_hw_get_inb_message - Fetch inbound message from
 * the KeyStone message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 *
 * Gets the next available inbound message from the inbound message queue.
 * A pointer to the message is returned on success or NULL on failure.
 */
static void *keystone_rio_hw_get_inb_message_dst(struct rio_mport *mport,
						 int mbox, int letter,
						 int *sz, int *did,
						 void **cookie)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_rx_chan_info *krx_chan = &(krio_priv->rx_channels[mbox][letter]);
	struct keystone_rio_packet *p_info = NULL;
	void *buff = NULL;
	u32 cc;

dma_get_next:
	if (unlikely(!krx_chan->running))
		return NULL;

#ifdef KEYSTONE_RIO_RX_OUTOFORDER_WA
	p_info = keystone_rio_hw_get_inb_packet(krio_priv, krx_chan);
#else
	p_info = (struct keystone_rio_packet *) dma_get_one(krx_chan->dma_channel);
#endif
	if (!p_info)
		goto end;

	buff = p_info->buff;
	if (cookie)
		*cookie = p_info->buff_cookie;

	dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);

	/* Check CC from PS descriptor word 1 */
	cc = keystone_rio_mp_get_cc(p_info->psdata[1], krx_chan->packet_type);
	if (cc) {
		dev_warn_ratelimited(krio_priv->dev,
			 "MP receive completion code is non zero (0x%x)\n", cc);
		keystone_rio_hw_add_inb_buffer_let(mport, mbox, letter, buff,
				p_info->buff_cookie);
		if (cookie)
			*cookie = NULL;
		kfree(p_info);
		buff = NULL;
		goto dma_get_next;
	}

	*did = (p_info->psdata[0] >> 16) & 0xffff;
end:
	if (p_info)
		kfree(p_info);

	return buff;
}

static void *keystone_rio_hw_get_inb_message(struct rio_mport *mport, int mbox,
		void **cookie)
{
	int sz, did;

	return keystone_rio_hw_get_inb_message_dst(mport, mbox, 0, &sz, &did,
			cookie);
}

static void keystone_rio_mp_outb_exit(struct keystone_rio_data *krio_priv, int mbox)
{
	struct keystone_rio_tx_chan_info *ktx_chan = &(krio_priv->tx_channels[mbox]);

	if (!(ktx_chan->tx_channel))
		return;

	dmaengine_pause(ktx_chan->tx_channel);
	dma_release_channel(ktx_chan->tx_channel);
	ktx_chan->tx_channel = NULL;

	return;
}

static int keystone_rio_garbage_queues_get(struct keystone_rio_data *krio_priv,
		u32 qid[KEYSTONE_RIO_MAX_GARBAGE_QUEUES])
{
	u32 *qreg = krio_priv->regs->garbage_coll_qid;
	u32 qids[3] = {0};
	u32 i = 0;
	qids[0] = __raw_readl(&qreg[0]);
	qids[1] = __raw_readl(&qreg[1]);
	qids[2] = __raw_readl(&qreg[2]);
	qid[0] = qids[0] & 0x0000ffff;
	qid[1] = qids[0] >> 16;
	qid[2] = qids[1] & 0x0000ffff;
	qid[3] = qids[1] >> 16;
	qid[4] = qids[2] & 0x0000ffff;
	qid[5] = qids[2] >> 16;

	if (qids[0] == 0 || qids[1] == 0 || qids[2] == 0) {
		dev_warn(krio_priv->dev, "garbage monitor wrong queues setup");
		return -EINVAL;
	}

	for (i = 0; i < KEYSTONE_RIO_MAX_GARBAGE_QUEUES; ++i) {

		if ((krio_priv->gq_peek_reg[i]))
			iounmap(krio_priv->gq_peek_reg[i]);
		if ((krio_priv->gq_pop_push_reg[i]))
			iounmap(krio_priv->gq_pop_push_reg[i]);

		krio_priv->gq_peek_reg[i] =
				ioremap(KEYSTONE_QMSS_QUEUE_PEEK(qid[i]), 4);
		krio_priv->gq_pop_push_reg[i] =
				ioremap(KEYSTONE_QMSS_QUEUE_FETCH(qid[i]), 4);

		if (!krio_priv->gq_peek_reg[i] || !krio_priv->gq_pop_push_reg[i]) {
			dev_warn(krio_priv->dev, "failed to ioremap garbqueue reg");
			return -EINVAL;
		}
	}

	return 0;
}

static int keystone_rio_hwdesc_valid(struct keystone_rio_data *krio_priv, unsigned int hwdesc)
{
	u32 start, end, rv, num_desc, desc_size;
	int i;

	for (i = 0; i < KEYSTONE_QMSS_MAX_REGIONS; i++) {
		rv = __raw_readl(&krio_priv->qmss_mem_region[4 * i + 2]);
		if (!(start = __raw_readl(&krio_priv->qmss_mem_region[4 * i])))
			continue;

		num_desc = 1 << (5 + (rv & 0xFF));
		rv = (rv >> 16) & 0x1FFF;
		desc_size = 16 * (rv + 1);
		end = start + (num_desc - 1) * desc_size;

		if (hwdesc >= start && hwdesc <= end) {
			if (hwdesc & (desc_size - 1)) {
				dev_info_ratelimited(krio_priv->dev,
						"hwdesc %#x in %#x-%#x not aligned to %d\n",
						hwdesc, start, (end + desc_size), desc_size);
				return 0;
			}
			return 1;
		}
	}

	return 0;
}

static void keystone_rio_garbage_monitor_handler(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv = container_of(
			to_delayed_work(work), struct keystone_rio_data,
			garbage_monitor_task);
	u32 qids = 0;
	static u32 qreg = 0;
	static u32 qid[KEYSTONE_RIO_MAX_GARBAGE_QUEUES] = {0};
	u32 stat[KEYSTONE_RIO_MAX_GARBAGE_QUEUES] = {0};
	u32 i = 0, cnt = 0;
	unsigned long *data;
	u32 fdq, *epib;
	u32 total = 0;
	resource_size_t orig_tx_fdq;
	void *__iomem orig_map;
	void *__iomem dsp_map;
	unsigned int hwdesc;

	static const char * const gc_log[] = {
		[0] = "TimeOut on receiving one of the segments",
		[1] = "Length mismatch size in the UDI packet and rx payload",
		[2] = "Transaction error. Message received an \"error\" response",
		[3] = "Excessive Retries. MSG received more retry_count \"retry\" RESP",
		[4] = "Type11, MSG length/16 must be less than or equal to Ssize",
		[5] = "Garbage queue for identifying all programming errors",
	};

	if (qreg == 0) {
		keystone_rio_garbage_queues_get(krio_priv, qid);
		qreg = __raw_readl(&krio_priv->regs->garbage_coll_qid[0]);
		dev_info(krio_priv->dev, "garbage queues %d-%d", qid[0], qid[5]);
	}

	qids = __raw_readl(&krio_priv->regs->garbage_coll_qid[0]);
	if (qreg != qids) {
		qreg = 0;
		dev_info(krio_priv->dev, "recycle and reconfigure in next sched\n");
	}

	for (i = 0; i < KEYSTONE_RIO_MAX_GARBAGE_QUEUES; ++i) {

		cnt = __raw_readl(krio_priv->gq_peek_reg[i]);
		if (!cnt) continue;

		total += cnt;
		dev_dbg(krio_priv->dev, "Garbage queue %d has %d descs. %s\n",
				qid[i], cnt, gc_log[i]);

		do {
			hwdesc = __raw_readl(krio_priv->gq_pop_push_reg[i]);
			if (hwdesc == 0) {
				dev_warn(krio_priv->dev, "failed to pop descriptor\n");
				break;
			}

			if (!keystone_rio_hwdesc_valid(krio_priv, hwdesc)) {
				dev_info_ratelimited(krio_priv->dev,
						"Discarding invalid hwdesc %#x from queue %d\n",
						hwdesc, qid[i]);
				continue;
			}

			/* phys linux memmap region */
			if (hwdesc < KEYSTONE_RIO_DDR3A_LINUX_REGION) {
				dsp_map = ioremap(hwdesc, 64);
				if (dsp_map == NULL) {
					dev_warn(krio_priv->dev, "Failed to ioremap dsp hwdesc\n");
					continue;
				}
				data = (unsigned long *) dsp_map;
				print_hex_dump_debug("keystone-rapidio 2900000.rapidio: ",
					     DUMP_PREFIX_OFFSET, 32, 4, data, 64, 0);
				fdq = (data[2] & KEYSTONE_RIO_QM_DESC_MASK);
				iounmap(dsp_map);

			} else { /* phys DSP memmap region */
				data = (unsigned long *) hwdesc;
				epib = ((u32 *)hwdesc) + 8;
				epib[1] = i + 1;
				print_hex_dump_debug("keystone-rapidio 2900000.rapidio: ",
					     DUMP_PREFIX_OFFSET, 32, 4, data, 128, 0);
				fdq = (data[2] & KEYSTONE_RIO_QM_DESC_MASK);
			}

			orig_tx_fdq = KEYSTONE_QMSS_QUEUE_FETCH(fdq);
			if ((orig_map = ioremap(orig_tx_fdq, 4)) == NULL) {
				dev_warn(krio_priv->dev, "Failed to ioremap origin fdq\n");
				continue;
			}
			__raw_writel(hwdesc, orig_map);
			++stat[i];
			dev_dbg(krio_priv->dev, "Garbage monitor recycle desc %08lx into %d complete fdq\n",
					data[0x0c], fdq);
			iounmap(orig_map);
		} while (--cnt);
	}

	if (total)
		dev_info_ratelimited(krio_priv->dev, "garbage monitor found %d descs recycled [%d %d %d %d %d %d]\n",
				total, stat[0], stat[1], stat[2], stat[3], stat[4], stat[5]);

	schedule_delayed_work(&krio_priv->garbage_monitor_task,
			msecs_to_jiffies(KEYSTONE_RIO_GM_SCHED));
}

static int keystone_rio_tx_garbage_queues_init(struct keystone_rio_data *krio_priv)
{
	u32 i;
	u32 *qids = krio_priv->board_rio_cfg.tx_garbage_queues;
	__raw_writel(qids[1] << 16 | qids[0],
			&krio_priv->regs->garbage_coll_qid[0]);
	__raw_writel(qids[3] << 16 | qids[2],
			&krio_priv->regs->garbage_coll_qid[1]);
	__raw_writel(qids[5] << 16 | qids[4],
			&krio_priv->regs->garbage_coll_qid[2]);

	for (i = 0; i < KEYSTONE_RIO_MAX_GARBAGE_QUEUES; ++i) {
		krio_priv->gq_peek_reg[i] = 0;
		krio_priv->gq_pop_push_reg[i] = 0;
	}

	krio_priv->qmss_mem_region = ioremap(KEYSTONE_QMSS_MEM_REGION_BASE, KEYSTONE_QMSS_MEM_REGION_SIZE);
	if (unlikely(krio_priv->qmss_mem_region == NULL)) {
		dev_err(krio_priv->dev, "Failed to ioremap(%#x, %d)\n", KEYSTONE_QMSS_MEM_REGION_BASE,
				KEYSTONE_QMSS_MEM_REGION_SIZE);
		return -ENOMEM;
	}

	return 0;
}

static int keystone_rio_mp_outb_init(u8 port_id, struct keystone_rio_data *krio_priv, int mbox)
{
	struct dma_keystone_info config;
	struct keystone_rio_tx_chan_info *ktx_chan = &(krio_priv->tx_channels[mbox]);
	dma_cap_mask_t mask;
	int err = -ENODEV;
	const char *name;
	static bool gqinit = 1;
	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	/* FIXME: Maybe a check is needed if the current channel is already intialized */

	/* DMA TX channel */
	name = ktx_chan->name;
	ktx_chan->tx_channel = dma_request_channel_by_name(mask, name);
	if (IS_ERR_OR_NULL(ktx_chan->tx_channel)) {
		dev_err(krio_priv->dev,
			"Error requesting TX channel for mbox %d, err %d\n",
			mbox, err);
		goto fail;
	}

	memset(&config, 0, sizeof(config));
	config.direction	= DMA_MEM_TO_DEV;
	config.tx_queue_depth	= ktx_chan->queue_depth;
	err = dma_keystone_config(ktx_chan->tx_channel, &config);
	if (err) {
		dev_err(krio_priv->dev,
			"Error configuring TX channel for mbox %d, err %d\n",
			mbox, err);
		goto fail;
	}

	dev_info(krio_priv->dev, "Opened tx channel: %p (mbox=%d)\n",
		 ktx_chan->tx_channel, mbox);

	/*
	 * For the time being we are using only the first transmit queue
	 * In the future we may use queue 0 to 16 with multiple mbox support
	 */
	__raw_writel(port_id << 4, &(krio_priv->regs->tx_queue_sch_info[mbox]));

	if (gqinit) {
		err = keystone_rio_tx_garbage_queues_init(krio_priv);
		if (err)
			goto fail;
		gqinit = 0;
	}

	/* Setup TX garbage queues here so we are sure they are available */
	if (schedule_delayed_work(&krio_priv->garbage_monitor_task,
			msecs_to_jiffies(KEYSTONE_RIO_GM_SCHED))) {
		dev_info(krio_priv->dev, "started garbage monitor scheduler");
	}

	return 0;

fail:
	if (ktx_chan->tx_channel) {
		dma_release_channel(ktx_chan->tx_channel);
		ktx_chan->tx_channel = NULL;
	}

	return err;
}

/**
 * keystone_rio_open_outb_mbox - Initialize KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring
 *
 * Initializes buffer ring, request the outbound message interrupt,
 * and enables the outbound message unit. Returns %0 on success and
 * %-EINVAL, %-EBUSY or %-ENOMEM on failure.
 */
static int keystone_rio_open_outb_mbox(struct rio_mport *mport, void *dev_id,
				       int mbox, int entries)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct keystone_rio_tx_chan_info *ktx_chan = &(krio_priv->tx_channels[mbox]);
	int res;
	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return -EINVAL;

	/* Check that number of entries is a power of two to ease ring management */
	if ((entries & (entries - 1)) != 0)
		return -EINVAL;

	/* Check if already initialized */
	if (ktx_chan->port)
		return -EBUSY;

	dev_info(krio_priv->dev,
		"open_outb_mbox: mport = 0x%x, dev_id = 0x%x, "
		"mbox = %d, entries = %d\n",
		(u32) mport, (u32) dev_id, mbox, entries);

	/* Initialization of RapidIO outbound MP */
	res = keystone_rio_mp_outb_init(mport->index, krio_priv, mbox);
	if (res)
		return res;

	ktx_chan->dev_id  = dev_id;
	ktx_chan->port    = mport;
	ktx_chan->entries = entries;
	ktx_chan->slot    = 0;
	ktx_chan->running = 1;

	return 0;
}

static void keystone_rio_close_tx_mbox(int mbox, struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_tx_chan_info *ktx_chan = &(krio_priv->tx_channels[mbox]);

	if (mbox >= KEYSTONE_RIO_MAX_MBOX)
		return;

	ktx_chan->running = 0;

	if (!ktx_chan->port)
		return;

	ktx_chan->port = NULL;

	keystone_rio_mp_outb_exit(krio_priv, mbox);
}

/**
 * keystone_rio_close_outb_mbox - Shut down KeyStone outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the outbound message unit, stop queues and free all resources
 */
static void keystone_rio_close_outb_mbox(struct rio_mport *mport, int mbox)
{
	struct keystone_rio_data *krio_priv = mport->priv;

	dev_info(krio_priv->dev, "close outb mbox: mport = 0x%x, mbox = %d\n",
		(u32) mport, mbox);

	keystone_rio_close_tx_mbox(mbox, krio_priv);
}

static void keystone_rio_tx_complete(void *data)
{
	struct keystone_rio_packet *p_info  = data;
	struct keystone_rio_data *krio_priv = p_info->priv;
	int mbox_id			    = p_info->mbox;
	struct keystone_rio_tx_chan_info *ktx_chan = &(krio_priv->tx_channels[mbox_id]);
	struct rio_mport *port		    = ktx_chan->port;
	void *dev_id			    = ktx_chan->dev_id;

	dev_dbg(krio_priv->dev,	"tx_complete: psdata[0] = %08x, psdata[1] = %08x, epib[1] = %08x\n",
		p_info->psdata[0], p_info->psdata[1], p_info->epib[1]);

	p_info->status = dma_async_is_tx_complete(ktx_chan->tx_channel,
						  p_info->cookie, NULL, NULL);

	WARN_ON(p_info->status != DMA_COMPLETE && p_info->status != DMA_ERROR);

	dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);

	if (p_info->status == DMA_ERROR)
		dev_warn(krio_priv->dev, "dma transfer failed\n");

	if (ktx_chan->running)
		port->outb_msg[mbox_id].mcback(port, dev_id, mbox_id,
				(p_info->status == DMA_ERROR) || p_info->epib[1],
				p_info->buff_cookie);
	kfree(p_info);
}

/**
 * keystone_rio_hw_add_outb_message - Add a message to the KeyStone
 * outbound message queue
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox: Outbound mailbox
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 * @cookie: Opaque pointer, should be given to completion callback
 *
 * Adds the @buffer message to the KeyStone outbound message queue. Returns
 * %0 on success or %-EBUSY on failure.
 */
int keystone_rio_hw_add_outb_message_dst(struct rio_mport *mport,
				u16 destid, int mbox, int letter, int flags,
				void *buffer, const size_t len, void *cookie)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	struct dma_async_tx_descriptor *desc;
	struct keystone_rio_packet *p_info;
	struct keystone_rio_tx_chan_info *ktx_chan = &(krio_priv->tx_channels[mbox]);
	u32 plen;
	u32 packet_type;
	int ret = 0;
	void *send_buffer = NULL;
	if (unlikely(ktx_chan->port != mport))
		return -EINVAL;

	/*
	 * Ensure that the number of bytes being transmitted is a multiple
	 * of double-word. This is as per the specification.
	 */
	plen = ((len + 7) & ~0x7);

	p_info = kzalloc(sizeof(*p_info), GFP_ATOMIC);
	if (!p_info) {
		if (send_buffer)
			kfree(send_buffer);
		if (printk_ratelimit())
			dev_warn(krio_priv->dev, "failed to alloc packet info\n");
		return -ENOMEM;
	}

	p_info->priv = krio_priv;

	/* Word 1: source id and dest id (common to packet 11 and packet 9) */
/*	p_info->psdata[0] = (rdev->destid & 0xffff) | (mport->host_deviceid << 16);*/
	p_info->psdata[0] = (destid & 0xffff) | (mport->host_deviceid << 16);

	/*
	 * Warning - Undocumented HW requirement:
	 *      For type9, packet type MUST be set to 30 in
	 *	keystone_hw_desc.desc_info[29:25] bits.
	 *
	 *	For type 11, setting packet type to 31 in
	 *	those bits is optional.
	 */
	if (keystone_rio_mp_get_type(mbox, letter, krio_priv) == RIO_PACKET_TYPE_MESSAGE) {
		/* Packet 11 case (Message) */
		packet_type = 31;

		/* Word 2: ssize = 32 dword, infinite retries, letter = 0, mbox */
		p_info->psdata[1] = (KEYSTONE_RIO_MSG_SSIZE << 17) |
				((letter & 0x7) << 6) | (mbox & 0x3f);
	} else {
		/* Packet 9 case (Data Streaming) */
		packet_type = 30;

		/* Word 2: COS = 0, stream id */
		p_info->psdata[1] = keystone_rio_mbox_to_strmid(mbox, letter, krio_priv) << 16;
	}

#ifdef CONFIG_RAPIDIO_NSN
	p_info->psdata[1] |= KEYSTONE_RIO_DESC_FLAG_TT_16; /* tt */
#else
/*	if (rdev->net->hport->sys_size)*/
	if (mport->sys_size)
		p_info->psdata[1] |= KEYSTONE_RIO_DESC_FLAG_TT_16; /* tt */
#endif

	dev_dbg(krio_priv->dev,
		"packet type %d: psdata[0] = %08x, psdata[1] = %08x\n",
		keystone_rio_mp_get_type(mbox, letter, krio_priv),
		p_info->psdata[0], p_info->psdata[1]);

	dev_dbg(krio_priv->dev, "buf(len=%d, plen=%d)\n", len, plen);

	p_info->mbox = mbox;
	p_info->buff = buffer;
	p_info->buff_cookie = cookie;

	sg_init_table(p_info->sg, KEYSTONE_RIO_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], p_info->buff, plen);

	p_info->sg_ents = 2 + dma_map_sg(krio_priv->dev, &p_info->sg[2],
					 1, DMA_TO_DEVICE);

	if (p_info->sg_ents != KEYSTONE_RIO_SGLIST_SIZE) {
		kfree(p_info);
		if (send_buffer)
			kfree(send_buffer);
		if (printk_ratelimit())
			dev_warn(krio_priv->dev, "failed to map transmit packet\n");
		return -ENXIO;
	}

	desc = dmaengine_prep_slave_sg(ktx_chan->tx_channel,
				       p_info->sg, p_info->sg_ents, DMA_MEM_TO_DEV,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO | DMA_HAS_PKTTYPE
				       | (packet_type  << DMA_PKTTYPE_SHIFT));

	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(krio_priv->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);
		kfree(p_info);
		if (send_buffer)
			kfree(send_buffer);
		if (printk_ratelimit())
			dev_warn(krio_priv->dev, "failed to prep slave dma\n");
		return -ENOBUFS;
	}

	desc->callback_param = p_info;
	desc->callback = keystone_rio_tx_complete;
	p_info->slot = ktx_chan->slot;
	p_info->cookie = dmaengine_submit(desc);

	if (dma_submit_error(p_info->cookie)) {
		if (printk_ratelimit())
			dev_warn(krio_priv->dev, "failed to submit packet for dma: %d\n",
					 p_info->cookie);
		kfree(p_info);
		if (send_buffer)
			kfree(send_buffer);
		return -EBUSY;
	}

	/*
	 * Move slot index to the next message to be sent.
	 * Client is in charge of freeing the associated buffers
	 * because we do not have explicit hardware ring but queues, we
	 * do not know where we are in the sw ring. Let's try to keep
	 * slot in sync with client.
	 */
	ktx_chan->slot++;
	if (ktx_chan->slot == ktx_chan->entries)
		ktx_chan->slot = 0;

	return ret;
}

static int keystone_rio_hw_add_outb_message(struct rio_mport *mport,
					    struct rio_dev *rdev,
					    int mbox,
					    void *buffer,
					    const size_t len, void *cookie)
{
	return keystone_rio_hw_add_outb_message_dst(mport, rdev->destid,
						    mbox, 0,
						    rdev->net->hport->sys_size,
						    buffer, len, cookie);
}

/*------------------------ Main Linux driver functions -----------------------*/

static int keystone_rio_query_mport(struct rio_mport *mport,
				    struct rio_mport_attr *attr)
{
	struct keystone_rio_data *krio_priv = mport->priv;
	u32 port = mport->index;
	u32 rval;

	if (!attr)
		return -EINVAL;

	rval = __raw_readl(&krio_priv->serial_port_regs->sp[port].err_stat);
	if (rval & RIO_PORT_N_ERR_STS_PORT_OK) {
		rval = __raw_readl(&krio_priv->serial_port_regs->sp[port].ctl2);
		attr->link_speed = (rval & RIO_PORT_N_CTL2_SEL_BAUD) >> 28;
		rval = __raw_readl(&krio_priv->serial_port_regs->sp[port].ctl);
		attr->link_width = (rval & RIO_PORT_N_CTL_IPW) >> 27;
	} else
		attr->link_speed = RIO_LINK_DOWN;

	attr->flags        = 0;
	attr->dma_max_sge  = 0;
	attr->dma_max_size = 0;
	attr->dma_align    = 0;

	return 0;
}

static struct rio_ops keystone_rio_ops = {
	.lcread			= keystone_local_config_read,
	.lcwrite		= keystone_local_config_write,
	.cread			= keystone_rio_config_read,
	.cwrite			= keystone_rio_config_write,
	.dsend			= keystone_rio_dbell_send,
	.transfer		= keystone_rio_dio_transfer,
	.query_mport		= keystone_rio_query_mport,
	.pwenable		= keystone_rio_pw_enable,

	.open_outb_mbox		= keystone_rio_open_outb_mbox,
	.close_outb_mbox	= keystone_rio_close_outb_mbox,
	.open_inb_mbox		= keystone_rio_open_inb_mbox,
	.close_inb_mbox		= keystone_rio_close_inb_mbox,
	.add_outb_message	= keystone_rio_hw_add_outb_message,
	.add_inb_buffer		= keystone_rio_hw_add_inb_buffer,
	.get_inb_message	= keystone_rio_hw_get_inb_message,
};

static struct rio_msg_ops keystone_rio_msg_ops = {
	.rio_open_outb_mbox		= keystone_rio_open_outb_mbox,
	.rio_close_outb_mbox		= keystone_rio_close_outb_mbox,
	.rio_open_inb_mbox		= keystone_rio_open_inb_mbox,
	.rio_close_inb_mbox		= keystone_rio_close_inb_mbox,
	.rio_add_outb_message		= keystone_rio_hw_add_outb_message,
	.rio_add_outb_message_dst	= keystone_rio_hw_add_outb_message_dst,
	.rio_add_inb_buffer		= keystone_rio_hw_add_inb_buffer,
	.rio_add_inb_buffer_let		= keystone_rio_hw_add_inb_buffer_let,
	.rio_get_inb_message		= keystone_rio_hw_get_inb_message,
	.rio_get_inb_message_dst	= keystone_rio_hw_get_inb_message_dst,
	.rio_open_inb_mbox_let		= keystone_rio_open_inb_mbox_let,
	.rio_close_inb_mbox_let		= keystone_rio_close_inb_mbox_let,
};

static void keystone_rio_release_mport(struct device *dev)
{
	struct rio_mport *mport = to_rio_mport(dev);

	dev_dbg(dev, "RIO: %s %s id=%d\n", __func__, mport->name, mport->id);
}

struct rio_mport *keystone_rio_register_mport(u32 port_id, u32 size,
					      struct keystone_rio_data *krio_priv)
{
	struct rio_mport *port;

	port = kzalloc(sizeof(struct rio_mport), GFP_KERNEL);
	rio_mport_initialize(port);

	/*
	 * Set the sRIO port physical Id into the index field,
	 * the id field will be set by rio_register_mport() to
	 * the logical Id
	 */
	port->index = port_id;
	port->priv  = krio_priv;
	INIT_LIST_HEAD(&port->dbells);

	/*
	 * Make a dummy per port region as ports are not
	 * really separated on KeyStone
	 */
	port->iores.start = (u32)(krio_priv->serial_port_regs) +
		offsetof(struct keystone_rio_serial_port_regs,
			sp[port_id].link_maint_req);

	port->iores.end = (u32)(krio_priv->serial_port_regs) +
		offsetof(struct keystone_rio_serial_port_regs,
			sp[port_id].ctl);

	port->iores.flags = IORESOURCE_MEM;

#ifdef CONFIG_RAPIDIO_MEM_MAP
	port->mem_map = krio_priv->mem_map;
#endif

	rio_init_dbell_res(&port->riores[RIO_DOORBELL_RESOURCE], 0, 0xffff);
	rio_init_mbox_res(&port->riores[RIO_INB_MBOX_RESOURCE], 0, KEYSTONE_RIO_MAX_MBOX);
	rio_init_mbox_res(&port->riores[RIO_OUTB_MBOX_RESOURCE], 0, KEYSTONE_RIO_MAX_MBOX);

	sprintf(port->name, "RIO%d mport", port_id);

	port->ops = &keystone_rio_ops;
	port->msg_ops = &keystone_rio_msg_ops;
	port->sys_size = size;
	/* Hard coded here because in rio_disc_mport(), it is used in
	   rio_enum_complete() before it is retrieved in
	   rio_disc_peer() => rio_setup_device() */
	port->phys_efptr = 0x100;
	port->dev.release = keystone_rio_release_mport;

	/* Correct the host device id if needed
	 */
	if (port->host_deviceid <= 0 && !krio_priv->board_rio_cfg.host)
		keystone_rio_set_host_deviceid(port);

	/* Enable promiscuous mode */
	rio_local_write_config_32(port, 0x10380, 0x309000);

	/*
	 * Register the new mport
	 */
	rio_register_mport(port);

	krio_priv->mport[port_id] = port;

	return port;
}


static int keystone_rio_get_rx_channel_defaults(int mbox,
						int rx_chan,
						struct device_node *node,
						struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_rx_chan_info *krx_chan;
	u32 letter;

	if (rx_chan >= KEYSTONE_RIO_MAX_LETTER)
		return -EINVAL;

	/*
	 * If letter is defined, this mbox is mapped to the corresponding
	 * letter and the channel is for type 11 packets.
	 * If stream_id is defined, this mbox is mapped to the corresponding
	 * streamid and the channel is for type 9 packets.
	 * If none of the above is defined, this mbox is mapped to all letters
	 * and the channel is for type 11 packets. Only one entry is possible.
	 */
	if (of_property_read_u32(node, "letter", &letter) == 0) {
		if (letter >= 0 && letter < KEYSTONE_RIO_MAX_LETTER) {
			krx_chan = &(krio_priv->rx_channels[mbox][letter]);
			krx_chan->packet_type = RIO_PACKET_TYPE_MESSAGE;
			krx_chan->letter = letter;
		} else {
			dev_err(krio_priv->dev,
				"invalid \"letter\" parameter\n");
			return -EINVAL;
		}
	} else {
		krx_chan = &(krio_priv->rx_channels[mbox][rx_chan]);

		if(of_property_read_u32(node, "stream_id",
					&krx_chan->stream_id) == 0) {
			krx_chan->packet_type = RIO_PACKET_TYPE_STREAM;
		} else if (rx_chan == 0) {
			krx_chan->packet_type = RIO_PACKET_TYPE_MESSAGE;
			krx_chan->letter = -1;
		} else {
			dev_err(krio_priv->dev,
				"invalid \"rx-channel\" node\n");
			return -EINVAL;
		}
	}

	if (of_property_read_string(node, "channel", &krx_chan->name) < 0) {
		dev_err(krio_priv->dev, "missing \"channel\" parameter\n");
		return -ENOENT;
	}

	if (of_property_read_u32_array(node, "queue_depth",
				       krx_chan->queue_depths,
				       KEYSTONE_QUEUES_PER_CHAN) < 0) {
		dev_err(krio_priv->dev, "missing \"queue_depth\" parameter\n");
		return -ENOENT;
	}

	if (of_property_read_u32_array(node, "buffer_size",
				       krx_chan->buffer_sizes,
				       KEYSTONE_QUEUES_PER_CHAN) < 0) {
		dev_err(krio_priv->dev, "missing \"buffer_size\" parameter\n");
		return -ENOENT;
	}

	return 0;
}

static int keystone_rio_get_mbox_defaults(int mbox,
					  struct device_node *node_rio,
					  struct keystone_rio_data *krio_priv)
{
	struct keystone_rio_tx_chan_info *ktx_chan = &(krio_priv->tx_channels[mbox]);
	struct device_node *node, *rx_chans, *rx_chan;
	char node_name[24];
	int rx_chan_num = 0;

	snprintf(node_name, sizeof(node_name), "mbox-%d", mbox);
	node = of_get_child_by_name(node_rio, node_name);
	if (!node) {
		dev_err(krio_priv->dev, "could not find %s\n", node_name);
		of_node_put(node);
		return -ENODEV;
	}

	dev_dbg(krio_priv->dev, "using node \"%s\"\n", node_name);

	/* DMA tx chan config */
	if (of_property_read_string(node, "tx_channel", &ktx_chan->name) < 0) {
		dev_err(krio_priv->dev, "missing \"tx_channel\" parameter\n");
	}

	if (of_property_read_u32(node, "tx_queue_depth", &ktx_chan->queue_depth) < 0) {
		dev_err(krio_priv->dev,
			"missing \"tx_queue_depth\" parameter\n");
		ktx_chan->queue_depth = 128;
	}

	/* DMA rx chan config */
	rx_chans = of_get_child_by_name(node, "rx-channels");
	if (!rx_chans) {
		dev_err(krio_priv->dev,	"missing \"rx-channels\" node");
		of_node_put(node);
		return -ENODEV;
	}

	for_each_child_of_node(rx_chans, rx_chan) {
		if (keystone_rio_get_rx_channel_defaults(
			mbox, rx_chan_num, rx_chan, krio_priv) == 0)
			rx_chan_num++;
	}

	of_node_put(rx_chans);
	of_node_put(node);

	return 0;
}

static int keystone_rio_get_controller_defaults(struct device_node *node,
						 struct keystone_rio_data *krio_priv)
{
	int ret = 0;
	struct keystone_rio_board_controller_info *c = &krio_priv->board_rio_cfg;
	u32 temp[24];
	int i;
	int lanes;
	int mbox;
	struct property *keystone2_serdes_prop = NULL;

	i = of_property_match_string(node, "reg-names", "rio");
	krio_priv->regs = of_iomap(node, i);
	if (!krio_priv->regs) {
		dev_err(krio_priv->dev, "missing \"rio\" regs\n");
		ret = -ENOENT;
		goto error;
	}

	krio_priv->car_csr_regs		= (void __iomem *)krio_priv->regs + 0xb000;
	krio_priv->serial_port_regs	= (void __iomem *)krio_priv->regs + 0xb100;
	krio_priv->err_mgmt_regs	= (void __iomem *)krio_priv->regs + 0xc000;
	krio_priv->phy_regs		= (void __iomem *)krio_priv->regs + 0x1b000;
	krio_priv->transport_regs	= (void __iomem *)krio_priv->regs + 0x1b300;
	krio_priv->pkt_buf_regs		= (void __iomem *)krio_priv->regs + 0x1b600;
	krio_priv->evt_mgmt_regs	= (void __iomem *)krio_priv->regs + 0x1b900;
	krio_priv->port_write_regs	= (void __iomem *)krio_priv->regs + 0x1ba00;
	krio_priv->link_regs		= (void __iomem *)krio_priv->regs + 0x1bd00;
	krio_priv->fabric_regs		= (void __iomem *)krio_priv->regs + 0x1be00;

	i = of_property_match_string(node, "reg-names", "jtagid");
	krio_priv->jtagid_reg = of_iomap(node, i);
	if (!krio_priv->jtagid_reg) {
		dev_err(krio_priv->dev, "missing \"jtagid\" reg\n");
		ret = -ENOENT;
		goto error;
	}

	i = of_property_match_string(node, "reg-names", "serdes");
	krio_priv->serdes_regs = of_iomap(node, i);
	if (!krio_priv->serdes_regs) {
		dev_err(krio_priv->dev, "missing \"serdes\" regs\n");
		ret = -ENOENT;
		goto error;
	}

	if (of_property_read_u32 (node, "dev-id-size", &c->size)) {
		dev_err(krio_priv->dev, "Could not get default dev-id-size\n");
		ret = -ENOENT;
		goto error;
	}

	if (of_property_read_u32 (node, "ports", &c->ports)) {
		dev_err(krio_priv->dev, "Could not get default ports\n");
		ret = -ENOENT;
		goto error;
	}

	if (of_property_read_u32_array(node, "ports_remote",
		c->ports_remote, KEYSTONE_RIO_MAX_PORT)) {
		dev_err(krio_priv->dev, "missing \"remote_ports\" paramter\n");
		ret = -ENOENT;
		goto error;
	}

	/* SerDes config */
	keystone2_serdes_prop = of_find_property(node, "keystone2-serdes", NULL);
	if (!keystone2_serdes_prop) {
		/*
		 * Mode 0: sRIO config 0: MPY = 5x, div rate = half,
		 * link rate = 3.125 Gbps, mode 1x
		 */
		c->path_mode						= 0;
		c->serdes_config.cfg_cntl			= 0x0c053860;
		c->serdes_config.serdes_cfg_pll		= 0x0229;
		c->serdes_config.prescalar_srv_clk	= 0x001e;

		for (i = 0; i < KEYSTONE_RIO_MAX_PORT; i++) {
			c->serdes_config.rx_chan_config[i] = 0x00440495;
			c->serdes_config.tx_chan_config[i] = 0x00180795;
		}

		i = of_property_match_string(node, "reg-names", "serdes_sts");
		krio_priv->serdes_sts_reg = of_iomap(node, i);
		if (!krio_priv->serdes_sts_reg) {
			dev_err(krio_priv->dev, "missing \"serdes_sts\" reg\n");
			ret = -ENOENT;
			goto error;
		}
	} else {
		u32 prop_length = keystone2_serdes_prop->length;

		c->keystone2_serdes			= 1;
		c->serdes_config.cfg_cntl		= 0x08053860;
		c->serdes_config.prescalar_srv_clk	= 0x001f;

		if (of_property_read_u32(node, "baudrate", &c->serdes_baudrate)) {
			dev_err(krio_priv->dev,
				"Missing \"baudrate\" parameter. "
				"Setting 5Gbps as a default\n");
			c->serdes_baudrate = KEYSTONE_RIO_BAUD_5_000;
		}

		if (c->serdes_baudrate > KEYSTONE_RIO_BAUD_5_000) {
			c->serdes_baudrate = KEYSTONE_RIO_BAUD_5_000;
			dev_err(krio_priv->dev,
				"Invalid baud rate, forcing it to 5Gbps\n");
		}

		if (of_property_read_u32(node, "path_mode", &c->path_mode)) {
			dev_err(krio_priv->dev,
				"Missing \"path_mode\" parameter\n");
			ret = -ENOENT;
			goto error;
		}

		krio_priv->serdes_val_len = -1;
		krio_priv->serdes_values = NULL;
		/*
		 * read serdes init pairs if available
		 */
		if (prop_length) {
			if (prop_length % sizeof(*krio_priv->serdes_values)) {
				dev_err(krio_priv->dev, "Wrong RapidIO SerDes format. Will use default values.\n");
			} else {
				krio_priv->serdes_values = (struct serdes_reg_value *) kzalloc(prop_length, GFP_KERNEL);
				if (of_property_read_u32_array(node, "keystone2-serdes",
							       (u32 *) krio_priv->serdes_values,
							       (size_t) (prop_length/sizeof(u32)))) {
					kfree(krio_priv->serdes_values);
					krio_priv->serdes_values = NULL;
					dev_err(krio_priv->dev, "RapidIO SerDes couldn't be read from DT. Will use default values.\n");
				} else {
					krio_priv->serdes_val_len = prop_length/sizeof(*krio_priv->serdes_values);
				}
			}
		}
	}

       /* Max possible ports configurations per path_mode */
	if ((c->path_mode == 0 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_0) ||
	    (c->path_mode == 1 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_1) ||
	    (c->path_mode == 2 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_2) ||
	    (c->path_mode == 3 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_3) ||
	    (c->path_mode == 4 &&
	     c->ports & ~KEYSTONE_MAX_PORTS_PATH_MODE_4)) {
		dev_err(krio_priv->dev,
			"\"path_mode\" and \"ports\" configuration mismatch\n");
		ret = -EINVAL;
		goto error;
	}

	lanes = keystone_rio_get_lane_config(c->ports, c->path_mode);
	if (lanes < 0) {
		dev_err(krio_priv->dev,
			"cannot determine used SerDes lanes\n");
		ret = -EINVAL;
		goto error;
	}
	c->lanes = (u32)lanes;

	/* LSUs */
	if (of_property_read_u32_array(node, "lsu", &temp[0], 2)) {
		krio_priv->lsu_dio   = 0;
		krio_priv->lsu_maint = 0;
	} else {
		krio_priv->lsu_dio   = (u8) temp[0];
		krio_priv->lsu_maint = (u8) temp[1];
	}
	dev_dbg(krio_priv->dev, "using LSU #%d for DIO, LSU #%d for maintenance packets\n",
		krio_priv->lsu_dio, krio_priv->lsu_maint);

	/* RX resources */
	if (of_property_read_u32_array(node, "rxu_map_range", &temp[0], 2)) {
		krio_priv->rxu_map_start = KEYSTONE_RIO_RXU_MAP_MIN;
		krio_priv->rxu_map_end = KEYSTONE_RIO_RXU_MAP_MAX;
	} else if (temp[1] > KEYSTONE_RIO_RXU_MAP_MAX || temp[0] > temp[1]) {
		dev_err(krio_priv->dev, "invalid \"rxu_map_range\" parameter\n");
		ret = -EINVAL;
		goto error;
	} else {
		krio_priv->rxu_map_start = temp[0];
		krio_priv->rxu_map_end = temp[1];
	}
	dev_dbg(krio_priv->dev, "using RXU map range %lu-%lu\n",
		krio_priv->rxu_map_start, krio_priv->rxu_map_end);

	/* Mailboxes configuration */
	if (of_property_read_u32(node, "num-mboxes",
				 &krio_priv->num_mboxes) < 0) {
		dev_err(krio_priv->dev,
			"missing \"num-mboxes\" parameter\n");
		krio_priv->num_mboxes = 1;
	}

	if (krio_priv->num_mboxes > KEYSTONE_RIO_MAX_MBOX) {
		dev_err(krio_priv->dev,
			"wrong \"num_mboxes\" parameter value %d, set to %d\n",
			krio_priv->num_mboxes, KEYSTONE_RIO_MAX_MBOX);
			krio_priv->num_mboxes = KEYSTONE_RIO_MAX_MBOX;
	}

	for (mbox = 0; mbox < krio_priv->num_mboxes; mbox++) {
		(void) keystone_rio_get_mbox_defaults(mbox, node, krio_priv);
	}

	/* Interrupt config */
	i = of_property_match_string(node, "interrupt-names", "rio");
	c->rio_irq = irq_of_parse_and_map(node, i);
	if (c->rio_irq < 0) {
		dev_err(krio_priv->dev, "missing \"rio_irq\" parameter\n");
		ret = -ENOENT;
		goto error;
	}

	i = of_property_match_string(node, "interrupt-names", "lsu");
	c->lsu_irq = irq_of_parse_and_map(node, i);
	if (c->lsu_irq < 0) {
		dev_err(krio_priv->dev, "missing \"lsu_irq\" parameter\n");
		ret = -ENOENT;
		goto error;
	}

	i = of_property_match_string(node, "interrupt-names", "rio-starvation");
	c->rio_starvation_irq = irq_of_parse_and_map(node, i);
	if (c->rio_starvation_irq < 0) {
		dev_err(krio_priv->dev, "missing \"rio_starvation_irq\" parameter");
		ret = -ENOENT;
		goto error;
	}

	/* Packet forwarding */
	if (of_property_read_u32_array(node, "pkt-forward", &temp[0], 24)) {
		c->pkt_forwarding = 0;
	} else {
		c->pkt_forwarding = 1;
		for (i = 0; i < 8; i++) {
			c->routing_config[i].dev_id_low  = (u16) temp[(i * 3)];
			c->routing_config[i].dev_id_high = (u16) temp[(i * 3) + 1];
			c->routing_config[i].port        = (u8)  temp[(i * 3) + 2];
		}
	}

	if (of_property_read_u32(node, "comp_tag", &c->comp_tag)) {
		c->comp_tag = 0;
	}

	/* old garbage queues 8288 for pdsp recycling, remove after integration*/
	if (of_property_read_u32_array(node, "tx-garbage-queues",
				       c->tx_garbage_queues,
				       KEYSTONE_RIO_MAX_GARBAGE_QUEUES)) {
		dev_warn(krio_priv->dev,
			 "obsolete: missing \"tx-garbage-queues\" configuration\n");
	}

	if (of_property_read_u32_array(node, "tx-bin-queues",
				       c->tx_garbage_queues,
				       KEYSTONE_RIO_MAX_GARBAGE_QUEUES)) {
		dev_warn(krio_priv->dev,
			 "missing \"tx-bin-queues\" config\n");
	}
	dev_info(krio_priv->dev, "garbage queues %d-%d\n",
			c->tx_garbage_queues[0],
			c->tx_garbage_queues[KEYSTONE_RIO_MAX_GARBAGE_QUEUES-1]);

	if (of_property_read_u32(node, "free-fdq", &c->free_fdq)) {
		c->free_fdq = 0;
		dev_warn(krio_priv->dev,
			"missing \"free-fdq\" configuration\n");
	}

	dev_info(krio_priv->dev,
		"free_fdq: %x\n", (unsigned int)(c->free_fdq));

	if (of_find_property(node, "host", NULL)) {
		c->host = 1;
		dev_info(krio_priv->dev, "running in host mode\n");
	}

	goto exit;

error:
	if (krio_priv->serdes_sts_reg)
		iounmap(krio_priv->serdes_sts_reg);
	if (krio_priv->serdes_regs)
		iounmap(krio_priv->serdes_regs);
	if (krio_priv->jtagid_reg)
		iounmap(krio_priv->jtagid_reg);
	if (krio_priv->regs)
		iounmap(krio_priv->regs);

exit:
	return ret;
}

static int keystone_rio_port_chk(struct keystone_rio_data *krio_priv)
{
	u32 ports = krio_priv->ports_registering;
	u32 size  = krio_priv->board_rio_cfg.size;
	struct rio_mport *mport;

	/* Check ports' status (only the requested ones) */
	krio_priv->ports_registering = 0;
	while (ports) {
		int status;
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		/*
		 * Check the port status here before calling the generic RapidIO
		 * layer. Port status check is done in rio_mport_is_active() as
		 * well but we need to do it our way first due to some delays in
		 * hw initialization.
		 */
		status = keystone_rio_port_status(port, krio_priv);
		if (status == 0) {
			/*
			* Do not register mport if this is not initial port check
			* e.g. after reset device control symbol handling
			*/
			if (!krio_priv->mport[port]) {
				mport = keystone_rio_register_mport(port, size, krio_priv);
				if (!mport) {
					dev_err(krio_priv->dev,
						"failed to register mport %d\n", port);
					return -1;
				}
				dev_info(krio_priv->dev,
					"port RIO%d host_deviceid 0x%x registered\n",
					port, mport->host_deviceid);
			} else {
				dev_info(krio_priv->dev,
					"port RIO%d host_deviceid 0x%x ready\n",
					port, krio_priv->mport[port]->host_deviceid);
			}

			/* update routing after discovery/enumeration with new dev id */
			if (krio_priv->board_rio_cfg.pkt_forwarding)
				keystone_rio_port_set_routing(port, krio_priv);
		} else {
			krio_priv->ports_registering |= (1 << port);
			dev_dbg(krio_priv->dev, "port %d not ready\n", port);
		}
	}

	return krio_priv->ports_registering;
}


static void keystone_rio_port_chk_task(struct work_struct *work)
{
	struct keystone_rio_data *krio_priv = container_of(
		to_delayed_work(work),
		struct keystone_rio_data, port_chk_task);

	if (keystone_rio_port_chk(krio_priv))
		schedule_delayed_work(&krio_priv->port_chk_task,
				KEYSTONE_RIO_REGISTER_DELAY);
}

/*
 * Platform configuration setup
 */
static int keystone_rio_setup_controller(struct keystone_rio_data *krio_priv)
{
	u32 ports;
	u32 lanes;
	u32 p;
	u32 baud;
	u32 path_mode;
	u32 size = 0;
	int res = 0;
	char str[8];

	size      = krio_priv->board_rio_cfg.size;
	ports     = krio_priv->board_rio_cfg.ports;
	lanes     = krio_priv->board_rio_cfg.lanes;
	baud      = krio_priv->board_rio_cfg.serdes_baudrate;
	path_mode = krio_priv->board_rio_cfg.path_mode;

	dev_dbg(krio_priv->dev,
		"size = %d, ports = 0x%x, lanes = 0x%x, baud = %d, path_mode = %d\n",
		size, ports, lanes, baud, path_mode);

	switch (baud) {
	case KEYSTONE_RIO_BAUD_1_250:
		snprintf(str, sizeof(str), "1.25");
		break;
	case KEYSTONE_RIO_BAUD_2_500:
		snprintf(str, sizeof(str), "2.50");
		break;
	case KEYSTONE_RIO_BAUD_3_125:
		snprintf(str, sizeof(str), "3.125");
		break;
	case KEYSTONE_RIO_BAUD_5_000:
		snprintf(str, sizeof(str), "5.00");
		break;
	default:
		return -EINVAL;
	}

	dev_info(krio_priv->dev,
		 "initializing %s Gbps interface with port configuration %d\n",
		 str, path_mode);

	/* Hardware set up of the controller */
	res = keystone_rio_hw_init(baud, krio_priv);
	if (res < 0) {
		dev_err(krio_priv->dev,
			"initialization of SRIO hardware failed\n");
		return res;
	}

	/* Initialize port write interface */
	res = keystone_rio_port_write_init(krio_priv);
	if (res)
		return res;

	/* Disable all ports */
	for (p = 0; p < KEYSTONE_RIO_MAX_PORT; p++)
		keystone_rio_port_disable(p, krio_priv);

	/* Initialize interrupts */
	keystone_rio_interrupt_setup(krio_priv);

	/* Start the controller */
	keystone_rio_start(krio_priv);

	if (K2_SERDES(krio_priv)) {
		if (lanes > 0) {
			res = k2_rio_serdes_wait_lock(krio_priv, lanes);
			if (res < 0)
			    dev_info(krio_priv->dev,
				     "SerDes for lane mask 0x%x on %s Gbps not locked\n",
				     lanes, str);
		}
	}

	/* Use ports (only the requested ones) */
	while (ports) {
		u32 port = __ffs(ports);
		ports &= ~(1 << port);

		res = keystone_rio_port_init(port, path_mode, krio_priv);
		if (res < 0) {
			dev_err(krio_priv->dev,
				"initialization of port %d failed\n", port);
			return res;
		}

		/* Start the port */
		keystone_rio_port_enable(port, krio_priv);
	}

	krio_priv->ports_registering = krio_priv->board_rio_cfg.ports;
	if (keystone_rio_port_chk(krio_priv))
		schedule_delayed_work(&krio_priv->port_chk_task,
			KEYSTONE_RIO_REGISTER_DELAY);

	return 0;
}

static void keystone_rio_shutdown_controller(struct keystone_rio_data *krio_priv)
{
	int i, j;

	keystone_rio_interrupt_release(krio_priv);

	for (i = 0; i < KEYSTONE_RIO_MAX_MBOX; i++) {
		keystone_rio_close_tx_mbox(i, krio_priv);

		for (j = 0; j < KEYSTONE_RIO_MAX_LETTER; j++)
			keystone_rio_close_rx_mbox(i, j, krio_priv, NULL);
	}

	if (cancel_delayed_work_sync(&krio_priv->garbage_monitor_task)) {
		for (i = 0; i < KEYSTONE_RIO_MAX_GARBAGE_QUEUES; ++i) {
			iounmap(krio_priv->gq_peek_reg[i]);
			iounmap(krio_priv->gq_pop_push_reg[i]);
		}
		iounmap(krio_priv->qmss_mem_region);
		dev_info(krio_priv->dev, "stopped garbage monitor scheduler");
	}

	keystone_rio_stop(krio_priv);

	/* Wait current DMA transfers to finish */
	mdelay(2000);

	keystone_rio_blocks_disable(krio_priv);

	/* Shutdown associated SerDes */
	keystone_rio_serdes_shutdown(krio_priv);
}

static int keystone_rio_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct keystone_rio_data *krio_priv;
	int res = 0;
	dev_info(&pdev->dev, "KeyStone RapidIO driver %s\n", DRIVER_VER);

	if (!node) {
		dev_err(&pdev->dev, "could not find device info\n");
		return -EINVAL;
	}
	krio_priv = kzalloc(sizeof(struct keystone_rio_data), GFP_KERNEL);
	if (!krio_priv) {
		dev_err(&pdev->dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, krio_priv);
	krio_priv->dev = &(pdev->dev);

	/* Get default config from device tree */
	res = keystone_rio_get_controller_defaults(node, krio_priv);
	if (res < 0) {
		dev_err(&pdev->dev, "failed to get configuration\n");
		return res;
	}

	/* sRIO main driver (global ressources) */
	mutex_init(&krio_priv->lsu_lock);
	init_completion(&krio_priv->lsu_completion);
	INIT_DELAYED_WORK(&krio_priv->port_chk_task, keystone_rio_port_chk_task);
	INIT_DELAYED_WORK(&krio_priv->garbage_monitor_task,
			keystone_rio_garbage_monitor_handler);
	INIT_WORK(&krio_priv->pe_work, keystone_rio_pe_dpc);
	INIT_WORK(&krio_priv->reset_work, keystone_rio_reset_dpc);
	tasklet_init(&krio_priv->task, keystone_rio_chan_work_handler,
		     (unsigned long)krio_priv);

	/* Enable srio clock */
	krio_priv->clk = clk_get(&pdev->dev, "clk_srio");
	if (IS_ERR(krio_priv->clk)) {
		dev_err(&pdev->dev, "Unable to get Keystone SRIO clock\n");
		return -EBUSY;
	} else {
		clk_prepare_enable(krio_priv->clk);
		ndelay(100);
		clk_disable_unprepare(krio_priv->clk);
		ndelay(100);
		clk_prepare_enable(krio_priv->clk);
	}

#ifdef CONFIG_RAPIDIO_DEV
	/* Register userspace interface */
	res = rio_dev_init();
	if (res < 0)
		dev_err(krio_priv->dev, "rio_dev_init failed!\n");
#endif

#ifdef CONFIG_RAPIDIO_MEM_MAP
	keystone_rio_sync_buf_alloc(krio_priv);
	krio_priv->mem_map = rio_mem_map_alloc(&keystone_rio_mem_map_ops);
	if(krio_priv->mem_map) {
		krio_priv->mem_map->region[RIO_MEM_MAP_REGION_SYNC].get_addr = keystone_rio_sync_buf_addr_get;
		krio_priv->mem_map->region[RIO_MEM_MAP_REGION_SYNC].get_size = keystone_rio_sync_buf_len_get;
	}

	if (krio_priv->mem_map && (res = rio_mem_map_init(krio_priv->mem_map))) {
		dev_err(&pdev->dev, "RIO memory map init failed: %d\n", res);
		rio_mem_map_free(krio_priv->mem_map);
		krio_priv->mem_map = NULL;
	}
	if (krio_priv->mem_map) {
		phys_addr_t paddr =
			virt_to_phys(krio_priv->mem_map->block);
		dev_info(&pdev->dev, "RIO memory map at 0x%p (%pa)\n",
				krio_priv->mem_map->block, &paddr);
		krio_priv->mem_map->priv = krio_priv;
	}
#endif
	/* Setup the sRIO controller */
	res = keystone_rio_setup_controller(krio_priv);
	if (res < 0) {
#ifdef CONFIG_RAPIDIO_MEM_MAP
		if (krio_priv->mem_map)
			rio_mem_map_free(krio_priv->mem_map);
#endif
		return res;
	}

	return 0;
}

static void keystone_rio_shutdown(struct platform_device *pdev)
{
	struct keystone_rio_data *krio_priv = platform_get_drvdata(pdev);
	u32 port_id;

	keystone_rio_shutdown_controller(krio_priv);

	if (krio_priv->clk) {
		clk_disable_unprepare(krio_priv->clk);
		clk_put(krio_priv->clk);
	}

	for (port_id = 0; port_id < KEYSTONE_RIO_MAX_PORT; port_id++) {
		struct rio_mport *port = krio_priv->mport[port_id];

		if (!port)
			continue;

		rio_unregister_mport(port);
		kfree(port);
	}

	platform_set_drvdata(pdev, NULL);

#ifdef CONFIG_RAPIDIO_MEM_MAP
	if (krio_priv->mem_map)
		rio_mem_map_free(krio_priv->mem_map);
	kfree(sync_buf);
#endif

	if (krio_priv->serdes_values)
		kfree(krio_priv->serdes_values);

	kfree(krio_priv);
}

static int keystone_rio_remove(struct platform_device *pdev)
{
	keystone_rio_shutdown(pdev);
	return 0;
}

static struct of_device_id of_match[] = {
	{ .compatible = "ti,keystone-rapidio", },
	{},
};

MODULE_DEVICE_TABLE(of, of_match);

static struct platform_driver keystone_rio_driver  = {
	.driver = {
		.name	        = "keystone-rapidio",
		.owner	        = THIS_MODULE,
		.of_match_table	= of_match,
	},
	.probe	= keystone_rio_probe,
	.remove = keystone_rio_remove,
};

static int __init keystone_rio_module_init(void)
{
	return platform_driver_register(&keystone_rio_driver);
}

static void __exit keystone_rio_module_exit(void)
{
	platform_driver_unregister(&keystone_rio_driver);
}

module_init(keystone_rio_module_init);
module_exit(keystone_rio_module_exit);

MODULE_AUTHOR("Aurelien Jacquiot");
MODULE_DESCRIPTION("TI KeyStone RapidIO device driver");
MODULE_LICENSE("GPL");
