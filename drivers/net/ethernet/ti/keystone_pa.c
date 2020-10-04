/*
 * Copyright (C) 2012 Texas Instruments Incorporated
 * Authors: Sandeep Paulraj <s-paulraj@ti.com>
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

#include <linux/io.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/firmware.h>
#include <linux/spinlock.h>
#include <linux/highmem.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/net_tstamp.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/byteorder/generic.h>
#include <linux/platform_device.h>
#include <linux/keystone-dma.h>
#include <linux/phy.h>
#include <linux/errqueue.h>
#include <linux/ptp_classify.h>
#include <net/sctp/checksum.h>
#include <linux/clocksource.h>

#include "keystone_net.h"
#include "keystone_pa.h"
#include "keystone_pasahost.h"

#define BITS(x) (BIT(x) - 1)

#define BITS(x)			(BIT(x) - 1)

#define	PA_NETIF_FEATURES	(NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM)

#define PSTREAM_ROUTE_PDSP0	0

#define PA_PDSP_ALREADY_ACTIVE	0
#define PA_PDSP_RESET_RELEASED	1
#define PA_PDSP_NO_RESTART	2
#define PA_MAX_PDSP_ENABLE_LOOP_COUNT	100000

#define PA_INVALID_PORT			0xff

#define PA_STATE_RESET			0  /* Sub-system state reset */
#define PA_STATE_ENABLE			1  /* Sub-system state enable  */
#define PA_STATE_QUERY			2  /* Query the Sub-system state */
#define PA_STATE_INCONSISTENT		3  /* Sub-system is partially enabled */
#define PA_STATE_INVALID_REQUEST	4  /* Invalid state command to the Sub-system */
#define PA_STATE_ENABLE_FAILED		5  /*  The Sub-system did not respond after restart */

/* System Timestamp */
#define PAFRM_SRAM_SIZE			0x2000
#define PAFRM_SYS_TIMESTAMP_ADDR	0x6460

/* PDSP Versions */
#define PAFRM_PDSP_VERSION_BASE		0x7F04

#define DEVICE_PA_BASE				0x02000000
#define DEVICE_PA_REGION_SIZE			0x48000
#define DEVICE_PA_NUM_PDSPS			6

#define PA_MEM_PDSP_IRAM(pdsp)			((pdsp) * 0x8000)
#define PA_MEM_PDSP_SRAM(num)			((num) * 0x2000)
#define PA_REG_PKTID_SOFT_RESET	                0x00404
#define PA_REG_LUT2_SOFT_RESET	                0x00504
#define PA_REG_STATS_SOFT_RESET	                0x06004

#define PA_PDSP_CONST_REG_INDEX_C25_C24     0
#define PA_PDSP_CONST_REG_INDEX_C27_C26     1
#define PA_PDSP_CONST_REG_INDEX_C29_C28     2
#define PA_PDSP_CONST_REG_INDEX_C31_C30     3

/* The pdsp control register */
#define PA_REG_VAL_PDSP_CTL_DISABLE_PDSP	1
#define PA_REG_VAL_PDSP_CTL_RESET_PDSP	        0
#define PA_REG_VAL_PDSP_CTL_STATE               (1 << 15)
#define PA_REG_VAL_PDSP_CTL_ENABLE              (1 << 1)
#define PA_REG_VAL_PDSP_CTL_SOFT_RESET          (1 << 0)
#define PA_REG_VAL_PDSP_CTL_ENABLE_PDSP(pcval)	(((pcval) << 16)	\
						 | PA_REG_VAL_PDSP_CTL_ENABLE \
						 | PA_REG_VAL_PDSP_CTL_SOFT_RESET)

/* Number of mailbox slots for each PDPS */
#define PA_NUM_MAILBOX_SLOTS	4
#define TEST_SWINFO0_TIMESTAMP	0x12340002

#define PACKET_DROP	0
#define PACKET_PARSE	1
#define PACKET_HST	2

#define NT 32

#define PA_SGLIST_SIZE	3

const u32 pap_pdsp_const_reg_map[DEVICE_PA_NUM_PDSPS][4] =
{
	/* PDSP0: C24-C31 */
	{
		0x0000007F,		/* C25-C24 */
		0x0000006E,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000473		/* C31-C30 */
	},
	/* PDSP1: C24-C31 */
	{
		0x0001007F,		/* C25-C24 */
		0x00480040,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000473		/* C31-C30 */
	},
	/* PDSP2: C24-C31 */
	{
		0x0002007F,		/* C25-C24 */
		0x00490044,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000473		/* C31-C30 */
	},
	/* PDSP3: C24-C31 */
	{
		0x0003007F,		/* C25-C24 */
		0x0000006E,		/* C27-C26 */
		0x00000000,		/* C29-C28 */
		0x00000473		/* C31-C30 */
	},
	/* PDSP4: C24-C31 */
	{
		0x0070007F,		/* C25-C24 */
		0x00000003,		/* C27-C26 */
		0x04080404,		/* C29-C28 */
		0x00000473		/* C31-C30 */
	},
	/* PDSP5: C24-C31 */
	{
		0x0071007F,		/* C25-C24 */
		0x00000003,		/* C27-C26 */
		0x04080404,		/* C29-C28 */
		0x00000473		/* C31-C30 */
	}
};

struct pa_mailbox_regs {
	u32 pdsp_mailbox_slot0;
	u32 pdsp_mailbox_slot1;
	u32 pdsp_mailbox_slot2;
	u32 pdsp_mailbox_slot3;
};

struct pa_packet_id_alloc_regs {
	u32 revision;
	u32 soft_reset;
	u32 range_limit;
	u32 idvalue;
};

struct pa_lut2_control_regs {
	u32 revision;
	u32 soft_reset;
	u32 rsvd[6];
	u32 add_data0;
	u32 add_data1;
	u32 add_data2;
	u32 add_data3;
	u32 add_del_key;
	u32 add_del_control;
};

struct pa_pdsp_control_regs {
	u32 control;
	u32 status;
	u32 wakeup_enable;
	u32 cycle_count;
	u32 stall_count;
	u32 rsvd[3];
	u32 const_tbl_blk_index0;
	u32 const_tbl_blk_index1;
	u32 const_tbl_prog_pointer0;
	u32 const_tbl_prog_pointer1;
	u32 rsvd1[52];
};

struct pa_pdsp_timer_regs {
	u32 timer_control;
	u32 timer_load;
	u32 timer_value;
	u32 timer_interrupt;
	u32 rsvd[60];
};

struct pa_statistics_regs {
	u32 revision;
	u32 soft_reset;
	u32 incr_flags;
	u32 stats_capture;
	u32 rsvd[4];
	u32 stats_red[32];
};

#define	CSUM_OFFLOAD_NONE	0
#define	CSUM_OFFLOAD_HARD	1
#define	CSUM_OFFLOAD_SOFT	2

#define	PA_TXHOOK_ORDER	10
#define	PA_RXHOOK_ORDER	10

static DEFINE_MUTEX(pa_modules_lock);

enum pa_lut_type {
	PA_LUT_MAC,
	PA_LUT_IP
};

struct pa_lut_entry {
	int			index;
	bool			valid, in_use;
	union {
		struct netcp_addr	*naddr;
		u8			ip_proto;
	} u;
};

struct pa_timestamp_info {
	u32	mult;
	u32	shift;
	u64	system_offset;
};

struct pa_intf {
	struct pa_device		*pa_device;
	struct net_device		*net_device;
	bool				 tx_timestamp_enable;
	bool				 rx_timestamp_enable;
	struct netcp_tx_pipe		 tx_pipe;
	unsigned			 data_flow_num;
	unsigned			 data_queue_num;
	u32				 saved_ss_state;
	char				 tx_chan_name[24];
};

struct pa_device {
	int				force_no_hwtstamp;
	u32				csum_offload;
	struct pa_timestamp_info	timestamp_info;
	u32				multi_if;
	u32				mark_mcast_match[2];
	unsigned			cmd_flow_num;
	unsigned			cmd_queue_num;

	struct netcp_device		*netcp_device;
	struct device			*dev;
	struct clk			*clk;
	struct dma_chan			*pdsp0_tx_channel;
	struct dma_chan			*pdsp1_tx_channel;
	struct dma_chan			*rx_channel;
	const char			*rx_chan_name;
	unsigned			 rx_flow_base;
	unsigned			 rx_queue_base;

	struct pa_mailbox_regs __iomem		*reg_mailbox;
	struct pa_packet_id_alloc_regs __iomem	*reg_packet_id;
	struct pa_lut2_control_regs __iomem	*reg_lut2;
	struct pa_pdsp_control_regs __iomem	*reg_control;
	struct pa_pdsp_timer_regs   __iomem	*reg_timer;
	struct pa_statistics_regs   __iomem	*reg_stats;
	void __iomem				*pa_sram;
	void __iomem				*pa_iram;

	u8				*mc_list;
	u8				 addr_count;
	struct tasklet_struct		 task;
	spinlock_t			 lock;

	u32				 tx_cmd_queue_depth;
	u32				 tx_data_queue_depth;
	u32				 rx_pool_depth;
	u32				 rx_buffer_size;
	u32				 txhook_order;
	u32				 txhook_softcsum;
	u32				 rxhook_order;
	u32				 inuse_if_count;
	u32				 lut_inuse_count;
	struct pa_lut_entry		 *lut;
	u32				 lut_size;
	struct pa_lut_entry		 *ip_lut;
	u32				 ip_lut_size;
	netdev_features_t		 netif_features;
	const char			*pdsp_fw[DEVICE_PA_NUM_PDSPS];
	u32				 opened;
};

#define pa_from_module(data)	container_of(data, struct pa_device, module)
#define pa_to_module(pa)	(&(pa)->module)

struct pa_packet {
	struct scatterlist		 sg[PA_SGLIST_SIZE];
	int				 sg_ents;
	struct pa_device		*priv;
	struct dma_chan			*chan;
	struct dma_async_tx_descriptor	*desc;
	dma_cookie_t			 cookie;
	u32				 epib[4];
	u32				 psdata[6];
	struct completion		 complete;
	void				*data;
};

static void pdsp_fw_put(u32 *dest, const u32 *src, u32 wc)
{
	int i;

	for (i = 0; i < wc; i++)
		*dest++ = be32_to_cpu(*src++);
}

static inline void swiz_fwd(struct pa_frm_forward *fwd)
{
	fwd->flow_id = fwd->flow_id;
	fwd->queue   = cpu_to_be16(fwd->queue);

	if (fwd->forward_type == PAFRM_FORWARD_TYPE_HOST) {
		fwd->u.host.context      = cpu_to_be32(fwd->u.host.context);
		fwd->u.host.ctrl_bm  = fwd->u.host.ctrl_bm;
		fwd->u.host.multi_idx    = fwd->u.host.multi_idx;
		fwd->u.host.pa_pdsp_router = fwd->u.host.pa_pdsp_router;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_SA) {
		fwd->u.sa.sw_info_0 = cpu_to_be32(fwd->u.sa.sw_info_0);
		fwd->u.sa.sw_info_1 = cpu_to_be32(fwd->u.sa.sw_info_1);
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_SRIO) {
		fwd->u.srio.ps_info0 = cpu_to_be32(fwd->u.srio.ps_info0);
		fwd->u.srio.ps_info1 = cpu_to_be32(fwd->u.srio.ps_info1);
		fwd->u.srio.pkt_type = fwd->u.srio.pkt_type;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_ETH) {
		fwd->u.eth.ps_flags	= fwd->u.eth.ps_flags;
	} else if (fwd->forward_type == PAFRM_FORWARD_TYPE_PA) {
		fwd->u.pa.pa_dest	= fwd->u.pa.pa_dest;
		fwd->u.pa.custom_type	= fwd->u.pa.custom_type;
		fwd->u.pa.custom_idx	= fwd->u.pa.custom_idx;
	}

	fwd->forward_type = fwd->forward_type;
}

static inline void swizFcmd (struct pa_frm_command *fcmd)
{
	fcmd->command_result =  cpu_to_be32(fcmd->command_result);
	fcmd->command	     =  fcmd->command;
	fcmd->magic          =  fcmd->magic;
	fcmd->com_id         =  cpu_to_be16(fcmd->com_id);
	fcmd->ret_context    =  cpu_to_be32(fcmd->ret_context);
	fcmd->reply_queue    =  cpu_to_be16(fcmd->reply_queue);
	fcmd->reply_dest     =  fcmd->reply_dest;
	fcmd->flow_id        =  fcmd->flow_id;
}

static inline void swizAl1 (struct pa_frm_cmd_add_lut1 *al1)
{
	al1->index         =  al1->index;
	al1->type          =  al1->type;
	al1->cust_index    =  al1->cust_index;

	if (al1->type == PAFRM_COM_ADD_LUT1_STANDARD) {
		al1->u.eth_ip.etype = cpu_to_be16(al1->u.eth_ip.etype);
		al1->u.eth_ip.vlan  = cpu_to_be16(al1->u.eth_ip.vlan);
		al1->u.eth_ip.spi   = cpu_to_be32(al1->u.eth_ip.spi);
		al1->u.eth_ip.flow  = cpu_to_be32(al1->u.eth_ip.flow);

		if (al1->u.eth_ip.key & PAFRM_LUT1_KEY_MPLS)
			al1->u.eth_ip.pm.mpls     =  cpu_to_be32(al1->u.eth_ip.pm.mpls);
		else {
			al1->u.eth_ip.pm.ports[0] =  cpu_to_be16(al1->u.eth_ip.pm.ports[0]);
			al1->u.eth_ip.pm.ports[1] =  cpu_to_be16(al1->u.eth_ip.pm.ports[1]);
		}

		al1->u.eth_ip.proto_next  =  al1->u.eth_ip.proto_next;
		al1->u.eth_ip.tos_tclass  =  al1->u.eth_ip.tos_tclass;
		al1->u.eth_ip.inport      =  al1->u.eth_ip.inport;
		al1->u.eth_ip.key         =  al1->u.eth_ip.key;
		al1->u.eth_ip.match_flags =  cpu_to_be16(al1->u.eth_ip.match_flags);
	} else if (al1->type == PAFRM_COM_ADD_LUT1_SRIO) {
		al1->u.srio.src_id	= cpu_to_be16(al1->u.srio.src_id);
		al1->u.srio.dest_id	= cpu_to_be16(al1->u.srio.dest_id);
		al1->u.srio.etype	= cpu_to_be16(al1->u.srio.etype);
		al1->u.srio.vlan	= cpu_to_be16(al1->u.srio.vlan);
		al1->u.srio.pri         = al1->u.srio.pri;
		al1->u.srio.type_param1 = cpu_to_be16(al1->u.srio.type_param1);
		al1->u.srio.type_param2 = al1->u.srio.type_param2;
		al1->u.srio.key         = al1->u.srio.key;
		al1->u.srio.match_flags = cpu_to_be16(al1->u.srio.match_flags);
		al1->u.srio.next_hdr    = al1->u.srio.next_hdr;
		al1->u.srio.next_hdr_offset = cpu_to_be16(al1->u.srio.next_hdr_offset);
	} else {
		al1->u.custom.etype		=  cpu_to_be16(al1->u.custom.etype);
		al1->u.custom.vlan		=  cpu_to_be16(al1->u.custom.vlan);
		al1->u.custom.key		=  al1->u.custom.key;
		al1->u.custom.match_flags	=  cpu_to_be16(al1->u.custom.match_flags);
	}

	swiz_fwd(&(al1->match));
	swiz_fwd(&(al1->next_fail));
}

static int pa_conv_routing_info(struct	pa_frm_forward *fwd_info,
			 struct	pa_route_info *route_info,
			 int cmd_dest, u16 fail_route)
{
	u8 *pcmd = NULL;
	fwd_info->flow_id = route_info->flow_id;
	fwd_info->queue   = route_info->queue;

	if (route_info->dest == PA_DEST_HOST) {
		fwd_info->forward_type   = PAFRM_FORWARD_TYPE_HOST;
		fwd_info->u.host.context = route_info->sw_info_0;

		if (route_info->route_type)
			fwd_info->u.host.ctrl_bm |=
				PAFRM_ROUTING_IF_DEST_SELECT_ENABLE;

		if (route_info->route_type == PA_ROUTE_INTF_FLOW)
			fwd_info->u.host.ctrl_bm |=
				PAFRM_ROUTING_FLOW_IF_BASE_ENABLE;

		if (route_info->m_route_index >= 0) {
			if (route_info->m_route_index >=
			    PA_MAX_MULTI_ROUTE_SETS)
				return (PA_ERR_CONFIG);

			fwd_info->u.host.ctrl_bm |= PAFRM_MULTIROUTE_ENABLE;
			fwd_info->u.host.multi_idx = route_info->m_route_index;
			fwd_info->u.host.pa_pdsp_router	= PAFRM_DEST_PA_M_0;
		}
		pcmd = fwd_info->u.host.cmd;
	} else if (route_info->dest == PA_DEST_DISCARD)	{
		fwd_info->forward_type = PAFRM_FORWARD_TYPE_DISCARD;
	} else if (route_info->dest == PA_DEST_EMAC) {
		fwd_info->forward_type = PAFRM_FORWARD_TYPE_ETH;
		fwd_info->u.eth.ps_flags = (route_info->pkt_type_emac_ctrl &
					    PA_EMAC_CTRL_CRC_DISABLE)?
			PAFRM_ETH_PS_FLAGS_DISABLE_CRC:0;
		fwd_info->u.eth.ps_flags |= ((route_info->pkt_type_emac_ctrl &
					      PA_EMAC_CTRL_PORT_MASK) <<
					     PAFRM_ETH_PS_FLAGS_PORT_SHIFT);
	} else if (fail_route) {
		return (PA_ERR_CONFIG);

	} else if (((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT1) &&
		    (route_info->custom_type != PA_CUSTOM_TYPE_LUT2)) ||
		   ((route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) &&
		    (route_info->custom_type != PA_CUSTOM_TYPE_LUT1))) {

		/* Custom Error check */
		if (((route_info->custom_type == PA_CUSTOM_TYPE_LUT1) &&
		     (route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT1)) ||
		    ((route_info->custom_type == PA_CUSTOM_TYPE_LUT2) &&
		     (route_info->custom_index >= PA_MAX_CUSTOM_TYPES_LUT2)))
			return(PA_ERR_CONFIG);

		fwd_info->forward_type = PAFRM_FORWARD_TYPE_PA;
		fwd_info->u.pa.custom_type = (u8)route_info->custom_type;
		fwd_info->u.pa.custom_idx  = route_info->custom_index;

		if (route_info->dest == PA_DEST_CONTINUE_PARSE_LUT2) {
			fwd_info->u.pa.pa_dest = PAFRM_DEST_PA_C2;
		} else {
			/*
			 * cmd_dest is provided by calling function
			 * There is no need to check error case
			 */
			fwd_info->u.pa.pa_dest = (cmd_dest == PA_CMD_TX_DEST_0)?
				PAFRM_DEST_PA_C1_1:PAFRM_DEST_PA_C1_2;
		}
	} else if (route_info->dest == PA_DEST_SASS) {
		fwd_info->forward_type   = PAFRM_FORWARD_TYPE_SA;
		fwd_info->u.sa.sw_info_0 = route_info->sw_info_0;
		fwd_info->u.sa.sw_info_1 = route_info->sw_info_1;
		pcmd = fwd_info->u.sa.cmd;
	} else if (route_info->dest == PA_DEST_SRIO) {
		fwd_info->forward_type		= PAFRM_FORWARD_TYPE_SRIO;
		fwd_info->u.srio.ps_info0	= route_info->sw_info_0;
		fwd_info->u.srio.ps_info1	= route_info->sw_info_1;
		fwd_info->u.srio.pkt_type	= route_info->pkt_type_emac_ctrl;
	} else {
		return (PA_ERR_CONFIG);
	}

	if (pcmd && route_info->pcmd) {
		struct pa_cmd_info *pacmd = route_info->pcmd;
		struct pa_patch_info *patch_info;
		struct pa_cmd_set *cmd_set;

		switch (pacmd->cmd) {
		case PA_CMD_PATCH_DATA:
			patch_info = &pacmd->params.patch;
			if ((patch_info->n_patch_bytes > 2) ||
			    (patch_info->overwrite) ||
			    (patch_info->patch_data == NULL))
				return (PA_ERR_CONFIG);

			pcmd[0] = PAFRM_RX_CMD_CMDSET;
			pcmd[1] = patch_info->n_patch_bytes;
			pcmd[2] = patch_info->patch_data[0];
			pcmd[3] = patch_info->patch_data[1];
			break;

		case PA_CMD_CMDSET:
			cmd_set = &pacmd->params.cmd_set;
			if(cmd_set->index >= PA_MAX_CMD_SETS)
				return (PA_ERR_CONFIG);

			pcmd[0] = PAFRM_RX_CMD_CMDSET;
			pcmd[1] = (u8)cmd_set->index;
			break;
		default:
			return(PA_ERR_CONFIG);
		}
	}
	return (PA_OK);
}

static int keystone_pa_reset(struct pa_device *pa_dev)
{
	struct pa_packet_id_alloc_regs __iomem	*packet_id_regs = pa_dev->reg_packet_id;
	struct pa_lut2_control_regs __iomem	*lut2_regs = pa_dev->reg_lut2;
	struct pa_statistics_regs   __iomem	*stats_regs = pa_dev->reg_stats;
	u32 i;

	/* Reset and disable all PDSPs */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		struct pa_pdsp_control_regs __iomem *ctrl_reg = &pa_dev->reg_control[i];
		__raw_writel(PA_REG_VAL_PDSP_CTL_RESET_PDSP,
			     &ctrl_reg->control);

		while((__raw_readl(&ctrl_reg->control)
		       & PA_REG_VAL_PDSP_CTL_STATE));
	}

	/* Reset packet Id */
	__raw_writel(1, &packet_id_regs->soft_reset);

	/* Reset LUT2 */
	__raw_writel(1, &lut2_regs->soft_reset);

	/* Reset statistic */
	__raw_writel(1, &stats_regs->soft_reset);

	/* Reset timers */
	for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
		struct pa_pdsp_timer_regs __iomem *timer_reg = &pa_dev->reg_timer[i];
		__raw_writel(0, &timer_reg->timer_control);
	}

	return 0;
}

/*
 *  Convert a raw PA timer count to nanoseconds
 */
static inline u64 tstamp_raw_to_ns(struct pa_device *pa_dev, u32 lo, u32 hi)
{
	u32 mult = pa_dev->timestamp_info.mult;
	u32 shift = pa_dev->timestamp_info.shift;
	u64 result;

	/* Minimize overflow errors by doing this in pieces */
	result  = ((u64)lo * mult) >> shift;
	result += ((u64)hi << (32 - shift)) * mult;

	return result;
}

static u64 pa_to_sys_time(struct pa_device *pa_dev, u64 pa_ns)
{
	s64 temp;
	u64 result;

	/* we need to compute difference from wallclock
	*  to time from boot dynamically since
	*  it will change whenever absolute time is adjusted by
	*  protocols above (ntp, ptpv2)
	*/

	temp = ktime_to_ns(ktime_get_monotonic_offset());
	result = (u64)((s64)pa_dev->timestamp_info.system_offset - temp +
			(s64)pa_ns);

	return result;
}

/*
 * calibrate the PA timer to the system time
 * ktime_get gives montonic time 
 * ktime_to_ns converts ktime to ns
 * this needs to be called before doing conversions
 */
static void pa_calibrate_with_system_timer(struct pa_device *pa_dev)
{
	struct pa_pdsp_timer_regs __iomem *timer_reg = &pa_dev->reg_timer[0];
	ktime_t ktime1, ktime2;
	u32 timer, low1, low2, high;
	u32 pa_lo, pa_hi;
	u64 pa_ns, sys_ns1, sys_ns2;
	int count;

	/* Obtain the internal PA timestamp counter values */
	count = 0;
	do {
		__iormb();
		ktime1 = ktime_get();
		low1  = __raw_readl(pa_dev->pa_sram + 0x6460);
		__iormb();
		high  = __raw_readl(pa_dev->pa_sram + 0x6464);
		timer = __raw_readl(&timer_reg->timer_value);
		__iormb();
		low2  = __raw_readl(pa_dev->pa_sram + 0x6460);
		ktime2 = ktime_get();
	} while (unlikely(low1 != low2) && (++count < 32));

	/* Convert the PA timestamp to nanoseconds */
	pa_lo = (low1 << 16) | (0x0000ffff - (timer & 0x0000ffff));
	pa_hi = (high << 16) | (low1 >> 16);
	pa_ns   = tstamp_raw_to_ns(pa_dev, pa_lo, pa_hi);

	/* Convert the system time values to nanoseconds */
	sys_ns1 = ktime_to_ns(ktime1);
	sys_ns2 = ktime_to_ns(ktime2);

	/* Compute the PA-to-system offset */
	pa_dev->timestamp_info.system_offset = sys_ns1 +
		((sys_ns2 - sys_ns1) / 2) - pa_ns;
}

static void pa_get_version(struct pa_device *pa_dev)
{
	u32 version;

	version = __raw_readl(pa_dev->pa_sram + PAFRM_PDSP_VERSION_BASE);

	dev_info(pa_dev->dev, "Using Packet Accelerator Firmware version "
				"0x%08x\n", version);
}

static int pa_pdsp_run(struct pa_device *pa_dev, int pdsp)
{
	struct pa_pdsp_control_regs __iomem *ctrl_reg = &pa_dev->reg_control[pdsp];
	struct pa_mailbox_regs __iomem *mailbox_reg = &pa_dev->reg_mailbox[pdsp];
	u32 i, v;

	/* Check for enabled PDSP */
	v = __raw_readl(&ctrl_reg->control);
	if ((v & PA_REG_VAL_PDSP_CTL_ENABLE) ==
	    PA_REG_VAL_PDSP_CTL_ENABLE) {
		/* Already enabled */
		return (PA_PDSP_ALREADY_ACTIVE);
	}

	/* Clear the mailbox */
	__raw_writel(0, &mailbox_reg->pdsp_mailbox_slot0);

	/* Set PDSP PC to 0, enable the PDSP */
	__raw_writel(PA_REG_VAL_PDSP_CTL_ENABLE |
		     PA_REG_VAL_PDSP_CTL_SOFT_RESET,
		     &ctrl_reg->control);

	/* Wait for the mailbox to become non-zero */
	for (i = 0; i < PA_MAX_PDSP_ENABLE_LOOP_COUNT; i++)
		v = __raw_readl(&mailbox_reg->pdsp_mailbox_slot0);
		if (v != 0)
			return (PA_PDSP_RESET_RELEASED);

	return (PA_PDSP_NO_RESTART);
}

static int keystone_pa_reset_control(struct pa_device *pa_dev, int new_state)
{
	struct pa_mailbox_regs __iomem *mailbox_reg = &pa_dev->reg_mailbox[0];
	int do_global_reset = 1;
	int i, res;
	int ret;

	if (new_state == PA_STATE_ENABLE) {
		ret = PA_STATE_ENABLE;

		/*
		 * Do nothing if a pdsp is already out of reset.
		 * If any PDSPs are out of reset
		 * a global init is not performed
		 */
		for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
			res = pa_pdsp_run(pa_dev, i);

			if (res == PA_PDSP_ALREADY_ACTIVE)
				do_global_reset = 0;

			if (res == PA_PDSP_NO_RESTART) {
				ret = PA_STATE_ENABLE_FAILED;
				do_global_reset = 0;
			}
		}

		/* If global reset is required any PDSP can do it */
		if (do_global_reset) {
			__raw_writel(1, &mailbox_reg->pdsp_mailbox_slot1);
			wmb();
			__raw_writel(0, &mailbox_reg->pdsp_mailbox_slot0);
			mb();

			while (__raw_readl(&mailbox_reg->pdsp_mailbox_slot1) != 0)
				rmb();

			for (i = 1; i < DEVICE_PA_NUM_PDSPS; i++) {
				struct pa_mailbox_regs __iomem *mbox_reg =
					&pa_dev->reg_mailbox[i];
				__raw_writel(0,
					     &mbox_reg->pdsp_mailbox_slot0);
			}
			wmb();
		} else {
			for (i = 0; i < DEVICE_PA_NUM_PDSPS; i++) {
				struct pa_mailbox_regs __iomem *mbox_reg =
					&pa_dev->reg_mailbox[i];
				__raw_writel(0,
					     &mbox_reg->pdsp_mailbox_slot0);
			}
			wmb();
		}

		return (ret);
	}

	return (PA_STATE_INVALID_REQUEST);
}

static int keystone_pa_set_firmware(struct pa_device *pa_dev,
			     int pdsp, const unsigned int *buffer, int len)
{
	struct pa_pdsp_control_regs __iomem *ctrl_reg = &pa_dev->reg_control[pdsp];

	if ((pdsp < 0) || (pdsp >= DEVICE_PA_NUM_PDSPS))
		return -EINVAL;

	pdsp_fw_put((u32 *)(pa_dev->pa_iram + PA_MEM_PDSP_IRAM(pdsp)), buffer,
		    len >> 2);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C25_C24],
		     &ctrl_reg->const_tbl_blk_index0);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C27_C26],
		     &ctrl_reg->const_tbl_blk_index1);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C29_C28],
		     &ctrl_reg->const_tbl_prog_pointer0);

	__raw_writel(pap_pdsp_const_reg_map[pdsp][PA_PDSP_CONST_REG_INDEX_C31_C30],
		     &ctrl_reg->const_tbl_prog_pointer1);

	return 0;
}

static inline void pa_free_packet(struct pa_device *pa_dev, void *pkt)
{
	devm_kfree(pa_dev->dev, pkt);
}

static struct pa_packet *pa_alloc_packet(struct pa_device *pa_dev,
					 unsigned cmd_size,
					 struct dma_chan *dma_chan)
{
	struct pa_packet *p_info;

	p_info = devm_kzalloc(pa_dev->dev, sizeof(*p_info) + cmd_size,
			      GFP_ATOMIC);
	if (!p_info)
		return NULL;

	p_info->priv = pa_dev;
	p_info->data = p_info + 1;
	p_info->chan = dma_chan;

	sg_init_table(p_info->sg, PA_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], p_info->data, cmd_size);
	return p_info;
}

static void pa_tx_dma_callback(void *data)
{
	struct pa_packet *p_info = data;
	struct pa_device *pa_dev = p_info->priv;
	enum dma_status status;
	unsigned long irqsave;
	dma_cookie_t cookie;

	spin_lock_irqsave(&pa_dev->lock, irqsave);
	cookie = p_info->cookie;
	spin_unlock_irqrestore(&pa_dev->lock, irqsave);

	if (unlikely(cookie <= 0))
		WARN(1, "invalid dma cookie == %d", cookie);
	else {
		status = dma_async_is_tx_complete(p_info->chan,
						  cookie, NULL, NULL);
		WARN((status != DMA_COMPLETE),
				"dma completion failure, status == %d", status);
	}

	dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);
	p_info->desc = NULL;
	pa_free_packet(pa_dev, p_info);
}

static int pa_submit_tx_packet(struct pa_packet *p_info)
{
	unsigned flags = DMA_HAS_EPIB | DMA_HAS_PSINFO;
	struct pa_device *pa_dev = p_info->priv;
	unsigned long irqsave;
	int ret;

	ret = dma_map_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);
	if (ret < 0)
		return ret;

	p_info->desc = dmaengine_prep_slave_sg(p_info->chan, p_info->sg, 3,
					       DMA_TO_DEVICE, flags);
	if (IS_ERR_OR_NULL(p_info->desc)) {
		dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);
		return PTR_ERR(p_info->desc);
	}

	p_info->desc->callback = pa_tx_dma_callback;
	p_info->desc->callback_param = p_info;

	spin_lock_irqsave(&pa_dev->lock, irqsave);
	p_info->cookie = dmaengine_submit(p_info->desc);
	spin_unlock_irqrestore(&pa_dev->lock, irqsave);

	return dma_submit_error(p_info->cookie) ? p_info->cookie : 0;
}

#define	PA_CONTEXT_MASK		0xffff0000
#define	PA_CONTEXT_CONFIG	0xdead0000
#define	PA_CONTEXT_TSTAMP	0xbeef0000

#define	TSTAMP_TIMEOUT	(HZ * 5)	/* 5 seconds (arbitrary) */

struct tstamp_pending {
	struct list_head	 list;
	u32			 context;
	struct sock		*sock;
	struct sk_buff		*skb;
	struct pa_device	*pa_dev;
	struct timer_list	 timeout;
};

static spinlock_t		 tstamp_lock;
static atomic_t			 tstamp_sequence = ATOMIC_INIT(0);
static struct list_head		 tstamp_pending = LIST_HEAD_INIT(tstamp_pending);

static struct tstamp_pending *tstamp_remove_pending(u32 context)
{
	struct tstamp_pending	*pend;

	spin_lock(&tstamp_lock);
	list_for_each_entry(pend, &tstamp_pending, list) {
		if (pend->context == context) {
			del_timer(&pend->timeout);
			list_del(&pend->list);
			spin_unlock(&tstamp_lock);
			return pend;
		}
	}
	spin_unlock(&tstamp_lock);

	return NULL;
}

static void tstamp_complete(u32, struct pa_packet *);

static void tstamp_purge_pending(struct pa_device *pa_dev)
{
	struct tstamp_pending	*pend;
	int			 found;

	/* This is ugly and inefficient, but very rarely executed */
	do {
		found = 0;

		spin_lock(&tstamp_lock);
		list_for_each_entry(pend, &tstamp_pending, list) {
			if (pend->pa_dev == pa_dev) {
				found = 1;
				break;
			}
		}
		spin_unlock(&tstamp_lock);

		if (found)
			tstamp_complete(pend->context, NULL);
	} while(found);
}

static void tstamp_timeout(unsigned long context)
{
	tstamp_complete((u32)context, NULL);
}

static int tstamp_add_pending(struct tstamp_pending *pend)
{
	init_timer(&pend->timeout);
	pend->timeout.expires = jiffies + TSTAMP_TIMEOUT;
	pend->timeout.function = tstamp_timeout;
	pend->timeout.data = (unsigned long)pend->context;

	spin_lock(&tstamp_lock);
	add_timer(&pend->timeout);
	list_add_tail(&pend->list, &tstamp_pending);
	spin_unlock(&tstamp_lock);

	return 0;
}

static void tstamp_complete(u32 context, struct pa_packet *p_info)
{
	struct pa_device	*pa_dev;
	struct tstamp_pending	*pend;
	struct sock_exterr_skb 	*serr;
	struct sk_buff 		*skb;
	struct skb_shared_hwtstamps *sh_hw_tstamps;
	u64			 pa_ns;
	u64			 sys_time;
	int			 err;

	pend = tstamp_remove_pending(context);
	if (!pend)
		return;

	pa_dev = pend->pa_dev;
	skb = pend->skb;
	if (!p_info) {
		dev_warn(pa_dev->dev, "Timestamp completion timeout\n");
		kfree_skb(skb);
	} else {
		pa_ns = tstamp_raw_to_ns(pa_dev,
				p_info->epib[0], p_info->epib[2]);
		sys_time = pa_to_sys_time(pa_dev, pa_ns);

		sh_hw_tstamps = skb_hwtstamps(skb);
		memset(sh_hw_tstamps, 0, sizeof(*sh_hw_tstamps));
		sh_hw_tstamps->hwtstamp = ns_to_ktime(pa_ns);
		sh_hw_tstamps->syststamp = ns_to_ktime(sys_time);

		serr = SKB_EXT_ERR(skb);
		memset(serr, 0, sizeof(*serr));
		serr->ee.ee_errno = ENOMSG;
		serr->ee.ee_origin = SO_EE_ORIGIN_TIMESTAMPING;

		err = sock_queue_err_skb(pend->sock, skb);
		if (err)
			kfree_skb(skb);
	}

	kfree(pend);
}

static void pa_rx_complete(void *param)
{
	struct pa_packet *p_info = param;
	struct pa_device *pa_dev = p_info->priv;
	struct pa_frm_command *fcmd;

	dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);

	switch (p_info->epib[1] & PA_CONTEXT_MASK) {
	case PA_CONTEXT_CONFIG:
		fcmd = p_info->data;
		swizFcmd(fcmd);

		if (fcmd->command_result != PAFRM_COMMAND_RESULT_SUCCESS) {
			dev_dbg(pa_dev->dev, "Command Result = 0x%x\n", fcmd->command_result);
			dev_dbg(pa_dev->dev, "Command = 0x%x\n", fcmd->command);
			dev_dbg(pa_dev->dev, "Magic = 0x%x\n", fcmd->magic);
			dev_dbg(pa_dev->dev, "Com ID = 0x%x\n", fcmd->com_id);
			dev_dbg(pa_dev->dev, "ret Context = 0x%x\n", fcmd->ret_context);
			dev_dbg(pa_dev->dev, "Flow ID = 0x%x\n", fcmd->flow_id);
			dev_dbg(pa_dev->dev, "reply Queue = 0x%x\n", fcmd->reply_queue);
			dev_dbg(pa_dev->dev, "reply dest = 0x%x\n", fcmd->reply_dest);
		}
		dev_dbg(pa_dev->dev, "command response complete\n");
		break;

	case PA_CONTEXT_TSTAMP:
		tstamp_complete(p_info->epib[1], p_info);
		break;

	default:
		dev_warn(pa_dev->dev, "bad response context, got 0x%08x\n", p_info->epib[1]);
		break;
	}

	p_info->desc = NULL;
	pa_free_packet(pa_dev, p_info);
}

/* Release a free receive buffer */
static void pa_rxpool_free(void *arg, unsigned q_num, unsigned bufsize,
		struct dma_async_tx_descriptor *desc)
{
	struct pa_device *pa_dev = arg;
	struct pa_packet *p_info = desc->callback_param;

	dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);
	p_info->desc = NULL;
	pa_free_packet(pa_dev, p_info);
}

static void pa_chan_work_handler(unsigned long data)
{
	struct pa_device *pa_dev = (struct pa_device *)data;

	dma_poll(pa_dev->rx_channel, -1);
	dma_rxfree_refill(pa_dev->rx_channel);
	dmaengine_resume(pa_dev->rx_channel);
}

static void pa_chan_notify(struct dma_chan *dma_chan, void *arg)
{
	struct pa_device *pa_dev = arg;

	dmaengine_pause(pa_dev->rx_channel);
	tasklet_schedule(&pa_dev->task);
	return;
}

/* Allocate a free receive buffer */
static struct dma_async_tx_descriptor *pa_rxpool_alloc(void *arg,
		unsigned q_num, unsigned bufsize)
{
	struct pa_device *pa_dev = arg;
	struct dma_async_tx_descriptor *desc;
	struct dma_device *device;
	u32 err = 0;

	struct pa_packet *rx;

	rx = pa_alloc_packet(pa_dev, bufsize, pa_dev->rx_channel);
	if (!rx) {
		dev_err(pa_dev->dev, "could not allocate cmd rx packet\n");
		return NULL;
	}

	rx->sg_ents = 2 + dma_map_sg(pa_dev->dev, &rx->sg[2],
				1, DMA_FROM_DEVICE);
	if (rx->sg_ents != 3) {
		dev_err(pa_dev->dev, "dma map failed\n");
		pa_free_packet(pa_dev, rx);
		return NULL;
	}

	device = rx->chan->device;
	desc = dmaengine_prep_slave_sg(rx->chan, rx->sg, 3, DMA_DEV_TO_MEM,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);
	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(pa_dev->dev, &rx->sg[2], 1, DMA_FROM_DEVICE);
		pa_free_packet(pa_dev, rx);
		err = PTR_ERR(desc);
		if (err != -ENOMEM) {
			dev_err(pa_dev->dev,
				"dma prep failed, error %d\n", err);
		}
		return NULL;
	}

	desc->callback_param = rx;
	desc->callback = pa_rx_complete;
	rx->cookie = desc->cookie;
	return desc;
}

static struct pa_lut_entry *pa_lut_alloc(struct pa_device *pa_dev,
					 enum pa_lut_type type, bool backwards)
{
	struct pa_lut_entry *lut_table, *entry;
	u32 lut_size;
	int i;

	if (type == PA_LUT_MAC) {
		lut_table = pa_dev->lut;
		lut_size = pa_dev->lut_size;
	} else {
		lut_table = pa_dev->ip_lut;
		lut_size = pa_dev->ip_lut_size;
	}

	if (!backwards) {
		for (i = 0; i < lut_size; i++) {
			entry = lut_table + i;
			if (!entry->valid || entry->in_use)
				continue;
			entry->in_use = true;
			return entry;
		}
	} else {
		for (i = lut_size - 1; i >= 0; i--) {
			entry = lut_table + i;
			if (!entry->valid || entry->in_use)
				continue;
			entry->in_use = true;
			return entry;
		}
	}

	return NULL;
}

static inline int pa_lut_entry_count(enum netcp_addr_type type)
{
	return (type == ADDR_DEV || type == ADDR_UCAST || type == ADDR_ANY) ?
		3 : 1;
}

static void pa_format_cmd_hdr(struct pa_device *dev,
		struct pa_frm_command *fcmd, u8 cmd, u16 cmd_id, u32 ctx)
{
	memset(fcmd, 0, sizeof(*fcmd));
	fcmd->command		= cmd;
	fcmd->magic		= PAFRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id		= cpu_to_be16(cmd_id);
	fcmd->ret_context	= cpu_to_be32(ctx);
	fcmd->flow_id		= dev->cmd_flow_num;
	fcmd->reply_queue	= cpu_to_be16(dev->cmd_queue_num);
	fcmd->reply_dest	= PAFRM_DEST_PKTDMA;
}

static int keystone_pa_add_ip_proto(struct pa_device *pa_dev, int index,
					u8 proto, int rule)
{
	struct pa_route_info route_info, fail_info;
	struct pa_frm_cmd_add_lut1 *al1;
	u32 context = PA_CONTEXT_CONFIG;
	struct pa_frm_command *fcmd;
	unsigned flow_num, q_num;
	struct pa_packet *tx;
	int size, ret;

	dev_dbg(pa_dev->dev, "%s: index %d, rule %d, proto %d\n",
		 __func__, index, rule, proto);

	memset(&fail_info, 0, sizeof(fail_info));
	memset(&route_info, 0, sizeof(route_info));

	q_num = pa_dev->rx_queue_base;
	flow_num = pa_dev->rx_flow_base;

	if (rule == PACKET_HST) {
		route_info.dest			= PA_DEST_HOST;
		route_info.flow_id		= flow_num;
		route_info.queue		= q_num;
		route_info.m_route_index	= -1;
		route_info.route_type		= PA_ROUTE_INTF_FLOW;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= flow_num;
		fail_info.queue			= q_num;
		fail_info.m_route_index		= -1;
		fail_info.route_type		= PA_ROUTE_INTF_FLOW;
	} else if (rule == PACKET_PARSE) {
		route_info.dest			= PA_DEST_CONTINUE_PARSE_LUT2;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= flow_num;
		fail_info.queue			= q_num;
		fail_info.m_route_index		= -1;
		fail_info.route_type		= PA_ROUTE_INTF_FLOW;
	} else if (rule == PACKET_DROP) {
		route_info.dest			= PA_DEST_DISCARD;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_DISCARD;
		fail_info.m_route_index		= -1;
	}

	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_cmd_add_lut1) + 4);
	tx = pa_alloc_packet(pa_dev, size, pa_dev->pdsp1_tx_channel);
	if (!tx) {
		dev_err(pa_dev->dev, "%s: could not allocate cmd tx packet\n",
			__func__);
		return -ENOMEM;
	}

	fcmd = tx->data;
	al1 = (struct pa_frm_cmd_add_lut1 *) &(fcmd->cmd);
	memset(al1, 0, sizeof(*al1));

	pa_format_cmd_hdr(pa_dev, fcmd, PAFRM_CONFIG_COMMAND_ADDREP_LUT1,
			PA_COMID_L3, context);

	al1->index = index;
	al1->type = PAFRM_COM_ADD_LUT1_STANDARD;

	al1->u.eth_ip.proto_next = proto;
	al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_PROTO;
	al1->u.eth_ip.match_flags = cpu_to_be16(al1->u.eth_ip.match_flags);

	ret = pa_conv_routing_info(&al1->match, &route_info, 0, 0);
	if (ret != 0) {
		dev_err(pa_dev->dev, "%s:route info config failed\n", __func__);
		goto fail;
	}

	ret = pa_conv_routing_info(&al1->next_fail, &fail_info, 0, 1);
	if (ret != 0) {
		dev_err(pa_dev->dev, "%s:fail info config failed\n", __func__);
		goto fail;
	}

	swiz_fwd(&(al1->match));
	swiz_fwd(&(al1->next_fail));

	/* Indicate that it is a configuration command */
	tx->psdata[0] = ((u32)(4 << 5) << 24);

	pa_submit_tx_packet(tx);
	dev_dbg(pa_dev->dev, "%s: waiting for command transmit complete\n",
		__func__);
	return 0;

fail:
	pa_free_packet(pa_dev, tx);
	return ret;
}

/* Configure route for exception packets in PA
 * All exceptions will be routed to Linux
 */
static int pa_config_exception_route(struct pa_device *pa_dev)
{
	struct pa_route_info eroutes[EROUTE_N_MAX];
	struct pa_frm_command_sys_config_pa *cpa;
	u32 context = PA_CONTEXT_CONFIG;
	struct pa_frm_command *fcmd;
	struct pa_packet *tx;
	int i, size, ret;

	memset(eroutes, 0, sizeof(eroutes));
	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_command_sys_config_pa) + 4);

	tx = pa_alloc_packet(pa_dev, size, pa_dev->pdsp1_tx_channel);
	if (!tx) {
		dev_err(pa_dev->dev, "%s: could not allocate cmd tx packet\n",
			__func__);
		ret = -ENOMEM;
		goto fail;
	}

	fcmd = tx->data;
	cpa = (struct pa_frm_command_sys_config_pa *) &(fcmd->cmd);
	memset(cpa, 0, sizeof(*cpa));
	pa_format_cmd_hdr(pa_dev, fcmd, PAFRM_CONFIG_COMMAND_SYS_CONFIG,
			0, context);
	cpa->cfg_code = PAFRM_SYSTEM_CONFIG_CODE_EROUTE;

	for (i = 0; i < EROUTE_N_MAX; i++) {
		eroutes[i].dest			= PA_DEST_HOST;
		eroutes[i].flow_id		= pa_dev->rx_flow_base;
		eroutes[i].queue		= pa_dev->rx_queue_base;
		eroutes[i].m_route_index	= -1;
		eroutes[i].route_type		= PA_ROUTE_INTF_FLOW;
		cpa->u.eroute.route_bitmap |= (1 << i);

		ret =  pa_conv_routing_info(&cpa->u.eroute.eroute[i],
					    &eroutes[i], PA_CMD_TX_DEST_5, 0);
		if (ret != 0) {
			dev_err(pa_dev->dev, "%s: route info config failed\n",
				__func__);
			goto fail;
		}
		swiz_fwd(&cpa->u.eroute.eroute[i]);
	}

	cpa->u.eroute.route_bitmap = cpu_to_be32(cpa->u.eroute.route_bitmap);

	/* Indicate that it is a configuration command */
	tx->psdata[0] = ((u32)(4 << 5) << 24);
	pa_submit_tx_packet(tx);
	dev_dbg(pa_dev->dev, "%s: waiting for command transmit complete\n",
		__func__);
	return 0;

fail:
	pa_free_packet(pa_dev, tx);
	return ret;
}

static int keystone_pa_add_mac(struct pa_intf *pa_intf, int index,
			       const u8 *smac, const u8 *dmac, int rule,
			       unsigned etype, int port)
{
	struct pa_route_info route_info, fail_info;
	struct pa_frm_command *fcmd;
	struct pa_frm_cmd_add_lut1 *al1;
	struct pa_packet *tx;
	struct pa_device *priv = pa_intf->pa_device;
	u32 context = PA_CONTEXT_CONFIG;
	int size, ret;

	dev_dbg(priv->dev, "add mac, index %d, smac %pM, dmac %pM, rule %d, "
		"type %04x, port %d\n", index, smac, dmac, rule, etype, port);

	memset(&fail_info, 0, sizeof(fail_info));
	memset(&route_info, 0, sizeof(route_info));

	if (rule == PACKET_HST) {
		route_info.dest			= PA_DEST_HOST;
		route_info.flow_id		= pa_intf->data_flow_num;
		route_info.queue		= pa_intf->data_queue_num;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= pa_intf->data_flow_num;
		fail_info.queue			= pa_intf->data_queue_num;
		fail_info.m_route_index		= -1;
	} else if (rule == PACKET_PARSE) {
		route_info.dest			= PA_DEST_CONTINUE_PARSE_LUT1;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_HOST;
		fail_info.flow_id		= pa_intf->data_flow_num;
		fail_info.queue			= pa_intf->data_queue_num;
		fail_info.m_route_index		= -1;
	} else if (rule == PACKET_DROP) {
		route_info.dest			= PA_DEST_DISCARD;
		route_info.m_route_index	= -1;
		fail_info.dest			= PA_DEST_DISCARD;
		fail_info.m_route_index		= -1;
	}

	size = (sizeof(struct pa_frm_command) +
		sizeof(struct pa_frm_cmd_add_lut1) + 4);
	tx = pa_alloc_packet(priv, size, priv->pdsp0_tx_channel);
	if (!tx) {
		dev_err(priv->dev, "could not allocate cmd tx packet\n");
		return -ENOMEM;
	}

	fcmd = tx->data;
	al1 = (struct pa_frm_cmd_add_lut1 *) &(fcmd->cmd);

	fcmd->command_result	= 0;
	fcmd->command		= PAFRM_CONFIG_COMMAND_ADDREP_LUT1;
	fcmd->magic		= PAFRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id		= PA_COMID_L2;
	fcmd->ret_context	= context;
	fcmd->flow_id		= priv->cmd_flow_num;
	fcmd->reply_queue	= priv->cmd_queue_num;
	fcmd->reply_dest	= PAFRM_DEST_PKTDMA;

	al1->index		= index;
	al1->type		= PAFRM_COM_ADD_LUT1_STANDARD;

	if (etype) {
		al1->u.eth_ip.etype	= etype;
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_ETYPE;
	}

	al1->u.eth_ip.vlan	= 0;
	al1->u.eth_ip.pm.mpls	= 0;
	if (port) {
		al1->u.eth_ip.inport    = port;
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_PORT;
	}

	if (dmac) {
		al1->u.eth_ip.dmac[0] = dmac[0];
		al1->u.eth_ip.dmac[1] = dmac[1];
		al1->u.eth_ip.dmac[2] = dmac[2];
		al1->u.eth_ip.dmac[3] = dmac[3];
		al1->u.eth_ip.dmac[4] = dmac[4];
		al1->u.eth_ip.dmac[5] = dmac[5];
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_DMAC;
	}
	if (smac) {
		al1->u.eth_ip.smac[0] = smac[0];
		al1->u.eth_ip.smac[1] = smac[1];
		al1->u.eth_ip.smac[2] = smac[2];
		al1->u.eth_ip.smac[3] = smac[3];
		al1->u.eth_ip.smac[4] = smac[4];
		al1->u.eth_ip.smac[5] = smac[5];
		al1->u.eth_ip.match_flags |= PAFRM_LUT1_MATCH_SMAC;
	}

	al1->u.eth_ip.key |= PAFRM_LUT1_KEY_MAC;
	al1->u.eth_ip.match_flags |= PAFRM_LUT1_CUSTOM_MATCH_KEY;

	ret = pa_conv_routing_info(&al1->match, &route_info, 0, 0);
	if (ret) {
		dev_err(priv->dev, "route info config failed\n");
		goto fail;
	}

	ret = pa_conv_routing_info(&al1->next_fail, &fail_info, 0, 1);
	if (ret) {
		dev_err(priv->dev, "fail info config failed\n");
		goto fail;
	}

	swizFcmd(fcmd);
	swizAl1((struct pa_frm_cmd_add_lut1 *)&(fcmd->cmd));

	tx->psdata[0] = ((u32)(4 << 5) << 24);

	tx->epib[1] = 0x11112222;
	tx->epib[2] = 0x33334444;
	tx->epib[3] = 0;

	pa_submit_tx_packet(tx);
	dev_dbg(priv->dev, "waiting for command transmit complete\n");
	return 0;

fail:
	pa_free_packet(priv, tx);
	return ret;
}

static void pa_init_crc_table4(u32 polynomial, u32 *crc_table4)
{
	int i, bit;

	/* 16 values representing all possible 4-bit values */
	for(i = 0; i < PARAM_CRC_TABLE_SIZE; i++) {
		crc_table4[i] = i << 28;
		for (bit = 0; bit < 4; bit++) {
			/* If shifting out a zero, then just shift */
			if (!(crc_table4[i] & 0x80000000))
				crc_table4[i] = (crc_table4[i] << 1);
			/* Else add in the polynomial as well */
			else
				crc_table4[i] = (crc_table4[i] << 1) ^ polynomial;
		}
		crc_table4[i] = cpu_to_be32(crc_table4[i]);
	}
}

#define	CRC32C_POLYNOMIAL	0x1EDC6F41
#define	SCTP_CRC_INITVAL	0xFFFFFFFF
static int pa_config_crc_engine(struct pa_device *priv)
{
	struct pa_frm_command *fcmd;
	struct pa_frm_config_crc *ccrc;
	struct pa_packet *tx;
	int size;

	/* Verify that there is enough room to create the command */
	size = sizeof(*fcmd) + sizeof(*ccrc) - sizeof(u32);
	tx = pa_alloc_packet(priv, size, priv->pdsp0_tx_channel);
	if (!tx) {
		dev_err(priv->dev, "could not allocate cmd tx packet\n");
		return -ENOMEM;
	}

	/* Create the command */
	fcmd = tx->data;
	fcmd->command_result	= 0;
	fcmd->command		= PAFRM_CONFIG_COMMAND_CRC_ENGINE;
	fcmd->magic		= PAFRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id		= 0;
	fcmd->ret_context	= PA_CONTEXT_CONFIG;
	fcmd->flow_id		= priv->cmd_flow_num;
	fcmd->reply_queue	= priv->cmd_queue_num;
	fcmd->reply_dest	= PAFRM_DEST_PKTDMA;
	swizFcmd(fcmd);

	ccrc = (struct pa_frm_config_crc *)&(fcmd->cmd);
	ccrc->ctrl_bitmap  = PARAM_CRC_SIZE_32;
	ccrc->ctrl_bitmap |= PARAM_CRC_CTRL_RIGHT_SHIFT;
	ccrc->ctrl_bitmap |= PARAM_CRC_CTRL_INV_RESULT;
	ccrc->init_val = cpu_to_be32(SCTP_CRC_INITVAL);

	/* Magic polynomial value is CRC32c defined by RFC4960 */
	pa_init_crc_table4(CRC32C_POLYNOMIAL, ccrc->crc_tbl);

	tx->psdata[0] = ((u32)(4 << 5) << 24);

	tx->epib[1] = 0x11112222;
	tx->epib[2] = 0x33334444;
	tx->epib[3] = 0;

	pa_submit_tx_packet(tx);
	dev_dbg(priv->dev, "waiting for command transmit complete\n");

	return 0;
}


static inline int pa_fmtcmd_tx_csum(struct netcp_packet *p_info)
{
	struct sk_buff *skb = p_info->skb;
	struct pasaho_com_chk_crc *ptx;
	int start, len;
	int size;

	size = sizeof(*ptx);
	ptx = (struct pasaho_com_chk_crc *)netcp_push_psdata(p_info, size);

	start = skb_checksum_start_offset(skb);
	len = skb->len - start;

	ptx->word0 = 0;
	ptx->word1 = 0;
	ptx->word2 = 0;
	PASAHO_SET_CMDID(ptx, PASAHO_PAMOD_CMPT_CHKSUM);
	PASAHO_CHKCRC_SET_START(ptx, start);
	PASAHO_CHKCRC_SET_LEN(ptx, len);
	PASAHO_CHKCRC_SET_RESULT_OFF(ptx, skb->csum_offset);
	PASAHO_CHKCRC_SET_INITVAL(ptx, 0);
	PASAHO_CHKCRC_SET_NEG0(ptx, 0);

	return size;
}

static inline int pa_fmtcmd_tx_crc32c(struct netcp_packet *p_info)
{
	struct sk_buff *skb = p_info->skb;
	struct pasaho_com_chk_crc *ptx;
	int start, len;
	int size;

	size = sizeof(*ptx);
	ptx = (struct pasaho_com_chk_crc *)netcp_push_psdata(p_info, size);

	start = skb_checksum_start_offset(skb);
	len = skb->len - start;

	ptx->word0 = 0;
	ptx->word1 = 0;
	ptx->word2 = 0;
	PASAHO_SET_CMDID             (ptx, PASAHO_PAMOD_CMPT_CRC);
	PASAHO_CHKCRC_SET_START      (ptx, start);
	PASAHO_CHKCRC_SET_LEN        (ptx, len);
	PASAHO_CHKCRC_SET_CTRL       (ptx, PAFRM_CRC_FLAG_CRC_OFFSET_VALID);
	PASAHO_CHKCRC_SET_RESULT_OFF (ptx, skb->csum_offset);

	return size;
}

static inline int pa_fmtcmd_next_route(struct netcp_packet *p_info, u8 ps_flags)
{
	struct pasaho_next_route *nr;

	nr = (struct pasaho_next_route *)netcp_push_psdata(p_info, sizeof(*nr));
	if (!nr)
		return -ENOMEM;

	/* Construct word0 */
	nr->word0 = 0;
	PASAHO_SET_CMDID(nr, PASAHO_PAMOD_NROUTE);
	PASAHO_SET_E(nr, 1);
	PASAHO_SET_DEST(nr, PAFRM_DEST_ETH);
	PASAHO_SET_FLOW(nr, 0);
	PASAHO_SET_QUEUE (nr, 0);

	/* Construct sw_info0 and sw_info1 */
	nr->sw_info0 = 0;
	nr->sw_info1 = 0;

	/* Construct word1 */
	nr->word1 = 0;
	PASAHO_SET_PKTTYPE(nr, ps_flags);
	
	return sizeof(*nr);
}

static inline int pa_fmtcmd_tx_timestamp(struct netcp_packet *p_info, const struct pa_cmd_tx_timestamp *tx_ts)
{
	struct pasaho_report_timestamp	*rt_info;
	int				 size;

	size = sizeof(*rt_info);
	rt_info = (struct pasaho_report_timestamp *)netcp_push_psdata(p_info, size);
	if (!rt_info)
		return -ENOMEM;

	rt_info->word0 = 0;
	PASAHO_SET_CMDID(rt_info, PASAHO_PAMOD_REPORT_TIMESTAMP);
	PASAHO_SET_REPORT_FLOW(rt_info, (u8)tx_ts->flow_id);
	PASAHO_SET_REPORT_QUEUE(rt_info, tx_ts->dest_queue);
	rt_info->sw_info0 = tx_ts->sw_info0;

	return size;
}

static inline int pa_fmtcmd_align(struct netcp_packet *p_info, const unsigned bytes)
{
	struct pasaho_cmd_info	*paCmdInfo;
	int i;

	if ((bytes & 0x03) != 0)
		return -EINVAL;

	paCmdInfo = (struct pasaho_cmd_info *)netcp_push_psdata(p_info, bytes);

	for (i = bytes/sizeof(u32); i > 0; --i ) {
		paCmdInfo->word0 = 0;
		PASAHO_SET_CMDID(paCmdInfo, PASAHO_PAMOD_DUMMY);
		++paCmdInfo;
	}

	return bytes;
}

static inline int extract_l4_proto(struct netcp_packet *p_info)
{
	struct sk_buff *skb = p_info->skb;
	int l4_proto = 0;
	__be16 l3_proto;

	l3_proto = skb->protocol;
	if (l3_proto == __constant_htons(ETH_P_8021Q)) {
		/* Can't use vlan_eth_hdr() here, skb->mac_header isn't valid */
		struct vlan_ethhdr *vhdr = (struct vlan_ethhdr *)skb->data;
		l3_proto = vhdr->h_vlan_encapsulated_proto;
	}

	switch (l3_proto) {
	case __constant_htons(ETH_P_IP):
		l4_proto = ip_hdr(skb)->protocol;
		break;
	case __constant_htons(ETH_P_IPV6):
		l4_proto = ipv6_hdr(skb)->nexthdr;
		break;
	default:
		if (unlikely(net_ratelimit())) {
			dev_warn(p_info->netcp->dev,
				 "partial checksum but L3 proto = 0x%04hx!\n",
				 ntohs(l3_proto));
		}
	}

	return l4_proto;
}

static int pa_add_ip_proto(struct pa_device *pa_dev, u8 proto)
{
	struct pa_lut_entry *entry;
	int ret;

	entry = pa_lut_alloc(pa_dev, PA_LUT_IP, 1);
	if (entry == NULL)
		return -1;
	entry->u.ip_proto = proto;

	ret = keystone_pa_add_ip_proto(pa_dev, entry->index, proto,
				       PACKET_PARSE);
	if (ret)
		dev_err(pa_dev->dev, "failed to add IP proto(%d) rule\n",
			proto);
	return ret;
}

static int pa_del_ip_proto(struct pa_device *pa_dev, u8 proto)
{
	struct pa_lut_entry *entry;
	int idx, ret = 0;

	for (idx = 0; idx < pa_dev->ip_lut_size; idx++) {
		entry = pa_dev->ip_lut + idx;
		if (!entry->valid || !entry->in_use || entry->u.ip_proto != proto)
			continue;
		ret = keystone_pa_add_ip_proto(pa_dev, entry->index, 0,
					       PACKET_DROP);
		if (ret)
			dev_err(pa_dev->dev,
				"failed to del IP proto(%d) rule\n", proto);
		entry->in_use = false;
		entry->u.ip_proto = 0;
	}
	return ret;
}

static int pa_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct pa_intf *pa_intf = data;
	struct pa_device *pa_dev = pa_intf->pa_device;
	struct netcp_priv *netcp_priv = netdev_priv(pa_intf->net_device);
	struct sk_buff *skb = p_info->skb;
	struct sock *sk = skb->sk;
	struct pa_cmd_tx_timestamp tx_ts;
	int size, total = 0;
	u8 ps_flags;
	struct tstamp_pending *pend;

	ps_flags = 0;
	if (pa_dev->multi_if) {
		if (likely(skb->mark == 0) ||
		    likely((skb->mark & pa_dev->mark_mcast_match[1]) !=
				pa_dev->mark_mcast_match[0])) {
			/* normal port-specific output packet */
			ps_flags |= (netcp_priv->cpsw_port & BITS(3)) <<
					PAFRM_ETH_PS_FLAGS_PORT_SHIFT;
		} else {
			/* Drop packet if port not in mask */
			if ((skb->mark & BIT(netcp_priv->cpsw_port - 1)) == 0) {
				return NETCP_TX_DROP;
			}
		}
	}

	/* Generate the next route command */
	size = pa_fmtcmd_next_route(p_info, ps_flags);
	if (unlikely(size < 0))
		return size;
	total += size;

	/* If TX Timestamp required, request it */
	if (unlikely(pa_intf->tx_timestamp_enable &&
		     !pa_dev->force_no_hwtstamp &&
		     sk && 
		     (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) &&
		     !(skb_shinfo(skb)->tx_flags & SKBTX_IN_PROGRESS))) {
		pend = kzalloc(sizeof(*pend), GFP_ATOMIC);
		if (pend) {
			void *saved_sp;
			if (!atomic_inc_not_zero(&sk->sk_refcnt))
				return -ENODEV;

			/* The SA module may have reused skb->sp */
			saved_sp = skb->sp;
			skb->sp = NULL;
			pend->skb = skb_clone(skb, GFP_ATOMIC);
			skb->sp = saved_sp;

			if (!pend->skb) {
				sock_put(sk);
				kfree(pend);
				return -ENOMEM;
			} else {
				pend->sock = sk;
				pend->pa_dev = pa_dev;
				pend->context =  PA_CONTEXT_TSTAMP |
					(~PA_CONTEXT_MASK &
					 atomic_inc_return(&tstamp_sequence));
				tstamp_add_pending(pend);

				memset(&tx_ts, 0, sizeof(tx_ts));
				tx_ts.dest_queue = pa_dev->cmd_queue_num;
				tx_ts.flow_id    = pa_dev->cmd_flow_num;
				tx_ts.sw_info0   = pend->context;

				size = pa_fmtcmd_tx_timestamp(p_info,
							      &tx_ts);
				if (unlikely(size < 0))
					return size;
				total += size;
			}
		}
	}

	/* If checksum offload required, request it */
	if ((skb->ip_summed == CHECKSUM_PARTIAL) &&
	    (pa_dev->csum_offload == CSUM_OFFLOAD_HARD)) {
		int l4_proto;

		l4_proto = extract_l4_proto(p_info);
		switch (l4_proto) {
		case IPPROTO_TCP:
		case IPPROTO_UDP:
			size = pa_fmtcmd_tx_csum(p_info);
			break;
		case IPPROTO_SCTP:
			size = pa_fmtcmd_tx_crc32c(p_info);
			break;
		default:
			if (unlikely(net_ratelimit())) {
				dev_warn(p_info->netcp->dev,
					 "partial checksum but L4 proto = %d!\n",
					 l4_proto);
			}
			size = 0;
			break;
		}

		if (unlikely(size < 0))
			return size;
		total += size;
	}

	/* The next hook may require the command stack to be 8-byte aligned */
	size = netcp_align_psdata(p_info, 8);
	if (unlikely(size < 0))
		return size;
	if (size > 0) {
		size = pa_fmtcmd_align(p_info, size);
		if (unlikely(size < 0))
			return size;
		total += size;
	}

	p_info->tx_pipe = &pa_intf->tx_pipe;
	return 0;
}


/* This code adapted from net/core/skbuff.c:skb_checksum() */
static __wsum skb_sctp_csum(struct sk_buff *skb, int offset,
			  int len, __wsum csum)
{
	int start = skb_headlen(skb);
	int i, copy = start - offset;
	struct sk_buff *frag_iter;

	/* Checksum header. */
	if (copy > 0) {
		if (copy > len)
			copy = len;
		csum = sctp_update_cksum(skb->data + offset, copy, csum);
		if ((len -= copy) == 0)
			return csum;
		offset += copy;
	}

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		int end;
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		WARN_ON(start > offset + len);

		end = start + skb_frag_size(frag);
		if ((copy = end - offset) > 0) {
			u8 *vaddr;
			skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

			if (copy > len)
				copy = len;
			vaddr = kmap_atomic(skb_frag_page(frag));
			csum = sctp_update_cksum(vaddr + frag->page_offset +
					 offset - start, copy, csum);
			kunmap_atomic(vaddr);
			if (!(len -= copy))
				return csum;
			offset += copy;
		}
		start = end;
	}

	skb_walk_frags(skb, frag_iter) {
		int end;

		WARN_ON(start > offset + len);

		end = start + frag_iter->len;
		if ((copy = end - offset) > 0) {
			if (copy > len)
				copy = len;
			csum = skb_sctp_csum(frag_iter,
						offset - start, copy, csum);
			if ((len -= copy) == 0)
				return csum;
			offset += copy;
		}
		start = end;
	}
	BUG_ON(len);

	return csum;
}

static void skb_warn_bad_offload(const struct sk_buff *skb)
{
	static const netdev_features_t null_features = 0;
	struct net_device *dev = skb->dev;
	const char *driver = "";

	if (dev && dev->dev.parent)
		driver = dev_driver_string(dev->dev.parent);

	WARN(1, "%s: caps=(%pNF, %pNF) len=%d data_len=%d gso_size=%d "
	     "gso_type=%d ip_summed=%d\n",
	     driver, dev ? &dev->features : &null_features,
	     skb->sk ? &skb->sk->sk_route_caps : &null_features,
	     skb->len, skb->data_len, skb_shinfo(skb)->gso_size,
	     skb_shinfo(skb)->gso_type, skb->ip_summed);
}

/* This code adapted from net/core/dev.c:skb_checksum_help() */
static int skb_sctp_csum_help(struct sk_buff *skb)
{
	__wsum csum;
	int ret = 0, offset;

	if (skb->ip_summed == CHECKSUM_COMPLETE)
		goto out_set_summed;

	if (unlikely(skb_shinfo(skb)->gso_size)) {
		skb_warn_bad_offload(skb);
		return -EINVAL;
	}

	offset = skb_checksum_start_offset(skb);
	BUG_ON(offset >= skb_headlen(skb));
	csum = skb_sctp_csum(skb, offset, skb->len - offset, ~0);

	offset += skb->csum_offset;
	BUG_ON(offset + sizeof(__le32) > skb_headlen(skb));

	if (skb_cloned(skb) &&
	    !skb_clone_writable(skb, offset + sizeof(__le32))) {
		ret = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
		if (ret)
			goto out;
	}

	*(__le32 *)(skb->data + offset) = sctp_end_cksum(csum);
out_set_summed:
	skb->ip_summed = CHECKSUM_NONE;
out:
	return ret;
}

static int pa_txhook_softcsum(int order, void *data, struct netcp_packet *p_info)
{
	struct pa_intf *pa_intf = data;
	struct pa_device *pa_dev = pa_intf->pa_device;
	struct sk_buff *skb = p_info->skb;
	int l4_proto;
	int ret = 0;

	if ((skb->ip_summed != CHECKSUM_PARTIAL) ||
	    (pa_dev->csum_offload != CSUM_OFFLOAD_SOFT))
		return 0;

	l4_proto = extract_l4_proto(p_info);
	if (unlikely(!l4_proto))
		return 0;

	switch (l4_proto) {
	case IPPROTO_TCP:
	case IPPROTO_UDP:
		ret = skb_checksum_help(skb);
		break;
	case IPPROTO_SCTP:
		ret = skb_sctp_csum_help(skb);
		break;
	default:
		if (unlikely(net_ratelimit())) {
			dev_warn(p_info->netcp->dev,
				 "partial checksum but L4 proto = %d!\n",
				 l4_proto);
		}
		return 0;
	}

	return ret;
}


static inline int pa_rx_timestamp(struct pa_intf *pa_intf,
				  struct netcp_packet *p_info)
{
	struct pa_device *pa_dev = pa_intf->pa_device;
	struct sk_buff *skb = p_info->skb;
	struct skb_shared_hwtstamps *sh_hw_tstamps;
	u64 pa_ns;
	u64 sys_time;

	if (!pa_intf->rx_timestamp_enable)
		return 0;

	if (p_info->rxtstamp_complete)
		return 0;

	pa_ns = tstamp_raw_to_ns(pa_dev, p_info->epib[0], p_info->psdata[6]);
	sys_time = pa_to_sys_time(pa_dev, pa_ns);

	sh_hw_tstamps = skb_hwtstamps(skb);
	memset(sh_hw_tstamps, 0, sizeof(*sh_hw_tstamps));
	sh_hw_tstamps->hwtstamp = ns_to_ktime(pa_ns);
	sh_hw_tstamps->syststamp = ns_to_ktime(sys_time);

	p_info->rxtstamp_complete = true;

	return 0;
}

/*  The NETCP sub-system performs IPv4 header checksum, UDP/TCP checksum and
 *  SCTP CRC-32c checksum autonomously.
 *  The checksum and CRC verification results are recorded at the 4-bit error
 *  flags in the CPPI packet descriptor as described below:
 *  bit 3: IPv4 header checksum error
 *  bit 2: UDP/TCP or SCTP CRC-32c checksum error
 *  bit 1: Custom CRC checksum error
 *  bit 0: reserved
 */
static inline void pa_rx_checksum(struct netcp_packet *p_info)
{
	struct pasaho_long_info *linfo =
		(struct pasaho_long_info *)p_info->psdata;
	/* L4 Checksum is verified only if the packet was sent for LUT-2
	 * processing. This can be confirmed by presence of payload offset.
	 */
	if (likely(PASAHO_LINFO_READ_L5_OFFSET(linfo))) {
		/* check for L3 & L4 checksum error */
		if (likely(!((p_info->eflags >> 2) & BITS(2))))
			p_info->skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
}

static int pa_rx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct pa_intf *pa_intf = data;
	struct pa_device *pa_dev = pa_intf->pa_device;

	/* Timestamping on Rx packets */
	if (!pa_dev->force_no_hwtstamp)
		pa_rx_timestamp(pa_intf, p_info);

	/* Checksum offload on Rx packets */
	if (pa_dev->csum_offload == CSUM_OFFLOAD_HARD)
		pa_rx_checksum(p_info);

	return 0;
}

static int pa_close(void *intf_priv, struct net_device *ndev)
{
	struct pa_intf *pa_intf = intf_priv;
	struct pa_device *pa_dev = pa_intf->pa_device;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);

	netcp_unregister_txhook(netcp_priv, pa_dev->txhook_order,
				pa_tx_hook, intf_priv);
	if (pa_dev->csum_offload == CSUM_OFFLOAD_SOFT)
		netcp_unregister_txhook(netcp_priv, pa_dev->txhook_softcsum,
					pa_txhook_softcsum, intf_priv);

	if ((!pa_dev->force_no_hwtstamp) ||
	     (pa_dev->csum_offload == CSUM_OFFLOAD_HARD))
		netcp_unregister_rxhook(netcp_priv, pa_dev->rxhook_order,
					pa_rx_hook, intf_priv);

	netcp_txpipe_close(&pa_intf->tx_pipe);

	/* De-Configure the streaming switch */
	netcp_set_streaming_switch(pa_dev->netcp_device,
				   netcp_priv->cpsw_port,
				   pa_intf->saved_ss_state);


	mutex_lock(&pa_modules_lock);
	if (!--pa_dev->inuse_if_count) {
		/* Do pa disable related stuff only if this is the last
		 * interface to go down
		 */
		if (pa_dev->csum_offload == CSUM_OFFLOAD_HARD) {
			pa_del_ip_proto(pa_dev, IPPROTO_TCP);
			pa_del_ip_proto(pa_dev, IPPROTO_UDP);
		}

		if (pa_dev->pdsp1_tx_channel) {
			dma_release_channel(pa_dev->pdsp1_tx_channel);
			pa_dev->pdsp1_tx_channel = NULL;
		}
		if (pa_dev->pdsp0_tx_channel) {
			dma_release_channel(pa_dev->pdsp0_tx_channel);
			pa_dev->pdsp0_tx_channel = NULL;
		}
		if (pa_dev->rx_channel) {
			dmaengine_pause(pa_dev->rx_channel);
			tasklet_kill(&pa_dev->task);
			dma_rxfree_flush(pa_dev->rx_channel);
			dma_poll(pa_dev->rx_channel, -1);
			dma_release_channel(pa_dev->rx_channel);
			pa_dev->rx_channel = NULL;
		}

		tstamp_purge_pending(pa_dev);

		if (pa_dev->clk) {
			clk_disable_unprepare(pa_dev->clk);
			clk_put(pa_dev->clk);
		}
		pa_dev->clk = NULL;
	}

	pa_dev->opened = 0;
	mutex_unlock(&pa_modules_lock);
	return 0;
}

static int pa_open(void *intf_priv, struct net_device *ndev)
{
	struct pa_intf *pa_intf = intf_priv;
	struct pa_device *pa_dev = pa_intf->pa_device;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);
	struct dma_keystone_info config;
	struct pa_pdsp_timer_regs __iomem *timer_reg = &pa_dev->reg_timer[0];
	const struct firmware *fw;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	int i, ret, err;
	unsigned long pa_rate;
	u64 max_sec;

	/* The first time an open is being called */
	mutex_lock(&pa_modules_lock);

	dev_dbg(pa_dev->dev, "pa_open() called for port: %d\n",
		 netcp_priv->cpsw_port);

	chan = netcp_get_rx_chan(netcp_priv);
	pa_intf->data_flow_num = dma_get_rx_flow(chan);
	pa_intf->data_queue_num = dma_get_rx_queue(chan);

	dev_dbg(pa_dev->dev, "configuring data receive flow %d, queue %d\n",
		 pa_intf->data_flow_num, pa_intf->data_queue_num);

	if (++pa_dev->inuse_if_count == 1) {

		/* Do pa enable, load firmware only for the first interface
		 * that comes up
		 */
		dev_dbg(pa_dev->dev, "pa_open() called for first time"
			" initializing per dev stuff\n");

		pa_dev->clk = clk_get(pa_dev->dev, "clk_pa");
		if (IS_ERR_OR_NULL(pa_dev->clk)) {
			dev_warn(pa_dev->dev, "unable to get Packet Accelerator clock\n");
			pa_dev->clk = NULL;
		}

		if (pa_dev->clk)
			clk_prepare_enable(pa_dev->clk);

		keystone_pa_reset(pa_dev);

		for (i = 0; i < DEVICE_PA_NUM_PDSPS; ++i) {
			if (!pa_dev->pdsp_fw[i])
				continue;

			ret = request_firmware(&fw, pa_dev->pdsp_fw[i],
					pa_dev->dev);
			if (ret != 0) {
				dev_err(pa_dev->dev, "cant find fw for pdsp %d",
					i);
				ret = -ENODEV;
				goto fail;
			}

			/* Download the firmware to the PDSP */
			keystone_pa_set_firmware(pa_dev, i,
					(const unsigned int *) fw->data,
					fw->size);

			release_firmware(fw);
		}

		ret = keystone_pa_reset_control(pa_dev, PA_STATE_ENABLE);
		if (ret != 1) {
			dev_err(pa_dev->dev, "enable failed, ret = %d\n", ret);
			ret = -ENODEV;
			goto fail;
		}

		pa_get_version(pa_dev);

		/* Start PDSP timer at a prescaler of divide by 2 */
		__raw_writel(0xffff, &timer_reg->timer_load);
		__raw_writel((PA_SS_TIMER_CNTRL_REG_GO |
			      PA_SS_TIMER_CNTRL_REG_MODE |
			      PA_SS_TIMER_CNTRL_REG_PSE |
			      (0 << PA_SS_TIMER_CNTRL_REG_PRESCALE_SHIFT)),
			      &timer_reg->timer_control);

		/* calculate the multiplier/shift to
		 * convert PA counter ticks to ns. */
		pa_rate = clk_get_rate(pa_dev->clk) / 2;

		max_sec = ((1ULL << 48) - 1) + (pa_rate - 1);
		do_div(max_sec, pa_rate);

		clocks_calc_mult_shift(&pa_dev->timestamp_info.mult,
				&pa_dev->timestamp_info.shift, pa_rate,
				NSEC_PER_SEC, max_sec);

		dev_info(pa_dev->dev, "pa_clk_rate(%lu HZ),mult(%u),shift(%u)\n",
				pa_rate, pa_dev->timestamp_info.mult,
				pa_dev->timestamp_info.shift);

		pa_dev->timestamp_info.system_offset = 0;

		pa_calibrate_with_system_timer(pa_dev);

		dma_cap_zero(mask);
		dma_cap_set(DMA_SLAVE, mask);

		/* Open the PA Command transmit channel */
		pa_dev->pdsp0_tx_channel = dma_request_channel_by_name(mask,
								"patx-pdsp0");
		if (IS_ERR_OR_NULL(pa_dev->pdsp0_tx_channel)) {
			dev_err(pa_dev->dev, "Couldnt get PATX cmd channel\n");
			pa_dev->pdsp0_tx_channel = NULL;
			ret = -ENODEV;
			goto fail;
		}

		pa_dev->pdsp1_tx_channel = dma_request_channel_by_name(mask,
								"patx-pdsp1");
		if (IS_ERR_OR_NULL(pa_dev->pdsp1_tx_channel)) {
			dev_err(pa_dev->dev,
				"Couldnt get PATX LUT-1 cmd channel\n");
			pa_dev->pdsp1_tx_channel = NULL;
			ret = -ENODEV;
			goto fail;
		}

		memset(&config, 0, sizeof(config));
		config.direction	= DMA_MEM_TO_DEV;
		config.tx_queue_depth	= pa_dev->tx_cmd_queue_depth;

		err = dma_keystone_config(pa_dev->pdsp0_tx_channel, &config);
		if (err)
			goto fail;

		err = dma_keystone_config(pa_dev->pdsp1_tx_channel, &config);
		if (err)
			goto fail;

		/* Open the PA common response channel */
		pa_dev->rx_channel = dma_request_channel_by_name(mask, "parx");
		if (IS_ERR_OR_NULL(pa_dev->rx_channel)) {
			dev_err(pa_dev->dev, "Could not get PA RX channel\n");
			pa_dev->rx_channel = NULL;
			ret = -ENODEV;
			goto fail;
		}

		memset(&config, 0, sizeof(config));

		config.direction		= DMA_DEV_TO_MEM;
		config.scatterlist_size		= PA_SGLIST_SIZE;
		config.rxpool_allocator		= pa_rxpool_alloc;
		config.rxpool_destructor	= pa_rxpool_free;
		config.rxpool_param		= pa_dev;
		config.rxpool_count		= 1;
		config.rxpool_thresh_enable	= DMA_THRESH_NONE;
		config.rxpools[0].pool_depth	= pa_dev->rx_pool_depth;
		config.rxpools[0].buffer_size	= pa_dev->rx_buffer_size;

		err = dma_keystone_config(pa_dev->rx_channel, &config);
		if (err)
			goto fail;

		tasklet_init(&pa_dev->task, pa_chan_work_handler,
			     (unsigned long) pa_dev);
		dma_set_notify(pa_dev->rx_channel, pa_chan_notify, pa_dev);
		pa_dev->cmd_flow_num = dma_get_rx_flow(pa_dev->rx_channel);
		pa_dev->cmd_queue_num = dma_get_rx_queue(pa_dev->rx_channel);
		dev_dbg(pa_dev->dev, "command receive flow %d, queue %d\n",
			pa_dev->cmd_flow_num, pa_dev->cmd_queue_num);
		pa_dev->addr_count = 0;
		dma_rxfree_refill(pa_dev->rx_channel);
		ret = pa_config_exception_route(pa_dev);
		if (ret < 0)
			goto fail;

		if (pa_dev->csum_offload == CSUM_OFFLOAD_HARD) {
			ret = pa_config_crc_engine(pa_dev);
			if (ret < 0)
				goto fail;

			/* make lut entries invalid */
			for (i = 0; i < pa_dev->lut_size; i++) {
				if (!pa_dev->lut[i].valid)
					continue;
				keystone_pa_add_mac(pa_intf, i, NULL, NULL,
						    PACKET_DROP, 0,
						    PA_INVALID_PORT);
			}

			/* make IP LUT entries invalid */
			for (i = 0; i < pa_dev->ip_lut_size; i++) {
				if (!pa_dev->ip_lut[i].valid)
					continue;
				keystone_pa_add_ip_proto(pa_dev, i, 0,
							 PACKET_DROP);
			}

			/* if Rx checksum is enabled, add IP LUT entries for
			 * Rx checksumming
			 */
			ret = pa_add_ip_proto(pa_dev, IPPROTO_TCP);
			if (ret)
				goto fail;
			ret = pa_add_ip_proto(pa_dev, IPPROTO_UDP);
			if (ret)
				goto fail;
		}
	}
	mutex_unlock(&pa_modules_lock);

	pa_intf->saved_ss_state = netcp_get_streaming_switch(
						     pa_dev->netcp_device,
						     netcp_priv->cpsw_port);
	dev_dbg(pa_dev->dev, "saved_ss_state for port %d is %d\n",
		 netcp_priv->cpsw_port, pa_intf->saved_ss_state);

	/* Configure the streaming switch */
	netcp_set_streaming_switch(pa_dev->netcp_device, netcp_priv->cpsw_port,
				   PSTREAM_ROUTE_PDSP0);

	/* Open the PA Data transmit channel */
	ret = netcp_txpipe_open(&pa_intf->tx_pipe);
	if (ret)
		goto fail;

	netcp_register_txhook(netcp_priv, pa_dev->txhook_order,
			      pa_tx_hook, intf_priv);
	if (pa_dev->csum_offload == CSUM_OFFLOAD_SOFT)
		netcp_register_txhook(netcp_priv, pa_dev->txhook_softcsum,
				      pa_txhook_softcsum, intf_priv);

	if ((!pa_dev->force_no_hwtstamp) ||
	     (pa_dev->csum_offload == CSUM_OFFLOAD_HARD))
		netcp_register_rxhook(netcp_priv, pa_dev->rxhook_order,
				      pa_rx_hook, intf_priv);

	pa_dev->opened = 1;
	return 0;

fail:
	mutex_unlock(&pa_modules_lock);
	pa_close(intf_priv, ndev);
	return ret;
}

int pa_add_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct pa_intf *pa_intf = intf_priv;
	struct pa_device *pa_dev = pa_intf->pa_device;
	struct netcp_priv *netcp_priv = netdev_priv(pa_intf->net_device);
	int count = pa_lut_entry_count(naddr->type);
	struct pa_lut_entry *entries[count];
	int port = netcp_priv->cpsw_port;
	int idx, error;
	const u8 *addr;

	if (!pa_dev->opened)
		return -ENXIO;

	for (idx = 0; idx < count; idx++) {
		entries[idx] = pa_lut_alloc(pa_dev, PA_LUT_MAC,
						naddr->type == ADDR_ANY);
		if (!entries[idx])
			goto fail_alloc;
		entries[idx]->u.naddr = naddr;
	}

	addr = (naddr->type == ADDR_ANY) ? NULL : naddr->addr;
	idx = 0;

	if (naddr->type == ADDR_ANY) {
		error = keystone_pa_add_mac(pa_intf, entries[idx++]->index,
					    NULL, addr, PACKET_HST, 0, port);
		if (error)
			return error;
	}

	if (count > 1) {
		error = keystone_pa_add_mac(pa_intf, entries[idx++]->index,
					    NULL, addr, PACKET_PARSE,
					    0x0800, port);
		if (error)
			return error;

		error = keystone_pa_add_mac(pa_intf, entries[idx++]->index,
					    NULL, addr, PACKET_PARSE,
					    0x86dd, port);
		if (error)
			return error;
	}

	if (naddr->type != ADDR_ANY) {
		error = keystone_pa_add_mac(pa_intf, entries[idx++]->index,
					    NULL, addr, PACKET_HST, 0, port);
		if (error)
			return error;
	}

	return error;

fail_alloc:
	for (idx--; idx >= 0; idx--)
		entries[idx]->in_use = false;
	return -ENOMEM;
}

static int pa_del_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct pa_intf *pa_intf = intf_priv;
	struct pa_device *pa_dev = pa_intf->pa_device;
	struct pa_lut_entry *entry;
	int idx;

	if (!pa_dev->opened)
		return -ENXIO;

	for (idx = 0; idx < pa_dev->lut_size; idx++) {
		entry = pa_dev->lut + idx;
		if (!entry->valid || !entry->in_use || entry->u.naddr != naddr)
			continue;
		keystone_pa_add_mac(pa_intf, entry->index, NULL, NULL,
				    PACKET_DROP, 0, PA_INVALID_PORT);
		entry->in_use = false;
		entry->u.naddr = NULL;
	}

	return 0;
}

static int pa_hwtstamp_ioctl(struct pa_intf *pa_intf,
			     struct ifreq *ifr, int cmd)
{
	struct hwtstamp_config cfg;

	if (pa_intf->pa_device->force_no_hwtstamp)
		return -EOPNOTSUPP;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags)
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
		pa_intf->tx_timestamp_enable = false;
		break;
	case HWTSTAMP_TX_ON:
		pa_intf->tx_timestamp_enable = true;
		break;
	default:
		return -ERANGE;
	}

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		pa_intf->rx_timestamp_enable = false;
		break;
	default:
		pa_intf->rx_timestamp_enable = true;
		break;
	}

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ? -EFAULT : 0;
}

int pa_ioctl(void *intf_priv, struct ifreq *req, int cmd)
{
	struct pa_intf *pa_intf = intf_priv;

	if (cmd == SIOCSHWTSTAMP)
		return pa_hwtstamp_ioctl(pa_intf, req, cmd);

	return -EOPNOTSUPP;
}

static int pa_attach(void *inst_priv, struct net_device *ndev, void **intf_priv)
{
	struct pa_device *pa_dev = inst_priv;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);
	struct pa_intf *pa_intf;
	int chan_id = 0;

	if (netcp_priv->cpsw_port)
		pa_dev->multi_if = 1;

	dev_dbg(pa_dev->dev, "pa_attach, port %d\n", netcp_priv->cpsw_port);
	pa_intf = devm_kzalloc(pa_dev->dev, sizeof(struct pa_intf), GFP_KERNEL);
	if (!pa_intf) {
		dev_err(pa_dev->dev, "memory allocation failed\n");
		return -ENOMEM;
	}

	pa_intf->net_device = ndev;
	pa_intf->pa_device = pa_dev;
	*intf_priv = pa_intf;

	/* Use pdsp5 with 0 as base */
	if (netcp_priv->cpsw_port)
		chan_id = netcp_priv->cpsw_port - 1;

	snprintf(pa_intf->tx_chan_name, sizeof(pa_intf->tx_chan_name),
		 "patx-pdsp5-%d", chan_id);
	netcp_txpipe_init(&pa_intf->tx_pipe, netdev_priv(ndev),
			  pa_intf->tx_chan_name, pa_dev->tx_data_queue_depth);

	if (pa_dev->csum_offload) {
		rtnl_lock();
		ndev->features		|= pa_dev->netif_features;
		ndev->hw_features	|= pa_dev->netif_features;
		ndev->wanted_features	|= pa_dev->netif_features;
		netdev_update_features(ndev);
		rtnl_unlock();
	}
	return 0;
}

static int pa_release(void *intf_priv)
{
	struct pa_intf *pa_intf = intf_priv;
	struct pa_device *pa_dev = pa_intf->pa_device;
	struct net_device *ndev = pa_intf->net_device;

	mutex_lock(&pa_modules_lock);
	if ((!--pa_dev->inuse_if_count) && (pa_dev->csum_offload)) {
		rtnl_lock();
		ndev->features		&= ~pa_dev->netif_features;
		ndev->hw_features	&= ~pa_dev->netif_features;
		ndev->wanted_features	&= ~pa_dev->netif_features;
		netdev_update_features(ndev);
		rtnl_unlock();
	}
	mutex_unlock(&pa_modules_lock);

	netif_napi_del(&pa_intf->tx_pipe.dma_poll_napi);

	devm_kfree(pa_dev->dev, pa_intf);
	return 0;
}

#define pa_cond_unmap(field)					\
	do {							\
		if (pa_dev->field)				\
			devm_iounmap(dev, pa_dev->field);	\
	} while(0)

static int pa_remove(struct netcp_device *netcp_device, void *inst_priv)
{
	struct pa_device *pa_dev = inst_priv;
	struct device *dev = pa_dev->dev;

	pa_cond_unmap(reg_mailbox);
	pa_cond_unmap(reg_packet_id);
	pa_cond_unmap(reg_lut2);
	pa_cond_unmap(reg_control);
	pa_cond_unmap(reg_timer);
	pa_cond_unmap(reg_stats);
	pa_cond_unmap(pa_iram);
	pa_cond_unmap(pa_sram);

	devm_kfree(dev, pa_dev->lut);
	devm_kfree(dev, pa_dev->ip_lut);
	devm_kfree(dev, pa_dev);
	return 0;
}

static int pa_probe(struct netcp_device *netcp_device,
		    struct device *dev,
		    struct device_node *node,
		    void **inst_priv)
{
	struct pa_device *pa_dev;
	int ret, len = 0, start, end, i, j;
	int table_size, num_ranges;
	u32 *prange, tmp[2];

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		return -ENODEV;
	}

	pa_dev = devm_kzalloc(dev, sizeof(struct pa_device), GFP_KERNEL);
	if (!pa_dev) {
		dev_err(dev, "memory allocation failed\n");
		return -ENOMEM;
	}
	*inst_priv = pa_dev;

	pa_dev->netcp_device = netcp_device;
	pa_dev->dev = dev;

	for (i = 0; i < DEVICE_PA_NUM_PDSPS; ++i) {
		ret = of_property_read_string_index(node, "firmware",
				i, &pa_dev->pdsp_fw[i]);
		if (ret < 0) {
			dev_warn(dev, "no firmware for pdsp %d\n", i);
			pa_dev->pdsp_fw[i] = NULL;
		} else {
			/*FIXME: make me dev_dbg*/
			dev_info(dev, "pdsp %d firmware: %s\n",
					i, pa_dev->pdsp_fw[i]);
		}
	}

	ret = of_property_read_u32(node, "tx_cmd_queue_depth",
				   &pa_dev->tx_cmd_queue_depth);
	if (ret < 0) {
		dev_err(dev, "missing tx_cmd_queue_depth parameter, err %d\n",
			ret);
		pa_dev->tx_cmd_queue_depth = 32;
	}
	dev_dbg(dev, "tx_cmd_queue_depth %u\n", pa_dev->tx_cmd_queue_depth);

	ret = of_property_read_u32(node, "tx_data_queue_depth",
				   &pa_dev->tx_data_queue_depth);
	if (ret < 0) {
		dev_err(dev, "missing tx_data_queue_depth parameter, err %d\n",
			ret);
		pa_dev->tx_data_queue_depth = 32;
	}
	dev_dbg(dev, "tx_data_queue_depth %u\n", pa_dev->tx_data_queue_depth);

	ret = of_property_read_u32(node, "rx_pool_depth",
				   &pa_dev->rx_pool_depth);
	if (ret < 0) {
		dev_err(dev, "missing rx_pool_depth parameter, err %d\n",
			ret);
		pa_dev->rx_pool_depth = 32;
	}
	dev_dbg(dev, "rx_pool_depth %u\n", pa_dev->rx_pool_depth);

	ret = of_property_read_u32(node, "rx_buffer_size",
				   &pa_dev->rx_buffer_size);
	if (ret < 0) {
		dev_err(dev, "missing rx_buffer_size parameter, err %d\n",
			ret);
		pa_dev->rx_buffer_size = 128;
	}
	dev_dbg(dev, "rx_buffer_size %u\n", pa_dev->rx_buffer_size);

	pa_dev->reg_mailbox	= devm_ioremap(dev, 0x2000000, 0x60);
	pa_dev->reg_packet_id	= devm_ioremap(dev, 0x2000400, 0x10);
	pa_dev->reg_lut2	= devm_ioremap(dev, 0x2000500, 0x40);
	pa_dev->reg_control	= devm_ioremap(dev, 0x2001000, 0x600);
	pa_dev->reg_timer	= devm_ioremap(dev, 0x2003000, 0x600);
	pa_dev->reg_stats	= devm_ioremap(dev, 0x2006000, 0x100);
	pa_dev->pa_iram		= devm_ioremap(dev, 0x2010000, 0x30000);
	pa_dev->pa_sram		= devm_ioremap(dev, 0x2040000, 0x8000);

	if (!pa_dev->reg_mailbox || !pa_dev->reg_packet_id ||
	    !pa_dev->reg_lut2 || !pa_dev->reg_control ||
	    !pa_dev->reg_timer || !pa_dev->reg_stats ||
	    !pa_dev->pa_sram || !pa_dev->pa_iram) {
		dev_err(dev, "failed to set up register areas\n");
		ret = -ENOMEM;
		goto exit;
	}

	ret = of_property_read_u32(node, "checksum-offload",
				   &pa_dev->csum_offload);
	if (ret < 0) {
		dev_warn(dev, "missing checksum-offload parameter, err %d\n",
			ret);
		pa_dev->csum_offload = CSUM_OFFLOAD_NONE;
	}
	if (pa_dev->csum_offload > CSUM_OFFLOAD_SOFT) {
		dev_err(dev, "invalid checksum-offload parameter %d, err %d\n",
			ret, pa_dev->csum_offload);
		pa_dev->csum_offload = CSUM_OFFLOAD_NONE;
	}
	dev_dbg(dev, "checksum-offload %u\n", pa_dev->csum_offload);

	ret = of_property_read_u32(node, "txhook-order",
				   &pa_dev->txhook_order);
	if (ret < 0) {
		dev_err(dev, "missing txhook-order parameter, err %d\n",
			ret);
		pa_dev->txhook_order = PA_TXHOOK_ORDER;
	}
	dev_dbg(dev, "txhook-order %u\n", pa_dev->txhook_order);

	if (pa_dev->csum_offload == CSUM_OFFLOAD_SOFT) {
		ret = of_property_read_u32(node, "txhook-softcsum",
					   &pa_dev->txhook_softcsum);
		if (ret < 0) {
			dev_err(dev, "missing txhook-softcsum parameter, err %d\n",
				ret);
			pa_dev->csum_offload = CSUM_OFFLOAD_NONE;
			pa_dev->txhook_order = ~0;
		}
		dev_dbg(dev, "txhook-softcsum %u\n", pa_dev->txhook_softcsum);
	}

	if (pa_dev->csum_offload != CSUM_OFFLOAD_NONE)
		pa_dev->netif_features = PA_NETIF_FEATURES;

	if (pa_dev->csum_offload == CSUM_OFFLOAD_HARD)
		pa_dev->netif_features |= NETIF_F_RXCSUM;

	ret = of_property_read_u32(node, "rxhook-order",
				   &pa_dev->rxhook_order);
	if (ret < 0) {
		dev_err(dev, "missing rxhook-order parameter, err %d\n",
			ret);
		pa_dev->rxhook_order = PA_RXHOOK_ORDER;
	}
	dev_dbg(dev, "rxhook-order %u\n", pa_dev->rxhook_order);

	ret = of_property_read_u32_array(node, "mark_mcast_match",
					pa_dev->mark_mcast_match, 2);
	if (ret < 0) {
		if (ret != -EINVAL) {
			dev_err(dev, "Error parsing \"mark_mcast_match\" value"
				" -- parameter ignored\n");
		}
		pa_dev->mark_mcast_match[0] = 0;
		pa_dev->mark_mcast_match[1] = 0;
	} else if (((pa_dev->mark_mcast_match[0] & 0xff) != 0) ||
		   ((pa_dev->mark_mcast_match[1] & 0xff) != 0) ||
		   ((pa_dev->mark_mcast_match[0] & ~pa_dev->mark_mcast_match[1]) != 0)) {
		dev_err(dev, "Error in \"mark_mcast_match\" value"
				" -- parameter ignored\n");
		pa_dev->mark_mcast_match[0] = 0;
		pa_dev->mark_mcast_match[1] = 0;
	}
	dev_dbg(dev, "mark_mcast_match = <%08x %08x>\n",
		 pa_dev->mark_mcast_match[0], pa_dev->mark_mcast_match[1]);

	if (!of_get_property(node, "lut-ranges", &len)) {
		dev_err(dev, "No lut-entry array in dt bindings for PA\n");
		return -ENODEV;
	}

	if (of_find_property(node, "force_no_hwtstamp", NULL)) {
		pa_dev->force_no_hwtstamp = 1;
		dev_warn(dev, "***** No PA timestamping *****\n");
	}

	prange = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!prange) {
		dev_err(dev, "memory allocation failed at PA lut entry range\n");
		return -ENOMEM;
	}
	len = len / sizeof(u32);
	if ((len % 2) != 0) {
		dev_err(dev, "invalid address map in dt binding\n");
		return -EINVAL;
	}
	num_ranges = len / 2;
	if (of_property_read_u32_array(node, "lut-ranges", prange, len)) {
		dev_err(dev, "No range-map array  in dt bindings\n");
		return -ENODEV;
	}

	table_size = prange[2 * num_ranges - 1] + 1;
	dev_dbg(dev, "lut size = %d\n", table_size);

	/* Initialize a table for storing entry listings locally */
	len = table_size * sizeof(struct pa_lut_entry);
	pa_dev->lut  = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!pa_dev->lut) {
		dev_err(dev, "devm_kzalloc mapping failed\n");
		return -ENOMEM;
	}
	pa_dev->lut_size = table_size;
	dev_dbg(dev, "lut size = %d\n", table_size);

	for (i = 0; i < num_ranges; i++) {
		start = prange[i * 2];
		end   = prange[i * 2 + 1];
		for (j = start; j <= end; j++) {
			pa_dev->lut[j].valid = true;
			pa_dev->lut[j].index = j;
			dev_dbg(dev, "setting entry %d to valid\n", j);
		}
	}

	devm_kfree(dev, prange);

	/* NOTE: DTS configuration MUST ensure that the completion queue &
	 * Rx flow for each interface is sequential.
	 */
	ret = of_property_read_u32_array(node, "rx-route", tmp, 2);
	if (ret) {
		dev_err(dev, "Couldn't get rx-route from dt bindings\n");
		return -ENODEV;
	} else {
		pa_dev->rx_queue_base = tmp[0];
		pa_dev->rx_flow_base = tmp[1];
	}

	/* Get IP lut ranges */
	if (!of_get_property(node, "ip-lut-ranges", &len)) {
		dev_err(dev,
		"No ip-lut-entry array in dt bindings for PA\n");
		return -ENODEV;
	}

	prange = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!prange) {
		dev_err(dev,
		"memory allocation failed for IP lut entry range\n");
		return -ENOMEM;
	}
	len = len / sizeof(u32);
	if ((len % 2) != 0) {
		dev_err(dev, "invalid address map in dt binding\n");
		return -EINVAL;
	}
	num_ranges = len / 2;
	if (of_property_read_u32_array(node, "ip-lut-ranges", prange, len)) {
		dev_err(dev, "No range-map array  in dt bindings\n");
		return -ENODEV;
	}
	table_size = prange[2 * num_ranges - 1] + 1;
	pa_dev->ip_lut_size = table_size;
	dev_dbg(dev, "IP lut size = %d\n", pa_dev->ip_lut_size);

	len = pa_dev->ip_lut_size * sizeof(struct pa_lut_entry);
	pa_dev->ip_lut  = devm_kzalloc(dev, len, GFP_KERNEL);

	for (i = 0; i < num_ranges; i++) {
		start = prange[i * 2];
		end   = prange[i * 2 + 1];
		for (j = start; j <= end; j++) {
			pa_dev->ip_lut[j].valid = true;
			pa_dev->ip_lut[j].index = j;
			dev_dbg(dev, "setting entry %d to valid\n", j);
		}
	}

	devm_kfree(pa_dev->dev, prange);
	spin_lock_init(&pa_dev->lock);
	spin_lock_init(&tstamp_lock);
	return 0;

exit:
	pa_remove(netcp_device, pa_dev);
	*inst_priv = NULL;
	return ret;
}


static struct netcp_module pa_module = {
	.name		= "keystone-pa",
	.owner		= THIS_MODULE,
	.probe		= pa_probe,
	.open		= pa_open,
	.close		= pa_close,
	.remove		= pa_remove,
	.attach		= pa_attach,
	.release	= pa_release,
	.add_addr	= pa_add_addr,
	.del_addr	= pa_del_addr,
	.ioctl		= pa_ioctl,
};

static int __init keystone_pa_init(void)
{
	return netcp_register_module(&pa_module);
}
module_init(keystone_pa_init);

static void __exit keystone_pa_exit(void)
{
	netcp_unregister_module(&pa_module);
}
module_exit(keystone_pa_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sandeep Paulraj <s-paulraj@ti.com>");
MODULE_DESCRIPTION("Packet Accelerator driver for Keystone devices");
