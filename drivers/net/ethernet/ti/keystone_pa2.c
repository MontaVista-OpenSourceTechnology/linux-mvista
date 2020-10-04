/*
 * Copyright (C) 2014 Texas Instruments Incorporated
 * Authors: Hao Zhang <hzhang@ti.com>
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
#include "keystone_pa2.h"
#include "keystone_pasahost2.h"

/* PA system registers */
#define PA2_MAILBOX_REGS_OFFSET			0x00000
#define PA2_RA_BRIDGE_REGS_OFFSET		0x00400
#define PA2_THREAD_MAPPER_REGS_OFFSET		0x00500
#define PA2_RA_REGS_OFFSET			0x00800
#define PA2_STATS_CTL_REGS_OFFSET		0x06000
#define PA2_QUERY_STATS_REGS_OFFSET		0x08000
#define PA2_COLLECT_STATS_REGS_OFFSET		0x0c000
#define PA2_SRAM_OFFSET				0x20000
#define PA2_SRAM_SIZE				0x08000

#define PA2_MAX_NUM_MAILBOX			16
#define PA2_STATS_CTL_ENABLE_ALLOC_MASK		BIT(31)

/* PA cluster registers */
#define PA2_CLUSTER_REGS_OFFSET			0x00400000
#define PA2_CLUSTER_REGS_SIZE			0x00100000
#define PA2_CLUSTER_REGS(x)			(PA2_CLUSTER_REGS_OFFSET + \
						 (PA2_CLUSTER_REGS_SIZE * x))
#define PA2_CLUSTER_SPLITTER_REGS_OFFSET	0x09800
#define PA2_CLUSTER_SPLITTER_REGS(x)	(PA2_CLUSTER_SPLITTER_REGS_OFFSET \
					 + PA2_CLUSTER_REGS(x))
#define PA2_CLUSTER_SRAM_OFFSET			0x80000
#define PA2_CLUSTER_SRAM_SIZE			0x10000
#define PA2_CLUSTER_SRAM_REGS(x)		(PA2_CLUSTER_SRAM_OFFSET \
						 + PA2_CLUSTER_REGS(x))
enum {
	PA2_CLUSTER0 = 0,
	PA2_CLUSTER1,
	PA2_CLUSTER2,
	PA2_CLUSTER3,
	PA2_CLUSTER4,
	PA2_CLUSTER5,
	PA2_CLUSTER6,
	PA2_CLUSTER7,
	PA2_CLUSTER8,
	PA2_NUM_CLUSTERS
};

#define PA2_CLUSTER_INGRESS0		PA2_CLUSTER0
#define PA2_CLUSTER_INGRESS1		PA2_CLUSTER1
#define PA2_CLUSTER_INGRESS2		PA2_CLUSTER2
#define PA2_CLUSTER_INGRESS3		PA2_CLUSTER3
#define PA2_CLUSTER_INGRESS4		PA2_CLUSTER4
#define PA2_CLUSTER_POST		PA2_CLUSTER5
#define PA2_CLUSTER_EGRESS0		PA2_CLUSTER6
#define PA2_CLUSTER_EGRESS1		PA2_CLUSTER7
#define PA2_CLUSTER_EGRESS2		PA2_CLUSTER8

#define PA2_CLUSTER_EF_REC1		PA2_CLUSTER_EGRESS0
#define PA2_CLUSTER_EF_REC2		PA2_CLUSTER_EGRESS0
#define PA2_CLUSTER_EF_REC3		PA2_CLUSTER_EGRESS1
#define PA2_CLUSTER_EF_REC4		PA2_CLUSTER_EGRESS2

/* PA Cluster Splitter
 * to avoid hardware bug: SOP + EOP + 32 + Control size <= 128
 * Restrict control size to 96, the SOP should be 896 - 128 */
#define PA2_SPLITTER_SOP_CTL_ENABLE_MASK		(0x80000000)

#define PA2_CLUSTER_SPLITTER_EOP_CTL		128
#define PA2_CLUSTER_SPLITTER_EOP_BUF_SIZE(x)	((x == 0) ? 0x4000 : 0x10000)
#define PA2_CLUSTER_SPLITTER_MOP_BUF_PTR		0xFFFC0000
#define PA2_CLUSTER_SPLITTER_SOP_CTL \
	(PA2_SPLITTER_SOP_CTL_ENABLE_MASK | (896 - 128))


/* PA Packet Processing Unit registers */
enum pa2_pdsp {
	PA2_PDSP0 = 0,
	PA2_PDSP1,
	PA2_PDSP2,
	PA2_PDSP3,
	PA2_PDSP4,
	PA2_PDSP5,
	PA2_PDSP6,
	PA2_PDSP7,
	PA2_PDSP8,
	PA2_PDSP9,
	PA2_PDSP10,
	PA2_PDSP11,
	PA2_PDSP12,
	PA2_PDSP13,
	PA2_PDSP14,
	PA2_NUM_PDSPS
};

#define PA2_INGRESS0_PDSP0		PA2_PDSP0
#define PA2_INGRESS0_PDSP1		PA2_PDSP1
#define PA2_INGRESS1_PDSP0		PA2_PDSP2
#define PA2_INGRESS1_PDSP1		PA2_PDSP3
#define PA2_INGRESS2_PDSP0		PA2_PDSP4
#define PA2_INGRESS3_PDSP0		PA2_PDSP5
#define PA2_INGRESS4_PDSP0		PA2_PDSP6
#define PA2_INGRESS4_PDSP1		PA2_PDSP7
#define PA2_POST_PDSP0			PA2_PDSP8
#define PA2_POST_PDSP1			PA2_PDSP9
#define PA2_EGRESS0_PDSP0		PA2_PDSP10
#define PA2_EGRESS0_PDSP1		PA2_PDSP11
#define PA2_EGRESS0_PDSP2		PA2_PDSP12
#define PA2_EGRESS1_PDSP0		PA2_PDSP13
#define PA2_EGRESS2_PDSP0		PA2_PDSP14

#define PA2_PPU_REGS_OFFSET		0x08000
#define PA2_PPU_REGS_SIZE		0x10000
#define PA2_PPU_REGS(x, y)		(PA2_CLUSTER_REGS(x) + \
					 (PA2_PPU_REGS_OFFSET + \
					 (PA2_PPU_REGS_SIZE * y)))
#define PA2_PPU_CTL_STATUS_REGS_OFFSET	0x0000
#define PA2_PPU_DEBUG_REGS_OFFSET	0x0400
#define PA2_PPU_CP_TIMER_REGS_OFFSET	0x0800
#define PA2_PPU_LUT1_REGS_OFFSET	0x1000
#define PA2_PPU_LUT2_REGS_OFFSET	0x1400
#define PA2_PPU_PCHECK_REGS_OFFSET	0x1c00
#define PA2_PPU_IRAM_OFFSET		0x4000
#define PA2_PPU_IRAM_SIZE		0x3000

struct pa2_cluster_pdsp_map {
	u32 cluster;
	u32 pdsp;
	u32 ver_base_addr;
};

#define PA2_PDSP_VERSION_SIZE		0x20
#define PA2_PDSP_VERSION_OFFSET(x, y)	(x + (y * PA2_PDSP_VERSION_SIZE))

/* PDSP/Cluster Mapping */
static const struct pa2_cluster_pdsp_map pa2_cluster_pdsp_map[PA2_NUM_PDSPS] = {
	{PA2_CLUSTER_INGRESS0, 0, 0x3f04},
	{PA2_CLUSTER_INGRESS0, 1, 0x3f04},
	{PA2_CLUSTER_INGRESS1, 0, 0x3f04},
	{PA2_CLUSTER_INGRESS1, 1, 0x3f04},
	{PA2_CLUSTER_INGRESS2, 0, 0x1f04},
	{PA2_CLUSTER_INGRESS3, 0, 0x1f04},
	{PA2_CLUSTER_INGRESS4, 0, 0x3f04},
	{PA2_CLUSTER_INGRESS4, 1, 0x3f04},
	{PA2_CLUSTER_POST, 0, 0x3f04},
	{PA2_CLUSTER_POST, 1, 0x3f04},
	{PA2_CLUSTER_EGRESS0, 0, 0x1f04},
	{PA2_CLUSTER_EGRESS0, 1, 0x1f04},
	{PA2_CLUSTER_EGRESS0, 2, 0x1f04},
	{PA2_CLUSTER_EGRESS1, 0, 0x0f04},
	{PA2_CLUSTER_EGRESS2, 0, 0x0f04}
};

#define	PA2_NETIF_FEATURES	(NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM)

#define PSTREAM_ROUTE_INGRESS0	2

#define PA2_PDSP_ALREADY_ACTIVE	0
#define PA2_PDSP_RESET_RELEASED	1
#define PA2_PDSP_NO_RESTART	2
#define PA2_MAX_PDSP_ENABLE_LOOP_COUNT	50000

#define PA2_INVALID_PORT		0xff

#define PA2_STATE_RESET			0  /* Sub-system state reset */
#define PA2_STATE_ENABLE		1  /* Sub-system state enable  */
#define PA2_STATE_QUERY			2  /* Query the Sub-system state */
#define PA2_STATE_INCONSISTENT		3  /* Sub-system is partially enabled */
#define PA2_STATE_INVALID_REQUEST	4  /* Invalid state command to the
					      Sub-system */
#define PA2_STATE_ENABLE_FAILED		5  /* The Sub-system did not respond
					      after restart */

/* pdsp LUT2 register */
#define PA2_REG_VAL_PDSP_LUT2_CLR_TABLE_GO	BIT(0)

/* pdsp control status register */
#define PA2_REG_VAL_PDSP_CTL_DISABLE_PDSP	1
#define PA2_REG_VAL_PDSP_CTL_RESET_PDSP	        0
#define PA2_REG_VAL_PDSP_CTL_STATE               BIT(15)
#define PA2_REG_VAL_PDSP_CTL_ENABLE              BIT(1)
#define PA2_REG_VAL_PDSP_CTL_SOFT_RESET          BIT(0)
#define PA2_REG_VAL_PDSP_CTL_ENABLE_PDSP(pcval)	(((pcval) << 16) | \
						 PA2_REG_VAL_PDSP_CTL_ENABLE | \
						PA2_REG_VAL_PDSP_CTL_SOFT_RESET)

#define PACKET_DROP	0
#define PACKET_PARSE	1
#define PACKET_HST	2

#define NT 32

#define PA2_SGLIST_SIZE	3

static const u32 pa2_ppu_regs_offset[PA2_NUM_PDSPS] = {
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS0, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS0, 1),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS1, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS1, 1),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS2, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS3, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS4, 0),
	PA2_PPU_REGS(PA2_CLUSTER_INGRESS4, 1),
	PA2_PPU_REGS(PA2_CLUSTER_POST, 0),
	PA2_PPU_REGS(PA2_CLUSTER_POST, 1),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS0, 0),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS0, 1),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS0, 2),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS1, 0),
	PA2_PPU_REGS(PA2_CLUSTER_EGRESS2, 0)
};

/*
 * PASS command ID formatting
 * Bit 14-15 is used to identify the type of table in the command comId field
 */
#define PA2_COMID_L2		(0 << 14)
#define PA2_COMID_L3		(1 << 14)
#define PA2_COMID_ACL		(2 << 14)
#define PA2_COMID_FC		(3 << 14)

#define PA2_COMID_L_MASK		(3 << 14)
#define PA2_COMID_IDX_MASK	(~(3 << 14))

#define PA2_PDSP_CONST_NUM_REG		32
static const u32 pa2_pdsp_const_reg_map[PA2_NUM_PDSPS][PA2_PDSP_CONST_NUM_REG] \
		= {
	/* Ingress0 PDSP0: Classify1 */
	{
		0xFFF84000,	/* C0: LUT1 Info */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000000,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF04000,	/* C6: CDE new packet input region */
		0xFFF00100,	/* C7: CDE new packet output region */
		0xFFF00200,	/* C8: CDE held packet region */
		0xFFF08800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF09000,	/* C10: LUT1 Registers */
		0xFFF09400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0xFFF80000,	/* C14: PDSP Context */
		0x00000000,	/* C15: Reserved*/
		0xFFF80400,	/* C16: IP Traffic Flow */
		0xFFF80800,	/* C17: IP Reassembly Control Block */
		0xFFF80C00,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF81000,	/* C21: Classify1 Parsing Call Table */
		0x00000000,	/* C22: Reserved */
		0xFFF83F00,	/* C23: PDSP Info */
		0xFFF81400,	/* C24: Next Route Global address table */
		0xFF980000,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Ingress0 PDSP1: Classify1 */
	{
		0xFFF88000,	/* C0: LUT1 Info */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000010,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF14000,	/* C6: CDE new packet input region */
		0xFFF10100,	/* C7: CDE new packet output region */
		0xFFF10200,	/* C8: CDE held packet region */
		0xFFF18800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF19000,	/* C10: LUT1 Registers */
		0xFFF19400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0xFFF80100,	/* C14: PDSP Context */
		0x00000000,	/* C15: Reserved*/
		0xFFF80400,	/* C16: IP Traffic Flow */
		0xFFF80800,	/* C17: IP Reassembly Control Block */
		0xFFF80D00,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF81100,	/* C21: Classify1 Parsing Call Table */
		0x00000000,	/* C22: Reserved */
		0xFFF83F20,	/* C23: PDSP Info */
		0xFFF81400,	/* C24: Next Route Global address table */
		0xFF980040,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
},
	/* Ingress1 PDSP0: Classify1 */
	{
		0xFFF84000,	/* C0: LUT1 Info */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000020,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF04000,	/* C6: CDE new packet input region */
		0xFFF00100,	/* C7: CDE new packet output region */
		0xFFF00200,	/* C8: CDE held packet region */
		0xFFF08800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF09000,	/* C10: LUT1 Registers */
		0xFFF09400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0xFFF80000,	/* C14: PDSP Context */
		0x00000000,	/* C15: Reserved*/
		0xFFF80400,	/* C16: IP Traffic Flow */
		0xFFF80800,	/* C17: IP Reassembly Control Block */
		0xFFF80C00,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF81000,	/* C21: Classify1 Parsing Call Table */
		0x00000000,	/* C22: Reserved */
		0xFFF83F00,	/* C23: PDSP Info */
		0xFFF81400,	/* C24: Next Route Global address table */
		0xFF980080,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Ingress1 PDSP1: Classify1 */
	{
		0xFFF88000,	/* C0: LUT1 Info */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000030,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF14000,	/* C6: CDE new packet input region */
		0xFFF10100,	/* C7: CDE new packet output region */
		0xFFF10200,	/* C8: CDE held packet region */
		0xFFF18800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF19000,	/* C10: LUT1 Registers */
		0xFFF19400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0xFFF80100,	/* C14: PDSP Context */
		0x00000000,	/* C15: Reserved*/
		0xFFF80400,	/* C16: IP Traffic Flow */
		0xFFF80800,	/* C17: IP Reassembly Control Block */
		0xFFF80D00,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF81100,	/* C21: Classify1 Parsing Call Table */
		0x00000000,	/* C22: Reserved */
		0xFFF83F20,	/* C23: PDSP Info */
		0xFFF81400,	/* C24: Next Route Global address table */
		0xFF9800C0,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Ingress2 PDSP0: Classify1 */
	{
		0xFFF82000,	/* C0: LUT1 Info */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000040,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF04000,	/* C6: CDE new packet input region */
		0xFFF00100,	/* C7: CDE new packet output region */
		0xFFF00200,	/* C8: CDE held packet region */
		0xFFF08800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF09000,	/* C10: LUT1 Registers */
		0xFFF09400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0xFFF80000,	/* C14: PDSP Context */
		0x00000000,	/* C15: Reserved*/
		0xFFF80400,	/* C16: IP Traffic Flow */
		0xFFF80800,	/* C17: IP Reassembly Control Block */
		0xFFF80C00,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF81000,	/* C21: Classify1 Parsing Call Table */
		0x00000000,	/* C22: Reserved */
		0xFFF81F00,	/* C23: PDSP Info */
		0xFFF81400,	/* C24: Next Route Global address table */
		0xFF980100,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Ingress3 PDSP0: Classify1 */
	{
		0xFFF82000,	/* C0: LUT1 Info */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000050,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF04000,	/* C6: CDE new packet input region */
		0xFFF00100,	/* C7: CDE new packet output region */
		0xFFF00200,	/* C8: CDE held packet region */
		0xFFF08800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF09000,	/* C10: LUT1 Registers */
		0xFFF09400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0xFFF80000,	/* C14: PDSP Context */
		0x00000000,	/* C15: Reserved*/
		0xFFF80400,	/* C16: IP Traffic Flow */
		0xFFF80800,	/* C17: IP Reassembly Control Block */
		0xFFF80C00,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF81000,	/* C21: Classify1 Parsing Call Table */
		0x00000000,	/* C22: Reserved */
		0xFFF81F00,	/* C23: PDSP Info */
		0xFFF81400,	/* C24: Next Route Global address table */
		0xFF980140,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Ingress4 PDSP0: Classify1 */
	{
		0xFFF84000,	/* C0: LUT1 Info */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000060,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF04000,	/* C6: CDE new packet input region */
		0xFFF00100,	/* C7: CDE new packet output region */
		0xFFF00200,	/* C8: CDE held packet region */
		0xFFF08800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF09000,	/* C10: LUT1 Registers */
		0xFFF09400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0xFFF80000,	/* C14: PDSP Context */
		0x00000000,	/* C15: Reserved*/
		0xFFF80400,	/* C16: IP Traffic Flow */
		0xFFF80800,	/* C17: IP Reassembly Control Block */
		0xFFF80C00,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF81000,	/* C21: Classify1 Parsing Call Table */
		0x00000000,	/* C22: Reserved */
		0xFFF83F00,	/* C23: PDSP Info */
		0xFFF81400,	/* C24: Next Route Global address table */
		0xFF980180,	/* C25: User Stats CB and FIFO
				   (Global address of Post Cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Ingress4 PDSP1: Classify2 */
	{
		0xFFF88000,	/* C0: LUT1 Info */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000070,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF14000,	/* C6: CDE new packet input region */
		0xFFF10100,	/* C7: CDE new packet output region */
		0xFFF10200,	/* C8: CDE held packet region */
		0xFFF18800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF19000,	/* C10: LUT1 Registers */
		0xFFF19400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0x00000000,	/* C14: Reserved*/
		0x00000000,	/* C15: Reserved*/
		0xFFF81800,	/* C16: Custom2 Info */
		0x00000000,	/* C17: Reserved*/
		0x00000000,	/* C18: Reserved*/
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF81100,	/* C21: Classify2 Parsing Call Table */
		0x00000000,	/* C22: Reserved */
		0xFFF83F20,	/* C23: PDSP Info */
		0xFFF81400,	/* C24: Next Route Global address table */
		0xFF9801C0,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Post PDSP0: Modifier */
	{
		0xFFF80000,	/* C0: User Stats FIFO Base */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000080,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF04000,	/* C6: CDE new packet input region */
		0xFFF00100,	/* C7: CDE new packet output region */
		0xFFF00200,	/* C8: CDE held packet region */
		0xFFF08800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF09000,	/* C10: LUT1 Registers */
		0xFFF09400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0x00000000,	/* C14: Reserved*/
		0xFFF80400,	/* C15: User Stats Control Block */
		0xFFF80800,	/* C16: User Stats  */
		0xFFF81000,	/* C17: Command Set Table */
		0xFFF81800,	/* C18: Multi-route table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF82800,	/* C21: CRC Verify FIFO */
		0xFFF82900,	/* C22: Split Context FIFO */
		0xFFF83F00,	/* C23: PDSP Info*/
		0x00000000,	/* C24: Reserved */
		0xFFF80200,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Post PDSP1: Modifier */
	{
		0xFFF80000,	/* C0: User Stats FIFO Base */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF000090,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF14000,	/* C6: CDE new packet input region */
		0xFFF10100,	/* C7: CDE new packet output region */
		0xFFF10200,	/* C8: CDE held packet region */
		0xFFF18800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF19000,	/* C10: LUT1 Registers */
		0xFFF19400,	/* C11: LUT2 Registers */
		0x00000000,	/* C12: Reserved*/
		0x00000000,	/* C13: Reserved*/
		0x00000000,	/* C14: Reserved*/
		0xFFF80400,	/* C15: User Stats Control Block */
		0xFFF80800,	/* C16: User Stats  */
		0xFFF81000,	/* C17: Command Set Table */
		0xFFF82000,	/* C18: Multi-route table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF82800,	/* C21: CRC Verify FIFO */
		0xFFF82900,	/* C22: Split Context FIFO */
		0xFFF83F20,	/* C23: PDSP Info*/
		0x00000000,	/* C24: Reserved */
		0xFFF80240,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0x00000000,	/* C26: Reserved*/
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Egress0 PDSP0: Flow Cache */
	{
		0xFFF82000,	/* C0: LUT1 Info */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF0000A0,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF04000,	/* C6: CDE new packet input region */
		0xFFF00100,	/* C7: CDE new packet output region */
		0xFFF00200,	/* C8: CDE held packet region */
		0xFFF08800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF09000,	/* C10: LUT1 Registers */
		0xFFF09400,	/* C11: LUT2 Registers */
		0xFFF84000,	/* C12: Egress Flow Record0 */
		0xFFF88000,	/* C13: Egress Flow Record1 */
		0xFFF80000,	/* C14: PDSP Context */
		0xFF980400,	/* C15: User Stats Control Block */
		0xFF980800,	/* C16: User Stats  */
		0xFFF81000,	/* C17: Modify Context */
		0xFFF80200,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF80500,	/* C21: Parse table */
		0x00000000,	/* C22: Reserved */
		0xFFF81F00,	/* C23: PDSP Info */
		0xFFF80A00,	/* C24: Temporary Buffer */
		0xFF980280,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0xFF020400,	/* C26: Eflow Exception route */
		0x00000000,	/* C27: Reserved*/
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Egress0 PDSP1: Flow Cache */
	{
		0x00000000,	/* C0: Reserved */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF0000B0,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF14000,	/* C6: CDE new packet input region */
		0xFFF10100,	/* C7: CDE new packet output region */
		0xFFF10200,	/* C8: CDE held packet region */
		0xFFF18800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF19000,	/* C10: LUT1 Registers */
		0xFFF19400,	/* C11: LUT2 Registers */
		0xFFF84000,	/* C12: Egress Flow Record0 */
		0xFFF88000,	/* C13: Egress Flow Record1 */
		0x00000000,	/* C14: Reserved */
		0xFF980400,	/* C15: User Stats Control Block */
		0xFF980800,	/* C16: User Stats  */
		0xFFF81000,	/* C17: Modify Context */
		0xFFF80300,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF80600,	/* C21: Parse table */
		0x00000000,	/* C22: Reserved */
		0xFFF81F20,	/* C23: PDSP Info */
		0xFFF80A00,	/* C24: Temporary Buffer */
		0xFF9802C0,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0xFF020400,	/* C26: Eflow Exception route */
		0xFFF80800,	/* C27: Command Buffer */
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Egress0 PDSP2: Flow Cache */
	{
		0x00000000,	/* C0: Reserved */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF0000C0,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF24000,	/* C6: CDE new packet input region */
		0xFFF20100,	/* C7: CDE new packet output region */
		0xFFF20200,	/* C8: CDE held packet region */
		0xFFF28800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF29000,	/* C10: LUT1 Registers */
		0xFFF29400,	/* C11: LUT2 Registers */
		0xFFF84000,	/* C12: Egress Flow Record0 */
		0xFFF88000,	/* C13: Egress Flow Record1 */
		0x00000000,	/* C14: Reserved */
		0xFF980400,	/* C15: User Stats Control Block */
		0xFF980800,	/* C16: User Stats  */
		0xFFF81000,	/* C17: Modify Context */
		0xFFF80400,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Exception Routes */
		0xFFF80700,	/* C21: Parse table */
		0x00000000,	/* C22: Reserved */
		0xFFF81F40,	/* C23: PDSP Info */
		0xFFF80A00,	/* C24: Temporary Buffer */
		0xFF980300,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0xFF020400,	/* C26: Eflow Exception route */
		0xFFF80900,	/* C27: Command Buffer */
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Egress1 PDSP0: Flow Cache */
	{
		0x00000000,	/* C0: Reserved */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF0000D0,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF04000,	/* C6: CDE new packet input region */
		0xFFF00100,	/* C7: CDE new packet output region */
		0xFFF00200,	/* C8: CDE held packet region */
		0xFFF08800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF09000,	/* C10: LUT1 Registers */
		0xFFF09400,	/* C11: LUT2 Registers */
		0xFFF81000,	/* C12: Egress Flow Record2 */
		0x00000000,	/* C13: Reserved*/
		0xFFF80000,	/* C14: PDSP Context */
		0xFF980400,	/* C15: User Stats Control Block */
		0xFF980800,	/* C16: User Stats  */
		0xFFF81000,	/* C17: Modify Context */
		0xFFF80200,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Egress Exception Routes */
		0xFFF80500,	/* C21: Parse table */
		0x00000000,	/* C22: Reserved */
		0xFFF80F00,	/* C23: PDSP Info */
		0xFFF80A00,	/* C24: Temporary Buffer */
		0xFF980340,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0xFF020400,	/* C26: Eflow Exception route */
		0xFFF80800,	/* C27: Command Buffer */
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000	/* C31: Reserved*/
	},
	/* Egress2 PDSP0: Flow Cache */
	{
		0x00000000,	/* C0: Reserved */
		0xFFF80000,	/* C1: Loacl SRAM */
		0xFF020000,	/* C2: Global SRAM */
		0xFF408800,	/* C3: System Timer (PDSP0 Timer) */
		0xFF0000E0,	/* C4: MailBox */
		0x00000000,	/* C5: Reserved*/
		0xFFF04000,	/* C6: CDE new packet input region */
		0xFFF00100,	/* C7: CDE new packet output region */
		0xFFF00200,	/* C8: CDE held packet region */
		0xFFF08800,	/* C9: PDSP Timer (PDSP specific) */
		0xFFF09000,	/* C10: LUT1 Registers */
		0xFFF09400,	/* C11: LUT2 Registers */
		0xFFF81000,	/* C12: Egress Flow Record3 */
		0x00000000,	/* C13: Reserved*/
		0xFFF80000,	/* C14: PDSP Context */
		0xFF980400,	/* C15: User Stats Control Block */
		0xFF980800,	/* C16: User Stats  */
		0xFFF81000,	/* C17: Modify Context */
		0xFFF80200,	/* C18: IP Protocol Table */
		0xFF020000,	/* C19: Custom LUT1 and global configuration */
		0xFF020200,	/* C20: Egress Exception Routes */
		0xFFF80500,	/* C21: Parse table */
		0x00000000,	/* C22: Reserved */
		0xFFF80F00,	/* C23: PDSP Info */
		0xFFF80A00,	/* C24: Temporary Buffer */
		0xFF980380,	/* C25: User Stats CB and FIFO
				   (Global address of Post cluster) */
		0xFF020400,	/* C26: Eflow Exception route */
		0xFFF80800,	/* C27: Command Buffer */
		0xFF020500,	/* C28: Port (Interface-based) configurations */
		0x00000000,	/* C29: Reserved*/
		0x00000000,	/* C30: Reserved*/
		0x00000000 	/* C31: Reserved*/
	}
};

/* Offset 0x0000 */
struct pa2_mailbox_regs {
	u32 pdsp_mailbox_slot0;
	u32 pdsp_mailbox_slot1;
	u32 pdsp_mailbox_slot2;
	u32 pdsp_mailbox_slot3;
};

/* Offset 0x0400 */
struct pa2_ra_bridge_regs {
	u32 rsvd0;
	u32 config;
};

/* Offset 0x0500 */
struct pa2_thread_mapper_regs {
	u32 map[16];
};

/* Offset 0x0400 */
struct pa2_ra_heap_region_regs {
	u32 low;
	u32 high;
};

struct pa2_ra_flow_override_regs {
	u32 timeout;
	u32 critical_err;
	u32 non_critical_err;
	u32 rsvd0;
};

struct pa2_ra_stats_regs {
	u32 pkts_reasm;
	u32 total_frags;
	u32 total_pkts;
	u32 context_timeout_w_sop;
	u32 context_timeout_w_sop_bytes;
	u32 context_timeout_wo_sop;
	u32 context_timeout_wo_sop_bytes;
	u32 rsvd0[2];
	u32 overlap_ipv6_discard;
	u32 overlap_ipv6_discard_bytes;
	u32 large_pkts;
	u32 ipv4_tcp_err;
	u32 frag_len_err;
	u32 illegal_ipv4_ihl;
	u32 illegal_small_pkt;
	u32 illegal_frag_len;
	u32 already_completed_discard;
	u32 already_completed_discard_bytes;
	u32 rsvd1[5];
};

/* Offset 0x0800 */
struct pa2_ra_regs {
	u32 revision;
	u32 config;
	u32 total_contexts;
	u32 discard_thresh;
	u32 timeout_val;
	u32 tick_val;
	u32 vbusm_config;
	u32 heap_region_thresh;
	struct pa2_ra_heap_region_regs heap_region[2];
	u32 rsvd0[4];
	struct pa2_ra_flow_override_regs flow_override[2];
	u32 rsvd1[12];
	u32 context_forced_timeout;
	u32 rsvd2[3];
	struct pa2_ra_stats_regs stats[2];
};

/* Offset 0x6000 */
struct pa2_stats_ctl_regs {
	u32 revision;
	u32 soft_reset;
	u32 enable_alloc;
	u32 counter_update;
	u32 timer_ctl;
	u32 timer_load;
	u32 timer_val;
	u32 pkt_routing_info;
};

/* Offset 0x8000 */
struct pa2_query_stats_regs {
	u32 stats[0x1000];
};

/* Offset 0xc000 */
struct pa2_collect_stats_regs {
	u32 stats[0x1000];
};

struct pa2_cl_splitter_regs {
	u32 revision;
	u32 rsvd0[3];
	u32 sop_ctl;
	u32 eop_ctl;
	u32 rsvd1[2];
	u32 mop_buf_size;
	u32 mop_buf_ptr;
};

/* PA Cluster registers, offset 0x0040_0000 + (clNum)*0x0010_0000 */
struct pa2_cluster {
	/* PA Cluster splitter regs, offset 0x9800 + cluster base */
	struct pa2_cl_splitter_regs __iomem	*splitter;
	/* PA SRAM, offset 0x80000 + PPU base */
	void __iomem				*sram;
};


struct pa2_ppu_ctl_status_regs {
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
};

struct pa2_ppu_debug_regs {
	u32 igp[32];	/* Internal General Purpose Register */
	u32 icte[32];	/* Internal Contants Table Entry Register */
};

struct pa2_ppu_cp_timer_regs {
	u32 timer_control;
	u32 timer_load;
	u32 timer_value;
	u32 timer_interrupt;
};

struct pa2_ppu_lut1_regs {
	u32 revision;
	u32 control;
	u32 config;
};

struct pa2_ppu_lut2_regs {
	u32 revision;
	u32 clr_table;
	u32 rsvd0[2];
	u32 max_entry_count;
	u32 curr_entry_count;
	u32 rsvd1[2];
	u32 add_data[4];
	u32 add_del_key[2];
	u32 add_del_control;
};

/* PA pcheck recipe control */
#define PA2_PCHECK_CONTROL_RSHIFT_MASK		(0x00000002u)
#define PA2_PCHECK_CONTROL_RSHIFT_SHIFT		(0x00000001u)
#define PA2_PCHECK_CONTROL_RSHIFT_RESETVAL	(0x00000000u)

#define PA2_PCHECK_CONTROL_FINAL_NOT_MASK	(0x00000001u)
#define PA2_PCHECK_CONTROL_FINAL_NOT_SHIFT	(0x00000000u)
#define PA2_PCHECK_CONTROL_FINAL_NOT_RESETVAL	(0x00000000u)

#define PA2_PCHECK_CONTROL_RESETVAL		(0x00000000u)

struct pa2_pcheck_recipe_regs {
	u32 control;
	u32 table[15];
};

struct pa2_ppu_pcheck_regs {
	u32 revision;
	u32 rsvd0[15];
	struct pa2_pcheck_recipe_regs recipe[4];
};

/* PA PPU registers, offset: 0x8000 + (pdspNum)*0x10000 + cluster offset */
struct pa2_ppu {
	/* PPU PDSP control/status regs, offset 0x0000 + PPU base */
	struct pa2_ppu_ctl_status_regs __iomem	*ctl_status;
	/* PPU PDSP debug regs, offset 0x0400 + PPU base */
	struct pa2_ppu_debug_regs __iomem	*debug;
	/* PPU CP Timer regs, offset 0x0800 + PPU base */
	struct pa2_ppu_cp_timer_regs __iomem	*cp_timer;
	/* PPU LUT1 regs, offset 0x1000 + PPU base */
	struct pa2_ppu_lut1_regs __iomem	*lut1;
	/* PPU LUT2 regs, offset 0x1400 + PPU base */
	struct pa2_ppu_lut2_regs __iomem	*lut2;
	/* PPU pcheck regs, offset 0x1c00 + PPU base */
	struct pa2_ppu_pcheck_regs __iomem	*pcheck;
	void __iomem				*iram;
};

#define	CSUM_OFFLOAD_NONE	0
#define	CSUM_OFFLOAD_HARD	1
#define	CSUM_OFFLOAD_SOFT	2

#define	PA2_TXHOOK_ORDER	10
#define	PA2_RXHOOK_ORDER	10

static DEFINE_MUTEX(pa2_modules_lock);

struct pa2_lut_entry {
	int			index;
	bool			valid, in_use;
	struct netcp_addr	*naddr;
};

struct pa2_intf {
	struct pa2_device		*pa_device;
	struct net_device		*net_device;
	struct netcp_tx_pipe		 tx_pipe;
	unsigned			 data_flow_num;
	unsigned			 data_queue_num;
	u32				 saved_ss_state;
	char				 tx_chan_name[24];
};

struct pa2_device {
	struct netcp_device		*netcp_device;
	struct device			*dev;
	struct clk			*clk;
	struct dma_chan			*pdsp0_tx_channel;
	struct dma_chan			*rx_channel;
	const char			*rx_chan_name;
	unsigned			 cmd_flow_num;
	unsigned			 cmd_queue_num;

	struct pa2_mailbox_regs __iomem		*reg_mailbox;
	struct pa2_ra_bridge_regs __iomem	*reg_ra_bridge;
	struct pa2_thread_mapper_regs __iomem	*reg_thread_mapper;
	struct pa2_ra_regs __iomem		*reg_ra;
	struct pa2_stats_ctl_regs   __iomem	*reg_stats_ctl;
	struct pa2_query_stats_regs   __iomem	*reg_query_stats;
	struct pa2_collect_stats_regs   __iomem	*reg_collect_stats;
	void __iomem				*pa_sram;

	struct pa2_cluster		cluster[PA2_NUM_CLUSTERS];
	struct pa2_ppu			ppu[PA2_NUM_PDSPS];
	u8				*mc_list;
	u8				 addr_count;
	struct tasklet_struct		 task;
	spinlock_t			 lock;

	u32				 tx_cmd_queue_depth;
	u32				 tx_data_queue_depth;
	u32				 rx_pool_depth;
	u32				 rx_buffer_size;
	u32				 csum_offload;
	u32				 txhook_order;
	u32				 txhook_softcsum;
	u32				 rxhook_order;
	u32				 multi_if;
	u32				 mark_mcast_match[2];
	u32				 inuse_if_count;
	u32				 lut_inuse_count;
	struct pa2_lut_entry		 *lut;
	u32				 lut_size;

	const char			*pdsp_fw[PA2_NUM_PDSPS];
	u32				 opened;
};

#define pa2_from_module(data)	container_of(data, struct pa2_device, module)
#define pa2_to_module(pa)	(&(pa)->module)

struct pa2_packet {
	struct scatterlist		 sg[PA2_SGLIST_SIZE];
	int				 sg_ents;
	struct pa2_device		*priv;
	struct dma_chan			*chan;
	struct dma_async_tx_descriptor	*desc;
	dma_cookie_t			 cookie;
	u32				 epib[4];
	u32				 psdata[6];
	struct completion		 complete;
	void				*data;
};

#define pa2_cond_unmap(field)				\
	do {						\
		if (field) {				\
			devm_iounmap(dev, field);	\
			field = NULL;			\
		}					\
	} while (0)

static void pdsp_fw_put(u32 *dest, const u32 *src, u32 wc)
{
	int i;

	for (i = 0; i < wc; i++)
		*dest++ = be32_to_cpu(*src++);
}

static inline void swizFwd(struct pa2_frm_forward *fwd)
{
	fwd->forward_type	= fwd->forward_type;
	fwd->flow_id		= fwd->flow_id;
	fwd->queue		= cpu_to_be16(fwd->queue);

	if (fwd->forward_type == PA2FRM_FORWARD_TYPE_HOST) {
		fwd->u.host.context      = cpu_to_be32(fwd->u.host.context);
		fwd->u.host.ctrl_bitmap  = fwd->u.host.ctrl_bitmap;
		fwd->u.host.multi_idx    = fwd->u.host.multi_idx;
		fwd->u.host.pa_pdsp_router = fwd->u.host.pa_pdsp_router;
	} else if (fwd->forward_type == PA2FRM_FORWARD_TYPE_SA) {
		fwd->u.sa.sw_info_0 = cpu_to_be32(fwd->u.sa.sw_info_0);
		fwd->u.sa.sw_info_1 = cpu_to_be32(fwd->u.sa.sw_info_1);
	} else if (fwd->forward_type == PA2FRM_FORWARD_TYPE_SRIO) {
		fwd->u.srio.ps_info0 = cpu_to_be32(fwd->u.srio.ps_info0);
		fwd->u.srio.ps_info1 = cpu_to_be32(fwd->u.srio.ps_info1);
		fwd->u.srio.pkt_type = fwd->u.srio.pkt_type;
	} else if (fwd->forward_type == PA2FRM_FORWARD_TYPE_ETH) {
		fwd->u.eth.ps_flags	= fwd->u.eth.ps_flags;
	} else if (fwd->forward_type == PA2FRM_FORWARD_TYPE_PA) {
		fwd->u.pa.pa_dest	= fwd->u.pa.pa_dest;
		fwd->u.pa.custom_type	= fwd->u.pa.custom_type;
		fwd->u.pa.custom_idx	= fwd->u.pa.custom_idx;
		fwd->u.pa.flags		= fwd->u.pa.flags;
	}
}

static inline void swizFcmd(struct pa2_frm_command *fcmd)
{
	fcmd->status		=  fcmd->status;
	fcmd->pdsp_index	=  fcmd->pdsp_index;
	fcmd->command_result	=  cpu_to_be16(fcmd->command_result);
	fcmd->com_id		=  cpu_to_be16(fcmd->com_id);
	fcmd->command		=  fcmd->command;
	fcmd->magic		=  fcmd->magic;
	fcmd->ret_context	=  cpu_to_be32(fcmd->ret_context);
	fcmd->reply_queue	=  cpu_to_be16(fcmd->reply_queue);
	fcmd->reply_dest	=  fcmd->reply_dest;
	fcmd->flow_id		=  fcmd->flow_id;
}

static inline void swizAl1(struct pa2_frm_cmd_add_lut1 *al1)
{
	al1->index		=  cpu_to_be16(al1->index);
	al1->type		=  al1->type;
	al1->cust_index		=  al1->cust_index;
	al1->vlink_num		=  cpu_to_be16(al1->vlink_num);
	al1->stats_index	=  cpu_to_be16(al1->stats_index);

	if (al1->type == PA2FRM_COM_ADD_LUT1_STANDARD) {
		al1->u.mac.etype	= cpu_to_be16(al1->u.mac.etype);
		al1->u.mac.session_id	= cpu_to_be16(al1->u.mac.session_id);
		al1->u.mac.mpls		= cpu_to_be32(al1->u.mac.mpls);
		al1->u.mac.pkt_flags	= al1->u.mac.pkt_flags;
		al1->u.mac.dst_mac5	= al1->u.mac.dst_mac5;
		al1->u.mac.vlan_id1	= cpu_to_be16(al1->u.mac.vlan_id1);
		al1->u.mac.vlan_id2	= cpu_to_be16(al1->u.mac.vlan_id2);
		al1->u.mac.pkt_type	= al1->u.mac.pkt_type;
		al1->u.mac.in_port	= al1->u.mac.in_port;
		al1->u.mac.vlan_pri1	= cpu_to_be16(al1->u.mac.vlan_pri1);
		al1->u.mac.vlan_pri2	= cpu_to_be16(al1->u.mac.vlan_pri2);
		al1->u.mac.src_vc	= cpu_to_be16(al1->u.mac.src_vc);
	} else if (al1->type == PA2FRM_COM_ADD_LUT1_SRIO) {
		al1->u.srio.next_hdr_offset =
			cpu_to_be16(al1->u.srio.next_hdr_offset);
		al1->u.srio.next_hdr	= al1->u.srio.next_hdr;
		al1->u.srio.pkt_flags	= al1->u.srio.pkt_flags;
		al1->u.srio.type_param2	= al1->u.srio.type_param2;
		al1->u.srio.type_param1	= cpu_to_be16(al1->u.srio.type_param1);
		al1->u.srio.src_id	= cpu_to_be16(al1->u.srio.src_id);
		al1->u.srio.dest_id	= cpu_to_be16(al1->u.srio.dest_id);
		al1->u.srio.pkt_type	= al1->u.srio.pkt_type;
		al1->u.srio.cc		= al1->u.srio.cc;
		al1->u.srio.pri		= cpu_to_be16(al1->u.srio.pri);
		al1->u.srio.src_vc	= cpu_to_be16(al1->u.srio.src_vc);
	} else {
		al1->u.custom.pkt_type	=  al1->u.custom.pkt_type;
		al1->u.custom.src_vc	=  cpu_to_be16(al1->u.custom.src_vc);
	}

	al1->range1_hi	=  cpu_to_be16(al1->range1_hi);
	al1->range0_hi	=  cpu_to_be16(al1->range0_hi);
	al1->cbwords0	=  cpu_to_be32(al1->cbwords0);
	al1->cbwords1	=  cpu_to_be32(al1->cbwords1);
	al1->bit_mask	=  cpu_to_be16(al1->bit_mask);
	al1->priority	=  cpu_to_be16(al1->priority);

	swizFwd(&(al1->match));
	swizFwd(&(al1->next_fail));
}

static int pa2_conv_fc_routing_info(struct pa2_frm_forward *fwd_info,
				   struct pa2_ef_op_info *ef_info)
{
	if (ef_info == NULL)
		return PA2_ERR_CONFIG;

	fwd_info->forward_type = PA2FRM_FORWARD_TYPE_EFLOW;

	if (ef_info->ctrl_flags & PA2_EF_OP_CONTROL_FLAG_FC_LOOKUP)
		/* Trigger Flow cache operation */
		fwd_info->u.ef.ctrl_flags = PA2FRM_EF_CTRL_FC_LOOKUP;
	else {
		/* Use Egress Flow records directly */
		fwd_info->u.ef.valid_bitmap  = (u8)ef_info->valid_bitmap;
		fwd_info->u.ef.lvl1_rec_idx = (u8)ef_info->lvl1_index;
		fwd_info->u.ef.lvl2_rec_idx = (u8)ef_info->lvl2_index;
		fwd_info->u.ef.lvl3_rec_idx = (u8)ef_info->lvl3_index;
		fwd_info->u.ef.lvl4_rec_idx = (u8)ef_info->lvl4_index;
	}

	return 0;
}

static int pa2_conv_routing_info(struct pa2_frm_forward *fwd_info,
			 struct pa2_route_info2 *route_info,
			 int cmd_dest, u16 fail_route,
			 u16 dest_pdsp, u8 pa_flags)
{
	u8 *pcmd = fwd_info->u.host.cmd;
	u8 ps_flags = 0;
	u32 no_fcmd = 0;
	fwd_info->flow_id = route_info->flow_id;
	fwd_info->queue   = route_info->queue;

	if ((route_info->dest == PA2_DEST_HOST) ||
	    (route_info->dest == PA2_DEST_EMAC)) {
		if (route_info->valid_bitmap & \
		    PA2_ROUTE_INFO_VALID_PKTTYPE_EMAC) {
			ps_flags = (route_info->pkt_type_emac_ctrl & \
				    PA2_EMAC_CTRL_CRC_DISABLE) ? \
				    PA2FRM_ETH_PS_FLAGS_DISABLE_CRC : 0;
			ps_flags |= ((route_info->pkt_type_emac_ctrl & \
				      PA2_EMAC_CTRL_PORT_MASK) << \
				     PA2FRM_ETH_PS_FLAGS_PORT_SHIFT);
		}
	}
	if (route_info->dest == PA2_DEST_HOST) {
		fwd_info->forward_type   = PA2FRM_FORWARD_TYPE_HOST;
		fwd_info->u.host.context = route_info->sw_info_0;
		fwd_info->u.host.ps_flags = ps_flags;

		if (route_info->valid_bitmap & \
		    PA2_ROUTE_INFO_VALID_PRIORITY_TYPE) {
			if (route_info->priority_type == \
				PA2_ROUTE_PRIORITY_VLAN)
				fwd_info->u.host.ctrl_bitmap |=
					PA2FRM_ROUTING_PRIORITY_VLAN_ENABLE;
			else if (route_info->priority_type == \
				PA2_ROUTE_PRIORITY_DSCP)
				fwd_info->u.host.ctrl_bitmap |=
					PA2FRM_ROUTING_PRIORITY_DSCP_ENABLE;
			else if (route_info->priority_type == PA2_ROUTE_INTF)
				fwd_info->u.host.ctrl_bitmap |=
					PA2FRM_ROUTING_IF_DEST_SELECT_ENABLE;
			else if (route_info->priority_type == \
				 PA2_ROUTE_INTF_W_FLOW)
				fwd_info->u.host.ctrl_bitmap |=
					(PA2FRM_ROUTING_IF_DEST_SELECT_ENABLE |\
					 PA2FRM_ROUTING_FLOW_IF_BASE_ENABLE);
			else
				return PA2_ERR_CONFIG;
		}

		if (route_info->valid_bitmap & \
		    PA2_ROUTE_INFO_VALID_MROUTEINDEX) {
			if (route_info->m_route_index >= 0) {
				if (route_info->m_route_index >= \
					PA2_MAX_MULTI_ROUTE_SETS)
					return PA2_ERR_CONFIG;
				fwd_info->u.host.ctrl_bitmap |=
					PA2FRM_MULTIROUTE_ENABLE;
				fwd_info->u.host.multi_idx =
					route_info->m_route_index;
				fwd_info->u.host.pa_pdsp_router	=
					PA2FRM_DEST_PA_M_0;
			}
		}
	} else if (route_info->dest == PA2_DEST_DISCARD)	{
		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_DISCARD;
	} else if (route_info->dest == PA2_DEST_EMAC) {
		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_ETH;
		fwd_info->u.eth.ps_flags = ps_flags;
	} else if (fail_route) {
		return PA2_ERR_CONFIG;

	} else if (((route_info->dest == PA2_DEST_CONTINUE_PARSE_LUT1) &&
		    (route_info->custom_type != PA2_CUSTOM_TYPE_LUT2)) ||
		   ((route_info->dest == PA2_DEST_CONTINUE_PARSE_LUT2) &&
		    (route_info->custom_type != PA2_CUSTOM_TYPE_LUT1))) {

		/* Custom Error check */
		if (((route_info->custom_type == PA2_CUSTOM_TYPE_LUT1) &&
		     (route_info->custom_index >= PA2_MAX_CUSTOM_TYPES_LUT1)) ||
		    ((route_info->custom_type == PA2_CUSTOM_TYPE_LUT2) &&
		     (route_info->custom_index >= PA2_MAX_CUSTOM_TYPES_LUT2)))
			return PA2_ERR_CONFIG;

		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_PA;
		fwd_info->u.pa.custom_type = (u8)route_info->custom_type;
		fwd_info->u.pa.custom_idx  = route_info->custom_index;
		fwd_info->u.pa.flags  = pa_flags;

		if (route_info->dest == PA2_DEST_CONTINUE_PARSE_LUT2) {
			fwd_info->u.pa.pa_dest = PA2FRM_DEST_INGRESS4;
		} else {
			/*
			 * cmd_dest is provided by calling function
			 * There is no need to check error case
			 */
			if (cmd_dest == PA2_CMD_TX_DEST_0)
				/* Layer 2 entry */
				fwd_info->u.pa.pa_dest = PA2FRM_DEST_INGRESS1;
			else if (cmd_dest == PA2_CMD_TX_DEST_1) {
				fwd_info->u.pa.pa_dest = (dest_pdsp == 0) ? \
				  PA2FRM_DEST_INGRESS1 : PA2FRM_DEST_INGRESS3;
				if (route_info->custom_type == \
						PA2_CUSTOM_TYPE_LUT1)
					fwd_info->u.pa.pa_dest =
						PA2FRM_DEST_INGRESS3;
			} else if (cmd_dest == PA2_CMD_TX_DEST_3)
				fwd_info->u.pa.pa_dest = PA2FRM_DEST_INGRESS4;
			else
				return PA2_ERR_CONFIG;
		}
		no_fcmd = 1;
	} else if (route_info->dest == PA2_DEST_CASCADED_FORWARDING_LUT1) {
		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_PA;
		fwd_info->u.pa.pa_dest = (cmd_dest == PA2_CMD_TX_DEST_0) ? \
				PA2FRM_DEST_INGRESS1 : PA2FRM_DEST_INGRESS4;
		fwd_info->u.pa.flags   |= PA2FRM_CASCADED_FORWARDING;
		no_fcmd = 1;
	} else if (route_info->dest == PA2_DEST_SASS) {
		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_SA;
		fwd_info->u.sa.sw_info_0 = route_info->sw_info_0;
		fwd_info->u.sa.sw_info_1 = route_info->sw_info_1;
	} else if (route_info->dest == PA2_DEST_SASS_LOC_DMA) {
		fwd_info->forward_type	= PA2FRM_FORWARD_TYPE_SA | \
					  PA2FRM_FORWARD_CONTROL_USE_LOC_DMA;
		fwd_info->u.sa.sw_info_0 = route_info->sw_info_0;
		fwd_info->u.sa.sw_info_1 = route_info->sw_info_1;
	} else if ((route_info->dest == PA2_DEST_RES_1) || \
		   (route_info->dest == PA2_DEST_RES_2)) {
		fwd_info->forward_type	= PA2FRM_FORWARD_TYPE_SA_DIRECT;
		fwd_info->flow_id = (route_info->dest == PA2_DEST_RES_1) ? \
					  PA2FRM_DEST_ACE0 : PA2FRM_DEST_ACE1;
		fwd_info->u.sa.sw_info_0 = route_info->sw_info_0;
		fwd_info->u.sa.sw_info_1 = route_info->sw_info_1;
	} else if (route_info->dest == PA2_DEST_SRIO) {
		fwd_info->forward_type = PA2FRM_FORWARD_TYPE_SRIO;
		fwd_info->u.srio.ps_info0 = route_info->sw_info_0;
		fwd_info->u.srio.ps_info1 = route_info->sw_info_1;
		fwd_info->u.srio.pkt_type = route_info->pkt_type_emac_ctrl;
		pcmd = NULL;
	} else if (route_info->dest == PA2_DEST_EFLOW) {
		return pa2_conv_fc_routing_info(fwd_info,
						route_info->ef_op);
	} else {
		return PA2_ERR_CONFIG;
	}

	if (pcmd && (route_info->valid_bitmap & PA2_ROUTE_INFO_VALID_PCMD)) {
		struct pa2_cmd_info *pacmd = route_info->pcmd;
		struct pa2_patch_info *patch_info;
		struct pa2_cmd_set *cmd_set;
		struct pa2_cmd_usr_stats *usr_stats;
		struct pa2_cmd_set_usr_stats *cmd_set_usr_stats;

		switch (pacmd->cmd) {
		case PA2_CMD_PATCH_DATA:
			patch_info = &pacmd->params.patch;
			if ((patch_info->n_patch_bytes > 2) || \
				(!(patch_info->ctrl_bit_field & \
				PA2_PATCH_OP_INSERT)) || \
				(patch_info->patch_data == NULL))
				return PA2_ERR_CONFIG;

			pcmd[0] = PA2FRM_RX_CMD_PATCH_DATA;
			pcmd[1] = patch_info->n_patch_bytes;
			pcmd[2] = patch_info->patch_data[0];
			pcmd[3] = patch_info->patch_data[1];
			break;

		case PA2_CMD_CMDSET:
			cmd_set = &pacmd->params.cmd_set;
			if (no_fcmd || (cmd_set->index >= PA2_MAX_CMD_SETS))
				return PA2_ERR_CONFIG;

			pcmd[0] = PA2FRM_RX_CMD_CMDSET;
			pcmd[1] = (u8)cmd_set->index;
			break;

		case PA2_CMD_USR_STATS:
			usr_stats = &pacmd->params.usr_stats;
			if (usr_stats->index >= 512)
				return PA2_ERR_CONFIG;

			pcmd[0] = PA2FRM_RX_CMD_USR_STATS;
			pcmd[1] = 4;
			pcmd[2] = usr_stats->index >> 8;
			pcmd[3] = usr_stats->index & 0xFF;
			break;

		case PA2_CMD_CMDSET_AND_USR_STATS:
			cmd_set_usr_stats = &pacmd->params.cmd_set_usr_stats;
			if ((no_fcmd) ||
			   (cmd_set_usr_stats->set_index >= PA2_MAX_CMD_SETS) ||
			   (cmd_set_usr_stats->stats_index >= 512))
				return PA2_ERR_CONFIG;

			pcmd[0] = PA2FRM_RX_CMD_CMDSET_USR_STATS;
			pcmd[1] = (u8)cmd_set_usr_stats->set_index;
			pcmd[2] = cmd_set_usr_stats->stats_index >> 8;
			pcmd[3] = cmd_set_usr_stats->stats_index & 0xFF;
			break;

		default:
			return PA2_ERR_CONFIG;
		}
	}
	return PA2_OK;
}

static void pa2_get_version(struct pa2_device *pa_dev)
{
	u32 version, i;

	for (i = 0; i < PA2_NUM_PDSPS; i++) {
		u32 cluster = pa2_cluster_pdsp_map[i].cluster;
		void __iomem *sram = pa_dev->cluster[cluster].sram;
		u32 base = pa2_cluster_pdsp_map[i].ver_base_addr;
		u32 pdsp = pa2_cluster_pdsp_map[i].pdsp;
		version = __raw_readl(sram +
				PA2_PDSP_VERSION_OFFSET(base, pdsp));

		dev_info(pa_dev->dev,
			 "Packet Accelerator PDSP %d Firmware Version 0x%08x\n",
			 i, version);
	}
}

static int pa2_pdsp_run(struct pa2_device *pa_dev, int pdsp)
{
	struct pa2_ppu_ctl_status_regs __iomem *ctrl_reg =
						pa_dev->ppu[pdsp].ctl_status;
	struct pa2_mailbox_regs __iomem *mailbox_reg =
						&pa_dev->reg_mailbox[pdsp];
	u32 i, v;

	/* Check for enabled PDSP */
	v = __raw_readl(&ctrl_reg->control);
	if ((v & PA2_REG_VAL_PDSP_CTL_ENABLE) ==
	    PA2_REG_VAL_PDSP_CTL_ENABLE) {
		/* Already enabled */
		return PA2_PDSP_ALREADY_ACTIVE;
	}

	/* Clear the mailbox */
	__raw_writel(0, &mailbox_reg->pdsp_mailbox_slot0);

	/* Set PDSP PC to 0, enable the PDSP */
	__raw_writel(PA2_REG_VAL_PDSP_CTL_ENABLE |
		     PA2_REG_VAL_PDSP_CTL_SOFT_RESET,
		     &ctrl_reg->control);

	/* Wait for the mailbox to become non-zero */
	for (i = 0; i < PA2_MAX_PDSP_ENABLE_LOOP_COUNT; i++) {
		v = __raw_readl(&mailbox_reg->pdsp_mailbox_slot0);
		if (v != 0)
			return PA2_PDSP_RESET_RELEASED;
	}

	return PA2_PDSP_NO_RESTART;
}

static int keystone_pa2_reset_control(struct pa2_device *pa_dev, int new_state)
{
	int i, res;
	int ret;

	if (new_state == PA2_STATE_RESET) {
		/* Put each of the PDSPs into reset (PC = 0) and reset timers */
		for (i = 0; i < PA2_NUM_PDSPS; i++)  {
			__raw_writel(0, &pa_dev->ppu[i].ctl_status->control);
			__raw_writel(0,
				     &pa_dev->ppu[i].cp_timer->timer_control);
		}

		/* Reset LUT2 */
		__raw_writel(PA2_REG_VAL_PDSP_LUT2_CLR_TABLE_GO,
			     &pa_dev->ppu[PA2_INGRESS4_PDSP1].lut2->clr_table);
		ret = PA2_STATE_RESET;
	} else if (new_state == PA2_STATE_ENABLE) {
		ret = PA2_STATE_ENABLE;

		/*
		 * Do nothing if a pdsp is already out of reset.
		 * If any PDSPs are out of reset
		 * a global init is not performed
		 */
		for (i = 0; i < PA2_NUM_PDSPS; i++) {
			res = pa2_pdsp_run(pa_dev, i);

			if (res == PA2_PDSP_NO_RESTART)
				ret = PA2_STATE_ENABLE_FAILED;
		}

		for (i = 0; i < PA2_NUM_PDSPS; i++) {
			struct pa2_mailbox_regs __iomem *mbox_reg =
					&pa_dev->reg_mailbox[i];
			__raw_writel(0, &mbox_reg->pdsp_mailbox_slot0);
		}
	} else
		ret = PA2_STATE_INVALID_REQUEST;

	return ret;
}

static int keystone_pa2_set_firmware(struct pa2_device *pa_dev,
			     int pdsp, const unsigned int *buffer, int len)
{
	struct pa2_ppu_debug_regs __iomem *debug_reg = pa_dev->ppu[pdsp].debug;
	u32 i;

	if ((pdsp < 0) || (pdsp >= PA2_NUM_PDSPS))
		return -EINVAL;

	if (len > PA2_PPU_IRAM_SIZE)
		return -ENODEV;

	pdsp_fw_put((u32 *)(pa_dev->ppu[pdsp].iram), buffer,
		    len >> 2);

	for (i = 0; i < PA2_PDSP_CONST_NUM_REG; i++)
		__raw_writel(pa2_pdsp_const_reg_map[pdsp][i],
		     &debug_reg->icte[i]);
	return 0;
}

static struct pa2_packet *pa2_alloc_packet(struct pa2_device *pa_dev,
					 unsigned cmd_size,
					 struct dma_chan *dma_chan)
{
	struct pa2_packet *p_info;

	p_info = kzalloc(sizeof(*p_info) + cmd_size, GFP_ATOMIC);
	if (!p_info)
		return NULL;

	p_info->priv = pa_dev;
	p_info->data = p_info + 1;
	p_info->chan = dma_chan;

	sg_init_table(p_info->sg, PA2_SGLIST_SIZE);
	sg_set_buf(&p_info->sg[0], p_info->epib, sizeof(p_info->epib));
	sg_set_buf(&p_info->sg[1], p_info->psdata, sizeof(p_info->psdata));
	sg_set_buf(&p_info->sg[2], p_info->data, cmd_size);

	return p_info;
}

static void pa2_tx_dma_callback(void *data)
{
	struct pa2_packet *p_info = data;
	struct pa2_device *pa_dev = p_info->priv;
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
		WARN((status != DMA_SUCCESS),
				"dma completion failure, status == %d", status);
	}

	dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_TO_DEVICE);

	p_info->desc = NULL;

	kfree(p_info);
}

static int pa2_submit_tx_packet(struct pa2_packet *p_info)
{
	unsigned flags = DMA_HAS_EPIB | DMA_HAS_PSINFO;
	struct pa2_device *pa_dev = p_info->priv;
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

	p_info->desc->callback = pa2_tx_dma_callback;
	p_info->desc->callback_param = p_info;

	spin_lock_irqsave(&pa_dev->lock, irqsave);
	p_info->cookie = dmaengine_submit(p_info->desc);
	spin_unlock_irqrestore(&pa_dev->lock, irqsave);

	return dma_submit_error(p_info->cookie) ? p_info->cookie : 0;
}

#define	PA2_CONTEXT_MASK	0xffff0000
#define	PA2_CONTEXT_CONFIG	0xdead0000

static void pa2_rx_complete(void *param)
{
	struct pa2_packet *p_info = param;
	struct pa2_device *pa_dev = p_info->priv;
	struct pa2_frm_command *fcmd;

	dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);

	switch (p_info->epib[1] & PA2_CONTEXT_MASK) {
	case PA2_CONTEXT_CONFIG:
		fcmd = p_info->data;
		swizFcmd(fcmd);

		if (fcmd->command_result != PA2FRM_COMMAND_RESULT_SUCCESS) {
			dev_dbg(pa_dev->dev, "Command Result = 0x%x\n",
				fcmd->command_result);
			dev_dbg(pa_dev->dev, "Command = 0x%x\n", fcmd->command);
			dev_dbg(pa_dev->dev, "Magic = 0x%x\n", fcmd->magic);
			dev_dbg(pa_dev->dev, "Com ID = 0x%x\n", fcmd->com_id);
			dev_dbg(pa_dev->dev, "ret Context = 0x%x\n",
				fcmd->ret_context);
			dev_dbg(pa_dev->dev, "Flow ID = 0x%x\n", fcmd->flow_id);
			dev_dbg(pa_dev->dev, "reply Queue = 0x%x\n",
				fcmd->reply_queue);
			dev_dbg(pa_dev->dev, "reply dest = 0x%x\n",
				fcmd->reply_dest);
		}
		dev_dbg(pa_dev->dev, "command response complete\n");
		break;

	default:
		dev_warn(pa_dev->dev, "bad response context, got 0x%08x\n",
			 p_info->epib[1]);
		break;
	}

	p_info->desc = NULL;
	kfree(p_info);
}

/* Release a free receive buffer */
static void pa2_rxpool_free(void *arg, unsigned q_num, unsigned bufsize,
		struct dma_async_tx_descriptor *desc)
{
	struct pa2_device *pa_dev = arg;
	struct pa2_packet *p_info = desc->callback_param;

	dma_unmap_sg(pa_dev->dev, &p_info->sg[2], 1, DMA_FROM_DEVICE);

	p_info->desc = NULL;

	kfree(p_info);
}

static void pa2_chan_work_handler(unsigned long data)
{
	struct pa2_device *pa_dev = (struct pa2_device *)data;

	dma_poll(pa_dev->rx_channel, -1);

	dma_rxfree_refill(pa_dev->rx_channel);

	dmaengine_resume(pa_dev->rx_channel);
}

static void pa2_chan_notify(struct dma_chan *dma_chan, void *arg)
{
	struct pa2_device *pa_dev = arg;

	dmaengine_pause(pa_dev->rx_channel);

	tasklet_schedule(&pa_dev->task);

	return;
}

/* Allocate a free receive buffer */
static struct dma_async_tx_descriptor *pa2_rxpool_alloc(void *arg,
		unsigned q_num, unsigned bufsize)
{
	struct pa2_device *pa_dev = arg;
	struct dma_async_tx_descriptor *desc;
	struct dma_device *device;
	u32 err = 0;

	struct pa2_packet *rx;

	rx = pa2_alloc_packet(pa_dev, bufsize, pa_dev->rx_channel);
	if (!rx) {
		dev_err(pa_dev->dev, "could not allocate cmd rx packet\n");
		kfree(rx);
		return NULL;
	}

	rx->sg_ents = 2 + dma_map_sg(pa_dev->dev, &rx->sg[2],
				1, DMA_FROM_DEVICE);
	if (rx->sg_ents != 3) {
		dev_err(pa_dev->dev, "dma map failed\n");

		kfree(rx);
		return NULL;
	}

	device = rx->chan->device;

	desc = dmaengine_prep_slave_sg(rx->chan, rx->sg, 3, DMA_DEV_TO_MEM,
				       DMA_HAS_EPIB | DMA_HAS_PSINFO);

	if (IS_ERR_OR_NULL(desc)) {
		dma_unmap_sg(pa_dev->dev, &rx->sg[2], 1, DMA_FROM_DEVICE);
		kfree(rx);
		err = PTR_ERR(desc);
		if (err != -ENOMEM) {
			dev_err(pa_dev->dev,
				"dma prep failed, error %d\n", err);
		}

		return NULL;
	}

	desc->callback_param = rx;
	desc->callback = pa2_rx_complete;
	rx->cookie = desc->cookie;

	return desc;
}

static struct pa2_frm_command *pa2_format_fcmd_hdr(void *p_cmd,
						 struct pa2_device *priv,
						 u8 cmd,
						 u16 com_id,
						 u8 first_pdsp,
						 u16 cmd_Size)
{
	struct pa2_frm_command *fcmd;

	memset(p_cmd, 0, cmd_Size);
	fcmd			= (struct pa2_frm_command *)p_cmd;
	fcmd->status		= PA2FRM_CFG_CMD_STATUS_PROC;
	fcmd->pdsp_index	= first_pdsp;
	fcmd->command		= cmd;
	fcmd->magic		= PA2FRM_CONFIG_COMMAND_SEC_BYTE;
	fcmd->com_id		= com_id;
	fcmd->ret_context	= PA2_CONTEXT_CONFIG;
	fcmd->flow_id		= priv->cmd_flow_num;
	fcmd->reply_queue	= priv->cmd_queue_num;
	fcmd->reply_dest	= PA2FRM_DEST_PKTDMA;

	return fcmd;
}

static int keystone_pa2_add_mac(struct pa2_intf *pa_intf, int index,
			       const u8 *smac, const u8 *dmac, int rule,
			       unsigned etype, int port)
{
	struct pa2_route_info2 route_info, fail_info;
	struct pa2_frm_command *fcmd;
	struct pa2_frm_cmd_add_lut1 *al1;
	struct pa2_packet *tx;
	struct pa2_device *priv = pa_intf->pa_device;
	int size, ret;
	u16 priority, bit_mask = 0;
	u32 cbwords0, cbwords1;

	dev_dbg(priv->dev, "add mac, index %d, smac %pM, dmac %pM, rule %d, "
		"type %04x, port %d\n", index, smac, dmac, rule, etype, port);

	memset(&fail_info, 0, sizeof(fail_info));

	memset(&route_info, 0, sizeof(route_info));

	if (rule == PACKET_HST) {
		route_info.dest			= PA2_DEST_HOST;
		route_info.flow_id		= pa_intf->data_flow_num;
		route_info.queue		= pa_intf->data_queue_num;
		route_info.m_route_index	= PA2_NO_MULTI_ROUTE;
		fail_info.dest			= PA2_DEST_HOST;
		fail_info.flow_id		= pa_intf->data_flow_num;
		fail_info.queue			= pa_intf->data_queue_num;
		fail_info.m_route_index		= PA2_NO_MULTI_ROUTE;
	} else if (rule == PACKET_PARSE) {
		route_info.dest			= PA2_DEST_CONTINUE_PARSE_LUT1;
		route_info.m_route_index	= PA2_NO_MULTI_ROUTE;
		fail_info.dest			= PA2_DEST_HOST;
		fail_info.flow_id		= pa_intf->data_flow_num;
		fail_info.queue			= pa_intf->data_queue_num;
		fail_info.m_route_index		= PA2_NO_MULTI_ROUTE;
	} else if (rule == PACKET_DROP) {
		route_info.dest			= PA2_DEST_DISCARD;
		route_info.m_route_index	= PA2_NO_MULTI_ROUTE;
		fail_info.dest			= PA2_DEST_DISCARD;
		fail_info.m_route_index		= PA2_NO_MULTI_ROUTE;
	}

	if (route_info.m_route_index != PA2_NO_MULTI_ROUTE)
		route_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_MROUTEINDEX;
	if (route_info.pkt_type_emac_ctrl)
		route_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_PKTTYPE_EMAC;
	if (route_info.pcmd)
		route_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_PCMD;
	if (fail_info.m_route_index != PA2_NO_MULTI_ROUTE)
		fail_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_MROUTEINDEX;
	if (fail_info.pkt_type_emac_ctrl)
		fail_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_PKTTYPE_EMAC;
	if (fail_info.pcmd)
		fail_info.valid_bitmap |= PA2_ROUTE_INFO_VALID_PCMD;

	size = (sizeof(struct pa2_frm_command) +
		sizeof(struct pa2_frm_cmd_add_lut1) - sizeof(u32));
	tx = pa2_alloc_packet(priv, size, priv->pdsp0_tx_channel);
	if (!tx) {
		dev_err(priv->dev, "could not allocate cmd tx packet\n");
		return -ENOMEM;
	}

	fcmd = pa2_format_fcmd_hdr((void *)tx->data,
				   priv,
				   PA2FRM_CONFIG_COMMAND_ADDREP_LUT1,
				   PA2_COMID_L2,
				   0,
				   size);

	al1		= (struct pa2_frm_cmd_add_lut1 *) &(fcmd->cmd);
	al1->index	= index;
	al1->type	= PA2FRM_COM_ADD_LUT1_STANDARD;

	cbwords0	= PA2FRM_LUT1_CLASS_STANDARD << PA2FRM_LUT1_CLASS_SHIFT;
	cbwords1	= PA2FRM_LUT1_VALID_PKTTYPE;
	priority	= 0;

	al1->u.mac.pkt_type = PA2FRM_L2_PKT_TYPE_MAC;

	if (etype) {
		al1->u.mac.etype = etype;
		cbwords0 |=  PA2FRM_LUT1_VALID_ETHERTYPE;
		priority += 10;
	}

	al1->u.mac.vlan_id1 = 0;
	al1->u.mac.mpls	= 0;
	if (port) {
		al1->u.mac.in_port = port;
		cbwords1 |=  PA2FRM_LUT1_VALID_INPORT;
		priority += 10;
	}

	if (dmac) {
		memcpy(al1->u.mac.dmac, dmac, 6);
		cbwords0 |= PA2FRM_LUT1_VALID_DMAC_ALL;
		priority += 10;
	}
	if (smac) {
		memcpy(al1->u.mac.smac, smac, 6);
		cbwords0 |= PA2FRM_LUT1_VALID_SMAC;
		priority += 10;
	}

	al1->cbwords0 = cbwords0;
	al1->cbwords1 = cbwords1;
	al1->priority = priority;
	al1->bit_mask = bit_mask;

	ret = pa2_conv_routing_info(&al1->match, &route_info, 0, 0, 0, 0);
	if (ret != 0)
		dev_err(priv->dev, "route info config failed\n");

	ret = pa2_conv_routing_info(&al1->next_fail, &fail_info, 0, 1, 0, 0);
	if (ret != 0)
		dev_err(priv->dev, "fail info config failed\n");

	swizFcmd(fcmd);
	swizAl1((struct pa2_frm_cmd_add_lut1 *)&(fcmd->cmd));

	tx->psdata[0] = PASAHO2_PACFG_CMD;

	tx->epib[1] = 0;
	tx->epib[2] = 0;
	tx->epib[3] = 0;

	pa2_submit_tx_packet(tx);
	dev_dbg(priv->dev, "waiting for command transmit complete\n");

	return 0;
}

static void pa2_init_crc_table4(u32 polynomial, u32 *crc_table4)
{
	int i, bit;

	/* 16 values representing all possible 4-bit values */
	for (i = 0; i < PARAM_CRC_TABLE_SIZE; i++) {
		crc_table4[i] = i << 28;
		for (bit = 0; bit < 4; bit++) {
			/* If shifting out a zero, then just shift */
			if (!(crc_table4[i] & 0x80000000))
				crc_table4[i] = (crc_table4[i] << 1);
			/* Else add in the polynomial as well */
			else
				crc_table4[i] =
					(crc_table4[i] << 1) ^ polynomial;
		}
	}
}

static int pa2_config_crc_engine(struct pa2_device *priv,
				enum pa2_crc_inst inst,
				struct pa2_crc_config *cfg_info,
				u32 recipe_idx)
{
	struct pa2_ppu_pcheck_regs __iomem *pcheck_regs;
	u32 i, pdsp, control, crc_tbl[16];

	switch (inst) {
	case PA2_CRC_INST_0_0:
		pdsp = PA2_INGRESS0_PDSP1;
		break;

	case PA2_CRC_INST_1_0:
		pdsp = PA2_INGRESS1_PDSP1;
		break;

	case PA2_CRC_INST_4_0:
		pdsp = PA2_INGRESS4_PDSP1;
		break;

	case PA2_CRC_INST_5_0:
		pdsp = PA2_POST_PDSP1;
		break;

	case PA2_CRC_INST_6_0:
		pdsp = PA2_EGRESS0_PDSP1;
		break;

	case PA2_CRC_INST_6_1:
		pdsp = PA2_EGRESS0_PDSP2;
		break;

	default:
		return PA2_ERR_CONFIG;
	}

	pcheck_regs = priv->ppu[pdsp].pcheck;
	control = (cfg_info->ctrl_bits & PA2_CRC_CONFIG_RIGHT_SHIFT) \
		  ? PA2_PCHECK_CONTROL_RSHIFT_MASK : 0;
	__raw_writel(control, &pcheck_regs->recipe[recipe_idx].control);
	control = __raw_readl(&pcheck_regs->recipe[recipe_idx].control);
	control |= (cfg_info->ctrl_bits & PA2_CRC_CONFIG_INVERSE_RESULT) ? \
		   PA2_PCHECK_CONTROL_FINAL_NOT_MASK : 0;

	pa2_init_crc_table4(cfg_info->polynomial, crc_tbl);
	for (i = 0; i < (PARAM_CRC_TABLE_SIZE - 1); i++)
		__raw_writel(crc_tbl[i+1],
			     &pcheck_regs->recipe[recipe_idx].table[i]);

	return 0;
}

#define	CRC32C_POLYNOMIAL	0x1EDC6F41
#define	SCTP_CRC_INITVAL	0xFFFFFFFF
/* SCTP TX CRC configuration data */
static struct pa2_crc_config sctp_tx_crc_cfg = {
	PA2_CRC_CONFIG_INVERSE_RESULT | PA2_CRC_CONFIG_RIGHT_SHIFT,
	PA2_CRC_SIZE_32,
	CRC32C_POLYNOMIAL,
	SCTP_CRC_INITVAL
};

static int pa2_config_sctp_crc_engine(struct pa2_device *priv)
{
	int ret;

	/* Configure CRC engine for SCTP TX, there are 4 recipes for each CRC
	 * engine, currently use recipe 0 */
	ret = pa2_config_crc_engine(priv, PA2_CRC_INST_6_1, &sctp_tx_crc_cfg,
				0);

	return ret;
}


static inline int pa2_fmtcmd_tx_csum(struct netcp_packet *p_info)
{
	struct sk_buff *skb = p_info->skb;
	struct pasaho2_com_chk_crc *ptx;
	int start, len;
	int size;

	size = sizeof(*ptx);
	ptx = (struct pasaho2_com_chk_crc *)netcp_push_psdata(p_info, size);

	start = skb_checksum_start_offset(skb);
	len = skb->len - start;

	ptx->word0 = 0;
	ptx->word1 = 0;
	ptx->word2 = 0;
	PASAHO2_SET_CMDID(ptx, PASAHO2_PAMOD_CMPT_CHKSUM);
	PASAHO2_CHKCRC_SET_START(ptx, start);
	PASAHO2_CHKCRC_SET_LEN(ptx, len);
	PASAHO2_CHKCRC_SET_RESULT_OFF(ptx, skb->csum_offset);
	PASAHO2_CHKCRC_SET_INITVAL(ptx, 0);
	PASAHO2_CHKCRC_SET_NEG0(ptx, 0);

	return size;
}

static inline int pa2_fmtcmd_tx_crc32c(struct netcp_packet *p_info)
{
	struct sk_buff *skb = p_info->skb;
	struct pasaho2_com_chk_crc *ptx;
	int start, len;
	int size;

	size = sizeof(*ptx);
	ptx = (struct pasaho2_com_chk_crc *)netcp_push_psdata(p_info, size);

	start = skb_checksum_start_offset(skb);
	len = skb->len - start;

	ptx->word0 = 0;
	ptx->word1 = 0;
	ptx->word2 = 0;
	PASAHO2_SET_CMDID(ptx, PASAHO2_PAMOD_CMPT_CRC);
	PASAHO2_CHKCRC_SET_CTRL(ptx, PA2FRM_CRC_FLAG_CRC_OFFSET_VALID);
	PASAHO2_CHKCRC_SET_CRCSIZE(ptx, 2);
	PASAHO2_CHKCRC_SET_START(ptx, start);
	PASAHO2_CHKCRC_SET_LEN(ptx, len);
	PASAHO2_CHKCRC_SET_RESULT_OFF(ptx, skb->csum_offset);
	PASAHO2_CHKCRC_SET_INITVAL32(ptx, 0);

	return size;
}

static inline int pa2_fmtcmd_next_route(struct netcp_packet *p_info,
					u8 ps_flags)
{
	struct pasaho2_next_route *nr;

	nr = (struct pasaho2_next_route *)netcp_push_psdata(p_info,
							sizeof(*nr));
	if (!nr)
		return -ENOMEM;

	/* Construct word0 */
	nr->word0 = 0;
	PASAHO2_SET_CMDID(nr, PASAHO2_PAMOD_NROUTE);
	PASAHO2_SET_E(nr, 1);
	PASAHO2_SET_DEST(nr, PA2FRM_DEST_ETH);
	PASAHO2_SET_FLOW(nr, 0);
	PASAHO2_SET_QUEUE(nr, 0);

	/* Construct sw_info0 and sw_info1 */
	nr->sw_info0 = 0;
	nr->sw_info1 = 0;

	/* Construct word1 */
	nr->word1 = 0;
	PASAHO2_SET_PKTTYPE(nr, ps_flags);

	return sizeof(*nr);
}

static inline int pa2_fmtcmd_align(struct netcp_packet *p_info,
		const unsigned bytes)
{
	struct pasaho2_cmd_info	*paCmdInfo;
	int i;

	if ((bytes & 0x03) != 0)
		return -EINVAL;

	paCmdInfo = (struct pasaho2_cmd_info *)netcp_push_psdata(p_info, bytes);

	for (i = bytes/sizeof(u32); i > 0; --i) {
		paCmdInfo->word0 = 0;
		PASAHO2_SET_CMDID(paCmdInfo, PASAHO2_PAMOD_DUMMY);
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

static int pa2_tx_hook(int order, void *data, struct netcp_packet *p_info)
{
	struct	pa2_intf *pa_intf = data;
	struct pa2_device *pa_dev = pa_intf->pa_device;
	struct netcp_priv *netcp_priv = netdev_priv(pa_intf->net_device);
	struct sk_buff *skb = p_info->skb;
	int size, total = 0;
	u8 ps_flags = 0;

	if (pa_dev->multi_if) {
		if (likely(skb->mark == 0) ||
		    likely((skb->mark & pa_dev->mark_mcast_match[1]) !=
				pa_dev->mark_mcast_match[0])) {
			/* normal port-specific output packet */
			ps_flags |= (netcp_priv->cpsw_port & \
				     PA2_EMAC_CTRL_PORT_MASK) << \
				     PA2FRM_ETH_PS_FLAGS_PORT_SHIFT;
		} else {
			/* Drop packet if port not in mask */
			if ((skb->mark & BIT(netcp_priv->cpsw_port - 1)) == 0)
				return NETCP_TX_DROP;
		}
	}

	/* Generate the next route command */
	size = pa2_fmtcmd_next_route(p_info, ps_flags);
	if (unlikely(size < 0))
		return size;
	total += size;

	/* If checksum offload required, request it */
	if ((skb->ip_summed == CHECKSUM_PARTIAL) &&
	    (pa_dev->csum_offload == CSUM_OFFLOAD_HARD)) {
		int l4_proto;

		l4_proto = extract_l4_proto(p_info);
		switch (l4_proto) {
		case IPPROTO_TCP:
		case IPPROTO_UDP:
			size = pa2_fmtcmd_tx_csum(p_info);
			break;
		case IPPROTO_SCTP:
			size = pa2_fmtcmd_tx_crc32c(p_info);
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
		size = pa2_fmtcmd_align(p_info, size);
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
		len -= copy;
		if (len == 0)
			return csum;
		offset += copy;
	}

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		int end;
		skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

		WARN_ON(start > offset + len);

		end = start + skb_frag_size(frag);
		copy = end - offset;
		if (copy > 0) {
			u8 *vaddr;
			skb_frag_t *frag = &skb_shinfo(skb)->frags[i];

			if (copy > len)
				copy = len;
			vaddr = kmap_atomic(skb_frag_page(frag));
			csum = sctp_update_cksum(vaddr + frag->page_offset +
					 offset - start, copy, csum);
			kunmap_atomic(vaddr);
			len -= copy;
			if (!len)
				return csum;
			offset += copy;
		}
		start = end;
	}

	skb_walk_frags(skb, frag_iter) {
		int end;

		WARN_ON(start > offset + len);

		end = start + frag_iter->len;
		copy = end - offset;
		if (copy > 0) {
			if (copy > len)
				copy = len;
			csum = skb_sctp_csum(frag_iter,
						offset - start, copy, csum);
			len -= copy;
			if (len == 0)
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

static int pa2_txhook_softcsum(int order, void *data,
			      struct netcp_packet *p_info)
{
	struct pa2_intf *pa_intf = data;
	struct pa2_device *pa_dev = pa_intf->pa_device;
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


static int pa2_close(void *intf_priv, struct net_device *ndev)
{
	struct pa2_intf *pa_intf = intf_priv;
	struct pa2_device *pa_dev = pa_intf->pa_device;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);

	netcp_unregister_txhook(netcp_priv, pa_dev->txhook_order,
				pa2_tx_hook, intf_priv);
	if (pa_dev->csum_offload == CSUM_OFFLOAD_SOFT)
		netcp_unregister_txhook(netcp_priv, pa_dev->txhook_softcsum,
					pa2_txhook_softcsum, intf_priv);

	netcp_txpipe_close(&pa_intf->tx_pipe);

	/* De-Configure the streaming switch */
	netcp_set_streaming_switch2(pa_dev->netcp_device,
				    netcp_priv->cpsw_port,
				    pa_intf->saved_ss_state);


	mutex_lock(&pa2_modules_lock);
	if (!--pa_dev->inuse_if_count) {
		/* Do pa disable related stuff only if this is the last
		 * interface to go down
		 */

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

		if (pa_dev->clk) {
			clk_disable_unprepare(pa_dev->clk);
			clk_put(pa_dev->clk);
		}
		pa_dev->clk = NULL;
	}

	pa_dev->opened = 0;
	mutex_unlock(&pa2_modules_lock);
	return 0;
}

static int pa2_open(void *intf_priv, struct net_device *ndev)
{
	struct pa2_intf *pa_intf = intf_priv;
	struct pa2_device *pa_dev = pa_intf->pa_device;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);
	struct dma_keystone_info config;
	const struct firmware *fw;
	struct dma_chan *chan;
	dma_cap_mask_t mask;
	int i, ret, err;

	/* The first time an open is being called */
	mutex_lock(&pa2_modules_lock);

	dev_dbg(pa_dev->dev, "pa2_open() called for port: %d\n",
		 netcp_priv->cpsw_port);

	if (++pa_dev->inuse_if_count == 1) {

		/* Do pa enable, load firmware only for the first interface
		 * that comes up
		 */
		dev_dbg(pa_dev->dev, "pa2_open() called for first time"
			" initializing per dev stuff\n");

		pa_dev->clk = clk_get(pa_dev->dev, "clk_pa");
		if (IS_ERR_OR_NULL(pa_dev->clk)) {
			dev_warn(pa_dev->dev,
				 "unable to get Packet Accelerator clock\n");
			pa_dev->clk = NULL;
		}

		if (pa_dev->clk)
			clk_prepare_enable(pa_dev->clk);

		/* System Statistics initialization */
		__raw_writel(PA2_STATS_CTL_ENABLE_ALLOC_MASK,
				&pa_dev->reg_stats_ctl->enable_alloc);
		__raw_writel(1, &pa_dev->reg_stats_ctl->soft_reset);

		/* Initialize all clusters */
		for (i = 0; i < PA2_NUM_CLUSTERS; i++) {
			__raw_writel(PA2_CLUSTER_SPLITTER_EOP_CTL,
				&pa_dev->cluster[i].splitter->eop_ctl);
			__raw_writel(PA2_CLUSTER_SPLITTER_EOP_BUF_SIZE(i),
				&pa_dev->cluster[i].splitter->mop_buf_size);
			__raw_writel(PA2_CLUSTER_SPLITTER_MOP_BUF_PTR,
				&pa_dev->cluster[i].splitter->mop_buf_ptr);
			__raw_writel(PA2_CLUSTER_SPLITTER_SOP_CTL,
				&pa_dev->cluster[i].splitter->sop_ctl);
		}

		keystone_pa2_reset_control(pa_dev, PA2_STATE_RESET);

		for (i = 0; i < PA2_NUM_PDSPS; i++) {
			if (!pa_dev->pdsp_fw[i])
				continue;

			ret = request_firmware(&fw, pa_dev->pdsp_fw[i],
						pa_dev->dev);
			if (ret != 0) {
				dev_err(pa_dev->dev,
					"can't find fw for pdsp %d", i);
				ret = -ENODEV;
				goto fail;
			}

			/* Download the firmware to the PDSP */
			ret = keystone_pa2_set_firmware(pa_dev, i,
						(const unsigned int *) fw->data,
						fw->size);
			if (ret != 0) {
				dev_err(pa_dev->dev,
					"failed to download fw for pdsp %d", i);
				ret = -ENODEV;
				goto fail;
			}


			release_firmware(fw);
		}

		ret = keystone_pa2_reset_control(pa_dev, PA2_STATE_ENABLE);
		if (ret != 1) {
			dev_err(pa_dev->dev, "enable failed, ret = %d\n", ret);
			ret = -ENODEV;
			goto fail;
		}

		pa2_get_version(pa_dev);

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

		memset(&config, 0, sizeof(config));
		config.direction	= DMA_MEM_TO_DEV;
		config.tx_queue_depth	= pa_dev->tx_cmd_queue_depth;

		err = dma_keystone_config(pa_dev->pdsp0_tx_channel, &config);
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
		config.scatterlist_size		= PA2_SGLIST_SIZE;
		config.rxpool_allocator		= pa2_rxpool_alloc;
		config.rxpool_destructor	= pa2_rxpool_free;
		config.rxpool_param		= pa_dev;
		config.rxpool_count		= 1;
		config.rxpool_thresh_enable	= DMA_THRESH_NONE;
		config.rxpools[0].pool_depth	= pa_dev->rx_pool_depth;
		config.rxpools[0].buffer_size	= pa_dev->rx_buffer_size;

		err = dma_keystone_config(pa_dev->rx_channel, &config);
		if (err)
			goto fail;

		tasklet_init(&pa_dev->task, pa2_chan_work_handler,
			     (unsigned long) pa_dev);

		dma_set_notify(pa_dev->rx_channel, pa2_chan_notify, pa_dev);

		pa_dev->cmd_flow_num = dma_get_rx_flow(pa_dev->rx_channel);
		pa_dev->cmd_queue_num = dma_get_rx_queue(pa_dev->rx_channel);

		dev_dbg(pa_dev->dev, "command receive flow %d, queue %d\n",
			pa_dev->cmd_flow_num, pa_dev->cmd_queue_num);

		pa_dev->addr_count = 0;

		dma_rxfree_refill(pa_dev->rx_channel);

		ret = pa2_config_sctp_crc_engine(pa_dev);
		if (ret < 0)
			goto fail;

		/* make lut entries invalid */
		/* for (i = 0; i < pa_dev->lut_size; i++) {
			if (!pa_dev->lut[i].valid)
				continue;
			keystone_pa2_add_mac(pa_intf, i, NULL, NULL,
					PACKET_DROP, 0, PA2_INVALID_PORT);
		} */
	}
	mutex_unlock(&pa2_modules_lock);

	pa_intf->saved_ss_state = netcp_get_streaming_switch2(
						     pa_dev->netcp_device,
						     netcp_priv->cpsw_port);
	dev_dbg(pa_dev->dev, "saved_ss_state for port %d is %d\n",
		 netcp_priv->cpsw_port, pa_intf->saved_ss_state);

	chan = netcp_get_rx_chan(netcp_priv);
	pa_intf->data_flow_num = dma_get_rx_flow(chan);
	pa_intf->data_queue_num = dma_get_rx_queue(chan);

	dev_dbg(pa_dev->dev, "configuring data receive flow %d, queue %d\n",
		 pa_intf->data_flow_num, pa_intf->data_queue_num);

	/* Configure the streaming switch */
	netcp_set_streaming_switch2(pa_dev->netcp_device, netcp_priv->cpsw_port,
				    PSTREAM_ROUTE_INGRESS0);

	/* Open the PA Data transmit channel */
	ret = netcp_txpipe_open(&pa_intf->tx_pipe);
	if (ret)
		goto fail;

	netcp_register_txhook(netcp_priv, pa_dev->txhook_order,
			      pa2_tx_hook, intf_priv);
	if (pa_dev->csum_offload == CSUM_OFFLOAD_SOFT)
		netcp_register_txhook(netcp_priv, pa_dev->txhook_softcsum,
				      pa2_txhook_softcsum, intf_priv);

	pa_dev->opened = 1;
	return 0;

fail:
	mutex_unlock(&pa2_modules_lock);
	pa2_close(intf_priv, ndev);
	return ret;
}

static struct pa2_lut_entry *pa2_lut_alloc(struct pa2_device *pa_dev,
					 bool backwards)
{
	struct pa2_lut_entry *entry;
	int i;

	if (!backwards) {
		for (i = 0; i < pa_dev->lut_size; i++) {
			entry = pa_dev->lut + i;
			if (!entry->valid || entry->in_use)
				continue;
			entry->in_use = true;
			return entry;
		}
	} else {
		for (i = pa_dev->lut_size - 1; i >= 0; i--) {
			entry = pa_dev->lut + i;
			if (!entry->valid || entry->in_use)
				continue;
			entry->in_use = true;
			return entry;
		}
	}
	return NULL;
}

static inline int pa2_lut_entry_count(enum netcp_addr_type type)
{
	return (type == ADDR_DEV || type == ADDR_UCAST || type == ADDR_ANY) ? \
		3 : 1;
}

static int pa2_add_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct pa2_intf *pa_intf = intf_priv;
	struct pa2_device *pa_dev = pa_intf->pa_device;
	struct netcp_priv *netcp_priv = netdev_priv(pa_intf->net_device);
	int count = pa2_lut_entry_count(naddr->type);
	struct pa2_lut_entry *entries[count];
	int port = netcp_priv->cpsw_port;
	int idx, error;
	const u8 *addr;

	if (!pa_dev->opened)
		return -ENXIO;

	for (idx = 0; idx < count; idx++) {
		entries[idx] = pa2_lut_alloc(pa_dev, naddr->type == ADDR_ANY);
		if (!entries[idx])
			goto fail_alloc;
		entries[idx]->naddr = naddr;
	}

	addr = (naddr->type == ADDR_ANY) ? NULL : naddr->addr;
	idx = 0;

	if (naddr->type == ADDR_ANY) {
		error = keystone_pa2_add_mac(pa_intf, entries[idx++]->index,
					    NULL, addr, PACKET_HST, 0, port);
		if (error)
			return error;
	}

	if (count > 1) {
		error = keystone_pa2_add_mac(pa_intf, entries[idx++]->index,
					    NULL, addr, PACKET_PARSE,
					    0x0800, port);
		if (error)
			return error;

		error = keystone_pa2_add_mac(pa_intf, entries[idx++]->index,
					    NULL, addr, PACKET_PARSE,
					    0x86dd, port);
		if (error)
			return error;
	}

	if (naddr->type != ADDR_ANY) {
		error = keystone_pa2_add_mac(pa_intf, entries[idx++]->index,
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

static int pa2_del_addr(void *intf_priv, struct netcp_addr *naddr)
{
	struct pa2_intf *pa_intf = intf_priv;
	struct pa2_device *pa_dev = pa_intf->pa_device;
	struct pa2_lut_entry *entry;
	int idx;

	if (!pa_dev->opened)
		return -ENXIO;

	for (idx = 0; idx < pa_dev->lut_size; idx++) {
		entry = pa_dev->lut + idx;
		if (!entry->valid || !entry->in_use || entry->naddr != naddr)
			continue;
		keystone_pa2_add_mac(pa_intf, entry->index, NULL, NULL,
				    PACKET_DROP, 0, PA2_INVALID_PORT);
		entry->in_use = false;
		entry->naddr = NULL;
	}

	return 0;
}

static int pa2_ioctl(void *intf_priv, struct ifreq *req, int cmd)
{
	return -EOPNOTSUPP;
}

static int pa2_attach(void *inst_priv, struct net_device *ndev,
			void **intf_priv)
{
	struct pa2_device *pa_dev = inst_priv;
	struct netcp_priv *netcp_priv = netdev_priv(ndev);
	struct pa2_intf *pa_intf;
	int chan_id = 0;

	if (netcp_priv->cpsw_port)
		pa_dev->multi_if = 1;

	dev_dbg(pa_dev->dev, "pa2_attach, port %d\n", netcp_priv->cpsw_port);
	pa_intf = devm_kzalloc(pa_dev->dev, sizeof(struct pa2_intf),
				GFP_KERNEL);
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
		ndev->features		|= PA2_NETIF_FEATURES;
		ndev->hw_features	|= PA2_NETIF_FEATURES;
		ndev->wanted_features	|= PA2_NETIF_FEATURES;
		netdev_update_features(ndev);
		rtnl_unlock();
	}
	return 0;
}

static int pa2_release(void *intf_priv)
{
	struct pa2_intf *pa_intf = intf_priv;
	struct pa2_device *pa_dev = pa_intf->pa_device;
	struct net_device *ndev = pa_intf->net_device;

	mutex_lock(&pa2_modules_lock);
	if ((!--pa_dev->inuse_if_count) && (pa_dev->csum_offload)) {
		rtnl_lock();
		ndev->features		&= ~PA2_NETIF_FEATURES;
		ndev->hw_features	&= ~PA2_NETIF_FEATURES;
		ndev->wanted_features	&= ~PA2_NETIF_FEATURES;
		netdev_update_features(ndev);
		rtnl_unlock();
	}
	mutex_unlock(&pa2_modules_lock);

	netif_napi_del(&pa_intf->tx_pipe.dma_poll_napi);

	devm_kfree(pa_dev->dev, pa_intf);
	return 0;
}

static int pa2_remove(struct netcp_device *netcp_device, void *inst_priv)
{
	struct pa2_device *pa_dev = inst_priv;
	struct device *dev = pa_dev->dev;
	u32 i;

	pa2_cond_unmap(pa_dev->reg_mailbox);
	pa2_cond_unmap(pa_dev->reg_ra_bridge);
	pa2_cond_unmap(pa_dev->reg_thread_mapper);
	pa2_cond_unmap(pa_dev->reg_ra);
	pa2_cond_unmap(pa_dev->reg_stats_ctl);
	pa2_cond_unmap(pa_dev->reg_query_stats);
	pa2_cond_unmap(pa_dev->reg_collect_stats);
	pa2_cond_unmap(pa_dev->pa_sram);

	for (i = 0; i < PA2_NUM_CLUSTERS; i++) {
		pa2_cond_unmap(pa_dev->cluster[i].splitter);
		pa2_cond_unmap(pa_dev->cluster[i].sram);
	}

	for (i = 0; i < PA2_NUM_PDSPS; i++) {
		pa2_cond_unmap(pa_dev->ppu[i].ctl_status);
		pa2_cond_unmap(pa_dev->ppu[i].debug);
		pa2_cond_unmap(pa_dev->ppu[i].cp_timer);
		pa2_cond_unmap(pa_dev->ppu[i].lut1);
		pa2_cond_unmap(pa_dev->ppu[i].lut2);
		pa2_cond_unmap(pa_dev->ppu[i].pcheck);
		pa2_cond_unmap(pa_dev->ppu[i].iram);
	}

	kfree(pa_dev);

	return 0;
}

static int pa2_probe(struct netcp_device *netcp_device,
		    struct device *dev,
		    struct device_node *node,
		    void **inst_priv)
{
	struct pa2_device *pa_dev;
	int ret, len = 0, start, end, i, j;
	int table_size, num_ranges;
	u32 *prange;
	u32 regs_base;

	if (!node) {
		dev_err(dev, "device tree info unavailable\n");
		return -ENODEV;
	}

	pa_dev = devm_kzalloc(dev, sizeof(struct pa2_device), GFP_KERNEL);
	if (!pa_dev) {
		dev_err(dev, "memory allocation failed\n");
		return -ENOMEM;
	}
	*inst_priv = pa_dev;

	pa_dev->netcp_device = netcp_device;
	pa_dev->dev = dev;

	ret = of_property_read_u32(node, "reg_base",
				   &regs_base);
	if (ret < 0) {
		dev_err(dev, "missing reg_base parameter, err %d\n",
			ret);
		goto exit;
	}
	dev_dbg(dev, "reg_base 0x%x\n", regs_base);

	for (i = 0; i < PA2_NUM_PDSPS; ++i) {
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

	pa_dev->reg_mailbox =
		devm_ioremap(dev, regs_base + PA2_MAILBOX_REGS_OFFSET,
			sizeof(struct pa2_mailbox_regs) * PA2_MAX_NUM_MAILBOX);
	pa_dev->reg_ra_bridge =
		devm_ioremap(dev, regs_base + PA2_RA_BRIDGE_REGS_OFFSET,
			     sizeof(struct pa2_ra_bridge_regs));
	pa_dev->reg_thread_mapper =
		devm_ioremap(dev, regs_base + PA2_THREAD_MAPPER_REGS_OFFSET,
			     sizeof(struct pa2_thread_mapper_regs));
	pa_dev->reg_ra =
		devm_ioremap(dev, regs_base + PA2_RA_REGS_OFFSET,
		sizeof(struct pa2_ra_regs));
	pa_dev->reg_stats_ctl =
		devm_ioremap(dev, regs_base + PA2_STATS_CTL_REGS_OFFSET,
			     sizeof(struct pa2_stats_ctl_regs));
	pa_dev->reg_query_stats =
		devm_ioremap(dev, regs_base + PA2_QUERY_STATS_REGS_OFFSET,
			     sizeof(struct pa2_query_stats_regs));
	pa_dev->reg_collect_stats =
		devm_ioremap(dev, regs_base + PA2_COLLECT_STATS_REGS_OFFSET,
			     sizeof(struct pa2_collect_stats_regs));
	pa_dev->pa_sram =
		devm_ioremap(dev, regs_base + PA2_SRAM_OFFSET, PA2_SRAM_SIZE);
	if (!pa_dev->reg_mailbox || !pa_dev->reg_ra_bridge ||
	    !pa_dev->reg_thread_mapper || !pa_dev->reg_ra ||
	    !pa_dev->reg_stats_ctl || !pa_dev->reg_query_stats ||
	    !pa_dev->reg_collect_stats || !pa_dev->pa_sram) {
		dev_err(dev, "failed to set up PA system register areas\n");
		ret = -ENOMEM;
		goto exit;
	}

	for (i = 0; i < PA2_NUM_CLUSTERS; i++) {
		pa_dev->cluster[i].splitter = devm_ioremap(dev,
				regs_base + PA2_CLUSTER_SPLITTER_REGS(i),
				sizeof(struct pa2_cl_splitter_regs));
		if (!pa_dev->cluster[i].splitter)
			break;
		pa_dev->cluster[i].sram = devm_ioremap(dev,
					regs_base + PA2_CLUSTER_SRAM_REGS(i),
					PA2_CLUSTER_SRAM_SIZE);
		if (!pa_dev->cluster[i].sram)
			break;
	}

	if (i != PA2_NUM_CLUSTERS) {
		dev_err(dev, "failed to set up PA Cluster register areas\n");
		ret = -ENOMEM;
		goto exit;
	}

	for (i = 0; i < PA2_NUM_PDSPS; i++) {
		pa_dev->ppu[i].ctl_status = devm_ioremap(dev,
			regs_base + pa2_ppu_regs_offset[i] + \
			PA2_PPU_CTL_STATUS_REGS_OFFSET,
			sizeof(struct pa2_ppu_ctl_status_regs));
		if (!pa_dev->ppu[i].ctl_status)
			break;
		pa_dev->ppu[i].debug = devm_ioremap(dev,
			regs_base + pa2_ppu_regs_offset[i] + \
			PA2_PPU_DEBUG_REGS_OFFSET,
			sizeof(struct pa2_ppu_debug_regs));
		if (!pa_dev->ppu[i].debug)
			break;
		pa_dev->ppu[i].cp_timer = devm_ioremap(dev,
			regs_base + pa2_ppu_regs_offset[i] + \
			PA2_PPU_CP_TIMER_REGS_OFFSET,
			sizeof(struct pa2_ppu_cp_timer_regs));
		if (!pa_dev->ppu[i].cp_timer)
			break;
		pa_dev->ppu[i].lut1 = devm_ioremap(dev,
			regs_base + pa2_ppu_regs_offset[i] + \
			PA2_PPU_LUT1_REGS_OFFSET,
			sizeof(struct pa2_ppu_lut1_regs));
		if (!pa_dev->ppu[i].lut1)
			break;
		pa_dev->ppu[i].lut2 = devm_ioremap(dev,
			regs_base + pa2_ppu_regs_offset[i] + \
			PA2_PPU_LUT2_REGS_OFFSET,
			sizeof(struct pa2_ppu_lut2_regs));
		if (!pa_dev->ppu[i].lut2)
			break;
		pa_dev->ppu[i].pcheck = devm_ioremap(dev,
			regs_base + pa2_ppu_regs_offset[i] + \
			PA2_PPU_PCHECK_REGS_OFFSET,
			sizeof(struct pa2_ppu_pcheck_regs));
		if (!pa_dev->ppu[i].pcheck)
			break;
		pa_dev->ppu[i].iram = devm_ioremap(dev,
		       regs_base + pa2_ppu_regs_offset[i] + PA2_PPU_IRAM_OFFSET,
			PA2_PPU_IRAM_SIZE);
		if (!pa_dev->ppu[i].iram)
			break;
	}
	if (i != PA2_NUM_PDSPS) {
		dev_err(dev, "failed to set up PA PPU register areas\n");
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
		pa_dev->txhook_order = PA2_TXHOOK_ORDER;
	}
	dev_dbg(dev, "txhook-order %u\n", pa_dev->txhook_order);

	if (pa_dev->csum_offload == CSUM_OFFLOAD_SOFT) {
		ret = of_property_read_u32(node, "txhook-softcsum",
					   &pa_dev->txhook_softcsum);
		if (ret < 0) {
			dev_err(dev, "missing txhook-softcsum param, err %d\n",
				ret);
			pa_dev->csum_offload = CSUM_OFFLOAD_NONE;
			pa_dev->txhook_order = ~0;
		}
		dev_dbg(dev, "txhook-softcsum %u\n", pa_dev->txhook_softcsum);
	}

	ret = of_property_read_u32(node, "rxhook-order",
				   &pa_dev->rxhook_order);
	if (ret < 0) {
		dev_err(dev, "missing rxhook-order parameter, err %d\n",
			ret);
		pa_dev->rxhook_order = PA2_RXHOOK_ORDER;
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
		   ((pa_dev->mark_mcast_match[0] & \
		     ~pa_dev->mark_mcast_match[1]) != 0)) {
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

	prange = devm_kzalloc(dev, len, GFP_KERNEL);
	if (!prange) {
		dev_err(dev, "memory alloc failed at PA lut entry range\n");
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
	len = table_size * sizeof(struct pa2_lut_entry);
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

	devm_kfree(pa_dev->dev, prange);

	spin_lock_init(&pa_dev->lock);

	return 0;

exit:
	pa2_remove(netcp_device, pa_dev);
	*inst_priv = NULL;
	return ret;
}


static struct netcp_module pa2_module = {
	.name		= "keystone-pa2",
	.owner		= THIS_MODULE,
	.probe		= pa2_probe,
	.open		= pa2_open,
	.close		= pa2_close,
	.remove		= pa2_remove,
	.attach		= pa2_attach,
	.release	= pa2_release,
	.add_addr	= pa2_add_addr,
	.del_addr	= pa2_del_addr,
	.ioctl		= pa2_ioctl,
};

static int __init keystone_pa2_init(void)
{
	return netcp_register_module(&pa2_module);
}
module_init(keystone_pa2_init);

static void __exit keystone_pa2_exit(void)
{
	netcp_unregister_module(&pa2_module);
}
module_exit(keystone_pa2_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Hao Zhang <hzhang@ti.com>");
MODULE_DESCRIPTION("Packet Accelerator 2 driver for Keystone devices");
