/*
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef _AXXIA_RIO_H_
#define _AXXIA_RIO_H_

#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>

#ifdef CONFIG_AXXIA_FAULT
#include <linux/axxia-fault.h>
#endif

#include "axxia-rio-irq.h"
/* Constants, Macros, etc. */

#define AXXIA_RIO_SYSMEM_BARRIER()	smp_mb()


/*****************************************/
/* *********** Byte Swapping *********** */
/*****************************************/
#define BSWAP(x)  __builtin_bswap32(x)     /* Use gcc built-in byte swap code */

#define DESTID_INVALID (u16)(0xff)
#define RIO_SET_DID(size, x)	(size ? (x & 0xffff) : ((x & 0x000000ff) << 16))

/*******************************/
/* *********** ACP *********** */
/*******************************/
#define ACP_MAX_RESOURCES       16
#define ACP_HW_DESC_RESOURCE    0
#define ACP_RESOURCE_HW_DESC	0x00000100


/*****************************************/
/* *********** ACP/AXXIA REG *********** */
/*****************************************/
#define SRIO_CONF_SPACE_SIZE          0x1000
#define SRIO_CONF_SPACE_SIZE_FIXED    0x0800
#define SRIO_CONF_SPACE_SIZE_PAGED    0x0800

#define SRIO_SPACE_SIZE         0x40000      /* Total size GRIO + RAB Spaces */

/* End point models & revisions */
#define AXXIA_DEVID_ACP34XX		0x5101000a
#define AXXIA_DEVID_ACP25XX		0x5108000a
#define AXXIA_DEVID_AXM55XX		0x5120000a
#define AXXIA_DEVID_AXM35XX		0x5102000a
#define   AXXIA_DEVREV_AXM55XX_V1_0	  0x00000000
#define   AXXIA_DEVREV_AXM55XX_V1_1	  0x80000000
#define   AXXIA_DEVREV_AXM55XX_V1_2	  0x40000000

/* End Point Controller Specific Registers (0x1_0000-0x1_FFFC) */
#define EPC_REG_BASE            0x10000
#define EPC_PNADIDCSR(x)        (EPC_REG_BASE + (0x100+((x)*0x80)))
#define  EPC_PNADIDCSR_ADE	(1 << 31)
#define  EPC_PNADIDCSR_ADID_SMALL(id)	((u32)((id) & 0x00ff) << 16)
#define  EPC_PNADIDCSR_ADID_LARGE(id)	((u32)((id) & 0xffff) <<  0)
#define EPC_PNPTAACR(x)	        (EPC_REG_BASE + (0x120+((x)*0x80)))
#define EPC_IECSR(x)            (EPC_REG_BASE + (0x130+((x)*0x80)))
#define  EPC_IECSR_RETE         0x80000000   /*WOCL*/

/* Peripheral Bus Bridge Specific Registers (0x2_0000-0x3_FFFC) */
#define RAB_REG_BASE            0x20000
#define RAB_VER                 (RAB_REG_BASE + 0x00)
#define RAB_APB_CSR_BASE        0x30
#define RAB_APB_CSR             (RAB_REG_BASE + RAB_APB_CSR_BASE)
#define RAB_CTRL                (RAB_REG_BASE + 0x08)
#define RAB_STAT                (RAB_REG_BASE + 0x0c)
#define AXI_TIMEOUT             (RAB_REG_BASE + 0x10)
#define DME_TIMEOUT             (RAB_REG_BASE + 0x14)


#define RAB_STAT_ESEL			(RAB_REG_BASE + 0x20)
#define  RAB_STAT_AXI_PIO_ENG   0u
#define  RAB_STAT_RIO_PIO_ENG   1u
#define  RAB_STAT_OB_DME_ENG    6u
#define  RAB_STAT_IB_DME_ENG    7u
#define RAB_STAT_STAT			(RAB_REG_BASE + 0x24)

#define RAB_PIO_RESET           (RAB_REG_BASE + 0x18)
#define  RAB_PIO_RESET_RPIO     0x00000100
#define  RAB_PIO_RESET_APIO     0x00000001

#define RAB_COOP_LOCK           (RAB_REG_BASE + 0x1C)
#define RAB_IB_PW_CSR           (RAB_REG_BASE + 0x28)
#define RAB_IB_PW_EN                    0x1UL
#define RAB_IB_PW_NUMWORDS(csr) (((csr) & 0x001f0000) >> 16)

#define RAB_IB_PW_DATA          (RAB_REG_BASE + 0x2C)

#define RAB_RPIO_CTRL           (RAB_REG_BASE + 0x80)
#define RAB_RPIO_PIO_EN                 0x1
#define RAB_RPIO_RELAX_ORDER            0x2

#define RAB_RPIO_STAT           (RAB_REG_BASE + 0x84)
#define  RAB_RPIO_STAT_RSP_ERR  0x00000004
#define  RAB_RPIO_STAT_ADDR_MAP 0x00000002
#define  RAB_RPIO_STAT_DISABLED 0x00000001

/* AXI PIO (outbound)*/
#define RAB_APIO_CTRL           (RAB_REG_BASE + 0x180)
#define RAB_APIO_CPP_EN                 0x8
#define RAB_APIO_MAINT_MAP_EN           0x4
#define RAB_APIO_MEM_MAP_EN             0x2
#define RAB_APIO_PIO_EN                 0x1

/* SRIO PHY Control */
#define RAB_SRDS_CTRL0          (RAB_REG_BASE + 0x980)
#define  RAB_SRDS_CTRL0_16B_ID   0x00000004

#define RAB_SRDS_CTRL1          (RAB_REG_BASE + 0x984)
#define  RAB_SRDS_CTRL1_RST      0x00000001
#define  RAB_SRDS_CTRL1_CLR_LNK  0x00000002
#define  RAB_SRDS_CTRL1_IES_EN   0x00000008
#define  RAB_SRDS_CTRL1_OES_EN   0x00000010
#define  RAB_SRDS_CTRL1_PERR_EN  0x00000080

#define RAB_SRDS_CTRL2          (RAB_REG_BASE + 0x988)
#define RAB_SRDS_STAT0          (RAB_REG_BASE + 0x990)
#define RAB_SRDS_STAT1          (RAB_REG_BASE + 0x994)
#define  RAB_SRDS_STAT1_LINKDOWN_INT    0x80000000

/* AW SMON control */
#define RAB_SMON_CTRL0          (RAB_REG_BASE + 0x9A0)
#define  RAB_SMON_CTRL0_INT_TM_OF       0x00200000
#define  RAB_SMON_CTRL0_INT_CNT_OF      0x00020000
#define  RAB_SMON_CTRL0_ENB             0x00000001


/* register contains transaction type, target id */
#define RAB_APIO_AMAP_CTRL(n)   (RAB_REG_BASE + (0x200 + (n * 0x10)))
#define CRF                             (0x80UL)
#define PRIO(prio)                      (((u32)(prio) & 0x3) << 3)
#define PRIO_VAL(reg)                   (((reg) >> 3) & 0x3)
#define MRD_MWR                         (0x0)
#define NRD_NWR                         (0x1)
#define NRD_NWR_R                       (0x2)
#define NRD_SWR                         (0x3)
#define TTYPE(type)                     (((u32)(type) & 0x3) << 1)
#define TTYPE_VAL(reg)                  (((reg) >> 1) & 0x3)
#define TARGID(tid)                     (((u32)(tid) & 0xffff) << 16)
#define ENABLE_AMBA                     (0x1UL)

/* register contains the AXI window size */
#define RAB_APIO_AMAP_SIZE(n)   (RAB_REG_BASE + (0x204 + (n * 0x10)))
#define WIN_SIZE(size)                  (size & 0xfffffc00)

/* register for AXI based address for window */
#define RAB_APIO_AMAP_ABAR(n)		(RAB_REG_BASE + (0x208 + (n * 0x10)))
#define AXI_BASE_HIGH(addr)             ((u32)(((u64)(addr) & 0x3f00000000ULL) \
					 >> 32) << 22)
#define AXI_BASE(addr)                  (((u32)(addr) & 0xfffffc00) >> 10)

/* Register for RIO base address */
#define RAB_APIO_AMAP_RBAR(n)   (RAB_REG_BASE + (0x20C + (n * 0x10)))
#define RIO_ADDR_BASE(taddr)            (((u32)(taddr) & 0xfffffc00) >> 10)
#define RIO_ADDR_OFFSET(taddr)          ((u32)(taddr) & 0x3ff)
#define HOP_COUNT(hop_cnt)              (((u32)(hop_cnt) & 0xff) << 14)


/* Other */
#define RAB_LFC_BLOCKED         (RAB_REG_BASE + 0x964)
#define RAB_SRDS_CTRL0          (RAB_REG_BASE + 0x980)


/* Interrupt registers */
#define RAB_INTR_ENAB_GNRL      (RAB_REG_BASE + 0x40)
/* General int enable  bits */
#define RAB_INT_OB_DB_EN        (1 << 31)
#define EXT_INT_OB_DB_EN        (0xff << 16)
#define MISC_INT_EN             (1 << 6)
#define OB_DME_INT_EN           (1 << 5)
#define IB_DME_INT_EN           (1 << 4)
#define RPIO_INT_EN             (1 << 1)
#define APIO_INT_EN             (1)

#define RAB_INTR_ENAB_GNRL_SET  (MISC_INT_EN | RPIO_INT_EN | \
			 APIO_INT_EN/* | OB_DME_INT_EN | IB_DME_INT_EN*/)

#define RAB_INTR_STAT_GNRL      (RAB_REG_BASE + 0x60)
/* General int status bits */
#define RAB_INTERNAL_STAT       (1 << 31)
#define EXT_INT_STATUS          (0xff << 16)
#define MISC_INT                (1 << 6)
#define OB_DME_INT              (1 << 5)
#define IB_DME_INT              (1 << 4)
#define RPIO_INT                (1 << 1)
#define APIO_INT                (1)

#define RAB_INTR_ENAB_APIO      (RAB_REG_BASE + 0x44)
#define RAB_INTR_ENAB_RPIO      (RAB_REG_BASE + 0x48)
#define RAB_INTR_ENAB_IDME      (RAB_REG_BASE + 0x54)
#define RAB_INTR_ENAB_ODME      (RAB_REG_BASE + 0x58)
#define RAB_INTR_ENAB_MISC      (RAB_REG_BASE + 0x5c)

#define RAB_INTR_STAT_APIO      (RAB_REG_BASE + 0x64)

/* Data_streaming */
#define RAB_INTR_ENAB_ODSE      (RAB_REG_BASE + 0x2a0c)
#define RAB_INTR_ENAB_IBDS      (RAB_REG_BASE + 0x2a04)
#define RAB_INTR_STAT_ODSE      (RAB_REG_BASE + 0x2a18)
#define RAB_INTR_STAT_IBSE_VSID_M (RAB_REG_BASE + 0x2a10)

/* PIO int status bits */
#define APIO_TRANS_FAILED       (1 << 8)
#define APIO_TRANS_COMPLETE     (1)
#define RAB_INTR_ENAB_APIO_SET  (APIO_TRANS_FAILED)

#define RAB_APIO_STAT           (RAB_REG_BASE + 0x184)
#define  RAB_APIO_STAT_RQ_ERR     0x00000040
#define  RAB_APIO_STAT_TO_ERR     0x00000020
#define  RAB_APIO_STAT_RSP_ERR    0x00000010
#define  RAB_APIO_STAT_MAP_ERR    0x00000008
#define  RAB_APIO_STAT_MAINT_DIS  0x00000004
#define  RAB_APIO_STAT_MEM_DIS    0x00000002
#define  RAB_APIO_STAT_DISABLED   0x00000001

#define RAB_INTR_STAT_RPIO      (RAB_REG_BASE + 0x68)
#define RPIO_TRANS_FAILED       (1 << 8)
#define RPIO_TRANS_COMPLETE     (1)
#define RAB_INTR_ENAB_RPIO_SET  (RPIO_TRANS_FAILED | RPIO_TRANS_COMPLETE)

#define RAB_INTR_STAT_MISC      (RAB_REG_BASE + 0x7c)
/* Misc int status bits */
#define UNEXP_MSG_LOG           (0xff << 24)
#define USR_INT                 (1 << 16)
#define AMST_INT                (1 << 11)
#define ASLV_INT                (1 << 10)
#define LFC_INT                 (1 << 9)
#define CO_LOCK_INT             (1 << 8)
#define LINK_REQ_INT            (1 << 7)
#define LL_TL_INT               (1 << 6)
#define GRIO_INT                (1 << 5)
#define PORT_WRITE_INT          (1 << 4)
#define UNSP_RIO_REQ_INT        (1 << 3)
#define UNEXP_MSG_INT           (1 << 2)
#define OB_DB_DONE_INT          (1 << 1)
#define IB_DB_RCV_INT           (1)

/* AMBA (AXI/AHB) Master/Slave */
#define RAB_ASLV_STAT_CMD       (RAB_REG_BASE + 0x1c0)
#define  RAB_ASLV_STAT_CMD_USUP 0x00000001

#define RAB_ASLV_STAT_ADDR      (RAB_REG_BASE + 0x1c4)
#define RAB_AMAST_STAT          (RAB_REG_BASE + 0x1e0)
#define  RAB_AMAST_STAT_WRTO    0x00000020
#define  RAB_AMAST_STAT_RDTO    0x00000010
#define  RAB_AMAST_STAT_WRDE    0x00000008
#define  RAB_AMAST_STAT_WRSE    0x00000004
#define  RAB_AMAST_STAT_RDDE    0x00000002
#define  RAB_AMAST_STAT_RDSE    0x00000001


#define MISC_FATAL (AMST_INT | ASLV_INT)

#if defined(CONFIG_AXXIA_RIO_STAT)

#define MISC_ERROR_INDICATION (MISC_FATAL | GRIO_INT | LL_TL_INT | \
			       UNEXP_MSG_LOG | UNSP_RIO_REQ_INT | \
			       UNEXP_MSG_INT)
#define MISC_DB_EVENT (OB_DB_DONE_INT | IB_DB_RCV_INT)

#else

#define MISC_ERROR_INDICATION MISC_FATAL
#define MISC_DB_EVENT IB_DB_RCV_INT

#endif

#define RAB_INTR_ENAB_MISC_SET  (MISC_ERROR_INDICATION | MISC_DB_EVENT);

/* Inbound/Outbound int bits */
#define RAB_INTR_ENAB_IDME_SET  (~(0UL))
#define RAB_INTR_ENAB_ODME_SET  (0x7)


/************************************/
/* *********** MESSAGES *********** */
/************************************/
/* Outbound Doorbell */
#define RAB_OB_DB_CSR(n)        (RAB_REG_BASE + (0x400 + (n * 0x8)))
#define OB_DB_DEST_ID(id)               (((u32)(id) & 0xffff) << 16)
#define OB_DB_CRF                       (1 << 6)
#define OB_DB_PRIO(prio)                (((u32)(prio) & 0x3) << 4)
#define OB_DB_STATUS(reg)               (((u32)(reg) & 0xe) >> 1)
#define OB_DB_SEND                      (1)

#define OB_DB_STATUS_DONE       (0)
#define OB_DB_STATUS_RETRY      (1)
#define OB_DB_STATUS_ERROR      (2)
#define OB_DB_STATUS_TIMEOUT    (3)
#define OB_DB_STATUS_PENDING    (4)

#define MAX_OB_DB               (8)

#define RAB_OB_DB_INFO(n)       (RAB_REG_BASE + (0x404 + (n * 0x8)))
#define OB_DB_INFO(info)                ((u32)(info) & 0xffff)

/* Inbound Doorbell */
#define RAB_IB_DB_CSR           (RAB_REG_BASE + 0x480)
#define IB_DB_CSR_NUM_MSG(csr)          (((u32)(csr) & 0x3f0000) >> 16)
#define IB_DB_CSR_EN                    (1)

#define RAB_IB_DB_INFO          (RAB_REG_BASE + 0x484)

#define DBELL_SID(info)		(((u32)(info) & 0xffff0000) >> 16)
#define DBELL_INF(info)		((u32)(info) & 0xffff)

/* Messages */
#define RAB_OB_DME_CTRL(e)      (RAB_REG_BASE + (0x500 + (0x10 * (e))))
#define RAB_OB_DME_DESC_ADDR(e) (RAB_REG_BASE + (0x504 + (0x10 * (e))))
#define RAB_OB_DME_STAT(e)      (RAB_REG_BASE + (0x508 + (0x10 * (e))))
#define RAB_OB_DME_DESC(e)      (RAB_REG_BASE + (0x50C + (0x10 * (e))))
#define RAB_OB_DME_TID_MASK     (RAB_REG_BASE + 0x5f0)

#define RAB_INTR_STAT_ODME      (RAB_REG_BASE + 0x78)
#define OB_DME_STAT_SLEEPING             (1 << 9)
#define OB_DME_STAT_TRANS_PEND           (1 << 8)
#define OB_DME_STAT_RESP_TO              (1 << 7)
#define OB_DME_STAT_RESP_ERR             (1 << 6)
#define OB_DME_STAT_DATA_TRANS_ERR       (1 << 5)
#define OB_DME_STAT_DESC_UPD_ERR         (1 << 4)
#define OB_DME_STAT_DESC_ERR             (1 << 3)
#define OB_DME_STAT_DESC_FETCH_ERR       (1 << 2)
#define OB_DME_STAT_DESC_XFER_CPLT       (1 << 1)
#define OB_DME_STAT_DESC_CHAIN_XFER_CPLT (1)

#define OB_DME_STAT_ERROR_MASK           0x000000FC
#define OB_DME_TID_MASK                  0xFFFFFFFF

#define RAB_IB_DME_CTRL(e)      (RAB_REG_BASE + (0x600 + (0x10 * (e))))
#define   RAB_IB_DME_CTRL_XMBOX(m)           (((m) & 0x3c) << 6)
#define   RAB_IB_DME_CTRL_MBOX(m)            (((m) & 0x03) << 6)
#define   RAB_IB_DME_CTRL_LETTER(l)          (((l) & 0x03) << 4)
#define RAB_IB_DME_DESC_ADDR(e) (RAB_REG_BASE + (0x604 + (0x10 * (e))))
#define RAB_IB_DME_STAT(e)      (RAB_REG_BASE + (0x608 + (0x10 * (e))))
#define RAB_IB_DME_DESC(e)      (RAB_REG_BASE + (0x60C + (0x10 * (e))))

#define RAB_INTR_STAT_IDME      (RAB_REG_BASE + 0x74)
#define IB_DME_STAT_SLEEPING             (1 << 9)
#define IB_DME_STAT_TRANS_PEND           (1 << 8)
#define IB_DME_STAT_MSG_TIMEOUT          (1 << 7)
#define IB_DME_STAT_MSG_ERR              (1 << 6)
#define IB_DME_STAT_DATA_TRANS_ERR       (1 << 5)
#define IB_DME_STAT_DESC_UPDATE_ERR      (1 << 4)
#define IB_DME_STAT_DESC_ERR             (1 << 3)
#define IB_DME_STAT_DESC_FETCH_ERR       (1 << 2)
#define IB_DME_STAT_DESC_XFER_CPLT       (1 << 1)
#define IB_DME_STAT_DESC_CHAIN_XFER_CPLT (1)

#define IB_DME_STAT_ERROR_MASK		0x000000FC

#define DME_WAKEUP			(2)
#define DME_ENABLE			(1)

/* DME Message Descriptor Table */
#define DESC_TABLE_W0_NDX(d)         (0x10 * (d))
#define DESC_TABLE_W0_RAB_BASE(d)    (RAB_REG_BASE+0x10000+DESC_TABLE_W0_NDX(d))
#define DESC_TABLE_W0(d)                (DESC_TABLE_W0_RAB_BASE(d) + 0x0)
#define DESC_TABLE_W1(d)                (DESC_TABLE_W0_RAB_BASE(d) + 0x4)
#define DESC_TABLE_W2(d)                (DESC_TABLE_W0_RAB_BASE(d) + 0x8)
#define DESC_TABLE_W3(d)                (DESC_TABLE_W0_RAB_BASE(d) + 0xC)

#define DESC_TABLE_W0_MEM_BASE(me, d)		\
	(((u8 *)(me)->descriptors) + DESC_TABLE_W0_NDX(d))
#define DESC_TABLE_W0_MEM(me, d)        (DESC_TABLE_W0_MEM_BASE(me, d) + 0x0)
#define DESC_TABLE_W1_MEM(me, d)        (DESC_TABLE_W0_MEM_BASE(me, d) + 0x4)
#define DESC_TABLE_W2_MEM(me, d)        (DESC_TABLE_W0_MEM_BASE(me, d) + 0x8)
#define DESC_TABLE_W3_MEM(me, d)        (DESC_TABLE_W0_MEM_BASE(me, d) + 0xC)

#define DME_DESC_DW0_SRC_DST_ID(id)     ((id) << 16)
#define DME_DESC_DW0_GET_DST_ID(dw0)    (((dw0) >> 16) & 0xffff)
#define DME_DESC_DW0_RIO_ERR            (1 << 11)
#define DME_DESC_DW0_AXI_ERR            (1 << 10)
#define DME_DESC_DW0_TIMEOUT_ERR        (1 << 9)
#define DME_DESC_DW0_DONE               (1 << 8)
#define DME_DESC_DW0_SZ_MASK            (3 << 4)
#define DME_DESC_DW0_EN_INT             (1 << 3)
#define DME_DESC_DW0_END_OF_CHAIN       (1 << 2)
#define DME_DESC_DW0_NXT_DESC_VALID     (1 << 1)
#define DME_DESC_DW0_VALID              (1)

#define DESC_STATE_TO_ERRNO(s)		(s & DME_DESC_DW0_TIMEOUT_ERR ? \
					 -ETIME : (s & (DME_DESC_DW0_RIO_ERR | \
					 DME_DESC_DW0_AXI_ERR) ? -EPROTO : 0))

#define DME_DESC_DW0_READY_MASK         0x00000F00
#define DME_DESC_DW0_ERROR_MASK         0x00000E00
#define DME_DESC_DW0_SEG(d)             ((d & DME_DESC_DW0_SZ_MASK) >> 4)
#define DME_DESC_DW0_SIZE(s)            (s == 0 ? 512 : \
					 (s == 1 ? 1024 :	\
					  (s == 2 ? 2048 : 4096)))

#define DME_DESC_DW1_PRIO(flags)        ((flags & 0x3) << 30)
#define DME_DESC_DW1_CRF(flags)         ((flags & 0x4) << 27)
#define DME_DESC_DW1_SEG_SIZE_256       (0x06 << 18)
#define DME_DESC_DW1_XMBOX(m)           (((m) & 0x3c) << 2)
#define DME_DESC_DW1_MBOX(m)            (((m) & 0x03) << 2)
#define DME_DESC_DW1_LETTER(l)          ((l) & 0x03)
#define DME_DESC_DW1_MSGLEN(s)          ((((s + 7) & ~7) >> 3) << 8) /* Round
					 up and shift to make double word */
#define DME_DESC_DW1_MSGLEN_F(d)        (((d) >> 8) & 0x3ff)
#define DME_DESC_DW1_MSGLEN_B(ml)       ((ml) << 3) /* double words to bytes */
#define DME_DESC_DW1_GET_LETTER(dw1)	((dw1) & 0x03)
#define DME_DESC_DW1_GET_MBOX(dw1)	((dw1 >> 2) &0x03)
/***********************************/
/* *********** RIO REG *********** */
/***********************************/
#define RIO_PLTOCCSR            0x120
#define RIO_PRTOCCSR            0x124
#define RIO_GCCSR		0x13c

#define RIO_LINK_TIMEOUT_VAL           (0x40)
#define RIO_RESPONSE_TIMEOUT_VAL       (0xfa0)

#define RIO_MNT_REQ_CSR(x)      (0x140+((x)*0x20))
#define  RIO_MNT_REQ_MASK       0x00000007
#define  RIO_MNT_REQ_RST        0x00000003
#define  RIO_MNT_REQ_STAT       0x00000004

#define RIO_MNT_RSP_CSR(x)      (0x144+((x)*0x20))
#define  RIO_MNT_RSP_LS         0x0000001f
#define  RIO_MNT_RSP_AS         0x000003e0
#define  RIO_MNT_RSP_RV         0x80000000

#define RIO_ACK_STS_CSR(x)      (0x148+((x)*0x20))
#define  RIO_ACK_STS_IA         0x1f000000
#define  RIO_ACK_STS_OUTA       0x00001f00
#define  RIO_ACK_STS_OBA        0x0000001f

#define RIO_ESCSR(x)            (0x158+((x)*0x20))
#define  RIO_ESCSR_I2E		 0x40000000   /*RW*/
#define  RIO_ESCSR_OPD           0x04000000   /*WOCL*/
#define  RIO_ESCSR_OFE           0x02000000   /*WOCL*/
#define  RIO_ESCSR_ODE           0x01000000   /*WOCL*/
#define  RIO_ESCSR_ORE           0x00100000   /*WOCL*/
#define  RIO_ESCSR_OR            0x00080000   /*R*/
#define  RIO_ESCSR_ORS           0x00040000   /*R*/
#define  RIO_ESCSR_OEE           0x00020000   /*WOCL*/
#define  RIO_ESCSR_OES           0x00010000   /*R--*/
#define  RIO_ESCSR_IRS           0x00000400   /*R*/
#define  RIO_ESCSR_IEE           0x00000200   /*WOCL*/
#define  RIO_ESCSR_IES           0x00000100   /*R--*/
#define  RIO_ESCSR_PWP           0x00000010   /*R*/
#define  RIO_ESCSR_PE            0x00000004   /*WOCL*/
#define  RIO_ESCSR_PO            0x00000002   /*R*/
#define  RIO_ESCSR_PU            0x00000001   /*R*/
#define  RIO_EXCSR_WOLR          (RIO_ESCSR_OPD | RIO_ESCSR_OFE | \
				  RIO_ESCSR_ODE | RIO_ESCSR_ORE | \
				  RIO_ESCSR_OEE | RIO_ESCSR_IEE | RIO_ESCSR_PE)

#define ESCSR_FATAL (RIO_ESCSR_OFE |		\
		     RIO_ESCSR_IES |		\
		     RIO_ESCSR_IRS |		\
		     RIO_ESCSR_ORS |		\
		     RIO_ESCSR_OES)

#define RIO_CCSR(x)		(0x15c+((x)*0x20))
#define  RIO_CCSR_PW             0xc0000000   /*R*/
#define  RIO_CCSR_IPW            0x38000000   /*R*/
#define  RIO_CCSR_PW_MASK        0x7
#define  RIO_CCSR_PWO_SHIFT      24
#define  RIO_CCSR_PWO            (RIO_CCSR_PW_MASK << RIO_CCSR_PWO_SHIFT)/*R/W*/
#define  RIO_CCSR_FORCE_LANE0    (2 << RIO_CCSR_PWO_SHIFT)
#define  RIO_CCSR_PD             0x00800000   /*R/W*/
#define  RIO_CCSR_OPE            0x00400000   /*R/W*/
#define  RIO_CCSR_IPE            0x00200000   /*R/W*/
#define  RIO_CCSR_FCP            0x00040000   /*R/W*/
#define  RIO_CCSR_EB             0x00020000   /*R*/
#define  RIO_CCSR_SPF            0x00000008   /*R/W*/
#define  RIO_CCSR_PL             0x00000002   /*R/W*/

#define RIO_PNPTAACR		0x10120

#define PCIE_SRIO_PHY0_CFG_OFS			(0x1E0U)

#define APB2SER0_CMD0_REG_OFS			(0x0u)
#define APB2SER0_DATA0_REG_OFS			(0x8u)

#define APB2SER0_CMD1_REG_OFS			(0x4u)
#define   APB2SER0_CMD1_VALID_FLD_OFS	(0x1Fu)
#define   APB2SER0_CMD1_VALID_FLD_SIZE	(1u)

#define   APB2SER0_CMD1_HWRITE_FLD_OFS	(0x1Eu)
#define   APB2SER0_CMD1_HWRITE_FLD_SIZE	(1u)

#define   APB2SER0_CMD1_TSHIFT_FLD_OFS	(0x1Au)
#define   APB2SER0_CMD1_TSHIFT_FLD_SIZE	(4u)

#define   APB2SER0_CMD1_HSIZE_FLD_OFS	(0x17u)
#define   APB2SER0_CMD1_HSIZE_FLD_SIZE	(3u)

#define   APB2SER0_CMD1_HTRANS_FLD_OFS	(0x15u)
#define   APB2SER0_CMD1_HTRANS_FLD_SIZE	(2u)

#define   APB2SER0_CMD1_HADDR_FLD_OFS	(0x00u)
#define   APB2SER0_CMD1_HADDR_FLD_SIZE	(16u)


#define APB2SER0_DATA1_REG_OFS			(0xCu)
#define   APB2SER0_DATA1_HRESP_FLD_OFS	(0x01u)
#define   APB2SER0_DATA1_HRESP_FLD_SIZE	(1u)

#define SET_FLD(DEST, FLD_SOURCE, VALUE) \
do { \
	DEST = ((DEST) & (~(((1 << FLD_SOURCE ## _SIZE) - 1) << FLD_SOURCE ## _OFS))) | \
		((VALUE) << FLD_SOURCE ## _OFS); \
	} \
while(0)

#define GET_FLD(SRC, FLD_SOURCE) 	(((SRC) & ((((1 << FLD_SOURCE ## _SIZE) - 1) << FLD_SOURCE ## _OFS))) >> FLD_SOURCE ## _OFS)

#define AXXIA_IBDME_INTERRUPT_MODE	0x1
#define AXXIA_IBDME_TIMER_MODE		0x2
/*************************************/
/* *********** Constants *********** */
/*************************************/

#define RIO_OUTB_ATMU_WINDOWS   16
#define AXXIA_BDMA_MAX_BCOUNT   (256*1024)
#define LSI_AXXIA_RIO_COOKIE	0x41734230	/* aka 'AsR0' */
#define AXXIA_BURST_STAT_PERIOD 10000000        /* 10 ms      */

/***********************************/
/* *********** STRUCTS *********** */
/***********************************/
struct atmu_outb {
	void __iomem *win;
	struct rio_atmu_regs __iomem *atmu_regs;
	struct resource *riores;
	int in_use;
};

struct event_regs {
	void __iomem *win;
	u64 phy_reset_start;
	u64 phy_reset_size;
	u32 reg_addr;
	u32 reg_mask;
	int in_use;
};

struct rio_desc {
	u32     d0;
	u32     d1;
	u32     d2;
	u32     d3;
};

#ifdef CONFIG_AXXIA_RIO_STAT
struct rio_time_stat {
	long long count;	/* access counter */
	ktime_t time;		/* current time */
	ktime_t accu;		/* time accu */
	ktime_t peak;		/* peak time */
};
#endif

struct rio_priv {
	u32     cookie;

	struct mutex api_lock;


	struct rio_mport *mport;
	struct device *dev;
	int  ndx;	/* From FDT description */
	int  port_ndx;
	u32  devid;     /* From GRIO register */
	u32  devrev;    /* From GRIO register */

	void __iomem *regs_win_fixed;
	void __iomem *regs_win_paged;

	struct mutex apio_mutex;

	int maint_win_id;
	struct atmu_outb outb_atmu[RIO_OUTB_ATMU_WINDOWS];
	struct resource acpres[ACP_MAX_RESOURCES];

	int intern_msg_desc;
	int desc_max_entries;

	/* Chip-specific DME availability */
	int num_outb_dmes[2];	/* [0]=MSeg, [1]=Sseg */
	int outb_dmes_in_use[2];
	int outb_dmes[2];	/* set of defined outbound DMEs:
				 *   [0]=MSeg, [1]=SSeg */
	int num_inb_dmes[2];	/* [0]=MSeg, [1]=Sseg */
	int inb_dmes_in_use[2];
	int inb_dmes[2];	/* set of defined inbound DMEs */

	u32 inb_dmes_map[4][RIO_MSG_MAX_LETTER];
	struct rio_tx_dme      ob_dme_shared[DME_MAX_OB_ENGINES];
	struct rio_tx_mbox     *ob_mbox[RIO_MAX_TX_MBOX];
	struct rio_rx_mbox __rcu *ib_mbox[RIO_MAX_RX_MBOX];
	struct rio_msg_dme     *ib_dme[DME_MAX_IB_ENGINES];
	struct rio_pw_irq *pw_data;
	unsigned int dme_mode;
	/* Linkdown Reset; Trigger via SRDS STAT1 */
	struct event_regs linkdown_reset;

	/* Interrupts */
	int irq_line;
	struct rio_irq_handler misc_irq;
	struct rio_irq_handler linkdown_irq; /* AXM55xx+SRDS STAT1+APB2SER */
	struct rio_irq_handler apio_irq;
	struct rio_irq_handler rpio_irq;
	struct rio_irq_handler ob_dme_irq;
	struct rio_irq_handler ib_dme_irq;

#ifdef CONFIG_AXXIA_RIO_STAT
	unsigned int rpio_compl_count;
	unsigned int rpio_failed_count;
	unsigned int apio_compl_count;
	unsigned int apio_failed_count;
	unsigned int rio_pw_count;
	unsigned int rio_pw_msg_count;
	unsigned int rio_pw_spurious;
	struct rio_time_stat maint_local_read;
	struct rio_time_stat maint_local_write;
	struct rio_time_stat maint_remote_read;
	struct rio_time_stat maint_remote_write;
#endif
#ifdef CONFIG_RAPIDIO_HOTPLUG
	/* Fatal err */
	spinlock_t port_lock;
	void (*port_notify_cb)(struct rio_mport *mport);
#endif
#ifdef CONFIG_AXXIA_RIO_DS
	/* Data_streaming */
	struct axxia_rio_ds_priv     ds_priv_data;
	struct axxia_rio_ds_cfg      ds_cfg_data;
#endif
#ifdef CONFIG_AXXIA_RIO_BURST_STAT
	/** Burst monitoring timer */
	struct hrtimer timer;
	/** Burst monitoring stats */
	struct {
		spinlock_t lock;
		unsigned count;
		ktime_t time, last;
		unsigned hist[12];
		bool reset_on_read;
	} burst;
#endif
} ____cacheline_internodealigned_in_smp;

/**********************************************/
/* *********** External Functions *********** */
/**********************************************/

extern int axxia_rio_start_port(struct rio_mport *mport);
extern void axxia_rio_set_mport_disc_mode(struct rio_mport *mport);
extern void axxia_rio_static_win_release(struct rio_mport *mport);
extern int axxia_rio_static_win_init(struct rio_mport *mport);

extern int axxia_local_config_read(struct rio_priv *priv,
				   u32 offset, u32 *data);
extern int axxia_local_config_write(struct rio_priv *priv,
				    u32 offset, u32 data);

#ifdef CONFIG_RAPIDIO_HOTPLUG

extern int axxia_rio_hotswap(struct rio_mport *mport, u8 flags);

#endif /* CONFIG_RAPIDIO_HOTPLUG */

#ifdef CONFIG_RAPIDIO_DMA_ENGINE

extern int axxia_rio_register_dma(struct rio_priv *priv);
extern void axxia_rio_unregister_dma(struct rio_priv *priv);

extern int axxia_rio_req_outb_region(struct rio_mport *mport,
				   resource_size_t size,
				   const char *name,
				   u32 mflags, u32 *id);
extern int axxia_rio_map_outb_mem(struct rio_mport *mport, u32 win,
			u16 destid, u32 addr, u32 mflags,
			u8 prio_lvl, void __iomem **va,
			resource_size_t *phys);
extern void axxia_rio_release_outb_region(struct rio_mport *mport,
					u32 win);

#endif /* CONFIG_RAPIDIO_DMA_ENGINE */

extern void apio_error_handler(struct rio_priv *priv, char const * const transfer_info);

int axxia_rio_is_x9(void);

extern u32 axxia_read_engine_status(struct rio_priv *priv, unsigned engine_type, unsigned engine_num);
#endif  /* _AXXIA_RIO_H_ */
