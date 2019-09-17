/*
 * Copyright 2015 Broadcom Corporation
 *
 * This program is the proprietary software of Broadcom Corporation and/or
 * its licensors, and may only be used, duplicated, modified or distribute
 * pursuant to the terms and conditions of a separate, written license
 * agreement executed between you and Broadcom (an "Authorized License").
 * Except as set forth in an Authorized License, Broadcom grants no license
 * (express or implied), right to use, or waiver of any kind with respect to
 * the Software, and Broadcom expressly reserves all rights in and to the
 * Software and all intellectual property rights therein.  IF YOU HAVE NO
 * AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO USE THIS SOFTWARE IN ANY
 * WAY, AND SHOULD IMMEDIATELY NOTIFY BROADCOM AND DISCONTINUE ALL USE OF
 * THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License,
 *
 * 1. This program, including its structure, sequence and organization,
 * constitutes the valuable trade secrets of Broadcom, and you shall use
 * all reasonable efforts to protect the confidentiality thereof, and to
 * use this information only in connection with your use of Broadcom
 * integrated circuit products.
 *
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED
 * "AS IS" AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES,
 * REPRESENTATIONS OR WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR
 * OTHERWISE, WITH RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY
 * DISCLAIMS ANY AND ALL IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY,
 * NONINFRINGEMENT, FITNESS FOR A PARTICULAR PURPOSE, LACK OF VIRUSES,
 * ACCURACY OR COMPLETENESS, QUIET ENJOYMENT, QUIET POSSESSION OR
 * CORRESPONDENCE TO DESCRIPTION. YOU ASSUME THE ENTIRE RISK ARISING
 * OUT OF USE OR PERFORMANCE OF THE SOFTWARE.
 *
 * 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL
 * BROADCOM OR ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL,
 * SPECIAL, INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR
 * IN ANY WAY RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE
 * EVEN
 * IF BROADCOM HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES; OR
 * (ii)
 * ANY AMOUNT IN EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE SOFTWARE
 * ITSELF
 * OR U.S. $1, WHICHEVER IS GREATER. THESE LIMITATIONS SHALL APPLY
 * NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY LIMITED REMEDY.
 */

#ifndef _Q_PKA_HW_H_
#define _Q_PKA_HW_H_

/* define all LIRs */
#define PKA_EOS     0x80000000

#define PKA_NULL    0x000

/* 64-bit */
#define PKA_LIR_A   0x100

/* 128-bit */
#define PKA_LIR_B   0x200

/* 256-bit */
#define PKA_LIR_C   0x300

/* 512-bit */
#define PKA_LIR_D   0x400

/* 768-bit */
#define PKA_LIR_E   0x500

/* 1024-bit */
#define PKA_LIR_F   0x600

/* 1536-bit */
#define PKA_LIR_G   0x700

/* 2048-bit */
#define PKA_LIR_H   0x800

/* 3072-bit */
#define PKA_LIR_I   0x900

/* 4096-bit */
#define PKA_LIR_J   0xA00

/* 6144-bit */
#define PKA_LIR_K   0xB00

/* 8192-bit */
#define PKA_LIR_L   0xC00

/* 12288-bit */
#define PKA_LIR_M   0xD00

/* 16384-bit */
#define PKA_LIR_N   0xE00

#define PKA_LIR(type, index) ((type) | (index))

/* define Opcodes */
#define PKA_LAST_OP     0x80000000

#define PKA_OP_MTLIR    0x01000000
#define PKA_OP_MFLIR    0x02000000
#define PKA_OP_MTLIRI   0x03000000
#define PKA_OP_MFLIRI   0x04000000
#define PKA_OP_CLIR     0x05000000
#define PKA_OP_SLIR     0x06000000
#define PKA_OP_NLIR     0x07000000
#define PKA_OP_MOVE     0x08000000
#define PKA_OP_RESIZE   0x09000000

#define PKA_OP_MODADD   0x10000000
#define PKA_OP_MODSUB   0x11000000
#define PKA_OP_MODREM   0x12000000
#define PKA_OP_MODMUL   0x13000000
#define PKA_OP_MODSQR   0x14000000
#define PKA_OP_MODEXP   0x15000000
#define PKA_OP_MODINV   0x16000000
#define PKA_OP_MODDIV2  0x17000000

#define PKA_OP_LCMP     0x20000000
#define PKA_OP_LADD     0x21000000
#define PKA_OP_LSUB     0x22000000
#define PKA_OP_LMUL     0x23000000
#define PKA_OP_LSQR     0x24000000
#define PKA_OP_LDIV     0x25000000
#define PKA_OP_LMUL2N   0x26000000
#define PKA_OP_LDIV2N   0x27000000
#define PKA_OP_LMOD2N   0x28000000
#define PKA_OP_SMUL     0x29000000
#define PKA_OP_PMADD    0x30000000
#define PKA_OP_PMSUB    0x31000000
#define PKA_OP_PMMUL    0x32000000

#define PKA_OP_PPSEL    0x40000000

struct pka_driver {
	void __iomem *hw_reg_ctlstat;
	void __iomem *hw_reg_din;
	void __iomem *hw_reg_dout;
};


struct opcode {
	uint32_t op1;
	uint32_t op2;
	uint32_t *ptr;
};
/* reset PKA hardware */
void q_pka_hw_rst(void);

/*
 * Zeroize PKA internal memory using a data loading sequence
 * param ctx QLIP context pointer
 */
int32_t q_pka_zeroize_mem(struct q_lip_ctx *ctx);

/*
 * Determine the type of LIR to use from the data size.
 * This function returns a LIR enumeration
 * param size data size in 32-bit words
 */
uint32_t q_pka_sel_lir(uint32_t size);

/*
 * return the size of the LIR register from the enumeration.
 * param lir the LIR register enumeration
 */
uint32_t q_pka_lir_size(uint32_t lir);

/* read PKA hardware status register and return the value. */
uint32_t q_pka_hw_rd_status(void);

/*
 * write PKA hardware status register with input value.
 * param status the updated value to be written
 */
void q_pka_hw_wr_status(uint32_t status);

/*
 * write opcode sequence to PKA hardware data register.
 * param count the number of opcodes to be written
 * param sequence pointer to the opcode sequence
 */
void q_pka_hw_write_sequence(int count, struct opcode *sequence);

/*
 * read PKA hardware data register to retrieve computation result.
 * param size the size of the data to be read
 * param data the pointer to data memory to store the read value.
 */
void q_pka_hw_read_lir(int size, uint32_t *data);

/*
 * Get the PKA hw values required to re-init PKA
 */
void q_initialize_pka(struct pka_driver *pka);

#define PACK_OP1(last_op, opcode, param0, param1) \
		((last_op) | (opcode) | ((param0) << 12) | param1)

#define PACK_OP2(param0, param1) \
		(((param0) << 12) | (param1))

extern void *pka_hw_base_addr;
extern void *pka_hw_reg_ctlstat;
extern void *pka_hw_reg_din;
extern void *pka_hw_reg_dout;

extern struct pka_driver *initialize_pka(void);

#define PKA_HW_BASE_ADDR     pka_hw_base_addr
#define PKA_HW_REG_CTLSTAT   pka_hw_reg_ctlstat
#define PKA_HW_REG_DIN       pka_hw_reg_din
#define PKA_HW_REG_DOUT      pka_hw_reg_dout

/* pka status bits we care about */
#define PKA_STAT_PPSEL_FAILED (0x1 << 9)
#define PKA_STAT_CARRY        (0x1 << 8)
#define PKA_CTL_RST           (0x1 << 7)
#define PKA_STAT_ERROR        (0x1 << 3)
#define PKA_STAT_BUSY         (0x1 << 2)
#define PKA_STAT_DONE         (0x1 << 1)
#define PKA_CTL_EN            (0x1 << 0)

#define PKA_CTL_ESCAPE_LOC    16
#define PKA_CTL_ESCAPE_MASK   0x7

#endif
