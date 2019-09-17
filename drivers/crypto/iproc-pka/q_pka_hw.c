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

#include <linux/kernel.h>
#include <linux/io.h>
#include "q_lip.h"
#include "q_lip_aux.h"
#include "q_pka_hw.h"

#define PKA_REG_WRITE_BIT 0x1L

void *pka_hw_base_addr;
void *pka_hw_reg_ctlstat;
void *pka_hw_reg_din;
void *pka_hw_reg_dout;

uint32_t q_pka_sel_lir(uint32_t size)
{
	if (size <= 2)
		return PKA_LIR_A;
	else if (size <= 4)
		return PKA_LIR_B;
	else if (size <= 8)
		return PKA_LIR_C;
	else if (size <= 16)
		return PKA_LIR_D;
	else if (size <= 24)
		return PKA_LIR_E;
	else if (size <= 32)
		return PKA_LIR_F;
	else if (size <= 48)
		return PKA_LIR_G;
	else if (size <= 64)
		return PKA_LIR_H;
	else if (size <= 96)
		return PKA_LIR_I;
	else if (size <= 128)
		return PKA_LIR_J;
	else if (size <= 192)
		return PKA_LIR_K;
	else if (size <= 256)
		return PKA_LIR_L;
	else if (size <= 384)
		return PKA_LIR_M;
	else if (size <= 512)
		return PKA_LIR_N;
	else
		return PKA_NULL;
}

uint32_t q_pka_lir_size(uint32_t lir)
{
	uint32_t size;

	switch (lir) {
	case PKA_LIR_A:
		size = 2;
		break;
	case PKA_LIR_B:
		size = 4;
		break;
	case PKA_LIR_C:
		size = 8;
		break;
	case PKA_LIR_D:
		size = 16;
		break;
	case PKA_LIR_E:
		size = 24;
		break;
	case PKA_LIR_F:
		size = 32;
		break;
	case PKA_LIR_G:
		size = 48;
		break;
	case PKA_LIR_H:
		size = 64;
		break;
	case PKA_LIR_I:
		size = 96;
		break;
	case PKA_LIR_J:
		size = 128;
		break;
	case PKA_LIR_K:
		size = 192;
		break;
	case PKA_LIR_L:
		size = 256;
		break;
	case PKA_LIR_M:
		size = 384;
		break;
	case PKA_LIR_N:
		size = 512;
		break;
	default:
		size = 0;
		break;
	}

	return size;
}

void q_pka_hw_rst()
{
	uint32_t value;

	value = ioread32(PKA_HW_REG_CTLSTAT);
	iowrite32(PKA_CTL_RST | PKA_CTL_EN, PKA_HW_REG_CTLSTAT);
	iowrite32(value | PKA_CTL_EN, PKA_HW_REG_CTLSTAT);
}

/*
 * this function call uses a sequence to zeroize the
 * internal memory of the PKA hardware
 */
int32_t q_pka_zeroize_mem(struct q_lip_ctx *ctx)
{
	uint32_t pka_status;
	int32_t status = Q_SUCCESS;
	struct q_lint zero;
	struct opcode sequence[12];
	int i;

	/*
	 * use the second biggest register size
	 * supported by PKA, 2k-bit.
	 */
	status = q_init(ctx, &zero, 64);
	if (status != Q_SUCCESS)
		goto Q_ZEROIZE_MEM_EXIT;

	for (i = 0; i < 11; i++) {
		sequence[i].op1 =
			PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(PKA_LIR_H, i), 64);
		sequence[i].ptr = zero.limb;
	}
	sequence[11].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_LADD, PKA_LIR(PKA_LIR_H, 2),
				 PKA_LIR(PKA_LIR_H, 0));
	sequence[11].op2 = PACK_OP2(PKA_LIR(PKA_LIR_H, 1), PKA_NULL);
	sequence[11].ptr = NULL;

	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(12, sequence);

	pka_status = q_pka_hw_rd_status();
	while (!(pka_status & PKA_STAT_DONE))
		;

	q_free(ctx, &zero);
	q_pka_hw_rst();

Q_ZEROIZE_MEM_EXIT:
	return status;
}

uint32_t q_pka_hw_rd_status()
{
	uint32_t status = 0;

	status = ioread32(PKA_HW_REG_CTLSTAT);

	return status;
}
EXPORT_SYMBOL(q_pka_hw_rd_status);

void q_pka_hw_wr_status(uint32_t status)
{
	iowrite32(status, PKA_HW_REG_CTLSTAT);
}

void q_pka_hw_write_sequence(int count, struct opcode *sequence)
{
	int i, j;
	uint32_t size;
	for (i = 0; i < count; i++) {
		iowrite32(sequence[i].op1, PKA_HW_REG_DIN);
		if (sequence[i].ptr == NULL)
			iowrite32(sequence[i].op2, PKA_HW_REG_DIN);
		else {
			size = sequence[i].op1 & 0x0fff;
			for (j = 0; j < size; j++)
				iowrite32(sequence[i].ptr[j], PKA_HW_REG_DIN);
		}
	}
}

void q_pka_hw_read_lir(int size, uint32_t *data)
{
	int i;

	for (i = 0; i < size; i++)
		data[i] = ioread32(PKA_HW_REG_DOUT);
}

void q_initialize_pka(struct pka_driver *pka)
{
	pka_hw_reg_ctlstat = pka->hw_reg_ctlstat;
	pka_hw_reg_din = pka->hw_reg_din;
	pka_hw_reg_dout = pka->hw_reg_dout;
	pka_hw_base_addr = pka_hw_reg_ctlstat;
}
EXPORT_SYMBOL(q_initialize_pka);
