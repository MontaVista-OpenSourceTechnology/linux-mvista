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

#include "q_lip.h"
#include "q_lip_aux.h"

#ifdef QLIP_USE_PKA_HW
#include "q_pka_hw.h"
#endif

int32_t q_read_write(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a,
					struct q_lint *e,
					struct q_lint *n)
{
	int i;
	uint32_t pka_status;
	uint32_t lir_p, lir_c;
	struct opcode sequence[14];

	/* initialize hardware first */
	q_pka_hw_rst();

	/* sending command sequence */
	lir_p = q_pka_sel_lir(n->size);
	lir_c = q_pka_sel_lir(n->size * 2);

	/*
	 * sending command sequence
	 * load exponent e
	 * x[0] = n (modulus)
	 */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), n->size);
	sequence[0].ptr = n->limb;

	/*
	 * load base a
	 * x[1] = a (base)
	 */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), a->size);
	sequence[1].ptr = a->limb;

	/* x[2] = a (base) */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), a->size);
	sequence[2].ptr = a->limb;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(3, sequence);

	for (i = 0; i < 4; i++)
		sequence[i].ptr = NULL;

	/* read back result */
	sequence[0].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 1), a->size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(1, sequence);

	z->size = a->size;

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_MODEXP_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_MODEXP_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(z->size, z->limb);
Q_MODEXP_EXIT:
	return ctx->status;
}
