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
#include "q_lip.h"
#include "q_lip_utils.h"

#ifdef QLIP_USE_PKA_HW
#include "q_pka_hw.h"
#endif

/*
 * Both Diffie-Hellman functions are effectively a single modexp
 * operation. These two functions are coded for completeness.
 */

/*
 * q_ppsel () - Prime number pre-selection
 * Description:
 * This function tests the user-provided seed value against the
 * following small prime factors:
 * 3, 5, 7, 11, 13, 17, 31
 * If the seed value can be divided by any of the prime factors, the
 * seed value is incremented by 2 and the test is repeated.  The
 * process repeats until the maximum number of retries is reached.
 * If a value is found to be non-divisible by any of the
 * aforementioned factors, it will replace the user provided seed
 * and the funtion returns Q_SUCCESS.
 * @param ctx		QLIP context pointer
 * @param seed		pointer to user defined seed
 * @param step_size	pointer to the step size
 * @param num_retries	maximum number of retries
 */
int32_t q_ppsel(struct q_lip_ctx *ctx,
				   struct q_lint *seed,
				   uint32_t step_size,
				   uint32_t num_retries)
{
#ifdef QLIP_USE_PKA_HW
	uint32_t pka_status;
	uint32_t lir_p;
	struct opcode sequence[4];
#endif

#ifdef QLIP_USE_GMP
	ctx->status = Q_ERR_PPSEL_FAILED;
	goto Q_PPSEL_EXIT;
#endif /* QLIP_USE_GMP */

#ifdef QLIP_USE_PKA_HW
	/* Note: The PKA routine supports seed size up to 4 Kb */

	/* initialize hardware first */
	q_pka_hw_rst();

	/* sending command sequence */
	lir_p = PKA_LIR_J;

	/* x[0] = seed */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0), seed->size);
	sequence[0].ptr = seed->limb;

	/* x[1] = step_size */
	sequence[1].op1 = PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), 1);
	sequence[1].ptr = &step_size;

	/* x[0] = ppsel(x[0], x[1], num_retries) */
	sequence[2].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_PPSEL, PKA_LIR(lir_p, 0),
				 PKA_LIR(lir_p, 0));
	sequence[2].op2 =
		PACK_OP2((num_retries - 1) & 0xfff, PKA_LIR(lir_p, 1));
	sequence[2].ptr = NULL;

	/* unload result x[0] */
	sequence[3].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 0), seed->size);
	sequence[3].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_PPSEL_EXIT;
	}
	q_pka_hw_write_sequence(4, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_PPSEL_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_PPSEL_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(seed->size, seed->limb);

	/* unload result if a good candidate is found */
	if (pka_status & PKA_STAT_PPSEL_FAILED)
		ctx->status = Q_ERR_PPSEL_FAILED;
	else
		ctx->status = Q_SUCCESS;

#endif /* QLIP_USE_PKA_HW */

Q_PPSEL_EXIT:
	return ctx->status;
}
