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
#include "q_lip_aux.h"
#include "q_lip_utils.h"

/*
 * This function implements both long division and modulo reduction.
 * Inputs:
 * opa -- dividend
 * opn -- divisor
 * Output:
 * quotient -- floor (opa / opn)
 * remainder -- opa % opn
 * Description:
 * This function implements Alg 4.3.1D in Knuth's Art of Computer
 * Programming Vol 2 (3rd Ed). The digit size is 16 bits.
 */
void get_digit(int *opa,	/* long integer LSD pointer */
	   int index,	/* 16-bit digit index (LSD = 0) */
	   int *value)
{
	/* 16-bit digit value */
	unsigned int word;

	word = *(opa + (index >> 1));
	*value = (index & 0x1) ? (word >> 16) : (word & 0xffff);
	if (index < 0)
		*value = 0;
}

void set_digit(int *opa,	/* long integer LSD pointer */
	   int index,	/* 16-bit digit index (LSD = 0) */
	   int value)	/* 16-bit digit value */
{
	int word;

	value &= 0xffff;
	word = *(opa + (index >> 1));
	word &= (index & 0x1) ? 0xffff : 0xffff0000;
	word |= (index & 0x1) ? (value << 16) : value;
	*(opa + (index >> 1)) = word;
}

int32_t q_fdivrem(struct q_lip_ctx *ctx,
		 struct q_lint *opa,
		 struct q_lint *opn,
		 struct q_lint *quotient,
		 struct q_lint *remainder)
{
	int32_t status = Q_SUCCESS;

	int dcnt;
	int opn_msd0, opn_msd1;
	int opa_msd0, opa_msd1, opa_msd2;
	int opn_norm_factor = 0;
	int mod_size = 0;
	int q_hat, r_hat;
	int i;

	struct q_lint tmp_lint1;
	struct q_lint tmp_lint2;
	struct q_lint norm_opa;
	struct q_lint norm_opn;

	status = q_init(ctx, &tmp_lint1, 2 + opa->alloc);
	status += q_init(ctx, &tmp_lint2, 2 + opa->alloc);
	status += q_init(ctx, &norm_opa, 2 + opa->alloc);
	status += q_init(ctx, &norm_opn, 2 + opn->alloc);

	if (status != Q_SUCCESS)
		goto Q_FDIVREM_EXIT;

	/* D1: Normalize */
	dcnt = (opn->size << 1) - 1;
	for (; dcnt >= 0; dcnt--) {
		get_digit((int *)opn->limb, dcnt, &opn_msd0);
		if (opn_msd0 == 0)
			continue;
		opn_norm_factor = opn_msd0;
		for (i = 0; !(opn_norm_factor & 0x8000); i++)
			opn_norm_factor <<= 1;
		opn_norm_factor = i;
		mod_size = dcnt;
		break;
	}

	tmp_lint1.limb[0] = (1 << opn_norm_factor);
	tmp_lint1.size = 1;
	q_mul_sw(ctx, &norm_opa, opa, &tmp_lint1);
	q_mul_sw(ctx, &norm_opn, opn, &tmp_lint1);
	get_digit((int *)norm_opn.limb, mod_size, &opn_msd0);
	get_digit((int *)norm_opn.limb, mod_size - 1, &opn_msd1);
	if (remainder != NULL)
		remainder->size = opn->size;

	/* D2: Initialize dcnt */
	dcnt = (norm_opa.size << 1) - 1;
	if (dcnt <= mod_size)
		dcnt = mod_size + 1;
	for (; dcnt > mod_size + 1; dcnt--) {
		get_digit((int *)opa->limb, dcnt, &opa_msd0);
		if (opa_msd0 == 0)
			continue;
		break;
	}
	dcnt++;
	if (quotient != NULL) {
		quotient->size = ((dcnt - mod_size + 1) >> 1);
		for (i = 0; i < quotient->alloc; i++)
			*(quotient->limb + i) = 0;
	}
	if (remainder != NULL) {
		for (i = 0; i < remainder->alloc; i++)
			*(remainder->limb + i) = 0;
		remainder->size = opn->size;
	}

	/* D7: Loop on dcnt */
	for (; dcnt > mod_size; dcnt--) {

		/* D3: Calculate estimated quotient q_hat */
		get_digit((int *)norm_opa.limb, dcnt, &opa_msd0);
		get_digit((int *)norm_opa.limb, dcnt - 1, &opa_msd1);
		get_digit((int *)norm_opa.limb, dcnt - 2, &opa_msd2);

		q_hat = ((unsigned)((opa_msd0 << 16) + opa_msd1)) / opn_msd0;
		r_hat =
			((unsigned)((opa_msd0 << 16) + opa_msd1)) -
			q_hat * opn_msd0;
		for (; r_hat < (1 << 16);) {
			if ((q_hat == (1 << 16)) ||
				((unsigned)(q_hat * opn_msd1) >
				 (unsigned)((r_hat << 16) + opa_msd2))) {
				q_hat--;
				r_hat += opn_msd0;
			} else
				break;
		}

		/* D4: Multiply and subtract */
		tmp_lint1.limb[0] = q_hat;
		tmp_lint1.size = 1;
		q_mul_sw(ctx, &tmp_lint2, &norm_opn, &tmp_lint1);
		q_mul_2pn(&tmp_lint1, &tmp_lint2, (dcnt - mod_size - 1) << 4);
		q_sub(&norm_opa, &norm_opa, &tmp_lint1);

		/* D5: Test remainder */
		if (norm_opa.neg) {

			/* D6: Add back */
			q_hat--;
			q_mul_2pn(&tmp_lint2, &norm_opn,
					  (dcnt - mod_size - 1) << 4);
			q_add(&norm_opa, &norm_opa, &tmp_lint2);
		}
		if (quotient != NULL)
			set_digit((int *)quotient->limb, dcnt - mod_size - 1,
					  q_hat);
	}

	/* D8: Unnormalize */
	if (remainder != NULL)
		q_div_2pn(remainder, &norm_opa, opn_norm_factor);

	q_free(ctx, &norm_opn);
	q_free(ctx, &norm_opa);
	q_free(ctx, &tmp_lint2);
	q_free(ctx, &tmp_lint1);

Q_FDIVREM_EXIT:
	return ctx->status;
}
