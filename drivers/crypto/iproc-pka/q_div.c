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

#ifdef QLIP_USE_GMP
#include "gmp.h"
#endif

/*
 * q_div ()
 * z = floor (a / b)
 * Description: This function returns the quotient.
 */

int32_t q_div(struct q_lip_ctx *ctx, /* QLIP context pointer */
			 struct q_lint *z, /* q_lint pointer to result z */
			 struct q_lint *a, /* q_lint pointer to source a */
			 struct q_lint *b) /* q_lint pointer to source b */
{
	int i;

#ifdef QLIP_USE_GMP
	mpz_t mz, ma, mb;
#else
	int a_neg, b_neg;
#endif

	if (q_is_zero(b)) {
		q_abort(__FILE__, __LINE__, "QLIP: q_div divide by zero!");

		ctx->status = Q_ERR_DIV_BY_0;
		goto Q_DIV_EXIT;
	}

	if (!a->size || a->size < b->size) {
		q_set_zero(z);
		goto Q_DIV_EXIT;
	}

	if (a->size == b->size) {
		/*to add leading 1 detection*/
		i = 0;
	}
#ifdef QLIP_USE_GMP
	mpz_init(mz);
	mpz_init(ma);
	mpz_init(mb);

	mpz_import(mb, b->size, -1, 4, 0, 0, b->limb);
	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_div(mz, ma, mb);

	mpz_export(z->limb, &(z->size), -1, 4, 0, 0, mz);

	mpz_clear(mz);
	mpz_clear(ma);
	mpz_clear(mb);
#else
	/* call Tao's function */
	a_neg = a->neg;
	b_neg = b->neg;

	q_fdivrem(ctx, a, b, z, NULL);

	z->neg = a_neg ^ b_neg;

	Q_NORMALIZE(z->limb, z->size);

#endif

Q_DIV_EXIT:
	return ctx->status;
}

/*
 * q_div_2pn ()
 * z = (a / b) where b is 2^n
 * Description: This function is a bit shift operation.
 */
int32_t q_div_2pn(struct q_lint *z, struct q_lint *a, uint32_t bits)
{
	int i;
	int shift;
	uint32_t *zp;
	uint32_t *ap;

	shift = (bits >> BITS_FOR_MACHINE_WD);

	if (shift >= a->size) {
		q_set_zero(z);
		goto Q_DIV_2PN_EXIT;
	}

	zp = z->limb;
	ap = a->limb;

	for (i = 0; i < (a->size - shift); i++)
		zp[i] = ap[i + shift];
	z->size = a->size - shift;

	shift = bits - (shift << BITS_FOR_MACHINE_WD);
	if (shift) {
		for (i = 0; i < (z->size - 1); i++)
			zp[i] =
				(zp[i] >> shift) |
			((zp[i + 1] & (UINT_MASK >> (MACHINE_WD - shift)))
				 << (MACHINE_WD - shift));
			zp[i] = (zp[i] >> shift);
	}

	Q_NORMALIZE(z->limb, z->size);

Q_DIV_2PN_EXIT:
	return Q_SUCCESS;
}

/*
 * q_mod_div2 ()
 * z = a / 2 mod n
 * Description: This function requires the modulus n be an odd
 * number. The algorithm is as follows.
 * If (a == even) z = a / 2;
 * Else z = (a + n) / 2;
 * Return z;
 */
int32_t q_mod_div2(struct q_lip_ctx *ctx,
					  struct q_lint *z,
					  struct q_lint *a,
					  struct q_lint *n)
{
	struct q_lint tmp;
	uint32_t size;

	size = a->size + 1;
	if (q_init(ctx, &tmp, size))
		goto Q_MOD_DIV2_EXIT;

	q_copy(&tmp, a);
	if (a->limb[0] & 0x00000001)
		q_uadd(&tmp, &tmp, n);
	q_div_2pn(&tmp, &tmp, 1);

	ctx->status = q_copy(z, &tmp);
	if (ctx->status != Q_SUCCESS)
		goto Q_MOD_DIV2_EXIT;

	q_free(ctx, &tmp);

Q_MOD_DIV2_EXIT:
	return ctx->status;
}
