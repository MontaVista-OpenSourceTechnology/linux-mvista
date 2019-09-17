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

#ifdef QLIP_USE_PKA_HW
#include "q_pka_hw.h"
#endif

#ifdef QLIP_USE_GMP
#include "gmp.h"
#endif

/*
 * q_mod_2pn ()
 * Description: z = a mod n, where n = 2^k
 */
int32_t q_mod_2pn(struct q_lint *z, struct q_lint *a, uint32_t bits)
{
	int32_t status = Q_SUCCESS;
	int i;
	int words;
	uint32_t *zp;
	uint32_t *ap;

	words = (bits >> BITS_FOR_MACHINE_WD);

	if (a->size < words) {
		status = q_copy(z, a);
		goto Q_MOD_2PN_EXIT;
	}

	if (z->alloc < words + 1) {
		q_abort(__FILE__, __LINE__,
				"QLIP: mod_2pn insufficient memory for the result!");

		status = Q_ERR_DST_OVERFLOW;
		goto Q_MOD_2PN_EXIT;
	}

	z->size = words;
	zp = z->limb;
	ap = a->limb;

	/* the operation should still work when z == a */
	for (i = 0; i < words; i++)
		zp[i] = ap[i];

	words = bits - (words << BITS_FOR_MACHINE_WD);

	if (words) {
		zp[i] = ap[i] & (UINT_MASK >> (MACHINE_WD - words));
		z->size++;
	}

Q_MOD_2PN_EXIT:
	return status;
}

#ifndef REDUCED_RELEASE_CODE_SIZE
/*
 * q_mod_partial ()
 * z = a mod n
 * Description: This function is valid only when the MSB of a and n
 * are the same.
 */
int32_t q_mod_partial(struct q_lint *z, struct q_lint *a, struct q_lint *n)
{
	int32_t status = Q_SUCCESS;
	int res;

	if (a->size != n->size) {
		q_abort(__FILE__, __LINE__, "QLIP: Simple mod size mismatch!");

		status = Q_ERR_DST_OVERFLOW;
		goto Q_MOD_PARTIAL_EXIT;
	}

	res = q_ucmp(a, n);

	/* if a > n, perform substraction */
	if (res)
		status = q_usub(z, a, n);
	else
		status = q_copy(z, a);

Q_MOD_PARTIAL_EXIT:
	return status;
}
#endif /* REDUCED_RELEASE_CODE_SIZE */

/*
 * q_mod ()
 * Description:  z = a mod n
 */
int32_t q_mod(struct q_lip_ctx *ctx, struct q_lint *z,
			struct q_lint *a, struct q_lint *n)
{
#ifdef QLIP_USE_GMP
	mpz_t ma, mn, mz;
#endif

	uint32_t pka_status;
	struct opcode sequence[4];
	uint32_t lir_n;
	uint32_t lir_a;

#ifdef QLIP_USE_GMP
	mpz_init(ma);
	mpz_init(mn);
	mpz_init(mz);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_import(mn, n->size, -1, 4, 0, 0, n->limb);

	mpz_mod(mz, ma, mn);
	mpz_export(z->limb, &(z->size), -1, 4, 0, 0, mz);

	mpz_clear(ma);
	mpz_clear(mn);
	mpz_clear(mz);
#else /* QLIP_USE_GMP */
#ifdef QLIP_USE_PKA_HW
	/*
	 * Limitation: n is limited to 2K bits
	 * a is limited to 4K bits
	 */
	lir_n = PKA_LIR_H;
	lir_a = PKA_LIR_J;

	/* J[0] = a */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_a, 0), a->size);
	sequence[0].ptr = a->limb;

	/* H[2] = n */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 2), n->size);
	sequence[1].ptr = n->limb;

	/* H[0] = J[0] mod H[2] */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MODREM, PKA_LIR(lir_n, 0),
			PKA_LIR(lir_a, 0));
	sequence[2].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_n, 2));
	sequence[2].ptr = NULL;

	/* unload H[0] */
	sequence[3].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 0), n->size);
	sequence[3].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_MOD_EXIT;
	}
	q_pka_hw_write_sequence(4, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_MOD_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_MOD_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(n->size, z->limb);
	z->size = n->size;

#else /* QLIP_USE_PKA_HW */
	/* call Tao's function */
	int a_neg;
	a_neg = a->neg;
	a->neg = 0;

	q_fdivrem(ctx, a, n, NULL, z);
	a->neg = a_neg;

#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_GMP */

Q_MOD_EXIT:
	return ctx->status;
}

/*
 * q_mod_sw ()
 * Description:  z = a mod n
 * create a software-only version for 4K RSA
 */
int32_t q_mod_sw(struct q_lip_ctx *ctx, struct q_lint *z,
				struct q_lint *a, struct q_lint *n)
{
	int a_neg;

	a_neg = a->neg;
	a->neg = 0;

	q_fdivrem(ctx, a, n, NULL, z);
	a->neg = a_neg;

	return ctx->status;
}
