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
#include "q_dh.h"
#include "q_lip_utils.h"

#ifdef QLIP_USE_PKE_HW
#include "q_pke_hw.h"
#endif

#ifdef QLIP_USE_PKA_HW
#include "q_pka_hw.h"
#endif

#ifdef QLIP_USE_GMP
#include "gmp.h"
#endif

/*
 * Both Diffie-Hellman functions are effectively
 * a single modexp operation.These two functions
 * are coded for completeness.
 */

/*
 * q_dh_ss ()
 * Description:  Diffe-Hellman public value.
 * xpub = g ^ x mod p
 * @param ctx		QLIP context pointer
 * @param xpub		pointer to public value q_lint
 * @param dh		pointer to DH parameter structure
 * @param x		pointer to private secret value q_lint
 */
int32_t q_dh_pk(struct q_lip_ctx *ctx,
				   struct q_lint *xpub,
				   struct q_dh_param *dh,
				   struct q_lint *x)
{
	int32_t status = Q_SUCCESS;

#ifdef QLIP_USE_PKE_HW
	uint32_t blen_p, blen_x;
	blen_p = dh->p.size * MACHINE_WD;
	blen_x = x->size * MACHINE_WD;

	xpub->size = dh->p.size;
	q_dh_pk_hw(xpub->limb, blen_p, dh->p.limb,
		dh->g.limb, blen_x, x->limb);
#else

#ifdef QLIP_USE_PKA_HW

	/* Note: The PKA routine supports modulus size up to 2 Kb */

	struct q_mont mont;
	uint32_t pka_status;
	uint32_t lir_p;
	struct opcode sequence[10];

	/* initialize hardware first */
	q_pka_hw_rst();

	status = q_init(ctx, &mont.n, dh->p.alloc);
	status += q_init(ctx, &mont.np, (dh->p.alloc + 1));
	status += q_init(ctx, &mont.rr, (dh->p.alloc + 1));
	if (status != Q_SUCCESS)
		goto Q_DH_PK_EXIT;

	ctx->status = q_mont_init(ctx, &mont, &dh->p);
	if (ctx->status != Q_SUCCESS)
		goto Q_DH_PK_EXIT;

	/* sending command sequence */
	lir_p = q_pka_sel_lir(dh->p.size);

	/* x[0] = g */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0), dh->g.size);
	sequence[0].ptr = dh->g.limb;

	/* x[1] = x */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), x->size);
	sequence[1].ptr = x->limb;

	/* x[2] = p.n */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), mont.n.size);
	sequence[2].ptr = mont.n.limb;

	/* x[3] = p.np */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), mont.np.size);
	sequence[3].ptr = mont.np.limb;

	/* x[4] = p.rr */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), mont.rr.size);
	sequence[4].ptr = mont.rr.limb;

	/* x[0] = x[0] * x[4] mod x[2] (convert to residue) */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 0),
			PKA_LIR(lir_p, 0));
	sequence[5].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 2));
	sequence[5].ptr = NULL;

	/*
	 * xpub = g ^ x mod p
	 * x[4] = x[0] ^ x[1] mod x[2]
	 */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MODEXP, PKA_LIR(lir_p, 4),
				PKA_LIR(lir_p, 0));
	sequence[6].op2 = PACK_OP2(PKA_LIR(lir_p, 1), PKA_LIR(lir_p, 2));
	sequence[6].ptr = NULL;

	/*
	 * convert back
	 * x[0] = 1
	 */
	sequence[7].op1 = PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 0),
			PKA_NULL);
	sequence[7].op2 = PACK_OP2(PKA_NULL, 1);
	sequence[7].ptr = NULL;

	/* x[0] = x[0] * x[4] mod x[2] */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 0),
				PKA_LIR(lir_p, 0));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 2));
	sequence[8].ptr = NULL;

	/* unload result x[0] */
	sequence[9].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 0),
			dh->p.size);
	sequence[9].op2 = 0x5a5a5a5a;
	sequence[9].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_DH_PK_EXIT;
	}
	q_pka_hw_write_sequence(10, sequence);

	q_free(ctx, &mont.rr);
	q_free(ctx, &mont.np);
	q_free(ctx, &mont.n);

	xpub->size = dh->p.size;

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_DH_PK_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_DH_PK_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(xpub->size, xpub->limb);

#else
#ifdef QLIP_USE_GMP
	mpz_t mxp, mp, mg, mx;
	mpz_init(mxp);
	mpz_init(mp);
	mpz_init(mg);
	mpz_init(mx);

	mpz_import(mp, dh->p.size, -1, 4, 0, 0, dh->p.limb);
	mpz_import(mg, dh->g.size, -1, 4, 0, 0, dh->g.limb);
	mpz_import(mx, x->size, -1, 4, 0, 0, x->limb);

	mpz_powm(mxp, mg, mx, mp);
	mpz_export(xpub->limb, &(xpub->size), -1, 4, 0, 0, mxp);
	mpz_clear(mxp);
	mpz_clear(mp);
	mpz_clear(mg);
	mpz_clear(mx);
#else
#ifdef QLIP_MOD_USE_MONT
	struct q_mont mont;
	struct q_lint mg;

	status = q_init(ctx, &mg, dh->g.alloc);
	status += q_init(ctx, &mont.n, dh->p.alloc);
	status += q_init(ctx, &mont.np, (dh->p.alloc + 1));
	status += q_init(ctx, &mont.rr, (dh->p.alloc + 1));

	if (status != Q_SUCCESS)
		goto Q_DH_PK_EXIT;

	q_mont_init(ctx, &mont, &dh->p);
	q_mont_mul(ctx, &mg, &mont.rr, &dh->g, &mont);
	q_modexp_mont(ctx, xpub, &mg, x, &mont);
	q_set_one(&mg);
	q_mont_mul(ctx, xpub, xpub, &mg, &mont);

	q_free(ctx, &mont.rr);
	q_free(ctx, &mont.np);
	q_free(ctx, &mont.n);
	q_free(ctx, &mg);
#else
	q_modexp(ctx, xpub, &dh->g, x, &dh->p);

#endif /* QLIP_MOD_USE_MONT */
#endif /* QLIP_USE_GMP */
#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_PKE_HW */

Q_DH_PK_EXIT:
	return ctx->status;
}
EXPORT_SYMBOL(q_dh_pk);

/*
 * q_dh_ss ()
 * Description:  Diffe-Hellman shared secret.
 * ss = xpub ^ y mod p
 * @param ctx		QLIP context pointer
 * @param ss		q_lint pointer to DH shared secret_lint
 * @param dh		pointer to DH parameter structure
 * @param xpub		q_lint pointer to DH public value_lint
 * @param y		q_lint pointer to DH private secret
 */
int32_t q_dh_ss(struct q_lip_ctx *ctx,
				   struct q_lint *ss,
				   struct q_dh_param *dh,
				   struct q_lint *xpub,
				   struct q_lint *y)
{
	int32_t status = Q_SUCCESS;

#ifdef QLIP_USE_PKE_HW
	uint32_t blen_p;
#else
#ifdef QLIP_USE_PKA_HW
	struct q_mont mont;
	uint32_t pka_status;
	uint32_t lir_p;
	struct opcode sequence[10];
#else
#ifdef QLIP_USE_GMP
	mpz_t mss, mxp, mp, my;
#else
	struct q_mont mont;
	struct q_lint mx;
#endif
#endif
#endif

#ifdef QLIP_USE_PKE_HW
	blen_p = dh->p.size * MACHINE_WD;

	ss->size = dh->p.size;
	q_dh_ss_hw(ss->limb, blen_p, dh->p.limb, xpub->limb, y->limb);
#else
#ifdef QLIP_USE_PKA_HW
	/*
	 * Note: The PKA routine supports modulus size up to 2 Kb
	 * xinitialize hardware first
	 */
	q_pka_hw_rst();

	status = q_init(ctx, &mont.n, dh->p.alloc);
	status += q_init(ctx, &mont.np, (dh->p.alloc + 1));
	status += q_init(ctx, &mont.rr, (dh->p.alloc + 1));
	if (status != Q_SUCCESS)
		goto Q_DH_SS_EXIT;

	ctx->status = q_mont_init(ctx, &mont, &dh->p);
	if (ctx->status != Q_SUCCESS)
		goto Q_DH_SS_EXIT;

	/* sending command sequence */
	lir_p = q_pka_sel_lir(dh->p.size);

	/* x[0] = xpub (public value) */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0), xpub->size);
	sequence[0].ptr = xpub->limb;

	/* x[1] = y (private secret) */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), y->size);
	sequence[1].ptr = y->limb;

	/* x[2] = p.n */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), mont.n.size);
	sequence[2].ptr = mont.n.limb;

	/* x[3] = p.np */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), mont.np.size);
	sequence[3].ptr = mont.np.limb;

	/* x[4] = p.rr */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), mont.rr.size);
	sequence[4].ptr = mont.rr.limb;

	/* x[0] = x[0] * x[4] mod x[2] (convert to residue) */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 0),
			PKA_LIR(lir_p, 0));
	sequence[5].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 2));
	sequence[5].ptr = NULL;

	/* x[4] = x[0] ^ x[1] mod x[2] (ss = xpub^y mod p) */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MODEXP, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 0));
	sequence[6].op2 = PACK_OP2(PKA_LIR(lir_p, 1), PKA_LIR(lir_p, 2));
	sequence[6].ptr = NULL;

	/* convert back x[0] = 1 */
	sequence[7].op1 = PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 0),
		PKA_NULL);
	sequence[7].op2 = PACK_OP2(PKA_NULL, 1);
	sequence[7].ptr = NULL;

	/* x[0] = x[0] * x[4] mod x[2] */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 0),
			PKA_LIR(lir_p, 0));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 2));
	sequence[8].ptr = NULL;

	/* unload result x[0] */
	sequence[9].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 0),
				dh->p.size);
	sequence[9].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_DH_SS_EXIT;
	}
	q_pka_hw_write_sequence(10, sequence);

	q_free(ctx, &mont.rr);
	q_free(ctx, &mont.np);
	q_free(ctx, &mont.n);

	ss->size = dh->p.size;

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_DH_SS_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status  != Q_SUCCESS)
				goto Q_DH_SS_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(ss->size, ss->limb);

#else
#ifdef QLIP_USE_GMP
	mpz_init(mss);
	mpz_init(mxp);
	mpz_init(mp);
	mpz_init(my);

	mpz_import(mxp, xpub->size, -1, 4, 0, 0, xpub->limb);
	mpz_import(mp, dh->p.size, -1, 4, 0, 0, dh->p.limb);
	mpz_import(my, y->size, -1, 4, 0, 0, y->limb);

	mpz_powm(mss, mxp, my, mp);
	mpz_export(ss->limb, &(ss->size), -1, 4, 0, 0, mss);
	mpz_clear(mss);
	mpz_clear(mxp);
	mpz_clear(mp);
	mpz_clear(my);
#else
#ifdef QLIP_MOD_USE_MONT

	status = q_init(ctx, &mx, xpub->alloc);
	status += q_init(ctx, &mont.n, dh->p.alloc);
	status += q_init(ctx, &mont.np, (dh->p.alloc + 1));
	status += q_init(ctx, &mont.rr, (dh->p.alloc + 1));
	if (status != Q_SUCCESS)
		goto Q_DH_SS_EXIT;

	ctx->status = q_mont_init(ctx, &mont, &dh->p);
	if (ctx->status)
		goto Q_DH_SS_EXIT;

	ctx->status = q_mont_mul(ctx, &mx, &mont.rr, xpub, &mont);
	if (ctx->status)
		goto Q_DH_SS_EXIT;
	q_modexp_mont(ctx, ss, &mx, y, &mont);
	q_set_one(&mx);
	q_mont_mul(ctx, ss, ss, &mx, &mont);

	q_free(ctx, &mont.rr);
	q_free(ctx, &mont.np);
	q_free(ctx, &mont.n);
	q_free(ctx, &mx);
#else
	q_modexp(ctx, ss, xpub, y, &dh->p);

#endif /* QLIP_MOD_USE_MONT */
#endif /* QLIP_USE_GMP */
#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_PKE_HW */

Q_DH_SS_EXIT:
	return ctx->status;
}
EXPORT_SYMBOL(q_dh_ss);
