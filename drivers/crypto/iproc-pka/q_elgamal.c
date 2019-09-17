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

#ifdef QLIP_USE_PKE_HW
#include "q_pke_hw.h"
#endif

#ifdef QLIP_USE_PKA_HW
#include "q_pka_hw.h"
#endif

#ifdef QLIP_USE_GMP
#include "gmp.h"
#endif

#include "q_elliptic.h"
#include "q_elgamal.h"

/*
 * q_ecp_lint_2_point()
 * Covert m to M(x,y) where x = m and y^2 = x^3 + a^x + b
 * @param ctx	QLIP context pointer
 * @param M	mapped point
 * @param m	long integer
 * @param curve	q_curve pointer to curve
 */
int32_t q_ecp_lint_2_point(struct q_lip_ctx *ctx,
			  struct q_point *M,
			  struct q_lint *m,
			  struct q_curve *curve)
{
	int32_t status = Q_SUCCESS;

#ifdef QLIP_USE_PKA_HW
	int i;
	struct q_mont mp;
	uint32_t pka_status;
	uint32_t lir_p;
	struct opcode sequence[20];
#endif

#ifdef QLIP_USE_GMP
	mpz_t mm, mp, ma, mb, mt1, mt2;
#endif

	if (q_copy(&M->X, m))
		goto Q_ECP_LINT_2_POINT_EXIT;

#ifdef QLIP_USE_GMP
	mpz_init(mm);
	mpz_init(mp);
	mpz_init(ma);
	mpz_init(mb);
	mpz_init(mt1);
	mpz_init(mt2);

	mpz_import(mm, m->size, -1, 4, 0, 0, m->limb);
	mpz_import(ma, curve->a.size, -1, 4, 0, 0, curve->a.limb);
	mpz_import(mb, curve->b.size, -1, 4, 0, 0, curve->b.limb);
	mpz_import(mp, curve->p.size, -1, 4, 0, 0, curve->p.limb);

	/* x^2 */
	mpz_mul(mt1, mm, mm);
	mpz_mod(mt1, mt1, mp);

	/* x^2 + a */
	mpz_add(mt1, mt1, ma);
	mpz_mod(mt1, mt1, mp);

	/* x*(x^2+a) */
	mpz_mul(mt1, mt1, mm);
	mpz_mod(mt1, mt1, mp);

	/* x^3 + a*x + b */
	mpz_add(mt1, mt1, mb);
	mpz_mod(mt1, mt1, mp);

	/*
	 * modulo square root y = y^((p+1)/4) mod p
	 * if p is congruent to 3 mod 4
	 */
	mpz_set_si(mt2, 1);
	mpz_add(mt2, mp, mt2);
	mpz_cdiv_q_2exp(mt2, mt2, 2);
	mpz_powm(mt2, mt1, mt2, mp);

	mpz_export(M->Y.limb, &(M->Y.size), -1, 4, 0, 0, mt2);

	mpz_clear(mm);
	mpz_clear(mp);
	mpz_clear(ma);
	mpz_clear(mb);
	mpz_clear(mt1);
	mpz_clear(mt2);
#else
#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 20; i++)
		sequence[i].ptr = NULL;

	q_pka_hw_rst();

	status = q_init(ctx, &mp.n, curve->p.alloc);
	status += q_init(ctx, &mp.np, (curve->p.alloc + 1));
	status += q_init(ctx, &mp.rr, (curve->p.alloc + 1));
	if (status != Q_SUCCESS)
		goto Q_ECP_LINT_2_POINT_EXIT;
	status = q_mont_init(ctx, &mp, &curve->p);
	if (status != Q_SUCCESS)
		goto Q_ECP_LINT_2_POINT_EXIT;

	/* create command sequence */
	lir_p = q_pka_sel_lir(curve->p.size);

	/* x[0] = p.n */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0), mp.n.size);
	sequence[0].ptr = mp.n.limb;

	/* x[1] = p.np */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), mp.np.size);
	sequence[1].ptr = mp.np.limb;

	/* x[2] = p.rr */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), mp.rr.size);
	sequence[2].ptr = mp.rr.limb;

	/* x[3] = m */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), m->size);
	sequence[3].ptr = m->limb;

	/* x[4] = a */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), curve->a.size);
	sequence[4].ptr = curve->a.limb;

	/* x[5] = b */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), curve->b.size);
	sequence[5].ptr = curve->b.limb;

	/* convert m to p-residue */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 3),
				PKA_LIR(lir_p, 3));
	sequence[6].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	/* convert ca to p-residue */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 4),
				PKA_LIR(lir_p, 4));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	/* convert b to p-residue */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
				PKA_LIR(lir_p, 5));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	/* m * m mod p  */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 6),
				PKA_LIR(lir_p, 3));
	sequence[9].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* m^2 + a  mod p  */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_p, 6),
				PKA_LIR(lir_p, 6));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 0));

	/* m*(m^2+a)  mod p  */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 6),
				PKA_LIR(lir_p, 6));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_p, 3), PKA_LIR(lir_p, 0));

	/* m^3 + m * a + b  mod p  */
	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_p, 3),
				PKA_LIR(lir_p, 6));
	sequence[12].op2 = PACK_OP2(PKA_LIR(lir_p, 5), PKA_LIR(lir_p, 0));

	/* consts */
	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 4), PKA_NULL);
	sequence[13].op2 = PACK_OP2(PKA_NULL, 1);

	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 5), PKA_NULL);
	sequence[14].op2 = PACK_OP2(PKA_NULL, 4);

	/* P+1 */
	sequence[15].op1 =
		PACK_OP1(0, PKA_OP_LADD, PKA_LIR(lir_p, 6),
				PKA_LIR(lir_p, 0));
	sequence[15].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_NULL);

	/* (P+1)/4 */
	sequence[16].op1 =
		PACK_OP1(0, PKA_OP_LDIV, PKA_LIR(lir_p, 7),
				PKA_LIR(lir_p, 6));
	sequence[16].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 5));

	/* sqrt */
	sequence[17].op1 =
		PACK_OP1(0, PKA_OP_MODEXP, PKA_LIR(lir_p, 5),
				PKA_LIR(lir_p, 3));
	sequence[17].op2 = PACK_OP2(PKA_LIR(lir_p, 7), PKA_LIR(lir_p, 0));

	/* convert y^2 back */
	sequence[18].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
				PKA_LIR(lir_p, 5));
	sequence[18].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 0));

	/* unload y */
	sequence[19].op1 = PACK_OP1(PKA_EOS, PKA_OP_MFLIRI,
							PKA_LIR(lir_p, 5),
							curve->p.size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		status = ctx->q_yield();
		if (status != Q_SUCCESS)
			goto Q_ECP_LINT_2_POINT_EXIT;
	}
	q_pka_hw_write_sequence(20, sequence);

	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			status = Q_ERR_PKA_HW_ERR;
			goto Q_ECP_LINT_2_POINT_EXIT;
		} else {
			status = ctx->q_yield();
			if (status != Q_SUCCESS)
				goto Q_ECP_LINT_2_POINT_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(curve->p.size, M->Y.limb);
	M->Y.size = curve->p.size;
#else
#ifdef QLIP_MOD_USE_MONT
	struct q_mont mp;
	struct q_lint ca;
	struct q_lint t1, t2;

	status = q_init(ctx, &t1, curve->p.alloc + 1);
	status += q_init(ctx, &t2, curve->p.alloc + 1);
	status += q_init(ctx, &mp.n, curve->p.alloc);
	status += q_init(ctx, &mp.np, (curve->p.alloc + 1));
	status += q_init(ctx, &mp.rr, (curve->p.alloc + 1));
	status += q_init(ctx, &ca, curve->a.alloc);
	if (status != Q_SUCCESS)
		goto Q_ECP_LINT_2_POINT_EXIT;

	status = q_mont_init(ctx, &mp, &curve->p);
	if (status)
		goto Q_ECP_LINT_2_POINT_EXIT;

	status = q_copy(&ca, &curve->a);
	if (status)
		goto Q_ECP_LINT_2_POINT_EXIT;

	/* covert */
	q_mont_mul(ctx, &t1, &mp.rr, m, &mp);
	q_mont_mul(ctx, &ca, &mp.rr, &ca, &mp);

	/* m^2 */
	q_mont_mul(ctx, &t2, &t1, &t1, &mp);

	/* m^2+a */
	q_modadd(ctx, &t2, &ca, &t2, &mp.n);

	/* m*(m^2+a) */
	q_mont_mul(ctx, &t1, &t2, &t1, &mp);

	/* m^3 + a*m + b */
	q_mont_mul(ctx, &t2, &mp.rr, &curve->b, &mp);
	q_modadd(ctx, &t1, &t1, &t2, &mp.n);

	/* sqrt */
	q_set_one(&t2);
	q_uadd(&t2, &t2, &mp.n);
	q_div_2pn(&t2, &t2, 2);
	q_modexp_mont(ctx, &t1, &t1, &t2, &mp);

	/* convert back */
	q_set_one(&t2);
	q_mont_mul(ctx, &M->Y, &t1, &t2, &mp);

	q_free(ctx, &ca);
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
	q_free(ctx, &t2);
	q_free(ctx, &t1);
#else

	status = q_init(ctx, &t1, curve->p.alloc + 1);
	status += q_init(ctx, &t2, curve->p.alloc + 1);
	if (status != Q_SUCCESS)
		goto Q_ECP_LINT_2_POINT_EXIT;

	/* m^2 */
	q_modmul(ctx, &t1, m, m, &curve->p);

	/* m^2+a */
	q_modadd(ctx, &t1, &curve->a, &t1, &curve->p);

	/* m*(m^2+a) */
	q_modmul(ctx, &t1, &t1, m, &curve->p);

	/* m^3 + a*m + b */
	q_modadd(ctx, &t1, &t1, &curve->b, &curve->p);

	/* sqrt */
	q_set_one(&t2);
	q_uadd(&t2, &t2, &curve->p);
	q_div_2pn(&t2, &t2, 2);
	q_modexp(ctx, &M->Y, &t1, &t2, &curve->p);

	q_free(ctx, &t2);
	q_free(ctx, &t1);
#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_GMP */

	q_set_one(&M->Z);

Q_ECP_LINT_2_POINT_EXIT:
	ctx->status = status;
	return ctx->status;

}

/*
 * q_ecp_elgamal_enc()
 * @param ctx	QLIP context pointer
 * @param C1	cipher text
 * @param C2	cipher text
 * @param M	plain text
 * @param k	secret
 * @param G	base point
 * @param Q	public value represented by a point
 * @param curve	q_curve pointer to curve
 */
int32_t q_ecp_elgamal_enc(struct q_lip_ctx *ctx,
			 struct q_point *C1,
			 struct q_point *C2,
			 struct q_point *M,
			 struct q_lint *k,
			 struct q_point *G,
			 struct q_point *Q,
			 struct q_curve *curve)
{
	int32_t status = Q_SUCCESS;
	struct q_point R;

	status = q_init(ctx, &R.X, curve->p.alloc);
	status += q_init(ctx, &R.Y, curve->p.alloc);
	status += q_init(ctx, &R.Z, curve->p.alloc);
	if (status != Q_SUCCESS)
		goto Q_ECP_ELGAMAL_ENC_EXIT;

	/* compute point C1 */
	status = q_ecp_pt_mul(ctx, C1, G, k, curve);
	if (status != Q_SUCCESS)
		goto Q_ECP_ELGAMAL_ENC_EXIT;

	/* compute point R */
	status = q_ecp_pt_mul(ctx, &R, Q, k, curve);
	if (status != Q_SUCCESS)
		goto Q_ECP_ELGAMAL_ENC_EXIT;

	/* compute point C2 = R + M */
	status = q_ecp_pt_add(ctx, C2, &R, M, curve);
	if (status != Q_SUCCESS)
		goto Q_ECP_ELGAMAL_ENC_EXIT;

	q_free(ctx, &R.Z);
	q_free(ctx, &R.Y);
	q_free(ctx, &R.X);

Q_ECP_ELGAMAL_ENC_EXIT:
	ctx->status = status;
	return ctx->status;
}

/*
 * q_ecp_elgamal_enc()
 * @param ctx	QLIP context pointer
 * @param M	plain text
 * @param C1	cipher text
 * @param C2	cipher text
 * @param d	secret
 * @param curve	q_curve pointer to curve
 */
int32_t q_ecp_elgamal_dec(struct q_lip_ctx *ctx,
			 struct q_point *M,
			 struct q_point *C1,
			 struct q_point *C2,
			 struct q_lint *d,
			 struct q_curve *curve)
{
	int32_t status = Q_SUCCESS;
	struct q_point R;

	status = q_init(ctx, &R.X, curve->p.alloc);
	status += q_init(ctx, &R.Y, curve->p.alloc);
	status += q_init(ctx, &R.Z, curve->p.alloc);
	if (status != Q_SUCCESS)
		goto Q_ECP_ELGAMAL_DEC_EXIT;

	/* compute point R */
	status = q_ecp_pt_mul(ctx, &R, C1, d, curve);
	if (status != Q_SUCCESS)
		goto Q_ECP_ELGAMAL_DEC_EXIT;

	/* compute point M = C2 - R */
	status = q_ecp_pt_sub(ctx, M, C2, &R, curve);
	if (status != Q_SUCCESS)
		goto Q_ECP_ELGAMAL_DEC_EXIT;

	q_free(ctx, &R.Z);
	q_free(ctx, &R.Y);
	q_free(ctx, &R.X);

Q_ECP_ELGAMAL_DEC_EXIT:
	ctx->status = status;

	return ctx->status;
}
