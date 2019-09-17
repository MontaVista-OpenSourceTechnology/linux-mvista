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
/*
 * q_ecp_ecdsa_sign ()
 * @param ctx		QLIP context pointer
 * @param rs		q_signature point to rs
 * @param G		q_point pointer to G
 * @param curve	q_curve pointer to curve
 * @param d		q_lint pointer to d
 * @param h		q_lint pointer to h
 * @param k		q_lint pointer to k
 */
int32_t q_ecp_ecdsa_sign(struct q_lip_ctx *ctx,
							struct q_signature *rs,
							struct q_point *G,
							struct q_curve *curve,
							struct q_lint *d,
							struct q_lint *h,
							struct q_lint *k)
{
	int32_t status = Q_SUCCESS;

	uint32_t size;
	struct q_point R;
	struct q_lint kinv;
	struct q_lint tmp;

#ifdef QLIP_USE_PKE_HW
	uint32_t blen_n;
	struct q_lint tmp1;
#endif

#ifdef QLIP_USE_PKA_HW
	int i;
	struct q_mont mont;
	uint32_t pka_status;
	uint32_t lir_n;
	struct opcode sequence[21];
#endif

#ifdef QLIP_USE_GMP
	mpz_t mk, mx, mn, md, mh, mr, ms;
#endif

	size = curve->n.alloc;
	status = q_init(ctx, &R.X, size);
	status += q_init(ctx, &R.Y, size);
	status += q_init(ctx, &R.Z, size);
	status += q_init(ctx, &kinv, size);
	status += q_init(ctx, &tmp, size + 1);
	if (status != Q_SUCCESS)
		goto Q_ECP_ECDSA_SIGN_EXIT;

	/* compute point R */
	if (q_ecp_pt_mul(ctx, &R, G, k, curve))
		goto Q_ECP_ECDSA_SIGN_EXIT;

#ifdef QLIP_USE_PKE_HW
	blen_n = curve->n.size * MACHINE_WD;
	q_init(ctx, &tmp1, size + 1);

	kinv.size = size;
	tmp.size = size;
	rs->r.size = size;
	rs->s.size = size;

	/* compute the inverse of k */
	q_modinv_hw(kinv.limb, blen_n, curve->n.limb, k->limb);

	/*
	 * compute signature
	 * the following part is still flaky because
	 * not all parameters are the same size
	 */
	q_modrem_hw(rs->r.limb, blen_n, curve->n.limb, R.X.limb);
	q_modmul_hw(tmp.limb, blen_n, curve->n.limb, rs->r.limb, d->limb);

	q_copy(&tmp1, h);
	tmp1.size = size;
	q_modadd_hw(tmp.limb, blen_n, curve->n.limb, tmp.limb, tmp1.limb);
	q_modmul_hw(rs->s.limb, blen_n, curve->n.limb, tmp.limb, kinv.limb);

	q_free(ctx, &tmp1);
#else
#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 21; i++)
		sequence[i].ptr = NULL;

	q_pka_hw_rst();

	status = q_init(ctx, &mont.n, curve->n.alloc);
	status += q_init(ctx, &mont.np, (curve->n.alloc + 1));
	status += q_init(ctx, &mont.rr, (curve->n.alloc + 1));
	if (status != Q_SUCCESS)
		goto Q_ECP_ECDSA_SIGN_EXIT;

	if (q_mont_init(ctx, &mont, &curve->n))
		goto Q_ECP_ECDSA_SIGN_EXIT;

	rs->r.size = size;
	rs->s.size = size;

	/* create command sequence */
	lir_n = q_pka_sel_lir(curve->n.size);

	/* x[0] = n.n */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 0), mont.n.size);
	sequence[0].ptr = mont.n.limb;

	/* x[1] = n.np */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 1), mont.np.size);
	sequence[1].ptr = mont.np.limb;

	/* x[2] = n.rr */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 2), mont.rr.size);
	sequence[2].ptr = mont.rr.limb;

	/* x[3] = k (random number) */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 3), k->size);
	sequence[3].ptr = k->limb;

	/* x[4] = d (private key) */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 4), d->size);
	sequence[4].ptr = d->limb;

	/* x[5] = h (hash) */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 5), h->size);
	sequence[5].ptr = h->limb;

	/* x[6] = R.X */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 6), R.X.size);
	sequence[6].ptr = R.X.limb;

	/* compute the inverse of k */
	sequence[7].op1 = PACK_OP1(0, PKA_OP_SLIR,
						PKA_LIR(lir_n, 7), PKA_NULL);
	sequence[7].op2 = PACK_OP2(PKA_NULL, 2);

	/*
	 * Convert k to n-residue domain
	 * x[9] = x[3] * x[2] mod x[0]
	 */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 9),
			PKA_LIR(lir_n, 3));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * compute the inverse of k
	 * x[8] = x[9] ^ (x[0] - 2) mod x[0]
	 */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODINV, PKA_LIR(lir_n, 8),
			PKA_LIR(lir_n, 9));
	sequence[9].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_n, 0));

	/*
	 * convert d to n-residue domain
	 * x[10] = x[4] * x[2] mod x[0]
	 */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 10),
			PKA_LIR(lir_n, 4));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert h to n-residue domain
	 * x[11] = x[5] * x[2] mod x[0]
	 */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 11),
			PKA_LIR(lir_n, 5));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * compute r (x[3])
	 * x[3] = x[6] mod x[0]
	 */
	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODREM, PKA_LIR(lir_n, 3),
			PKA_LIR(lir_n, 6));
	sequence[12].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_n, 0));

	/*
	 * convert r to n-residue domain
	 * x[7] = x[3] * x[2] mod x[0]
	 */
	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 7),
			PKA_LIR(lir_n, 3));
	sequence[13].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * r * d mod n
	 * x[4] = x[7] * x[10] mod x[0]
	 */
	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 4),
			PKA_LIR(lir_n, 7));
	sequence[14].op2 = PACK_OP2(PKA_LIR(lir_n, 10), PKA_LIR(lir_n, 0));

	/*
	 * (h + r * d) mod n
	 * x[5] = x[4] + x[11] mod x[0]
	 */
	sequence[15].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_n, 5),
			PKA_LIR(lir_n, 4));
	sequence[15].op2 = PACK_OP2(PKA_LIR(lir_n, 11), PKA_LIR(lir_n, 0));

	/* x[6] = x[5] * x[8] mod x[0] */
	sequence[16].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 6),
			PKA_LIR(lir_n, 5));
	sequence[16].op2 = PACK_OP2(PKA_LIR(lir_n, 8), PKA_LIR(lir_n, 0));

	sequence[17].op1 =
		PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_n, 7), PKA_NULL);
	sequence[17].op2 = PACK_OP2(PKA_NULL, 1);

	sequence[18].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 4),
			PKA_LIR(lir_n, 6));
	sequence[18].op2 = PACK_OP2(PKA_LIR(lir_n, 7), PKA_LIR(lir_n, 0));

	sequence[19].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 3), rs->r.size);

	sequence[20].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 4), rs->s.size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_ECP_ECDSA_SIGN_EXIT;
	}
	q_pka_hw_write_sequence(21, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {

		/*
		 * While we wait for the the first unload opcode,
		 * we want to monitor the CMD_ERR bit in the status
		 * register, as the math opcodes before the first
		 * unload opcode may trigger PKA HW	related error.
		 * We do not need to monitor the CMD_ERR for the
		 * subsequent unload opcode.
		*/

		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_ECP_ECDSA_SIGN_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_ECP_ECDSA_SIGN_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(rs->r.size, rs->r.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(rs->s.size, rs->s.limb);

	q_free(ctx, &mont.rr);
	q_free(ctx, &mont.np);
	q_free(ctx, &mont.n);
#else
#ifdef QLIP_USE_GMP
	mpz_init(mk);
	mpz_init(mx);
	mpz_init(mn);
	mpz_init(md);
	mpz_init(mh);
	mpz_init(mr);
	mpz_init(ms);

	mpz_import(mk, k->size, -1, 4, 0, 0, k->limb);
	mpz_import(md, d->size, -1, 4, 0, 0, d->limb);
	mpz_import(mh, h->size, -1, 4, 0, 0, h->limb);
	mpz_import(mx, R.X.size, -1, 4, 0, 0, R.X.limb);
	mpz_import(mn, curve->n.size, -1, 4, 0, 0, curve->n.limb);

	/* compute the inverse of k */
	mpz_invert(mk, mk, mn);

	/* compute signature */
	mpz_mod(mx, mx, mn);
	mpz_export(rs->r.limb, &(rs->r.size), -1, 4, 0, 0, mx);

	mpz_mul(mx, mx, md);
	mpz_mod(mx, mx, mn);

	mpz_add(mx, mx, mh);
	mpz_mod(mx, mx, mn);

	mpz_mul(mx, mx, mk);
	mpz_mod(mx, mx, mn);
	mpz_export(rs->s.limb, &(rs->s.size), -1, 4, 0, 0, mx);

	mpz_clear(mk);
	mpz_clear(mx);
	mpz_clear(mn);
	mpz_clear(md);
	mpz_clear(mh);
	mpz_clear(mr);
	mpz_clear(ms);
#else
#ifdef QLIP_MOD_USE_MONT
	struct q_mont mont;
	struct q_lint mt, md, mh, mr, ms;
	q_init(ctx, &mt, curve->n.alloc);
	q_init(ctx, &md, curve->n.alloc + 1);
	q_init(ctx, &mh, curve->n.alloc);
	q_init(ctx, &mr, curve->n.alloc);
	q_init(ctx, &ms, curve->n.alloc);
	q_init(ctx, &mont.n, curve->n.alloc);
	q_init(ctx, &mont.np, (curve->n.alloc + 1));
	q_init(ctx, &mont.rr, (curve->n.alloc + 1));

	q_mont_init(ctx, &mont, &curve->n);

	/* compute the inverse of k */
	md.size = 1;
	md.limb[0] = 2L;
	q_usub(&md, &curve->n, &md);
	q_mont_mul(ctx, &mt, &mont.rr, k, &mont);
	q_modexp_mont(ctx, &mt, &mt, &md, &mont);

	q_mont_mul(ctx, &md, &mont.rr, d, &mont);
	q_mont_mul(ctx, &mh, &mont.rr, h, &mont);

	/* compute signature */
	q_mod(ctx, &rs->r, &R.X, &curve->n);
	q_mont_mul(ctx, &mr, &mont.rr, &rs->r, &mont);
	q_mont_mul(ctx, &mr, &mr, &md, &mont);
	q_modadd(ctx, &md, &mr, &mh, &mont.n);
	q_mont_mul(ctx, &md, &md, &mt, &mont);
	q_set_one(&mt);
	q_mont_mul(ctx, &rs->s, &md, &mt, &mont);

	q_free(ctx, &mont.rr);
	q_free(ctx, &mont.np);
	q_free(ctx, &mont.n);
	q_free(ctx, &ms);
	q_free(ctx, &mr);
	q_free(ctx, &mh);
	q_free(ctx, &md);
	q_free(ctx, &mt);
#else
	/* compute the inverse of k */
	q_modpinv(ctx, &kinv, k, &curve->n);

	/* compute signature */
	q_mod(ctx, &rs->r, &R.X, &curve->n);
	q_modmul(ctx, &tmp, &rs->r, d, &curve->n);
	q_modadd(ctx, &tmp, &tmp, h, &curve->n);
	q_modmul(ctx, &rs->s, &tmp, &kinv, &curve->n);
#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_GMP */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_PKE_HW */

	q_free(ctx, &tmp);
	q_free(ctx, &kinv);
	q_free(ctx, &R.Z);
	q_free(ctx, &R.Y);
	q_free(ctx, &R.X);

Q_ECP_ECDSA_SIGN_EXIT:
	return ctx->status;
}
EXPORT_SYMBOL(q_ecp_ecdsa_sign);

/*
 * q_ecp_ecdsa_verify ()
 * @param ctx		QLIP context pointer
 * @param v		q_lint pointer to v
 * @param G		q_lint pointer to G
 * @param curve	q_curve pointer to curve
 * @param Q		q_point pointer to Q
 * @param h		q_lint pointer to h
 * @param rs		q_signature point to rs
 */
int32_t q_ecp_ecdsa_verify(struct q_lip_ctx *ctx,
						  struct q_lint *v,
						  struct q_point *G,
						  struct q_curve *curve,
						  struct q_point *Q,
						  struct q_lint *h,
						  struct q_signature *rs)
{
	int32_t status = Q_SUCCESS;

	int i;
	uint32_t size;
	struct q_point R1, R2, GG, QQ, g_minus, q_minus;
	struct q_lint u1, u2, ca;
	struct q_lint tmp[6];

#ifdef QLIP_USE_PKE_HW
	uint32_t blen_n;
#endif

#ifdef QLIP_USE_PKA_HW
	struct q_mont mp;
	uint32_t pka_status;
	uint32_t lir_n;
	struct opcode sequence[21];
#endif

#ifdef QLIP_USE_GMP
	mpz_t mx, mu1, mu2, mh, ms, mr, mn;
#endif

	size = curve->p.alloc;
	status = q_init(ctx, &u1, size + 1);
	status += q_init(ctx, &u2, size + 1);
	status += q_init(ctx, &R1.X, size);
	status += q_init(ctx, &R1.Y, size);
	status += q_init(ctx, &R1.Z, size);
	status += q_init(ctx, &R2.X, size);
	status += q_init(ctx, &R2.Y, size);
	status += q_init(ctx, &R2.Z, size);
	status += q_init(ctx, &GG.X, size);
	status += q_init(ctx, &GG.Y, size);
	status += q_init(ctx, &GG.Z, size);
	status += q_init(ctx, &QQ.X, size);
	status += q_init(ctx, &QQ.Y, size);
	status += q_init(ctx, &QQ.Z, size);
	status += q_init(ctx, &g_minus.X, size);
	status += q_init(ctx, &g_minus.Y, size);
	status += q_init(ctx, &g_minus.Z, size);
	status += q_init(ctx, &q_minus.X, size);
	status += q_init(ctx, &q_minus.Y, size);
	status += q_init(ctx, &q_minus.Z, size);
	status += q_init(ctx, &ca, size);

	for (i = 0; i < 6; i++)
		status += q_init(ctx, &tmp[i], size + 1);
	if (status != Q_SUCCESS)
		goto Q_ECP_ECDSA_VERIFY_EXIT;

	ctx->status = q_pt_copy(&GG, G);
	if (ctx->status != Q_SUCCESS)
		goto Q_ECP_ECDSA_VERIFY_EXIT;
	ctx->status = q_pt_copy(&g_minus, G);
	if (ctx->status != Q_SUCCESS)
		goto Q_ECP_ECDSA_VERIFY_EXIT;
	ctx->status = q_usub(&g_minus.Y, &curve->p, &g_minus.Y);
	if (ctx->status != Q_SUCCESS)
		goto Q_ECP_ECDSA_VERIFY_EXIT;
	ctx->status = q_pt_copy(&QQ, Q);
	if (ctx->status != Q_SUCCESS)
		goto Q_ECP_ECDSA_VERIFY_EXIT;
	ctx->status = q_pt_copy(&q_minus, Q);
	if (ctx->status != Q_SUCCESS)
		goto Q_ECP_ECDSA_VERIFY_EXIT;
	ctx->status = q_usub(&q_minus.Y, &curve->p, &q_minus.Y);
	if (ctx->status != Q_SUCCESS)
		goto Q_ECP_ECDSA_VERIFY_EXIT;

	u1.size = size;
	u2.size = size;
	q_copy(&ca, &curve->a);

#ifdef QLIP_USE_PKE_HW
	blen_n = curve->n.size * MACHINE_WD;

	/* compute the inverse of s */
	q_modinv_hw(u2.limb, blen_n, curve->n.limb, rs->s.limb);

	/*
	 * compute u1 and u2
	 * same concern as the signing function,
	 * h is a different size parameter
	 */
	q_copy(&tmp[0], h);
	tmp[0].size = size;

	q_modmul_hw(u1.limb, blen_n, curve->n.limb, u2.limb, tmp[0].limb);
	q_modmul_hw(u2.limb, blen_n, curve->n.limb, u2.limb, rs->r.limb);
#else
#ifdef QLIP_USE_PKA_HW
	/* initialize hardware first */
	for (i = 0; i < 21; i++)
		sequence[i].ptr = NULL;

	q_pka_hw_rst();

	ctx->status = q_init(ctx, &mp.n, curve->n.alloc);
	ctx->status += q_init(ctx, &mp.np, curve->n.alloc + 1);
	ctx->status += q_init(ctx, &mp.rr, curve->n.alloc + 1);
	if (ctx->status != Q_SUCCESS) {
		ctx->status = Q_ERR_CTX_OVERFLOW;
		goto Q_ECP_ECDSA_VERIFY_EXIT;
	}

	ctx->status = q_mont_init(ctx, &mp, &curve->n);
	if (ctx->status != Q_SUCCESS)
		goto Q_ECP_ECDSA_VERIFY_EXIT;

	/* create command sequence */
	lir_n = q_pka_sel_lir(curve->n.size);

	/* x[0] = n.n */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 0), mp.n.size);
	sequence[0].ptr = mp.n.limb;

	/* x[1] = n.np */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 1), mp.np.size);
	sequence[1].ptr = mp.np.limb;

	/* x[2] = n.rr */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 2), mp.rr.size);
	sequence[2].ptr = mp.rr.limb;

	/* n[3] = s */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 3), rs->s.size);
	sequence[3].ptr = rs->s.limb;

	/* n[4] = r */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 4), rs->r.size);
	sequence[4].ptr = rs->r.limb;

	/* n[5] = h (hash) */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 5), h->size);
	sequence[5].ptr = h->limb;

	/* This opcode should be removed */
	sequence[6].op1 = PACK_OP1(0, PKA_OP_SLIR,
						PKA_LIR(lir_n, 7), PKA_NULL);
	sequence[6].op2 = PACK_OP2(PKA_NULL, 2);

	/*
	 * compute the inverse of s
	 * convert s to n-residue domain
	 * x[6] = x[3] * x[2] mod x[0]
	 */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 6),
			PKA_LIR(lir_n, 3));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/* x[3] = x[6] ^ (x[0] - 2) mod x[0] */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODINV, PKA_LIR(lir_n, 3),
			PKA_LIR(lir_n, 6));
	sequence[8].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_n, 0));

	/*
	 * compute u1 and u2
	 * convert r to n-residue domain
	 * x[8] = x[4] * x[2] mod x[0]
	 */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 8),
			PKA_LIR(lir_n, 4));
	sequence[9].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert h to n-residue domain
	 * x[9] = x[5] * x[2] mod x[0]
	 */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 9),
			PKA_LIR(lir_n, 5));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/* x[4] = x[8] * x[3] mod x[0] (u2 = r * s^(-1) mod n) */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 4),
			PKA_LIR(lir_n, 8));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_n, 3), PKA_LIR(lir_n, 0));

	/* x[5] = x[9] * x[3] mod x[0] (u1 = h * s^(-1) mod n) */
	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 5),
			PKA_LIR(lir_n, 9));
	sequence[12].op2 = PACK_OP2(PKA_LIR(lir_n, 3), PKA_LIR(lir_n, 0));

	/* x[7] = 1 */
	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_n, 7), PKA_NULL);
	sequence[13].op2 = PACK_OP2(PKA_NULL, 1);

	/*
	 * convert u2 back to normal domain
	 * x[3] = x[4] * x[7] mod x[0]
	 */
	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 3),
			PKA_LIR(lir_n, 4));
	sequence[14].op2 = PACK_OP2(PKA_LIR(lir_n, 7), PKA_LIR(lir_n, 0));

	/*
	 * convert u1 back to normal domain
	 * x[4] = x[5] * x[7] mod x[0]
	 */
	sequence[15].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 4),
			PKA_LIR(lir_n, 5));
	sequence[15].op2 = PACK_OP2(PKA_LIR(lir_n, 7), PKA_LIR(lir_n, 0));

	/* unload x[3] (u2) */
	sequence[16].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 3), u2.size);

	/* unload x[4] (u1) */
	sequence[17].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 4), u1.size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_ECP_ECDSA_VERIFY_EXIT;
	}
	q_pka_hw_write_sequence(18, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {

		/*
		 * While we wait for the the first unload opcode,
		 * we want to monitor the CMD_ERR bit in the status
		 * register, as the math opcodes before the first
		 * unload opcode may trigger PKA HW	related error.
		 * We do not need to monitor the CMD_ERR for the
		 * subsequent unload opcode.
		*/

		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_ECP_ECDSA_VERIFY_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_ECP_ECDSA_VERIFY_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(u2.size, u2.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(u1.size, u1.limb);

#else
#ifdef QLIP_USE_GMP
	mpz_init(mx);
	mpz_init(mu1);
	mpz_init(mu2);
	mpz_init(mh);
	mpz_init(ms);
	mpz_init(mr);
	mpz_init(mn);

	mpz_import(mh, h->size, -1, 4, 0, 0, h->limb);
	mpz_import(ms, rs->s.size, -1, 4, 0, 0, rs->s.limb);
	mpz_import(mr, rs->r.size, -1, 4, 0, 0, rs->r.limb);
	mpz_import(mn, curve->n.size, -1, 4, 0, 0, curve->n.limb);

	/* compute the inverse of s */
	mpz_invert(mu2, ms, mn);

	/* compute u1 and u2 */
	mpz_mul(mu1, mu2, mh);
	mpz_mod(mu1, mu1, mn);

	mpz_mul(mu2, mu2, mr);
	mpz_mod(mu2, mu2, mn);

	mpz_export(u1.limb, &u1.size, -1, 4, 0, 0, mu1);
	mpz_export(u2.limb, &u2.size, -1, 4, 0, 0, mu2);
	mpz_clear(mu1);
	mpz_clear(mu2);
	mpz_clear(mh);
	mpz_clear(ms);
	mpz_clear(mr);
#else
#ifdef QLIP_MOD_USE_MONT
	struct q_lint mt;
	struct q_mont mp;
	q_init(ctx, &mp.n, curve->n.alloc);
	q_init(ctx, &mp.np, curve->n.alloc + 1);
	q_init(ctx, &mp.rr, curve->n.alloc + 1);
	q_mont_init(ctx, &mp, &curve->n);

	/* compute the inverse of s */
	q_init(ctx, &mt, curve->n.alloc + 1);
	mt.size = 1;
	mt.limb[0] = 2L;
	q_usub(&mt, &curve->n, &mt);
	q_mont_mul(ctx, &u2, &mp.rr, &rs->s, &mp);
	q_modexp_mont(ctx, &u2, &u2, &mt, &mp);

	/* compute u1 and u2 */
	q_mont_mul(ctx, &u1, &mp.rr, h, &mp);
	q_mont_mul(ctx, &u1, &u2, &u1, &mp);
	q_mont_mul(ctx, &mt, &mp.rr, &rs->r, &mp);
	q_mont_mul(ctx, &u2, &u2, &mt, &mp);

	q_set_one(&mt);
	q_mont_mul(ctx, &u1, &u1, &mt, &mp);
	q_mont_mul(ctx, &u2, &u2, &mt, &mp);
	q_free(ctx, &mt);
#else
	/* compute the inverse of s */
	q_modpinv(ctx, &u2, &rs->s, &curve->n);

	/* compute u1 and u2 */
	q_modmul(ctx, &u1, &u2, h, &curve->n);
	q_modmul(ctx, &u2, &u2, &rs->r, &curve->n);
#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_GMP */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_PKE_HW */

#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 21; i++)
		sequence[i].ptr = NULL;

	q_mont_init(ctx, &mp, &curve->p);
	curve->mp = &mp;

	/* create command sequence */
	lir_n = q_pka_sel_lir(curve->n.size);

	/* x[0] = p.n */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 0), mp.n.size);
	sequence[0].ptr = mp.n.limb;

	/* x[1] = p.np */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 1), mp.np.size);
	sequence[1].ptr = mp.np.limb;

	/* x[2] = p.rr */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 2), mp.rr.size);
	sequence[2].ptr = mp.rr.limb;

	/*
	 * base point coordinates (Jacobian)
	 * x[5] = G.X
	 */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 5), GG.X.size);
	sequence[3].ptr = GG.X.limb;

	/* x[6] = G.Y */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 6), GG.Y.size);
	sequence[4].ptr = GG.Y.limb;

	/* x[7] = G.Z */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 7), GG.Z.size);
	sequence[5].ptr = GG.Z.limb;

	/*
	 * public point coordinates (Jacobian)
	 * x[8] = Q.X
	 * */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 8), QQ.X.size);
	sequence[6].ptr = QQ.X.limb;

	/* x[9] = Q.Y */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 9), QQ.Y.size);
	sequence[7].ptr = QQ.Y.limb;

	/* x[10] = Q.Z */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 10), QQ.Z.size);
	sequence[8].ptr = QQ.Z.limb;

	/*
	 * convert G.X to p-residue domain
	 * x[4] = x[5] * x[2] mod x[0]
	 */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 4),
			PKA_LIR(lir_n, 5));
	sequence[9].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert G.Y to p-residue domain
	 * x[5] = x[6] * x[2] mod x[0]
	 */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 5),
			PKA_LIR(lir_n, 6));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert G.Z to p-residue domain
	 * x[6] = x[7] * x[2] mod x[0]
	 */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 6),
			PKA_LIR(lir_n, 7));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert Q.X to p-residue domain
	 * x[7] = x[8] * x[2] mod x[0]
	 */
	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 7),
			PKA_LIR(lir_n, 8));
	sequence[12].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert Q.Y to p-residue domain
	 * x[8] = x[9] * x[2] mod x[0]
	 */
	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 8),
			PKA_LIR(lir_n, 9));
	sequence[13].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert Q.Z to p-residue domain
	 * x[9] = x[10] * x[2] mod x[0]
	 */
	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 9),
			PKA_LIR(lir_n, 10));
	sequence[14].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/* unload x[4] (G.X in p-residue domain) */
	sequence[15].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 4), size);

	/* unload x[5] (G.Y in p-residue domain) */
	sequence[16].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 5), size);

	/* unload x[6] (G.Z in p-residue domain) */
	sequence[17].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 6), size);

	/* unload x[7] (Q.X in p-residue domain) */
	sequence[18].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 7), size);

	/* unload x[8] (Q.Y in p-residue domain) */
	sequence[19].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 8), size);

	/* unload x[9] (Q.Z in p-residue domain) */
	sequence[20].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 9), size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_ECP_ECDSA_VERIFY_EXIT;
	}
	q_pka_hw_write_sequence(21, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {

		/*
		 * While we wait for the the first unload opcode,
		 * we want to monitor the CMD_ERR bit in the status
		 * register, as the math opcodes before the first
		 * unload opcode may trigger PKA HW	related error.
		 * We do not need to monitor the CMD_ERR for the
		 * subsequent unload opcode.
		*/

		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_ECP_ECDSA_VERIFY_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_ECP_ECDSA_VERIFY_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, GG.X.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, GG.Y.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, GG.Z.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, QQ.X.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, QQ.Y.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, QQ.Z.limb);

	GG.X.size = GG.Y.size = GG.Z.size = size;
	QQ.X.size = QQ.Y.size = QQ.Z.size = size;

	for (i = 0; i < 21; i++)
		sequence[i].ptr = NULL;

	/*
	 * create command sequence
	 * x[4] = curve.a
	 */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 4), curve->a.size);
	sequence[0].ptr = curve->a.limb;

	/* x[5] = g_minus.X (Jacobian) */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 5), g_minus.X.size);
	sequence[1].ptr = g_minus.X.limb;

	/* x[6] = g_minus.Y (Jacobian) */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 6), g_minus.Y.size);
	sequence[2].ptr = g_minus.Y.limb;

	/* x[7] = g_minus.Z (Jacobian) */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 7), g_minus.Z.size);
	sequence[3].ptr = g_minus.Z.limb;

	/* x[8] = q_minus.X (Jacobian) */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 8), q_minus.X.size);
	sequence[4].ptr = q_minus.X.limb;

	/* x[9] = q_minus.Y (Jacobian) */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 9), q_minus.Y.size);
	sequence[5].ptr = q_minus.Y.limb;

	/* x[10] = q_minus.Z (Jacobian) */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 10),
			q_minus.Z.size);
	sequence[6].ptr = q_minus.Z.limb;

	/*
	 * convert parameters
	 * convert curve.a to p-residue domain
	 * x[3] = x[4] * x[2] mod x[0]
	 */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 3),
			PKA_LIR(lir_n, 4));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert g_minus.X to p-residue domain
	 * x[4] = x[5] * x[2] mod x[0]
	 */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 4),
			PKA_LIR(lir_n, 5));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert g_minus.Y to p-residue domain
	 * x[5] = x[6] * x[2] mod x[0]
	 */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 5),
			PKA_LIR(lir_n, 6));
	sequence[9].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert g_minus.Z to p-residue domain
	 * x[6] = x[7] * x[2] mod x[0]
	 */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 6),
			PKA_LIR(lir_n, 7));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert q_minus.X to p-residue domain
	 * x[7] = x[8] * x[2] mod x[0]
	 */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 7),
			PKA_LIR(lir_n, 8));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert q_minus.Y to p-residue domain
	 * x[8] = x[9] * x[2] mod x[0]
	 */
	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 8),
			PKA_LIR(lir_n, 9));
	sequence[12].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/*
	 * convert q_minus.Z to p-residue domain
	 * x[9] = x[10] * x[2] mod x[0]
	 */
	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_n, 9),
			PKA_LIR(lir_n, 10));
	sequence[13].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));

	/* unload x[3] (curve.a in p-residue domain) */
	sequence[14].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 3), size);

	/* unload x[4] (g_minus.X in p-residue domain) */
	sequence[15].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 4), size);

	/* unload x[5] (g_minus.Y in p-residue domain) */
	sequence[16].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 5), size);

	/* unload x[6] (g_minus.Z in p-residue domain) */
	sequence[17].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 6), size);

	/* unload x[7] (q_minus.X in p-residue domain) */
	sequence[18].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 7), size);

	/* unload x[8] (q_minus.Y in p-residue domain) */
	sequence[19].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 8), size);

	/* unload x[9] (q_minus.Z in p-residue domain) */
	sequence[20].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 9), size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_ECP_ECDSA_VERIFY_EXIT;
	}
	q_pka_hw_write_sequence(21, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {

		/*
		 * While we wait for the the first unload opcode,
		 * we want to monitor the CMD_ERR bit in the status
		 * register, as the math	opcodes before the first
		 * unload opcode may trigger PKA HW related error.
		 * We do not need to monitor the CMD_ERR for the
		 * subsequent unload opcode.
		 */
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_ECP_ECDSA_VERIFY_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_ECP_ECDSA_VERIFY_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, curve->a.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, g_minus.X.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, g_minus.Y.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, g_minus.Z.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, q_minus.X.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, q_minus.Y.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(size, q_minus.Z.limb);

	g_minus.X.size = g_minus.Y.size = g_minus.Z.size = size;
	q_minus.X.size = q_minus.Y.size = q_minus.Z.size = size;

#else
#ifdef QLIP_MOD_USE_MONT
	q_mont_init(ctx, &mp, &curve->p);
	curve->mp = &mp;
	q_mont_mul(ctx, &curve->a, &curve->a, &mp.rr, &mp);
	q_mont_mul(ctx, &GG.X, &GG.X, &mp.rr, &mp);
	q_mont_mul(ctx, &GG.Y, &GG.Y, &mp.rr, &mp);
	q_mont_mul(ctx, &GG.Z, &GG.Z, &mp.rr, &mp);
	q_mont_mul(ctx, &QQ.X, &QQ.X, &mp.rr, &mp);
	q_mont_mul(ctx, &QQ.Y, &QQ.Y, &mp.rr, &mp);
	q_mont_mul(ctx, &QQ.Z, &QQ.Z, &mp.rr, &mp);
	q_mont_mul(ctx, &g_minus.X, &g_minus.X, &mp.rr, &mp);
	q_mont_mul(ctx, &g_minus.Y, &g_minus.Y, &mp.rr, &mp);
	q_mont_mul(ctx, &g_minus.Z, &g_minus.Z, &mp.rr, &mp);
	q_mont_mul(ctx, &q_minus.X, &q_minus.X, &mp.rr, &mp);
	q_mont_mul(ctx, &q_minus.Y, &q_minus.Y, &mp.rr, &mp);
	q_mont_mul(ctx, &q_minus.Z, &q_minus.Z, &mp.rr, &mp);
#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_PKA_HW */

#ifdef QLIP_USE_PKA_HW
	q_pka_hw_rst();
#endif

	/* compute point operations */
	if (q_ecp_pt_mul_prj(ctx, &R1, &GG, &g_minus, &u1, curve, tmp))
		goto Q_ECP_ECDSA_VERIFY_EXIT;
	if (q_ecp_pt_mul_prj(ctx, &R2, &QQ, &q_minus, &u2, curve, tmp))
		goto Q_ECP_ECDSA_VERIFY_EXIT;
	if (q_ecp_pt_add_prj(ctx, &R1, &R1, &R2, curve, tmp))
		goto Q_ECP_ECDSA_VERIFY_EXIT;
	if (q_ecp_prj_2_affine(ctx, &R1, &R1, curve))
		goto Q_ECP_ECDSA_VERIFY_EXIT;

#ifdef QLIP_USE_PKA_HW
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#endif
#ifdef QLIP_MOD_USE_MONT
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#endif

#ifdef QLIP_USE_PKE_HW
	q_modrem_hw(v->limb, blen_n, curve->n.limb, R1.X.limb);
#else
#ifdef QLIP_USE_PKA_HW
	q_pka_hw_rst();
	q_mod(ctx, v, &R1.X, &curve->n);
#else
#ifdef QLIP_USE_GMP
	mpz_import(mx, R1.X.size, -1, 4, 0, 0, R1.X.limb);
	mpz_mod(mx, mx, mn);
	mpz_export(v->limb, &v->size, -1, 4, 0, 0, mx);
	mpz_clear(mx);
	mpz_clear(mn);
#else
	q_mod(ctx, v, &R1.X, &curve->n);
#endif /*QLIP_USE_GMP */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_PKE_HW */

	q_copy(&curve->a, &ca);
	for (i = 5; i >= 0; i--)
		q_free(ctx, &tmp[i]);

	q_free(ctx, &ca);
	q_free(ctx, &q_minus.Z);
	q_free(ctx, &q_minus.Y);
	q_free(ctx, &q_minus.X);
	q_free(ctx, &g_minus.Z);
	q_free(ctx, &g_minus.Y);
	q_free(ctx, &g_minus.X);
	q_free(ctx, &QQ.Z);
	q_free(ctx, &QQ.Y);
	q_free(ctx, &QQ.X);
	q_free(ctx, &GG.Z);
	q_free(ctx, &GG.Y);
	q_free(ctx, &GG.X);
	q_free(ctx, &R2.Z);
	q_free(ctx, &R2.Y);
	q_free(ctx, &R2.X);
	q_free(ctx, &R1.Z);
	q_free(ctx, &R1.Y);
	q_free(ctx, &R1.X);
	q_free(ctx, &u2);
	q_free(ctx, &u1);

Q_ECP_ECDSA_VERIFY_EXIT:
	return ctx->status;
}
EXPORT_SYMBOL(q_ecp_ecdsa_verify);
