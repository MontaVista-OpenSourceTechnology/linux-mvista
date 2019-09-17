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
#include "q_dsa.h"
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
 * q_dsa_sign ()
 * Description: DSA sign function.
 * r = (g ^ k mod p) mod q
 * s = (k^-1 * (h + d * r)) mod q
 * Note: Do we need to support hash sizes different from 5 words ?
 * @param ctx		QLIP context pointer
 * @param rs		q_signature point to rs
 * @param dsa		q_dsa_param pointer to dsa
 * @param d		q_lint pointer to private key d
 * @param h		q_lint poitner to message hash h
 * @param k		q_lint pointer to k
 */
int32_t q_dsa_sign(struct q_lip_ctx *ctx,
					  struct q_signature *rs,
					  struct q_dsa_param *dsa,
					  struct q_lint *d,
					  struct q_lint *h,
					  struct q_lint *k)
{
	int32_t status = Q_SUCCESS;

#ifdef QLIP_USE_PKE_HW
	uint32_t blen_p;
#endif

#ifdef QLIP_USE_PKA_HW
	int i;
	struct q_mont mont1, mont2;
	uint32_t pka_status;
	uint32_t lir_h, lir_p;
	struct opcode sequence[27];
#endif

#ifdef QLIP_USE_PKE_HW
	blen_p = dsa->p.size * MACHINE_WD;
	rs->r.size = h->size;
	rs->s.size = h->size;
	q_dsa_sign_hw(rs->r.limb, rs->s.limb, blen_p, dsa->q.limb,
				dsa->p.limb, dsa->g.limb, d->limb,
				h->limb, k->limb);
#else
#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 27; i++)
		sequence[i].ptr = NULL;

	/* initialize hardware first */
	q_pka_hw_rst();

	/*
	 * ZQI we need to allocate the LIRs more efficiently
	 * because some	parameters are smaller in size
	 */
	status = q_init(ctx, &mont1.n, dsa->p.alloc);
	status += q_init(ctx, &mont1.np, (dsa->p.alloc + 1));
	status += q_init(ctx, &mont1.rr, (dsa->p.alloc + 1));

	status += q_init(ctx, &mont2.n, dsa->q.alloc);
	status += q_init(ctx, &mont2.np, (dsa->q.alloc + 1));
	status += q_init(ctx, &mont2.rr, (dsa->q.alloc + 1));
	if (status != Q_SUCCESS)
		goto Q_DSA_SIGN_EXIT;
	if (q_mont_init(ctx, &mont1, &dsa->p))
		goto Q_DSA_SIGN_EXIT;
	q_pka_hw_rst();
	if (q_mont_init(ctx, &mont2, &dsa->q))
		goto Q_DSA_SIGN_EXIT;
	q_pka_hw_rst();

	/* create command sequence */
	lir_p = PKA_LIR_F;
	lir_h = PKA_LIR_C;

	/*
	 * Load Montgomery parameters of q (always 160 bits)
	 * h[0] = q.n
	 */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 0), mont2.n.size);
	sequence[0].ptr = mont2.n.limb;

	/* h[1] = q.np */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 1), mont2.np.size);
	sequence[1].ptr = mont2.np.limb;

	/* h[2] = q.rr */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 2), mont2.rr.size);
	sequence[2].ptr = mont2.rr.limb;

	/*
	 * Load d = a randomly or pseudorandomly generated
	 * integer with 0 < d < q (160 bits)
	 * h[3] = d
	 */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 3), d->size);
	sequence[3].ptr = d->limb;

	/*
	 * Load h = hash of message (160 bits)
	 * h[4] = h
	 * */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 4), h->size);
	sequence[4].ptr = h->limb;

	/*
	 * Load k = a randomly or pseudorandomly generated
	 * integer with 0 < k < q (160 bits)
	 * h[5] = k
	 */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 5), k->size);
	sequence[5].ptr = k->limb;

	/*
	 * Load g (max 1024 bits)
	 * p[3] = g
	 * */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), dsa->g.size);
	sequence[6].ptr = dsa->g.limb;

	/*
	 * Load Montgomery parameters of p (max 1024 bits)
	 * p[4] = p.n
	 */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), mont1.n.size);
	sequence[7].ptr = mont1.n.limb;

	/* p[5] = p.np */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), mont1.np.size);
	sequence[8].ptr = mont1.np.limb;

	/* p[6] = p.rr */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 6), mont1.rr.size);
	sequence[9].ptr = mont1.rr.limb;

	/*
	 * compute signature r
	 * convert g to p-residue
	 * p[3] = p[3] * p[6] mod p[4]
	 */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 3));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_p, 6), PKA_LIR(lir_p, 4));

	/* p[7] = p[3] ^ h[5] mod p[4] */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODEXP, PKA_LIR(lir_p, 7),
			PKA_LIR(lir_p, 3));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_h, 5), PKA_LIR(lir_p, 4));

	/* convert back p[3] = 1 */
	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 3), PKA_NULL);
	sequence[12].op2 = PACK_OP2(PKA_NULL, 1);

	/* p[7] = p[3] * p[7] mod p[4] */
	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 7),
			PKA_LIR(lir_p, 3));
	sequence[13].op2 = PACK_OP2(PKA_LIR(lir_p, 7), PKA_LIR(lir_p, 4));

	/*
	 * r = h[6]
	 * h[6] = p[7] mod h[0]
	 */
	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_MODREM, PKA_LIR(lir_h, 6),
			PKA_LIR(lir_p, 7));
	sequence[14].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_h, 0));

	/*
	 * convert d to q-residue
	 * h[7] = h[6] * h[2] mod h[0]
	 */
	sequence[15].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 7),
			PKA_LIR(lir_h, 6));
	sequence[15].op2 = PACK_OP2(PKA_LIR(lir_h, 2), PKA_LIR(lir_h, 0));

	/*
	 * convert g to q-residue
	 * h[3] = h[3] * h[2] mod h[0]
	 */
	sequence[16].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 3),
			PKA_LIR(lir_h, 3));
	sequence[16].op2 = PACK_OP2(PKA_LIR(lir_h, 2), PKA_LIR(lir_h, 0));

	/*
	 * mul d
	 * h[7] = h[7] * h[3] mod h[0]
	 */
	sequence[17].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 7),
			PKA_LIR(lir_h, 7));
	sequence[17].op2 = PACK_OP2(PKA_LIR(lir_h, 3), PKA_LIR(lir_h, 0));

	/*
	 * convert h to q-residue
	 * h[4] = h[4] * h[2] mod h[0]
	 */
	sequence[18].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 4),
			PKA_LIR(lir_h, 4));
	sequence[18].op2 = PACK_OP2(PKA_LIR(lir_h, 2), PKA_LIR(lir_h, 0));

	/* h[7] = h[7] + h[4] mod h[0] */
	sequence[19].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_h, 7),
			PKA_LIR(lir_h, 7));
	sequence[19].op2 = PACK_OP2(PKA_LIR(lir_h, 4), PKA_LIR(lir_h, 0));

	/* compute kinv
	 * convert k to q-residue
	 * h[5] = h[5] * h[2] mod h[0]
	 * */
	sequence[20].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 5),
			PKA_LIR(lir_h, 5));
	sequence[20].op2 = PACK_OP2(PKA_LIR(lir_h, 2), PKA_LIR(lir_h, 0));

	/* h[3] = h[5] ^ (h[0] - 2) mod h[0] */
	sequence[21].op1 =
		PACK_OP1(0, PKA_OP_MODINV, PKA_LIR(lir_h, 3),
			PKA_LIR(lir_h, 5));
	sequence[21].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_h, 0));

	/* h[7] = h[7] * h[3] mod h[0] */
	sequence[22].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 7),
			PKA_LIR(lir_h, 7));
	sequence[22].op2 = PACK_OP2(PKA_LIR(lir_h, 3), PKA_LIR(lir_h, 0));

	/* convert back
	 * h[3] = 1
	 */
	sequence[23].op1 =
		PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_h, 3), PKA_NULL);
	sequence[23].op2 = PACK_OP2(PKA_NULL, 1);

	/* s = h[7]
	 * h[7] = h[7] * h[3] mod h[0]
	 */
	sequence[24].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 7),
			PKA_LIR(lir_h, 7));
	sequence[24].op2 = PACK_OP2(PKA_LIR(lir_h, 3), PKA_LIR(lir_h, 0));

	/* unload r = h[6] */
	sequence[25].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_h, 6), h->size);

	/* unload s = h[7] */
	sequence[26].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_h, 7), h->size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_DSA_SIGN_EXIT;
	}
	q_pka_hw_write_sequence(27, sequence);
	q_free(ctx, &mont2.rr);
	q_free(ctx, &mont2.np);
	q_free(ctx, &mont2.n);

	q_free(ctx, &mont1.rr);
	q_free(ctx, &mont1.np);
	q_free(ctx, &mont1.n);

	rs->r.size = h->size;
	rs->s.size = h->size;

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
			goto Q_DSA_SIGN_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_DSA_SIGN_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(rs->r.size, rs->r.limb);
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(rs->s.size, rs->s.limb);
#else
#ifdef QLIP_USE_GMP
	mpz_t mp, mg, mq, mk, mh, md, mr, ms;
	mpz_t t1, t2;
	mpz_init(mp);
	mpz_init(mg);
	mpz_init(mq);
	mpz_init(mk);
	mpz_init(mh);
	mpz_init(md);
	mpz_init(mr);
	mpz_init(ms);
	mpz_init(t1);
	mpz_init(t2);

	mpz_import(mp, dsa->p.size, -1, 4, 0, 0, dsa->p.limb);
	mpz_import(mg, dsa->g.size, -1, 4, 0, 0, dsa->g.limb);
	mpz_import(mq, dsa->q.size, -1, 4, 0, 0, dsa->q.limb);
	mpz_import(mk, k->size, -1, 4, 0, 0, k->limb);
	mpz_import(mh, h->size, -1, 4, 0, 0, h->limb);
	mpz_import(md, d->size, -1, 4, 0, 0, d->limb);

	/* compute signature */
	mpz_powm(t1, mg, mk, mp);
	mpz_mod(mr, t1, mq);

	mpz_mul(t1, mr, md);
	mpz_add(t1, t1, mh);
	mpz_mod(t1, t1, mq);

	/* compute kinv */
	mpz_set_si(t2, 2);
	mpz_sub(t2, mq, t2);
	mpz_powm(t2, mk, t2, mq);

	mpz_mul(t1, t2, t1);
	mpz_mod(ms, t1, mq);

	/* output result */
	mpz_export(rs->r.limb, &(rs->r.size), -1, 4, 0, 0, mr);
	mpz_export(rs->s.limb, &(rs->s.size), -1, 4, 0, 0, ms);

	mpz_clear(mp);
	mpz_clear(mg);
	mpz_clear(mq);
	mpz_clear(mk);
	mpz_clear(mh);
	mpz_clear(md);
	mpz_clear(mr);
	mpz_clear(ms);
	mpz_clear(t1);
	mpz_clear(t2);
#else
#ifdef QLIP_MOD_USE_MONT
	struct q_lint t1, t2, t3;
	q_init(ctx, &t1, dsa->p.alloc);
	q_init(ctx, &t2, dsa->p.alloc);
	q_init(ctx, &t3, dsa->p.alloc);

	struct q_mont mont1, mont2;
	q_init(ctx, &mont1.n, dsa->p.alloc);
	q_init(ctx, &mont1.np, (dsa->p.alloc + 1));
	q_init(ctx, &mont1.rr, (dsa->p.alloc + 1));

	q_init(ctx, &mont2.n, dsa->q.alloc);
	q_init(ctx, &mont2.np, (dsa->q.alloc + 1));
	q_init(ctx, &mont2.rr, (dsa->q.alloc + 1));

	q_mont_init(ctx, &mont1, &dsa->p);
	q_mont_init(ctx, &mont2, &dsa->q);

	/* compute signature */
	q_mont_mul(ctx, &t1, &mont1.rr, &dsa->g, &mont1);
	q_modexp_mont(ctx, &t1, &t1, k, &mont1);

	q_set_one(&t2);
	q_mont_mul(ctx, &t1, &t1, &t2, &mont1);
	q_mod(ctx, &rs->r, &t1, &dsa->q);

	q_mont_mul(ctx, &t1, &mont2.rr, &rs->r, &mont2);
	q_mont_mul(ctx, &t2, &mont2.rr, d, &mont2);
	q_mont_mul(ctx, &t1, &t1, &t2, &mont2);

	q_mont_mul(ctx, &t2, &mont2.rr, h, &mont2);
	q_modadd(ctx, &t1, &t1, &t2, &dsa->q);

	/* compute kinv */
	t3.size = 1;
	t3.limb[0] = 2L;
	q_usub(&t3, &dsa->q, &t3);
	q_mont_mul(ctx, &t2, &mont2.rr, k, &mont2);
	q_modexp_mont(ctx, &t2, &t2, &t3, &mont2);

	q_mont_mul(ctx, &t1, &t2, &t1, &mont2);

	q_set_one(&t2);
	q_mont_mul(ctx, &rs->s, &t1, &t2, &mont2);

	q_free(ctx, &mont2.rr);
	q_free(ctx, &mont2.np);
	q_free(ctx, &mont2.n);

	q_free(ctx, &mont1.rr);
	q_free(ctx, &mont1.np);
	q_free(ctx, &mont1.n);

	q_free(ctx, &t3);
	q_free(ctx, &t2);
	q_free(ctx, &t1);
#else
	struct q_lint t1, t2;
	q_init(ctx, &t1, dsa->p.alloc);
	q_init(ctx, &t2, dsa->p.alloc);

	/* compute kinv */
	t1.size = 1;
	t1.limb[0] = 2L;
	q_usub(&t1, &dsa->q, &t1);
	q_modexp(ctx, &t2, k, &t1, &dsa->q);

	/* compute signature */
	q_modexp(ctx, &t1, &dsa->g, k, &dsa->p);
	q_mod(ctx, &rs->r, &t1, &dsa->q);

	q_modmul(ctx, &t1, &rs->r, d, &dsa->q);
	q_modadd(ctx, &t1, &t1, h, &dsa->q);

	q_modmul(ctx, &rs->s, &t2, &t1, &dsa->q);
	q_free(ctx, &t2);
	q_free(ctx, &t1);
#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_GMP */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_PKE_HW */

Q_DSA_SIGN_EXIT:
	return ctx->status;
}
EXPORT_SYMBOL(q_dsa_sign);

/*
 * q_dsa_verify ()
 * Description: DSA verify function.
 * @param ctx		QLIP context pointer
 * @param v		q_lint pointer to v
 * @param dsa		q_dsa_param pointer to dsa
 * @param y		q_lint pointer to y
 * @param h		q_lint pointer to h
 * @param rs		q_signature point to rs
 */
int32_t q_dsa_verify(struct q_lip_ctx *ctx,
						struct q_lint *v,
						struct q_dsa_param *dsa,
						struct q_lint *y,
						struct q_lint *h,
						struct q_signature *rs)
{
	int32_t status = Q_SUCCESS;

#ifdef QLIP_USE_PKE_HW
	uint32_t blen_p;
#endif

#ifdef QLIP_USE_PKA_HW
	int i;
	struct q_mont mont1, mont2;
	uint32_t pka_status;
	uint32_t lir_h, lir_p;
	struct opcode sequence[29];
#endif

#ifdef QLIP_USE_PKE_HW
	blen_p = dsa->p.size * MACHINE_WD;
	v->size = h->size;
	q_dsa_verf_hw(v->limb, blen_p, dsa->q.limb, dsa->p.limb,
				  dsa->g.limb, y->limb, h->limb,
				  rs->r.limb, rs->s.limb);
#else
#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 29; i++)
		sequence[i].ptr = NULL;

	/* initialize hardware first */
	q_pka_hw_rst();

	/* create command sequence */
	lir_p = PKA_LIR_F;
	lir_h = PKA_LIR_C;

	/*
	 * ZQI we need to allocate the LIRs more efficiently because some
	 * parameters are smaller in size
	 */
	status = q_init(ctx, &mont1.n, dsa->p.alloc);
	status += q_init(ctx, &mont1.np, (dsa->p.alloc + 1));
	status += q_init(ctx, &mont1.rr, (dsa->p.alloc + 1));

	status += q_init(ctx, &mont2.n, dsa->q.alloc);
	status += q_init(ctx, &mont2.np, (dsa->q.alloc + 1));
	status += q_init(ctx, &mont2.rr, (dsa->q.alloc + 1));
	if (status != Q_SUCCESS)
		goto Q_DSA_VERIFY_EXIT;

	if (q_mont_init(ctx, &mont1, &dsa->p))
		goto Q_DSA_VERIFY_EXIT;
	q_pka_hw_rst();
	if (q_mont_init(ctx, &mont2, &dsa->q))
		goto Q_DSA_VERIFY_EXIT;
	q_pka_hw_rst();

	/* create command sequence */
	lir_p = q_pka_sel_lir(dsa->p.size);

	/* h[0] = q.n (160 bits) */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 0), mont2.n.size);
	sequence[0].ptr = mont2.n.limb;

	/* h[1] = q.np */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 1), mont2.np.size);
	sequence[1].ptr = mont2.np.limb;

	/* h[2] = q.rr */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 2), mont2.rr.size);
	sequence[2].ptr = mont2.rr.limb;

	/* h[3] = h (hash 160 bits) */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 3), h->size);
	sequence[3].ptr = h->limb;

	/* h[4] = r */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 4), rs->r.size);
	sequence[4].ptr = rs->r.limb;

	/* h[5] = s */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_h, 5), rs->s.size);
	sequence[5].ptr = rs->s.limb;

	/* p[4] = p.n (max 1024 bits) */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), mont1.n.size);
	sequence[6].ptr = mont1.n.limb;

	/* p[5] = p.np */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), mont1.np.size);
	sequence[7].ptr = mont1.np.limb;

	/* p[6] = p.rr */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 6), mont1.rr.size);
	sequence[8].ptr = mont1.rr.limb;

	/* p[7] = g (max 1024 bits) */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 7), dsa->g.size);
	sequence[9].ptr = dsa->g.limb;

	/* p[8] = y */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 8), y->size);
	sequence[10].ptr = y->limb;

	/*
	 * compute sinv
	 * h[6] = h[5] * h[2] mod h[0]
	 */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 6),
			PKA_LIR(lir_h, 5));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_h, 2), PKA_LIR(lir_h, 0));

	/*
	 * h[5] = h[6] ^ (h[0] - 2) mod h[0]
	 */
	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODINV, PKA_LIR(lir_h, 5),
			PKA_LIR(lir_h, 6));
	sequence[12].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_h, 0));

	/*
	 * compute signature
	 * h[3] = h[3] * h[2] mod h[0]
	 */
	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 3),
			PKA_LIR(lir_h, 3));
	sequence[13].op2 = PACK_OP2(PKA_LIR(lir_h, 2), PKA_LIR(lir_h, 0));

	/* h[3] = h[5] * h[3] mod h[0] */
	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 3),
			PKA_LIR(lir_h, 5));
	sequence[14].op2 = PACK_OP2(PKA_LIR(lir_h, 3), PKA_LIR(lir_h, 0));

	/* h[4] = h[4] * h[2] mod h[0] */
	sequence[15].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 4),
			PKA_LIR(lir_h, 4));
	sequence[15].op2 = PACK_OP2(PKA_LIR(lir_h, 2), PKA_LIR(lir_h, 0));

	/* h[4] = h[5] * h[4] mod h[0] */
	sequence[16].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 4),
			PKA_LIR(lir_h, 5));
	sequence[16].op2 = PACK_OP2(PKA_LIR(lir_h, 4), PKA_LIR(lir_h, 0));

	/* convert the exponents back */
	/* h[6] = 1 */
	sequence[17].op1 =
		PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_h, 6), PKA_NULL);
	sequence[17].op2 = PACK_OP2(PKA_NULL, 1);

	/* h[3] = h[3] * h[6] mod h[0] */
	sequence[18].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 3),
			PKA_LIR(lir_h, 3));
	sequence[18].op2 = PACK_OP2(PKA_LIR(lir_h, 6), PKA_LIR(lir_h, 0));

	/* h[4] = h[4] * h[6] mod h[0] */
	sequence[19].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_h, 4),
			PKA_LIR(lir_h, 4));
	sequence[19].op2 = PACK_OP2(PKA_LIR(lir_h, 6), PKA_LIR(lir_h, 0));

	/*
	 * convert y to p-residue
	 * p[8] = p[8] * p[6] mod p[4]
	 */
	sequence[20].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 8),
			PKA_LIR(lir_p, 8));
	sequence[20].op2 = PACK_OP2(PKA_LIR(lir_p, 6), PKA_LIR(lir_p, 4));

	/*
	 * convert g to p-residue
	 * p[7] = p[7] * p[6] mod p[4]
	 */
	sequence[21].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 7),
			PKA_LIR(lir_p, 7));
	sequence[21].op2 = PACK_OP2(PKA_LIR(lir_p, 6), PKA_LIR(lir_p, 4));

	/* p[9] = p[8] ^ h[4] mod p[4] */
	sequence[22].op1 =
		PACK_OP1(0, PKA_OP_MODEXP, PKA_LIR(lir_p, 9),
			PKA_LIR(lir_p, 8));
	sequence[22].op2 = PACK_OP2(PKA_LIR(lir_h, 4), PKA_LIR(lir_p, 4));

	/* p[8] = p[7] ^ h[3] mod p[4] */
	sequence[23].op1 =
		PACK_OP1(0, PKA_OP_MODEXP, PKA_LIR(lir_p, 8),
			PKA_LIR(lir_p, 7));
	sequence[23].op2 = PACK_OP2(PKA_LIR(lir_h, 3), PKA_LIR(lir_p, 4));

	/* p[8] = p[9] * p[8] mod p[4] */
	sequence[24].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 8),
			PKA_LIR(lir_p, 9));
	sequence[24].op2 = PACK_OP2(PKA_LIR(lir_p, 8), PKA_LIR(lir_p, 4));

	/*
	 * convert back
	 * p[7] = 1
	 */
	sequence[25].op1 =
		PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 7), PKA_NULL);
	sequence[25].op2 = PACK_OP2(PKA_NULL, 1);

	/* p[8] = p[8] * p[7] mod p[4] */
	sequence[26].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 8),
			PKA_LIR(lir_p, 8));
	sequence[26].op2 = PACK_OP2(PKA_LIR(lir_p, 7), PKA_LIR(lir_p, 4));

	/* h[3] = p[8] mod h[0] */
	sequence[27].op1 =
		PACK_OP1(0, PKA_OP_MODREM, PKA_LIR(lir_h, 3),
			PKA_LIR(lir_p, 8));
	sequence[27].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_h, 0));

	/* v = h[3] */
	sequence[28].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_h, 3), h->size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_DSA_VERIFY_EXIT;
	}
	q_pka_hw_write_sequence(29, sequence);

	q_free(ctx, &mont2.rr);
	q_free(ctx, &mont2.np);
	q_free(ctx, &mont2.n);

	q_free(ctx, &mont1.rr);
	q_free(ctx, &mont1.np);
	q_free(ctx, &mont1.n);

	v->size = h->size;

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_DSA_VERIFY_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_DSA_VERIFY_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(v->size, v->limb);

#else
#ifdef QLIP_USE_GMP
	mpz_t mp, mg, mq, mh, my, mr, ms, mv;
	mpz_t t1, t2;
	mpz_init(mp);
	mpz_init(mg);
	mpz_init(mq);
	mpz_init(mh);
	mpz_init(my);
	mpz_init(mr);
	mpz_init(ms);
	mpz_init(mv);
	mpz_init(t1);
	mpz_init(t2);

	mpz_import(mp, dsa->p.size, -1, 4, 0, 0, dsa->p.limb);
	mpz_import(mg, dsa->g.size, -1, 4, 0, 0, dsa->g.limb);
	mpz_import(mq, dsa->q.size, -1, 4, 0, 0, dsa->q.limb);
	mpz_import(mh, h->size, -1, 4, 0, 0, h->limb);
	mpz_import(my, y->size, -1, 4, 0, 0, y->limb);
	mpz_import(mr, rs->r.size, -1, 4, 0, 0, rs->r.limb);
	mpz_import(ms, rs->s.size, -1, 4, 0, 0, rs->s.limb);

	/* compute sinv */
	mpz_set_si(t1, 2);
	mpz_sub(t1, mq, t1);
	mpz_powm(t1, ms, t1, mq);

	/* compute signature */
	mpz_mul(mv, t1, mh);
	mpz_mod(t2, mv, mq);

	mpz_mul(mv, t1, mr);
	mpz_mod(t1, mv, mq);

	mpz_powm(t2, mg, t2, mp);
	mpz_powm(t1, my, t1, mp);

	mpz_mul(t1, t1, t2);
	mpz_mod(t1, t1, mp);
	mpz_mod(mv, t1, mq);

	/* output result */
	mpz_export(v->limb, &(v->size), -1, 4, 0, 0, mv);

	mpz_clear(mp);
	mpz_clear(mg);
	mpz_clear(mq);
	mpz_clear(mh);
	mpz_clear(my);
	mpz_clear(mr);
	mpz_clear(ms);
	mpz_clear(mv);
	mpz_clear(t1);
	mpz_clear(t2);

#else
#ifdef QLIP_MOD_USE_MONT
	struct q_lint u1, u2;
	struct q_lint t1, t2;

	q_init(ctx, &u1, dsa->p.alloc);
	q_init(ctx, &u2, dsa->p.alloc);
	q_init(ctx, &t1, dsa->p.alloc);
	q_init(ctx, &t2, dsa->p.alloc);

	struct q_mont mont1, mont2;
	q_init(ctx, &mont1.n, dsa->p.alloc);
	q_init(ctx, &mont1.np, (dsa->p.alloc + 1));
	q_init(ctx, &mont1.rr, (dsa->p.alloc + 1));

	q_init(ctx, &mont2.n, dsa->q.alloc);
	q_init(ctx, &mont2.np, (dsa->q.alloc + 1));
	q_init(ctx, &mont2.rr, (dsa->q.alloc + 1));

	q_mont_init(ctx, &mont1, &dsa->p);
	q_mont_init(ctx, &mont2, &dsa->q);

	/* compute sinv */
	u1.size = 1;
	u1.limb[0] = 2L;
	q_usub(&u1, &dsa->q, &u1);

	/* convert to residue */
	q_mont_mul(ctx, &u2, &mont2.rr, &rs->s, &mont2);
	q_modexp_mont(ctx, &t1, &u2, &u1, &mont2);

	/* compute signature */
	q_mont_mul(ctx, &u1, &mont2.rr, h, &mont2);
	q_mont_mul(ctx, &u2, &mont2.rr, &rs->r, &mont2);

	q_mont_mul(ctx, &u1, &u1, &t1, &mont2);
	q_mont_mul(ctx, &u2, &u2, &t1, &mont2);

	/* convert the exponents back */
	q_set_one(&t2);
	q_mont_mul(ctx, &u1, &u1, &t2, &mont2);
	q_mont_mul(ctx, &u2, &u2, &t2, &mont2);

	/* convert to residue */
	q_mont_mul(ctx, &t1, &mont1.rr, &dsa->g, &mont1);
	q_mont_mul(ctx, &t2, &mont1.rr, y, &mont1);

	q_modexp_mont(ctx, &t1, &t1, &u1, &mont1);
	q_modexp_mont(ctx, &t2, &t2, &u2, &mont1);
	q_mont_mul(ctx, &t1, &t1, &t2, &mont1);

	/* convert back */
	q_set_one(&t2);
	q_mont_mul(ctx, &t1, &t1, &t2, &mont1);
	q_mod(ctx, v, &t1, &dsa->q);

	q_free(ctx, &mont2.rr);
	q_free(ctx, &mont2.np);
	q_free(ctx, &mont2.n);

	q_free(ctx, &mont1.rr);
	q_free(ctx, &mont1.np);
	q_free(ctx, &mont1.n);

	q_free(ctx, &t2);
	q_free(ctx, &t1);
	q_free(ctx, &u2);
	q_free(ctx, &u1);
#else
	struct q_lint u1, u2;
	struct q_lint t1, t2;

	q_init(ctx, &u1, dsa->q.alloc);
	q_init(ctx, &u2, dsa->q.alloc);
	q_init(ctx, &t1, dsa->p.alloc);
	q_init(ctx, &t2, dsa->p.alloc);

	/* compute sinv */
	u1.size = 1;
	u1.limb[0] = 2L;
	q_usub(&u1, &dsa->q, &u1);
	q_modexp(ctx, &t1, &rs->s, &u1, &dsa->q);

	/* compute signature */
	q_modmul(ctx, &u1, h, &t1, &dsa->q);
	q_modmul(ctx, &u2, &rs->r, &t1, &dsa->q);

	q_modexp(ctx, &t1, &dsa->g, &u1, &dsa->p);
	q_modexp(ctx, &t2, y, &u2, &dsa->p);
	q_modmul(ctx, &t1, &t1, &t2, &dsa->p);
	q_mod(ctx, v, &t1, &dsa->q);

	q_free(ctx, &t2);
	q_free(ctx, &t1);
	q_free(ctx, &u2);
	q_free(ctx, &u1);
#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_GMP */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_PKE_HW */

Q_DSA_VERIFY_EXIT:
	return ctx->status;
}
EXPORT_SYMBOL(q_dsa_verify);
