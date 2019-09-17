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
#include "q_rsa.h"
#include "q_rsa_4k.h"
#include "q_lip_utils.h"

#ifdef QLIP_USE_PKA_HW
#include "q_pka_hw.h"
#endif

/*
 * q_64b_mul ()
 * q_64b_add ()
 * Description: inline primitive functions
 */

#define lo64(r) (((unsigned *) &r)[0])
#define hi64(r) (((unsigned *) &r)[1])
static inline uint64_t q_64b_mul(uint64_t r, uint32_t a, uint32_t b)
{
#ifdef TARGET_ARM1136
	__asm {
		UMLAL lo64(r), hi64(r), a, b
	}
#else
	r += (uint64_t) a *(uint64_t) b;
#endif
	return r;
}

static inline uint64_t q_64b_add(uint64_t a, uint64_t b)
{
#ifdef TARGET_ARM1136
	__asm {
		ADDS lo64(a), lo64(a), lo64(b);
		ADC hi64(a), hi64(a), hi64(b)
	}
#else
	a = a + b;
#endif
	return a;
}

/*
 * q_modexp_sw ()
 * z = a ^ e mod n
 * Description: Modulo exponentiation.
 * Assumptions:
 * 1. all numbers are bignum
 * 2. no optimization based on bit length
 * 3. size (e) <= size (n) = size (a)
 */
int32_t q_modexp_sw(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a,
					struct q_lint *e,
					struct q_lint *n)
{
	int32_t status = Q_SUCCESS;
	struct q_mont mont;

	status = q_init(ctx, &mont.n, n->alloc);
	status += q_init(ctx, &mont.np, (n->alloc + 1));
	status += q_init(ctx, &mont.rr, (n->alloc + 1));
	if (status != Q_SUCCESS)
		goto Q_MODEXP_SW_EXIT;

	/* Step 1: Montgomery context initialization */
	q_mont_init_sw(ctx, &mont, n);

	/* Step 2: Modexp using Montgomery method */
	q_modexp_mont_sw(ctx, z, a, e, &mont);

	q_free(ctx, &mont.rr);
	q_free(ctx, &mont.np);
	q_free(ctx, &mont.n);

Q_MODEXP_SW_EXIT:
	return ctx->status;
}

/*
 * q_mont_init_sw ()
 * Description: Montgomery parameter initialization. This function
 * computes the following elements from the modulus n:
 * np, rr, where
 * r = 1 << (word_size (n) * 32),
 * rr = r ^ 2 mod n,
 * r * r_inv - n * np = 1.
 */
int32_t q_mont_init_sw(struct q_lip_ctx *ctx,
				struct q_mont *mont, struct q_lint *n)
{
	int32_t status = Q_SUCCESS;
	int i;
	uint32_t *np;
	uint32_t ns;
	int br;
	struct q_lint c1;
	struct q_lint rr;

	np = n->limb;
	ns = n->size;
	ctx->status = q_copy(&mont->n, n);
	if (ctx->status != Q_SUCCESS)
		goto Q_MONT_INIT_SW_EXIT;

	br = (n->size) << BITS_FOR_MACHINE_WD;
	mont->br = br;

	status += q_init(ctx, &c1, 1);
	status += q_init(ctx, &rr, (n->alloc * 2 + 1));
	if (status != Q_SUCCESS)
		goto Q_MONT_INIT_SW_EXIT;
	q_set_one(&c1);

#ifdef QLIP_MONT_WORD
	uint32_t n0, np0;
	uint64_t z;
	/* compute n0' = - n0 ^(-1) mod b - Dusse and Kaliski */
	z = (1L << 32);
	n0 = z - n->limb[0];
	np0 = 1;
	for (i = 1; i < 32; i++) {
		z = n0 * np0;
		if ((z % (1 << (i + 1))) >= (1 << i))
			np0 += (1 << i);
	}

	/* mont->np */
	mont->np.size = 1;
	mont->np.limb[0] = np0;

#else
	/*
	 * R = 2^br
	 * determine the size of R
	 * PKA HW requires R be constructed as follows:
	 * R = 1 << (32*(n->size)),
	 * where n->size is number of 32-bit words of n.
	 */
	rr.size = 1 + n->size;
	rr.limb[n->size] = 1L;

	/* R = R mod N */
	if (q_mod_sw(ctx, &rr, &rr, n))
		goto Q_MONT_INIT_SW_EXIT;

	/*
	 * Ri = R^-1 mod N
	 * the corresponding PKA call takes the first iteration of
	 * q_euclid outside of the main loop, therefore the corresponding
	 * section of q_euclid need to be able to accept non-default
	 * last_x and x	inputs. The firmware-only call doesn't use this
	 * method.
	 */
	if (q_modinv_sw(ctx, &mont->rr, &rr, n))
		goto Q_MONT_INIT_SW_EXIT;

	/* R * Ri - 1 */
	q_mul_2pn(&rr, &mont->rr, br);
	q_usub(&rr, &rr, &c1);

	/* Np = (R * Ri - 1) / N */
	if (q_div(ctx, &mont->np, &rr, n))
		goto Q_MONT_INIT_SW_EXIT;

	/* need to make sure that np is zero-extended to the size of n */
	if (mont->np.size < mont->n.size) {
		for (i = mont->np.size; i < mont->n.size; i++)
			mont->np.limb[i] = 0;
		mont->np.size = mont->n.size;
	}
#endif

	/* mont->rr = R ^ 2 mod N */
	q_mul_2pn(&rr, &c1, mont->br * 2);
	if (q_mod_sw(ctx, &mont->rr, &rr, n))
		goto Q_MONT_INIT_SW_EXIT;

	/* need to make sure that rr is zero-extended to the size of n */
	if (mont->rr.size < mont->n.size) {
		for (i = mont->rr.size; i < mont->n.size; i++)
			mont->rr.limb[i] = 0;
		mont->rr.size = mont->n.size;
	}

	q_free(ctx, &rr);
	q_free(ctx, &c1);

Q_MONT_INIT_SW_EXIT:
	return ctx->status;
}

/*
 * q_mont_mul_sw ()
 * Description: Montgomery Multiplication
 * Assumptions: a and b are already residue numbers
 */
int32_t q_mont_mul_sw(struct q_lip_ctx *ctx,
						 struct q_lint *z,
						 struct q_lint *a,
						 struct q_lint *b,
						 struct q_mont *mont)
{
	int32_t status = Q_SUCCESS;
	struct q_lint t, m, u;
	uint32_t st;

#ifdef QLIP_MONT_WORD
	int i, j;
	uint64_t cs;
	uint32_t ta, tb, tm, carry;
	status = q_init(ctx, &t, b->alloc + 2);
	if (status != Q_SUCCESS)
		goto Q_MONT_MUL_SW_EXIT;

	/* optimized MonPro routine based on CIOS - Cetin Kaya Koc */
	for (i = 0; i < mont->n.size; i++) {
		ta = (i >= a->size) ? 0 : a->limb[i];
		carry = 0;
		for (j = 0; j < mont->n.size; j++) {
			tb = (j >= b->size) ? 0 : b->limb[j];
			cs = t.limb[j];
			cs = q_64b_mul((uint64_t) t.limb[j], ta, tb);
			cs = q_64b_add(cs, (uint64_t) carry);
			t.limb[j] = (uint32_t) cs;
			carry = (uint32_t) (cs >> 32);
		}
		cs = q_64b_add((uint64_t) t.limb[j], (uint64_t) carry);
		t.limb[j] = (uint32_t) cs;
		carry = (uint32_t) (cs >> 32);
		t.limb[j + 1] = carry;

		carry = 0;
		tm = t.limb[0] * mont->np.limb[0];
		cs = q_64b_mul((uint64_t) t.limb[0], tm, mont->n.limb[0]);
		carry = (uint32_t) (cs >> 32);
		for (j = 1; j < mont->n.size; j++) {
			cs = q_64b_mul((uint64_t) t.limb[j], tm,
						   mont->n.limb[j]);
			cs = q_64b_add(cs, (uint64_t) carry);
			t.limb[j - 1] = (uint32_t) cs;
			carry = (uint32_t) (cs >> 32);
		}
		cs = q_64b_add((uint64_t) t.limb[j], (uint64_t) carry);
		t.limb[j - 1] = (uint32_t) cs;
		carry = (uint32_t) (cs >> 32);
		t.limb[j] = t.limb[j + 1] + carry;
	}
	t.size = mont->n.size + 1;
	Q_NORMALIZE(t.limb, t.size);
#else
	/* allocate temporary memory */
	st = a->alloc + b->alloc + mont->np.alloc;
	status = q_init(ctx, &t, st);
	status += q_init(ctx, &m, st);
	status += q_init(ctx, &u, st);
	if (status != Q_SUCCESS)
		goto Q_MONT_MUL_SW_EXIT;

	if (q_is_one(a))
		q_copy(&t, b);
	else if (q_is_one(b))
		q_copy(&t, a);
	else if (q_mul_sw(ctx, &t, a, b))
		goto Q_MONT_MUL_SW_EXIT;

	q_mod_2pn(&u, &t, (uint32_t) mont->br);

	q_mul_sw(ctx, &m, &u, &mont->np);
	q_mod_2pn(&u, &m, (uint32_t) mont->br);

	q_mul_sw(ctx, &m, &u, &mont->n);
	q_uadd(&t, &t, &m);

	q_free(ctx, &u);
	q_free(ctx, &m);

	q_div_2pn(&t, &t, (uint32_t) mont->br);
	Q_NORMALIZE(t.limb, t.size);
#endif

	if ((z->alloc < t.size) && t.limb[t.size - 1] != 1L) {
		q_abort(__FILE__, __LINE__,
				"QLIP: q_mont_mul insufficient memory!");

		ctx->status = Q_ERR_DST_OVERFLOW;
		goto Q_MONT_MUL_SW_EXIT;
	}

	if (q_ucmp(&t, &mont->n) > 0)
		q_usub(&t, &t, &mont->n);

	if (q_copy(z, &t))
		goto Q_MONT_MUL_SW_EXIT;
	q_free(ctx, &t);

Q_MONT_MUL_SW_EXIT:
	return ctx->status;
}

/*
 * q_modexp_mont_sw ()
 * Description: Montgomery modulo exponentiation
 * Assumptions: a is not a residue number. A pre and a post conversion step
 * is needed.
 */
int32_t q_modexp_mont_sw(struct q_lip_ctx *ctx,
							struct q_lint *z,
							struct q_lint *a,
							struct q_lint *e,
							struct q_mont *mont)
{
	int i, j;
	uint32_t es;
	uint32_t *zp;
	uint32_t *ep;
	struct q_lint mm;
	struct q_lint mx;
	int bitset; /* points to the current machine word */
	int msb_e;
	int32_t status = Q_SUCCESS;

	status = q_init(ctx, &mm, a->alloc);
	status += q_init(ctx, &mx, a->alloc);
	if (status != Q_SUCCESS)
		goto Q_MODEXP_MONT_SW_EXIT;

	/* convert input a to residue */
	q_mont_mul_sw(ctx, &mm, &mont->rr, a, mont);

	if (q_is_zero(e)) {
		q_set_one(z);
		goto Q_MODEXP_MONT_SW_EXIT;
	}

	if (!q_is_odd(&mont->n)) {
		q_abort(__FILE__, __LINE__,
		"QLIP: modulus must be odd number to do Montgomery modexp!");

		ctx->status = Q_ERR_GENERIC;
		goto Q_MODEXP_MONT_SW_EXIT;
	}

	zp = z->limb;
	ep = e->limb;
	es = e->size;

	q_set_one(&mx);
	if (q_mont_mul_sw(ctx, &mx, &mx, &mont->rr, mont))
		goto Q_MODEXP_MONT_SW_EXIT;

	/* LR binary method */
	if (es == 1) {
		msb_e = q_leading_one(ep[0]);
		for (j = msb_e; j >= 0; j--) {
			if (q_mont_mul_sw(ctx, &mx, &mx, &mx, mont))
				goto Q_MODEXP_MONT_SW_EXIT;
			bitset = ep[0] & (1 << j);
			if (bitset) {
				if (q_mont_mul_sw(ctx, &mx, &mx, &mm, mont))
					goto Q_MODEXP_MONT_SW_EXIT;
			}
		}
	} else {
		for (i = es - 1; i >= 0; i--) {
			for (j = MACHINE_WD - 1; j >= 0; j--) {
				if (q_mont_mul_sw(ctx, &mx, &mx, &mx, mont))
					goto Q_MODEXP_MONT_SW_EXIT;
				bitset = ep[i] & (1 << j);
				if (bitset) {
					if (q_mont_mul_sw(ctx, &mx, &mx, &mm,
									mont))
						goto Q_MODEXP_MONT_SW_EXIT;
				}
			}
		}
	}

	if (q_copy(z, &mx))
		goto Q_MODEXP_MONT_SW_EXIT;
	q_free(ctx, &mx);
	Q_NORMALIZE(zp, z->size);

	q_set_one(&mm);	/* convert result back */
	q_mont_mul_sw(ctx, z, z, &mm, mont);
	q_free(ctx, &mm);

Q_MODEXP_MONT_SW_EXIT:
	return ctx->status;
}

/*
 * q_rsa_crt_4k ()
 * Description: RSA CRT decryption.
 */

int32_t q_rsa_crt_4k(struct q_lip_ctx *ctx, struct q_lint *m,
					struct q_rsa_crt_key *rsa,
					struct q_lint *c)
{
	int32_t status = Q_SUCCESS;

#ifdef QLIP_USE_PKA_HW
	int i;
	struct q_lint m2;
	struct q_mont mont1, mont2;
	uint32_t pka_status;
	uint32_t lir_p, lir_c;
	struct opcode sequence[12];
#endif

#ifdef QLIP_USE_PKA_HW
	/* initialize hardware first */
	q_pka_hw_rst();

	/* need to initialize Montgomery context in FW */
	status = q_init(ctx, &mont1.n, rsa->p.alloc);
	status += q_init(ctx, &mont1.np, (rsa->p.alloc + 1));
	status += q_init(ctx, &mont1.rr, (rsa->p.alloc + 1));

	status += q_init(ctx, &mont2.n, rsa->q.alloc);
	status += q_init(ctx, &mont2.np, (rsa->q.alloc + 1));
	status += q_init(ctx, &mont2.rr, (rsa->q.alloc + 1));
	status += q_init(ctx, &m2, rsa->p.alloc);

	if (status != Q_SUCCESS)
		goto Q_RSA_CRT_4K_EXIT;

	if (q_mont_init(ctx, &mont1, &rsa->p))
		goto Q_RSA_CRT_4K_EXIT;
	q_pka_hw_rst();

	if (q_mont_init(ctx, &mont2, &rsa->q))
		goto Q_RSA_CRT_4K_EXIT;
	q_pka_hw_rst();

	m2.size = rsa->p.size;

	/* create command sequence */
	lir_p =
		q_pka_sel_lir((rsa->p.size >=
				rsa->q.size) ? rsa->p.size : rsa->q.size);
	lir_c = q_pka_sel_lir(c->size);

	/* compute m2 */
	for (i = 0; i < 3; i++)
		sequence[i].ptr = NULL;

	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_c, 0), c->size);
	sequence[0].ptr = c->limb;

	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), mont2.n.size);
	sequence[1].ptr = mont2.n.limb;

	/* c mod q */
	sequence[2].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MODREM, PKA_LIR(lir_p, 0),
				 PKA_LIR(lir_c, 0));
	sequence[2].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 2));

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(3, sequence);

	/* wait for completion */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_RSA_CRT_4K_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);

	for (i = 0; i < 9; i++)
		sequence[i].ptr = NULL;

	/* load exponent */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), rsa->dq.size);
	sequence[0].ptr = rsa->dq.limb;

	/* load mont */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), mont2.n.size);
	sequence[1].ptr = mont2.n.limb;

	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), mont2.np.size);
	sequence[2].ptr = mont2.np.limb;

	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), mont2.rr.size);
	sequence[3].ptr = mont2.rr.limb;

	/* convert to residue */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 0),
		PKA_LIR(lir_p, 0));
	sequence[4].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 2));

	/* m2 = (c mod q)^dq mod q */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MODEXP, PKA_LIR(lir_p, 4),
		PKA_LIR(lir_p, 0));
	sequence[5].op2 = PACK_OP2(PKA_LIR(lir_p, 1), PKA_LIR(lir_p, 2));

	/* convert m2 from q-residue to normal */
	sequence[6].op1 = PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 0), PKA_NULL);
	sequence[6].op2 = PACK_OP2(PKA_NULL, 1);

	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 0),
		PKA_LIR(lir_p, 0));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 2));

	sequence[8].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 0), m2.size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(9, sequence);

	/* wait for completion */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_RSA_CRT_4K_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);

	/* read m2 back */
	q_pka_hw_read_lir(m2.size, m2.limb);

	/* compute m1 */
	for (i = 0; i < 3; i++)
		sequence[i].ptr = NULL;

	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_c, 0), c->size);
	sequence[0].ptr = c->limb;

	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), mont1.n.size);
	sequence[1].ptr = mont1.n.limb;

	/* c mod p */
	sequence[2].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MODREM, PKA_LIR(lir_p, 0),
				 PKA_LIR(lir_c, 0));
	sequence[2].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 2));

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(3, sequence);

	/* wait for completion */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_RSA_CRT_4K_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);

	for (i = 0; i < 9; i++)
		sequence[i].ptr = NULL;

	/* load exponent */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), rsa->dp.size);
	sequence[0].ptr = rsa->dp.limb;

	/* load mont */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), mont1.n.size);
	sequence[1].ptr = mont1.n.limb;

	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), mont1.np.size);
	sequence[2].ptr = mont1.np.limb;

	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), mont1.rr.size);
	sequence[3].ptr = mont1.rr.limb;

	/* convert to residue */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 0),
		PKA_LIR(lir_p, 0));
	sequence[4].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 2));

	/* m1 = (c mod p)^dp mod p */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MODEXP, PKA_LIR(lir_p, 4),
		PKA_LIR(lir_p, 0));
	sequence[5].op2 = PACK_OP2(PKA_LIR(lir_p, 1), PKA_LIR(lir_p, 2));

	sequence[6].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MOVE, PKA_LIR(lir_p, 0),
				 PKA_LIR(lir_p, 4));
	sequence[6].op2 = 0;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(7, sequence);

	/* wait for completion */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_RSA_CRT_4K_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);

	/* continue in p-residue domain */
	for (i = 0; i < 12; i++)
		sequence[i].ptr = NULL;

	/* load m2 back in */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), m2.size);
	sequence[0].ptr = m2.limb;

	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), mont1.n.size);
	sequence[1].ptr = mont1.n.limb;

	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), mont1.np.size);
	sequence[2].ptr = mont1.np.limb;

	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), mont1.rr.size);
	sequence[3].ptr = mont1.rr.limb;

	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), rsa->qinv.size);
	sequence[4].ptr = rsa->qinv.limb;

	/* convert m2 from normal to p-residue */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 1),
		PKA_LIR(lir_p, 1));
	sequence[5].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 2));

	/* (m1-m2) mod p */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 0),
		PKA_LIR(lir_p, 0));
	sequence[6].op2 = PACK_OP2(PKA_LIR(lir_p, 1), PKA_LIR(lir_p, 2));

	/* convert qinv */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
		PKA_LIR(lir_p, 5));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 2));

	/* (m1-m2)*qinv mod p */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 0),
		PKA_LIR(lir_p, 0));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_p, 5), PKA_LIR(lir_p, 2));

	/* convert back */
	sequence[9].op1 = PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 1), PKA_NULL);
	sequence[9].op2 = PACK_OP2(PKA_NULL, 1);

	sequence[10].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MODMUL, PKA_LIR(lir_p, 0),
				 PKA_LIR(lir_p, 0));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_p, 1), PKA_LIR(lir_p, 2));

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(11, sequence);

	/* wait for completion */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_RSA_CRT_4K_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);

	for (i = 0; i < 5; i++)
		sequence[i].ptr = NULL;

	/* load m2 back in */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), rsa->q.size);
	sequence[0].ptr = rsa->q.limb;

	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), m2.size);
	sequence[1].ptr = m2.limb;

	/* long multiply ((m1-m2)*qinv mod p)*q */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_LMUL, PKA_LIR(lir_c, 0), PKA_LIR(lir_p, 0));
	sequence[2].op2 = PACK_OP2(PKA_LIR(lir_p, 1), PKA_NULL);

	/* addition */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_LADD, PKA_LIR(lir_c, 0), PKA_LIR(lir_c, 0));
	sequence[3].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_NULL);

	sequence[4].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_c, 0), c->size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(5, sequence);

	/* wait for completion */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_RSA_CRT_4K_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);

	/* read result back */
	m->size = c->size;
	q_pka_hw_read_lir(m->size, m->limb);

	q_free(ctx, &m2);

	q_free(ctx, &mont2.rr);
	q_free(ctx, &mont2.np);
	q_free(ctx, &mont2.n);

	q_free(ctx, &mont1.rr);
	q_free(ctx, &mont1.np);
	q_free(ctx, &mont1.n);
#else
	q_rsa_crt(ctx, m, rsa, c);
#endif

Q_RSA_CRT_4K_EXIT:
	return ctx->status;
}
EXPORT_SYMBOL(q_rsa_crt_4k);
