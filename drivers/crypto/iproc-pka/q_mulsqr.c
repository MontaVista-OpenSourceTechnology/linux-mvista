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
 * q_mul_2pn ()
 * z = a << bits
 * Description: Long integer left shift.
 */
int32_t q_mul_2pn(struct q_lint *z, struct q_lint *a, uint32_t bits)
{
	int32_t status = Q_SUCCESS;
	int i;
	uint32_t shift, rshift;
	uint32_t *zp;
	uint32_t *ap;
	uint32_t tmp;

	shift = (bits >> BITS_FOR_MACHINE_WD);
	rshift = bits - (shift << BITS_FOR_MACHINE_WD);

	z->size = (rshift) ? a->size + shift + 1 : a->size + shift;
	if (z->size > z->alloc) {
		q_abort(__FILE__, __LINE__,
				"QLIP: q_mul_2pn insufficient memory for the result!");
		status = Q_ERR_DST_OVERFLOW;
		goto Q_MUL_2PN_EXIT;
	}

	zp = z->limb;
	ap = a->limb;

	for (i = 0; i < shift; i++)
		zp[i] = 0;

	if (rshift) {
		tmp = 0;
		for (i = shift; i < z->size - 1; i++) {
			zp[i] = (ap[i - shift] << rshift) + tmp;
			tmp = ap[i - shift] >> (MACHINE_WD - rshift);
		}
		zp[i] = tmp;
	} else {
		for (i = shift; i < z->size; i++)
			zp[i] = ap[i - shift];
	}

	Q_NORMALIZE(z->limb, z->size);

Q_MUL_2PN_EXIT:
	return status;
}

#ifndef REDUCED_RELEASE_CODE_SIZE
/*
 * q_sqr ()
 * z = a ^ 2
 * Description: Long integer squaring.
 */
int q_sqr(struct q_lip_ctx *ctx, struct q_lint *z, struct q_lint *a)
{
	struct q_lint *r;
	struct q_lint rr;
	uint32_t zs;

#ifdef QLIP_USE_GMP
	mpz_t ma, mr;
#endif

#ifdef QLIP_USE_PKA_HW
	uint32_t pka_status;
	struct opcode sequence[3];
	uint32_t lir_x;
	uint32_t lir_r;
#endif

	if (a->size == 0) {
		q_set_zero(z);
		goto Q_SQR_EXIT;
	}

	zs = a->size * 2;

	if (z->alloc < zs) {
		q_abort(__FILE__, __LINE__,
				"QLIP: q_mul insufficient memory for the result!");

		ctx->status = Q_ERR_DST_OVERFLOW;
		goto Q_SQR_EXIT;
	}

	/* if the result is the same as one of the operand */
	if (z == a) {
		if (q_init(ctx, &rr, zs))
			goto Q_SQR_EXIT;
		r = &rr;
	} else
		r = z;

	r->size = zs;

#ifdef QLIP_USE_GMP
	mpz_init(ma);
	mpz_init(mr);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_mul(mr, ma, ma);
	mpz_export(r->limb, &(r->size), -1, 4, 0, 0, mr);

	mpz_clear(ma);
	mpz_clear(mr);
#else /* QLIP_USE_GMP */

#ifdef QLIP_USE_PKA_HW
	lir_x = PKA_LIR_H;
	lir_r = PKA_LIR_J;

	/* H[0] = a */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 0), a->size);
	sequence[0].ptr = a->limb;

	/* J[0] = H[0] ^ 2 */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_LSQR, PKA_LIR(lir_r, 0), PKA_LIR(lir_x, 0));
	sequence[1].ptr = NULL;

	/* unload J[0] */
	sequence[2].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_r, 0), r->size);
	sequence[2].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_SQR_EXIT;
	}
	q_pka_hw_write_sequence(3, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_SQR_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_SQR_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(r->size, r->limb);

#else /* QLIP_USE_PKA_HW */

	int i, j;
	uint32_t k;
	uint64_t t;

	/* M1: Initialize*/
	for (i = 0; i < a->size; i++)
		r->limb[i] = 0;

	/* M6: Loop on j*/
	for (j = 0; j < a->size; j++) {

		/* M2: Zero multiplier?*/
		if (a->limb[j] == 0) {
			r->limb[j + a->size] = 0;
			continue;
		}
		/* M3: Initialize k*/
		k = 0;

		/* M5a: Loop on i*/
		for (i = 0; i < a->size; i++) {

			/*
			 * M4: Multiply and add: t = a[i] * b[j] + r[i+j] + k
			 * Set r[i+j] = t mod (1<<32 - 1)
			 * Set k = t >> 32
			 * Note: All inputs are 32-bit numbers. Therefore,
			 * the operations in this step are all 64-bit wide.
			 * The math guarantees that the final result, t,
			 * is bounded by 64 bits.
			 */
			t = (uint64_t) r->limb[i + j];
			t = q_64b_mul(t, a->limb[i], a->limb[j]);
			t = q_64b_add(t, (uint64_t) k);
			r->limb[i + j] = (uint32_t) t;
			k = (uint32_t) (t >> 32);
		}

		/* M5b: Set r[j+m] = k*/
		r->limb[j + a->size] = k;
	}

	Q_NORMALIZE(r->limb, r->size);

#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_GMP */

	if (r != z) {
		q_copy(z, r);
		q_free(ctx, &rr);
	}
	z->neg = 0;

Q_SQR_EXIT:
	return ctx->status;
}

/*
 * q_modsqr ()
 * z = (a ^ 2) % n
 * Description: Long integer modulo squaring.
 */
int32_t q_modsqr(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *n)
{
#ifdef QLIP_USE_PKE_HW
	struct q_lint ta, tn;
#endif

#ifdef QLIP_USE_PKA_HW
	uint32_t pka_status;
	struct opcode sequence[5];
	uint32_t lir_x;
	uint32_t lir_r;
#endif

#ifdef QLIP_USE_GMP
	mpz_t ma, mn, mw, mz;
#endif

#ifdef QLIP_USE_PKE_HW
	q_init(ctx, &ta, n->alloc);
	q_init(ctx, &tn, n->alloc);
	q_copy(&ta, a);
	q_copy(&tn, n);
	q_modmul_hw(z->limb, n->size * MACHINE_WD, tn.limb, ta.limb, ta.limb);
	z->size = n->size;
	Q_NORMALIZE(z->limb, z->size);
	q_free(ctx, &tn);
	q_free(ctx, &ta);
#else /* QLIP_USE_PKE_HW */

#ifdef QLIP_USE_PKA_HW
	lir_x = PKA_LIR_H;
	lir_r = PKA_LIR_J;

	if (z->alloc < n->size) {
		ctx->status = Q_ERR_DST_OVERFLOW;
		goto Q_MODSQR_EXIT;
	}

	z->size = n->size;

	/* H[0] = a */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 0), a->size);
	sequence[0].ptr = a->limb;

	/* H[2] = n */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 2), n->size);
	sequence[1].ptr = n->limb;

	/* J[0] = H[0] ^ 2 */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_LSQR, PKA_LIR(lir_r, 0), PKA_LIR(lir_x, 0));
	sequence[2].ptr = NULL;

	/* H[0] = J[0] mod H[2] */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MODREM, PKA_LIR(lir_x, 0),
				PKA_LIR(lir_r, 0));
	sequence[3].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_x, 2));
	sequence[3].ptr = NULL;

	/* unload H[0] */
	sequence[4].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_x, 0), z->size);
	sequence[4].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_MODSQR_EXIT;
	}
	q_pka_hw_write_sequence(5, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_MODSQR_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_MODSQR_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(z->size, z->limb);

#else /* QLIP_USE_PKA_HW */
#ifdef QLIP_USE_GMP
	mpz_init(ma);
	mpz_init(mn);
	mpz_init(mz);
	mpz_init(mw);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_import(mn, n->size, -1, 4, 0, 0, n->limb);
	mpz_import(mz, z->size, -1, 4, 0, 0, z->limb);

	mpz_mul(mw, ma, ma);
	mpz_mod(mz, mw, mn);

	mpz_export(z->limb, &(z->size), -1, 4, 0, 0, mz);
	mpz_clear(ma);
	mpz_clear(mn);
	mpz_clear(mz);
	mpz_clear(mw);

#else /* QLIP_USE_GMP */
	uint32_t size;
	struct q_lint p;
	size = a->alloc * 2;
	if (q_init(ctx, &p, size))
		goto Q_MODSQR_EXIT;

	if (q_sqr(ctx, &p, a))
		goto Q_MODSQR_EXIT;
	if (q_mod(ctx, z, &p, n))
		goto Q_MODSQR_EXIT;

	q_free(ctx, &p);
#endif /* QLIP_USE_GMP */
#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_PKE_HW */

Q_MODSQR_EXIT:
	return ctx->status;
}

/*
 * q_modmul_pb ()
 * z = (a*b) % n
 * Description: Paul Barrett's method of modmul and modsqr
 */
int32_t q_modmul_pb(struct q_lip_ctx *ctx,
					   struct q_lint *z,
					   struct q_lint *r, struct q_lint *a,
					   struct q_lint *b, struct q_lint *n)
{
#ifdef QLIP_USE_GMP
	mpz_t ma, mb, mn, mr, mw;
#else
	/* create temporary storage */
	uint32_t *yp;
	struct q_lint w;
	struct q_lint y;
#endif

#ifdef QLIP_USE_GMP
	mpz_init(ma);
	mpz_init(mb);
	mpz_init(mn);
	mpz_init(mr);
	mpz_init(mw);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_import(mb, b->size, -1, 4, 0, 0, b->limb);
	mpz_import(mn, n->size, -1, 4, 0, 0, n->limb);
	mpz_import(mr, r->size, -1, 4, 0, 0, r->limb);
	/* multiplication */
	mpz_mul(mw, ma, mb);

	/* mod reduction */
	mpz_t my;
	mpz_t mc1;
	mpz_init(my);
	mpz_init(mc1);

	mpz_div_2exp(my, mw, (mpz_size(mn) * MACHINE_WD));
	mpz_mul(my, my, mr);
	mpz_div_2exp(my, my, (mpz_size(mn) * MACHINE_WD));
	mpz_mul(my, my, mn);

	mpz_set_si(mc1, 1);
	mpz_mul_2exp(mc1, mc1, ((mpz_size(mn) + 1) * MACHINE_WD));
	mpz_mod(mw, mw, mc1);

	mpz_mod(my, my, mc1);
	mpz_sub(mw, mw, my);

	if (!mpz_sgn(mw))
		mpz_add(mw, mw, mc1);

	while (mpz_cmp(mw, mn) > 0)
		mpz_sub(mw, mw, mn);

	mpz_export(z->limb, &(z->size), -1, 4, 0, 0, mw);
	mpz_clear(ma);
	mpz_clear(mb);
	mpz_clear(mn);
	mpz_clear(mr);
	mpz_clear(mw);
	mpz_clear(my);
	mpz_clear(mc1);
#else
	q_init(ctx, &w, (n->alloc * 2));
	q_init(ctx, &y, (n->alloc * 2 + 1));

	/* multiplication */
	q_mul(ctx, &w, a, b);

	/* Barrett modulo reduction */
	q_div_2pn(&y, &w, n->size * MACHINE_WD);
	q_mul(ctx, &y, &y, r);
	q_div_2pn(&y, &y, n->size * MACHINE_WD);
	q_mul(ctx, &y, &y, n);

	q_mod_2pn(&y, &y, (n->size + 1) * MACHINE_WD);
	q_mod_2pn(&w, &w, (n->size + 1) * MACHINE_WD);

	q_sub(&w, &w, &y);

	/* w = w + 2^n */
	if (w.neg) {
		q_set_zero(&y);
		y.size = (n->size + 1);
		yp = y.limb;
		yp[n->size] = 1;
		q_uadd(&w, &w, &y);
	}

	while (!w.neg)
		q_sub(&w, &w, n);

	q_add(&w, &w, n);
	q_copy(z, &w);
	q_free(ctx, &y);
	q_free(ctx, &w);
#endif

	return Q_SUCCESS;
}

/*
 * q_modsar_pb ()
 * z = (a^2) % n
 * Description: Paul Barrett's method of modsqr
 */
int32_t q_modsqr_pb(struct q_lip_ctx *ctx,
					   struct q_lint *z, struct q_lint *r,
					   struct q_lint *a, struct q_lint *n)
{

#ifdef QLIP_USE_GMP
	mpz_t ma, mn, mr, mw;
#else
	/* create temporary storage */
	uint32_t *yp;
	struct q_lint w;
	struct q_lint y;
#endif

#ifdef QLIP_USE_GMP
	mpz_init(ma);
	mpz_init(mn);
	mpz_init(mr);
	mpz_init(mw);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_import(mn, n->size, -1, 4, 0, 0, n->limb);
	mpz_import(mr, r->size, -1, 4, 0, 0, r->limb);
	/* multiplication */
	mpz_mul(mw, ma, ma);

	/* mod reduction */
	mpz_t my;
	mpz_t mc1;
	mpz_init(my);
	mpz_init(mc1);

	mpz_div_2exp(my, mw, (mpz_size(mn) * MACHINE_WD));
	mpz_mul(my, my, mr);
	mpz_div_2exp(my, my, (mpz_size(mn) * MACHINE_WD));
	mpz_mul(my, my, mn);

	mpz_set_si(mc1, 1);
	mpz_mul_2exp(mc1, mc1, ((mpz_size(mn) + 1) * MACHINE_WD));
	mpz_mod(mw, mw, mc1);

	mpz_mod(my, my, mc1);
	mpz_sub(mw, mw, my);

	if (!mpz_sgn(mw))
		mpz_add(mw, mw, mc1);

	while (mpz_cmp(mw, mn) > 0)
		mpz_sub(mw, mw, mn);

	mpz_export(z->limb, &(z->size), -1, 4, 0, 0, mw);
	mpz_clear(ma);
	mpz_clear(mn);
	mpz_clear(mr);
	mpz_clear(mw);
	mpz_clear(my);
	mpz_clear(mc1);
#else
	q_init(ctx, &w, (n->alloc * 2));
	q_init(ctx, &y, (n->alloc * 2 + 1));

	/* squaring */
	q_sqr(ctx, &w, a);

	/* Barrett modulo reduction */
	q_div_2pn(&y, &w, n->size * MACHINE_WD);
	q_mul(ctx, &y, &y, r);
	q_div_2pn(&y, &y, n->size * MACHINE_WD);
	q_mul(ctx, &y, &y, n);

	q_mod_2pn(&y, &y, (n->size + 1) * MACHINE_WD);
	q_mod_2pn(&w, &w, (n->size + 1) * MACHINE_WD);

	q_sub(&w, &w, &y);

	/* w = w + 2^n */
	if (w.neg) {
		q_set_zero(&y);
		y.size = (n->size + 1);
		yp = y.limb;
		yp[n->size] = 1;
		q_uadd(&w, &w, &y);
	}

	while (!w.neg)
		q_sub(&w, &w, n);

	q_add(&w, &w, n);
	q_copy(z, &w);
	q_free(ctx, &y);
	q_free(ctx, &w);
#endif

	return Q_SUCCESS;
}
#endif /*REDUCED_RELEASE_CODE_SIZE */

/*
 * q_mul ()
 * z = a * b
 * Description: Long integer multiplication.
 */
int32_t q_mul(struct q_lip_ctx *ctx, struct q_lint *z,
				struct q_lint *a, struct q_lint *b)
{
	struct q_lint *r;
	struct q_lint rr;
	uint32_t zs;

#ifdef QLIP_USE_GMP
	mpz_t ma, mb, mr;
#endif

#ifdef QLIP_USE_PKA_HW
	uint32_t pka_status;
	struct opcode sequence[4];
	uint32_t lir_x;
	uint32_t lir_r;
#endif

	if ((a->size == 0) || (b->size == 0)) {
		q_set_zero(z);
		goto Q_MUL_EXIT;
	}

	zs = a->size + b->size;

	if (z->alloc < zs) {
		q_abort(__FILE__, __LINE__,
				"QLIP: q_mul insufficient memory for the result!");

		ctx->status = Q_ERR_DST_OVERFLOW;
		goto Q_MUL_EXIT;
	}

	/* if the result is the same as one of the operand */
	if ((z == a) || (z == b)) {
		if (q_init(ctx, &rr, zs))
			goto Q_MUL_EXIT;
		r = &rr;
	} else
		r = z;

	r->size = zs;

#ifdef QLIP_USE_GMP
	mpz_init(ma);
	mpz_init(mb);
	mpz_init(mr);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_import(mb, b->size, -1, 4, 0, 0, b->limb);

	mpz_mul(mr, ma, mb);
	mpz_export(r->limb, &(r->size), -1, 4, 0, 0, mr);

	mpz_clear(ma);
	mpz_clear(mb);
	mpz_clear(mr);
#else /* QLIP_USE_GMP */

#ifdef QLIP_USE_PKA_HW
	lir_x = q_pka_sel_lir((a->size >= b->size) ? a->size : b->size);
	lir_r = q_pka_sel_lir(r->size);

	/* H[0] = a */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 0), a->size);
	sequence[0].ptr = a->limb;

	/* H[1] = b */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 1), b->size);
	sequence[1].ptr = b->limb;

	/* J[0] = H[0] * H[1] */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_LMUL, PKA_LIR(lir_r, 0),
				PKA_LIR(lir_x, 0));
	sequence[2].op2 = PACK_OP2(PKA_LIR(lir_x, 1), PKA_NULL);
	sequence[2].ptr = NULL;

	/* unload J[0] */
	sequence[3].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_r, 0), r->size);
	sequence[3].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_MUL_EXIT;
	}
	q_pka_hw_write_sequence(4, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_MUL_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_MUL_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(r->size, r->limb);

#else /* QLIP_USE_PKA_HW */
	int i, j;
	uint32_t k;
	uint64_t t;
	/* M1: Initialize*/
	for (i = 0; i < a->size; i++)
		r->limb[i] = 0;

	/* M6: Loop on j*/
	for (j = 0; j < b->size; j++) {

		/* M2: Zero multiplier?*/
		if (b->limb[j] == 0) {
			r->limb[j + a->size] = 0;
			continue;
		}
		/* M3: Initialize k */
		k = 0;

		/* M5a: Loop on i */
		/* M4: Multiply and add: t = a[i] * b[j] + r[i+j] + k
		 * Set r[i+j] = t mod (1<<32 - 1)
		 * Set k = t >> 32
		 * Note: All inputs are 32-bit numbers. Therefore,
		 * the operations in this step are all 64-bit wide.
		 * The math guarantees that the
		 * final result, t, is bounded by 64 bits.
		 */
		for (i = 0; i < a->size; i++) {
			t = (uint64_t) r->limb[i + j];
			t = q_64b_mul(t, a->limb[i], b->limb[j]);
			t = q_64b_add(t, (uint64_t) k);
			r->limb[i + j] = (uint32_t) t;
			k = (uint32_t) (t >> 32);
		}

		/* M5b: Set r[j+m] = k*/
		r->limb[j + a->size] = k;
	}

	Q_NORMALIZE(r->limb, r->size);

#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_GMP */

	if (r != z) {
		q_copy(z, r);
		q_free(ctx, &rr);
	}
	z->neg = a->neg ^ b->neg;

Q_MUL_EXIT:
	return ctx->status;
}

/*
 * q_mul_sw ()
 * z = a * b
 * Description: Long integer multiplication.
 * create a software only version for 4K RSA
 */
int32_t q_mul_sw(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *b)
{
	struct q_lint *r;
	struct q_lint rr;
	uint32_t zs;
	int i, j;
	uint32_t k;
	uint64_t t;

	if ((a->size == 0) || (b->size == 0)) {
		q_set_zero(z);
		goto Q_MUL_SW_EXIT;
	}

	zs = a->size + b->size;

	if (z->alloc < zs) {
		q_abort(__FILE__, __LINE__,
				"QLIP: q_mul insufficient memory for the result!");
		ctx->status = Q_ERR_DST_OVERFLOW;
		goto Q_MUL_SW_EXIT;
	}

	/* if the result is the same as one of the operand */
	if ((z == a) || (z == b)) {
		if (q_init(ctx, &rr, zs))
			goto Q_MUL_SW_EXIT;
		r = &rr;
	} else
		r = z;

	r->size = zs;

	/*
	 * The following code implements Algorithm 4.3.1M in
	 * The Art of Computer Programming Vol 2, 3rd Ed
	 * D. Knuth
	 */
	/* M1: Initialize*/
	for (i = 0; i < a->size; i++)
		r->limb[i] = 0;

	/* M6: Loop on j*/
	for (j = 0; j < b->size; j++) {

		/* M2: Zero multiplier?*/
		if (b->limb[j] == 0) {
			r->limb[j + a->size] = 0;
			continue;
		}
		/* M3: Initialize k*/
		k = 0;

		/* M5a: Loop on i*/
		for (i = 0; i < a->size; i++) {
			/*
			 * M4: Multiply and add:
			 * t = a[i] * b[j] + r[i+j] + k
			 * Set r[i+j] = t mod (1<<32 - 1)
			 * Set k = t >> 32
			 * Note: All inputs are 32-bit numbers. Therefore,
			 * the operations in this step are all 64-bit wide.
			 * The math guarantees that the final result, t,
			 * is bounded by 64 bits.
			 */
			t = (uint64_t) r->limb[i + j];
			t = q_64b_mul(t, a->limb[i], b->limb[j]);
			t = q_64b_add(t, (uint64_t) k);
			r->limb[i + j] = (uint32_t) t;
			k = (uint32_t) (t >> 32);
		}

		/* M5b: Set r[j+m] = k*/
		r->limb[j + a->size] = k;
	}

	Q_NORMALIZE(r->limb, r->size);

	if (r != z) {
		q_copy(z, r);
		q_free(ctx, &rr);
	}
	z->neg = a->neg ^ b->neg;

Q_MUL_SW_EXIT:
	return ctx->status;
}

/*
 * q_modmul ()
 * z = (a * b) % n
 * Description: Long integer modulo multiplication.
 */
int32_t q_modmul(struct q_lip_ctx *ctx,
					struct q_lint *z, struct q_lint *a,
					struct q_lint *b, struct q_lint *n)
{
#ifdef QLIP_USE_PKE_HW
	struct q_lint ta, tb, tn;
#endif

#ifdef QLIP_USE_PKA_HW
	uint32_t pka_status;
	struct opcode sequence[6];
	uint32_t lir_x;
	uint32_t lir_r;
#endif

#ifdef QLIP_USE_GMP
	mpz_t ma, mb, mn, mw, mz;
#endif

#ifdef QLIP_USE_PKE_HW
	q_init(ctx, &ta, n->alloc);
	q_init(ctx, &tb, n->alloc);
	q_init(ctx, &tn, n->alloc);
	q_copy(&ta, a);
	q_copy(&tb, b);
	q_copy(&tn, n);
	q_modmul_hw(z->limb, n->size * MACHINE_WD, tn.limb,
				ta.limb, tb.limb);
	z->size = n->size;
	Q_NORMALIZE(z->limb, z->size);
	q_free(ctx, &tn);
	q_free(ctx, &tb);
	q_free(ctx, &ta);
#else /* QLIP_USE_PKE_HW */

#ifdef QLIP_USE_PKA_HW
	lir_x = PKA_LIR_H;
	lir_r = PKA_LIR_J;

	if (z->alloc < n->size) {
		ctx->status = Q_ERR_DST_OVERFLOW;
		goto Q_MODMUL_EXIT;
	}

	z->size = n->size;

	/* H[0] = a */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 0), a->size);
	sequence[0].ptr = a->limb;

	/* H[1] = b */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 1), b->size);
	sequence[1].ptr = b->limb;

	/* H[2] = n */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 2), n->size);
	sequence[2].ptr = n->limb;

	/* J[0] = H[0] * H[1] */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_LMUL, PKA_LIR(lir_r, 0), PKA_LIR(lir_x, 0));
	sequence[3].op2 = PACK_OP2(PKA_LIR(lir_x, 1), PKA_NULL);
	sequence[3].ptr = NULL;

	/* H[0] = J[0] mod H[2] */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MODREM, PKA_LIR(lir_x, 0),
				PKA_LIR(lir_r, 0));
	sequence[4].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_x, 2));
	sequence[4].ptr = NULL;

	/* unload H[0] */
	sequence[5].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_x, 0), z->size);
	sequence[5].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_MODMUL_EXIT;
	}
	q_pka_hw_write_sequence(6, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_MODMUL_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_MODMUL_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(z->size, z->limb);

#else /* QLIP_USE_PKA_HW */

#ifdef QLIP_USE_GMP
	mpz_init(ma);
	mpz_init(mb);
	mpz_init(mn);
	mpz_init(mw);
	mpz_init(mz);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_import(mb, b->size, -1, 4, 0, 0, b->limb);
	mpz_import(mn, n->size, -1, 4, 0, 0, n->limb);

	mpz_mul(mw, ma, mb);
	mpz_mod(mz, mw, mn);

	mpz_export(z->limb, &(z->size), -1, 4, 0, 0, mz);
	mpz_clear(ma);
	mpz_clear(mb);
	mpz_clear(mn);
	mpz_clear(mw);
	mpz_clear(mz);

#else /* QLIP_USE_GMP */
	uint32_t size;
	struct q_lint p;
	size = a->alloc + b->alloc;
	if (q_init(ctx, &p, size))
		goto Q_MODMUL_EXIT;

	if (q_mul(ctx, &p, a, b))
		goto Q_MODMUL_EXIT;
	if (q_mod(ctx, z, &p, n))
		goto Q_MODMUL_EXIT;
	q_free(ctx, &p);
#endif /* QLIP_USE_GMP */
#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_PKE_HW */

Q_MODMUL_EXIT:
	return ctx->status;
}
