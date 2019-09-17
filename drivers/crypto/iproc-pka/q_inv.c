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
 * q_modpinv ()
 * z = a ^ (p - 2) mod p
 * Description: this is a special inverse function using Fermat's
 * Little Theorem when p is prime
 */
int32_t q_modpinv(struct q_lip_ctx *ctx,
		 struct q_lint *z, struct q_lint *a, struct q_lint *p)
{

	int32_t status = Q_SUCCESS;

#ifdef QLIP_USE_PKE_HW
	struct q_lint ta, tp;
#endif

#ifdef QLIP_USE_PKA_HW
	struct q_mont mont;
	uint32_t pka_status;
	uint32_t lir_p;
	struct opcode sequence[9];
#endif

#ifdef QLIP_USE_PKE_HW
	q_init(ctx, &ta, p->alloc);
	q_init(ctx, &tp, p->alloc);
	q_copy(&ta, a);
	q_copy(&tp, p);
	q_modinv_hw(z->limb, p->size * MACHINE_WD, tp.limb, ta.limb);
	z->size = p->size;
	Q_NORMALIZE(z->limb, z->size);
	q_free(ctx, &tp);
	q_free(ctx, &ta);
#else /* QLIP_USE_PKE_HW */
#ifdef QLIP_USE_PKA_HW
	/* initialize hardware first */
	q_pka_hw_rst();

	status = q_init(ctx, &mont.n, p->alloc);
	status += q_init(ctx, &mont.np, p->alloc + 1);
	status += q_init(ctx, &mont.rr, p->alloc + 1);
	if (status != Q_SUCCESS)
		goto Q_MODPINV_EXIT;

	if (q_mont_init(ctx, &mont, p))
		goto Q_MODPINV_EXIT;
	q_pka_hw_rst();

	/* sending command sequence */
	lir_p = q_pka_sel_lir(p->size);

	/* load modulus n x[0] = n.n */
	sequence[0].op1 =
	PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0), mont.n.size);
	sequence[0].ptr = mont.n.limb;

	/*
	 * load montgomery parameter np
	 * x[1] = n.np
	 */
	sequence[1].op1 =
	PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1), mont.np.size);
	sequence[1].ptr = mont.np.limb;

	/*
	 * load montgomery parameter rr
	 * x[2] = n.rr
	 */
	sequence[2].op1 =
	PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2), mont.rr.size);
	sequence[2].ptr = mont.rr.limb;

	/*
	 * load base a
	 * x[3] = a (base)
	 */
	sequence[3].op1 =
	PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), a->size);
	sequence[3].ptr = a->limb;

	/*
	 * convert base a to n-residue domain
	 * x[3] = x[3] * x[2] mod x[0]
	 */
	sequence[4].op1 =
	PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 3), PKA_LIR(lir_p, 3));
	sequence[4].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));
	sequence[4].ptr = NULL;

	/*
	 * perform montgomery modinv
	 * x[2] = x[3] ^ (x[0] - 2) mod x[0]
	 */
	sequence[5].op1 =
	PACK_OP1(0, PKA_OP_MODINV, PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 3));
	sequence[5].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));
	sequence[5].ptr = NULL;

	/*
	 * set LIR = 1
	 * x[3] = 1
	 */
	sequence[6].op1 = PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 3), PKA_NULL);
	sequence[6].op2 = PACK_OP2(PKA_NULL, 1);
	sequence[6].ptr = NULL;

	/*
	 * convert result back to regular domain
	 * x[2] = x[2] * x[3] mod x[0]
	 */
	sequence[7].op1 =
	PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 2));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_p, 3), PKA_LIR(lir_p, 0));
	sequence[7].ptr = NULL;

	/* read back result */
	sequence[8].op1 =
	PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 2), p->size);
	sequence[8].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(9, sequence);

	q_free(ctx, &mont.rr);
	q_free(ctx, &mont.np);
	q_free(ctx, &mont.n);

	z->size = p->size;

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_MODPINV_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(z->size, z->limb);

#else /* QLIP_USE_PKA_HW */
#ifdef QLIP_USE_GMP
	mpz_t ma, mp, mz;

	mpz_init(ma);
	mpz_init(mp);
	mpz_init(mz);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_import(mp, p->size, -1, 4, 0, 0, p->limb);

	mpz_set_si(mz, 2);
	mpz_sub(mz, mp, mz);
	mpz_powm(mz, ma, mz, mp);
	mpz_export(z->limb, &z->size, -1, 4, 0, 0, mz);

	mpz_clear(ma);
	mpz_clear(mp);
	mpz_clear(mz);

#else
	struct q_lint tmp;
	if (q_init(ctx, &tmp, p->alloc))
		goto Q_MODPINV_EXIT;

	tmp.size = 1;
	tmp.limb[0] = 2L;
	q_usub(&tmp, p, &tmp);

	if (q_modexp(ctx, z, a, &tmp, p))
		goto Q_MODPINV_EXIT;
	q_free(ctx, &tmp);
#endif /* QLIP_USE_GMP */
#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_PKE_HW */

Q_MODPINV_EXIT:
	return ctx->status;
}

#ifndef REDUCED_RELEASE_CODE_SIZE
/*
 * q_gcd ()
 * Description: non-recursive GCD
 */
int32_t q_gcd(struct q_lip_ctx *ctx, struct q_lint *z,
			struct q_lint *a, struct q_lint *b)
{
	struct q_lint tmp;

	if (a->alloc < b->size) {
		ctx->status = Q_ERR_DST_OVERFLOW;
		goto Q_GCD_EXIT;
	}

	if (q_init(ctx, &tmp, b->alloc))
		goto Q_GCD_EXIT;

	q_copy(&tmp, a);
	while (!q_is_zero(b)) {
		if (q_mod(ctx, &tmp, a, b))
			goto Q_GCD_EXIT;
		q_copy(a, b);
		q_copy(b, &tmp);
	}

	ctx->status = q_copy(z, a);
	if (ctx->status != Q_SUCCESS)
		goto Q_GCD_EXIT;
	q_free(ctx, &tmp);

Q_GCD_EXIT:
	return ctx->status;
}
#endif /* REDUCED_RELEASE_CODE_SIZE */

/*
 * q_euclide ()
 * Description: Extended Euclidian Algorithm
 * d = gcd (a, b) and d = a * x + b * y
 * assume size(b) == size(a), a > 0, b > 0
 * return value is x
 */
int32_t q_euclid(struct q_lip_ctx *ctx,
			struct q_lint *x, struct q_lint *lx,
			struct q_lint *a, struct q_lint *b)
{
#ifdef QLIP_USE_PKA_HW
	uint32_t lir_x;
	uint32_t pka_status;
	struct opcode sequence[16];
	struct q_lint c1;
#endif

#ifdef QLIP_USE_PKA_HW
	if (q_init(ctx, &c1, b->size))
		goto Q_EUCLID_EXIT;
	c1.limb[0] = 1;

	if (lx == NULL) {
		lx = &c1;
		q_set_zero(x);
		x->neg = 0;
		x->size = b->size;
	}

	/* create command sequence */
	lir_x = q_pka_sel_lir(b->size);

	/* x[0] = a */
	sequence[0].op1 =
	PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 0), a->size);
	sequence[0].ptr = a->limb;

	/* x[1] = b */
	sequence[1].op1 =
	PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 1), b->size);
	sequence[1].ptr = b->limb;

	/* x[2] = lx */
	sequence[2].op1 =
	PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 2), b->size);
	sequence[2].ptr = lx->limb;

	/* x[3] = x */
	sequence[3].op1 =
	PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_x, 3), b->size);
	sequence[3].ptr = x->limb;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(4, sequence);

	/* x[4] = x[1]  (tmp = b) */
	sequence[0].op1 =
	PACK_OP1(0, PKA_OP_MOVE, PKA_LIR(lir_x, 4), PKA_LIR(lir_x, 1));
	sequence[0].op2 = 0;
	sequence[0].ptr = NULL;

	/* x[5] = x[0] / x[1] (q = a / b) */
	sequence[1].op1 =
	PACK_OP1(0, PKA_OP_LDIV, PKA_LIR(lir_x, 5), PKA_LIR(lir_x, 0));
	sequence[1].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_x, 1));
	sequence[1].ptr = NULL;

	/* resize x[5] (normalize q) */
	sequence[2].op1 =
	PACK_OP1(0, PKA_OP_RESIZE, PKA_LIR(lir_x, 5), PKA_LIR(lir_x, 5));
	sequence[2].op2 = 0;
	sequence[2].ptr = NULL;

	/* x[0] = x[0] % x[1] (a = a % b) */
	sequence[3].op1 =
	PACK_OP1(0, PKA_OP_MODREM, PKA_LIR(lir_x, 0), PKA_LIR(lir_x, 0));
	sequence[3].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_x, 1));
	sequence[3].ptr = NULL;

	/* x[1] = x[0] (b = a) */
	sequence[4].op1 =
	PACK_OP1(0, PKA_OP_MOVE, PKA_LIR(lir_x, 1), PKA_LIR(lir_x, 0));
	sequence[4].op2 = 0;
	sequence[4].ptr = NULL;

	/* x[0] = x[4] (a = tmp) */
	sequence[5].op1 =
	PACK_OP1(0, PKA_OP_MOVE, PKA_LIR(lir_x, 0), PKA_LIR(lir_x, 4));
	sequence[5].op2 = 0;
	sequence[5].ptr = NULL;

	/* x[4] = x[3] (tmp = x) */
	sequence[6].op1 =
	PACK_OP1(0, PKA_OP_MOVE, PKA_LIR(lir_x, 4), PKA_LIR(lir_x, 3));
	sequence[6].op2 = 0;
	sequence[6].ptr = NULL;

	/* x[3] = x[3] * x[5] (x = x * q) */
	sequence[7].op1 =
	PACK_OP1(0, PKA_OP_LMUL, PKA_LIR(lir_x, 3), PKA_LIR(lir_x, 3));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_x, 5), PKA_NULL);
	sequence[7].ptr = NULL;

	/* x[3] = x[2] + x[3] (x = lx + x) */
	sequence[8].op1 =
	PACK_OP1(0, PKA_OP_LADD, PKA_LIR(lir_x, 3), PKA_LIR(lir_x, 2));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_x, 3), PKA_NULL);
	sequence[8].ptr = NULL;

	/* x[2] = x[4] (lx = tmp) */
	sequence[9].op1 =
	PACK_OP1(0, PKA_OP_MOVE, PKA_LIR(lir_x, 2), PKA_LIR(lir_x, 4));
	sequence[9].op2 = 0;
	sequence[9].ptr = NULL;

	/* x[4] = 0 (tmp = 0) */
	sequence[10].op1 =
	PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_x, 4), PKA_NULL);
	sequence[10].op2 = PACK_OP2(PKA_NULL, 0);
	sequence[10].ptr = NULL;

	/* ? x[4] >= x[1]  (? b == 0) */
	sequence[11].op1 =
	PACK_OP1(PKA_EOS, PKA_OP_LCMP, PKA_NULL, PKA_LIR(lir_x, 4));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_x, 1), PKA_NULL);
	sequence[11].ptr = NULL;

LOOP_EEA:
	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(12, sequence);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_EUCLID_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);

	/* invert output sign */
	x->neg = x->neg ? 0 : 1;

	if ((q_pka_hw_rd_status() & PKA_STAT_CARRY))
		goto LOOP_EEA;

	/* unload x[0] (a) */
	sequence[0].op1 =
	PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_x, 0), b->size);
	sequence[0].ptr = NULL;

	/* unload x[2] (lx) */
	sequence[1].op1 =
	PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_x, 2), b->size);
	sequence[1].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(2, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		/*
		 * While we wait for the the first unload opcode, we want to
		 * monitor the CMD_ERR bit in the status register, as the math
		 * opcodes before the first unload opcode may trigger PKA HW
		 * related error. We do not need to monitor the CMD_ERR for the
		 * subsequent unload opcode.
		 */
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
				goto Q_EUCLID_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(b->size, a->limb);

	x->size = b->size;
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(x->size, x->limb);

	Q_NORMALIZE(a->limb, a->size);

	q_free(ctx, &c1);
#else
#ifdef QLIP_USE_GMP
	mpz_t mlx, mx, mly, my, ma, mb, mq, mt;
	mpz_init(mlx);
	mpz_init(mx);
	mpz_init(mly);
	mpz_init(my);
	mpz_init(ma);
	mpz_init(mb);
	mpz_init(mq);
	mpz_init(mt);

	mpz_set_si(mlx, 0);
	mpz_set_si(mly, 1);
	mpz_set_si(mx, 1);
	mpz_set_si(my, 0);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_import(mb, b->size, -1, 4, 0, 0, b->limb);

	while (mpz_sgn(mb) != 0) {
		mpz_set(mt, mb);
		mpz_div(mq, ma, mb);
		mpz_mod(mb, ma, mb);
		mpz_set(ma, mt);

		mpz_mul(mt, mlx, mq);
		mpz_sub(mt, mx, mt);
		mpz_set(mx, mlx);
		mpz_set(mlx, mt);
	}

	mpz_export(a->limb, &(a->size), -1, 4, 0, 0, ma);
	mpz_export(x->limb, &(x->size), -1, 4, 0, 0, mx);
	if (mpz_sgn(mx) < 0)
		x->neg = 1;

	mpz_clear(mlx);
	mpz_clear(mx);
	mpz_clear(mly);
	mpz_clear(my);
	mpz_clear(ma);
	mpz_clear(mb);
	mpz_clear(mq);
	mpz_clear(mt);
#else
	struct q_lint qt;
	struct q_lint c1;
	struct q_lint ta, tb;
	struct q_lint tmp;

	q_init(ctx, &qt, (a->alloc + b->alloc));
	q_init(ctx, &c1, (a->alloc + b->alloc));
	q_init(ctx, &ta, (a->alloc + b->alloc));
	q_init(ctx, &tb, (a->alloc + b->alloc));
	q_init(ctx, &tmp, (a->alloc + b->alloc));

	if (lx != NULL) {
		/*
		 * this branch is never taken in firmware only version
		 * because EEA always starts from default x and last_x
		 */
		q_copy(&c1, lx);
	} else {
		q_set_one(&c1);
		q_set_zero(x);
	}
	q_copy(&ta, a);
	q_copy(&tb, b);

	while (!q_is_zero(&tb)) {
		q_copy(&tmp, &tb);
		q_div(ctx, &qt, &ta, &tb);
		q_mod(ctx, &ta, &ta, &tb);
		q_copy(&tb, &ta);
		q_copy(&ta, &tmp);

		q_mul(ctx, &tmp, x, &qt);
		q_sub(&tmp, &c1, &tmp);
		q_copy(&c1, x);
		q_copy(x, &tmp);
	}

	q_copy(a, &ta);
	q_copy(x, &c1);

	q_free(ctx, &tmp);
	q_free(ctx, &tb);
	q_free(ctx, &ta);
	q_free(ctx, &c1);
	q_free(ctx, &qt);
#endif /* QLIP_USE_GMP */
#endif /* QLIP_USE_PKA_HW */

Q_EUCLID_EXIT:
	return ctx->status;
}

/*
 * q_modinv ()
 * z = a ^ (-1) mod n
 * Description: This is a generic inverse function using Extended
 * Euclid.
 */
int32_t q_modinv(struct q_lip_ctx *ctx, struct q_lint *z,
				struct q_lint *a, struct q_lint *n)
{
	int32_t status = Q_SUCCESS;

#ifdef QLIP_USE_GMP
	mpz_t ma, mn, mz;
#else
	struct q_lint x, ta;
#endif

#ifdef QLIP_USE_GMP
	mpz_init(ma);
	mpz_init(mn);
	mpz_init(mz);

	mpz_import(ma, a->size, -1, 4, 0, 0, a->limb);
	mpz_import(mn, n->size, -1, 4, 0, 0, n->limb);

	mpz_invert(mz, ma, mn);
	mpz_export(z->limb, &(z->size), -1, 4, 0, 0, mz);

	mpz_clear(ma);
	mpz_clear(mn);
	mpz_clear(mz);
#else
	status = q_init(ctx, &x, a->alloc);
	status += q_init(ctx, &ta, a->alloc);
	if (status != Q_SUCCESS)
		goto Q_MODINV_EXIT;

	q_copy(&ta, a);

	if (q_euclid(ctx, &x, NULL, &ta, n))
		goto Q_MODINV_EXIT;
	if (!q_is_one(&ta)) {
		q_abort(__FILE__, __LINE__,
			"QLIP: Modulo inverse doesn't exist for the input!");

		ctx->status = Q_ERR_NO_MODINV;
		goto Q_MODINV_EXIT;
	}

	if (x.neg)
		q_add(&x, &x, n);
	ctx->status = q_copy(z, &x);
	if (ctx->status != Q_SUCCESS)
		goto Q_MODINV_EXIT;

	q_free(ctx, &ta);
	q_free(ctx, &x);
#endif /* QLIP_USE_GMP */

Q_MODINV_EXIT:
	return ctx->status;
}

/*
 * q_modinv_sw ()
 * z = a ^ (-1) mod n
 * Description: This is a generic inverse function using Extended
 * Euclid.
 * create software-only version for 4K RSA
 */
int32_t q_modinv_sw(struct q_lip_ctx *ctx,
			   struct q_lint *z, struct q_lint *a, struct q_lint *n)
{
	int32_t status = Q_SUCCESS;

	struct q_lint x, ta;

	status = q_init(ctx, &x, a->alloc);
	status += q_init(ctx, &ta, a->alloc);
	if (status != Q_SUCCESS)
		goto Q_MODINV_SW_EXIT;

	q_copy(&ta, a);

	if (q_euclid_sw(ctx, &x, NULL, &ta, n))
		goto Q_MODINV_SW_EXIT;
	if (!q_is_one(&ta)) {
		q_abort(__FILE__, __LINE__,
			"QLIP: Modulo inverse doesn't exist for the input!");

		ctx->status = Q_ERR_NO_MODINV;
		goto Q_MODINV_SW_EXIT;
	}

	if (x.neg)
		q_add(&x, &x, n);
	ctx->status = q_copy(z, &x);
	if (ctx->status != Q_SUCCESS)
		goto Q_MODINV_SW_EXIT;

	q_free(ctx, &ta);
	q_free(ctx, &x);

Q_MODINV_SW_EXIT:
	return ctx->status;
}

/*
 * q_euclide_sw ()
 * Description: Extended Euclidian Algorithm
 * d = gcd (a, b) and d = a * x + b * y
 * assume size(b) == size(a), a > 0, b > 0
 * return value is x
 * create software-only version for 4K RSA
 */
int32_t q_euclid_sw(struct q_lip_ctx *ctx,
			   struct q_lint *x, struct q_lint *lx,
			   struct q_lint *a, struct q_lint *b)
{
	int32_t status = Q_SUCCESS;
	struct q_lint qt;
	struct q_lint c1;
	struct q_lint ta, tb;
	struct q_lint tmp;

	status = q_init(ctx, &qt, (a->alloc + b->alloc));
	status += q_init(ctx, &c1, (a->alloc + b->alloc));
	status += q_init(ctx, &ta, (a->alloc + b->alloc));
	status += q_init(ctx, &tb, (a->alloc + b->alloc));
	status += q_init(ctx, &tmp, (a->alloc + b->alloc));
	if (status != Q_SUCCESS)
		goto Q_EUCLID_SW_EXIT;

	if (lx != NULL) {
		/*
		 * this branch is never taken in firmware only version
		 * because EEA always starts from default x and last_x
		 */
		q_copy(&c1, lx);
	} else {
		q_set_one(&c1);
		q_set_zero(x);
	}
	q_copy(&ta, a);
	q_copy(&tb, b);

	while (!q_is_zero(&tb)) {
		q_copy(&tmp, &tb);
		q_div(ctx, &qt, &ta, &tb);
		q_mod_sw(ctx, &ta, &ta, &tb);
		q_copy(&tb, &ta);
		q_copy(&ta, &tmp);

		q_mul_sw(ctx, &tmp, x, &qt);
		q_sub(&tmp, &c1, &tmp);
		q_copy(&c1, x);
		q_copy(x, &tmp);
	}

	q_copy(a, &ta);
	q_copy(x, &c1);

	q_free(ctx, &tmp);
	q_free(ctx, &tb);
	q_free(ctx, &ta);
	q_free(ctx, &c1);
	q_free(ctx, &qt);

Q_EUCLID_SW_EXIT:
	return ctx->status;
}
