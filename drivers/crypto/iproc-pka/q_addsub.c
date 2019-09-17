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

/*
 * q_asm_addc ()
 * Description: inline primitive functions
 */
inline uint32_t q_asm_addc(uint32_t al, uint32_t bl, uint32_t *carry)
{
#ifdef TARGET_ARM1136
	uint32_t ret;
	__asm {
		MOV R1,
#0;
		ADDS R0, al, bl;
		ORRCS R1, R1,
#1;
		ADDS ret, R0, (*carry);
		ORRCS R1, R1,
#1;
		MOV(*carry), R1;
	}
	return ret;
#else
	uint32_t ret;
	ret = al + bl + *carry;
	*carry = (al > (UINT_MASK - bl)) ||
			((al == (UINT_MASK - bl)) && *carry);
	return ret;
#endif
}

/*
 * q_uadd ()
 * Description: unsigned z = a + b
 * Notes: without dynamic allocation, the sum z must always be
 * allocated to one limb larger than the max_size(a, b)
 */

int32_t q_uadd(struct q_lint *z,/* q_lint pointer to result z */
			  struct q_lint *a,	/* q_lint pointer to source a */
			  struct q_lint *b) /* q_lint pointer to source b */
{
	int32_t status = Q_SUCCESS;
	int i;
	uint32_t carry;
	uint32_t *zp;
	uint32_t *ap;
	uint32_t *bp;
	uint32_t size_s, size_l;

	/* need normalization here ? */
	size_s = (a->size > b->size) ? b->size : a->size;
	size_l = (a->size > b->size) ? a->size : b->size;

	if (z->alloc < size_l + 1) {
		q_abort(__FILE__, __LINE__,
				"QLIP: q_uadd insufficient memory for the result!");

		status = Q_ERR_DST_OVERFLOW;
		goto Q_UADD_EXIT;
	}

	ap = (a->size > b->size) ? a->limb : b->limb;
	bp = (a->size > b->size) ? b->limb : a->limb;
	zp = z->limb;
	carry = 0;
	for (i = 0; i < size_s; i++) {
		*zp = q_asm_addc(*ap, *bp, &carry);
		zp++;
		ap++;
		bp++;
	}

	/* skip optimization of direct copy when carry = 0 */
	for (i = size_s; i < size_l; i++) {
		*zp = q_asm_addc(*ap, 0, &carry);
		zp++;
		ap++;
		bp++;
	}

	*zp = carry;
	z->size = carry ? size_l + 1 : size_l;
	z->neg = a->neg;

Q_UADD_EXIT:
	return status;
}

/*
 * q_add ()
 * Description: signed z = a+b
 */
int32_t q_add(struct q_lint *z,	/* q_lint pointer to result z */
			 struct q_lint *a, /* q_lint pointer to source a */
			 struct q_lint *b) /* q_lint pointer to source b */
{
	int32_t status = Q_SUCCESS;

	if (a->neg ^ b->neg) {
		if (q_ucmp(a, b) > 0) {
			status = q_usub(z, a, b);
			z->neg = a->neg;
		} else {
			status = q_usub(z, b, a);
			z->neg = b->neg;
		}
		goto Q_ADD_EXIT;
	}

	status = q_uadd(z, a, b);
	z->neg = a->neg;

Q_ADD_EXIT:
	return status;
}

/*
 * q_usub ()
 * Description: unsigned z = a - b
 * Note: without dynamic allocation, the sum z must always be
 * allocated to the max_size(a, b)
 */

int32_t q_usub(struct q_lint *z, /* q_lint pointer to result z */
		  struct q_lint *a,	/* q_lint pointer to source a */
		  struct q_lint *b) /* q_lint pointer to source b */
{
	int32_t status = Q_SUCCESS;
	int i;
	uint32_t carry;
	uint32_t *zp;
	uint32_t *ap;
	uint32_t *bp;
	uint32_t tmp;
	uint32_t size_a, size_b;

	size_a = a->size;
	size_b = b->size;

	if (z->alloc < size_a || size_a < size_b) {
		q_abort(__FILE__, __LINE__,
				"QLIP: q_usub result incorrect size!");

		status = Q_ERR_DST_OVERFLOW;
		goto Q_USUB_EXIT;
	}

	ap = a->limb;
	bp = b->limb;
	zp = z->limb;

	carry = 1;

	for (i = 0; i < size_b; i++) {
		tmp = *bp;
		tmp = ~(tmp);
		*zp = q_asm_addc(*ap, tmp, &carry);
		zp++;
		ap++;
		bp++;
	}

	for (i = size_b; i < size_a; i++) {
		*zp = q_asm_addc(*ap, UINT_MASK, &carry);
		zp++;
		ap++;
	}

	z->size = size_a;
	z->neg = a->neg;
	Q_NORMALIZE(z->limb, z->size);

Q_USUB_EXIT:
	return status;
}

/*
 * q_sub ()
 * Description: signed z = a + b
 */
int32_t q_sub(struct q_lint *z,	/* q_lint pointer to result z */
			 struct q_lint *a,	/* q_lint pointer to source a */
			 struct q_lint *b)
{
	/* q_lint pointer to source b */
	int32_t status = Q_SUCCESS;

	if (a->neg ^ b->neg) {
		status = q_uadd(z, a, b);
		z->neg = a->neg;
		goto Q_SUB_EXIT;
	}

	if (q_ucmp(a, b) > 0) {
		status = q_usub(z, a, b);
		z->neg = a->neg;
	} else if (q_ucmp(a, b) < 0) {
		status = q_usub(z, b, a);
		z->neg = !a->neg;
	} else
		q_set_zero(z);

Q_SUB_EXIT:
	return status;
}

/*
 * q_ucmp ()
 * Description: unsigned comparison
 */
int q_ucmp(struct q_lint *a,	/* q_lint pointer to source a */
		   struct q_lint *b)
{
	/* q_lint pointer to source b */
	int result = 0;
	int i;
	uint32_t *ap;
	uint32_t *bp;

	result = a->size - b->size;
	if (result != 0)
		goto Q_UCMP_EXIT;

	ap = a->limb;
	bp = b->limb;

	for (i = a->size - 1; i >= 0; i--) {
		if (ap[i] != bp[i]) {
			result = (ap[i] > bp[i]) ? 1 : -1;
			goto Q_UCMP_EXIT;
		}
	}

Q_UCMP_EXIT:
	return result;
}

/*
 * q_cmp ()
 * Description: signed comparison
 */
int q_cmp(struct q_lint *a,	/* q_lint pointer to source a */
		  struct q_lint *b)
{
	/* q_lint pointer to source b */
	int result = 0;
	int i;
	uint32_t *ap;
	uint32_t *bp;
	int res;

	if (a->neg ^ b->neg) {
		if (a->neg) {
			result = -1;
			goto Q_CMP_EXIT;
		} else {
			result = 1;
			goto Q_CMP_EXIT;
		}
	}

	res = a->size - b->size;
	if (res != 0) {
		result = res ^ a->neg;
		goto Q_CMP_EXIT;
	}

	ap = a->limb;
	bp = b->limb;
	for (i = a->size - 1; i >= 0; i--) {
		if (ap[i] != bp[i]) {
			if (a->neg) {
				result = (ap[i] > bp[i]) ? -1 : 1;
				goto Q_CMP_EXIT;
			} else {
				result = (ap[i] > bp[i]) ? 1 : -1;
				goto Q_CMP_EXIT;
			}
		}
	}

Q_CMP_EXIT:
	return result;
}

/*
 * q_modadd ()
 * Description: Modolo addition z = (a + b) % n
 * Assumptions:
 * (1) 0 <= a < n, 0 <= b < n
 * (2) a, b < 2 ^ 4096
 * @param ctx		QLIP context pointer
 * @param z		q_lint pointer to result z
 * @param a		q_lint pointer to source a
 * @param b		q_lint pointer to source b
 * @param n		q_lint pointer to modulus n
 */
int32_t q_modadd(struct q_lip_ctx *ctx,
					struct q_lint *z,
					struct q_lint *a,
					struct q_lint *b,
					struct q_lint *n)
{

#ifdef QLIP_USE_PKE_HW
	struct q_lint ta, tb, tn;
#endif

#ifdef QLIP_USE_PKA_HW
	uint32_t pka_status;
	uint32_t lir_n;
	struct opcode sequence[5];
#endif

#ifdef QLIP_USE_PKE_HW
	q_init(ctx, &ta, n->alloc);
	q_init(ctx, &tb, n->alloc);
	q_init(ctx, &tn, n->alloc);
	q_copy(&ta, a);
	q_copy(&tb, b);
	q_copy(&tn, n);
	q_modadd_hw(z->limb, n->size * MACHINE_WD, tn.limb, ta.limb, tb.limb);
	z->size = n->size;
	Q_NORMALIZE(z->limb, z->size);
	q_free(ctx, &tn);
	q_free(ctx, &tb);
	q_free(ctx, &ta);
#else /* QLIP_USE_PKE_HW */
#ifdef QLIP_USE_PKA_HW
	if (z->alloc < n->size) {
		ctx->status = Q_ERR_DST_OVERFLOW;
		goto Q_MODADD_EXIT;
	}

	z->size = n->size;
	lir_n = PKA_LIR_J;

	/* load modulus n to x[0] */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 0), n->size);
	sequence[0].ptr = n->limb;

	/* load a to x[1] */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 1), a->size);
	sequence[1].ptr = a->limb;

	/* load b to x[2] */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 2), b->size);
	sequence[2].ptr = b->limb;

	/* x[1] = x[1] + x[2] mod x[0] */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_n, 1),
			PKA_LIR(lir_n, 1));
	sequence[3].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));
	sequence[3].ptr = NULL;

	/* read back result x[1] */
	sequence[4].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 1), n->size);
	sequence[4].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_MODADD_EXIT;
	}
	q_pka_hw_write_sequence(5, sequence);

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
			goto Q_MODADD_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_MODADD_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(n->size, z->limb);

#else /* QLIP_USE_PKA_HW */

	ctx->status = q_add(z, a, b);
	if (ctx->status)
		goto Q_MODADD_EXIT;
	q_mod(ctx, z, z, n);

#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_PKE_HW */

Q_MODADD_EXIT:
	return ctx->status;
}

/*
 * q_modsub ()
 * Description: Modulo subtraction z = (a - b) % n
 * Assumptions:
 * (1) 0 <= a < n, 0 <= b < n
 * (2) a, b < 2 ^ 4096
 * @param ctx		QLIP context pointer
 * @param z		q_lint pointer to result z
 * @param a		q_lint pointer to source a
 * @param b		q_lint pointer to source b
 * @param n		q_lint pointer to modulus n
 */
int32_t q_modsub(struct q_lip_ctx *ctx,
					struct q_lint *z,
					struct q_lint *a,
					struct q_lint *b,
					struct q_lint *n)
{
#ifdef QLIP_USE_PKE_HW
	struct q_lint ta, tb, tn;
#endif

#ifdef QLIP_USE_PKA_HW
	uint32_t pka_status;
	uint32_t lir_n;
	struct opcode sequence[5];
#endif

#ifdef QLIP_USE_PKE_HW
	q_init(ctx, &ta, n->alloc);
	q_init(ctx, &tb, n->alloc);
	q_init(ctx, &tn, n->alloc);
	q_copy(&ta, a);
	q_copy(&tb, b);
	q_copy(&tn, n);
	q_modsub_hw(z->limb, n->size * MACHINE_WD, tn.limb, ta.limb, tb.limb);
	z->size = n->size;
	Q_NORMALIZE(z->limb, z->size);
	q_free(ctx, &tn);
	q_free(ctx, &tb);
	q_free(ctx, &ta);
#else /* QLIP_USE_PKE_HW */
#ifdef QLIP_USE_PKA_HW
	if (z->alloc < n->size) {
		ctx->status = Q_ERR_DST_OVERFLOW;
		goto Q_MODSUB_EXIT;
	}

	z->size = n->size;
	lir_n = PKA_LIR_J;

	/* load modulus n to x[0] */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 0), n->size);
	sequence[0].ptr = n->limb;

	/* load a to x[1] */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 1), a->size);
	sequence[1].ptr = a->limb;

	/* load b to x[2] */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_n, 2), b->size);
	sequence[2].ptr = b->limb;

	/* x[1] = x[1] - x[2] mod x[0] */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_n, 1),
			PKA_LIR(lir_n, 1));
	sequence[3].op2 = PACK_OP2(PKA_LIR(lir_n, 2), PKA_LIR(lir_n, 0));
	sequence[3].ptr = NULL;

	/* read back result x[1] */
	sequence[4].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_n, 1), n->size);
	sequence[4].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY) {
		ctx->status = ctx->q_yield();
		if (ctx->status != Q_SUCCESS)
			goto Q_MODSUB_EXIT;
	}
	q_pka_hw_write_sequence(5, sequence);

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
			goto Q_MODSUB_EXIT;
		} else {
			ctx->status = ctx->q_yield();
			if (ctx->status != Q_SUCCESS)
				goto Q_MODSUB_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(n->size, z->limb);

#else /* QLIP_USE_PKA_HW */
	ctx->status = q_sub(z, a, b);
	if (ctx->status)
		goto Q_MODSUB_EXIT;
	if (q_mod(ctx, z, z, n))
		goto Q_MODSUB_EXIT;
	if (z->neg)
		ctx->status = q_add(z, z, n);
#endif /* QLIP_USE_PKA_HW */
#endif /* QLIP_USE_PKE_HW */

Q_MODSUB_EXIT:
	return ctx->status;
}
