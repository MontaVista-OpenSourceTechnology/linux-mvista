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
 * q_pt_copy ()
 * Description: EC point copy function.
 */
int32_t q_pt_copy(struct q_point *r, /* destination q_point pointer */
					 struct q_point *p)
{
	/* source q_point pointer */
	int32_t status = Q_SUCCESS;

	status = q_copy(&r->X, &p->X);
	status += q_copy(&r->Y, &p->Y);
	status += q_copy(&r->Z, &p->Z);
	if (status != Q_SUCCESS)
		status = Q_ERR_DST_OVERFLOW;

	return status;
}

/*
 * q_pt_is_affine ()
 * Description: This function tests the representation fo the EC point.
 */

int q_pt_is_affine(struct q_point *p)
{
	uint32_t size;
	uint32_t *zp;
	int result = 1;
	int i = 0;

	size = p->Z.size;
	zp = p->Z.limb;

	/* assume BIGNUM */
	if (zp[i] != 1) {
		result = 0;
		goto Q_PT_IS_AFFINE_EXIT;
	}
	for (i = 1; i < size; i++) {
		if (zp[i] != 0) {
			result = 0;
			goto Q_PT_IS_AFFINE_EXIT;
		}
	}

Q_PT_IS_AFFINE_EXIT:
	return result;
}

/*
 * q_ecp_pt_mul ()
 * Description: This function performs EC point multiplication in
 * affine coordinate.
 * @param ctx		QLIP context pointer
 * @param r		Result EC point pointer
 * @param p		Input EC point pointer
 * @param k		Multiplication factor pointer
 * @param curve	EC curve parameter pointer
 */
int32_t q_ecp_pt_mul(struct q_lip_ctx *ctx,
					struct q_point *r,
					struct q_point *p,
					struct q_lint *k,
					struct q_curve *curve)
{
	int32_t status = Q_SUCCESS;
	uint32_t size;
	struct q_point cp;
	struct q_point p_minus;
	struct q_lint ca;
	int i;

	/* move the memory allocation to top level */
	struct q_lint tmp[6];

#ifdef QLIP_USE_PKA_HW
	struct q_mont mp;
	uint32_t pka_status;
	uint32_t lir_p;
	struct opcode sequence[17];
#endif

#ifdef QLIP_MOD_USE_MONT
	/* if we use Montgomery, as in the case of optimized software */
	struct q_mont mp;
#endif

	if (!q_pt_is_affine(p)) {
		status = Q_ERR_EC_PT_NOT_AFFINE;
		goto Q_ECP_PT_MUL_EXIT;
	}

	/* take care of the k=1 case */
	if (q_is_one(k)) {
		status = q_pt_copy(r, p);
		goto Q_ECP_PT_MUL_EXIT;
	}
	status = q_init(ctx, &cp.X, p->X.alloc);
	status += q_init(ctx, &cp.Y, p->Y.alloc);
	status += q_init(ctx, &cp.Z, p->Z.alloc);
	status += q_init(ctx, &ca, curve->a.alloc);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_EXIT;

	status = q_pt_copy(&cp, p);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_EXIT;
	status = q_copy(&ca, &curve->a);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_EXIT;

	size = curve->p.alloc + 1;
	for (i = 0; i < 6; i++)
		status += q_init(ctx, &tmp[i], size);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_EXIT;

	/* get -p for NAF */
	status = q_init(ctx, &p_minus.X, size);
	status += q_init(ctx, &p_minus.Y, size);
	status += q_init(ctx, &p_minus.Z, size);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_EXIT;

#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 17; i++)
		sequence[i].ptr = NULL;

	/* initialize hardware first */
	q_pka_hw_rst();

	/*
	 * if we use Montgomery, as in the case of PKA
	 * hardware or optimized software
	 */
	status = q_init(ctx, &mp.n, curve->p.alloc);
	status += q_init(ctx, &mp.np, curve->p.alloc + 1);
	status += q_init(ctx, &mp.rr, curve->p.alloc + 1);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_EXIT;

	status = q_mont_init(ctx, &mp, &curve->p);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_EXIT;
	curve->mp = &mp;

	/* create command sequence */
	lir_p = q_pka_sel_lir(curve->p.size);

	/* Load curve prime number Montgomery context n, np and rr */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0),
			curve->mp->n.size);
	sequence[0].ptr = curve->mp->n.limb;

	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1),
			curve->mp->np.size);
	sequence[1].ptr = curve->mp->np.limb;

	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2),
			curve->mp->rr.size);
	sequence[2].ptr = curve->mp->rr.limb;

	/* Load curve parameter a */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), curve->a.size);
	sequence[3].ptr = curve->a.limb;

	/* Load base point coordinate X */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), cp.X.size);
	sequence[4].ptr = cp.X.limb;

	/* Load base point coordinate Y */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), cp.Y.size);
	sequence[5].ptr = cp.Y.limb;

	/* Load base point coordinate Z */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 6), cp.Z.size);
	sequence[6].ptr = cp.Z.limb;

	/* Convert a to n-residue domain */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 3));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	/* Convert X to n-residue domain */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 4));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	/* Convert Y to n-residue domain */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
			PKA_LIR(lir_p, 5));
	sequence[9].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	/* Convert Z to n-residue domain */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 6),
			PKA_LIR(lir_p, 6));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	/* Convert -Y to n-residue domain */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_LSUB, PKA_LIR(lir_p, 7), PKA_LIR(lir_p, 0));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_p, 5), PKA_NULL);

	/* read back a */
	sequence[12].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 3),
			curve->p.size);

	/* read back X */
	sequence[13].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 4),
			curve->p.size);

	/* read back Y */
	sequence[14].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 5),
			curve->p.size);

	/* read back Z */
	sequence[15].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 6),
			curve->p.size);

	/* read back -Y */
	sequence[16].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 7),
			curve->p.size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(17, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {

		/*
		 * While we wait for the the first unload opcode,
		 * we want to monitor the CMD_ERR bit in the status
		 * register, as the math opcodes before the first
		 * unload opcode may trigger PKA HW related error.
		 * We do not need to monitor the CMD_ERR
		 * for the subsequent unload opcode.
		 */
		if (pka_status & PKA_STAT_ERROR) {
			status = Q_ERR_PKA_HW_ERR;
			goto Q_ECP_PT_MUL_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(curve->p.size, curve->a.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(curve->p.size, cp.X.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(curve->p.size, cp.Y.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(curve->p.size, cp.Z.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(curve->p.size, p_minus.Y.limb);

	cp.X.size = curve->p.size;
	cp.Y.size = curve->p.size;
	cp.Z.size = curve->p.size;
	p_minus.Y.size = curve->p.size;
	curve->a.size = curve->p.size;

#else
#ifdef QLIP_MOD_USE_MONT
	/* if we use Montgomery, as in the case of optimized software */
	status = q_init(ctx, &mp.n, curve->p.alloc);
	status += q_init(ctx, &mp.np, curve->p.alloc + 1);
	status += q_init(ctx, &mp.rr, curve->p.alloc + 1);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_EXIT;

	status = q_mont_init(ctx, &mp, &curve->p);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_EXIT;
	curve->mp = &mp;

	/* now convert all point coordinates to residue format */
	status = q_mont_mul(ctx, &curve->a, &curve->a, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp.X, &cp.X, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp.Y, &cp.Y, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp.Z, &cp.Z, &mp.rr, &mp);
	status += q_sub(&p_minus.Y, &mp.n, &cp.Y);
	if (status != Q_SUCCESS) {
		status = Q_ERR_CTX_MEM_SIZE_LOW;
		goto Q_ECP_PT_MUL_EXIT;
	}
#else
	status += q_sub(&p_minus.Y, &curve->p, &cp.Y);
	if (status != Q_SUCCESS) {
		status = Q_ERR_CTX_MEM_SIZE_LOW;
		goto Q_ECP_PT_MUL_EXIT;
	}
#endif /* QLIP_MOD_USE_MONT */
#endif /* QLIP_USE_PKA_HW */

	q_copy(&p_minus.X, &cp.X);
	q_copy(&p_minus.Z, &cp.Z);
	if (q_ecp_pt_mul_prj(ctx, r, &cp, &p_minus, k, curve, tmp))
		goto Q_ECP_PT_MUL_EXIT;
	if (q_ecp_prj_2_affine(ctx, r, r, curve))
		goto Q_ECP_PT_MUL_EXIT;

#ifdef QLIP_USE_PKA_HW
	if (q_copy(&curve->a, &ca))
		goto Q_ECP_PT_MUL_EXIT;
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#else
#ifdef QLIP_MOD_USE_MONT
	q_copy(&curve->a, &ca);
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#endif /* QLIP_MOD_USE_MONT */
#endif /* QLIP_USE_PKA_HW */

	/* free up allocated memory */
	q_free(ctx, &p_minus.Z);
	q_free(ctx, &p_minus.Y);
	q_free(ctx, &p_minus.X);

	for (i = 5; i >= 0; i--)
		q_free(ctx, &tmp[i]);

	q_free(ctx, &ca);
	q_free(ctx, &cp.Z);
	q_free(ctx, &cp.Y);
	q_free(ctx, &cp.X);

Q_ECP_PT_MUL_EXIT:
	ctx->status = status;
	return ctx->status;
}

/*
 * q_ecp_pt_mul_prj ()
 * Description: This function performs EC point multiplication in the
 * Jocobian projective coordinate.
 * @param ctx		QLIP context pointer
 * @param r		Result EC point pointer
 * @param p		Input EC point pointer
 * @param p_minus	Negated input EC point pointer
 * @param k		Multiplication factor
 * @param curve	EC curve parameter pointer
 * @param tmp		Temporary q_lint pointer
 */
int32_t q_ecp_pt_mul_prj(struct q_lip_ctx *ctx,
							struct q_point *r,
							struct q_point *p,
							struct q_point *p_minus,
							struct q_lint *k,
							struct q_curve *curve,
							struct q_lint *tmp)
{
	int32_t status = Q_SUCCESS;
	uint32_t size;
	struct q_lint h;
	int i, j;
	long long int carry = 0;
	int msb = 0;
	int lsb = 0;
	uint32_t k_word, h_word;

#ifdef QLIP_USE_PKA_HW
	struct opcode sequence[QLIP_PKA_ECP_DBL_SEQ_LEN +
					QLIP_PKA_ECP_SUB_SEQ_LEN];
	struct opcode *dbl_sequence, *add_sequence, *sub_sequence;
	uint32_t lir_p;
#endif

	status = q_pt_copy(r, p);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_MUL_PRJ_EXIT;

	/* take care of the k=1 case */
	if (q_is_one(k))
		goto Q_ECP_PT_MUL_PRJ_EXIT;

	/* get NAF representation of k and store it in k and h */
	size = k->size;
	if (q_init(ctx, &h, size + 1))
		goto Q_ECP_PT_MUL_PRJ_EXIT;
	h.size = size + 1;

	for (i = 0; i < size; i++) {
		carry = carry + k->limb[i] + k->limb[i] + k->limb[i];
		h.limb[i] = (uint32_t) carry;
		carry >>= 32;
	}
	h.limb[size] = (uint32_t) carry;

	if (h.limb[size] == 0)
		size--;
	msb = q_leading_one(h.limb[size]);

#ifdef QLIP_USE_PKA_HW
	dbl_sequence = sequence;
	sub_sequence = sequence + QLIP_PKA_ECP_DBL_SEQ_LEN;
	add_sequence = sequence + QLIP_PKA_ECP_DBL_SEQ_LEN + 1;

	lir_p = q_pka_sel_lir(curve->p.size);

	/* Construct point addition and doubling opcode sequences */
	q_ecp_pt_dbl_prj_pka(dbl_sequence, lir_p);
	q_ecp_pt_add_prj_pka(add_sequence, lir_p);

	sub_sequence[0].op1 =
		PACK_OP1(0, PKA_OP_LSUB, PKA_LIR(lir_p, 7), PKA_LIR(lir_p, 0));
	sub_sequence[0].op2 = PACK_OP2(PKA_LIR(lir_p, 7), PKA_NULL);
	sub_sequence[0].ptr = NULL;

	sub_sequence[QLIP_PKA_ECP_SUB_SEQ_LEN - 1].op1 =
		PACK_OP1(0, PKA_OP_LSUB, PKA_LIR(lir_p, 7), PKA_LIR(lir_p, 0));
	sub_sequence[QLIP_PKA_ECP_SUB_SEQ_LEN - 1].op2 =
		PACK_OP2(PKA_LIR(lir_p, 7), PKA_NULL);
	sub_sequence[QLIP_PKA_ECP_SUB_SEQ_LEN - 1].ptr = NULL;

	/* Reset PKA HW */
	q_pka_hw_rst();

	/* Load data into PKA HW */
	if (q_ecp_prj_pka_init(ctx, curve, r, r))
		goto Q_ECP_PT_MUL_PRJ_EXIT;

	/* Send in point addition and doubling sequences */
	for (i = size; i >= 0; i--) {
		k_word = (i == k->size) ? 0 : k->limb[i];
		h_word = h.limb[i];
		if (i == 0)
			lsb = 1;

		for (j = msb; j > lsb; j--) {

			/* send point doubling sequnence */
			while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
				;
			q_pka_hw_write_sequence(QLIP_PKA_ECP_DBL_SEQ_LEN,
								dbl_sequence);

			if ((~k_word) & (h_word) & (1 << (j - 1))) {
				/* send point addition sequnence */
				while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
					;
				q_pka_hw_write_sequence
				(QLIP_PKA_ECP_ADD_SEQ_LEN, add_sequence);
			}

			if ((k_word) & (~h_word) & (1 << (j - 1))) {
				/* send point subtraction sequnence */
				while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
					;
				q_pka_hw_write_sequence
				(QLIP_PKA_ECP_SUB_SEQ_LEN, sub_sequence);
			}
		}
		msb = MACHINE_WD;
	}

	/* Read back the result */
	if (q_ecp_prj_pka_get_result(ctx, r))
		goto Q_ECP_PT_MUL_PRJ_EXIT;
#else
	/* bit scan the multiplicant using NAF with LR method */
	for (i = size; i >= 0; i--) {
		k_word = (i == k->size) ? 0 : k->limb[i];
		h_word = h.limb[i];
		if (i == 0)
			lsb = 1;

		for (j = msb; j > lsb; j--) {
			if (q_ecp_pt_dbl_prj(ctx, r, r, curve, tmp))
				goto Q_ECP_PT_MUL_PRJ_EXIT;

			if ((~k_word) & (h_word) & (1 << (j - 1))) {
				if (q_ecp_pt_add_prj(ctx, r, r, p, curve, tmp))
					goto Q_ECP_PT_MUL_PRJ_EXIT;
			}

			if ((k_word) & (~h_word) & (1 << (j - 1))) {
				if (q_ecp_pt_add_prj
					(ctx, r, r, p_minus, curve, tmp))
					goto Q_ECP_PT_MUL_PRJ_EXIT;
			}
		}

		msb = MACHINE_WD;
	}

#endif

	/* free up allocated memory */
	q_free(ctx, &h);

Q_ECP_PT_MUL_PRJ_EXIT:
	ctx->status = status;
	return ctx->status;
}

/*
 * q_ecp_pt_dbl_prj ()
 * Description: This function performs EC point doubling in the
 * Jocobian projective coordinate.
 * @param ctx		QLIP context pointer
 * @param r		Result EC point pointer
 * @param p		Input EC point pointer
 * @param curve	EC curve parameter pointer
 * @param tmp		Temporary q_lint pointer
 */
int32_t q_ecp_pt_dbl_prj(struct q_lip_ctx *ctx,
							struct q_point *r,
							struct q_point *p,
							struct q_curve *curve,
							struct q_lint *tmp)
{
	int32_t status = Q_SUCCESS;
	int z_is_one;

#ifdef QLIP_USE_PKA_HW
	struct opcode sequence[QLIP_PKA_ECP_DBL_SEQ_LEN];
	uint32_t lir_p;
#endif

	z_is_one = q_pt_is_affine(p);

	/*
	 * the hardware mapping based on QLIP_USE_PKE_HW
	 * is only applicable to OLD PKE mapping method is
	 * different with the new PKA hardware
	 */
#ifdef QLIP_USE_PKE_HW
	uint32_t blen_n;
	int i;
	blen_n = curve->p.size * MACHINE_WD;

	for (i = 0; i < 3; i++) {
		tmp[i].size = curve->p.size;
		q_set_zero(&tmp[i]);
		tmp[i].size = curve->p.size;
	}

	if (z_is_one)
		q_copy(&tmp[0], &curve->a);
	else {
		q_modmul_hw(tmp[0].limb, blen_n, curve->p.limb,
			p->Z.limb, p->Z.limb);	/* Z^2 mod p */
		q_modmul_hw(tmp[0].limb, blen_n, curve->p.limb,
			tmp[0].limb, tmp[0].limb);	/* Z^4 mod p */
		q_modmul_hw(tmp[0].limb, blen_n, curve->p.limb,
			tmp[0].limb, curve->a.limb);	/* aZ^4 mod p */
	}

	q_modmul_hw(tmp[1].limb, blen_n, curve->p.limb,
		p->X.limb, p->X.limb);	/* X^2 mod p */
	q_modadd_hw(tmp[0].limb, blen_n, curve->p.limb,
		tmp[0].limb, tmp[1].limb);	/* M = (X^2+aZ^4) mod p */
	q_modadd_hw(tmp[0].limb, blen_n, curve->p.limb,
		tmp[0].limb, tmp[1].limb);	/* M = (2X^2+aZ^4) mod p */
	q_modadd_hw(tmp[1].limb, blen_n, curve->p.limb,
		tmp[0].limb, tmp[1].limb);	/* M = (3X^2+aZ^4) mod p */

	q_modadd_hw(tmp[0].limb, blen_n, curve->p.limb,
		p->Y.limb, p->Y.limb);	/* 2Y mod p */
	if (z_is_one) {
		q_copy(&r->Z, &tmp[0]);	/* Z' = 2Y mod p */
	} else {
		q_modmul_hw(r->Z.limb, blen_n, curve->p.limb,
			tmp[0].limb, r->Z.limb);	/* Z' = 2YZ mod p */
	}
	r->Z.size = curve->p.size;
	Q_NORMALIZE(r->Z.limb, r->Z.size);

	q_modmul_hw(tmp[0].limb, blen_n, curve->p.limb,
		tmp[0].limb, tmp[0].limb);	/* 4Y^2 mod p */
	q_modmul_hw(tmp[2].limb, blen_n, curve->p.limb,
		tmp[0].limb, p->X.limb);	/* S = 4XY^2 mod p */

	q_modmul_hw(r->X.limb, blen_n, curve->p.limb,
		tmp[1].limb, tmp[1].limb);	/* M^2 mod p */
	q_modsub_hw(r->X.limb, blen_n, curve->p.limb,
		r->X.limb, tmp[2].limb);	/* X' = (M^2-S) mod p */
	q_modsub_hw(r->X.limb, blen_n, curve->p.limb,
		r->X.limb, tmp[2].limb);	/* X' = (M^2-2S) mod p */
	r->X.size = curve->p.size;
	Q_NORMALIZE(r->X.limb, r->X.size);

	q_modsub_hw(tmp[2].limb, blen_n, curve->p.limb,
		tmp[2].limb, r->X.limb);	/* (S-X') mod p */
	q_modmul_hw(tmp[2].limb, blen_n, curve->p.limb,
		tmp[1].limb, tmp[2].limb);	/* M (S-X') mod p */

	/* replace 8Y^4 = 8Y^2 * Y^2 with 8Y^4 = 16Y^4/2 */
	q_modmul_hw(tmp[1].limb, blen_n, curve->p.limb,
		tmp[0].limb, tmp[0].limb);	/* 16Y^4 mod p */
	q_mod_div2(ctx, &tmp[0], &tmp[1], &curve->p);	/* T = 8Y^4 */

	q_modsub_hw(r->Y.limb, blen_n, curve->p.limb,
		tmp[2].limb, tmp[0].limb);	/* Y' = (M (S-X') - T) mod p */
	r->Y.size = curve->p.size;
	Q_NORMALIZE(r->Y.limb, r->Y.size);
#else
#ifdef QLIP_USE_PKA_HW
	r->X.size = curve->mp->n.size;
	r->Y.size = curve->mp->n.size;
	r->Z.size = curve->mp->n.size;

	lir_p = q_pka_sel_lir(curve->p.size);

	if (q_ecp_prj_pka_init(ctx, curve, p, p))
		goto Q_ECP_PT_DBL_PRJ_EXIT;

	q_ecp_pt_dbl_prj_pka(sequence, lir_p);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(QLIP_PKA_ECP_DBL_SEQ_LEN, sequence);

	q_ecp_prj_pka_get_result(ctx, r);

#else
#ifdef QLIP_USE_GMP
	mpz_t mx, my, mz, ma, mp, mt0, mt1, mt2;
	mpz_init(mx);
	mpz_init(my);
	mpz_init(mz);
	mpz_init(ma);
	mpz_init(mp);
	mpz_init(mt0);
	mpz_init(mt1);
	mpz_init(mt2);

	mpz_import(mx, p->X.size, -1, 4, 0, 0, p->X.limb);
	mpz_import(my, p->Y.size, -1, 4, 0, 0, p->Y.limb);
	mpz_import(mz, p->Z.size, -1, 4, 0, 0, p->Z.limb);
	mpz_import(ma, curve->a.size, -1, 4, 0, 0, curve->a.limb);
	mpz_import(mp, curve->p.size, -1, 4, 0, 0, curve->p.limb);

	if (z_is_one)
		mpz_set(mt0, ma);
	else {
		mpz_mul(mt0, mz, mz);	/* Z^2 mod p */
		mpz_mod(mt0, mt0, mp);
		mpz_mul(mt0, mt0, mt0);	/* Z^4 mod p */
		mpz_mod(mt0, mt0, mp);
		mpz_mul(mt0, mt0, ma);
		mpz_mod(mt0, mt0, mp);	/* aZ^4 mod p */
	}

	mpz_mul(mt1, mx, mx);	/* X^2 mod p */
	mpz_mod(mt1, mt1, mp);
	mpz_set_si(mt2, 3);
	mpz_mul(mt1, mt1, mt2);	/* M = (3X^2+aZ^4) mod p */
	mpz_mod(mt1, mt1, mp);
	mpz_add(mt1, mt1, mt0);
	mpz_mod(mt1, mt1, mp);

	mpz_set_si(mt2, 2);	/* 2Y mod p */
	mpz_mul(mt0, mt2, my);
	mpz_mod(mt0, mt0, mp);

	if (z_is_one) {
		/* Z' = 2Y mod p */
		mpz_export(r->Z.limb, &(r->Z.size), -1, 4, 0, 0, mt0);
	} else {
		mpz_mul(mz, mt0, mz);
		mpz_mod(mz, mz, mp);
		/* Z' = 2YZ mod p */
		mpz_export(r->Z.limb, &(r->Z.size), -1, 4, 0, 0, mz);
	}

	mpz_mul(mt0, mt0, mt0);	/* 4Y^2 mod p */
	mpz_mod(mt0, mt0, mp);

	mpz_mul(mt2, mt0, mx);	/* S = 4XY^2 mod p */
	mpz_mod(mt2, mt2, mp);

	mpz_mul(mx, mt1, mt1);	/* M^2 mod p */
	mpz_mod(mx, mx, mp);

	mpz_sub(mx, mx, mt2);	/* X' = (M^2-2S) mod p */
	mpz_sub(mx, mx, mt2);
	mpz_mod(mx, mx, mp);
	mpz_export(r->X.limb, &(r->X.size), -1, 4, 0, 0, mx);

	mpz_sub(mt2, mt2, mx);	/* (S-X') mod p */
	mpz_mod(mt2, mt2, mp);

	mpz_mul(mt2, mt2, mt1);	/* M (S-X') mod p */
	mpz_mod(mt2, mt2, mp);

	mpz_mul(mt1, mt0, mt0);	/* 16Y^4 mod p */
	mpz_mod(mt1, mt1, mp);

	if (mpz_odd_p(mt1))
		mpz_add(mt1, mt1, mp);
	mpz_fdiv_q_2exp(mt0, mt1, 1);	/* T = 8Y^4 */

	mpz_sub(my, mt2, mt0);	/* Y' = (M (S-X') - T) mod p */
	mpz_mod(my, my, mp);
	mpz_export(r->Y.limb, &(r->Y.size), -1, 4, 0, 0, my);

	mpz_clear(mx);
	mpz_clear(my);
	mpz_clear(mz);
	mpz_clear(ma);
	mpz_clear(mp);
	mpz_clear(mt0);
	mpz_clear(mt1);
	mpz_clear(mt2);
#else
#ifdef QLIP_MOD_USE_MONT
	if (z_is_one)
		q_copy(&tmp[0], &curve->a);
	else {
		/* Z^2 mod p */
		q_mont_mul(ctx, &tmp[0], &p->Z, &p->Z, curve->mp);
		/* Z^4 mod p */
		q_mont_mul(ctx, &tmp[0], &tmp[0], &tmp[0], curve->mp);
		/* aZ^4 mod p */
		q_mont_mul(ctx, &tmp[0], &tmp[0], &curve->a, curve->mp);
	}

	/* X^2 mod p */
	q_mont_mul(ctx, &tmp[1], &p->X, &p->X, curve->mp);
	/* M = (X^2+aZ^4) mod p */
	q_modadd(ctx, &tmp[0], &tmp[0], &tmp[1], &curve->p);
	/* M = (2X^2+aZ^4) mod p */
	q_modadd(ctx, &tmp[0], &tmp[0], &tmp[1], &curve->p);
	/* M = (3X^2+aZ^4) mod p */
	q_modadd(ctx, &tmp[1], &tmp[0], &tmp[1], &curve->p);

	q_modadd(ctx, &tmp[0], &p->Y, &p->Y, &curve->p); /* 2Y mod p */
	if (z_is_one) {
		q_copy(&r->Z, &tmp[0]);	/* Z' = 2Y mod p */
	} else {
		/* Z' = 2YZ mod p */
		q_mont_mul(ctx, &r->Z, &p->Z, &tmp[0], curve->mp);
	}

	/* 4Y^2 mod p */
	q_mont_mul(ctx, &tmp[0], &tmp[0], &tmp[0], curve->mp);
	/* S = 4XY^2 mod p */
	q_mont_mul(ctx, &tmp[2], &tmp[0], &p->X, curve->mp);

	/* M^2 mod p */
	q_mont_mul(ctx, &r->X, &tmp[1], &tmp[1], curve->mp);
	/* X' = (M^2-S) mod p */
	q_modsub(ctx, &r->X, &r->X, &tmp[2], &curve->p);
	/* X' = (M^2-2S) mod p */
	q_modsub(ctx, &r->X, &r->X, &tmp[2], &curve->p);

	/* (S-X') mod p */
	q_modsub(ctx, &tmp[2], &tmp[2], &r->X, &curve->p);
	/* M (S-X') mod p */
	q_mont_mul(ctx, &tmp[2], &tmp[1], &tmp[2], curve->mp);

	/* replace 8Y^4 = 8Y^2 * Y^2 with 8Y^4 = 16Y^4/2 */
		/* 16Y^4 mod p */
	q_mont_mul(ctx, &tmp[1], &tmp[0], &tmp[0], curve->mp);

	/*the mod_div2 has to be computed in non-residue format */
	q_set_one(&tmp[0]);
	q_mont_mul(ctx, &tmp[1], &tmp[1], &tmp[0], curve->mp);
	q_mod_div2(ctx, &tmp[0], &tmp[1], &curve->p);	/* T = 8Y^4 */
	q_mont_mul(ctx, &tmp[0], &tmp[0], &curve->mp->rr, curve->mp);

	/* M = (X^2+aZ^4) mod p */
	q_modsub(ctx, &r->Y, &tmp[2], &tmp[0], &curve->p);
#else
	if (z_is_one)
		q_copy(&tmp[0], &curve->a);
	else {
		q_modsqr(ctx, &tmp[0], &p->Z, &curve->p);	/* Z^2 mod p */
		q_modsqr(ctx, &tmp[0], &tmp[0], &curve->p);	/* Z^4 mod p */
		q_modmul(ctx, &tmp[0], &tmp[0], &curve->a,
			&curve->p);	/* aZ^4 mod p */
	}

	q_modsqr(ctx, &tmp[1], &p->X, &curve->p);	/* X^2 mod p */
	/* M = (X^2+aZ^4) mod p */
	q_modadd(ctx, &tmp[0], &tmp[0], &tmp[1], &curve->p);
	/* M = (2X^2+aZ^4) mod p */
	q_modadd(ctx, &tmp[0], &tmp[0], &tmp[1], &curve->p);
	/* M = (3X^2+aZ^4) mod p */
	q_modadd(ctx, &tmp[1], &tmp[0], &tmp[1], &curve->p);

	q_modadd(ctx, &tmp[0], &p->Y, &p->Y, &curve->p); /* 2Y mod p */
	if (z_is_one) {
		q_copy(&r->Z, &tmp[0]);	/* Z' = 2Y mod p */
	} else {
		/* Z' = 2YZ mod p */
		q_modmul(ctx, &r->Z, &p->Z, &tmp[0], &curve->p);
	}

	q_modsqr(ctx, &tmp[0], &tmp[0], &curve->p);	/* 4Y^2 mod p */
	/* S = 4XY^2 mod p */
	q_modmul(ctx, &tmp[2], &tmp[0], &p->X, &curve->p);

	q_modsqr(ctx, &r->X, &tmp[1], &curve->p);	/* M^2 mod p */
	/* X' = (M^2-S) mod p */
	q_modsub(ctx, &r->X, &r->X, &tmp[2], &curve->p);
	/* X' = (M^2-2S) mod p */
	q_modsub(ctx, &r->X, &r->X, &tmp[2], &curve->p);

	/* (S-X') mod p */
	q_modsub(ctx, &tmp[2], &tmp[2], &r->X, &curve->p);
	/* M (S-X') mod p */
	q_modmul(ctx, &tmp[2], &tmp[1], &tmp[2], &curve->p);

	/* 16Y^4 mod p */
	q_modsqr(ctx, &tmp[1], &tmp[0], &curve->p);
	/* T = 8Y^4 */
	q_mod_div2(ctx, &tmp[0], &tmp[1], &curve->p);
	/* Y' = (M (S-X') - T) mod p */
	q_modsub(ctx, &r->Y, &tmp[2], &tmp[0], &curve->p);

#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_GMP */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_PKE_HW */

Q_ECP_PT_DBL_PRJ_EXIT:
	ctx->status = status;
	return ctx->status;
}

/*
 * q_ecp_pt_add_prj ()
 * Description: This function performs EC point addition in the
 * Jocobian projective coordinate.
 * @param ctx		QLIP context pointer
 * @param r		Result EC point pointer
 * @param p0		First EC point pointer
 * @param p1		Second EC point pointer
 * @param curve	EC curve parameter pointer
 * @param tmp		Temporary q_lint pointer
 */
int32_t q_ecp_pt_add_prj(struct q_lip_ctx *ctx,
							struct q_point *r,
							struct q_point *p0,
							struct q_point *p1,
							struct q_curve *curve,
							struct q_lint *tmp)
{
	int32_t status = Q_SUCCESS;
	int z0_is_one, z1_is_one;

#ifdef QLIP_USE_PKE_HW
	uint32_t blen_n;
	int i;
#endif

#ifdef QLIP_USE_PKA_HW
	struct opcode sequence[QLIP_PKA_ECP_ADD_SEQ_LEN];
	uint32_t lir_p;
#endif

	z0_is_one = q_pt_is_affine(p0);
	z1_is_one = q_pt_is_affine(p1);

#ifdef QLIP_USE_PKE_HW
	blen_n = curve->p.size * MACHINE_WD;

	for (i = 0; i < 6; i++) {
		tmp[i].size = curve->p.size;
		q_set_zero(&tmp[i]);
		tmp[i].size = curve->p.size;
	}

	if (z1_is_one)
		q_copy(&tmp[0], &p0->X);
	else {
		q_modmul_hw(tmp[4].limb, blen_n, curve->p.limb,
			p1->Z.limb, p1->Z.limb);	/* Z2^2 mod p */
		q_modmul_hw(tmp[0].limb, blen_n, curve->p.limb,
			tmp[4].limb, p0->X.limb);	/* U1=X1*Z2^2 mod p */
	}

	if (z1_is_one)
		q_copy(&tmp[1], &p0->Y);
	else {
		q_modmul_hw(tmp[4].limb, blen_n, curve->p.limb,
			tmp[4].limb, p1->Z.limb);	/* Z2^3 mod p */
		q_modmul_hw(tmp[1].limb, blen_n, curve->p.limb,
			tmp[4].limb, p0->Y.limb);	/* S1=Y1*Z2^3 mod p */
	}

	if (z0_is_one) {
		q_copy(&tmp[2], &p1->X);
		q_copy(&tmp[3], &p1->Y);
	} else {
		q_modmul_hw(tmp[4].limb, blen_n, curve->p.limb,
			p0->Z.limb, p0->Z.limb);	/* Z1^2 mod p */
		q_modmul_hw(tmp[2].limb, blen_n, curve->p.limb,
			tmp[4].limb, p1->X.limb);	/* U2=X2*Z1^2 mod p */
		q_modmul_hw(tmp[4].limb, blen_n, curve->p.limb,
			tmp[4].limb, p0->Z.limb);	/* Z1^3 mod p */
		q_modmul_hw(tmp[3].limb, blen_n, curve->p.limb,
			tmp[4].limb, p1->Y.limb);	/* S2=Y2*Z1^3 mod p */
	}

	q_modsub_hw(tmp[4].limb, blen_n, curve->p.limb,
		tmp[0].limb, tmp[2].limb);	/* W = U1-U2 */
	q_modsub_hw(tmp[5].limb, blen_n, curve->p.limb,
		tmp[1].limb, tmp[3].limb);	/* R = S1-S2 */

	q_modadd_hw(tmp[0].limb, blen_n, curve->p.limb,
		tmp[0].limb, tmp[2].limb);	/* T=U1+U2 */
	q_modadd_hw(tmp[1].limb, blen_n, curve->p.limb,
		tmp[1].limb, tmp[3].limb);	/* M=S1+S2 */

	q_copy(&tmp[2], &tmp[4]);	/* Z3 = Z1*Z2*W */
	if (!z0_is_one)
		q_modmul_hw(tmp[2].limb, blen_n, curve->p.limb, tmp[2].limb,
					p0->Z.limb);
	if (!z1_is_one)
		q_modmul_hw(tmp[2].limb, blen_n, curve->p.limb, tmp[2].limb,
					p1->Z.limb);
	q_copy(&r->Z, &tmp[2]);
	r->Z.size = curve->p.size;
	Q_NORMALIZE(r->Z.limb, r->Z.size);

	q_modmul_hw(r->X.limb, blen_n, curve->p.limb,
		tmp[5].limb, tmp[5].limb);	/* R^2 */
	q_modmul_hw(tmp[2].limb, blen_n, curve->p.limb,
		tmp[4].limb, tmp[4].limb);	/* W^2 */
	q_modmul_hw(tmp[0].limb, blen_n, curve->p.limb,
		tmp[2].limb, tmp[0].limb);	/* T*W^2 */
	q_modsub_hw(r->X.limb, blen_n, curve->p.limb,
		r->X.limb, tmp[0].limb);	/* X3 = R^2 - TW^2 */
	r->X.size = curve->p.size;
	Q_NORMALIZE(r->X.limb, r->X.size);

	q_modadd_hw(tmp[3].limb, blen_n, curve->p.limb,
		r->X.limb, r->X.limb);	/* 2*X3 */
	q_modsub_hw(tmp[3].limb, blen_n, curve->p.limb,
		tmp[0].limb, tmp[3].limb);	/* V = T^W^2 - 2*X3 */

	q_modmul_hw(tmp[3].limb, blen_n, curve->p.limb,
		tmp[3].limb, tmp[5].limb);	/* V*R */
	q_modmul_hw(tmp[2].limb, blen_n, curve->p.limb,
		tmp[2].limb, tmp[4].limb);	/* W^3 */
	q_modmul_hw(tmp[2].limb, blen_n, curve->p.limb,
		tmp[2].limb, tmp[1].limb);	/* M*W^3 */
	q_modsub_hw(tmp[0].limb, blen_n, curve->p.limb,
		tmp[3].limb, tmp[2].limb);	/* Y3 = V*R - M*W^3 */
	q_mod_div2(ctx, &r->Y, &tmp[0], &curve->p);
	r->Y.size = curve->p.size;
	Q_NORMALIZE(r->Y.limb, r->Y.size);

#else
#ifdef QLIP_USE_PKA_HW
	r->X.size = curve->mp->n.size;
	r->Y.size = curve->mp->n.size;
	r->Z.size = curve->mp->n.size;

	lir_p = q_pka_sel_lir(curve->p.size);

	if (q_ecp_prj_pka_init(ctx, curve, p0, p1))
		goto Q_ECP_PT_ADD_PRJ_EXIT;

	q_ecp_pt_add_prj_pka(sequence, lir_p);
	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(QLIP_PKA_ECP_ADD_SEQ_LEN, sequence);

	q_ecp_prj_pka_get_result(ctx, r);

#else
#ifdef QLIP_USE_GMP
	mpz_t mx0, my0, mz0, mx1, my1, mz1, mp, mt0, mt1, mt2, mt3, mt4, mt5;
	mpz_init(mx0);
	mpz_init(my0);
	mpz_init(mz0);
	mpz_init(mx1);
	mpz_init(my1);
	mpz_init(mz1);
	mpz_init(mp);
	mpz_init(mt0);
	mpz_init(mt1);
	mpz_init(mt2);
	mpz_init(mt3);
	mpz_init(mt4);
	mpz_init(mt5);

	mpz_import(mx0, p0->X.size, -1, 4, 0, 0, p0->X.limb);
	mpz_import(my0, p0->Y.size, -1, 4, 0, 0, p0->Y.limb);
	mpz_import(mz0, p0->Z.size, -1, 4, 0, 0, p0->Z.limb);
	mpz_import(mx1, p1->X.size, -1, 4, 0, 0, p1->X.limb);
	mpz_import(my1, p1->Y.size, -1, 4, 0, 0, p1->Y.limb);
	mpz_import(mz1, p1->Z.size, -1, 4, 0, 0, p1->Z.limb);
	mpz_import(mp, curve->p.size, -1, 4, 0, 0, curve->p.limb);

	if (z1_is_one)
		mpz_set(mt0, mx0);
	else {
		mpz_mul(mt4, mz1, mz1);	/* Z2^2 mod p */
		mpz_mul(mt0, mt4, mx0);	/* U1=X1*Z2^2 mod p */
		mpz_mod(mt0, mt0, mp);
	}

	if (z1_is_one)
		mpz_set(mt1, my0);
	else {
		mpz_mul(mt4, mt4, mz1);	/* Z2^3 mod p */
		mpz_mul(mt1, mt4, my0);	/* S1=Y1*Z2^3 mod p */
		mpz_mod(mt1, mt1, mp);
	}

	if (z0_is_one) {
		mpz_set(mt2, mx1);
		mpz_set(mt3, my1);
	} else {
		mpz_mul(mt4, mz0, mz0);	/* Z1^2 mod p */
		mpz_mul(mt2, mt4, mx1);	/* U2=X2*Z1^2 mod p */
		mpz_mod(mt2, mt2, mp);
		mpz_mul(mt4, mt4, mz0);	/* Z1^3 mod p */
		mpz_mul(mt3, mt4, my1);	/* S2=Y2*Z1^2 mod p */
		mpz_mod(mt3, mt3, mp);
	}

	mpz_sub(mt4, mt0, mt2);	/* W = U1-U2 */
	mpz_mod(mt4, mt4, mp);
	mpz_sub(mt5, mt1, mt3);	/* R = S1-S2 */
	mpz_mod(mt5, mt5, mp);
	mpz_add(mt0, mt0, mt2);	/* T=U1+U2 */
	mpz_mod(mt0, mt0, mp);
	mpz_add(mt1, mt1, mt3);	/* M=S1+S2 */
	mpz_mod(mt1, mt1, mp);

	mpz_set(mt2, mt4);	/* Z3 = Z1*Z2*W */
	if (!z0_is_one) {
		mpz_mul(mt2, mt2, mz0);
		mpz_mod(mt2, mt2, mp);
	}
	if (!z1_is_one) {
		mpz_mul(mt2, mt2, mz1);
		mpz_mod(mt2, mt2, mp);
	}
	mpz_export(r->Z.limb, &(r->Z.size), -1, 4, 0, 0, mt2);

	mpz_mul(mx0, mt5, mt5);	/* R^2 */
	mpz_mul(mt2, mt4, mt4);	/* W^2 */
	mpz_mul(mt0, mt2, mt0);	/* T*W^2 */
	mpz_sub(mx0, mx0, mt0);	/* X3 = R^2 - TW^2 */
	mpz_mod(mx0, mx0, mp);
	mpz_export(r->X.limb, &(r->X.size), -1, 4, 0, 0, mx0);

	mpz_add(mt3, mx0, mx0);	/* 2*X3 */
	mpz_sub(mt3, mt0, mt3);	/* V = T^W^2 - 2*X3 */
	mpz_mod(mt3, mt3, mp);

	mpz_mul(mt3, mt3, mt5);	/* V*R */
	mpz_mul(mt2, mt2, mt4);	/* W^3 */
	mpz_mul(mt2, mt2, mt1);	/* M*W^3 */
	mpz_mod(mt2, mt2, mp);
	mpz_sub(mt0, mt3, mt2);	/* Y3 = V*R - M*W^3 */
	mpz_mod(mt0, mt0, mp);

	if (mpz_odd_p(mt0))
		mpz_add(mt0, mt0, mp);
	mpz_fdiv_q_2exp(my0, mt0, 1);
	mpz_export(r->Y.limb, &(r->Y.size), -1, 4, 0, 0, my0);

	mpz_clear(mx0);
	mpz_clear(my0);
	mpz_clear(mz0);
	mpz_clear(mx1);
	mpz_clear(my1);
	mpz_clear(mz1);
	mpz_clear(mp);
	mpz_clear(mt0);
	mpz_clear(mt1);
	mpz_clear(mt2);
	mpz_clear(mt3);
	mpz_clear(mt4);
	mpz_clear(mt5);
#else
#ifdef QLIP_MOD_USE_MONT
	if (z1_is_one)
		q_copy(&tmp[0], &p0->X);
	else {
		/* Z2^2 mod p */
		q_mont_mul(ctx, &tmp[4], &p1->Z, &p1->Z, curve->mp);
		/* U1=X1*Z2^2 mod p */
		q_mont_mul(ctx, &tmp[0], &tmp[4], &p0->X, curve->mp);
	}

	if (z1_is_one)
		q_copy(&tmp[1], &p0->Y);
	else {
		/* Z2^3 mod p */
		q_mont_mul(ctx, &tmp[4], &tmp[4], &p1->Z, curve->mp);
		/* S1=Y1*Z2^3 mod p */
		q_mont_mul(ctx, &tmp[1], &tmp[4], &p0->Y, curve->mp);
	}

	if (z0_is_one) {
		q_copy(&tmp[2], &p1->X);
		q_copy(&tmp[3], &p1->Y);
	} else {
		/* Z1^2 mod p */
		q_mont_mul(ctx, &tmp[4], &p0->Z, &p0->Z, curve->mp);
		/* U2=X2*Z1^2 mod p */
		q_mont_mul(ctx, &tmp[2], &tmp[4], &p1->X, curve->mp);
		/* Z1^3 mod p */
		q_mont_mul(ctx, &tmp[4], &tmp[4], &p0->Z, curve->mp);
		/* S2=Y2*Z1^3 mod p */
		q_mont_mul(ctx, &tmp[3], &tmp[4], &p1->Y, curve->mp);
	}
	/* W = U1-U2 */
	q_modsub(ctx, &tmp[4], &tmp[0], &tmp[2], &curve->p);
	/* R = S1-S2 */
	q_modsub(ctx, &tmp[5], &tmp[1], &tmp[3], &curve->p);
	/* T=U1+U2 */
	q_modadd(ctx, &tmp[0], &tmp[0], &tmp[2], &curve->p);
	/* M=S1+S2 */
	q_modadd(ctx, &tmp[1], &tmp[1], &tmp[3], &curve->p);

	q_copy(&tmp[2], &tmp[4]);	/* Z3 = Z1*Z2*W */
	if (!z0_is_one)
		q_mont_mul(ctx, &tmp[2], &p0->Z, &tmp[2], curve->mp);
	if (!z1_is_one)
		q_mont_mul(ctx, &tmp[2], &p1->Z, &tmp[2], curve->mp);
	q_copy(&r->Z, &tmp[2]);
	/* R^2 */
	q_mont_mul(ctx, &r->X, &tmp[5], &tmp[5], curve->mp);
	/* W^2 */
	q_mont_mul(ctx, &tmp[2], &tmp[4], &tmp[4], curve->mp);
	/* T*W^2 */
	q_mont_mul(ctx, &tmp[0], &tmp[2], &tmp[0], curve->mp);
	/* X3 = R^2 - TW^2 */
	q_modsub(ctx, &r->X, &r->X, &tmp[0], &curve->p);
	/* 2*X3 */
	q_modadd(ctx, &tmp[3], &r->X, &r->X, &curve->p);
	/* V = T^W^2 - 2*X3 */
	q_modsub(ctx, &tmp[3], &tmp[0], &tmp[3], &curve->p);
	/* V*R */
	q_mont_mul(ctx, &tmp[3], &tmp[3], &tmp[5], curve->mp);
	/* W^3 */
	q_mont_mul(ctx, &tmp[2], &tmp[2], &tmp[4], curve->mp);
	/* M*W^3 */
	q_mont_mul(ctx, &tmp[2], &tmp[2], &tmp[1], curve->mp);
	/* Y3 = V*R - M*W^3 */
	q_modsub(ctx, &tmp[0], &tmp[3], &tmp[2], &curve->p);

	/*the mod_div2 has to be computed in non-residue format */
	q_set_one(&tmp[1]);
	q_mont_mul(ctx, &tmp[0], &tmp[0], &tmp[1], curve->mp);
	q_mod_div2(ctx, &r->Y, &tmp[0], &curve->p);
	q_mont_mul(ctx, &r->Y, &r->Y, &curve->mp->rr, curve->mp);

#else
	if (z1_is_one)
		q_copy(&tmp[0], &p0->X);
	else {
		/* Z2^2 mod p */
		q_modsqr(ctx, &tmp[4], &p1->Z, &curve->p);
		/* U1=X1*Z2^2 mod p */
		q_modmul(ctx, &tmp[0], &tmp[4], &p0->X, &curve->p);
	}

	if (z1_is_one)
		q_copy(&tmp[1], &p0->Y);
	else {
		/* Z2^3 mod p */
		q_modmul(ctx, &tmp[4], &tmp[4], &p1->Z, &curve->p);
		/* S1=Y1*Z2^3 mod p */
		q_modmul(ctx, &tmp[1], &tmp[4], &p0->Y, &curve->p);
	}

	if (z0_is_one) {
		q_copy(&tmp[2], &p1->X);
		q_copy(&tmp[3], &p1->Y);
	} else {
		/* Z1^2 mod p */
		q_modsqr(ctx, &tmp[4], &p0->Z, &curve->p);
		/* U2=X2*Z1^2 mod p */
		q_modmul(ctx, &tmp[2], &tmp[4], &p1->X, &curve->p);
		/* Z1^3 mod p */
		q_modmul(ctx, &tmp[4], &tmp[4], &p0->Z, &curve->p);
		/* S2=Y2*Z1^2 mod p */
		q_modmul(ctx, &tmp[3], &tmp[4], &p1->Y, &curve->p);
	}
	/* W = U1-U2 */
	q_modsub(ctx, &tmp[4], &tmp[0], &tmp[2], &curve->p);
	/* R = S1-S2 */
	q_modsub(ctx, &tmp[5], &tmp[1], &tmp[3], &curve->p);

	/* T=U1+U2 */
	q_modadd(ctx, &tmp[0], &tmp[0], &tmp[2], &curve->p);
	/* M=S1+S2 */
	q_modadd(ctx, &tmp[1], &tmp[1], &tmp[3], &curve->p);

	q_copy(&tmp[2], &tmp[4]);	/* Z3 = Z1*Z2*W */
	if (!z0_is_one)
		q_modmul(ctx, &tmp[2], &p0->Z, &tmp[2], &curve->p);
	if (!z1_is_one)
		q_modmul(ctx, &tmp[2], &p1->Z, &tmp[2], &curve->p);
	q_copy(&r->Z, &tmp[2]);

	q_modsqr(ctx, &r->X, &tmp[5], &curve->p);	/* R^2 */
	q_modsqr(ctx, &tmp[2], &tmp[4], &curve->p);	/* W^2 */
	/* T*W^2 */
	q_modmul(ctx, &tmp[0], &tmp[2], &tmp[0], &curve->p);
	/* X3 = R^2 - TW^2 */
	q_modsub(ctx, &r->X, &r->X, &tmp[0], &curve->p);

	/* 2*X3 */
	q_modadd(ctx, &tmp[3], &r->X, &r->X, &curve->p);
	/* V = T^W^2 - 2*X3 */
	q_modsub(ctx, &tmp[3], &tmp[0], &tmp[3], &curve->p);

	/* V*R */
	q_modmul(ctx, &tmp[3], &tmp[3], &tmp[5], &curve->p);
	/* W^3 */
	q_modmul(ctx, &tmp[2], &tmp[2], &tmp[4], &curve->p);
	/* M*W^3 */
	q_modmul(ctx, &tmp[2], &tmp[2], &tmp[1], &curve->p);
	/* Y3 = V*R - M*W^3 */
	q_modsub(ctx, &tmp[0], &tmp[3], &tmp[2], &curve->p);
	q_mod_div2(ctx, &r->Y, &tmp[0], &curve->p);

#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_GMP */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_PKE_HW */

Q_ECP_PT_ADD_PRJ_EXIT:
	ctx->status = status;
	return ctx->status;
}

/*
 * q_ecp_prj_2_affine ()
 * Description: This function converts EC point from Jocobian
 * projective coordinate to affine coordinate.
 * X = X/Z^2, Y=Y/Z^3
 * Limitation:
 * This sequence supports up to type-G LIR, i.e., 1536-bit modulus
 * size.
 */
int32_t q_ecp_prj_2_affine(struct q_lip_ctx *ctx,
							  struct q_point *r,
							  struct q_point *p,
							  struct q_curve *curve)
{
	int32_t status = Q_SUCCESS;
	struct q_lint zi;
	uint32_t size;
	int i;

#ifdef QLIP_USE_PKE_HW
	uint32_t blen_n;
#endif

#ifdef QLIP_USE_PKA_HW
	uint32_t pka_status;
	uint32_t lir_p;
	struct opcode sequence[14];
#endif

	/*already in Affine */
	if (q_pt_is_affine(p)) {
		if (r != p)
			q_pt_copy(r, p);
		goto Q_ECP_PRJ_2_AFFINE_EXIT;
	}

	size = p->Z.alloc;
	q_init(ctx, &zi, size);

	/*
	 * the following operations can be calculated
	 * via hardware accelearation
	 */
#ifdef QLIP_USE_PKE_HW
	blen_n = curve->p.size * MACHINE_WD;

	q_modinv_hw(zi.limb, blen_n, curve->p.limb, p->Z.limb);
	q_modmul_hw(p->Z.limb, blen_n, curve->p.limb, zi.limb, zi.limb);
	q_modmul_hw(r->X.limb, blen_n, curve->p.limb,
		p->X.limb, p->Z.limb);
	q_modmul_hw(r->Z.limb, blen_n, curve->p.limb, p->Z.limb, zi.limb);
	q_modmul_hw(r->Y.limb, blen_n, curve->p.limb,
		p->Y.limb, p->Z.limb);
#else
#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 14; i++)
		sequence[i].ptr = NULL;

	/*
	 * assume LIR_0, LIR_1 and LIR_2 contains
	 * the Montgomery context
	 * create command sequence
	 */
	lir_p = q_pka_sel_lir(curve->p.size);

	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), p->X.size);
	sequence[0].ptr = p->X.limb;

	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), p->Y.size);
	sequence[1].ptr = p->Y.limb;

	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), p->Z.size);
	sequence[2].ptr = p->Z.limb;

	/* compute Z inverse */
	sequence[3].op1 = PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 6),
		PKA_NULL);
	sequence[3].op2 = PACK_OP2(PKA_NULL, 2);

	/* 1/Z */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MODINV, PKA_LIR(lir_p, 7),
			PKA_LIR(lir_p, 5));
	sequence[4].op2 = PACK_OP2(PKA_LIR(lir_p, 6), PKA_LIR(lir_p, 0));

	/* 1/Z^2 */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 6),
			PKA_LIR(lir_p, 7));
	sequence[5].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* 1/Z^3 */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
			PKA_LIR(lir_p, 7));
	sequence[6].op2 = PACK_OP2(PKA_LIR(lir_p, 6), PKA_LIR(lir_p, 0));

	/* X */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 7),
			PKA_LIR(lir_p, 3));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_p, 6), PKA_LIR(lir_p, 0));

	/* Y */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 8),
			PKA_LIR(lir_p, 4));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_p, 5), PKA_LIR(lir_p, 0));

	/* convert X and Y back from residue domain */
	sequence[9].op1 = PACK_OP1(0, PKA_OP_SLIR, PKA_LIR(lir_p, 5), PKA_NULL);
	sequence[9].op2 = PACK_OP2(PKA_NULL, 1);

	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 6),
			PKA_LIR(lir_p, 7));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_p, 5), PKA_LIR(lir_p, 0));

	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 7),
			PKA_LIR(lir_p, 8));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_p, 5), PKA_LIR(lir_p, 0));

	sequence[12].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 6), r->X.size);

	sequence[13].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 7), r->Y.size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(14, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			status = Q_ERR_PKA_HW_ERR;
			goto Q_ECP_PRJ_2_AFFINE_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(r->X.size, r->X.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(r->Y.size, r->Y.limb);

#else
#ifdef QLIP_USE_GMP
	mpz_t mp, mx, my, mz, mzi;
	mpz_init(mp);
	mpz_init(mx);
	mpz_init(my);
	mpz_init(mz);
	mpz_init(mzi);

	mpz_import(mx, p->X.size, -1, 4, 0, 0, p->X.limb);
	mpz_import(my, p->Y.size, -1, 4, 0, 0, p->Y.limb);
	mpz_import(mz, p->Z.size, -1, 4, 0, 0, p->Z.limb);
	mpz_import(mp, curve->p.size, -1, 4, 0, 0, curve->p.limb);

	mpz_invert(mzi, mz, mp);
	mpz_mul(mz, mzi, mzi);

	mpz_mul(mx, mx, mz);
	mpz_mod(mx, mx, mp);

	mpz_mul(mz, mzi, mz);
	mpz_mod(mz, mz, mp);

	mpz_mul(my, my, mz);
	mpz_mod(my, my, mp);

	mpz_export(r->X.limb, &(r->X.size), -1, 4, 0, 0, mx);
	mpz_export(r->Y.limb, &(r->Y.size), -1, 4, 0, 0, my);

	mpz_clear(mp);
	mpz_clear(mx);
	mpz_clear(my);
	mpz_clear(mz);
	mpz_clear(mzi);
#else
#ifdef QLIP_MOD_USE_MONT
	/*
	 * assume point coordinates are already in residue
	 * format for q_ecp_prj_2_affine
	 */
	struct q_lint t1;
	q_init(ctx, &t1, curve->p.alloc);

	/* get inverse */
	t1.size = 1;
	t1.limb[0] = 2L;
	q_usub(&t1, &curve->p, &t1);
	q_modexp_mont(ctx, &zi, &p->Z, &t1, curve->mp);

	/* 1/Z square */
	q_mont_mul(ctx, &p->Z, &zi, &zi, curve->mp);
	q_mont_mul(ctx, &r->X, &p->X, &p->Z, curve->mp);
	q_mont_mul(ctx, &r->Z, &p->Z, &zi, curve->mp);
	q_mont_mul(ctx, &r->Y, &p->Y, &r->Z, curve->mp);

	/* convert back from residue */
	q_set_one(&zi);
	q_mont_mul(ctx, &r->X, &r->X, &zi, curve->mp);
	q_mont_mul(ctx, &r->Y, &r->Y, &zi, curve->mp);
	q_free(ctx, &t1);
#else
	q_modpinv(ctx, &zi, &p->Z, &curve->p);
	q_modsqr(ctx, &p->Z, &zi, &curve->p);
	q_modmul(ctx, &r->X, &p->X, &p->Z, &curve->p);
	q_modmul(ctx, &r->Z, &p->Z, &zi, &curve->p);
	q_modmul(ctx, &r->Y, &p->Y, &r->Z, &curve->p);
#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_GMP */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_PKE_HW */

	/* set Z to one */
	q_set_one(&r->Z);

	q_free(ctx, &zi);

Q_ECP_PRJ_2_AFFINE_EXIT:
	ctx->status = status;
	return ctx->status;
}

#ifdef QLIP_USE_PKA_HW

/*
 * q_ecp_prj_pka_init ()
 * Description: This function initializes the PKA HW for EC point
 * addition or doubling operations.  Specifically, it loads the
 * required parameters to the proper LIR registers as follows.
 * x[0] = n  (curve prime number Montgomery context)
 * x[1] = np (curve prime number Montgomery context)
 * x[2] = rr (curve prime number Montgomery context)
 * x[3] = pt0.X (point0 X coordinate)
 * x[4] = pt0.Y (point0 Y coordinate)
 * x[5] = pt0.Z (point0 Z coordinate)
 * x[6] = pt1.X (point1 X coordinate)
 * x[7] = pt1.Y (point1 Y coordinate)
 * x[8] = pt1.Z (point1 Z coordinate)
 * x[9] = a (curve parameter a)
 */
int32_t q_ecp_prj_pka_init(struct q_lip_ctx *ctx,
							  struct q_curve *curve,
							  struct q_point *pt0,
							  struct q_point *pt1)
{
	struct opcode sequence[10];
	uint32_t lir_p;

	/* create command sequence */
	lir_p = q_pka_sel_lir(curve->p.size);

	/* Load n */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0),
			curve->mp->n.size);
	sequence[0].ptr = curve->mp->n.limb;

	/* Load np */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1),
			curve->mp->np.size);
	sequence[1].ptr = curve->mp->np.limb;

	/* Load rr */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2),
			curve->mp->rr.size);
	sequence[2].ptr = curve->mp->rr.limb;

	/* Load point0 coordinate x */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), pt0->X.size);
	sequence[3].ptr = pt0->X.limb;

	/* Load point0 coordinate y */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), pt0->Y.size);
	sequence[4].ptr = pt0->Y.limb;

	/* Load point0 coordinate z */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), pt0->Z.size);
	sequence[5].ptr = pt0->Z.limb;

	/* Load point1 coordinate x */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 6), pt1->X.size);
	sequence[6].ptr = pt1->X.limb;

	/* Load point1 coordinate y */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 7), pt1->Y.size);
	sequence[7].ptr = pt1->Y.limb;

	/* Load point1 coordinate z */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 8), pt1->Z.size);
	sequence[8].ptr = pt1->Z.limb;

	/* Load curve parameter a */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 9), curve->a.size);
	sequence[9].ptr = curve->a.limb;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(10, sequence);

	return ctx->status;
}
#endif /* QLIP_USE_PKA_HW */

#ifdef QLIP_USE_PKA_HW

/*
 * q_ecp_prj_pka_get_result ()
 * Description: This function reads back the EC point operation result.
 * Specifically, the result of PKA EC operations is located at:
 * pt.X = x[3]
 * pt.Y = x[4]
 * pt.Z = x[5]
 */
int32_t q_ecp_prj_pka_get_result(struct q_lip_ctx *ctx, struct q_point *pt)
{
	struct opcode sequence[3];
	uint32_t pka_status;
	uint32_t lir_p;

	lir_p = q_pka_sel_lir(pt->X.size);

	sequence[0].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 3), pt->X.size);
	sequence[0].ptr = NULL;

	sequence[1].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 4), pt->Y.size);
	sequence[1].ptr = NULL;

	sequence[2].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 5), pt->Z.size);
	sequence[2].ptr = NULL;

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(3, sequence);

	/* read result back */
	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE)) {
		if (pka_status & PKA_STAT_ERROR) {
			ctx->status = Q_ERR_PKA_HW_ERR;
			goto Q_ECP_PRJ_PKA_GET_RESULT_EXIT;
		}
	}
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(pt->X.size, pt->X.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(pt->Y.size, pt->Y.limb);

	while (!((pka_status = q_pka_hw_rd_status()) & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(pka_status);
	q_pka_hw_read_lir(pt->Z.size, pt->Z.limb);

Q_ECP_PRJ_PKA_GET_RESULT_EXIT:
	return ctx->status;
}
#endif /* QLIP_USE_PKA_HW */

#ifdef QLIP_USE_PKA_HW

/*
 * q_ecp_pt_dbl_prj_pka ()
 * Description: This function constructs the EC point doubling
 * sequence.  It assumes the following.
 * x0 = n  (constant)
 * x1 = np (constant)
 * x2 = rr (constant)
 * x3 = p.x (result r.x will replace this value)
 * x4 = p.y (result r.y will replace this value)
 * x5 = p.z (result r.z will replace this value)
 * x6 = unavailable
 * x7 = unavailable
 * x8 = unavailable
 * x9 = a (constant)
 * Notes:
 * 1. The values in contant LIRs should be preserved.
 * 2. Unavailable LIRs should NOT be used at all.
 * Limitation:
 * This sequence supports up to type-F LIR, i.e., 1024-bit
 * modulus size.
 */
void q_ecp_pt_dbl_prj_pka(struct opcode *sequence, uint32_t lir_p)
{
	int i;

	for (i = 0; i < 19; i++)
		sequence[i].ptr = NULL;

	/*
	 * Assumptions:
	 * x0 = n  (constant)
	 * x1 = np (constant)
	 * x2 = rr (constant)
	 * x3 = p.x (result r.x will replace this value)
	 * x4 = p.y (result r.y will replace this value)
	 * x5 = p.z (result r.z will replace this value)
	 * x6 = unavailable
	 * x7 = unavailable
	 * x8 = unavailable
	 * x9 = a (constant)
	 * Notes:
	 * 1. The values in contant LIRs should be preserved.
	 * 2. Unavailable LIRs should NOT be used at all.
	 */

	/* x11 = x5 ^ 2 mod x0  (Z^2) */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 5));
	sequence[0].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x11 = x11 ^ 2 mod x0  (Z^4) */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 11));
	sequence[1].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x11 = x9 * x11 mod x0 (a * Z^4) */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 9));
	sequence[2].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x10 = x3 ^ 2 mod x0  (X^2) */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 3));
	sequence[3].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x11 = x10 + x11 mod x0 (M = X^2 + a * Z^4) */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 10));
	sequence[4].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x11 = x10 + x11 mod x0 (M = 2 * X^2 + a * Z^4) */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 10));
	sequence[5].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x11 = x10 + x11 mod x0 (M = 3 * X^2 + a * Z^4) */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 10));
	sequence[6].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x10 = x4 + x4 mod x0 (2 * Y) */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 4));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 0));

	/* x5 = x10 * x5 mod x0 (Z' = 2 * Y * Z) */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
			PKA_LIR(lir_p, 10));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_p, 5), PKA_LIR(lir_p, 0));

	/* x10 = x10 ^ 2 mod x0 (4 * Y^2) */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 10));
	sequence[9].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x4 = x10 ^ 2 mod x0 (16 * Y^4) */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 10));
	sequence[10].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x4 = x4 / 2 mod x0 (T = 8 * Y^4) */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODDIV2, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 4));
	sequence[11].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x10 = x10 * x3 mod x0 (S = 4 * X * Y^2) */
	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 10));
	sequence[12].op2 = PACK_OP2(PKA_LIR(lir_p, 3), PKA_LIR(lir_p, 0));

	/* x3 = x11 ^ 2 mod x0 (M^2) */
	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 11));
	sequence[13].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x3 = x3 - x10 mod x0 (X' = (M^2 - S) */
	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 3));
	sequence[14].op2 = PACK_OP2(PKA_LIR(lir_p, 10), PKA_LIR(lir_p, 0));

	/* x3 = x3 - x10 mod x0 (X' = (M^2 - 2 * S) */
	sequence[15].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 3));
	sequence[15].op2 = PACK_OP2(PKA_LIR(lir_p, 10), PKA_LIR(lir_p, 0));

	/* x10 = x10 - x3 mod x0 (S - X') */
	sequence[16].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 10));
	sequence[16].op2 = PACK_OP2(PKA_LIR(lir_p, 3), PKA_LIR(lir_p, 0));

	/* x10 = x10 * x11 mod x0 (M * (S - X')) */
	sequence[17].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 10));
	sequence[17].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x4 = x10 - x4 mod x0 (Y' = (M * (S - X') - T) */
	sequence[18].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 10));
	sequence[18].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 0));
}
#endif /* QLIP_USE_PKA_HW */

#ifdef QLIP_USE_PKA_HW
/*
 * q_ecp_pt_add_prj_pka ()
 * Description: This function constructs the EC point addition
 * sequence.  It assumes the following.
 * x0 = n  (constant)
 * x1 = np (constant)
 * x2 = rr (constant)
 * x3 = p0.x (result r.x will replace this value)
 * x4 = p0.y (result r.y will replace this value)
 * x5 = p0.z (result r.z will replace this value)
 * x6 = p1.x (constant)
 * x7 = p1.y (constant)
 * x8 = p1.z (constant)
 * x9 = unavailable
 * Notes:
 * 1. The values in contant LIRs should be preserved.
 * 2. Unavailable LIRs should NOT be used at all.
 * Limitation:
 * This sequence supports up to type-F LIR, i.e., 1024-bit
 * modulus size.
 */
void q_ecp_pt_add_prj_pka(struct opcode *sequence, uint32_t lir_p)
{
	int i;
	for (i = 0; i < 25; i++)
		sequence[i].ptr = NULL;

	/*
	 * Assumptions:
	 * x0 = n  (constant)
	 * x1 = np (constant)
	 * x2 = rr (constant)
	 * x3 = p0.x (result r.x will replace this value)
	 * x4 = p0.y (result r.Y will replace this value)
	 * x5 = p0.z (result r.Z will replace this value)
	 * x6 = p1.x (constant)
	 * x7 = p1.y (constant)
	 * x8 = p1.z (constant)
	 * x9 = unavailable
	 * Notes:
	 * 1. The values in contant LIRs should be preserved.
	 * 2. Unavailable LIRs should NOT be used at all.
	 */

	/* x12 = x8 ^ 2 mod x0 (Z2^2) */
	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 12),
			PKA_LIR(lir_p, 8));
	sequence[0].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x3 = x3 * x12 mod x0 (U1 = X1 * Z2^2) */
	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 3));
	sequence[1].op2 = PACK_OP2(PKA_LIR(lir_p, 12), PKA_LIR(lir_p, 0));

	/* x12 = x8 * x12 mod x0 (Z2^3) */
	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 12),
			PKA_LIR(lir_p, 8));
	sequence[2].op2 = PACK_OP2(PKA_LIR(lir_p, 12), PKA_LIR(lir_p, 0));

	/* x12 = x4 * x12 mod x0 (S1 = Y1 * Z2^3) */
	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 12),
			PKA_LIR(lir_p, 4));
	sequence[3].op2 = PACK_OP2(PKA_LIR(lir_p, 12), PKA_LIR(lir_p, 0));

	/* x4 = x5 ^ 2 mod x0 (Z1^2) */
	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 5));
	sequence[4].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x11 = x5 * x4 mod x0 (Z1^3) */
	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 5));
	sequence[5].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 0));

	/* x4 = x6 * x4 mod x0 (U2 = X2 * Z1^2) */
	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 6));
	sequence[6].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 0));

	/* x13 = x3 - x4 mod x0 (W = U1 - U2) */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 13),
			PKA_LIR(lir_p, 3));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 0));

	/* x11 = x7 * x11 mod x0 (S2 = Y2 * Z1^3) */
	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 7));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x5 = x5 * x8 mod x0 (Z3 = Z1 * Z2) */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
			PKA_LIR(lir_p, 5));
	sequence[9].op2 = PACK_OP2(PKA_LIR(lir_p, 8), PKA_LIR(lir_p, 0));

	/* x5 = x5 * x13 mod x0 (Z3 = Z1 * Z2 * W) */
	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
			PKA_LIR(lir_p, 5));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_p, 13), PKA_LIR(lir_p, 0));

	/* x3 = x3 + x4 mod x0 (T = U1 + U2) */
	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 3));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_LIR(lir_p, 0));

	/* x4 = x12 - x11 mod x0 (R = S1 - S2) */
	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 12));
	sequence[12].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x12 = x12 + x11 mod x0 (M = S1 + S2) */
	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_p, 12),
			PKA_LIR(lir_p, 12));
	sequence[13].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x10 = x13 ^ 2 mod x0 (W^2) */
	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 13));
	sequence[14].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x11 = x3 * x10 mod x0 (T * W^2) */
	sequence[15].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 3));
	sequence[15].op2 = PACK_OP2(PKA_LIR(lir_p, 10), PKA_LIR(lir_p, 0));

	/* x3 = x4 ^ 2 mod x0 (R^2) */
	sequence[16].op1 =
		PACK_OP1(0, PKA_OP_MODSQR, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 4));
	sequence[16].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));

	/* x3 = x3 - x11 mod x0 (X3 = R^2 - T * W^2) */
	sequence[17].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 3));
	sequence[17].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x10 = x10 * x13 mod x0 (W^3) */
	sequence[18].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 10));
	sequence[18].op2 = PACK_OP2(PKA_LIR(lir_p, 13), PKA_LIR(lir_p, 0));

	/* x10 = x12 * x10 mod x0 (M * W^3) */
	sequence[19].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 12));
	sequence[19].op2 = PACK_OP2(PKA_LIR(lir_p, 10), PKA_LIR(lir_p, 0));

	/* x12 = x3 + x3 mod x0 (2 * X3) */
	sequence[20].op1 =
		PACK_OP1(0, PKA_OP_MODADD, PKA_LIR(lir_p, 12),
			PKA_LIR(lir_p, 3));
	sequence[20].op2 = PACK_OP2(PKA_LIR(lir_p, 3), PKA_LIR(lir_p, 0));

	/* x11 = x11 - x12 mod x0 (V = T * W^2 - 2 * X3) */
	sequence[21].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 11));
	sequence[21].op2 = PACK_OP2(PKA_LIR(lir_p, 12), PKA_LIR(lir_p, 0));

	/* x4 = x4 * x11 mod x0 (V * R) */
	sequence[22].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 4));
	sequence[22].op2 = PACK_OP2(PKA_LIR(lir_p, 11), PKA_LIR(lir_p, 0));

	/* x4 = x4 - x10 mod x0 (Y3 = V * R - M * W^3) */
	sequence[23].op1 =
		PACK_OP1(0, PKA_OP_MODSUB, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 4));
	sequence[23].op2 = PACK_OP2(PKA_LIR(lir_p, 10), PKA_LIR(lir_p, 0));

	/* x4 = x4 / 2 mod x0 */
	sequence[24].op1 =
		PACK_OP1(0, PKA_OP_MODDIV2, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 4));
	sequence[24].op2 = PACK_OP2(PKA_NULL, PKA_LIR(lir_p, 0));
}
#endif /* QLIP_USE_PKA_HW */

/*
 * The following are wrapper functions provided for the convenience
 * of usage.
 * q_ecp_pt_dbl () - point doubling in Affine coordinates
 * q_ecp_pt_add () - point addition in Affine coordinates
 * q_ecp_pt_sub () - point subtraction in Affine coordinates
 * These functions call the corresponding functions in Jacobian
 * coordinate system.
 * @param ctx		QLIP context pointer
 * @param r		Result EC point pointer
 * @param p		Input EC point pointer
 * @param curve	EC curve parameter pointer
 */
#ifndef REDUCED_RELEASE_CODE_SIZE
int32_t q_ecp_pt_dbl(struct q_lip_ctx *ctx,
						struct q_point *r,
						struct q_point *p,
						struct q_curve *curve)
{
	int32_t status = Q_SUCCESS;
	struct q_mont mp;
	struct q_point cp;
	struct q_lint dbl_tmp[6], ca;
	uint32_t lir_p;
	int i;

#ifdef QLIP_USE_PKA_HW
	struct opcode sequence[15];
#endif

	/*
	 * Testing Affine to make sure that the function does
	 * what it is supposed to do.This isn't completely necessary.
	 */
	if (!q_pt_is_affine(p)) {
		status = Q_ERR_EC_PT_NOT_AFFINE;
		goto Q_ECP_PT_DBL_EXIT;
	}
	/*make copy to the input*/
	status = q_init(ctx, &cp.X, p->X.alloc);
	status += q_init(ctx, &cp.Y, p->Y.alloc);
	status += q_init(ctx, &cp.Z, p->Z.alloc);
	status += q_init(ctx, &ca, curve->a.alloc);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_DBL_EXIT;

	status = q_pt_copy(&cp, p);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_DBL_EXIT;
	status = q_copy(&ca, &curve->a);
	if (status  != Q_SUCCESS)
		goto Q_ECP_PT_DBL_EXIT;

	for (i = 0; i < 6; i++)
		status += q_init(ctx, &dbl_tmp[i], curve->p.alloc + 1);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_DBL_EXIT;

#ifdef QLIP_MOD_USE_MONT
	status = q_init(ctx, &mp.n, curve->p.alloc);
	status += q_init(ctx, &mp.np, curve->p.alloc + 1);
	status += q_init(ctx, &mp.rr, curve->p.alloc + 1);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_DBL_EXIT;

	status = q_mont_init(ctx, &mp, &curve->p);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_DBL_EXIT;
	curve->mp = &mp;

	status = q_mont_mul(ctx, &cp.X, &cp.X, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp.Y, &cp.Y, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp.Z, &cp.Z, &mp.rr, &mp);
	status += q_mont_mul(ctx, &curve->a, &curve->a, &mp.rr, &mp);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_DBL_EXIT;

#endif

#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 15; i++)
		sequence[i].ptr = NULL;

	q_init(ctx, &mp.n, curve->p.alloc);
	q_init(ctx, &mp.np, curve->p.alloc + 1);
	q_init(ctx, &mp.rr, curve->p.alloc + 1);

	/* initialize hardware first */
	q_pka_hw_rst();

	q_mont_init(ctx, &mp, &curve->p);
	curve->mp = &mp;

	lir_p = q_pka_sel_lir(curve->p.size);

	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0),
			curve->mp->n.size);
	sequence[0].ptr = curve->mp->n.limb;

	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1),
			curve->mp->np.size);
	sequence[1].ptr = curve->mp->np.limb;

	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2),
			curve->mp->rr.size);
	sequence[2].ptr = curve->mp->rr.limb;

	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), cp.X.size);
	sequence[3].ptr = cp.X.limb;

	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), cp.Y.size);
	sequence[4].ptr = cp.Y.limb;

	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), cp.Z.size);
	sequence[5].ptr = cp.Z.limb;

	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 6), curve->a.size);
	sequence[6].ptr = curve->a.limb;

	/* conversion */
	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 3));
	sequence[7].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 4));
	sequence[8].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 12),
			PKA_LIR(lir_p, 5));
	sequence[9].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 6));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	/* read back */
	sequence[11].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 10),
			curve->p.size);

	sequence[12].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 11),
			curve->p.size);

	sequence[13].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 12),
			curve->p.size);

	sequence[14].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 3),
			curve->p.size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(15, sequence);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp.X.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp.Y.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp.Z.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, curve->a.limb);

	cp.X.size = curve->p.size;
	cp.Y.size = curve->p.size;
	cp.Z.size = curve->p.size;

	q_pka_hw_rst();
#endif

	/* read result back */
	if (q_ecp_pt_dbl_prj(ctx, r, &cp, curve, dbl_tmp))
		goto Q_ECP_PT_DBL_EXIT;
	q_ecp_prj_2_affine(ctx, r, r, curve);
	q_copy(&curve->a, &ca);

#ifdef QLIP_MOD_USE_MONT
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#endif

#ifdef QLIP_USE_PKA_HW
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#endif

	for (i = 5; i >= 0; i--)
		q_free(ctx, &dbl_tmp[i]);
	q_free(ctx, &ca);
	q_free(ctx, &cp.Z);
	q_free(ctx, &cp.Y);
	q_free(ctx, &cp.X);

Q_ECP_PT_DBL_EXIT:
	ctx->status = status;
	return ctx->status;
}
#endif /*REDUCED_RELEASE_CODE_SIZE*/

/*
 * q_ecp_pt_add ()
 * @param ctx		QLIP context pointer
 * @param r		Result EC point pointer
 * @param p0		First EC point pointer
 * @param p1		Second EC point pointer
 * @param curve	EC curve parameter pointer
 */
int32_t q_ecp_pt_add(struct q_lip_ctx *ctx,
						struct q_point *r,
						struct q_point *p0,
						struct q_point *p1,
						struct q_curve *curve)
{
	int32_t status = Q_SUCCESS;
	struct q_mont mp;
	struct q_point cp0, cp1;
	struct q_lint add_tmp[6];
	uint32_t lir_p;
	int i;

#ifdef QLIP_USE_PKA_HW
	struct opcode sequence[21];
#endif

	/*
	 * Testing Affine to make sure that the function does
	 * what it is supposed to do. This isn't completely necessary
	 */
	if (!q_pt_is_affine(p0)) {
		status = Q_ERR_EC_PT_NOT_AFFINE;
		goto Q_ECP_PT_ADD_EXIT;
	}

	if (!q_pt_is_affine(p1)) {
		status = Q_ERR_EC_PT_NOT_AFFINE;
		goto Q_ECP_PT_ADD_EXIT;
	}
	/*make copy to the input*/
	status = q_init(ctx, &cp0.X, p0->X.alloc);
	status += q_init(ctx, &cp0.Y, p0->Y.alloc);
	status += q_init(ctx, &cp0.Z, p0->Z.alloc);
	status += q_init(ctx, &cp1.X, p1->X.alloc);
	status += q_init(ctx, &cp1.Y, p1->Y.alloc);
	status += q_init(ctx, &cp1.Z, p1->Z.alloc);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_ADD_EXIT;

	q_pt_copy(&cp0, p0);
	q_pt_copy(&cp1, p1);

	for (i = 0; i < 6; i++)
		q_init(ctx, &add_tmp[i], curve->p.alloc + 1);

#ifdef QLIP_MOD_USE_MONT
	q_init(ctx, &mp.n, curve->p.alloc);
	q_init(ctx, &mp.np, curve->p.alloc + 1);
	q_init(ctx, &mp.rr, curve->p.alloc + 1);

	status = q_mont_init(ctx, &mp, &curve->p);
	curve->mp = &mp;

	status += q_mont_mul(ctx, &cp0.X, &cp0.X, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp0.Y, &cp0.Y, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp0.Z, &cp0.Z, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp1.X, &cp1.X, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp1.Y, &cp1.Y, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp1.Z, &cp1.Z, &mp.rr, &mp);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_ADD_EXIT;
#endif

#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 21; i++)
		sequence[i].ptr = NULL;

	q_init(ctx, &mp.n, curve->p.alloc);
	q_init(ctx, &mp.np, curve->p.alloc + 1);
	q_init(ctx, &mp.rr, curve->p.alloc + 1);

	/* initialize hardware first */
	q_pka_hw_rst();

	q_mont_init(ctx, &mp, &curve->p);
	curve->mp = &mp;

	lir_p = q_pka_sel_lir(curve->p.size);

	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0),
			curve->mp->n.size);
	sequence[0].ptr = curve->mp->n.limb;

	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1),
			curve->mp->np.size);
	sequence[1].ptr = curve->mp->np.limb;

	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2),
			curve->mp->rr.size);
	sequence[2].ptr = curve->mp->rr.limb;

	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), cp0.X.size);
	sequence[3].ptr = cp0.X.limb;

	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), cp0.Y.size);
	sequence[4].ptr = cp0.Y.limb;

	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), cp0.Z.size);
	sequence[5].ptr = cp0.Z.limb;

	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 6), cp1.X.size);
	sequence[6].ptr = cp1.X.limb;

	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 7), cp1.Y.size);
	sequence[7].ptr = cp1.Y.limb;

	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 8), cp1.Z.size);
	sequence[8].ptr = cp1.Z.limb;

	/* conversion */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 3));
	sequence[9].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 4));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 12),
			PKA_LIR(lir_p, 5));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 6));
	sequence[12].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 7));
	sequence[13].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
			PKA_LIR(lir_p, 8));
	sequence[14].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	/* read back */
	sequence[15].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 10),
			curve->p.size);

	sequence[16].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 11),
			curve->p.size);

	sequence[17].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 12),
			curve->p.size);

	sequence[18].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 3),
			curve->p.size);

	sequence[19].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 4),
			curve->p.size);

	sequence[20].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 5),
			curve->p.size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(21, sequence);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp0.X.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp0.Y.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp0.Z.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp1.X.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp1.Y.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp1.Z.limb);

	cp0.X.size = curve->p.size;
	cp0.Y.size = curve->p.size;
	cp0.Z.size = curve->p.size;
	cp1.X.size = curve->p.size;
	cp1.Y.size = curve->p.size;
	cp1.Z.size = curve->p.size;

	q_pka_hw_rst();
#endif

	/* read result back */
	if (q_ecp_pt_add_prj(ctx, r, &cp1, &cp0, curve, add_tmp))
		goto Q_ECP_PT_ADD_EXIT;
	q_ecp_prj_2_affine(ctx, r, r, curve);

#ifdef QLIP_MOD_USE_MONT
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#endif

#ifdef QLIP_USE_PKA_HW
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#endif

	for (i = 5; i >= 0; i--)
		q_free(ctx, &add_tmp[i]);
	q_free(ctx, &cp1.Z);
	q_free(ctx, &cp1.Y);
	q_free(ctx, &cp1.X);

	q_free(ctx, &cp0.Z);
	q_free(ctx, &cp0.Y);
	q_free(ctx, &cp0.X);

Q_ECP_PT_ADD_EXIT:
	ctx->status = status;
	return ctx->status;
}

/*
 * q_ecp_pt_sub ()
 * Description: This function computes r = p0 - p1 = p0 + (-p1)
 * @param ctx		QLIP context pointer
 * @param r		Result EC point pointer
 * @param p0		First EC point pointer
 * @param p1		Second EC point pointer
 * @param curve	EC curve parameter pointer
 */
int32_t q_ecp_pt_sub(struct q_lip_ctx *ctx,
						struct q_point *r,
						struct q_point *p0,
						struct q_point *p1,
						struct q_curve *curve)
{

	int32_t status = Q_SUCCESS;
	struct q_mont mp;
	struct q_point cp0, cp1;
	struct q_lint add_tmp[6];
	uint32_t lir_p;
	int i;

#ifdef QLIP_USE_GMP
	mpz_t ma, mn;
	mpz_init(ma);
	mpz_init(mn);
#endif

#ifdef QLIP_USE_PKA_HW
	struct opcode sequence[22];
#endif

	/*
	 * testing Affine to make sure that the function does
	 * what it is supposed to do.This isn't completely necessary.
	 */
	if (!q_pt_is_affine(p0)) {
		status = Q_ERR_EC_PT_NOT_AFFINE;
		goto Q_ECP_PT_SUB_EXIT;
	}

	if (!q_pt_is_affine(p1)) {
		status = Q_ERR_EC_PT_NOT_AFFINE;
		goto Q_ECP_PT_SUB_EXIT;
	}
	/* make copy to the input */
	status = q_init(ctx, &cp0.X, p0->X.alloc);
	status += q_init(ctx, &cp0.Y, p0->Y.alloc);
	status += q_init(ctx, &cp0.Z, p0->Z.alloc);
	status += q_init(ctx, &cp1.X, p1->X.alloc);
	status += q_init(ctx, &cp1.Y, p1->Y.alloc);
	status += q_init(ctx, &cp1.Z, p1->Z.alloc);
	if (status != Q_SUCCESS)
		goto Q_ECP_PT_SUB_EXIT;

	q_pt_copy(&cp0, p0);
	q_pt_copy(&cp1, p1);

	for (i = 0; i < 6; i++)
		q_init(ctx, &add_tmp[i], curve->p.alloc + 1);

#ifdef QLIP_USE_GMP
	mpz_import(ma, cp1.Y.size, -1, 4, 0, 0, cp1.Y.limb);
	mpz_import(mn, curve->p.size, -1, 4, 0, 0, curve->p.limb);
	mpz_sub(ma, mn, ma);
	mpz_export(cp1.Y.limb, &(cp1.Y.size), -1, 4, 0, 0, ma);

	mpz_clear(ma);
	mpz_clear(mn);
#else
#ifdef QLIP_USE_PKA_HW
	for (i = 0; i < 22; i++)
		sequence[i].ptr = NULL;

	q_init(ctx, &mp.n, curve->p.alloc);
	q_init(ctx, &mp.np, curve->p.alloc + 1);
	q_init(ctx, &mp.rr, curve->p.alloc + 1);

	/* initialize hardware first */
	q_pka_hw_rst();

	q_mont_init(ctx, &mp, &curve->p);
	curve->mp = &mp;

	lir_p = q_pka_sel_lir(curve->p.size);

	sequence[0].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 0),
			curve->mp->n.size);
	sequence[0].ptr = curve->mp->n.limb;

	sequence[1].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 1),
			curve->mp->np.size);
	sequence[1].ptr = curve->mp->np.limb;

	sequence[2].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 2),
			curve->mp->rr.size);
	sequence[2].ptr = curve->mp->rr.limb;

	sequence[3].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 3), cp0.X.size);
	sequence[3].ptr = cp0.X.limb;

	sequence[4].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 4), cp0.Y.size);
	sequence[4].ptr = cp0.Y.limb;

	sequence[5].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 5), cp0.Z.size);
	sequence[5].ptr = cp0.Z.limb;

	sequence[6].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 6), cp1.X.size);
	sequence[6].ptr = cp1.X.limb;

	sequence[7].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 7), cp1.Y.size);
	sequence[7].ptr = cp1.Y.limb;

	sequence[8].op1 =
		PACK_OP1(0, PKA_OP_MTLIRI, PKA_LIR(lir_p, 8), cp1.Z.size);
	sequence[8].ptr = cp1.Z.limb;

	/* conversion */
	sequence[9].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 10),
			PKA_LIR(lir_p, 3));
	sequence[9].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[10].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 11),
			PKA_LIR(lir_p, 4));
	sequence[10].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[11].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 12),
			PKA_LIR(lir_p, 5));
	sequence[11].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[12].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 3),
			PKA_LIR(lir_p, 6));
	sequence[12].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[13].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 4),
			PKA_LIR(lir_p, 7));
	sequence[13].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[14].op1 =
		PACK_OP1(0, PKA_OP_MODMUL, PKA_LIR(lir_p, 5),
			PKA_LIR(lir_p, 8));
	sequence[14].op2 = PACK_OP2(PKA_LIR(lir_p, 2), PKA_LIR(lir_p, 0));

	sequence[15].op1 =
		PACK_OP1(0, PKA_OP_LSUB, PKA_LIR(lir_p, 6),
		PKA_LIR(lir_p, 0));
	sequence[15].op2 = PACK_OP2(PKA_LIR(lir_p, 4), PKA_NULL);

	/* read back */
	sequence[16].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 10),
			curve->p.size);

	sequence[17].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 11),
			curve->p.size);

	sequence[18].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 12),
			curve->p.size);

	sequence[19].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 3),
			curve->p.size);

	sequence[20].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 6),
			curve->p.size);

	sequence[21].op1 =
		PACK_OP1(PKA_EOS, PKA_OP_MFLIRI, PKA_LIR(lir_p, 5),
			curve->p.size);

	/* send sequnence */
	while (q_pka_hw_rd_status() & PKA_STAT_BUSY)
		;
	q_pka_hw_write_sequence(22, sequence);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp0.X.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp0.Y.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp0.Z.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp1.X.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp1.Y.limb);

	while (!(q_pka_hw_rd_status() & PKA_STAT_DONE))
		;
	q_pka_hw_wr_status(PKA_CTL_EN | PKA_STAT_DONE);
	q_pka_hw_read_lir(curve->p.size, cp1.Z.limb);

	cp0.X.size = curve->p.size;
	cp0.Y.size = curve->p.size;
	cp0.Z.size = curve->p.size;
	cp1.X.size = curve->p.size;
	cp1.Y.size = curve->p.size;
	cp1.Z.size = curve->p.size;

	q_pka_hw_rst();
#else
#ifdef QLIP_MOD_USE_MONT
	q_init(ctx, &mp.n, curve->p.alloc);
	q_init(ctx, &mp.np, curve->p.alloc + 1);
	q_init(ctx, &mp.rr, curve->p.alloc + 1);

	status = q_mont_init(ctx, &mp, &curve->p);
	curve->mp = &mp;

	status += q_mont_mul(ctx, &cp0.X, &cp0.X, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp0.Y, &cp0.Y, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp0.Z, &cp0.Z, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp1.X, &cp1.X, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp1.Y, &cp1.Y, &mp.rr, &mp);
	status += q_mont_mul(ctx, &cp1.Z, &cp1.Z, &mp.rr, &mp);
	status += q_sub(&cp1.Y, &mp.n, &cp1.Y);
#else
	q_sub(&cp1.Y, &curve->p, &cp1.Y);
#endif /*QLIP_MOD_USE_MONT */
#endif /*QLIP_USE_PKA_HW */
#endif /*QLIP_USE_GMP */

	/* read result back */
	if (q_ecp_pt_add_prj(ctx, r, &cp1, &cp0, curve, add_tmp))
		goto Q_ECP_PT_SUB_EXIT;
	q_ecp_prj_2_affine(ctx, r, r, curve);

#ifdef QLIP_MOD_USE_MONT
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#endif

#ifdef QLIP_USE_PKA_HW
	q_free(ctx, &mp.rr);
	q_free(ctx, &mp.np);
	q_free(ctx, &mp.n);
#endif

	for (i = 5; i >= 0; i--)
		q_free(ctx, &add_tmp[i]);
	q_free(ctx, &cp1.Z);
	q_free(ctx, &cp1.Y);
	q_free(ctx, &cp1.X);

	q_free(ctx, &cp0.Z);
	q_free(ctx, &cp0.Y);
	q_free(ctx, &cp0.X);

Q_ECP_PT_SUB_EXIT:
	ctx->status = status;
	return ctx->status;
}
