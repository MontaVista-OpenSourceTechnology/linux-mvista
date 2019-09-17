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

#ifndef _Q_ELLIPTIC_H_
#define _Q_ELLIPTIC_H_

#define QLIP_PKA_ECP_DBL_SEQ_LEN 19
#define QLIP_PKA_ECP_ADD_SEQ_LEN 25
#define QLIP_PKA_ECP_SUB_SEQ_LEN 27

/* data structure of a point on the elliptic curve */
struct q_point {
	struct q_lint X;
	struct q_lint Y;
	struct q_lint Z; /* Z=1 if Affine */
};

/* data structure of an elliptic curve */
struct q_curve {
	uint32_t type;		/* 0: prime field, 1: binary field */
	struct q_lint n;	/* the order or the curve */
	struct q_lint a;	/* parameter a */
	struct q_lint b;	/* parameter b */
	struct q_lint p;	/* prime p */
	struct q_mont *mn;	/* Montgomery context for curve order n */
	struct q_mont *mp;	/* Montgomery context for prime p */
};

/* copy a point to another point */
int32_t q_pt_copy(struct q_point *r, struct q_point *p);

/* determine if the coordinates of a point is in Affine format */
int q_pt_is_affine(struct q_point *p);

/*
 * Jacobian coordinates to Affine coordinates conversion.
 * param ctx The QLIP context
 * param r the converted Affine point
 * param p the Jacobian point
 * param curve the Elliptic curve data structure
 */
int32_t q_ecp_prj_2_affine(struct q_lip_ctx *ctx,
				  struct q_point *r, struct q_point *p,
				  struct q_curve *curve);

/*
 * Affine prime field point multiplication, r(x,y) = p(x,y) * k
 * param ctx The QLIP context
 * param r the result point
 * param p the source point
 * param k the multiplier value
 * param curve the Elliptic curve data structure
 */
int32_t q_ecp_pt_mul(struct q_lip_ctx *ctx,
			struct q_point *r, struct q_point *p,
			struct q_lint *k, struct q_curve *curve);
/*
 * Jacobian prime field point multiplication with precomputed
 * negative and pre-allocated temporary storage for performance.
 * note this function is intended to be called multiple times
 * by a higher level function such as ECDSA functions.
 * param ctx The QLIP context
 * param r the result point
 * param p the source point
 * param p the negative source point
 * param k the multiplier value
 * param curve the Elliptic curve data structure
 * param tmp temporary storage allocated at higher level
 */
int32_t q_ecp_pt_mul_prj(struct q_lip_ctx *ctx,
				struct q_point *r, struct q_point *p,
				struct q_point *p_minus, struct q_lint *k,
				struct q_curve *curve, struct q_lint *tmp);

/*
 * Affine prime field point doubling, r(x,y) = p(x,y) + p(x,y).
 * note this is a wrapper function of q_ecp_pt_dbl_prj
 * param ctx The QLIP context
 * param r the result point
 * param p the source point
 * param curve the Elliptic curve data structure
 */
int32_t q_ecp_pt_dbl(struct q_lip_ctx *ctx,
			struct q_point *r, struct q_point *p,
			struct q_curve *curve);

/*
 * Affine prime field point addition, r(x,y) = p0(x,y) + p1(x,y)
 * note this is a wrapper function of q_ecp_pt_add_prj
 * param ctx The QLIP context
 * param r the result point
 * param p0 the source point p0
 * param p1 the source point p1
 * param curve the Elliptic curve data structure
 */
int32_t q_ecp_pt_add(struct q_lip_ctx *ctx,
			struct q_point *r, struct q_point *p0,
			struct q_point *p1, struct q_curve *curve);

/*
 * Affine prime field point subtraction, r(x,y) = p0(x,y) - p1(x,y).
 * param ctx The QLIP context
 * param r the result point
 * param p0 the source point p0
 * param p1 the source point p1
 * param curve the Elliptic curve data structure
 */
int32_t q_ecp_pt_sub(struct q_lip_ctx *ctx,
			struct q_point *r, struct q_point *p0,
			struct q_point *p1, struct q_curve *curve);

/*
 * Jacobian prime field point doubling, r(x,y) = p(x,y) + p(x,y).
 * note this function is intended to called multiple times
 * by a higher level function such as q_ecp_pt_mul_prj
 * param ctx The QLIP context
 * param r the result point
 * param p the source point
 * param curve the Elliptic curve data structure
 * param tmp temporary storage allocated at higher level
 */
int32_t q_ecp_pt_dbl_prj(struct q_lip_ctx *ctx,
			struct q_point *r, struct q_point *p,
				struct q_curve *curve, struct q_lint *tmp);

/*
 * Jacobian prime field point addition, r(x,y) = p0(x,y) + p1(x,y).
 * note this function is intended to called multiple times
 * by a higher level function such as q_ecp_pt_mul_prj
 * param ctx The QLIP context
 * param r the result point
 * param p0 the source point p0
 * param p1 the source point p1
 * param curve the Elliptic curve data structure
 * param tmp temporary storage allocated at higher level
 */
int32_t q_ecp_pt_add_prj(struct q_lip_ctx *ctx,
			struct q_point *r, struct q_point *p0,
			struct q_point *p1, struct q_curve *curve,
			struct q_lint *tmp);

/*
 * EC-DSA signature singing function
 * param ctx The QLIP context
 * param rs the computed signature
 * param G the base point
 * param curve the Elliptic curve data structure
 * param d the private key for signing
 * param h the hash result of the message to be signed
 * param k the random nonce
 */
int32_t q_ecp_ecdsa_sign(struct q_lip_ctx *ctx, struct q_signature *rs,
				struct q_point *G, struct q_curve *curve,
				struct q_lint *d, struct q_lint *h,
				struct q_lint *k);

/*
 * EC-DSA signature verification function
 * note  higher level function still has to compare v == s
 * to verify the signature is indeed correct.
 * param ctx The QLIP context
 * param v the computed signature verification value
 * param G the base point
 * param curve the Elliptic curve data structure
 * param Q the public key for signature verification
 * param h the hash result of the message to be signed
 * param rs the signature to be verified
 */
int32_t q_ecp_ecdsa_verify(struct q_lip_ctx *ctx,
			struct q_lint *v, struct q_point *G,
			struct q_curve *curve,
			struct q_point *Q, struct q_lint *h,
			struct q_signature *rs);

#ifdef QLIP_USE_PKA_HW
/*
 * Optimized PKA hardware initialization and data loading function
 * note this function is only used with PKA hardware.
 * It represents a hardware micro-code sequence invoked
 * by higher level functions.
 */
int32_t q_ecp_prj_pka_init(struct q_lip_ctx *ctx,
			struct q_curve *curve, struct q_point *pt0,
			struct q_point *pt1);
/*
 * Optimized PKA hardware point doubling function
 * note this function is only used with PKA hardware.
 * It represents a hardware micro-code sequence invoked
 * by higher level functions.
 */
void q_ecp_pt_dbl_prj_pka(struct opcode *sequence, uint32_t lir_p);

/*
 * Optimized PKA hardware point addition function
 * note this function is only used with PKA hardware.
 * It represents a hardware micro-code sequence invoked
 * by higher level functions.
 */
void q_ecp_pt_add_prj_pka(struct opcode *sequence, uint32_t lir_p);

/*
 * Optimized PKA hardware result unloading function
 * note this function is only used with PKA hardware.
 * It represents a hardware micro-code sequence invoked
 * by higher level functions.
 */
int32_t q_ecp_prj_pka_get_result(struct q_lip_ctx *ctx,
			struct q_point *pt);
#endif

#endif
