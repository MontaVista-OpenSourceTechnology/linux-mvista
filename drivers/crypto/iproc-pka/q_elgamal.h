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

#ifndef _Q_ELGAMAL_H_
#define _Q_ELGAMAL_H_

#define SIG_VERIFIED 1
#define SIG_NOT_VERIFIED -1

/*
 * function to map a long integer to a point on the elliptic curve
 * M.x = m, (M.y)^2 = (M.x)^3 + a*(M.x) + b
 * M.y is computed using modulo square root function.
 * This implementation assumes
 * the prime p is congruent to 3 modulo 4.
 * param ctx The QLIP context
 * param M the mapped point
 * param m the long integer
 * param curve the elliptic curve data structure
 */
int32_t q_ecp_lint_2_point(struct q_lip_ctx *ctx, struct q_point *M,
						  struct q_lint *m,
						  struct q_curve *curve);

/*
 * Prime field ECC El Gamal encryption
 * param ctx The QLIP context
 * param C1 the point representing the ephemeral public value
 * param C2 the point representing the encrypted cipher text
 * param M the point representing the clear text
 * param k the random generated the sender
 * param G the base point
 * param Q the public key of the receiver
 * param curve the elliptic curve data structure
 */
int32_t q_ecp_elgamal_enc(struct q_lip_ctx *ctx, struct q_point *C1,
					 struct q_point *C2, struct q_point *M,
					 struct q_lint *k, struct q_point *G,
					 struct q_point *Q,
					 struct q_curve *curve);

/*
 * Prime field ECC El Gamal decryption
 * param ctx The QLIP context
 * param M the point representing the decrypted clear text
 * param C1 the point representing the ephemeral public value
 * param C2 the point representing the cipher text
 * param d the private key of the receiver
 * param curve the elliptic curve data structure
 */
int32_t q_ecp_elgamal_dec(struct q_lip_ctx *ctx, struct q_point *M,
						 struct q_point *C1,
						 struct q_point *C2,
						 struct q_lint *d,
						 struct q_curve *curve);
#endif
