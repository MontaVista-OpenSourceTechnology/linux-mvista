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

#ifndef _Q_RSA_4K_H_
#define _Q_RSA_4K_H_

#include "q_rsa.h"

/*
 * long integer modulo exponentiation , z = a^e mod n
 * param ctx QLIP context pointer
 * param z result
 * param a base
 * param e exponent
 * param n modulus
 */
int32_t q_modexp_sw(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *e,
					struct q_lint *n);

/*
 * Montgomery field initialization function
 * param ctx QLIP context pointer
 * param n modulus
 * param mont Montgomery fields
 */
int32_t q_mont_init_sw(struct q_lip_ctx *ctx, struct q_mont *mont,
					struct q_lint *n);

/*
 * long integer modulo mulitply,
 * z = a * b mod n using Montgomery's method
 * param ctx QLIP context pointer
 * param z result
 * param a source
 * param b source
 * param mont Montgomery fields
 */
int32_t q_mont_mul_sw(struct q_lip_ctx *ctx, struct q_lint *z,
					  struct q_lint *a, struct q_lint *b,
					  struct q_mont *mont);

/*
 * long integer modulo exponentiation ,
 * z = a^e mod n using Montgomery's method
 * param ctx QLIP context pointer
 * param z result
 * param a base
 * param e exponent
 * param mont Montgomery fields
 */
int32_t q_modexp_mont_sw(struct q_lip_ctx *ctx, struct q_lint *z,
						struct q_lint *a,
						struct q_lint *e,
						struct q_mont *mont);

/*
 * greatest common divisor function
 * param ctx QLIP context pointer
 * param z result
 * param a source
 * param b source
 */
int32_t q_gcd_sw(struct q_lip_ctx *ctx, struct q_lint *z,
				 struct q_lint *a, struct q_lint *b);

/*
 * Extended Euclid Function d = a * x + b * y return x
 * param ctx QLIP context pointer
 * param x result
 * param lx result
 * param a source
 * param b source
 */
int32_t q_euclid_sw(struct q_lip_ctx *ctx, struct q_lint *x,
				   struct q_lint *lx, struct q_lint *a,
				   struct q_lint *b);

/*
 * RSA CRT decryption function
 * param ctx QLIP context pointer
 * param m clear text
 * param rsa RSA CRT private key structure
 * param c cipher text
 */
int32_t q_rsa_crt_4k(struct q_lip_ctx *ctx, struct q_lint *m,
					 struct q_rsa_crt_key *rsa,
					 struct q_lint *c);
#endif /*_Q_RSA_4K_H_*/
