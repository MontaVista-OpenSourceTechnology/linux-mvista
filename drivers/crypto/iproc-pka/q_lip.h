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

#ifndef _QLIP_H_
#define _QLIP_H_

#ifndef NULL
#define NULL 0
#endif
#define MACHINE_WD            32
#define BITS_FOR_MACHINE_WD   5
#define UINT_MASK             0xffffffffL
#define EEA_INV               1
#define MIN_CTX_SIZE          512

#define QLIP_MOD_USE_PB

#define QLIP_MODEXP_LR_BIN

/* QLIP status */
#define Q_SUCCESS                            0
#define Q_ERR_CTX_MEM_SIZE_LOW              -1
#define Q_ERR_CTX_ERR                       -2
#define Q_ERR_CTX_OVERFLOW                  -3
#define Q_ERR_MEM_DEALLOC                   -4
#define Q_ERR_DST_OVERFLOW                  -5
#define Q_ERR_DIV_BY_0                      -6
#define Q_ERR_NO_MODINV                     -7

#define Q_ERR_EC_PT_NOT_AFFINE              -10
#define Q_ERR_PKA_HW_ERR                    -100
#define Q_ERR_PPSEL_FAILED                  -200

#define Q_ERR_GENERIC                       -1000

#define QLIP_USE_PKA_HW

#include <linux/types.h>

/* data structure of the long integer */
struct q_lint {
	uint32_t *limb;	/* pointer to long integer value*/
	int size;	/* the size of the long integer in words*/
	int alloc;	/* the memory allocated for the long integer*/
	int neg;	/* sign flag of the long integer*/
};

/* data structure of the Montgomery fields */
struct q_mont {
	struct q_lint n;
	struct q_lint np;
	struct q_lint rr;
	int br;
};
#ifdef QLIP_USE_COUNTER_SPADPA
struct lfsr_s {
	uint32_t l[5];
};
#endif

/* data structure of the QLIP context */
struct q_lip_ctx {
	int32_t status;	/* QLIP status*/
	int32_t(*q_yield) (void);/* QLIP yield function pointer*/
	/* QLIP context data memory size in words*/
	uint32_t cur_mem_lmt;
	/* Pointer that points to the next unused QLIP context address*/
	uint32_t cur_mem_ptr;
	/* Pointer to the start of QLIP context data memory*/
	uint32_t *ctx_mem;
#ifdef QLIP_USE_COUNTER_SPADPA
	uint32_t ESCAPE;/* Flag to configure SPA/DPA counter measure */
	struct lfsr_s lfsr;
	void (*q_get_random) (struct lfsr_s *lfsr, int32_t length,
					uint32_t *rnd);
#endif
};
/* data structure of the DSA or EC-DSA signature*/
struct q_signature {
	struct q_lint r;
	struct q_lint s;
};

#ifndef HOST_ENDIAN
static const uint32_t endian_test = (1L << 25) - 1;
#define HOST_ENDIAN    (*(signed char *) &endian_test)
#endif

#ifndef BSWAP
#define BSWAP(value) ((value & 0xff000000) >> 24 | (value & 0x00ff0000) >> 8 | \
			(value & 0x0000ff00) << 8 | (value & 0x000000ff) << 24)
#endif

#ifndef Q_NORMALIZE
#define Q_NORMALIZE(DST, LIMBS)\
do {                                              \
	while (LIMBS > 0) {                           \
		if ((DST)[(LIMBS) - 1] != 0)              \
			break;                                \
		LIMBS--;                                  \
	}                                             \
} while (0)
#endif

/*
 * 32-bit addition with carry, may be mapped to a single instruction
 * @param al source operand
 * @param bl source operand
 * @param carry carry input
 */
inline uint32_t q_asm_addc(uint32_t al, uint32_t bl, uint32_t *carry);

/*
 * unsigned long integer add z = a + b
 * @param z result
 * @param a source
 * @param b source
 */
int32_t q_uadd(struct q_lint *z, struct q_lint *a, struct q_lint *b);

/*
 * signed long integer add z = a + b
 * @param z result
 * @param a source
 * @param b source
 */
int32_t q_add(struct q_lint *z, struct q_lint *a, struct q_lint *b);

/*
 * unsigned long integer subtract z = a - b
 * @param z result
 * @param a source
 * @param b source
 */
int32_t q_usub(struct q_lint *z, struct q_lint *a, struct q_lint *b);

/*
 * signed long integer subtract z = a - b
 * @param z result
 * @param a source
 * @param b source
 */
int32_t q_sub(struct q_lint *z, struct q_lint *a, struct q_lint *b);

/*
 * unsigned long integer comparison (a>b) return 1;
 * (a<b) return -1; (a==b) return 0;
 * @param a source
 * @param b source
 */
int32_t q_ucmp(struct q_lint *a, struct q_lint *b);

/*
 * signed long integer comparison (a>b) return 1;
 * (a<b) return -1; (a==b) return 0;
 * @param a source
 * @param b source
 */
int32_t q_cmp(struct q_lint *a, struct q_lint *b);

/*
 * long integer modulo add z = (a + b) mod n
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param b source
 * @param n modulus
 */
int32_t q_modadd(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *b,
					struct q_lint *n);
/*
 * long integer modulo subtract z = (a - b) mod n
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param b source
 * @param n modulus
 */
int32_t q_modsub(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *b,
					struct q_lint *n);

/*
 * long integer modulo remainder z = a mod n
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param n modulus
 */
int32_t q_mod(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *n);
/*
 * long integer modulo remainder z = a mod n
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param n modulus
 */
int32_t q_mod_sw(struct q_lip_ctx *ctx, struct q_lint *z,
				struct q_lint *a, struct q_lint *n);
/*
 * long integer modulo remainder z = a mod n when the size of a
 * is no bigger than the size of n
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param n modulus
 */
int32_t q_mod_partial(struct q_lint *z, struct q_lint *a,
				struct q_lint *n);
/*
 * long integer modulo remainder z = a mod n when n is power-of-2
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param bits the number of bits of the remainder
 */
int32_t q_mod_2pn(struct q_lint *z, struct q_lint *a, uint32_t bits);

/*
 * long integer multiply z = a * b
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param b source
 */
int32_t q_mul(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *b);
/*
 * long integer multiply z = a * b
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param b source
 */
int32_t q_mul_sw(struct q_lip_ctx *ctx, struct q_lint *z,
				struct q_lint *a, struct q_lint *b);
/*
 * long integer square z = a^2
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 */
int32_t q_sqr(struct q_lip_ctx *ctx, struct q_lint *z,
			struct q_lint *a);

/*
 * long integer modulo multiply z = a * b mod n
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param b source
 * @param n modulus
 */
int32_t q_modmul(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *b,
					struct q_lint *n);
/*
 * long integer modulo square z = a^2 mod n
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param n modulus
 */
int32_t q_modsqr(struct q_lip_ctx *ctx, struct q_lint *z,
				struct q_lint *a, struct q_lint *n);
/*
 * long integer modulo multiply z = a * b mod n using Barrett's method
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param b source
 * @param n modulus
 */
int32_t q_modmul_pb(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *r, struct q_lint *a,
					struct q_lint *b, struct q_lint *n);

/*
 * long integer modulo square z = a^2 mod n using Barret's method
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param n modulus
 */
int32_t q_modsqr_pb(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *r, struct q_lint *a,
					struct q_lint *n);

/*
 * long integer multiply when the second operand is a power-of-2,
 * effectively a left shift function
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param bits second operand in number of bits
 */
int32_t q_mul_2pn(struct q_lint *z, struct q_lint *a, uint32_t bits);

void get_digit(int *opa, int index, int *value);
void set_digit(int *opa, int index, int value);

/*
 * long integer divide and remainder routine
 * opa = opn * quotient + remainder
 * @param ctx QLIP context pointer
 * @param opa dividend
 * @param opn divider
 * @param quotient
 * @param remainder
 */
int32_t q_fdivrem(struct q_lip_ctx *ctx, struct q_lint *opa,
				struct q_lint *opn, struct q_lint *quotient,
				struct q_lint *remainder);

/*
 * long integer divide z = a / b
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param b source
 */
int32_t q_div(struct q_lip_ctx *ctx, struct q_lint *z,
			struct q_lint *a, struct q_lint *b);
/*
 * long integer divide when the second operand is power-of-2,
 * effective a right shift function
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param bits second operand in number of bits
 */
int32_t q_div_2pn(struct q_lint *z, struct q_lint *a, uint32_t bits);

/*
 * long integer modulo divide by 2, z = a/2 mod n
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param n modulus
 */
int32_t q_mod_div2(struct q_lip_ctx *ctx, struct q_lint *z,
					 struct q_lint *a, struct q_lint *n);
/*
 * long integer modulo exponentiation , z = a^e mod n
 * @param ctx QLIP context pointer
 * @param z result
 * @param a base
 * @param e exponent
 * @param n modulus
 */
int32_t q_modexp(struct q_lip_ctx *ctx, struct q_lint *z,
				struct q_lint *a, struct q_lint *e,
					struct q_lint *n);
/*
 * Montgomery field initialization function
 * @param ctx QLIP context pointer
 * @param n modulus
 * @param mont Montgomery fields
 */
int32_t q_mont_init(struct q_lip_ctx *ctx, struct q_mont *mont,
					struct q_lint *n);
/*
 * long integer modulo mulitply,
 * z = a * b mod n using Montgomery's method
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param b source
 * @param mont Montgomery fields
 */
int32_t q_mont_mul(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *b,
					struct q_mont *mont);
/*
 * long integer modulo exponentiation ,
 * z = a^e mod n using Montgomery's method
 * @param ctx QLIP context pointer
 * @param z result
 * @param a base
 * @param e exponent
 * @param mont Montgomery fields
 */
int32_t q_modexp_mont(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *e,
					struct q_mont *mont);
/*
 * greatest common divisor function
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param b source
 */
int32_t q_gcd(struct q_lip_ctx *ctx, struct q_lint *z,
				struct q_lint *a, struct q_lint *b);
/*
 * Extended Euclid Function d = a * x + b * y return x
 * @param ctx QLIP context pointer
 * @param x result
 * @param lx result
 * @param a source
 * @param b source
 */
int32_t q_euclid(struct q_lip_ctx *ctx, struct q_lint *x,
				struct q_lint *lx, struct q_lint *a,
				struct q_lint *b);
/*
 * Extended Euclid Function d = a * x + b * y return x
 * @param ctx QLIP context pointer
 * @param x result
 * @param lx result
 * @param a source
 * @param b source
 */
int32_t q_euclid_sw(struct q_lip_ctx *ctx, struct q_lint *x,
					struct q_lint *lx, struct q_lint *a,
					struct q_lint *b);
/*
 * long integer modulo inverse,
 * z = a ^ (-1) mod n using Extended Euclid Algorithm
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param n modulus
 */
int32_t q_modinv(struct q_lip_ctx *ctx, struct q_lint *z,
				struct q_lint *a, struct q_lint *n);
/*
 * long integer modulo inverse,
 * z = a ^ (-1) mod n using Extended Euclid Algorithm
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param n modulus
 */
int32_t q_modinv_sw(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *n);
/*
 * long integer modulo inverse,
 * z = a ^ (-1) mod p when modulus is a prime number
 * @note using this separate function for prime modulus
 * to avoid loop in EEA
 * @param ctx QLIP context pointer
 * @param z result
 * @param a source
 * @param p prime modulus
 */
int32_t q_modpinv(struct q_lip_ctx *ctx, struct q_lint *z,
					struct q_lint *a, struct q_lint *p);
#endif /*_QLIP_H_*/
