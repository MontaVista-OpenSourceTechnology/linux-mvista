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

#ifndef _Q_LIP_UTILS_H_
#define _Q_LIP_UTILS_H_

#if !defined(QLIP_COPY)
#define QLIP_COPY(dst, src, n)                   \
do {                                             \
	if ((n) != 0) {                              \
		uint32_t __n = (n);                      \
		uint32_t *__dst = (dst);                 \
		uint32_t *__src = (src);                 \
		uint32_t __x;                            \
		if (__n != 0) {                          \
			do {                                 \
				__x = *__src++;                  \
				*__dst++ = __x;                  \
			} while (--__n);                     \
		}                                        \
	}                                            \
} while (0)
#endif

#if !defined(QLIP_COPY_BSWAP)
#define QLIP_COPY_BSWAP(dst, src, n)              \
do {                                              \
	if ((n) != 0) {                               \
		uint32_t __n = (n);                       \
		uint32_t *__dst = (dst);                  \
		uint32_t *__src = (src);                  \
		uint32_t __x;                             \
		if (__n != 0) {                           \
			do {                                  \
				__x = *__src++;                   \
				__x = BSWAP(__x);                 \
				*__dst++ = __x;                   \
			} while (--__n);                      \
		}                                         \
	}                                             \
} while (0)
#endif

#define QLIP_RCOPY(dst, src, size)                \
do {                                              \
	uint32_t *__dst = (dst);                   \
	uint32_t  __size = (size);                  \
	uint32_t *__src = (src) + __size - 1;       \
	uint32_t  __i;                              \
	for (__i = 0; __i < __size; __i++) {          \
		*__dst = *__src;                          \
		__dst++;                                  \
		__src--;                                  \
	}                                             \
} while (0)

#define QLIP_RCOPY_BSWAP(dst, src, size)          \
do {                                              \
	uint32_t *__dst = (dst);                      \
	uint32_t  __size = (size);                    \
	uint32_t *__src = (src) + __size - 1;         \
	uint32_t  __i;                                \
	for (__i = 0; __i < __size; __i++) {          \
		*__dst = *__src;                          \
		*__dst = BSWAP(*__dst);                   \
		__dst++;                                  \
		__src--;                                  \
	}                                             \
} while (0)

/*
 * QLIP context initialization
 * param ctx QLIP context pointer provided by higher level software
 * param ctx_data_mem_ptr QLIP context data memory pointer
 * param ctx_data_mem_size QLIP context data memory size (in words)
 * param q_yield function pointer to a yield function
 * provided by higher level software
 * note  QLIP relies on a piece of scratch memory being
 * provided by the calling function.
 */
int32_t q_ctx_init(struct q_lip_ctx *ctx,
				  uint32_t *ctx_data_mem_ptr,
				  uint32_t ctx_data_mem_size,
				  int32_t(*q_yield) (void));
/*
 * The default yield function
 * note:
 * QLIP is blocking. The yield function provides
 * some flexibility to instrument non-blocking behavior
 * by higher level software. When QLIP function enters
 * hardware polling loop, the yield function can be called.
 */
int32_t q_yield_default(void);

/*
 * initialize a long integer and import long integer
 * value from a data array
 * param ctx QLIP context pointer
 * param z long integer to be initialized
 * param size the memory size (in 32-bit words) to be
 * allocated for the long integer
 * param order the long integer can be initialized
 * in NORMAL (1) or BIGNUM order (-1)
 * param endian the Endianness to be applied
 * param pointer to the data array
 * note:
 * long integer must be initialized before it is used.
 * Initialization allocates memory storage in the context.
 */
int32_t q_import(struct q_lip_ctx *ctx, struct q_lint *z,
					uint32_t size, int order,
					int endian, const void *data);

/* export the value of a long integer to a data array
 * param pointer to the data array
 * param size the size of the data
 * param order the long integer can be initialized in
 * NORMAL (1) or BIGNUM order (-1)
 * param endian the Endianness to be applied
 * param a long integer exported
 */
int32_t q_export(void *data, int *size, int order,
				int endian, struct q_lint *a);
/*
 * print the value of a long integer for debug
 * param name name string to be printed
 * param z long integer
 */
void q_print(const char *name, struct q_lint *z);

/*
 * abort routine for error handling and debugging
 * param filename the name of the file that causes the error
 * param line the line number where the error occured
 * param str the error message to be printed
 */
void q_abort(const char *filename, const int line, const char *str);

#ifdef QLIP_USE_COUNTER_SPADPA
/*
 * configure SPA/DPA counter measure
 * param ctx QLIP context pointer provided by higher level software
 * param ESCAPE flags to configure SPA/DPA counter measure settings
 * param seed seed value to LFSR
 * param q_get_random function pointer to a random number
 * generator function provided by higher level software
 */
int32_t q_cfg_counter_spadpa(struct q_lip_ctx *ctx,
				uint32_t ESCAPE,
				uint32_t seed,
				void (*q_get_random) (struct lfsr_s *lfsr,
					int32_t length, uint32_t *rnd));

/*
 * function to facilitate higher level software to seed the LFSR
 * param lfsr pointer to the lfsr used to extract the random data
 * param seed seed value to LFSR
 */
void q_seed_lfsr(struct lfsr_s *lfsr, uint32_t seed);

/*
 * function to shift LFSR
 * param lfsr pointer to the lfsr used to extract the random data
 * param count number of steps to shift
 */
void q_shift_lfsr(struct lfsr_s *lfsr, int32_t count);

/*
 * default function to generate random value from LFSR
 * param lfsr pointer to the lfsr used to extract the random data
 * param length the word length of the random to be generated
 * param rnd the location of the memory buffer to store the
 * generated random
 */
void q_get_random_default(struct lfsr_s *lfsr, int32_t length, uint32_t *rnd);
#endif

#endif /*_Q_LIP_UTILS_H_*/
