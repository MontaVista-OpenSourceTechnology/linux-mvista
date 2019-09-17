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

#ifdef QLIP_USE_PKA_HW
#include "q_pka_hw.h"
#endif

/*
 * q_ctx_init()
 * QLIP context initialization routine
 * Description: This routine resets hardware if applicable, and then
 * initializes the QLIP context data structure.  The context data
 * structure contains the following information: context memory base
 * pointer and size, yield function pointer, and current unused memory
 * pointer.
 * @param ctx	QLIP context pointer
 * @param ctx_data_mem_ptr	QLIP data memory pointer
 * @param ctx_data_mem_size	QLIP data memory size (in words)
 * @param q_yield	QLIP yield function pointer
 */
int32_t q_ctx_init(struct q_lip_ctx *ctx,
					  uint32_t *ctx_data_mem_ptr,
					  uint32_t ctx_data_mem_size,
					  int32_t(*q_yield) (void))
{
	/* Initialize QLIP context memory upper bound */
	ctx->cur_mem_lmt = (uint32_t) ctx_data_mem_size;

	/* Initialize pointer to the start of the context data memory */
	ctx->ctx_mem = (uint32_t *) ctx_data_mem_ptr;
	ctx->cur_mem_ptr = 0;

	/* Initialize funtion pointer for yeild function */
	if (q_yield == NULL)
		ctx->q_yield = q_yield_default;
	else
		ctx->q_yield = q_yield;

	/* Check context memory size */
	if (ctx_data_mem_size < MIN_CTX_SIZE)
		ctx->status = Q_ERR_CTX_MEM_SIZE_LOW;
	else
		ctx->status = Q_SUCCESS;

	/* add this one to disable SPA/DPA by default */
#ifdef QLIP_USE_COUNTER_SPADPA
	int i;
	ctx->ESCAPE = 0;

	for (i = 0; i < 4; i++)
		ctx->lfsr.l[i] = 0x5a5a5a5a;

	ctx->lfsr.l[4] = 0x1;
#endif
	/* reset hardware */
#ifdef QLIP_USE_PKA_HW
	q_pka_hw_rst();
#endif

	return ctx->status;
}
EXPORT_SYMBOL(q_ctx_init);

/*
 * q_yield_default()
 * QLIP default yield function.
 * Description: This function is called when QLIP functions waits for
 * the hardware to complete a sequence of operations.  This default
 * yield function is only used when the application does not provide
 * its own yield function.
 */
int32_t q_yield_default()
{
	/*
	 * This function is called when the QLIP is waiting for
	 * HW to finish a task. As part of the QLIP, this function
	 * simply returns 0.
	 */

	return Q_SUCCESS;
}

/*
 * q_import ()
 * Description: This function imports data from an array into a q_line
 * structure.  Both word order (BIG_NUM or normal) and endianness can
 * be specified.
 * @param ctx	QLIP context pointer
 * @param z	destination q_lint pointer
 * @param size	data size (in word)
 * @param order	export order: 1 = BIG_NUM, -1 = normal
 * @param endian	endianness: 0 = native, 1 = big, -1 = little
 * @param data		source data pointer
 */
int32_t q_import(struct q_lip_ctx *ctx,  struct q_lint *z, uint32_t size,
					int order, int endian, const void *data)
{
	uint32_t *zp;

	z->alloc = size;
	z->limb = (uint32_t *) q_malloc(ctx, size);
	if (!z->limb) {
		q_abort(__FILE__, __LINE__,
				"QLIP: Insufficient memory or zero memory allocation!");

		ctx->status = Q_ERR_CTX_OVERFLOW;
		goto Q_IMPORT_EXIT;
	}

	z->size = size;
	z->neg = 0;

	if (endian == 0)
		endian = HOST_ENDIAN;

	zp = z->limb;

	if ((order == 1) && (endian == HOST_ENDIAN))
		QLIP_COPY(zp, (uint32_t *) data, size);

	if ((order == 1) && (endian == -HOST_ENDIAN))
		QLIP_COPY_BSWAP(zp, (uint32_t *) data, size);

	if ((order == -1) && (endian == HOST_ENDIAN))
		QLIP_RCOPY(zp, (uint32_t *) data, size);

	if ((order == -1) && (endian == -HOST_ENDIAN))
		QLIP_RCOPY_BSWAP(zp, (uint32_t *) data, size);

	Q_NORMALIZE(zp, (z->size));

Q_IMPORT_EXIT:
	return ctx->status;
}
EXPORT_SYMBOL(q_import);

/*
 * q_export ()
 * Description: This function exports data from a q_lint structure to
 * an array.  Both word order (BIG_NUM or normal) and endianness can
 * be specified.
 * @param data	destination data array pointer
 * @param size	variable pointer of number of words exported
 * @param order	export order: 1 = BIG_NUM, -1 = normal
 * @param endian	endianness: 0 = native, 1 = big, -1 = little
 * @param a		pointer to source q_lint
 */
int32_t q_export(void *data, int *size, int order, int endian, struct q_lint *a)
{
	uint32_t *ap;

	ap = a->limb;
	*(size) = a->neg ? a->size * -1 : a->size;

	if (endian == 0)
		endian = HOST_ENDIAN;

	if ((order == 1) && (endian == HOST_ENDIAN))
		QLIP_COPY((uint32_t *) data, ap, (a->size));

	if ((order == 1) && (endian == -HOST_ENDIAN))
		QLIP_COPY_BSWAP((uint32_t *) data, ap, (a->size));

	if ((order == -1) && (endian == HOST_ENDIAN))
		QLIP_RCOPY((uint32_t *) data, ap, (a->size));

	if ((order == -1) && (endian == -HOST_ENDIAN))
		QLIP_RCOPY_BSWAP((uint32_t *) data, ap, (a->size));

	return Q_SUCCESS;
}

/*
 * q_print ()
 * Description: print utility for debug purpose.
 * Embedded system can use this function to redirect debug data to
 * special debug console.
 */
void q_print(const char *name, struct q_lint *z)
{
	uint32_t *zp;
	int i;

	zp = z->limb;
	if (z->neg)
		pr_info("%s = -0x", name);
	else
		pr_info("%s = 0x", name);

	if (!z->size)
		pr_info("0");

	/* print in normal format */
	for (i = z->size - 1; i >= 0; i--)
		pr_info("%08X", zp[i]);
	pr_info("\n");
}

/*
 * q_abort ()
 * Description: Default abort handler.
 */
void q_abort(const char *filename, const int line, const char *str)
{
	pr_err("ERROR at line %d in file %s.\n\t-->%s\n", line,
		   filename, str);
}

#ifdef QLIP_USE_COUNTER_SPADPA
/*
 * q_cfg_counter_spadpa ()
 * Description:
 * This function enables/disables SPA/DPA counter measure.
 * When PKA hardware is used, the configuration is written
 * into PKA Control and Status Register ESCAPE field bit
 * range [18:16].
 * The ESCAPE field is encoded as the following:
 * bit [16]    - (ESCAPE & 0x1): enabled modexp protection
 * bit [18:17] - (ESCAPE & 0x6): enabled random pipeline stall
 * 0x1: high stall, 0x2: medium stall; 0x3: low stall
 * This function also seeds the LFSR and install the
 * get_random function.
 * @param ctx	QLIP context pointer
 * @param ESCAPE flags to configure SPA/DPA counter measure settings
 * @param seed		seed valud to LFSR
 * @param q_get_random	get_random function pointer
 */
int32_t q_cfg_counter_spadpa(struct q_lip_ctx *ctx,
				uint32_t ESCAPE, uint32_t seed,
				void (*q_get_random)(struct lfsr_s *lfsr,
								int32_t length,
								uint32_t *rnd))
{
#ifdef QLIP_USE_PKA_HW
	uint32_t pka_status;
#endif

	ctx->ESCAPE = ESCAPE;

	if (seed != 0)
		q_seed_lfsr(&ctx->lfsr, seed);

	q_shift_lfsr(&ctx->lfsr, 253);

	if (q_get_random == NULL)
		ctx->q_get_random = q_get_random_default;
	else
		ctx->q_get_random = q_get_random;

#ifdef QLIP_USE_PKA_HW
	pka_status = q_pka_hw_rd_status();
	pka_status |= ((ESCAPE & PKA_CTL_ESCAPE_MASK) << PKA_CTL_ESCAPE_LOC);
	q_pka_hw_wr_status(pka_status);
#endif

	return Q_SUCCESS;
}

/*
 * q_seed_lfsr ()
 * Description: This function seeds the LFSR.
 */
void q_seed_lfsr(struct lfsr_s *lfsr, uint32_t seed)
{
	/*seed the MSW */
	if (lfsr->l[1] != seed)
		lfsr->l[1] ^= seed;
	if (lfsr->l[1] != seed)
		lfsr->l[3] ^= seed;
}

/*
 * q_shift_lfsr ()
 * Description: This function shifts the LFSR by count.
 */
void q_shift_lfsr(struct lfsr_s *lfsr, int32_t count)
{
	int i;
	uint32_t t;

	/*
	 * a 64-bit LFSR and a 65-bit LFSR polynomial
	 * lfsr0: x(64) + x(4) + x(3) + x + 1
	 * lfsr1: x(65) + x(18)+ 1
	 */
	for (i = 0; i < count; i++) {
		t = ((lfsr->l[1] & 0x80000000) >> 31) ^
			((lfsr->l[1] & 0x20000000) >> 29) ^
			((lfsr->l[1] & 0x10000000) >> 28) ^ (lfsr->l[0] & 0x1);
		lfsr->l[0] = (lfsr->l[0] >> 1) | ((lfsr->l[1] & 0x1) << 31);
		lfsr->l[1] = (lfsr->l[1] >> 1) | (t << 31);

		t = ((lfsr->l[3] & 0x00008000) >> 15) ^ (lfsr->l[2] & 0x1);
		lfsr->l[2] = (lfsr->l[2] >> 1) | ((lfsr->l[3] & 0x1) << 31);
		lfsr->l[3] = (lfsr->l[3] >> 1) | ((lfsr->l[4] & 0x1) << 31);
		lfsr->l[4] = t;
	}
}

/*
 * q_get_random_default ()
 * Description: This function is the default function to generate pseudo
 * random numbers from the LFSR.
 */
void q_get_random_default(struct lfsr_s *lfsr, int32_t length, uint32_t *rnd)
{
	int i;

	for (i = 0; i < length; i++) {
		/* take random data from LFSR */
		rnd[i] = lfsr->l[0 + i % 4];

		/*shift LFSR */
		if ((i % 4) == 0)
			q_shift_lfsr(lfsr, 7);
	}
}

#endif
