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

/*
 * name: q_init ()
 * Description: This function initializes a q_lint structure. It
 * allocates the data memory in the QLIP context, clears the allocated
 * memory, and initializes the parameters.
 * @ctx	- QLIP context pointer
 * @z	- pointer to q_lint to be initialized
 * @size - maximum data word size
 */
int32_t q_init(struct q_lip_ctx *ctx, struct q_lint *z, uint32_t size)
{
	int i;

	z->neg = 0;
	z->size = 0;
	z->alloc = size;
	z->limb = (uint32_t *) q_malloc(ctx, size);

	if (z->limb == NULL) {
		q_abort(__FILE__, __LINE__,
				"QLIP: Insufficient memory or zero memory allocation!");

		ctx->status = Q_ERR_CTX_OVERFLOW;
		goto Q_INIT_EXIT;
	}

	for (i = 0; i < size; i++)
		z->limb[i] = 0L;

Q_INIT_EXIT:
	return ctx->status;
}
EXPORT_SYMBOL(q_init);

/*
 * q_copy ()
 * Description: This function copies the data content from a source
 * q_lint structure to the destination q_lint structure.  The
 * destination q_lint structure does not have to have the same memory
 * capacity, but is expected to have enough memory space to hold the
 * data in the source q_lint structure.
 */
int32_t q_copy(struct q_lint *z, /* destination q_lint pointer */
				  struct q_lint *a)
{
	/* source q_lint pointer */
	int32_t status = Q_SUCCESS;
	int i;
	uint32_t *ap;
	uint32_t *zp;

	if (z->alloc < a->size) {
		q_abort(__FILE__, __LINE__,
				"QLIP: Copy target doesn't have sufficient memory!");

		status = Q_ERR_DST_OVERFLOW;
		goto Q_COPY_EXIT;
	}

	ap = a->limb;
	zp = z->limb;

	for (i = 0; i < a->size; i++)
		zp[i] = ap[i];

	z->size = a->size;
	z->neg = a->neg;

Q_COPY_EXIT:
	return status;
}

/*
 * q_malloc ()
 * Description: This function allocates context memory for a long integer
 * data.
 * @ctx		QLIP context pointer
 */
uint32_t *q_malloc(struct q_lip_ctx *ctx, uint32_t size)
{
	/* memory size in words */
	uint32_t *ret;
	if ((ctx->cur_mem_ptr + size) > ctx->cur_mem_lmt)
		ret = 0;
	else if (!size)
		ret = 0;
	else {
		ret = (ctx->ctx_mem + ctx->cur_mem_ptr);
		ctx->cur_mem_ptr += size;
	}

	return ret;
}
EXPORT_SYMBOL(q_malloc);

/*
 * q_free ()
 * Description: This function de-allocates context memory for
 * a long integer data.
 * @ctx		QLIP context pointer
 */
int32_t q_free(struct q_lip_ctx *ctx, struct q_lint *z)
{
	/* pointer to the q_lint to be freed */
	if (z->limb + z->alloc != ctx->ctx_mem + ctx->cur_mem_ptr)
		ctx->status = Q_ERR_MEM_DEALLOC;
	else
		ctx->cur_mem_ptr -= z->alloc;

	return ctx->status;
}
EXPORT_SYMBOL(q_free);

/* common functions */
int q_is_zero(struct q_lint *a)
{
	int i;
	uint32_t *ap;
	ap = a->limb;

	if (!a->size)
		return 1;

	for (i = 0; i < (a->size); i++) {
		if (ap[i])
			return 0;
	}
	return 1;
}

int q_is_one(struct q_lint *a)
{
	uint32_t *ap;
	ap = a->limb;

	if (a->size > 1)
		return 0;
	if (a->size == 0)
		return 0;

	return (a->limb[0] == 1L);
}

void q_set_zero(struct q_lint *a)
{
	int i;
	uint32_t *ap;
	ap = a->limb;
	a->size = 0;
	a->neg = 0;

	for (i = 0; i < (a->alloc); i++)
		ap[i] = 0;
}

void q_set_one(struct q_lint *a)
{
	int i;
	uint32_t *ap;
	ap = a->limb;
	a->size = 1;
	a->neg = 0;

	for (i = 1; i < (a->alloc); i++)
		ap[i] = 0;

	*ap = 1;
}

int q_is_odd(struct q_lint *a)
{
	uint32_t *ap;
	ap = a->limb;

	return (*ap) & 1;
}

int q_leading_one(uint32_t r)
{
	/* do a binary search to find the leading 1 */
	uint32_t last, mlast, bmask;
	int k, br;

	br = 0;
	k = MACHINE_WD / 2;
	bmask = -1L << k;
	last = r;

	while (k) {
		mlast = last & bmask;
		if (mlast) {
			br += k;
			last = mlast;
		}
		k = k >> 1;
		bmask ^= (bmask >> k);
	}

	return br;
}
