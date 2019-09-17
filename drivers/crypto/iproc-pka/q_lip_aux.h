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

#ifndef _Q_LIP_AUX_H_
#define _Q_LIP_AUX_H_

/*
 * initialize a long integer and allocate storage in the QLIP context
 * param ctx QLIP context pointer
 * param z long integer to be initialized
 * param size the memory size (in 32-bit words)
 * to be allocated for the long integer
 * note  long integer must be initialized before it is used.
 * Initialization allocates memory storage in the context.
 */
int32_t q_init(struct q_lip_ctx *ctx, struct q_lint *z, uint32_t size);

/*
 * copy a long integer
 * param z result long integer
 * param a source long integer
 */
int32_t q_copy(struct q_lint *z, struct q_lint *a);

/*
 * allocate memory in QLIP context
 * param ctx QLIP context pointer
 * param size the size of the memory to be allocated (in 32-bit words)
 */
uint32_t *q_malloc(struct q_lip_ctx *ctx, uint32_t size);

/*
 * free memory allocated from the long integer from QLIP context
 * param ctx QLIP context pointer
 * param z the long integer
 */
int32_t q_free(struct q_lip_ctx *ctx, struct q_lint *z);

/*
 * check if the long integer is zero
 * param a the long integer
 */
int q_is_zero(struct q_lint *a);

/*
 * check if the long integer is one
 * param a the long integer
 */
int q_is_one(struct q_lint *a);

/*
 * set the long integer to zero
 * param a the long integer
 */
void q_set_zero(struct q_lint *a);

/*
 * set the long integer to one
 * param a the long integer
 */
void q_set_one(struct q_lint *a);

/*
 * check if the long integer is odd
 * param a the long integer
 */
int q_is_odd(struct q_lint *a);

/*
 * find the leading '1' of a 32-bit integer,
 * may be mapped to a single instruction
 * param r the 32-b integer
 */
int q_leading_one(uint32_t r);
#endif /*_Q_LIP_AUX_H_*/
