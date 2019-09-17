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

#ifndef _Q_PKE_HW_H_
#define _Q_PKE_HW_H_

/* PKE mode */
#define PKE_MODE_DHPK 0x01
#define PKE_MODE_DHSS 0x02
#define PKE_MODE_RSAE 0x03
#define PKE_MODE_RCRT 0x04
#define PKE_MODE_DSAS 0x05
#define PKE_MODE_DSAV 0x06
#define PKE_MODE_MADD 0x43
#define PKE_MODE_MSUB 0x44
#define PKE_MODE_MMUL 0x45
#define PKE_MODE_MREM 0x46
#define PKE_MODE_MEXP 0x47
#define PKE_MODE_MINV 0x48

/* use '1' for normal word order and '-1' for BigNUM word order */
#define PKE_GMP_ORDER -1

/* these are wrappers to setup the PKE hardware memory */
int q_dh_pk_hw(uint32_t *xp, uint32_t blen_p, uint32_t *p,
			   uint32_t *g, uint32_t blen_x, uint32_t *x);
int q_dh_ss_hw(uint32_t *ss, uint32_t blen_p, uint32_t *p,
			   uint32_t *xp, uint32_t *y);
int q_rsa_enc_hw(uint32_t *c, uint32_t blen_n, uint32_t blen_e,
				 uint32_t *n, uint32_t *e, uint32_t *m);
int q_rsa_crt_hw(uint32_t *m, uint32_t blen_p, uint32_t blen_q,
				 uint32_t *p, uint32_t *q, uint32_t *dp,
				 uint32_t *dq, uint32_t *pinv,
				 uint32_t blen_c, uint32_t *c);
int q_dsa_sign_hw(uint32_t *r, uint32_t *s, uint32_t blen_p,
				  uint32_t *q, uint32_t *p, uint32_t *g,
				  uint32_t *d, uint32_t *h,
				  uint32_t *k);
int q_dsa_verf_hw(uint32_t *v, uint32_t blen_p, uint32_t *q,
				  uint32_t *p, uint32_t *g, uint32_t *y,
				  uint32_t *h, uint32_t *r,
				  uint32_t *s);
int q_modadd_hw(uint32_t *r, uint32_t blen_n, uint32_t *n,
				uint32_t *a, uint32_t *b);
int q_modsub_hw(uint32_t *r, uint32_t blen_n, uint32_t *n,
				uint32_t *a, uint32_t *b);
int q_modmul_hw(uint32_t *r, uint32_t blen_n, uint32_t *n,
				uint32_t *a, uint32_t *b);
int q_modexp_hw(uint32_t *r, uint32_t blen_n, uint32_t blen_e,
				uint32_t *n, uint32_t *e, uint32_t *a);
int q_modinv_hw(uint32_t *r, uint32_t blen_n, uint32_t *n, uint32_t *a);
int q_modrem_hw(uint32_t *r, uint32_t blen_n, uint32_t *n, uint32_t *a);

/* address mapped PKE registers */
#define PKE_HW_BASE_ADDR      0x00000000
#define PKE_HW_REG_CTLSTAT    (PKE_HW_BASE_ADDR + 0)
#define PKE_HW_REG_DIN        (PKE_HW_BASE_ADDR + 4)
#define PKE_HW_REG_DOUT       (PKE_HW_BASE_ADDR + 8)

#define PKE_CTL_RST           0x00000080
#define PKE_CTL_EN            0x00000001
#define PKE_STAT_DONE         0x00000002
#define PKE_STAT_BUSY         0x00000004

#endif /*_Q_PKE_HW_H_*/
