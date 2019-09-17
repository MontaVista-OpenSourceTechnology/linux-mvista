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

#include "q_lip.h"
#include "q_lip_aux.h"
#include "q_pke_hw.h"

#ifdef QLIP_USE_GMP
#include "gmp.h"
#endif

/* these functions model PKE hardware access */
inline int is_pke_busy(void)
{
	return (*(int *)PKE_HW_REG_CTLSTAT) & PKE_STAT_BUSY;
}

inline int is_pke_done(void)
{
	return *((int *)PKE_HW_REG_CTLSTAT) & PKE_STAT_DONE;
}

inline void q_pke_hw_rst(void)
{
	*((int *)PKE_HW_REG_CTLSTAT) = (PKE_CTL_RST | PKE_CTL_EN);
}

/* send one 32-bit word data to PKE */
inline void q_pke_hw_load(uint32_t data)
{
	*((int *)PKE_HW_REG_DIN) = data;
}

inline void q_pke_hw_unload(uint32_t *data)
{
	*data = *((int *)PKE_HW_REG_DOUT);
}

/* firmware wrappers */
int q_modadd_hw(uint32_t *r, uint32_t blen_n, uint32_t *n, uint32_t *a,
				uint32_t *b)
{
	int i, size;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size = ((blen_n + 31) >> 5);
	context = ((PKE_MODE_MADD) << 16) + (2 + size * 3);
	q_pke_hw_load(context);
	context = (blen_n) << 16;
	q_pke_hw_load(context);

	for (i = 0; i < size; i++) {
		context = n[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = a[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = b[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size; i++) {
		q_pke_hw_unload(&context);
		r[i] = context;
	}

	return 0;
}

int q_modsub_hw(uint32_t *r, uint32_t blen_n, uint32_t *n, uint32_t *a,
				uint32_t *b)
{
	int i, size;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size = ((blen_n + 31) >> 5);
	context = ((PKE_MODE_MSUB) << 16) + (2 + size * 3);
	q_pke_hw_load(context);
	context = (blen_n) << 16;
	q_pke_hw_load(context);

	for (i = 0; i < size; i++) {
		context = n[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = a[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = b[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size; i++) {
		q_pke_hw_unload(&context);
		r[i] = context;
	}

	return 0;
}

int q_modmul_hw(uint32_t *r, uint32_t blen_n, uint32_t *n, uint32_t *a,
				uint32_t *b)
{
	int i, size;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size = ((blen_n + 31) >> 5);
	context = ((PKE_MODE_MMUL) << 16) + (2 + size * 3);
	q_pke_hw_load(context);
	context = (blen_n) << 16;
	q_pke_hw_load(context);

	for (i = 0; i < size; i++) {
		context = n[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = a[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = b[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size; i++) {
		q_pke_hw_unload(&context);
		r[i] = context;
	}

	return 0;
}

int q_modrem_hw(uint32_t *r, uint32_t blen_n, uint32_t *n, uint32_t *a)
{
	int i, size;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size = ((blen_n + 31) >> 5);
	context = ((PKE_MODE_MREM) << 16) + (2 + size * 2);
	q_pke_hw_load(context);
	context = (blen_n) << 16;
	q_pke_hw_load(context);

	for (i = 0; i < size; i++) {
		context = n[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = a[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size; i++) {
		q_pke_hw_unload(&context);
		r[i] = context;
	}

	return 0;
}

int q_modexp_hw(uint32_t *r, uint32_t blen_n, uint32_t blen_e, uint32_t *n,
				uint32_t *e, uint32_t *a)
{
	int i, size, size_e;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size = ((blen_n + 31) >> 5);
	size_e = ((blen_e + 31) >> 5);

	context = ((PKE_MODE_MEXP) << 16) + (2 + size * 2 + size_e);
	q_pke_hw_load(context);
	context = ((blen_n) << 16) + blen_e;
	q_pke_hw_load(context);

	for (i = 0; i < size; i++) {
		context = n[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = a[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_e; i++) {
		context = e[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size; i++) {
		q_pke_hw_unload(&context);
		r[i] = context;
	}

	return 0;
}

int q_modinv_hw(uint32_t *r, uint32_t blen_n, uint32_t *n, uint32_t *a)
{
	int i, size;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size = (blen_n >> 5);
	context = ((PKE_MODE_MINV) << 16) + (2 + size * 2);
	q_pke_hw_load(context);
	context = (blen_n) << 16;
	q_pke_hw_load(context);

	for (i = 0; i < size; i++) {
		context = n[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = a[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size; i++) {
		q_pke_hw_unload(&context);
		r[i] = context;
	}

	return 0;
}

int q_dh_pk_hw(uint32_t *xp, uint32_t blen_p, uint32_t *p,
				uint32_t *g, uint32_t blen_x, uint32_t *x)
{
	int i, size_p, size_x;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size_p = ((blen_p + 31) >> 5);
	size_x = ((blen_x + 31) >> 5);
	context = ((PKE_MODE_DHPK) << 16) + (2 + size_p * 2 + size_x);
	q_pke_hw_load(context);
	context = (blen_x << 16);
	q_pke_hw_load(context);
	context = (blen_p << 16) + blen_p;
	q_pke_hw_load(context);

	for (i = 0; i < size_p; i++) {
		context = p[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_p; i++) {
		context = g[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_x; i++) {
		context = x[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size_p; i++) {
		q_pke_hw_unload(&context);
		xp[i] = context;
	}

	return 0;
}

int q_dh_ss_hw(uint32_t *ss, uint32_t blen_p, uint32_t *p,
				uint32_t *xp, uint32_t *y)
{
	int i, size_p;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size_p = ((blen_p + 31) >> 5);
	context = ((PKE_MODE_DHSS) << 16) + (2 + size_p * 3);
	q_pke_hw_load(context);
	context = 0L;
	q_pke_hw_load(context);
	context = (blen_p << 16);
	q_pke_hw_load(context);

	for (i = 0; i < size_p; i++) {
		context = p[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_p; i++) {
		context = xp[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_p; i++) {
		context = y[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size_p; i++) {
		q_pke_hw_unload(&context);
		ss[i] = context;
	}

	return 0;
}

int q_rsa_enc_hw(uint32_t *c, uint32_t blen_n, uint32_t blen_e,
				uint32_t *n, uint32_t *e, uint32_t *m)
{
	int i, size, size_e;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size = ((blen_n + 31) >> 5);
	size_e = ((blen_e + 31) >> 5);

	context = ((PKE_MODE_RSAE) << 16) + (2 + size * 2 + size_e);
	q_pke_hw_load(context);
	context = ((blen_n) << 16) + blen_e;
	q_pke_hw_load(context);

	for (i = 0; i < size; i++) {
		context = n[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_e; i++) {
		context = e[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = m[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size; i++) {
		q_pke_hw_unload(&context);
		c[i] = context;
	}

	return 0;
}

int q_rsa_crt_hw(uint32_t *m, uint32_t blen_p, uint32_t blen_q,
				uint32_t *p, uint32_t *q, uint32_t *dp,
				uint32_t *dq, uint32_t *pinv,
				uint32_t blen_c, uint32_t *c)
{
	int i, size_p, size_q, size_c;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size_p = ((blen_p + 31) >> 5);
	size_q = ((blen_q + 31) >> 5);
	size_c = ((blen_c + 31) >> 5);

	context = ((PKE_MODE_RCRT) << 16) +
			(2 + size_p * 3 + size_q * 2 + size_c);
	q_pke_hw_load(context);
	context = ((blen_p) << 16) + blen_q;
	q_pke_hw_load(context);

	for (i = 0; i < size_p; i++) {
		context = p[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_q; i++) {
		context = q[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_p; i++) {
		context = dp[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_q; i++) {
		context = dq[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_p; i++) {
		context = pinv[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size_c; i++) {
		context = c[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < size_c; i++) {
		q_pke_hw_unload(&context);
		m[i] = context;
	}

	return 0;
}

int q_dsa_sign_hw(uint32_t *r, uint32_t *s, uint32_t blen_p,
					uint32_t *q, uint32_t *p, uint32_t *g,
					uint32_t *d, uint32_t *h, uint32_t *k)
{
	int i, size;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size = ((blen_p + 31) >> 5);
	context = ((PKE_MODE_DSAS) << 16) + (2 + size * 2 + 20);
	q_pke_hw_load(context);
	context = (blen_p) << 16;
	q_pke_hw_load(context);

	for (i = 0; i < 5; i++) {
		context = q[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = p[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = g[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < 5; i++) {
		context = d[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < 5; i++) {
		context = h[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < 5; i++) {
		context = k[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < 5; i++) {
		q_pke_hw_unload(&context);
		r[i] = context;
	}
	for (i = 0; i < 5; i++) {
		q_pke_hw_unload(&context);
		s[i] = context;
	}

	return 0;
}

int q_dsa_verf_hw(uint32_t *v, uint32_t blen_p, uint32_t *q,
					uint32_t *p, uint32_t *g, uint32_t *y,
					uint32_t *h, uint32_t *r, uint32_t *s)
{
	int i, size;
	uint32_t context;

	q_pke_hw_rst();
	while (is_pke_busy())
		;

	size = ((blen_p + 31) >> 5);
	context = ((PKE_MODE_DSAV) << 16) + (2 + size * 3 + 20);
	q_pke_hw_load(context);
	context = (blen_p) << 16;
	q_pke_hw_load(context);

	for (i = 0; i < 5; i++) {
		context = q[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = p[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = g[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < size; i++) {
		context = y[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < 5; i++) {
		context = h[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < 5; i++) {
		context = r[i];
		q_pke_hw_load(context);
	}

	for (i = 0; i < 5; i++) {
		context = s[i];
		q_pke_hw_load(context);
	}

	while (!is_pke_done())
		;
	for (i = 0; i < 5; i++) {
		q_pke_hw_unload(&context);
		v[i] = context;
	}

	return 0;
}
