/*
 * Copyright 2015, 2018 Broadcom
 *
 * This program is the proprietary software of Broadcom and/or
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

#include <generated/autoconf.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/crypto.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>

#include "q_lip.h"
#include "q_lip_utils.h"
#include "q_lip_aux.h"
#include "q_pka_hw.h"
#include "q_dh.h"
#include "q_dsa.h"
#include "q_rsa.h"
#include "q_rsa_4k.h"
#include "q_elliptic.h"

/* PKA din dout offsets */
#define reg_din_offset	0x4
#define reg_dout_offset	0x8

/*
 * Test buffer used to verify the
 * algorithms with predifined values
 */
#define Q_MAXIMUM_ALLOC 8192

enum {
	ECDSA_192,
	ECDSA_224,
	ECDSA_256,
	ECDSA_CUST,
	ECDSA_MAX
};

int cust_key;

/* data structure for ecdsa test input */
struct ecdsa_curve {
	/* Elliptic Curve parameters */
	uint32_t A[8], B[8], p[8], n[8];
	uint32_t GX[8], GY[8];
	uint32_t nbits;
};

struct ecdsa_input {
	uint32_t d[8]; /* Private Key */
	uint32_t QX[8], QY[8], QZ[8]; /* Public key */
	uint32_t k[8]; /* Random integer */
	uint32_t h[8]; /* Hash */
	uint32_t hlen; /* Hash length in bits */
	uint32_t ER[8], ES[8]; /* Expected R and S */
};

#define RSA_BITS 32

static uint32_t QLIP_MEM[Q_MAXIMUM_ALLOC];

static struct ecdsa_curve ec[] = {
	{ /* ECDSA_192 */
		.A = { 0xFFFFFFFC, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF },
		.B = {0xC146B9B1, 0xFEB8DEEC, 0x72243049, 0x0FA7E9AB, 0xE59C80E7, 0x64210519 },
		.p = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF },
		.n = {0xB4D22831, 0x146BC9B1, 0x99DEF836, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF },
		.GX = {0x82FF1012, 0xF4FF0AFD, 0x43A18800, 0x7CBF20EB, 0xB03090F6, 0x188DA80E },
		.GY = {0x1E794811, 0x73F977A1, 0x6B24CDD5, 0x631011ED, 0xFFC8DA78, 0x07192B95 },
		.nbits = 192,
	}, { /* ECDSA_224 */
		.A = {0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFE, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF},
		.B = {0x2355FFB4, 0x270B3943, 0xD7BFD8BA, 0x5044B0B7, 0xF5413256, 0x0C04B3AB, 0xB4050A85},
		.p = {0x00000001, 0x00000000, 0x00000000, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF},
		.n = {0x5C5C2A3D, 0x13DD2945, 0xE0B8F03E, 0xFFFF16A2, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF},
		.GX = {0x115C1D21, 0x343280D6, 0x56C21122, 0x4A03C1D3, 0x321390B9, 0x6BB4BF7F, 0xB70E0CBD},
		.GY = {0x85007E34, 0x44D58199, 0x5A074764, 0xCD4375A0, 0x4C22DFE6, 0xB5F723FB, 0xBD376388},
		.nbits = 224,
	}, { /* ECDSA_256 */
		.A = {0xfffffffc, 0xffffffff, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000001, 0xffffffff},
		.B = {0x27d2604b, 0x3bce3c3e, 0xcc53b0f6, 0x651d06b0, 0x769886bc, 0xb3ebbd55, 0xaa3a93e7, 0x5ac635d8},
		.p = {0xffffffff, 0xffffffff, 0xffffffff, 0x00000000, 0x00000000, 0x00000000, 0x00000001, 0xffffffff},
		.n = {0xfc632551, 0xf3b9cac2, 0xa7179e84, 0xbce6faad, 0xffffffff, 0xffffffff, 0x00000000, 0xffffffff},
		.GX = {0xd898c296, 0xf4a13945, 0x2deb33a0, 0x77037d81, 0x63a440f2, 0xf8bce6e5, 0xe12c4247, 0x6b17d1f2},
		.GY = {0x37bf51f5, 0xcbb64068, 0x6b315ece, 0x2bce3357, 0x7c0f9e16, 0x8ee7eb4a, 0xfe1a7f9b, 0x4fe342e2},
		.nbits = 256,
	}
};

static struct ecdsa_input ei[] = {
	{ /* ECDSA_192 */
		.d = { 0x5EA475FB, 0x92AE8BAF, 0xB1111AEB, 0x89030B5C, 0xC15BF0FD, 0x1A8D598F },
		.QX = { 0x1AB60DC8, 0x17869EA3, 0x72DF8B17, 0x6159B3B5, 0x77E3E772, 0x46B771CE },
		.QY = { 0xDCC1ABBE, 0x24CA0309, 0xB24D9C5D, 0x56959D5A, 0x95AF1523, 0x734E6348 },
		.QZ = { 0x105A99BB, 0xEC25DD49, 0x1881337F, 0x2E5BA6CF, 0x9AF1D0C4, 0x96E27CAC },
		.h = { 0x9cd0d89d, 0x7850c26c, 0xba3e2571, 0x4706816a, 0xa9993e36 },
		.hlen = 20,
		.k = { 0x82FA2F4E, 0x2983169D, 0xF85F7DFB, 0x8BB1E761, 0x46BBEB7F, 0xFA6DE297 },
		.ER = { 0xF749FEAD, 0xA89F29B0, 0x3D39B2C4, 0x34C330C4, 0x0FF147B7, 0x88505238 },
		.ES = { 0x951DF686, 0xCB390046, 0xD4D804C3, 0xF1070CF1, 0x06DEF82B, 0xE9ECC781 }
	}, { /* ECDSA_224 */
		.d = {0xc830d615, 0x072000df, 0x3c0f46bf, 0x8e6eafa0, 0x1e2ff1b8, 0x0c7ed546, 0x16797b5c},
		.QX = {0x12174eab, 0xd1a9492a, 0x4da8a641, 0x8787af9b, 0xd07ae5f9, 0x6e6e88f1, 0x60549575},
		.QY = {0x8dfc6669, 0x3df3959b, 0x0af9ef7c, 0x1a42505d, 0x6ef1df86, 0x17decc80, 0xf5cc733b},
		.QZ = {0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
		.h = {0x27e65e67, 0x77ec651a, 0x600437d0, 0x14a490e7, 0x74676006, 0xbf70eee8, 0x07eb2a50},
		.hlen = 28,
		.k = {0x7137eec0, 0x4bd29a52, 0x756b3ff6, 0x7dae722e, 0x4b8dd8c1, 0x8117f48b, 0xd9a5a732},
		.ER = {0xdbc7b0d6, 0x88049d0f, 0x46b7af08, 0x07d333af, 0x1d74e45b, 0xcdd4866b, 0x2fc2cff8},
		.ES = {0x1dac41eb, 0x396321b1, 0x8f281793, 0xa1fd99b8, 0x9d6431b9, 0xea93e0fd, 0x8d9cc4c8}
	}, { /* ECDSA_256 */
		.d = { 0x3452b38a, 0x9f2d7d5b, 0x851bf634, 0x3f04d7d6, 0xc21a472b, 0x56ff68cf, 0xb16845ed, 0x70a12c2d},
		.QX = {0xf26680a8, 0xf7635eaf, 0x2d22cba4, 0x8691a326, 0x6e2bd3d8, 0xd70cf69a, 0x7464a6ea, 0x8101ece4},
		.QY = {0x36c0c3a9, 0xa6240799, 0x8f0a5aba, 0xd3ca43e7, 0xd58f1783, 0xf67d9cb4, 0x1d599235, 0xd8a12ba6},
		.QZ = {0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
		.h = { 0xba2b8377, 0xf020bd39, 0xbb59e9c5, 0x8f30f9d6, 0x9324222c, 0xf96eac5e, 0xdc8bd688, 0x7c3e883d},
		.hlen = 32,
		.k = { 0x34ee45d0, 0x1965c7b1, 0xfa47055b, 0x5b12ae37, 0xecaed496, 0x4cef3f71, 0x85643433, 0x580ec00d},
		.ER = {0xd4e6f27c, 0x61e805d5, 0x86bb8156, 0xddd70ddf, 0x533f5dc6, 0x39ff2f80, 0x47160bbd, 0x7214bc96},
		.ES = {0xaf2bc367, 0x92dbeaa1, 0xf9e14935, 0x3317d3e3, 0x6209f401, 0xdaa3233b, 0x980f961b, 0x7d1ff961}
	}, { /* ECDSA_CUST*/
		.QZ = {0x00000001, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000},
		.h = { 0xba2b8377, 0xf020bd39, 0xbb59e9c5, 0x8f30f9d6, 0x9324222c, 0xf96eac5e, 0xdc8bd688, 0x7c3e883d},
		.hlen = 32,
		.k = { 0x34ee45d0, 0x1965c7b1, 0xfa47055b, 0x5b12ae37, 0xecaed496, 0x4cef3f71, 0x85643433, 0x580ec00d},
		.ER = {0xd4e6f27c, 0x61e805d5, 0x86bb8156, 0xddd70ddf, 0x533f5dc6, 0x39ff2f80, 0x47160bbd, 0x7214bc96},
		.ES = {0xa43e0d72, 0xc722c0ac, 0xccfbc831, 0xafc8aee0, 0xcf556e53, 0x48edfb97, 0x0840cb7e, 0x642fa418}
	}
};

static int do_dsa_sign_verify(void)
{
	struct q_lip_ctx ctx, ctx_v;
	struct q_lint d, h, k, v, y;
	uint32_t status;
	struct q_dsa_param dsa;
	struct q_signature rs, rs_v;
	struct timespec ts1, ts2;
	unsigned long time_ns;

	/* Private key */
	unsigned int key[1] = {
		0x3
	};

	/* Hash of the data */
	unsigned int hash[1] = {
		0x2
	};

	/*
	 * DSA public key is (p, q, g, y)
	 * pub = y = g.x mod p
	 */
	unsigned int g[1] = {
		0xA
	};
	unsigned int p[1] = {
		0x29
	};
	unsigned int q[1] = {
		0x5
	};
	unsigned int pub[1] = {
		0x10
	};

	/* Random number  0 < random < q */
	unsigned int random[1] = {
		0x2
	};

	q_ctx_init(&ctx, QLIP_MEM, Q_MAXIMUM_ALLOC, NULL);
	q_import(&ctx, &k, 1, -1, 0, random);
	q_import(&ctx, &d, 1, -1, 0, key);
	q_import(&ctx, &h, 1, -1, 1, hash);
	q_import(&ctx, &dsa.g, 1, -1, 0, g);
	q_import(&ctx, &dsa.p, 1, -1, 0, p);
	q_import(&ctx, &dsa.q, 1, -1, 0, q);

	/* DSA signature ==> (r,s) */
	q_init(&ctx, &rs.r, 1);
	q_init(&ctx, &rs.s, 1);

	ktime_get_ts(&ts1);
	/* Calculate (r, s) suing DSA signing algorithm */
	status = q_dsa_sign(&ctx, &rs, &dsa, &d, &h, &k);
	ktime_get_ts(&ts2);
	time_ns = (ts2.tv_sec * 1000000000 + (ts2.tv_nsec)) -
			(ts1.tv_sec * 1000000000 + (ts1.tv_nsec));
	pr_info("Time taken for DSA Sign = %luns\n", time_ns);
	if (status != Q_SUCCESS) {
		pr_err("%s: q_dsa_sign failed status %d\n", __func__, status);
		status = ECANCELED;
	}
	pr_info("r=%x s=%x\n", *(rs.r.limb), *(rs.s.limb));

	/* Verify (r,s) using DSA verification algorithm */
	q_ctx_init(&ctx_v, QLIP_MEM, Q_MAXIMUM_ALLOC, NULL);
	q_init(&ctx_v, &v, 1);
	q_import(&ctx_v, &dsa.g, 1, -1, 0, g);
	q_import(&ctx_v, &dsa.p, 1, -1, 0, p);
	q_import(&ctx_v, &dsa.q, 1, -1, 0, q);
	q_import(&ctx_v, &y, 1, -1, 0, pub);
	q_import(&ctx_v, &h, 1, -1, 1, hash);
	q_import(&ctx_v, &rs_v.r, 1, -1, 0, rs.r.limb);
	q_import(&ctx_v, &rs_v.s, 1, -1, 0, rs.s.limb);

	ktime_get_ts(&ts1);
	/* Verify the signature */
	status = q_dsa_verify(&ctx_v, &v, &dsa, &y, &h, &rs_v);
	ktime_get_ts(&ts2);
	time_ns = (ts2.tv_sec * 1000000000 + (ts2.tv_nsec)) -
			(ts1.tv_sec * 1000000000 + (ts1.tv_nsec));
	pr_info("Time taken for DSA Verify = %luns\n", time_ns);

	if (status != Q_SUCCESS) {
		pr_err("%s: q_dsa_sign failed status %d\n", __func__, status);
		status = ECANCELED;
	}

	pr_info("DSA sign and verify passed\n");

	q_free(&ctx, &rs.s);
	q_free(&ctx, &rs_v.s);
	q_free(&ctx, &rs.r);
	q_free(&ctx, &rs_v.r);
	q_free(&ctx, &dsa.q);
	q_free(&ctx, &dsa.p);
	q_free(&ctx, &dsa.g);
	q_free(&ctx, &h);
	q_free(&ctx, &d);
	q_free(&ctx, &k);
	q_free(&ctx, &v);
	q_free(&ctx, &y);

	return status;
}
static int do_dh_compute_key(void)
{
	struct q_lip_ctx ctx;
	struct q_dh_param dh;
	struct q_lint res1, priv_val, shared_key, secret, xpub;
	int32_t status;
	struct timespec ts1, ts2;
	unsigned long time_ns;

	/* DH costants p and g */
	unsigned int p[1] = {
		0x21D
	};
	unsigned int g[1] = {
		0xA
	};

	/* Data from source one */
	unsigned int a[1] = {
		0x2
	};

	/* Data from source 2 */
	unsigned int b_secret[1] = {
		0x3
	};

	q_ctx_init(&ctx, QLIP_MEM, Q_MAXIMUM_ALLOC, NULL);

	ktime_get_ts(&ts1);
	/* Find the public value from source 1 */
	q_import(&ctx, &dh.p, 1, -1, 0, p);
	q_import(&ctx, &dh.g, 1, -1, 0, g);
	q_import(&ctx, &priv_val, 1, -1, 0, a);

	q_init(&ctx, &res1, 1);

	status = q_dh_pk(&ctx, &res1, &dh, &priv_val);

	if (status != Q_SUCCESS) {
		pr_err("%s: q_dh_pk failed status %d\n", __func__, status);
		status = ECANCELED;
	}

	/* Find the Shared key */
	q_import(&ctx, &dh.p, 1, -1, 0, p);
	q_import(&ctx, &dh.g, 1, -1, 0, g);
	q_import(&ctx, &xpub, 1, -1, 0, res1.limb);
	q_import(&ctx, &secret, 1, -1, 0, b_secret);

	q_init(&ctx, &shared_key, 1);

	status = q_dh_ss(&ctx, &shared_key, &dh, &xpub, &secret);

	if (status != Q_SUCCESS) {
		pr_err("%s: q_dh_pk failed status %d\n", __func__, status);
		status = ECANCELED;
	}
	ktime_get_ts(&ts2);
	time_ns = (ts2.tv_sec * 1000000000 + (ts2.tv_nsec)) -
			(ts1.tv_sec * 1000000000 + (ts1.tv_nsec));
	pr_info("Time taken for DH key computation = %luns\n", time_ns);

	pr_info("DH Result:\np=%x g=%x a=%x b=%x\nshared_key=%x\n",
		p[0], g[0], a[0], b_secret[0], *(shared_key.limb));
	q_free(&ctx, &res1);
	q_free(&ctx, &priv_val);
	q_free(&ctx, &shared_key);
	q_free(&ctx, &xpub);
	q_free(&ctx, &secret);
	q_free(&ctx, &dh.g);
	q_free(&ctx, &dh.p);

	return status;
}

/*
 * r = (p-1)*(q-1)
 * 1 < e < r, such that gcd(e, r) = 1
 */
unsigned int e[RSA_BITS / 32] = {
	0x00000003
};

/* Data given to RSA algorithm */
unsigned int a[RSA_BITS / 32] = {
	0x00029983
};

unsigned int q[RSA_BITS / 32] = {
	0x15B
};

/* dQ = (1/e) mod (q-1) */
unsigned int dq[RSA_BITS / 32] = {
	0xE7
};

/* dP = (1/e) mod (p-1) */
unsigned int dp[RSA_BITS / 32] = {
	0x3B7
};

/* qInv = (1/q) mod p */
unsigned int qinv[RSA_BITS / 32] = {
	0x425
};

/* RSA prime number p and q */
unsigned int p[RSA_BITS / 32] = {
	0x593
};

/* n = pq */
unsigned int n[RSA_BITS / 32] = {
	0x00078e41
};

static int do_encr_decr(void)
{
	struct q_lint exp, base, mod, res0, res1, res2;
	struct q_lip_ctx ctx;
	struct q_rsa_crt_key rsa;
	int32_t status;
	struct timespec ts1, ts2;
	unsigned long time_ns;

	pr_info("Testing RSA encryption decryption\n");

	q_ctx_init(&ctx, QLIP_MEM, Q_MAXIMUM_ALLOC, NULL);

	q_import(&ctx, &exp,  RSA_BITS / 32, 1, 0, e);
	q_import(&ctx, &base, RSA_BITS / 32, 1, 0, a);
	q_import(&ctx, &mod,  RSA_BITS / 32, 1, 0, n);
	q_init(&ctx, &res1, RSA_BITS / 32);


	ktime_get_ts(&ts1);
	/* calls the modexp function for encryption*/
	status = q_modexp(&ctx, &res1, &base, &exp, &mod);
	ktime_get_ts(&ts2);
	time_ns = (ts2.tv_sec * 1000000000 + (ts2.tv_nsec)) -
			(ts1.tv_sec * 1000000000 + (ts1.tv_nsec));
	pr_info("Time taken for RSA [%d bit] enc = %luns\n", RSA_BITS, time_ns);

	if (status != Q_SUCCESS)
		pr_err("RSA encrypt failed %d\n", status);

	/* decrypt the same and verify */
	q_import(&ctx, &rsa.p, RSA_BITS / 32, 1, 0, p);
	q_import(&ctx, &rsa.q, RSA_BITS / 32, 1, 0, q);
	q_import(&ctx, &rsa.dp, RSA_BITS / 32, 1, 0, dp);
	q_import(&ctx, &rsa.dq, RSA_BITS / 32, 1, 0, dq);
	q_import(&ctx, &rsa.qinv, RSA_BITS / 32, 1, 0, qinv);

	q_init(&ctx, &res2, RSA_BITS / 32);

	ktime_get_ts(&ts1);
	status = q_rsa_crt_4k(&ctx, &res2, &rsa, &res1);
	ktime_get_ts(&ts2);
	time_ns = (ts2.tv_sec * 1000000000 + (ts2.tv_nsec)) -
			(ts1.tv_sec * 1000000000 + (ts1.tv_nsec));
	pr_info("Time taken for RSA [%d bit] dec = %luns\n", RSA_BITS, time_ns);

	if (status != Q_SUCCESS)
		pr_err("RSA encrypt decrypt failed %d\n", status);

	/* Compare the decrypted data to data sent for encryption */
	if (*(res2.limb) == *(base.limb))
		pr_info("RSA encrypt and decrypt successful\n");

	q_free(&ctx, &res1);
	q_free(&ctx, &res0);
	q_free(&ctx, &res2);
	q_free(&ctx, &mod);
	q_free(&ctx, &base);
	q_free(&ctx, &exp);
	q_free(&ctx, &rsa.p);
	q_free(&ctx, &rsa.q);
	q_free(&ctx, &rsa.dp);
	q_free(&ctx, &rsa.dq);
	q_free(&ctx, &rsa.qinv);

	return status;
}

#define BITLEN2BYTELEN(len) (((len)+7) >> 3)
#define BYTELEN2BITLEN(len) ((len) << 3)

/*
 * Function: set_param_bn
 * This function formats a parameter for processing by the qlip library
 * The input data is already in big number format.

 * Inputs:
 * qparm     = Pointer to qlib_t structure
 * param     = uint8_t pointer to input parameter
 * param_len = length of param in bits
 * pad_len   = length of padded param in bytes
 */
void set_param_bn(struct q_lint *qparam, uint8_t *param,
				uint32_t param_len, uint32_t pad_len)
{
	qparam->limb = (uint32_t *)param;
	qparam->size = BITLEN2BYTELEN(param_len);

	if (qparam->size % 4)
		qparam->size = (qparam->size/4)+1;
	else
		qparam->size = (qparam->size/4);
	qparam->alloc = qparam->size;
	qparam->neg = 0;
}

/*---------------------------------------------------------------
 * Name    : do_ecdsa_verify
 * Purpose : Verify an Elliptic curve DSA signature
 * Input   : pointers to the data parameters
 *			hash: SHA1 hash of the message
 *			length: hashLength in bytes
 *			type: Curve type, 0 - Prime field, 1 - binary field
 *			p: prime p
 *			p_bitlen: size of prime p in bits
 *			a: Parameter a
 *			b: Parameter b
 *			n: The order of the Base point G (represented by "r"
 *			in sample curves defined in FIPS 186-2 document)
 *			Gx: x coordinate of Base point G
 *			Gy: y coordinate of Base point G
 *			Qx: x coordinate of Base point Q
 *			Qy: y coordinate of Base point Q
 *			Qz: z coordinate of Base point Q
 *			r: first part of signature
 *			s: second part of signature
 * Return  : v. If v == r then verify success
 *---------------------------------------------------------------
 */
static int do_ecdsa_verify(
			struct q_lip_ctx *ctx,
			uint8_t *hash,
			uint32_t hashLength,
			uint8_t type,
			uint8_t *p,
			uint32_t p_bitlen,
			uint8_t *a,
			uint8_t *b,
			uint8_t *n,
			uint8_t *Gx,
			uint8_t *Gy,
			uint8_t *Qx,
			uint8_t *Qy,
			uint8_t *Qz,
			uint8_t *r,
			uint8_t  *s)
{
	int status;
	struct q_point ptG;	/* q_point pointer to base point G */
	struct q_point ptQ;	/* q_point pointer to base point Q Public key */
	struct q_curve ec;	/* q_point pointer to curve parameters */

	struct q_lint qlip_v;		/* q_lint pointer to v */
	struct q_lint qlip_hash;	/* q_lint pointer to h */
	struct q_signature qlip_rs;	/* q_signature pointer to rs */
	uint8_t *Gz;
	int i;
	uint32_t QV[(p_bitlen/8)];
	uint8_t *v = (uint8_t *)QV;
	struct timespec ts1, ts2;
	unsigned long time_ns;

	/* Set curve parameters */
	ec.type = type;
	set_param_bn(&ec.n, n, p_bitlen, 0);

	Gz = (uint8_t *)q_malloc(ctx, ec.n.alloc);
	if (!Gz)	{
		pr_err("%s Error Allocating Memory\n", __func__);
		return -1;
	}

	for (i = 0; i < (ec.n.alloc * 4); i++)
		Gz[i] = 0;

	((uint32_t *)Gz)[0] = 0x00000001;

	set_param_bn(&ec.a, a, p_bitlen, 0);
	set_param_bn(&ec.b, b, p_bitlen, 0);
	set_param_bn(&ec.p, p, p_bitlen, 0);

	/* Set Base pointer G parameters */
	set_param_bn(&ptG.X, Gx, p_bitlen, 0);
	set_param_bn(&ptG.Y, Gy, p_bitlen, 0);
	set_param_bn(&ptG.Z, Gz, p_bitlen, 0);

	/* Set public key Q parameters */
	set_param_bn(&ptQ.X, Qx, p_bitlen, 0);
	set_param_bn(&ptQ.Y, Qy, p_bitlen, 0);
	set_param_bn(&ptQ.Z, Qz, p_bitlen, 0);

	set_param_bn(&qlip_hash, hash, BYTELEN2BITLEN(hashLength), hashLength);

	/* Set signature */
	set_param_bn(&qlip_rs.r, r, p_bitlen, 0);
	set_param_bn(&qlip_rs.s, s, p_bitlen, 0);
	set_param_bn(&qlip_v, v, p_bitlen, 0);

	ktime_get_ts(&ts1);
	/* Call the qlip function. */
	status = q_ecp_ecdsa_verify(ctx,	/* QLIP context pointer */
				&qlip_v,	/* q_lint pointer to v */
				&ptG,		/* q_lint pointer to G */
				&ec,		/* q_curve pointer to curve */
				&ptQ,		/* q_point pointer to Q */
				&qlip_hash,	/* q_lint pointer to hash */
				&qlip_rs);
	ktime_get_ts(&ts2);
	time_ns = (ts2.tv_sec * 1000000000 + (ts2.tv_nsec)) -
			(ts1.tv_sec * 1000000000 + (ts1.tv_nsec));
	pr_info("Time taken for ecdsa_verify = %luns\n", time_ns);

	if (memcmp(v, r, (p_bitlen / 8)))
		return -1;

	return 0;
}

static int do_ecdsa_sign(struct q_lip_ctx *ctx,
			uint8_t *h,
			uint32_t hashLength,
			uint8_t type,
			uint8_t *p,
			uint32_t p_bitlen,
			uint8_t *a,
			uint8_t *b,
			uint8_t *n,
			uint8_t *Gx,
			uint8_t *Gy,
			uint8_t *k,
			uint32_t check_sign,
			uint8_t *d,
			uint8_t *r,
			uint8_t *s,
			uint8_t *er,
			uint8_t *es)
{
	struct q_point ptG;     /* q_point pointer to base point G */
	struct q_curve ec;      /* q_point pointer to curve parameters */
	struct q_signature qlip_rs;   /* q_signature pointer to rs */
	struct q_lint qd;         /* q_lint pointer to d */
	struct q_lint qlip_hash;         /* q_lint poitner to hash */
	struct q_lint qk;         /* q_lint pointer to k */
	uint8_t *Gz;
	int i, status;
	struct timespec ts1, ts2;
	unsigned long time_ns;

	ec.type = type; /* Prime Field */
	set_param_bn(&ec.n, n, p_bitlen, 0);

	Gz = (uint8_t *)q_malloc(ctx, ec.n.alloc);
	if (!Gz)  {
		pr_err("Error Allocating Memory\n");
		return -1;
	}

	for (i = 0; i < (ec.n.alloc * 4); i++)
		Gz[i] = 0;

	((uint32_t *)Gz)[0] = 0x00000001;

	set_param_bn(&ec.a, a, p_bitlen, 0);
	set_param_bn(&ec.b, b, p_bitlen, 0);
	set_param_bn(&ec.p, p, p_bitlen, 0);

	/* Set Base pointer G parameters */
	set_param_bn(&ptG.X, Gx, p_bitlen, 0);
	set_param_bn(&ptG.Y, Gy, p_bitlen, 0);
	set_param_bn(&ptG.Z, Gz, p_bitlen, 0);

	/* Set private key */
	set_param_bn(&qd, d, p_bitlen, 0);
	/* Set hash, 20 bytes */
	set_param_bn(&qlip_hash, h, BYTELEN2BITLEN(hashLength), hashLength);
	/* Set random number k */
	set_param_bn(&qk, k, p_bitlen, 0);

	/* Set rs, we need to break into two fragments. */
	set_param_bn(&qlip_rs.r, r, p_bitlen, 0);
	set_param_bn(&qlip_rs.s, s, p_bitlen, 0);

	ktime_get_ts(&ts1);
	/* Call the qlip function. */
	status = q_ecp_ecdsa_sign(ctx,		/* QLIP context pointer */
				&qlip_rs,	/* q_signature pointer to rs */
				&ptG,		/* q_point poitner to G */
				&ec,		/* q_curve pointer to curve */
				&qd,		/* q_lint pointer to d */
				&qlip_hash,	/* q_lint pointer to hash */
				&qk);		/* q_lint pointer to k */
	ktime_get_ts(&ts2);
	time_ns = (ts2.tv_sec * 1000000000 + (ts2.tv_nsec)) -
			(ts1.tv_sec * 1000000000 + (ts1.tv_nsec));
	pr_info("Time taken for ecdsa_sign = %luns\n", time_ns);

	if (check_sign) {
		if (memcmp(r, er, p_bitlen/8) || memcmp(s, es, p_bitlen/8))
			return -1;
	}

	return 0;
}

static int do_ecdsa_sign_verify(int ec_type, int ei_type)
{
	uint32_t QR[8] = {};
	uint32_t QS[8] = {};
	struct q_lip_ctx ctx;
	int i, status;
	int check_sign = 1;
	uint32_t *qr, *qs;

	qr = ei[ei_type].ER;
	qs = ei[ei_type].ES;
	if (ei_type == ECDSA_CUST) {
		check_sign = 0;
		qr = QR;
		qs = QS;
	}

	q_ctx_init(&ctx, QLIP_MEM, Q_MAXIMUM_ALLOC, NULL);

	/* Testing ecdsa_sign, external random number... */
	status = do_ecdsa_sign(&ctx, (uint8_t *)ei[ei_type].h, ei[ei_type].hlen,
			0, (uint8_t *)ec[ec_type].p, ec[ec_type].nbits,
			(uint8_t *)ec[ec_type].A, (uint8_t *)ec[ec_type].B,
			(uint8_t *)ec[ec_type].n, (uint8_t *)ec[ec_type].GX,
			(uint8_t *)ec[ec_type].GY, (uint8_t *)ei[ei_type].k,
			check_sign, (uint8_t *)ei[ei_type].d, (uint8_t *)QR,
			(uint8_t *)QS, (uint8_t *)ei[ei_type].ER,
			(uint8_t *)ei[ei_type].ES);
	if (status)
		pr_err("EC DSA (P-%d) Sign Failed\n", ec[ec_type].nbits);
	else if (check_sign)
		pr_info("EC DSA (P-%d) Sign Sucessful\n", ec[ec_type].nbits);
	else {
		pr_info("EC DSA (P-%d) Sign\n", ec[ec_type].nbits);
		pr_info("R :\n");
		for (i = 0; i < (ec[ec_type].nbits / 32); i++)
			pr_info("\t%08x\n", QR[i]);
		pr_info("S :\n");
		for (i = 0; i < (ec[ec_type].nbits / 32); i++)
			pr_info("\t%08x\n", QS[i]);
	}

	/* Testing ecdsa_verify... */
	status = do_ecdsa_verify(&ctx, (uint8_t *)ei[ei_type].h,
			ei[ei_type].hlen, 0, (uint8_t *)ec[ec_type].p,
			ec[ec_type].nbits, (uint8_t *)ec[ec_type].A,
			(uint8_t *)ec[ec_type].B, (uint8_t *)ec[ec_type].n,
			(uint8_t *)ec[ec_type].GX, (uint8_t *)ec[ec_type].GY,
			(uint8_t *)ei[ei_type].QX, (uint8_t *)ei[ei_type].QY,
			(uint8_t *)ei[ei_type].QZ, (uint8_t *)qr,
			(uint8_t *)qs);

	if (status)
		pr_err("EC DSA (P-%d) Verify Failed\n", ec[ec_type].nbits);
	else
		pr_info("EC DSA (P-%d) Verify Sucessful\n", ec[ec_type].nbits);

	return status;
}


static int do_test(int m)
{
	int ret = 0;

	switch (m) {
	case 0:
		ret = do_encr_decr();
		break;
	case 1:
		ret = do_dh_compute_key();
		break;
	case 2:
		ret = do_dsa_sign_verify();
		break;
	case 3:
		ret = do_ecdsa_sign_verify(ECDSA_192, ECDSA_192);
		break;
	case 4:
		ret = do_ecdsa_sign_verify(ECDSA_224, ECDSA_224);
		break;
	case 5:
		ret = do_ecdsa_sign_verify(ECDSA_256, ECDSA_256);
		break;
	case 6:
		ret = do_ecdsa_sign_verify(cust_key, ECDSA_CUST);
		break;
	default:
		pr_err("Invalid Mode selected\n");
		pr_info("Available Options are...\n");
		pr_info("\t 0 -> RSA [%d bit]\n", RSA_BITS);
		pr_info("\t 1 -> DH\n");
		pr_info("\t 2 -> DSA\n");
		pr_info("\t 3 -> ECDSA P-192\n");
		pr_info("\t 4 -> ECDSA P-224\n");
		pr_info("\t 5 -> ECDSA P-256\n");
	}

	return ret;

}

static ssize_t store_pka_mode(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	int mode, err;

	sscanf(buf, "%d", &mode);

	err = do_test(mode);

	if (err < 0)
		pr_err("pka test failed\n");
	return strnlen(buf, count);
}

static int parse_input(const char *buf, int indx, uint32_t *out, int count)
{
	int i, nbl = 0, ret = 0;
	uint32_t val, tmp = 0;

	for (i = indx + count - 2; i >= indx; i--) {
		if (buf[i] == ':')
			continue;
		else if (buf[i] >= 'a' && buf[i] <= 'f')
			val = 10 + buf[i] - 'a';
		else if (buf[i] >= 'A' && buf[i] <= 'F')
			val = 10 + buf[i] - 'A';
		else if (buf[i] >= '0' && buf[i] <= '9')
			val = buf[i] - '0';
		else
			return -EINVAL;

		ret++;
		tmp = (tmp >> 4) | (val << 28);
		nbl++;
		if (nbl == 8) {
			*out = tmp;
			nbl = 0;
			tmp = 0;
			out++;
		}
	}
	return ret;
}

static ssize_t store_ecdsa_pub(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = parse_input(buf, 3, ei[ECDSA_CUST].QX, count/2);
	if (ret != 48 && ret != 56 && ret != 64)
		pr_err("Invalid public key\n");
	ret = parse_input(buf, count/2 + 1, ei[ECDSA_CUST].QY, count/2);
	if (ret != 48 && ret != 56 && ret != 64)
		pr_err("Invalid public key\n");
	return strnlen(buf, count);
}

static ssize_t store_ecdsa_priv(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;

	ret = parse_input(buf, 0, ei[ECDSA_CUST].d, count);

	if (ret == 48)
		cust_key = ECDSA_192;
	else if (ret == 56)
		cust_key = ECDSA_224;
	else if (ret == 64)
		cust_key = ECDSA_256;
	else
		pr_err("Invalid private key");
	return strnlen(buf, count);
}

static ssize_t store_ecdsa_hash(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	ei[ECDSA_CUST].hlen = parse_input(buf, 0, ei[ECDSA_CUST].h, count) / 2;
	return strnlen(buf, count);
}

static ssize_t store_ecdsa_rndm(struct device *d,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	parse_input(buf, 0, ei[ECDSA_CUST].k, count);
	return strnlen(buf, count);
}

static DEVICE_ATTR(pka_mode, 0220,
		NULL, store_pka_mode);

static DEVICE_ATTR(ecdsa_pub, 0220,
		NULL, store_ecdsa_pub);

static DEVICE_ATTR(ecdsa_priv, 0220,
		NULL, store_ecdsa_priv);

static DEVICE_ATTR(ecdsa_hash, 0220,
		NULL, store_ecdsa_hash);

static DEVICE_ATTR(ecdsa_rndm, 0220,
		NULL, store_ecdsa_rndm);

static struct attribute *pka_dev_attrs[] = {
	&dev_attr_pka_mode.attr,
	&dev_attr_ecdsa_pub.attr,
	&dev_attr_ecdsa_priv.attr,
	&dev_attr_ecdsa_hash.attr,
	&dev_attr_ecdsa_rndm.attr,
	NULL
};

static struct attribute_group pka_attr_group = {
	.name = "brcm-iproc-pka",
	.attrs = pka_dev_attrs,
};

static int pka_probe(struct platform_device *pdev)
{
	unsigned status;
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct pka_driver *pka;

	pka = devm_kzalloc(dev, sizeof(struct pka_driver),
			GFP_KERNEL);
	if (!pka)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pka->hw_reg_ctlstat = devm_ioremap_resource(dev, res);
	if (!pka->hw_reg_ctlstat) {
		dev_err(&pdev->dev, "PKA could not map ctlstat register\n");
		return -ENOMEM;
	}

	pka->hw_reg_din = pka->hw_reg_ctlstat + reg_din_offset;
	pka->hw_reg_dout = pka->hw_reg_ctlstat + reg_dout_offset;
	q_initialize_pka(pka);
	status = q_pka_hw_rd_status();
	if (status != 0x00000001) {
		dev_err(&pdev->dev, "PKA CTLSTAT is not correct.\n");
		return -ENODEV;
	}

	q_initialize_pka(pka);
	platform_set_drvdata(pdev, pka);
	status = sysfs_create_group(&pdev->dev.kobj, &pka_attr_group);
	if (status < 0)
		return status;

	dev_info(&pdev->dev, "%s: success\n", __func__);
	return 0;
}

static int pka_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &pka_attr_group);
	dev_info(&pdev->dev, "%s:success\n", __func__);
	return 0;
}

static const struct of_device_id bcm_iproc_dt_ids[] = {
	{ .compatible = "brcm,iproc-pka"},
	{ }
};
MODULE_DEVICE_TABLE(of, bcm_iproc_dt_ids);

static struct platform_driver iproc_pka_driver = {
	.probe = pka_probe,
	.remove = pka_remove,
	.driver = {
		.name = "bcm-iproc-pka",
		.of_match_table = of_match_ptr(bcm_iproc_dt_ids),
	},
};
module_platform_driver(iproc_pka_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom IPROC PKA Driver");
