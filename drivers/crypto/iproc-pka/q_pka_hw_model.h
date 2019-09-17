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

#ifndef _Q_PKA_HW_MODEL_H_
#define _Q_PKA_HW_MODEL_H_

struct lir_cam {
	uint32_t LIR;
	uint32_t index;
	uint32_t size;
	int valid;
};

/*
 * ZQI Q_PKA_LIR_ALLOC reflects the actual
 * hardware memory that includes the scratch space
 */
#define Q_PKA_LIR_ALLOC   704
#define Q_PKA_OPQUE_ALLOC 32
#define Q_PKA_CAM_ALLOC   16

/* define the port for the PKA model to listen to */
#define Q_PKA_SOCK_PORT  5880
#define BACKLOG 10

/* PKA model CPP object */
class PKA
{
private:
	/* LIR memory */
	uint32_t *PKA_LIRMEM;
	uint32_t *lir_mem_ptr;

	/* opcode queue */
	opcode *PKA_OPQUE;
	opcode *op_rd_ptr;
	opcode *op_wr_ptr;
	int op_que_empty;
	int op_que_full;

	/* size tracking CAM */
	struct lir_cam *PKA_SCAM;
	int cam_ptr;

	uint32_t PKA_STATUS;

	/* fake IO */
	uint32_t *PKA_IO_BLOB;
	int io_size;

	/*
	 * server is the PKA hardware model,
	 * client is a firmware running on the host
	 */
	int sockfd, new_fd;
	struct hostent *host;
	struct sockaddr_in srvr_addr;
	struct sockaddr_in clnt_addr;

	/* PKA data transfer rountines */
	int model_sock_write(int sockfd, uint32_t *data, int size);
	int model_sock_read(int sockfd, uint32_t *data, int size);
	int model_recv_sequence(int sockfd, int size);
	void model_dispatch_opcode();
	int model_output_lir(int sockfd);

	/* PKA OPCODE processing routines */
	int get_lir_size(uint32_t lir);
	/* check whether two LIRs share some common physical memory segment */
	int check_lir_overlap(uint32_t LIR1, uint32_t LIR2);
	/* corrupt scratch space in the LIR */
	int corrupt_lir(uint32_t n_words);
	/* look-up the LIR in the CAM */
	int CAM_LIR_LU(uint32_t LIR, uint32_t index);

	void pka_nop(opcode *opcode);
	void pka_mt_lir(opcode *opcode);
	void pka_mt_liri(opcode *opcode);
	int pka_mf_lir(opcode *opcode);
	int pka_mf_liri(opcode *opcode);
	void pka_clir(opcode *opcode);
	void pka_slir(opcode *opcode);
	void pka_nlir(opcode *opcode);
	void pka_move(opcode *opcode);
	void pka_resize(opcode *opcode);

	void pka_modadd(opcode *opcode);
	void pka_modsub(opcode *opcode);
	void pka_modrem(opcode *opcode);
	void pka_modmul(opcode *opcode);
	void pka_modsqr(opcode *opcode);
	void pka_modexp(opcode *opcode);
	void pka_modinv(opcode *opcode);
	void pka_moddiv2(opcode *opcode);

	void pka_lcmp(opcode *opcode);
	void pka_ladd(opcode *opcode);
	void pka_lsub(opcode *opcode);
	void pka_lmul(opcode *opcode);
	void pka_lsqr(opcode *opcode);
	void pka_ldiv(opcode *opcode);
	void pka_lmul2n(opcode *opcode);
	void pka_ldiv2n(opcode *opcode);
	void pka_lmod2n(opcode *opcode);
	void pka_smul(opcode *opcode);

	void pka_pmadd(opcode *opcode);
	void pka_pmsub(opcode *opcode);
	void pka_pmmul(opcode *opcode);

	void pka_ppsel(opcode *opcode);

	void pka_mont_mul(mpz_t *mr, mpz_t ma, mpz_t mb,
					mpz_t mn, mpz_t mnp,
					uint32_t br);

public:
	PKA();
	~PKA();
	void model_reset();
	int model_activate(char *addr);
};

#endif /*_Q_PKA_HW_MODEL_H_*/
