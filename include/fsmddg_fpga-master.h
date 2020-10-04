/* FPGA master framework - the framework is supposed to be used by FPGA drivers
 *                         (like BMU/PICASSO) that would like to allow accessing
 *                         its resources (registers) by slave drivers.
 *
 * Copyright (C) 2018 Nokia
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _FSMDDG_FPGA_MASTER_H
#define _FSMDDG_FPGA_MASTER_H

enum fpgamaster_op {
	FPGAM_OP_LOCK,
	FPGAM_OP_GET_ADDR,
	FPGAM_OP_RECONFIG,
};

typedef int (*fpga_master_func_t)(enum fpgamaster_op, void *, void *);

struct regmap *fpga_master_node_to_regmap(const struct device *dev);
struct fpga_master *fpga_master_register(struct device *dev,
					struct regmap *regm,
					fpga_master_func_t f, void *priv);
int fpga_master_unregister_regmap(struct regmap *regm);
struct fpga_master *devm_fpga_master_register(struct device *dev,
					      struct regmap *regm,
					      fpga_master_func_t f, void *priv);
struct fpga_master *devm_fpga_master_register_regmap(struct device *dev,
						     struct regmap *r);
struct fpga_master *fpga_master_get(const struct device *dev);
int fpga_master_call(struct fpga_master *fpgam, int op, void *data);
#endif
