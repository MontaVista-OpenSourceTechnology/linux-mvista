/*
 * $Copyright Open Broadcom Corporation$
 */


#ifndef _XGS_IPROC_MDIO_H_
#define _XGS_IPROC_MDIO_H_

enum {
	MII_OP_MODE_READ,
	MII_OP_MODE_WRITE,
	MII_OP_MODE_MAX
};

/* iProc General Interface for mdio bus support */
struct iproc_mdiobus_data {
	/* required for cmicd mdio controller supports several buses */
	u32 phybus_num;
	u32 phybus_type;
};

/*
 * struct iproc_mdio_ctrl
 * @base: base address of cmic_common
 * @iproc_mdio_enable_reg: register addr of mdio bus enable
 * @iproc_mdio_sel_bit: bit position in register for enabling mdio bus access
 * @lock: spin lock protecting io access
 */
struct iproc_mdio_ctrl {
	void __iomem *base;
	void __iomem *iproc_mdio_enable_reg;
	u32 iproc_mdio_sel_bit;
	spinlock_t lock;
	int ref_cnt;
};

struct iproc_mdiobus_private {
	struct iproc_mdiobus_data *bus_data;
	struct iproc_mdio_ctrl *hw_ctrl;
};

#define SET_REG_FIELD(reg_value, fshift, fmask, fvalue)	\
	(reg_value) = ((reg_value) & ~((fmask) << (fshift))) |  \
		(((fvalue) & (fmask)) << (fshift))
#define ISET_REG_FIELD(reg_value, fshift, fmask, fvalue) \
		(reg_value) = (reg_value) | (((fvalue) & (fmask)) << (fshift))
#define GET_REG_FIELD(reg_value, fshift, fmask)	\
	(((reg_value) & ((fmask) << (fshift))) >> (fshift))

#define MII_OP_MAX_HALT_USEC	500

#define IPROC_MDIOBUS_TYPE_INTERNAL     0
#define IPROC_MDIOBUS_TYPE_EXTERNAL     1

#endif /* _XGS_IPROC_MDIO_H_ */
