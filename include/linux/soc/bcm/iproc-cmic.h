#ifndef _IPROC_CMIC_H
#define _IPROC_CMIC_H

enum iproc_dev_id {
	IPROC_DEVICE_HX4 = 0,
	IPROC_DEVICE_KT2,
	IPROC_DEVICE_HR2,
	IPROC_DEVICE_GH,
	IPROC_DEVICE_SB2,
	IPROC_DEVICE_HR3,
	IPROC_DEVICE_GH2,
	IPROC_DEVICE_WH2,
	IPROC_DEVICE_HX5,
	IPROC_DEVICE_HR4,
	IPROC_MAX_DEVICES,
};

struct iproc_cmic {
	void __iomem *base;
	int device;
	const struct sbus_ops *sbus_ops;
};

struct sbus_ops {
	int (*init)(struct iproc_cmic *cmic);
	int	(*reg32_write)(struct iproc_cmic *cmic, u32 block, u32 addr, u32 val);
	u32	(*reg32_read)(struct iproc_cmic *cmic, u32 block, u32 addr);
	int	(*reg64_write)(struct iproc_cmic *cmic, u32 block, u32 addr, u64 val);
	u64	(*reg64_read)(struct iproc_cmic *cmic, u32 block, u32 addr);
	int	(*ucmem_write)(struct iproc_cmic *cmic, u32 block, u32 *mem);
	int (*ucmem_read)(struct iproc_cmic *cmic, u32 block, u32 *mem);
};

enum cmic_block_type {
	CMIC_BLOCK_TYPE_TOP = 0,
	CMIC_BLOCK_TYPE_APM = 1,
};

extern int iproc_cmic_schan_reg32_write(u32 blk_type, u32 addr, u32 val);
extern u32 iproc_cmic_schan_reg32_read(u32 blk_type, u32 addr);
extern int iproc_cmic_schan_reg64_write(u32 blk_type, u32 addr, u64 val);
extern u64 iproc_cmic_schan_reg64_read(u32 blk_type, u32 addr);
extern int iproc_cmic_schan_ucmem_write(u32 blk_type, u32 *mem);
extern int iproc_cmic_schan_ucmem_read(u32 blk_type, u32 *mem);
extern void inline __iomem *iproc_cmic_base_get(void);

extern int xgs_iproc_cmic_init(int dev_id);

#endif /* _IPROC_CMIC_H */
