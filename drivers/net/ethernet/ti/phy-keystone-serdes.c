/*
 * Texas Instruments Keystone SerDes driver
 * Authors: WingMan Kwok <w-kwok2@ti.com>
 *
 * This is the SerDes Phy driver for Keystone devices. This is
 * required to support PCIe RC functionality based on designware
 * PCIe hardware, gbe and 10gbe found on these devices.
 *
 * Revision History:
 *    This revision is modified for 10gbe MCSDK 3.1.3
 *
 *    3.3.0.2c
 *	- Full update based on CSL version 3.3.0.2c
 *	- This update requires the remodelling of each SerDes lane
 *	  as a separate PHY device, as opposed to each SerDes as a
 *	  separate PHY device prior to this patch.
 *
 *    1.0.0
 *	- Initial revision.
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the
 * distribution.
 *
 * Neither the name of Texas Instruments Incorporated nor the names of
 * its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <linux/module.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/random.h>
#include <linux/sizes.h>
#include <linux/platform_device.h>
#include "keystone_xserdes.h"
#include "keystone_xge_fw.h"

#define ks_debug    pr_debug
#define ks_info     pr_info
#define ks_err      pr_err
#define ks_dump     pr_info

#define KSERDES_SS_OFFSET	0x1fc0
#define MOD_VER_REG		(KSERDES_SS_OFFSET + 0x00)
#define MEM_ADR_REG		(KSERDES_SS_OFFSET + 0x04)
#define MEM_DAT_REG		(KSERDES_SS_OFFSET + 0x08)
#define MEM_DATINC_REG		(KSERDES_SS_OFFSET + 0x0c)
#define CPU_CTRL_REG		(KSERDES_SS_OFFSET + 0x10)
#define LANE_CTRL_STS_REG(x)	(KSERDES_SS_OFFSET + 0x20 + (x * 0x04))
#define LINK_LOSS_WAIT_REG	(KSERDES_SS_OFFSET + 0x30)
#define PLL_CTRL_REG		(KSERDES_SS_OFFSET + 0x34)

#define CMU0_SS_OFFSET		0x0000
#define CMU0_REG(x)		(CMU0_SS_OFFSET + x)

#define LANE0_SS_OFFSET		0x0200
#define LANEX_SS_OFFSET(x)	(LANE0_SS_OFFSET * (x + 1))
#define LANEX_REG(x, y)		(LANEX_SS_OFFSET(x) + y)

#define CML_SS_OFFSET		0x0a00
#define CML_REG(x)		(CML_SS_OFFSET + x)

#define CMU1_SS_OFFSET		0x0c00
#define CMU1_REG(x)		(CMU1_SS_OFFSET + x)

#define PCSR_OFFSET(x)		(x * 0x80)

#define PCSR_TX_CTL(x)		(PCSR_OFFSET(x) + 0x00)
#define PCSR_TX_STATUS(x)	(PCSR_OFFSET(x) + 0x04)
#define PCSR_RX_CTL(x)		(PCSR_OFFSET(x) + 0x08)
#define PCSR_RX_STATUS(x)	(PCSR_OFFSET(x) + 0x0C)

#define XGE_CTRL_OFFSET		0x0c
#define PCIE_PL_GEN2_OFFSET	0x180c

#define reg_rmw(addr, value, mask) \
	writel(((readl(addr) & (~(mask))) | (value & (mask))), (addr))

#define FINSR(base, offset, msb, lsb, val) \
	reg_rmw((base) + (offset), ((val) << (lsb)), GENMASK((msb), (lsb)))

#define FEXTR(val, msb, lsb) \
	(((val) >> (lsb)) & ((1 << ((msb) - (lsb) + 1)) - 1))

#define MOD_VER(serdes) \
	((kserdes_readl(serdes, MOD_VER_REG) >> 16) & 0xffff)

#define PHY_A(serdes) (MOD_VER(serdes) != 0x4eba)

#define FOUR_LANE(serdes) \
	((MOD_VER(serdes) == 0x4eb9) || (MOD_VER(serdes) == 0x4ebd))

#define KSERDES_MAX_LANES	4
#define OFFSET_SAMPLES		100

#define for_each_cmp(i)	\
	for (i = 1; i < MAX_CMP; i++)

#ifdef CONFIG_MACH_AGIB
#define SET_BIT(variable, offset) (variable |= (0x1 << offset))
#define UNSET_BIT(variable, offset) (variable &= (~(0x1 << offset)))
#define NEGATIVE_PARAMETER_PARSE(parameter, polarity, offset)      \
	({                                                         \
		if (parameter >> (8 * sizeof(parameter) - 1)) {    \
			parameter = ~parameter + 1;               \
			SET_BIT(polarity, offset);                 \
		} else {                                           \
			UNSET_BIT(polarity, offset);               \
		}                                                  \
	})
#endif

#define CPU_EN			BIT(31)
#define CPU_GO			BIT(30)
#define POR_EN			BIT(29)
#define CPUREG_EN		BIT(28)
#define AUTONEG_CTL		BIT(27)
#define DATASPLIT		BIT(26)
#define LNKTRN_SIG_DET		BIT(8)

#define PLL_ENABLE_1P25G	0xe0000000
#define LANE_CTRL_1P25G		0xf800f8c0
#define XFM_FLUSH_CMD		0x00009c9c

#define ANEG_LINK_CTL_10GKR_MASK	GENMASK(21, 20)
#define ANEG_LINK_CTL_1GKX_MASK		GENMASK(17, 16)
#define ANEG_LINK_CTL_1G10G_MASK \
	(ANEG_LINK_CTL_10GKR_MASK | ANEG_LINK_CTL_1GKX_MASK)

#define ANEG_1G_10G_OPT_MASK		GENMASK(7, 5)

#define SERDES_REG_INDEX		0

#define KSERDES_XFW_MEM_SIZE		SZ_64K
#define KSERDES_XFW_CONFIG_MEM_SIZE	SZ_64
#define KSERDES_XFW_NUM_PARAMS		5

#define KSERDES_XFW_CONFIG_START_ADDR \
	(KSERDES_XFW_MEM_SIZE - KSERDES_XFW_CONFIG_MEM_SIZE)

#define KSERDES_XFW_PARAM_START_ADDR \
	(KSERDES_XFW_MEM_SIZE - (KSERDES_XFW_NUM_PARAMS * 4))

#define LANE_ENABLE(sc, n) ((sc)->lane[n].enable)

/* bit mask from bit-a to bit-b inclusive */
#define MASK(msb, lsb) \
	((((msb) - (lsb)) == 31) ? 0xffffffff :  \
		((((u32)1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))

/*
 * All firmware file names end up here. List the firmware file names below.
 * Newest first. Search starts from the 0-th array entry until a firmware
 * file is found.
 */
static const char * const ks2_gbe_serdes_firmwares[] = {"ks2_gbe_serdes.bin"};
static const char * const ks2_pcie_serdes_firmwares[] = {"ks2_pcie_serdes.bin"};

#define MAX_VERSION		64
#define INIT_FW_MAGIC_1		0xfaceface
#define INIT_FW_MAGIC_2		0xcafecafe

static char *compatible_init_fw_version[] = {
	"3.3.0.2c",
	NULL,
};

struct serdes_cfg_header {
	u32 magic_1;
	char version[MAX_VERSION];
	u32 magic_2;
};

struct serdes_cfg {
	u32 ofs;
	u32 msb;
	u32 lsb;
	u32 val;
};

struct serdes_cfg cfg_156p25mhz_16bit_10p3125g_3_3_0_2c[] = {
	{0x1ff4,	17, 16,		0x03},

	{0x1ff4,	31, 29,		0x04},
	{0x1ff4,	27, 25,		0x04},

	{0x1fe0,	31, 29,		0x04},
	{0x1fe0,	15, 13,		0x04},

	{0x1fe4,	31, 29,		0x04},
	{0x1fe4,	15, 13,		0x04},

	{0x0000,	 7,  0,		0x02},
	{0x0000,	23, 16,		0x80},
	{0x0014,	 7,  0,		0x38},
	{0x0014,	15,  8,		0x38},
	{0x0060,	 7,  0,		0x38},
	{0x0060,	15,  8,		0xe4},
	{0x0060,	23, 16,		0x44},
	{0x0060,	31, 24,		0x1c},
	{0x0064,	 7,  0,		0x00},
	{0x0064,	15,  8,		0x84},
	{0x0064,	23, 16,		0xc1},
	{0x0068,	15,  8,		0x82},
	{0x0068,	23, 16,		0x07},
	{0x0068,	31, 24,		0x17},
	{0x006c,	 7,  0,		0x14},
	{0x0078,	15,  8,		0xc0},
	{0x0000,	 7,  0,		0x03},

	{0x0c00,	 7,  0,		0x02},
	{0x0c00,	23, 16,		0x03},
	{0x0c14,	 7,  0,		0x52},
	{0x0c14,	15,  8,		0x52},
	{0x0c28,	31, 24,		0x80},
	{0x0c2c,	 7,  0,		0xf6},
	{0x0c3c,	 7,  0,		0x05},
	{0x0c3c,	15,  8,		0x04},
	{0x0c3c,	31, 24,		0x04},
	{0x0c40,	23, 16,		0x80},
	{0x0c40,	31, 24,		0xc0},
	{0x0c44,	 7,  0,		0x62},
	{0x0c44,	15,  8,		0x20},
	{0x0c44,	23, 16,		0x20},
	{0x0c44,	31, 24,		0x5a},
	{0x0c48,	 7,  0,		0x24},
	{0x0c48,	15,  8,		0x04},
	{0x0c48,	23, 16,		0x04},
	{0x0c48,	31, 24,		0x40},
	{0x0c4c,	 7,  0,		0x02},
	{0x0c4c,	15,  8,		0x40},
	{0x0c50,	15,  8,		0x1c},
	{0x0c50,	31, 24,		0x19},
	{0x0c54,	15,  8,		0x21},
	{0x0c58,	 7,  0,		0x60},
	{0x0c60,	 7,  0,		0x7c},
	{0x0c60,	15,  8,		0x1e},
	{0x0c60,	23, 16,		0x13},
	{0x0c60,	31, 24,		0x80},
	{0x0c64,	 7,  0,		0x02},
	{0x0c64,	15,  8,		0xcb},
	{0x0c64,	31, 24,		0x84},
	{0x0c68,	15,  8,		0x82},
	{0x0c68,	23, 16,		0x07},
	{0x0c68,	31, 24,		0x17},
	{0x0c6c,	 7,  0,		0x16},
	{0x0c74,	15,  8,		0x04},
	{0x0c78,	15,  8,		0xc0},
	{0x0c00,	 7,  0,		0x03},

	{0x0204,	 7,  0,		0x80},
	{0x0208,	 7,  0,		0x0d},
	{0x0208,	15,  8,		0x92},
	{0x0204,	31, 24,		0xfc},
	{0x0208,	 7,  0,		0x04},
	{0x0208,	15,  8,		0x91},
	{0x0208,	23, 16,		0x01},
	{0x0210,	31, 24,		0x1a},
	{0x0214,	 7,  0,		0x5c},
	{0x0214,	15,  8,		0x6b},
	{0x0214,	23, 16,		0x00},
	{0x0218,	 7,  0,		0x84},
	{0x0218,	23, 16,		0x80},
	{0x0218,	31, 24,		0x7a},
	{0x022c,	23, 16,		0x30},
	{0x0230,	15,  8,		0x08},
	{0x024c,	23, 16,		0x80},
	{0x0250,	31, 24,		0x30},
	{0x0260,	 7,  0,		0x02},
	{0x0264,	 7,  0,		0x57},
	{0x0268,	15,  8,		0x57},
	{0x0268,	23, 16,		0x57},
	{0x0278,	31, 24,		0xff},
	{0x0280,	 7,  0,		0x50},
	{0x0280,	23, 16,		0x50},
	{0x0284,	 7,  0,		0x15},
	{0x0284,	15,  8,		0x1f},
	{0x028c,	15,  8,		0x6f},
	{0x0294,	15,  8,		0x00},
	{0x0294,	23, 16,		0x00},
	{0x0294,	31, 24,		0x01},
	{0x0298,	 7,  0,		0x41},
	{0x0298,	15,  8,		0x26},
	{0x0298,	31, 24,		0x00},
	{0x029c,	 7,  0,		0x03},
	{0x02a4,	 7,  0,		0x13},
	{0x02a4,	15,  8,		0x0f},
	{0x02a8,	15,  8,		0xb6},
	{0x02a8,	23, 16,		0x01},
	{0x0380,	 7,  0,		0x30},
	{0x03c0,	15,  8,		0x02},
	{0x03cc,	 7,  0,		0x18},
	{0x03cc,	 7,  0,		0x00},
	{0x0380,	 4,  4,		0x00},
	{0x03c0,	 9,  9,		0x00},

	{0x0404,	 7,  0,		0x80},
	{0x0408,	 7,  0,		0x0d},
	{0x0408,	15,  8,		0x92},
	{0x0404,	31, 24,		0xfc},
	{0x0408,	 7,  0,		0x04},
	{0x0408,	15,  8,		0x91},
	{0x0408,	23, 16,		0x01},
	{0x0410,	31, 24,		0x1a},
	{0x0414,	 7,  0,		0x5c},
	{0x0414,	15,  8,		0x6b},
	{0x0414,	23, 16,		0x00},
	{0x0418,	 7,  0,		0x84},
	{0x0418,	23, 16,		0x80},
	{0x0418,	31, 24,		0x7a},
	{0x042c,	23, 16,		0x30},
	{0x0430,	15,  8,		0x08},
	{0x044c,	23, 16,		0x80},
	{0x0450,	31, 24,		0x30},
	{0x0460,	 7,  0,		0x02},
	{0x0464,	 7,  0,		0x57},
	{0x0468,	15,  8,		0x57},
	{0x0468,	23, 16,		0x57},
	{0x0478,	31, 24,		0xff},
	{0x0480,	 7,  0,		0x50},
	{0x0480,	23, 16,		0x50},
	{0x0484,	 7,  0,		0x15},
	{0x0484,	15,  8,		0x1f},
	{0x048c,	15,  8,		0x6f},
	{0x0494,	15,  8,		0x00},
	{0x0494,	23, 16,		0x00},
	{0x0494,	31, 24,		0x01},
	{0x0498,	 7,  0,		0x41},
	{0x0498,	15,  8,		0x26},
	{0x0498,	31, 24,		0x00},
	{0x049c,	 7,  0,		0x03},
	{0x04a4,	 7,  0,		0x13},
	{0x04a4,	15,  8,		0x0f},
	{0x04a8,	15,  8,		0xb6},
	{0x04a8,	23, 16,		0x01},
	{0x0580,	 7,  0,		0x30},
	{0x05c0,	15,  8,		0x02},
	{0x05cc,	 7,  0,		0x18},
	{0x05cc,	 7,  0,		0x00},
	{0x0580,	 4,  4,		0x00},
	{0x05c0,	 9,  9,		0x00},

	{0x0a00,	15,  8,		0x08},
	{0x0a84,	 7,  0,		0x00},
	{0x0a8c,	23, 16,		0x13},
	{0x0a90,	23, 16,		0xa0},
	{0x0a90,	31, 24,		0x77},
	{0x0a94,	 7,  0,		0x77},
	{0x0a94,	15,  8,		0x77},
	{0x0b08,	23, 16,		0x04},
	{0x0b08,	31, 24,		0x00},
	{0x0b0c,	 7,  0,		0x00},
	{0x0b0c,	15,  8,		0x00},
	{0x0b0c,	23, 16,		0x0f},
	{0x0b10,	31, 24,		0xbe},
	{0x0b14,	 7,  0,		0xff},
	{0x0b18,	 7,  0,		0x14},
	{0x0b5c,	23, 16,		0x1b},
	{0x0b5c,	31, 24,		0x98},
	{0x0b64,	15,  8,		0x11},
	{0x0b78,	15,  8,		0x0c},
	{0x0abc,	31, 24,		0xe0},
	{0x0ac0,	 7,  0,		0x8b},
};

static inline int next_enable_lane(struct kserdes_config *sc, int i)
{
	int j = i;

	while (++j < sc->lanes) {
		if (sc->lane[j].enable)
			return j;
	}
	return j;
}

#define for_each_lane(sc, i) \
	for (i = 0; i < (sc)->lanes; i++)

#define for_each_enable_lane(sc, i) \
	for (i = -1; i = next_enable_lane(sc, i), i < sc->lanes; )

static inline int xregmap_update_bits(void __iomem *base, unsigned int reg,
				     unsigned int mask, unsigned int val)
{
	reg_rmw(base + reg, val, mask);
	return 0;
}

static inline int xregmap_read(void __iomem *base, unsigned int reg, u32 *val)
{
	*val = readl(base + reg);
	return 0;
}

static inline u32 kserdes_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline void kserdes_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static void kserdes_do_config(void __iomem *base,
			      struct serdes_cfg *cfg, u32 size)
{
	u32 i;

	for (i = 0; i < size; i++)
		FINSR(base, cfg[i].ofs, cfg[i].msb, cfg[i].lsb, cfg[i].val);
}

static bool is_init_fw_compatible(struct kserdes_config *sc,
				  struct serdes_cfg_header *hdr)
{
	int i = 0;

	if ((hdr->magic_1 != INIT_FW_MAGIC_1) ||
	    (hdr->magic_2 != INIT_FW_MAGIC_2)) {
		dev_err(sc->dev, "incompatible fw %s\n", sc->init_fw);
		return false;
	}

	while (compatible_init_fw_version[i]) {
		if (!strcmp(compatible_init_fw_version[i], hdr->version)) {
			dev_info(sc->dev, "init fw %s: version %s\n",
				 sc->init_fw, hdr->version);
			return true;
		}

		++i;
	}

	dev_err(sc->dev, "incompatible fw %s: version %s\n",
		sc->init_fw, hdr->version);
	return false;
}

static int kserdes_load_init_fw(struct kserdes_config *sc,
				const char * const *a_firmwares,
				int n_firmwares)
{
	const struct firmware *fw;
	bool found = false;
	int ret, i;
	struct serdes_cfg_header hdr;
	int hdr_sz;

	for (i = 0; i < n_firmwares; i++) {
		if (a_firmwares[i]) {
			ret = request_firmware(&fw, a_firmwares[i], sc->dev);
			if (!ret) {
				found = true;
				break;
			}
		}
	}

	if (!found) {
		dev_err(sc->dev, "can't get any serdes init fw");
		return -ENODEV;
	}

	sc->init_fw = a_firmwares[i];

	memcpy((void *)&hdr, fw->data, sizeof(hdr));
	hdr_sz = sizeof(hdr);
	hdr.version[MAX_VERSION - 1] = 0;
	if (!is_init_fw_compatible(sc, &hdr))
		return -EINVAL;

	sc->init_cfg = devm_kzalloc(sc->dev, fw->size - hdr_sz, GFP_KERNEL);
	memcpy((void *)sc->init_cfg, fw->data + hdr_sz, fw->size - hdr_sz);
	sc->init_cfg_len = fw->size - hdr_sz;
	release_firmware(fw);

	kserdes_do_config(sc->regs, sc->init_cfg,
			  sc->init_cfg_len / sizeof(struct serdes_cfg));

	return 0;
}

static inline u32 _kserdes_read_tbus_val(void __iomem *sregs)
{
	u32 tmp;

	if (PHY_A(sregs)) {
		tmp  = ((kserdes_readl(sregs, CMU0_REG(0xec))) >> 24) & 0x0ff;
		tmp |= ((kserdes_readl(sregs, CMU0_REG(0xfc))) >> 16) & 0xf00;
	} else {
		tmp  = ((kserdes_readl(sregs, CMU0_REG(0xf8))) >> 16) & 0xfff;
	}

	return tmp;
}

static void _kserdes_write_tbus_addr(void __iomem *sregs, int select, int ofs)
{
	if (select && !FOUR_LANE(sregs))
		++select;

	if (PHY_A(sregs))
		FINSR(sregs, CMU0_REG(0x8), 31, 24, ((select << 5) + ofs));
	else
		FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((select << 8) + ofs));
}

static u32 _kserdes_read_select_tbus(void __iomem *sregs, int select, int ofs, spinlock_t *tbus_lock)
{
	u32 val;
	unsigned long flags;

	spin_lock_irqsave(tbus_lock, flags);
	_kserdes_write_tbus_addr(sregs, select, ofs);
	val = _kserdes_read_tbus_val(sregs);
	spin_unlock_irqrestore(tbus_lock, flags);

	return val;
}

static inline void kserdes_set_tx_idle(struct kserdes_config *sc, u32 lane)
{
	if (sc->phy_type != KSERDES_PHY_XGE)
		FINSR(sc->regs, LANEX_REG(lane, 0xb8), 17, 16, 3);

	FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 25, 24, 3);
	FINSR(sc->regs, LANEX_REG(lane, 0x28), 21, 20, 0);
}

static inline void kserdes_clr_tx_idle(struct kserdes_config *sc, u32 lane)
{
	if (sc->phy_type != KSERDES_PHY_XGE)
		FINSR(sc->regs, LANEX_REG(lane, 0xb8), 17, 16, 0);

	FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 25, 24, 0);
	FINSR(sc->regs, LANEX_REG(lane, 0x28), 21, 20, 0);
}

static void kserdes_set_lane_ov(struct kserdes_config *sc, u32 lane)
{
	u32 val_0, val_1, val;

	val_0 = _kserdes_read_select_tbus(sc->regs, lane + 1, 0, sc->tbus_lock);
	val_1 = _kserdes_read_select_tbus(sc->regs, lane + 1, 1, sc->tbus_lock);

	val = 0;
	val |= ((val_1 >> 9) & 0x3) << 1;
	val |= (val_0 & 0x3) << 3;
	val |= ((val_0 >> 2) & 0x1ff) << 5;
	val |= (1 << 14);
	val &= ~0x60;

	FINSR(sc->regs, LANEX_REG(lane, 0x028), 29, 15, val);
}

static inline void kserdes_assert_reset(struct kserdes_config *sc)
{
	int lane;

	for_each_enable_lane(sc, lane)
		kserdes_set_lane_ov(sc, lane);
}

static inline void kserdes_config_c1_c2_cm(struct kserdes_config *sc, u32 lane)
{
	u32 c1, c2, cm;

#ifdef CONFIG_MACH_AGIB
    u32 polarity;
#endif
	c1 = sc->lane[lane].tx_coeff.c1;
	c2 = sc->lane[lane].tx_coeff.c2;
	cm = sc->lane[lane].tx_coeff.cm;

#ifdef CONFIG_MACH_AGIB
	polarity = (kserdes_readl(sc->regs, LANEX_REG(lane, 0x78)) >> 12) & 0xf;
	NEGATIVE_PARAMETER_PARSE(c1, polarity, 2);
	NEGATIVE_PARAMETER_PARSE(c2, polarity, 3);
	NEGATIVE_PARAMETER_PARSE(cm, polarity, 0);
#endif

	if (sc->phy_type == KSERDES_PHY_XGE) {
		FINSR(sc->regs, LANEX_REG(lane, 0x8), 11,  8, (cm & 0xf));
		FINSR(sc->regs, LANEX_REG(lane, 0x8),  4,  0, (c1 & 0x1f));
		FINSR(sc->regs, LANEX_REG(lane, 0x8),  7,  5, (c2 & 0x7));
		FINSR(sc->regs, LANEX_REG(lane, 0x4),
		      18, 18, ((c2 >> 3) & 0x1));
	} else {
		FINSR(sc->regs, LANEX_REG(lane, 0x8), 15, 12, (cm & 0xf));
		FINSR(sc->regs, LANEX_REG(lane, 0x8),  4,  0, (c1 & 0x1f));
		FINSR(sc->regs, LANEX_REG(lane, 0x8), 11,  8, (c2 & 0xf));
	}

#ifdef CONFIG_MACH_AGIB
	FINSR(sc->regs, LANEX_REG(lane, 0x78), 15, 12, (polarity & 0xf));
#endif
}

static inline void kserdes_config_att_boost(struct kserdes_config *sc, u32 lane)
{
	u32 att, boost;

	att = sc->lane[lane].rx_force.att;
	boost = sc->lane[lane].rx_force.boost;

	pr_info("%s: setting att(%u) and boost(%u) for lane %u\n",
			__func__, att, boost, lane);

	if (sc->phy_type == KSERDES_PHY_XGE) {
		FINSR(sc->regs, LANEX_REG(lane, 0x98), 13, 13, 0);
		FINSR(sc->regs, LANEX_REG(lane, 0x8c), 15, 12, boost);
		FINSR(sc->regs, LANEX_REG(lane, 0x8c), 11, 8, att);
	} else {
		if (att != -1) {
			FINSR(sc->regs, CML_REG(0x84), 0, 0, 0);
			FINSR(sc->regs, CML_REG(0x8c), 24, 24, 0);
			FINSR(sc->regs, LANEX_REG(lane, 0x8c), 11, 8, att);
		}
		if (boost != -1) {
			FINSR(sc->regs, CML_REG(0x84), 1, 1, 0);
			FINSR(sc->regs, CML_REG(0x8c), 25, 25, 0);
			FINSR(sc->regs, LANEX_REG(lane, 0x8c), 15, 12, boost);
		}
	}
}

static void kserdes_set_tx_rx_fir_coeff(struct kserdes_config *sc, u32 lane)
{
	struct kserdes_tx_coeff *tc = &sc->lane[lane].tx_coeff;

	if (sc->phy_type == KSERDES_PHY_XGE) {
		FINSR(sc->regs, LANEX_REG(lane, 0x004), 29, 26, tc->att);
		FINSR(sc->regs, LANEX_REG(lane, 0x0a4), 2, 0, tc->vreg);
	} else {
		FINSR(sc->regs, LANEX_REG(lane, 0x004), 28, 25, tc->att);
		FINSR(sc->regs, LANEX_REG(lane, 0x084), 7, 5, tc->vreg);
	}

	kserdes_config_c1_c2_cm(sc, lane);

	if (sc->rx_force_enable)
		kserdes_config_att_boost(sc, lane);
}

static inline void
_kserdes_force_signal_detect_low(void __iomem *sregs, u32 lane)
{
	FINSR(sregs, LANEX_REG(lane, 0x004), 2, 1, 0x2);
}

static inline void
kserdes_force_signal_detect_low(struct kserdes_config *sc, u32 lane)
{
	_kserdes_force_signal_detect_low(sc->regs, lane);
}

static inline void
_kserdes_force_signal_detect_high(void __iomem *sregs, u32 lane)
{
	FINSR(sregs, LANEX_REG(lane, 0x004), 2, 1, 0x0);
}

static inline void
kserdes_force_signal_detect_high(struct kserdes_config *sc, u32 lane)
{
	_kserdes_force_signal_detect_high(sc->regs, lane);
}

static int kserdes_deassert_reset_poll_others(struct kserdes_config *sc)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned long time_check;
	u32 lanes_not_ok = 0;
	u32 ofs = 28;
	u32 ret, i;

	for_each_enable_lane(sc, i)
		lanes_not_ok |= BIT(i);

	if (!FOUR_LANE(sc->regs))
		ofs = 29;

	do {
		time_check = jiffies;
		for_each_enable_lane(sc, i) {
			if (!(lanes_not_ok & (1 << i)))
				continue;

			ret = kserdes_readl(sc->regs, CML_REG(0x1f8));

			if (ret & BIT(ofs + i))
				lanes_not_ok &= ~BIT(i);
		}

		if (!lanes_not_ok)
			return 0;

		if (time_after(time_check, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);
}

static int kserdes_deassert_reset_poll_pcie(struct kserdes_config *sc)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned long time_check;
	u32 lanes_not_ok = 0;
	u32 ret, i;

	for_each_enable_lane(sc, i)
		lanes_not_ok |= (1 << i);

	do {
		time_check = jiffies;
		for_each_enable_lane(sc, i) {
			if (!(lanes_not_ok & BIT(i)))
				continue;

			ret = _kserdes_read_select_tbus(sc->regs, i + 1, 0x02, sc->tbus_lock);

			if (!(ret & BIT(4)))
				lanes_not_ok &= ~BIT(i);
		}

		if (!lanes_not_ok)
			return 0;

		if (time_after(time_check, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);
}

static inline void _kserdes_lane_reset(void __iomem *serdes,
				       u32 lane, u32 reset)
{
	FINSR(serdes, LANEX_REG(lane, 0x28), 29, 29, !!reset);
}

static inline void kserdes_release_reset(struct kserdes_config *sc, u32 lane)
{
	if (sc->phy_type == KSERDES_PHY_XGE) {
		FINSR(sc->regs, LANEX_REG(lane, 0x60), 0, 0, 0x1);
	}
	_kserdes_lane_reset(sc->regs, lane, 0);
}

static int kserdes_deassert_reset(struct kserdes_config *sc, u32 poll)
{
	int ret = 0, lane;

	for_each_enable_lane(sc, lane)
		kserdes_release_reset(sc, lane);

	if (!poll)
		goto done;

	if (sc->phy_type == KSERDES_PHY_PCIE)
		ret = kserdes_deassert_reset_poll_pcie(sc);
	else
		ret = kserdes_deassert_reset_poll_others(sc);

done:
	return ret;
}

static inline void _kserdes_lane_enable(void __iomem *sregs, u32 lane)
{
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 31, 29, 0x7);
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 15, 13, 0x7);
}

static inline void _kserdes_lane_disable(void __iomem *sregs, u32 lane)
{
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 31, 29, 0x4);
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 15, 13, 0x4);
}

static inline int _kserdes_set_lane_ctrl_rate(void __iomem *sregs, u32 lane,
					      enum KSERDES_LANE_CTRL_RATE rate)
{
	u32 rate_mode;

	switch (rate) {
	case KSERDES_FULL_RATE:
		rate_mode = 0x4;
		break;
	case KSERDES_QUARTER_RATE:
		rate_mode = 0x6;
		break;
	case KSERDES_HALF_RATE:
		rate_mode = 0x5;
		break;
	default:
		return -EINVAL;
	}

	FINSR(sregs, LANE_CTRL_STS_REG(lane), 28, 26, rate_mode);
	FINSR(sregs, LANE_CTRL_STS_REG(lane), 12, 10, rate_mode);
	return 0;
}

static inline void _kserdes_set_lane_loopback(void __iomem *sregs, u32 lane,
					      enum KSERDES_PHY_TYPE phy_type)
{
	if (phy_type == KSERDES_PHY_XGE) {
		FINSR(sregs, LANEX_REG(lane, 0x0), 7, 0, 0x4);
		FINSR(sregs, LANEX_REG(lane, 0x4), 2, 1, 0x3);
	} else {
		FINSR(sregs, LANEX_REG(lane, 0x0), 31, 24, 0x40);
	}
}

static inline void _kserdes_set_wait_after(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 17, 16, 0x3);
}

static inline void _kserdes_clear_wait_after(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 17, 16, 0);
}

static inline void _kserdes_clear_lane_wait_after(void __iomem *sregs, u32 lane)
{
	FINSR(sregs, PLL_CTRL_REG, lane + 12, lane + 12, 1);
	FINSR(sregs, PLL_CTRL_REG, lane + 4, lane + 4, 1);
}

static inline void _kserdes_pll_enable(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 31, 29, 0x7);
}

static inline void _kserdes_pll2_enable(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 27, 25, 0x7);
}

static inline void _kserdes_pll_disable(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 31, 29, 0x4);
}

static inline void _kserdes_pll2_disable(void __iomem *sregs)
{
	FINSR(sregs, PLL_CTRL_REG, 27, 25, 0x4);
}

static inline u32 _kserdes_get_pll_status(void __iomem *sregs)
{
	return FEXTR(kserdes_readl(sregs, PLL_CTRL_REG), 28, 28);
}

static inline u32 _kserdes_get_pll2_status(void __iomem *sregs)
{
	return FEXTR(kserdes_readl(sregs, PLL_CTRL_REG), 24, 24);
}

static inline void kserdes_lane_enable_loopback(void __iomem *serdes, u32 lane)
{
	FINSR(serdes, LANEX_REG(lane, 0), 31, 24, 0x40);
}

static inline u32 _kserdes_get_lane_status(void __iomem *sregs, u32 lane,
					   enum KSERDES_PHY_TYPE phy_type)
{
	int d = ((phy_type == KSERDES_PHY_PCIE) ? 0 : 8);

	return FEXTR(kserdes_readl(sregs, PLL_CTRL_REG), lane + d, lane + d);
}

static u32 kserdes_get_pll_lanes_status(struct kserdes_config *sc)
{
	u32 val, i;

	val = _kserdes_get_pll_status(sc->regs);
	if (!val) {
		goto done;
	}

	if (sc->phy_type == KSERDES_PHY_XGE) {
		val = _kserdes_get_pll2_status(sc->regs);
		if (!val)
			goto done;
	}

	for_each_enable_lane(sc, i)
		val &= _kserdes_get_lane_status(sc->regs, i, sc->phy_type);

done:
	return val;
}

static int kserdes_get_status(struct kserdes_config *sc)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned long time_check;

	do {
		time_check = jiffies;
		if (kserdes_get_pll_lanes_status(sc))
			break;

		if (time_after(time_check, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);

	return 0;
}

static inline u32 _kserdes_get_tx_termination(struct kserdes_config *sc)
{
	return (_kserdes_read_select_tbus(sc->regs, 1,
					  ((sc->phy_type == KSERDES_PHY_XGE) ?
					  0x1a : 0x1b), sc->tbus_lock) & 0xff);
}

static void kserdes_set_tx_terminations(struct kserdes_config *sc, u32 term)
{
	int i;

	for_each_lane(sc, i) {
		FINSR(sc->regs, LANEX_REG(i, 0x7c), 31, 24, term);
		FINSR(sc->regs, LANEX_REG(i, 0x7c), 20, 20, 0x1);
	}
}

static void
_kserdes_write_ofs_xge(void __iomem *sregs, u32 lane, u32 cmp,
		       struct kserdes_cmp_coef_ofs *ofs)
{
	FINSR(sregs, CML_REG(0x8c), 23, 21, cmp);

	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x11);
	ofs->cmp = (_kserdes_read_tbus_val(sregs) & 0x0ff0) >> 4;

	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x11);
	ofs->coef1 = (_kserdes_read_tbus_val(sregs) & 0x000f) << 3;

	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x12);
	ofs->coef1 |= (_kserdes_read_tbus_val(sregs) & 0x0e00) >> 9;
	ofs->coef2  = (_kserdes_read_tbus_val(sregs) & 0x01f8) >> 3;
	ofs->coef3  = (_kserdes_read_tbus_val(sregs) & 0x0007) << 3;

	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x13);
	ofs->coef3 |= (_kserdes_read_tbus_val(sregs) & 0x0e00) >> 9;
	ofs->coef4  = (_kserdes_read_tbus_val(sregs) & 0x01f8) >> 3;
	ofs->coef5  = (_kserdes_read_tbus_val(sregs) & 0x0007) << 3;

	FINSR(sregs, CMU0_REG(0xfc), 26, 16, ((lane + 2) << 8) + 0x14);
	ofs->coef5 |= (_kserdes_read_tbus_val(sregs) & 0x0e00) >> 9;
}

static void kserdes_add_ofs_xge(struct kserdes_config *sc,
				struct kserdes_ofs *sofs)
{
	struct kserdes_cmp_coef_ofs *ctofs;
	struct kserdes_cmp_coef_ofs sample;
	struct kserdes_lane_ofs *lofs;
	u32 lane, cmp;

	for_each_enable_lane(sc, lane) {
		lofs = &sofs->lane_ofs[lane];
		for_each_cmp(cmp) {
			ctofs = &lofs->ct_ofs[cmp];

			_kserdes_write_ofs_xge(sc->regs, lane, cmp, &sample);

			ctofs->cmp  += sample.cmp;
			ctofs->coef1 += sample.coef1;
			ctofs->coef2 += sample.coef2;
			ctofs->coef3 += sample.coef3;
			ctofs->coef4 += sample.coef4;
			ctofs->coef5 += sample.coef5;
		}
	}
}

static void
kserdes_get_cmp_coef_ofs_non_xge(void __iomem *sregs, u32 lane, u32 cmp,
				 struct kserdes_cmp_coef_ofs *ofs)
{
	FINSR(sregs, CML_REG(0x8c), 23, 21, cmp);
	FINSR(sregs, CMU0_REG(0x8), 31, 24, ((lane + 1) << 5) + 0x12);
	ofs->cmp = (_kserdes_read_tbus_val(sregs) & 0x0ff0) >> 4;
}

static void kserdes_add_ofs_non_xge(struct kserdes_config *sc,
				    struct kserdes_ofs *sofs)
{
	struct kserdes_cmp_coef_ofs *ctofs;
	struct kserdes_cmp_coef_ofs sample;
	struct kserdes_lane_ofs *lofs;
	u32 lane, cmp;

	for_each_enable_lane(sc, lane) {
		lofs = &sofs->lane_ofs[lane];
		for_each_cmp(cmp) {
			ctofs = &lofs->ct_ofs[cmp];

			kserdes_get_cmp_coef_ofs_non_xge(sc->regs, lane,
							 cmp, &sample);

			ctofs->cmp  += sample.cmp;
		}
	}
}

static void kserdes_get_average_ofs(struct kserdes_config *sc, u32 samples,
				    struct kserdes_ofs *sofs)
{
	struct kserdes_cmp_coef_ofs *ctofs;
	struct kserdes_lane_ofs *lofs;
	u32 i, lane, cmp;
	int ret;

	memset(sofs, 0, sizeof(*sofs));

	for (i = 0; i < samples; i++) {
		kserdes_assert_reset(sc);
		ret = kserdes_deassert_reset(sc, 1);
		if (ret) {
			dev_err(sc->dev,
				"kserdes_get_average_ofs: reset failed %d\n",
				ret);
			return;
		}

		if (sc->phy_type == KSERDES_PHY_XGE)
			kserdes_add_ofs_xge(sc, sofs);
		else
			kserdes_add_ofs_non_xge(sc, sofs);
	}

	for_each_enable_lane(sc, lane) {
		lofs = &sofs->lane_ofs[lane];
		for_each_cmp(cmp) {
			ctofs = &lofs->ct_ofs[cmp];
			if (sc->phy_type == KSERDES_PHY_XGE) {
				ctofs->cmp  /= samples;
				ctofs->coef1 /= samples;
				ctofs->coef2 /= samples;
				ctofs->coef3 /= samples;
				ctofs->coef4 /= samples;
				ctofs->coef5 /= samples;
			} else {
				ctofs->cmp  /= samples;
			}
		}
	}
}

static void _kserdes_set_ofs(void __iomem *sregs, u32 lane, u32 cmp,
			     struct kserdes_cmp_coef_ofs *ofs)
{
	FINSR(sregs, CML_REG(0xf0), 27, 26, (lane + 1));
	FINSR(sregs, CML_REG(0x98), 24, 24, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x2c), 2, 2, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x30), 7, 5, cmp);
	FINSR(sregs, LANEX_REG(lane, 0x5c), 31, 31, 0x1);
	FINSR(sregs, CML_REG(0x9c), 7, 0, ofs->cmp);
	FINSR(sregs, LANEX_REG(lane, 0x58), 30, 24, ofs->coef1);
	FINSR(sregs, LANEX_REG(lane, 0x5c),  5,  0, ofs->coef2);
	FINSR(sregs, LANEX_REG(lane, 0x5c), 13,  8, ofs->coef3);
	FINSR(sregs, LANEX_REG(lane, 0x5c), 21, 16, ofs->coef4);
	FINSR(sregs, LANEX_REG(lane, 0x5c), 29, 24, ofs->coef5);

	FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x0);

	FINSR(sregs, CML_REG(0x98), 24, 24, 0x0);
	FINSR(sregs, LANEX_REG(lane, 0x2c), 2, 2, 0x0);
	FINSR(sregs, LANEX_REG(lane, 0x5c), 31, 31, 0x0);
}

static inline void _kserdes_set_cmp_ofs_phyb(void __iomem *sregs, u32 lane,
					     u32 cmp, u32 cmp_ofs)
{
	FINSR(sregs, LANEX_REG(lane, 0x58), 18, 18, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x4c), 5, 2, (0x1 << (cmp - 1)));
	FINSR(sregs, LANEX_REG(lane, 0x48), 24, 17, cmp_ofs);
	FINSR(sregs, LANEX_REG(lane, 0x48), 29, 29, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x48), 29, 29, 0x0);
	FINSR(sregs, LANEX_REG(lane, 0x58), 18, 18, 0x0);
}

static inline void _kserdes_set_coef_ofs(void __iomem *sregs, u32 lane,
					 u32 coef, u32 width, u32 coef_ofs)
{
	FINSR(sregs, LANEX_REG(lane, 0x58), 23, 19, (0x1 << (coef - 1)));
	FINSR(sregs, LANEX_REG(lane, 0x48), 17 + (width - 1), 17, coef_ofs);
	FINSR(sregs, LANEX_REG(lane, 0x48), 29, 29, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x48), 29, 29, 0x0);
}

static void _kserdes_set_ofs_phyb(void __iomem *sregs, u32 lane, u32 cmp,
				  struct kserdes_cmp_coef_ofs *ofs)
{
	FINSR(sregs, LANEX_REG(lane, 0x58), 16, 16, 0x1);
	FINSR(sregs, LANEX_REG(lane, 0x48), 16, 16, 0x1);

	_kserdes_set_cmp_ofs_phyb(sregs, lane, cmp, ofs->cmp);

	FINSR(sregs, LANEX_REG(lane, 0x58), 17, 17, 0x1);

	_kserdes_set_coef_ofs(sregs, lane, 1, 7, ofs->coef1);
	_kserdes_set_coef_ofs(sregs, lane, 2, 6, ofs->coef2);
	_kserdes_set_coef_ofs(sregs, lane, 3, 6, ofs->coef3);
	_kserdes_set_coef_ofs(sregs, lane, 4, 6, ofs->coef4);
	_kserdes_set_coef_ofs(sregs, lane, 5, 6, ofs->coef5);

	FINSR(sregs, LANEX_REG(lane, 0x58), 16, 16, 0x0);
	FINSR(sregs, LANEX_REG(lane, 0x48), 16, 16, 0x0);
	FINSR(sregs, LANEX_REG(lane, 0x58), 18, 18, 0x0);
	FINSR(sregs, LANEX_REG(lane, 0x58), 17, 17, 0x0);
}

static void kserdes_set_ofs_xge(struct kserdes_config *sc,
				struct kserdes_ofs *sofs)
{
	struct kserdes_cmp_coef_ofs *ctofs;
	struct kserdes_lane_ofs *lofs;
	int lane, cmp;

	for_each_enable_lane(sc, lane) {
		lofs = &sofs->lane_ofs[lane];
		for_each_cmp(cmp) {
			ctofs = &lofs->ct_ofs[cmp];
			_kserdes_set_ofs(sc->regs, lane, cmp, ctofs);
			_kserdes_set_ofs_phyb(sc->regs, lane, cmp, ctofs);
		}
	}
}

static void kserdes_set_ofs_non_xge(struct kserdes_config *sc,
				    struct kserdes_ofs *sofs)
{
	struct kserdes_cmp_coef_ofs *ctofs;
	struct kserdes_lane_ofs *lofs;
	u32 lane, cmp;

	for_each_enable_lane(sc, lane) {
		lofs = &sofs->lane_ofs[lane];
		for_each_cmp(cmp) {
			ctofs = &lofs->ct_ofs[cmp];
			_kserdes_set_ofs(sc->regs, lane, cmp, ctofs);
		}
	}
}

static void kserdes_set_average_ofs(struct kserdes_config *sc,
				    struct kserdes_ofs *sofs)
{
	if (sc->phy_type == KSERDES_PHY_XGE)
		kserdes_set_ofs_xge(sc, sofs);
	else
		kserdes_set_ofs_non_xge(sc, sofs);
}

static void kserdes_phyb_init_config(struct kserdes_config *sc,
				     struct kserdes_ofs *sofs)
{
	int lane;

	for_each_enable_lane(sc, lane)
		kserdes_force_signal_detect_low(sc, lane);

	usleep_range(10, 20);

	kserdes_get_average_ofs(sc, OFFSET_SAMPLES, sofs);
	kserdes_set_average_ofs(sc, sofs);
	usleep_range(10, 20);

	for_each_enable_lane(sc, lane)
		kserdes_force_signal_detect_high(sc, lane);

	usleep_range(10, 20);
}

static int kserdes_wait_lane_rx_valid(struct kserdes_config *sc, u32 lane)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned long time_check;
	u32 status;

	do {
		time_check = jiffies;
		status = _kserdes_read_select_tbus(sc->regs, lane + 1, 0x02, sc->tbus_lock);

		if (status & 0x20)
			return 0;

		if (time_after(time_check, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);
}

static inline void _kserdes_reset(void __iomem *sregs)
{
	FINSR(sregs, CPU_CTRL_REG, 29, 29, 0x1);
	usleep_range(10, 20);
	FINSR(sregs, CPU_CTRL_REG, 29, 29, 0x0);
	usleep_range(10, 20);
}

static inline void kserdes_xge_pll_enable(struct kserdes_config *sc)
{
	if (!sc->firmware)
		FINSR(sc->regs, CML_REG(0), 7, 0, 0x1f);

	if (sc->link_rate == KSERDES_LINK_RATE_10P3125G) {
		_kserdes_pll_enable(sc->regs);
		_kserdes_pll2_enable(sc->regs);
	} else if (sc->link_rate == KSERDES_LINK_RATE_1P25G) {
		kserdes_writel(sc->regs, PLL_CTRL_REG, PLL_ENABLE_1P25G);
	}
}

static inline void _kserdes_enable_xgmii_port_select(void __iomem *sw_regs,
						     u32 port_selects)
{
	kserdes_writel(sw_regs, XGE_CTRL_OFFSET, port_selects);
}

static inline void _kserdes_enable_xgmii_port(struct regmap *peripheral_regmap,
					      u32 port)
{
	xregmap_update_bits(peripheral_regmap, XGE_CTRL_OFFSET,
			   GENMASK(port, port), BIT(port));
}

static inline void _kserdes_reset_rx(void __iomem *sregs, int lane)
{
	_kserdes_force_signal_detect_low(sregs, lane);
	usleep_range(1000, 2000);
	_kserdes_force_signal_detect_high(sregs, lane);
}

static int kserdes_check_link_status(struct kserdes_config *sc,
				     u32 *current_lane_state,
				     u32 lanes_chk_mask,
				     u32 *lanes_up_mask)
{
	struct device *dev = sc->dev;
	u32 pcsr_rx_stat, blk_lock, blk_errs;
	int loss, i, link_up = 1;
	int ret;
	unsigned long lmask = (unsigned long)lanes_chk_mask;

	for_each_set_bit(i, &lmask, 8) {
		loss = (kserdes_readl(sc->regs, LANE_CTRL_STS_REG(i))) & 0x01;

		ret = xregmap_read(sc->pcsr_regmap, PCSR_RX_STATUS(i),
				  &pcsr_rx_stat);

		if (ret)
			return ret;

		blk_lock = (pcsr_rx_stat >> 30) & 0x1;
		blk_errs = (pcsr_rx_stat >> 16) & 0x0ff;

		if (blk_errs)
			blk_lock = 0;

		switch (current_lane_state[i]) {
		case 0:
			if (!loss && blk_lock) {
				dev_info(dev, "XGE PCSR Linked Lane: %d\n", i);
				FINSR(sc->regs, LANEX_REG(i, 0x04), 2, 1, 0);
				current_lane_state[i] = 1;
			} else if (!blk_lock) {
				dev_dbg(dev,
					"XGE PCSR Recover Lane: %d\n", i);
				_kserdes_reset_rx(sc->regs, i);
			}
			break;
		case 1:
			if (!blk_lock)
				current_lane_state[i] = 2;

			break;
		case 2:
			if (blk_lock) {
				current_lane_state[i] = 1;
			} else {
				_kserdes_reset_rx(sc->regs, i);
				current_lane_state[i] = 0;
			}
			break;
		default:
			dev_info(dev,
				 "XGE: unknown current_lane_state[%d] %d\n",
				 i, current_lane_state[i]);
			break;
		}

		if (blk_errs) {
			xregmap_update_bits(sc->pcsr_regmap, PCSR_RX_CTL(i),
					   GENMASK(7, 0), 0x19);
			xregmap_update_bits(sc->pcsr_regmap, PCSR_RX_CTL(i),
					   GENMASK(7, 0), 0x00);
		}

		if (current_lane_state[i] == 1) {
			*lanes_up_mask |= BIT(i);
		} else {
			*lanes_up_mask &= ~BIT(i);
			link_up = 0;
		}
	}

	return link_up;
}

static int kserdes_wait_link_up(struct kserdes_config *sc,
				u32 lanes_chk_mask,
				u32 *lanes_up_mask)
{
	u32 current_state[KSERDES_MAX_LANES];
	unsigned long time_check = 0;
	int i, link_up, ret = 0;

	memset(current_state, 0, sizeof(current_state));

	do {
		usleep_range(10000, 20000);
		link_up = kserdes_check_link_status(sc, current_state,
						    lanes_chk_mask,
						    lanes_up_mask);

		if (link_up)
			break;

		for_each_enable_lane(sc, i) {
			if (!(*lanes_up_mask & BIT(i))) {
				dev_dbg(sc->dev,
					"XGE: detected lane %d down\n", i);
			}
		}

		if (++time_check >= 200) {
			ret = -ETIMEDOUT;
			break;
		}

	} while (1);

	return ret;
}

static inline void kserdes_xfw_get_lane_params(struct kserdes_config *sc,
					       int lane)
{
	struct kserdes_fw_config *fw = &sc->fw;
	u32 tx_ctrl, val_0, val_1;
	u32 phy_a = PHY_A(sc->regs);

	val_0 = kserdes_readl(sc->regs, LANEX_REG(lane, 0x04));
	val_1 = kserdes_readl(sc->regs, LANEX_REG(lane, 0x08));

	tx_ctrl = ((((val_0 >> 18) & 0x1)    << 24) |
		   (((val_1 >> 0)  & 0xffff) <<  8) |
		   (((val_0 >> 24) & 0xff)   <<  0));

	if (phy_a) {
		fw->cm = (val_1 >> 12) & 0xf;
		fw->c1 = (val_1 >> 0) & 0x1f;
		fw->c2 = (val_1 >> 8) & 0xf;
	} else {
		fw->cm = (tx_ctrl >> 16) & 0xf;
		fw->c1 = (tx_ctrl >> 8) & 0x1f;
		fw->c2 = (tx_ctrl >> 13) & 0x7;
		fw->c2 = fw->c2 | (((tx_ctrl >> 24) & 0x1) << 3);
	}

	val_0 = _kserdes_read_select_tbus(sc->regs, lane + 1,
					  (phy_a ? 0x11 : 0x10), sc->tbus_lock);
	fw->attn = (val_0 >> 4) & 0xf;
	fw->boost = (val_0 >> 8) & 0xf;

	val_0 = _kserdes_read_select_tbus(sc->regs, lane + 1, 0x5, sc->tbus_lock);
	fw->dlpf = (val_0 >> 2) & 0x3ff;

	val_0 = _kserdes_read_select_tbus(sc->regs, lane + 1, 0x6, sc->tbus_lock);
	fw->cdrcal = (val_0 >> 3) & 0xff;
}

static inline void kserdes_xfw_mem_init(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	/* initialize 64B data mem */
	kserdes_writel(sc->regs, MEM_ADR_REG, 0x0000ffc0);
	/*bytes 3C, 3D, 3E, 3F */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 38, 39, 3A, 3B */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 34, 35, 36, 37 */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 30, 31, 32, 33 */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 2C, 2D, 2E, 2F */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 28, 29, 2A, 2B */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x03000303);
	/*bytes 24, 25, 26, 27 */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x03000303);
	/*bytes 20, 21, 22, 23 */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 1C, 1D, 1E, 1F */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 18, 19, 1A, 1B */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 14, 15, 16, 17 */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 10, 11, 12, 13 */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00009c9c);
	/*bytes C, D, E, F */
	kserdes_writel(sc->regs, MEM_DATINC_REG, fw->fast_train);
	/*bytes 8, 9, A, B */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	/*bytes 4, 5, 6, 7 */
	kserdes_writel(sc->regs, MEM_DATINC_REG, fw->lane_seeds);
	/*bytes 0, 1, 2, 3 */
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00e4e400);
}

static void _kserdes_xfw_check_download(void __iomem *sregs)
{
	struct kserdes_fw_entry *ent = &(kserdes_firmware[0]);
	int a_size, i;
	u32 val, addr;

	a_size = ARRAY_SIZE(kserdes_firmware);

	for (i = 0; i < a_size; i++, ent++) {
		if (ent->reg_ofs == MEM_ADR_REG)
			kserdes_writel(sregs, MEM_ADR_REG, ent->data);
		else if (ent->reg_ofs == MEM_DATINC_REG) {
			addr = kserdes_readl(sregs, MEM_ADR_REG);
			val  = kserdes_readl(sregs, MEM_DATINC_REG);
			if (val != ent->data) {
				ks_err("diff@ %d 0x%08x: 0x%08x 0x%08x\n",
					i, addr, ent->data, val);
			}
		} else
			ks_err("unknown reg_ofs %08x\n", ent->reg_ofs);
	}
}

static int kserdes_pcie_lanes_enable(struct kserdes_config *sc)
{
	int ret, i;
	u32 lanes_enable = 0;

	for_each_enable_lane(sc, i)
		lanes_enable |= BIT(i);

	for_each_lane(sc, i) {
		kserdes_release_reset(sc, i);

		if (sc->lane[i].loopback)
			_kserdes_set_lane_loopback(sc->regs, i, sc->phy_type);
	}

	ret = kserdes_get_status(sc);
	if (ret)
		return ret;

	return lanes_enable;
}

static void kserdes_clear_wait_after(struct kserdes_config *sc,
				     unsigned long lanes_mask)
{
	u32 lane;

	if (!sc->rx_force_enable) {
		for_each_set_bit(lane, &lanes_mask, 8) {
			if (!LANE_ENABLE(sc, lane))
				continue;

			_kserdes_clear_lane_wait_after(sc->regs, lane);
		}
	} else {
		_kserdes_clear_wait_after(sc->regs);
	}
}

static int kserdes_check_lanes_status(struct kserdes_config *sc)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned long time_check;
	u32 val, i;

	do {
		time_check = jiffies;
		val = 1;

		for_each_enable_lane(sc, i)
			val &= _kserdes_get_lane_status(sc->regs, i,
							sc->phy_type);

		if (val)
			break;

		if (time_after(time_check, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);

	return 0;
}

static void kserdes_phya_init_config(struct kserdes_config *sc, u32 lane)
{
	u32 coef1val, coef2val, coef3val, coef4val, coef5val;
	void __iomem *sregs = sc->regs;
	u32 cmp, coef1_ofs;

	for_each_cmp(cmp) {
		if (!(cmp & 0x1))
			continue;

		FINSR(sregs, CML_REG(0x8c), 23, 21, cmp);

		FINSR(sregs, CMU0_REG(0x8), 31, 24, ((lane + 1) << 5) + 0x12);
		coef1_ofs = (_kserdes_read_tbus_val(sregs) & 0x000f) << 3;

		FINSR(sregs, CMU0_REG(0x8), 31, 24, ((lane + 1) << 5) + 0x13);
		coef1_ofs |= (_kserdes_read_tbus_val(sregs) & 0x0e00) >> 9;

		coef1val = coef1_ofs - 14;
		coef2val = 31;
		coef3val = 31;
		coef4val = 31;
		coef5val = 31;

		FINSR(sregs, CML_REG(0xf0), 27, 26, lane + 1);
		FINSR(sregs, LANEX_REG(lane, 0x2c), 2, 2, 0x1);
		FINSR(sregs, LANEX_REG(lane, 0x30), 7, 5, cmp);
		FINSR(sregs, LANEX_REG(lane, 0x5c), 31, 31, 0x1);

		FINSR(sregs, LANEX_REG(lane, 0x58), 30, 24, coef1val);
		FINSR(sregs, LANEX_REG(lane, 0x5c),  6,  0, coef2val);
		FINSR(sregs, LANEX_REG(lane, 0x5c), 13,  8, coef3val);
		FINSR(sregs, LANEX_REG(lane, 0x5c), 21, 16, coef4val);
		FINSR(sregs, LANEX_REG(lane, 0x5c), 29, 24, coef5val);

		FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x1);
		FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x0);

		FINSR(sregs, LANEX_REG(lane, 0x2c), 2, 2, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x5c), 31, 31, 0x0);

		FINSR(sregs, LANEX_REG(lane, 0x58), 16, 16, 0x1);
		FINSR(sregs, LANEX_REG(lane, 0x48), 16, 16, 0x1);

		FINSR(sregs, LANEX_REG(lane, 0x4c), 5, 2, (0x1 << (cmp - 1)));
		FINSR(sregs, LANEX_REG(lane, 0x58), 17, 17, 0x1);

		_kserdes_set_coef_ofs(sregs, lane, 1, 7, coef1val);
		_kserdes_set_coef_ofs(sregs, lane, 2, 6, coef2val);
		_kserdes_set_coef_ofs(sregs, lane, 3, 6, coef3val);
		_kserdes_set_coef_ofs(sregs, lane, 4, 6, coef4val);
		_kserdes_set_coef_ofs(sregs, lane, 5, 6, coef5val);

		FINSR(sregs, LANEX_REG(lane, 0x58), 16, 16, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x48), 16, 16, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x58), 17, 17, 0x0);
	}
}

static int kserdes_check_pll_status(struct kserdes_config *sc)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned long time_check;
	u32 val;

	do {
		time_check = jiffies;
		val = _kserdes_get_pll_status(sc->regs);

		if (sc->phy_type == KSERDES_PHY_XGE)
			val &= _kserdes_get_pll2_status(sc->regs);

		if (val)
			break;

		if (time_after(time_check, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);

	return 0;
}

static void kserdes_enable_common_set_lane_rate(struct kserdes_config *sc,
						u32 lane)
{
	int ret;

	ret = _kserdes_set_lane_ctrl_rate(sc->regs, lane,
					  sc->lane[lane].ctrl_rate);
	if (ret) {
		dev_err(sc->dev, "set_lane_rate FAILED: lane = %d err = %d\n",
			lane, ret);
		return;
	}

	switch (sc->phy_type) {
	case KSERDES_PHY_SGMII:
		FINSR(sc->regs, LANE_CTRL_STS_REG(lane),  7,  6, 0x3);
		FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 23, 21, 0x4);
		FINSR(sc->regs, LANE_CTRL_STS_REG(lane),  5,  3, 0x4);
		break;

	case KSERDES_PHY_XGE:
		FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 23, 21, 0x7);
		FINSR(sc->regs, LANE_CTRL_STS_REG(lane),  5,  3, 0x7);
		if (sc->link_rate == KSERDES_LINK_RATE_10P3125G) {
			FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 16, 16, 0x1);
			FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 19, 19, 0x1);
		}
		break;

	case KSERDES_PHY_PCIE:
		break;

	default:
		FINSR(sc->regs, LANE_CTRL_STS_REG(lane), 23, 21, 0x6);
		FINSR(sc->regs, LANE_CTRL_STS_REG(lane),  5,  3, 0x6);
		break;
	}

	if (sc->lane[lane].loopback)
		_kserdes_set_lane_loopback(sc->regs, lane, sc->phy_type);

	if (sc->phy_type != KSERDES_PHY_XGE) {
		FINSR(sc->regs, LANEX_REG(lane, 0x30), 11, 11, 0x1);
		FINSR(sc->regs, LANEX_REG(lane, 0x30), 13, 12, 0x0);
	}
}

static inline void kserdes_set_lane_rx_starts(struct kserdes_config *sc,
					      u32 lane)
{
	FINSR(sc->regs, LANEX_REG(lane, 0x8c), 11, 8,
	      sc->lane[lane].rx_start.att);
	FINSR(sc->regs, LANEX_REG(lane, 0x8c), 15, 12,
	      sc->lane[lane].rx_start.boost);

	FINSR(sc->regs, LANEX_REG(lane, 0x84), 27, 24,
	      sc->lane[lane].rx_start.att);
	FINSR(sc->regs, LANEX_REG(lane, 0x84), 31, 28,
	      sc->lane[lane].rx_start.boost);

	FINSR(sc->regs, LANEX_REG(lane, 0x84), 19, 16,
	      sc->lane[lane].rx_start.att);
	FINSR(sc->regs, LANEX_REG(lane, 0x84), 23, 20,
	      sc->lane[lane].rx_start.boost);
}

static void kserdes_hs_init_config(struct kserdes_config *sc)
{
	int i;

	if (sc->phy_type != KSERDES_PHY_XGE) {
		if (sc->link_rate >= KSERDES_LINK_RATE_9P8304G)
			FINSR(sc->regs, CML_REG(0xbc), 28, 24, 0x1e);
	}

	for_each_enable_lane(sc, i)
		kserdes_set_tx_idle(sc, i);

	if (sc->link_rate >= KSERDES_LINK_RATE_9P8304G) {
		if (sc->phy_type != KSERDES_PHY_XGE) {
			for_each_enable_lane(sc, i)
				kserdes_force_signal_detect_low(sc, i);

			for_each_enable_lane(sc, i)
				FINSR(sc->regs, LANEX_REG(i, 0x78),
				      30, 24, 0x7f);
		} else {
			FINSR(sc->regs, CML_REG(0x10c), 7, 0, 0xff);
		}
	}
}

static int kserdes_lanes_enable_common(struct kserdes_config *sc,
				       struct kserdes_ofs *sofs)
{
	u32 val, lane_mask = 0;
	int i, ret;

	for_each_lane(sc, i) {
		if (sc->lane[i].enable)
			lane_mask |= BIT(i);
		else
			sc->lane[i].enable = 1;
	}

	if (sc->phy_type == KSERDES_PHY_PCIE) {
		dev_err(sc->dev, "kserdes_lanes_enable_common: pcie TBD.\n");
		return -EINVAL;
	}

	kserdes_hs_init_config(sc);

	for_each_enable_lane(sc, i)
		kserdes_set_lane_rx_starts(sc, i);

	kserdes_assert_reset(sc);

	for_each_enable_lane(sc, i)
		kserdes_set_tx_rx_fir_coeff(sc, i);

	for_each_enable_lane(sc, i)
		kserdes_force_signal_detect_low(sc, i);

	ret = kserdes_deassert_reset(sc, 0);
	if (ret) {
		dev_err(sc->dev, "kserdes_deassert_reset FAILED %d\n", ret);
		return ret;
	}

	for_each_enable_lane(sc, i)
		kserdes_enable_common_set_lane_rate(sc, i);

	if (sc->phy_type == KSERDES_PHY_XGE)
		kserdes_xge_pll_enable(sc);
	else
		_kserdes_pll_enable(sc->regs);

	ret = kserdes_check_pll_status(sc);
	if (ret) {
		dev_err(sc->dev,
			"common init: check pll status FAILED %d\n", ret);
		return ret;
	}

	for_each_enable_lane(sc, i)
		_kserdes_lane_enable(sc->regs, i);

	ret = kserdes_check_lanes_status(sc);
	if (ret) {
		dev_err(sc->dev,
			"common init: check lanes status FAILED %d\n", ret);
		return ret;
	}

	usleep_range(5, 10);

	val = _kserdes_get_tx_termination(sc);

	kserdes_set_tx_terminations(sc, val);

	if (sc->phy_type == KSERDES_PHY_XGE)
		kserdes_phyb_init_config(sc, sofs);
	else if (sc->link_rate >= KSERDES_LINK_RATE_9P8304G)
		for_each_enable_lane(sc, i)
			kserdes_phya_init_config(sc, i);

	for_each_enable_lane(sc, i)
		kserdes_clr_tx_idle(sc, i);

	for_each_lane(sc, i)
		sc->lane[i].enable = (lane_mask & BIT(i)) >> i;

	return 0;
}

static inline u32 _kserdes_get_lane_sd(void __iomem *sregs, u32 lane)
{
	return FEXTR(kserdes_readl(sregs, PLL_CTRL_REG), lane, lane);
}

static int _kserdes_wait_lane_sd(void __iomem *sregs, u32 lane)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(500);
	unsigned long time_check;

	do {
		time_check = jiffies;

		if (_kserdes_get_lane_sd(sregs, lane))
			break;

		if (time_after(time_check, timeout))
			return -ETIMEDOUT;

		cpu_relax();
	} while (true);

	return 0;
}

static void kserdes_rx_att_boost_config_phyb(struct kserdes_config *sc,
					     u32 lane)
{
	u32 tbus_ofs, rxeq_init_reg_ofs, rxeq_ln_reg_ofs, rxeq_ln_force_bit;
	void __iomem *sregs = sc->regs;
	u32 att_start, att_read, boost_read;
	int ret;

	if (sc->phy_type == KSERDES_PHY_XGE) {
		tbus_ofs = 0x10;
		rxeq_init_reg_ofs = 0x9c;
		rxeq_ln_reg_ofs = 0x98;
		rxeq_ln_force_bit = 14;
	} else {
		tbus_ofs = 0x11;
		rxeq_init_reg_ofs = 0x84;
		rxeq_ln_reg_ofs = 0xac;
		rxeq_ln_force_bit = 11;
	}

	att_start = kserdes_readl(sregs, LANEX_REG(lane, 0x8c));
	att_start = (att_start >> 8) & 0xf;

	att_read = _kserdes_read_select_tbus(sregs, lane + 1, tbus_ofs, sc->tbus_lock);
	att_read = (att_read >> 4) & 0xf;

	FINSR(sregs, LANEX_REG(lane, 0x8c), 11, 8, att_read);
	FINSR(sregs, LANEX_REG(lane, rxeq_init_reg_ofs), 0, 0, 0x0);
	FINSR(sregs, CML_REG(0x8c), 24, 24, 0x0);

	FINSR(sregs, LANEX_REG(lane, rxeq_ln_reg_ofs),
	      rxeq_ln_force_bit, rxeq_ln_force_bit, 0x1);
	FINSR(sregs, LANEX_REG(lane, rxeq_ln_reg_ofs),
	      rxeq_ln_force_bit, rxeq_ln_force_bit, 0x0);

	ret = kserdes_wait_lane_rx_valid(sc, lane);
	if (ret) {
		dev_info(sc->dev, "kserdes_wait_lane_rx_valid %d FAILED: %d\n",
			lane, ret);
	}
	usleep_range(300, 600);

	boost_read = _kserdes_read_select_tbus(sregs, lane + 1, tbus_ofs, sc->tbus_lock);
	boost_read = (boost_read >> 8) & 0xf;

	if (!boost_read) {
		FINSR(sregs, LANEX_REG(lane, 0x2c),  2,  2, 0x1);
		FINSR(sregs, LANEX_REG(lane, 0x2c), 18, 12, 0x2);
		FINSR(sregs, LANEX_REG(lane, 0x2c),  9,  3, 0x1);
		FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x1);
		FINSR(sregs, LANEX_REG(lane, 0x2c), 10, 10, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x2c),  2,  2, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x2c), 18, 12, 0x0);
		FINSR(sregs, LANEX_REG(lane, 0x2c),  9,  3, 0x0);
	}

	FINSR(sregs, LANEX_REG(lane, 0x8c), 11, 8, att_start);
	FINSR(sregs, LANEX_REG(lane, rxeq_init_reg_ofs), 0, 0, 0x1);
	FINSR(sregs, CML_REG(0x8c), 24, 24, 0x1);
}

static int kserdes_set_dlev_patt_adapt(struct kserdes_config *sc,
				       u32 lane, u32 pattern,
				       struct kserdes_lane_ofs *lofs)
{
	struct kserdes_cmp_coef_ofs *ctofs = &lofs->ct_ofs[4];
	void __iomem *sregs = sc->regs;
	u32 dlevp, dlevn, dlevavg;
	int ret;

	FINSR(sregs, CML_REG(0x158), 14, 8, pattern);
	FINSR(sregs, LANEX_REG(lane, 0x98), 14, 14, 1);
	FINSR(sregs, LANEX_REG(lane, 0x98), 14, 14, 0);

	ret = kserdes_wait_lane_rx_valid(sc, lane);
	if (ret) {
		dev_info(sc->dev,
			"set dlev patt: wait_lane_rx_valid FAILED for lane %u, retval %d\n", lane, ret);
		return ret;
	}

	dlevp = _kserdes_read_select_tbus(sregs, lane + 1, 0x44, sc->tbus_lock);
	dlevp = (dlevp >> 4) & 0xff;

	dlevn = _kserdes_read_select_tbus(sregs, lane + 1, 0x45, sc->tbus_lock);
	dlevn &= 0xff;

	if (ctofs->cmp <= 120)
		dlevavg = ctofs->cmp - dlevn;
	else if (ctofs->cmp >= 134)
		dlevavg = dlevp - ctofs->cmp;
	else
		dlevavg = (dlevp - dlevn) / 2;

	return dlevavg;
}

static u32 kserdes_eye_monitor_dll_ovr(struct kserdes_config *sc,
				       u32 lane, u32 phase_num, u32 t_offset,
				       u32 phase_shift)
{
	void __iomem *sregs = sc->regs;
	u32 tbus_data, delay, partial_eye, i;
	u32 start_bin = 0, end_bin = 0;
	u32 eye_scan_errors_array[128];
	u32 error_free = 0, val_0, val_1;
	u32 max_dly = 128;
	bool not_phy_xge = (sc->phy_type != KSERDES_PHY_XGE);

	if (t_offset == 0)
		t_offset++;

	if (phase_num == 1) {
		val_0 = 0x00400000;
		val_1 = 0x00000011;
	} else {
		val_0 = 0x00800000;
		val_1 = 0x00000009;
	}
	reg_rmw(sregs + LANEX_REG(lane, 0x2c), val_0, GENMASK(23, 22));
	reg_rmw(sregs + LANEX_REG(lane, 0x30), val_1, GENMASK(4, 0));

	reg_rmw(sregs + CML_REG(0xb8), 0x00004000, GENMASK(15, 8));

	if (phase_num == 1)
		val_0 = 0xffef0000;
	else
		val_0 = 0xfff60000;

	reg_rmw(sregs + CML_REG(0xb8), val_0, GENMASK(31, 16));

	reg_rmw(sregs + CML_REG(0xbc), 0x000fffff, GENMASK(19, 0));
	tbus_data = _kserdes_read_select_tbus(sregs, lane + 1, 0x02, sc->tbus_lock);
	tbus_data = tbus_data;
	usleep_range(250, 500);

	for (i = 0; i < max_dly; i = i + t_offset) {
		reg_rmw(sregs + LANEX_REG(lane, 0x2c),
			(i & 0xff) << 24, GENMASK(31, 24));

		reg_rmw(sregs + LANEX_REG(lane, 0x30),
			phase_shift, GENMASK(1, 0));
		usleep_range(5, 10);
		reg_rmw(sregs + CML_REG(0xb8), 0x0000c000, GENMASK(15, 8));
		usleep_range(500, 1000);

		val_0 = _kserdes_read_select_tbus(sregs, lane + 1,
						  not_phy_xge ?  0x1a : 0x19, sc->tbus_lock);
		val_0 <<= 4;

		val_1 = _kserdes_read_select_tbus(sregs, lane + 1,
						  not_phy_xge ?  0x1b : 0x1a, sc->tbus_lock);
		val_1 = (val_1 >> 8) & 0xf;

		eye_scan_errors_array[i] = (val_0 | val_1);

		reg_rmw(sregs + CML_REG(0xb8), 0x00004000, GENMASK(15, 8));
	}

	partial_eye = 0;
	error_free = 0;

	for (i = 0; i < max_dly; i = i + t_offset) {
		if (i == 0) {
			if (eye_scan_errors_array[i] < 16384)
				partial_eye = 1;
		} else {
			if (eye_scan_errors_array[i] > 16384 + 3000)
				partial_eye = 0;
		}

		if ((eye_scan_errors_array[i] < 16384) &&
		    (partial_eye == 0) &&
		    (error_free  == 0)) {
			if (!((eye_scan_errors_array[i - 1] > 16384) &&
			      (eye_scan_errors_array[i + 1] > 16384))) {
				error_free = 1;
				start_bin = i;
			}
		} else if ((eye_scan_errors_array[i] > 16384) &&
			   (partial_eye == 0) &&
			   (error_free  == 1)) {
			if (!((eye_scan_errors_array[i - 1] < 16384) &&
			      (eye_scan_errors_array[i + 1] < 16384))) {
				end_bin = i;
				break;
			}
		}
	}

	delay = (end_bin - start_bin) / 4 + start_bin;
	reg_rmw(sregs + LANEX_REG(lane, 0x30), 0x00000000, GENMASK(7, 0));
	reg_rmw(sregs + LANEX_REG(lane, 0x2c), 0x00000003, GENMASK(7, 0));
	reg_rmw(sregs + CML_REG(0x98), 0x00000000, GENMASK(31, 0));
	reg_rmw(sregs + CML_REG(0xb8), 0x00000000, GENMASK(15, 14));
	reg_rmw(sregs + LANEX_REG(lane, 0x2c), 0x00000000, GENMASK(23, 22));
	return delay;
}

static void kserdes_rx_calibration_phyb(struct kserdes_config *sc, u32 lane,
					struct kserdes_lane_ofs *lofs,
					struct kserdes_lane_dlev_out *ldlevo)
{
	struct kserdes_cmp_coef_ofs *ctofs, *ctofs_temp;
	struct kserdes_lane_ofs lofs_temp;
	void __iomem *sregs = sc->regs;
	u32 att, boost, comp_no, att_start, boost_start;
	u32 delay_ovr = 0;
	int dlevavg_temp[6];

	delay_ovr = kserdes_eye_monitor_dll_ovr(sc, lane, 0, 1, 0);
	ldlevo->delay = delay_ovr;

	FINSR(sregs, CML_REG(0x164), 15, 15, 1);

	FINSR(sregs, CML_REG(0x164), 16, 16, 1);
	FINSR(sregs, CML_REG(0x164), 31, 26, (128 + delay_ovr) & 0x3f);
	FINSR(sregs, CML_REG(0x168),  2,  0, (128 + delay_ovr) >> 6);
	FINSR(sregs, LANEX_REG(lane, 0x9c), 1, 1, 0);
	FINSR(sregs, LANEX_REG(lane, 0x9c), 0, 0, 0);

	att_start = (kserdes_readl(sregs, LANEX_REG(lane, 0x8c)) >> 8) & 0xf;
	boost_start = (kserdes_readl(sregs, LANEX_REG(lane, 0x8c)) >> 12) & 0xf;

	att = _kserdes_read_select_tbus(sregs, lane + 1, 0x10, sc->tbus_lock);
	boost = (att >> 8) & 0xf;
	att = (att >> 4) & 0xf;

	FINSR(sregs, LANEX_REG(lane, 0x8c), 11, 8, att);
	FINSR(sregs, LANEX_REG(lane, 0x8c), 15, 12, boost);

	dlevavg_temp[0] = kserdes_set_dlev_patt_adapt(sc, lane, 0x71, lofs);
	dlevavg_temp[1] = kserdes_set_dlev_patt_adapt(sc, lane, 0x61, lofs);
	dlevavg_temp[2] = kserdes_set_dlev_patt_adapt(sc, lane, 0x79, lofs);
	dlevavg_temp[3] = kserdes_set_dlev_patt_adapt(sc, lane, 0x75, lofs);
	dlevavg_temp[4] = kserdes_set_dlev_patt_adapt(sc, lane, 0x73, lofs);
	dlevavg_temp[5] = kserdes_set_dlev_patt_adapt(sc, lane, 0x70, lofs);

	ldlevo->coef_vals[0] = (dlevavg_temp[0] - dlevavg_temp[1]) /  2;
	ldlevo->coef_vals[1] = (dlevavg_temp[0] - dlevavg_temp[2]) / -2;
	ldlevo->coef_vals[2] = (dlevavg_temp[0] - dlevavg_temp[3]) / -2;
	ldlevo->coef_vals[3] = (dlevavg_temp[0] - dlevavg_temp[4]) / -2;
	ldlevo->coef_vals[4] = (dlevavg_temp[0] - dlevavg_temp[5]) /  2;

	ldlevo->coef_vals[0] = ldlevo->coef_vals[0] -
					ldlevo->coef_vals[0] / 3;

	for (comp_no = 1; comp_no < 5; comp_no++) {
		ctofs = &lofs->ct_ofs[comp_no];
		ctofs_temp = &lofs_temp.ct_ofs[comp_no];

		ctofs_temp->cmp = ctofs->cmp;

		if ((comp_no == 1) || (comp_no == 3)) {
			ctofs_temp->coef1 = ldlevo->coef_vals[0] + ctofs->coef1;
			ctofs_temp->coef2 = ldlevo->coef_vals[1] + ctofs->coef2;
			ctofs_temp->coef3 = ldlevo->coef_vals[2] + ctofs->coef3;
			ctofs_temp->coef4 = ldlevo->coef_vals[3] + ctofs->coef4;
			ctofs_temp->coef5 = ldlevo->coef_vals[4] + ctofs->coef5;
		} else {
			ctofs_temp->coef1 = ctofs->coef1;
			ctofs_temp->coef2 = ctofs->coef2;
			ctofs_temp->coef3 = ctofs->coef3;
			ctofs_temp->coef4 = ctofs->coef4;
			ctofs_temp->coef5 = ctofs->coef5;
		}

		_kserdes_set_ofs(sregs, lane, comp_no, ctofs_temp);
	}

	FINSR(sregs, LANEX_REG(lane, 0x8c), 11, 8, att_start);
	FINSR(sregs, LANEX_REG(lane, 0x8c), 15, 12, boost_start);
	FINSR(sregs, LANEX_REG(lane, 0x9c), 1, 1, 1);
	FINSR(sregs, LANEX_REG(lane, 0x9c), 0, 0, 1);
}

static int kserdes_rx_boost_config_phya(struct kserdes_config *sc, u32 lane)
{
	u32 boost_read;
	int ret;
	bool phy_xge = (sc->phy_type == KSERDES_PHY_XGE);

	ret = kserdes_wait_lane_rx_valid(sc, lane);
	if (ret) {
		dev_err(sc->dev,
			"config_phya: wait_lane_rx_valid FAILED %d\n", ret);
		return ret;
	}

	boost_read = _kserdes_read_select_tbus(sc->regs, lane + 1,
					       phy_xge ?  0x10 : 0x11, sc->tbus_lock);

	boost_read = (boost_read >> 8) & 0xf;

	if (!boost_read) {
		FINSR(sc->regs, LANEX_REG(lane, 0x2c),  2,  2, 0x1);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c), 18, 12, 0x2);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c),  9,  3, 0x1);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c), 10, 10, 0x1);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c), 10, 10, 0x0);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c),  2,  2, 0x0);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c), 18, 12, 0x0);
		FINSR(sc->regs, LANEX_REG(lane, 0x2c),  9,  3, 0x0);
	}
	return 0;
}

static int kserdes_enable_lane_rx(struct kserdes_config *sc, u32 lane,
				  struct kserdes_lane_ofs *lofs,
				  struct kserdes_lane_dlev_out *ldlevo)
{
	int ret = 0;

	_kserdes_force_signal_detect_high(sc->regs, lane);

	if (!sc->rx_force_enable) {
		ret = _kserdes_wait_lane_sd(sc->regs, lane);
		if (ret) {
			dev_err(sc->dev,
				"init_lane_rx wait sd valid FAILED %d\n", ret);
			return ret;
		}

		if ((sc->phy_type == KSERDES_PHY_XGE) ||
		    (sc->link_rate >= KSERDES_LINK_RATE_5G)) {
			if (sc->lane[lane].ctrl_rate == KSERDES_FULL_RATE) {
				ret = kserdes_wait_lane_rx_valid(sc, lane);
				if (ret) {
					dev_err(sc->dev,
						"init_lane_rx wait rx valid FAILED %d\n",
					       ret);
					return ret;
				}
			}
		}

		if (sc->phy_type == KSERDES_PHY_XGE) {
			kserdes_rx_att_boost_config_phyb(sc, lane);
		} else if ((sc->link_rate >= KSERDES_LINK_RATE_5G) &&
			   (sc->lane[lane].ctrl_rate == KSERDES_FULL_RATE)) {
			kserdes_rx_boost_config_phya(sc, lane);
		}
	}

	if (sc->phy_type == KSERDES_PHY_XGE)
		kserdes_rx_calibration_phyb(sc, lane, lofs, ldlevo);

	return ret;
}

static int kserdes_recover_lane_rx(struct kserdes_config *sc, u32 lane,
				   struct kserdes_lane_ofs *lofs,
				   struct kserdes_lane_dlev_out *ldlevo)
{
	int ret;

	_kserdes_force_signal_detect_high(sc->regs, lane);

	if (!sc->rx_force_enable) {
		ret = _kserdes_wait_lane_sd(sc->regs, lane);
		if (ret) {
			dev_info(sc->dev,
				"init_lane_rx wait sd valid FAILED %d\n", ret);
			return ret;
		}
		dev_info(sc->dev, "recover_lane_rx sig detcected\n");

		if ((sc->phy_type == KSERDES_PHY_XGE) ||
		    (sc->link_rate >= KSERDES_LINK_RATE_5G)) {
			if (sc->lane[lane].ctrl_rate == KSERDES_FULL_RATE) {
				ret = kserdes_wait_lane_rx_valid(sc, lane);
				if (ret) {
					dev_err(sc->dev,
						"init_lane_rx wait rx valid FAILED %d\n",
					       ret);
					return ret;
				}
				dev_info(sc->dev, "recover_lane_rx rx valid\n");
			}
		}

		if (sc->phy_type == KSERDES_PHY_XGE) {
			kserdes_rx_att_boost_config_phyb(sc, lane);
		} else if ((sc->link_rate >= KSERDES_LINK_RATE_5G) &&
			   (sc->lane[lane].ctrl_rate == KSERDES_FULL_RATE)) {
			kserdes_rx_boost_config_phya(sc, lane);
		}
	}

	if (sc->phy_type == KSERDES_PHY_XGE)
		kserdes_rx_calibration_phyb(sc, lane, lofs, ldlevo);

	return 0;
}

static int kserdes_sgmii_init(struct kserdes_config *sc)
{
	return kserdes_load_init_fw(sc, ks2_gbe_serdes_firmwares,
				    ARRAY_SIZE(ks2_gbe_serdes_firmwares));
}

static inline void _kserdes_set_link_loss_wait(void __iomem *sregs,
					       u32 link_loss_wait)
{
	kserdes_writel(sregs, LINK_LOSS_WAIT_REG, link_loss_wait);
}

static void _kserdes_xfw_download(void __iomem *sregs)
{
	struct kserdes_fw_entry *ent = &(kserdes_firmware[0]);
	int a_size, i;

	a_size = ARRAY_SIZE(kserdes_firmware);

	for (i = 0; i < a_size; i++, ent++)
		kserdes_writel(sregs, ent->reg_ofs, ent->data);
}

static inline void _kserdes_xfw_restart_cpu(void __iomem *sregs)
{
	u32 val;

	/* place serdes in reset and allow cpu to access regs */
	val = (POR_EN | CPUREG_EN | AUTONEG_CTL | DATASPLIT);
	kserdes_writel(sregs, CPU_CTRL_REG, val);

	/* let reset propagate to uC */
	mdelay(50);

	val &= ~POR_EN;
	kserdes_writel(sregs, CPU_CTRL_REG, val);

	/* set VCO div to match firmware */
	FINSR(sregs, CMU0_REG(0x0), 23, 16, 0x80);
	/* override CMU1 pin reset */
	FINSR(sregs, CMU1_REG(0x10), 31, 24, 0x40);

	/* kick off cpu */
	val |= (CPU_EN | CPU_GO);
	kserdes_writel(sregs, CPU_CTRL_REG, val);
}

static inline void kserdes_xfw_get_params(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	u32 val, val_1, i, lanes = sc->lanes;

	val = kserdes_readl(sc->regs, PLL_CTRL_REG);
	kserdes_writel(sc->regs, MEM_ADR_REG, 0x0000ffeb);
	val_1 = kserdes_readl(sc->regs, MEM_DAT_REG);

	ks_info("Initialized KR firmware version: %x\n", val_1);
	ks_info("firmware restarted status:\n");
	ks_info("  pll_ctrl = 0x%08x", val);

	for (i = 0; i < lanes; i++) {
		if (!(BIT(i) & fw->active_lane))
			continue;

		val = kserdes_readl(sc->sw_regs, PCSR_RX_STATUS(i));
		val_1 = kserdes_readl(sc->regs, LANE_CTRL_STS_REG(i));

		/* get FW adaptation parameters from phy */
		kserdes_xfw_get_lane_params(sc, i);
		ks_info("LANE%d:\n", i);
		ks_info("  pcsr_rx_sts = 0x%08x, lane_ctrl_sts = 0x%08x\n",
				val, val_1);
		ks_info("  cm = %d, c1 = %d, c2 = %d\n",
				fw->cm, fw->c1, fw->c2);
		ks_info("  attn = %d, boost = %d, dlpf = %d, cdrcal = %d\n",
				fw->attn, fw->boost, fw->dlpf, fw->cdrcal);
	}
}

static void kserdes_xfw_auto_neg_status(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	u32 aneg_in_prog = 0, i, aneg_ctl, tmp;
	unsigned long timeout;

	/* set aneg_in_prog to lane(s) that is/are active,
	 * set to auto-negotiate, and on the 1G/10G rate.
	 */
	for (i = 0; i < sc->lanes; i++) {
		tmp = (fw->lane_config[i] & ANEG_1G_10G_OPT_MASK);

		if ((tmp == ANEG_1G_10G_OPT_MASK) &&
		    (fw->active_lane & BIT(i)))
			aneg_in_prog |= BIT(i);
	}

	if (aneg_in_prog == 0)
		return;

	timeout = jiffies + msecs_to_jiffies(5000);

	ks_info("Waiting for autonegotiated link up.\n");

	while (aneg_in_prog) {
		for (i = 0; i < sc->lanes; i++) {
			aneg_ctl = kserdes_readl(sc->regs, LANEX_REG(i, 0x1d8));
			aneg_ctl = (aneg_ctl & ANEG_LINK_CTL_1G10G_MASK);

			if ((aneg_ctl == ANEG_LINK_CTL_10GKR_MASK) ||
			    (aneg_ctl == ANEG_LINK_CTL_1GKX_MASK))
				aneg_in_prog &= ~BIT(i);
		}
		if (time_after(jiffies, timeout))
			break;
		cpu_relax();
	}

	ks_debug("Lanes auto neg completed (mask): 0x%x\n",
		~aneg_in_prog & fw->active_lane);
}

static int kserdes_xfw_get_status(struct kserdes_config *sc)
{
	u32 lanes_ok = 1;
	int i;

	for (i = 0; i < sc->lanes; i++) {
		lanes_ok &= _kserdes_get_lane_status(sc->regs, i, sc->phy_type);
		cpu_relax();
	}

	return lanes_ok;
}

static int kserdes_xfw_wait_pll_locked(struct kserdes_config *sc)
{
	unsigned long timeout;
	int ret = 0;
	u32 status;

	timeout = jiffies + msecs_to_jiffies(500);
	do {
		status = kserdes_xfw_get_status(sc);

		if (status)
			return 0;

		if (time_after(jiffies, timeout)) {
			ret = -ETIMEDOUT;
			break;
		}
		cpu_relax();
	} while (true);

	ks_info("XGE serdes not locked: time out.\n");
	return ret;
}

static int kserdes_xfw_start(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	u32 i;
	int ret = 0;

	_kserdes_pll_disable(sc->regs);
	_kserdes_pll2_disable(sc->regs);

	_kserdes_reset(sc->regs);

	for (i = 0; i < sc->lanes; i++)
		_kserdes_lane_enable(sc->regs, i);

	_kserdes_set_link_loss_wait(sc->regs, fw->link_loss_wait);

	kserdes_xge_pll_enable(sc);

	kserdes_xfw_mem_init(sc);

	_kserdes_xfw_download(sc->regs);

	_kserdes_xfw_restart_cpu(sc->regs);

	/* 10G Auto-Negotiation Handling:
	 * Wait to see if we can synchronize with other side.
	 * If it doesn't it may require an interface
	 * toggle after boot
	 */
	kserdes_xfw_auto_neg_status(sc);

	_kserdes_enable_xgmii_port_select(sc->sw_regs, MASK(sc->lanes - 1, 0));

	mdelay(100);

	ret = kserdes_xfw_wait_pll_locked(sc);
	if (ret) {
		_kserdes_xfw_check_download(sc->regs);
		return ret;
	}

	sc->fw.on = true;
	kserdes_xfw_get_params(sc);

	return ret;
}

static int kserdes_xge_init(struct kserdes_config *sc)
{
	int i;
	u32 cpu_ctrl_reg;
	unsigned int val;

	if (sc->firmware) {
		if (sc->fw.on) {
			ks_info("serdes firmware already started\n");
			return 0;
		}

		return kserdes_xfw_start(sc);
	}

	cpu_ctrl_reg = kserdes_readl(sc->regs, 0x1FD0);
	if (cpu_ctrl_reg != 0) {
		pr_err("%s: cpu_ctrl_reg = 0x%x, assuming previous mode KR\n", __func__, cpu_ctrl_reg);
	}
	sc->prev_cpu_ctrl_reg = cpu_ctrl_reg;

	/*
	 * __M: reset all programmable WIZ registers back to their default states:
	 * WIZ:
	 * 0231ffc0: 4eba3100 00000000 00000000 00000000
	 * 0231ffd0: 00000000 00000000 00000000 00000000
	 * 0231ffe0: 00000001 00000001 00000000 00000000
	 * 0231fff0: 00008000 08000000 00000000 000124f8
	 * */

	kserdes_writel(sc->regs, MEM_ADR_REG, 0x00000000);
	kserdes_writel(sc->regs, MEM_DAT_REG, 0x00000000);
	kserdes_writel(sc->regs, MEM_DATINC_REG, 0x00000000);
	kserdes_writel(sc->regs, CPU_CTRL_REG, 0x00000000);
	kserdes_writel(sc->regs, LANE_CTRL_STS_REG(0), 0x00000001);
	kserdes_writel(sc->regs, LANE_CTRL_STS_REG(1), 0x00000001);
	kserdes_writel(sc->regs, LINK_LOSS_WAIT_REG, 0x00008000);
	kserdes_writel(sc->regs, PLL_CTRL_REG, 0x08000000);

	_kserdes_pll_disable(sc->regs);
	_kserdes_pll2_disable(sc->regs);

	for_each_lane(sc, i)
		_kserdes_lane_disable(sc->regs, i);

	_kserdes_reset(sc->regs);

	/*
	 * __M: make sure SerDes is actually reset...
	 * CMU0_0 should be 0x00820802 after reset (was 0x00800803 after program).
	 * Yell if not!
	 * */
	val = kserdes_readl(sc->regs, CMU0_REG(0));
	if (val != 0x00820802)
		pr_err("%s: CMU0_0 IS NOT RESET!\n", __func__);

	kserdes_do_config(sc->regs, cfg_156p25mhz_16bit_10p3125g_3_3_0_2c,
			  ARRAY_SIZE(cfg_156p25mhz_16bit_10p3125g_3_3_0_2c));
	return 0;
}

static int kserdes_pcie_init(struct kserdes_config *sc)
{
	return kserdes_load_init_fw(sc, ks2_pcie_serdes_firmwares,
				    ARRAY_SIZE(ks2_pcie_serdes_firmwares));
}

static int kserdes_of_parse_lane(struct device *dev,
				 struct device_node *np,
				 struct kserdes_config *sc)
{
	struct kserdes_lane_config *lc;
	struct kserdes_equalizer *eq;
	struct kserdes_tx_coeff *tc;
	int lane_num, ret;

	ret = of_property_read_u32(np, "reg", &lane_num);
	if (ret) {
		dev_err(dev, "%s: Failed to parse reg\n", np->full_name);
		return -EINVAL;
	}

	if (lane_num >= sc->lanes) {
		dev_err(dev, "Invalid lane number %u\n", lane_num);
		return -EINVAL;
	}

	lc = &sc->lane[lane_num];

	lc->enable = of_device_is_available(np);
	dev_info(dev, "lane %d enabled\n", lane_num);

	if (of_property_read_u32(np, "control-rate", &lc->ctrl_rate)) {
		dev_info(dev, "use default lane control-rate: %u\n",
			 lc->ctrl_rate);
	}
	dev_info(dev, "lane control-rate: %d\n", lc->ctrl_rate);

	if (of_find_property(np, "loopback", NULL))
		lc->loopback = true;
	else
		lc->loopback = false;

	dev_info(dev, "lane loopback: %d\n", lc->loopback);

	eq = &lc->rx_start;
	if (of_property_read_u32_array(np, "rx-start", &eq->att, 2)) {
		dev_info(dev, "use default lane rx-start 0 0\n");
		eq->att = 0;
		eq->boost = 0;
	}
	dev_info(dev, "lane rx-start: %d %d\n", eq->att, eq->boost);

	eq = &lc->rx_force;
	if (of_property_read_u32_array(np, "rx-force", &eq->att, 2)) {
		dev_info(dev, "use default lane rx-force 0 0\n");
		eq->att = 0;
		eq->boost = 0;
	}
	dev_info(dev, "lane rx-force: %d %d\n", eq->att, eq->boost);

	tc = &lc->tx_coeff;
	if (of_property_read_u32_array(np, "tx-coeff", &tc->c1, 5)) {
		dev_info(dev, "use default tx-coeff 0\n");
		tc->c1 = 0;
	}
	dev_info(dev, "tx-coeff: %d %d %d %d %d\n",
		tc->c1, tc->c2, tc->cm, tc->att, tc->vreg);

	return lane_num;
}

static void kserdes_set_sgmii_defaults(struct kserdes_config *sc)
{
	int i;

	sc->link_rate		= KSERDES_LINK_RATE_1P25G;
	sc->lanes		= 4;
	sc->rx_force_enable	= false;

	for_each_lane(sc, i) {
		memset(&sc->lane[i], 0, sizeof(sc->lane[i]));
		sc->lane[i].ctrl_rate = KSERDES_QUARTER_RATE;
	}
}

static void kserdes_set_xge_defaults(struct kserdes_config *sc)
{
	int i;

	sc->link_rate		= KSERDES_LINK_RATE_10P3125G;
	sc->lanes		= 2;
	sc->rx_force_enable	= false;

	for_each_lane(sc, i) {
		memset(&sc->lane[i], 0, sizeof(sc->lane[i]));
		sc->lane[i].ctrl_rate = KSERDES_FULL_RATE;
	}
}

static void kserdes_set_pcie_defaults(struct kserdes_config *sc)
{
	int i;

	sc->link_rate		= KSERDES_LINK_RATE_5G;
	sc->lanes		= 2;
	sc->rx_force_enable	= false;

	for_each_lane(sc, i)
		memset(&sc->lane[i], 0, sizeof(sc->lane[i]));
}

static void kserdes_set_defaults(struct kserdes_config *sc,
				 enum KSERDES_PHY_TYPE phy_type)
{
	switch (phy_type) {
	case KSERDES_PHY_SGMII:
		kserdes_set_sgmii_defaults(sc);
		break;
	case KSERDES_PHY_XGE:
		kserdes_set_xge_defaults(sc);
		break;
	case KSERDES_PHY_PCIE:
		kserdes_set_pcie_defaults(sc);
		break;
	default:
		break;
	}
}

int kserdes_phy_enable_rx(struct kserdes_config *sc, u32 lane)
{
	struct kserdes_ofs *sofs = &sc->sofs;
	struct kserdes_dlev_out dlevo;
	u32 lanes_up_map = 0;
	u32 i = lane;
	int ret;

	ret = kserdes_enable_lane_rx(sc, i, &sofs->lane_ofs[i],
				     &dlevo.lane_dlev_out[i]);

	kserdes_clear_wait_after(sc, BIT(i));

	if (sc->phy_type == KSERDES_PHY_XGE) {
		_kserdes_enable_xgmii_port(sc->peripheral_regmap, i);
		kserdes_wait_link_up(sc, BIT(i), &lanes_up_map);
	}

	return 0;
}

int kserdes_phy_reset(struct kserdes_config *sc, u32 lane)
{
	struct kserdes_ofs *sofs = &sc->sofs;
	struct kserdes_dlev_out dlevo;
	u32 i = lane;
	u32 lanes_up_map = 0;
	int ret;

	if (sc->fw.on) {
		dev_dbg(sc->dev, "KR FW enabled - ignore phy reset for %u\n",
		        lane);
		return 0;
	}
	ret = kserdes_recover_lane_rx(sc, i, &sofs->lane_ofs[i],
				      &dlevo.lane_dlev_out[i]);

	kserdes_clear_wait_after(sc, BIT(i));

	_kserdes_enable_xgmii_port(sc->peripheral_regmap, i);

	kserdes_wait_link_up(sc, BIT(i), &lanes_up_map);

	dev_info(sc->dev, "phy reset: recover lane %u rx\n", i);

	return ret;
}

static void kserdes_show_fw_config(struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;

	ks_dump("fw configs:\n");
	ks_dump("  lane_configs: 0x%02x, 0x%02x\n",
		fw->lane_config[0], fw->lane_config[1]);
	ks_dump("  lnk_loss_wait: %d, lane_seeds: 0x%08x, fast_train: 0x%08x\n",
		fw->link_loss_wait, fw->lane_seeds, fw->fast_train);
}

static void kserdes_show_lane_config(struct kserdes_lane_config *lc)
{
	ks_dump("%s\n", (lc->enable ? "enable" : "disable"));
	ks_dump("ctrl_rate 0x%x\n", lc->ctrl_rate);
	ks_dump("rx_start %u %u\n",
		lc->rx_start.att, lc->rx_start.boost);
	ks_dump("rx_force %u %u\n",
		lc->rx_force.att, lc->rx_force.boost);
	ks_dump("tx_coeff %u %u %u %u %u\n",
		lc->tx_coeff.c1, lc->tx_coeff.c2, lc->tx_coeff.cm,
		lc->tx_coeff.att, lc->tx_coeff.vreg);
	ks_dump("loopback %u\n", lc->loopback);
}

static void kserdes_show_config(struct kserdes_config *sc)
{
	u32 i;

	if (!sc->debug)
		return;

	ks_dump("serdes regs 0x%p\n", sc->regs);
	if (sc->sw_regs)
		ks_dump("sw regs 0x%p\n", sc->sw_regs);
	ks_dump("clk_rate %u\n", sc->clk_rate);
	ks_dump("link_rate %u\n", sc->link_rate);
	ks_dump("phy_type %u\n", sc->phy_type);
	ks_dump("lanes %u\n", sc->lanes);

	if (sc->firmware) {
		kserdes_show_fw_config(sc);
		return;
	}

	ks_dump("rx-force-%s\n", (sc->rx_force_enable ? "enable" : "disable"));

	for (i = 0; i < sc->lanes; i++) {
		ks_dump("lane[%u]:\n", i);
		kserdes_show_lane_config(&sc->lane[i]);
	}
}

static int kserdes_xfw_get_bindings(struct device_node *node,
				struct kserdes_config *sc)
{
	struct kserdes_fw_config *fw = &sc->fw;
	struct device_node *lane;
	u32 rate, lnum;

	/* Get random lane seeds */
	get_random_bytes(&fw->lane_seeds, sizeof(u32));
	fw->lane_seeds &= 0x00ffff00;

	fw->link_loss_wait = 20000;
	/* Flush 64 bytes 0c,0d,0e,0f FAST Train */
	fw->fast_train = 0x60000000;

	/* get lane configs via DTS */
	for_each_available_child_of_node(node, lane) {
		if (of_property_read_u32(lane, "lane", &lnum))
			lnum = 0;

		/* Set active lane(s) for polling */
		fw->active_lane |= BIT(lnum);

		/* get lane rate from DTS
		 * 0=1g/10g, 1=force 1g, 2=force 10g
		 */
		of_property_read_u32(lane, "rate", &rate);
		if (rate == 0) {
			fw->lane_config[lnum] |= BIT(6);
			fw->lane_config[lnum] |= BIT(5);
		} else if (rate == 1)
			fw->lane_config[lnum] |= BIT(5);
		else if (rate == 2)
			fw->lane_config[lnum] |= BIT(6);
		else {
			/* default to 1G/10G */
			ks_err("Invalid lane rate defined. Using 1/10G.\n");
			fw->lane_config[lnum] |= BIT(6);
			fw->lane_config[lnum] |= BIT(5);
		}

		/* get lane properties from DTS */
		if (of_find_property(lane, "autonegotiate", NULL))
			fw->lane_config[lnum] |= BIT(7);

		if (of_find_property(lane, "tx_pause", NULL))
			fw->lane_config[lnum] |= BIT(4);

		if (of_find_property(lane, "rx_pause", NULL))
			fw->lane_config[lnum] |= BIT(3);

		if (of_find_property(lane, "10g_train", NULL))
			fw->lane_config[lnum] |= BIT(2);

		if (of_find_property(lane, "fec", NULL))
			fw->lane_config[lnum] |= BIT(1);
	}

	if (fw->active_lane == 0) {
		ks_err("No active serdes firmware lanes defined.");
		return -EINVAL;
	}

	ks_debug("Active serdes fw lane(s): 0x%x", fw->active_lane);

	/* Both lanes should be configured even if one is not in use, just
	 * mirror the config over in that case.
	 */
	if (fw->active_lane == 0x1 || fw->active_lane == 0x2) {
		if (fw->lane_config[0] & 0xff)
			fw->lane_config[1] = fw->lane_config[0];
		else if (fw->lane_config[1] & 0xff)
			fw->lane_config[0] = fw->lane_config[1];
	}

	return 0;
}

int kserdes_of_parse(struct device *dev,
		     struct kserdes_config *sc,
		     void __iomem *sw_ss_regs,
		     void __iomem *pcsr_port_regs,
		     struct device_node *np)
{
	struct device_node *child;
	struct device_node *fw_node;
	int ret, lane = 0;
	u32 temp[2];

	if (of_property_read_u32_array(np, "regs", (u32 *)&(temp[0]), 2)) {
		dev_err(dev, "No serdes regs defined\n");
		return -ENODEV;
	}
	sc->regs = ioremap(temp[0], temp[1]);
	if (!sc->regs) {
		dev_err(dev, "can't map serdes regs\n");
		return -ENOMEM;
	}

	dev_info(dev, "0x%08x sz=0x%x mapped to %p\n",
		temp[0], temp[1], sc->regs);

	sc->phy_type = KSERDES_PHY_XGE;

	sc->dev = dev;

	kserdes_set_defaults(sc, sc->phy_type);

	if (sc->phy_type == KSERDES_PHY_XGE) {
		sc->peripheral_regmap = sw_ss_regs;
		sc->pcsr_regmap = pcsr_port_regs;
	}

	if (of_property_read_u32(np, "link-rate-kbps", &sc->link_rate)) {
		dev_info(dev, "use default link-rate-kbps: %u\n",
			 sc->link_rate);
	}

	if (of_property_read_u32(np, "num-lanes", &sc->lanes))
		dev_info(dev, "use default num-lanes %d\n", sc->lanes);

	if (sc->lanes > KSERDES_MAX_LANES) {
		sc->lanes = KSERDES_MAX_LANES;
		dev_info(dev, "use max allowed lanes %d\n", sc->lanes);
	}

	if (of_property_read_bool(np, "rx-force-enable"))
		sc->rx_force_enable = true;
	else
		sc->rx_force_enable = false;

	fw_node = of_get_child_by_name(np, "firmware");
	if (fw_node && of_device_is_available(fw_node)) {
		dev_info(dev, "fw_node available");
		sc->firmware = 1;
		ret = kserdes_xfw_get_bindings(fw_node, sc);
		of_node_put(fw_node);
		return ret;
	} else {
		dev_info(dev, "fw_node not found or not available\n");
	}

	for_each_child_of_node(np, child) {
		if (!strcmp(child->name, "firmware"))
			continue;

		lane = kserdes_of_parse_lane(dev, child, sc);
		if (lane < 0) {
			ret = lane;
			goto err_child;
		}
	}

	return 0;
err_child:
	of_node_put(child);
	return ret;
}

static int kserdes_provider_lanes_enable_common(struct kserdes_config *sc)
{
	struct kserdes_ofs *sofs = &sc->sofs;
	unsigned long lanes_needed = 0;
	int ret, i;

	if (sc->firmware)
		return 0;

	for_each_enable_lane(sc, i)
		lanes_needed |= BIT(i);

	if (sc->phy_type == KSERDES_PHY_PCIE) {
		kserdes_pcie_lanes_enable(sc);
		return lanes_needed;
	}

	ret = kserdes_lanes_enable_common(sc, sofs);
	if (ret)
		dev_err(sc->dev, "provider lanes enable: FAILED %d\n", ret);

	return ret;
}

int kserdes_provider_init(struct kserdes_config *sc)
{
	struct device *dev = sc->dev;
	int ret;
	kserdes_show_config(sc);

	switch (sc->phy_type) {
	case KSERDES_PHY_SGMII:
		ret = kserdes_sgmii_init(sc);
		break;
	case KSERDES_PHY_XGE:
		ret = kserdes_xge_init(sc);
		break;
	case KSERDES_PHY_PCIE:
		ret = kserdes_pcie_init(sc);
		break;
	default:
		ret = -EINVAL;
	}

	if (ret < 0) {
		dev_err(dev, "serdes procider init failed %d\n", ret);
		return ret;
	}

	return kserdes_provider_lanes_enable_common(sc);
}

void kserdes_relatch_att_boost(struct kserdes_config *sc, u32 lane)
{
	u32 att, boost;
	void __iomem *sregs = sc->regs;

	att = _kserdes_read_select_tbus(sregs, lane + 1, 0x10, sc->tbus_lock);
	boost = (att >> 8) & 0xf;
	att = (att >> 4) & 0xf;
	if (boost == 0 && att == 0) {
		pr_err("%s: KERN_10GE: boost, att = %u, %u, relatching\n", __func__, boost, att);

		FINSR(sregs, LANEX_REG(lane, 0x04), 2, 1, 0x2);
		FINSR(sregs, LANEX_REG(lane, 0x98), 14, 14, 0x1);
		FINSR(sregs, LANEX_REG(lane, 0x04), 2, 1, 0x0);
	}
}
