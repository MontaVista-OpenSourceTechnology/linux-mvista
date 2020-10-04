/*
 *   This program is free software;  you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY;  without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See
 *   the GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program;  if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

/*
** Debug Build Flags
**/
/* #define AXM55XX_OUTB_DME_BBS 1 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/of_platform.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/dmapool.h>

#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>

#include "axxia-rio.h"
#include "axxia-rio-irq.h"

#define NUM_DME_PER_MBOX      (4)
/* Minimum allowed segment size (ssize) for the mailbox messages
 * Allowed values are 8, 16, 32, 64, 128, 256 */
#define MBOX_MIN_SEG_SIZE	(64)
#define SEG_SIZE_LIMITER	(MBOX_MIN_SEG_SIZE * 16)
#define ib_mbox(priv, mbox) 	rcu_pointer_handoff(priv->ib_mbox[mbox])

static DEFINE_SPINLOCK(rio_doorbell_lock);
static DEFINE_SPINLOCK(rio_rx_mbox_lock);

static int apb2ser0_read_32(void __iomem *, uint32_t, uint32_t *);
static int apb2ser0_write_32(void __iomem *, uint32_t, uint32_t);
static void link_reset_req(struct rio_mport *mport);

unsigned int axxia_dme_tmr_mode[2] = { AXXIA_IBDME_INTERRUPT_MODE,
					AXXIA_IBDME_TIMER_MODE };

static struct rio_reg_range const rio_regs_backup[] = {
	{
		.start_addr = RIO_DID_CSR,
		.access_count = 1
	},
	{
		.start_addr = RIO_COMPONENT_TAG_CSR,
		.access_count = 1
	},
	{
		.start_addr = RIO_PLTOCCSR,
		.access_count = 1
	},
	{
		.start_addr = RIO_PRTOCCSR,
		.access_count = 1
	},
	{
		.start_addr = EPC_PNADIDCSR(0),
		.access_count = 1
	},
	{
		.start_addr = RAB_CTRL,
		.access_count = 1
	},
	{
		.start_addr = AXI_TIMEOUT,
		.access_count = 1
	},
	{
		.start_addr = DME_TIMEOUT,
		.access_count = 1
	},
	{
		.start_addr = RAB_INTR_ENAB_GNRL,
		.access_count = 1
	},
	{
		.start_addr = RAB_INTR_ENAB_APIO,
		.access_count = 1
	},
	{
		.start_addr = RAB_INTR_ENAB_RPIO,
		.access_count = 1
	},
	{
		.start_addr = RAB_INTR_ENAB_IDME,
		.access_count = 1
	},
	{
		.start_addr = RAB_INTR_ENAB_ODME,
		.access_count = 1
	},
	{
		.start_addr = RAB_INTR_ENAB_MISC,
		.access_count = 1
	},
	{
		.start_addr = RAB_RPIO_CTRL,
		.access_count = 1
	},
	{
		.start_addr = RAB_APIO_CTRL,
		.access_count = 1
	},
	{
		.start_addr = RAB_APIO_AMAP_CTRL(0),
		.access_count = ((RAB_APIO_AMAP_RBAR(0xf)-RAB_APIO_AMAP_CTRL(0)) >> 2)+1
	},
	{
		.start_addr = RAB_IB_DB_CSR,
		.access_count = 1
	},
	{
		.start_addr = RAB_IB_DB_CSR,
		.access_count = 1
	},
	{
		.start_addr = RAB_OB_DME_CTRL(0),
		.access_count = ((RAB_OB_DME_DESC(0x2)-RAB_OB_DME_CTRL(0)) >> 2)+1
	},
	{
		.start_addr = RAB_OB_DME_TID_MASK,
		.access_count = 1
	},
	{
		.start_addr = RAB_IB_DME_CTRL(0),
		.access_count = ((RAB_IB_DME_DESC_ADDR(0x1f)-RAB_IB_DME_CTRL(0)) >> 2)+1
	}
};

static uint32_t const * const save_rio_reg_config(struct rio_mport const * const mport);
static uint32_t restore_rio_reg_config(struct rio_mport const * const mport, uint32_t const * const rio_reg_config);

static int axxia_timer_mode_setup(char *str)
{
	unsigned int tmr_mode[3];
	int i;
	(void)get_options(str, ARRAY_SIZE(tmr_mode), tmr_mode);
	for (i = 0; i < tmr_mode[0]; i++) {
		if (tmr_mode[i+1] > 1)
			pr_debug("RIO: Invalid param value for Timer Mode\n");
		else
			axxia_dme_tmr_mode[i] = AXXIA_IBDME_TIMER_MODE;
	}
	return 1;
}
__setup("axm_srio_tmr_mode=", axxia_timer_mode_setup);

static int axxia_int_mode_setup(char *str)
{
	unsigned int int_mode[3];
	int i;
	(void)get_options(str, ARRAY_SIZE(int_mode), int_mode);
	for (i = 0; i < int_mode[0]; i++) {
		if (int_mode[i+1] > 1)
			pr_debug("RIO: Invalid param value for Int Mode\n");
		else
			axxia_dme_tmr_mode[i] = AXXIA_IBDME_INTERRUPT_MODE;
	}
	return 1;
}
__setup("axm_srio_int_mode=", axxia_int_mode_setup);

#define AXXIA_HRTIMER_DELAY	(200 * 1000UL)
unsigned int axxia_hrtimer_delay = AXXIA_HRTIMER_DELAY;
static int __init axxia_hrtimer_setup(char *str)
{
	get_option(&str, &axxia_hrtimer_delay);
	return 1;
}
__setup("axm_srio_tmr_period=", axxia_hrtimer_setup);
static void  ib_dme_irq_handler(struct rio_irq_handler *h/*, u32 state*/);

static void fill_default_inb_dme_map(int idx, u32 *inb_map)
{
	if (idx == 0) {
		inb_map[0] = 8; inb_map[1] = 0;
		inb_map[2] = 0; inb_map[3] = 0;
		inb_map[4] = 0; inb_map[5] = 2;
		inb_map[6] = 0; inb_map[7] = 6;
		inb_map[8] = 0; inb_map[9] = 2;
		inb_map[10] = 0; inb_map[11] = 6;
		inb_map[12] = 8; inb_map[13] = 0;
		inb_map[14] = 0; inb_map[15] = 0;
	} else if (idx == 1) {
		inb_map[0] = 2; inb_map[1] = 2;
		inb_map[2] = 2; inb_map[3] = 2;
		inb_map[4] = 2; inb_map[5] = 2;
		inb_map[6] = 2; inb_map[7] = 2;
		inb_map[8] = 2; inb_map[9] = 2;
		inb_map[10] = 2; inb_map[11] = 2;
		inb_map[12] = 2; inb_map[13] = 2;
		inb_map[14] = 2; inb_map[15] = 2;
	} else
		pr_err("%s: invalid idx = %d\n", __func__, idx);

}

int rio_parse_inb_dme_map(
	struct platform_device *dev, int idx, u32 *inb_map)
{
	const u32 *cell;
	int rlen;
	u32 i, tot_dmes = 0;

	if (!dev->dev.of_node) {
		dev_err(&dev->dev, "Device OF-Node is NULL");
		return -EFAULT;
	}

	if (!of_device_is_available(dev->dev.of_node)) {
		pr_err("rio[%d]: AR[%d] stat = not available\n", 99, __LINE__);
		return -ENODEV;
	}

	for (i = 0 ; i < 16; i++)
		inb_map[i] = 0;
	cell = of_get_property(dev->dev.of_node, "inb_dmes_map", &rlen);
	if (!cell) {
		dev_err(&dev->dev,
		"%s property 'inb_dmes_map' unavailable, Using defaults\n",
				dev->dev.of_node->full_name);
		goto fill_default_return;
	} else {
		if (rlen < (16 * sizeof(int))) {
			dev_err(&dev->dev,
			"Invalid %s property 'inb_dmes_map', Using defaults\n",
				dev->dev.of_node->full_name);
			goto fill_default_return;
		} else {
			for (i = 0; i < 16; i++) {
				inb_map[i] = of_read_number((cell + i), 1);
				if (inb_map[i] > 32)
					dev_err(&dev->dev,
					"Invalid dme_map value %d for %d\n",
					inb_map[i], i);
			}
		}
	}
	for (i = 0; i < 16; i++)
		tot_dmes += inb_map[i];
	if (tot_dmes > 32) {
		dev_err(&dev->dev, "Total inb DMEs > 32, Using Defaults\n");
		goto fill_default_return;
	}
	return 0;
fill_default_return:
	fill_default_inb_dme_map(idx, inb_map);
	return 0;
}


static inline void __dme_dw_dbg(struct device *dev, struct rio_msg_dme *dme,
			u32 iout, u32 dw0, u32 dw1)
{
	int did, mb, let;
	char *io;
	char *id;
	if (dw0 & DME_DESC_DW0_ERROR_MASK) {
		did = DME_DESC_DW0_GET_DST_ID(dw0);
		let = DME_DESC_DW1_GET_LETTER(dw1);
		mb = DME_DESC_DW1_GET_MBOX(dw1);
		if (iout) {
			io = "OB";
			id = "DID";
		} else {
			io = "IB";
			id = "SID";
		}
#if defined(CONFIG_AXXIA_RIO_STAT)
		dme->desc_error_count++;
#endif
		if (dw0 & DME_DESC_DW0_RIO_ERR) {
			pr_info_ratelimited(
			"SRIO: %s:%s RIO ERR: %s = %x,Type:11,mbox=%d,let=%d\n",
			 dev_name(dev), io, id, did, mb, let);
#if defined(CONFIG_AXXIA_RIO_STAT)
			dme->desc_rio_err_count++;
#endif
		}
		if (dw0 & DME_DESC_DW0_AXI_ERR) {
			pr_info_ratelimited(
			"SRIO: %s:%s AXI ERR: %s = %x,Type:11,mbox=%d,let=%d\n",
			 dev_name(dev), io, id, did, mb, let);
#if defined(CONFIG_AXXIA_RIO_STAT)
			dme->desc_axi_err_count++;
#endif
		}
		if (dw0 & DME_DESC_DW0_TIMEOUT_ERR) {
			pr_info_ratelimited(
			"SRIO: %s:%s TIMEOUT %s = %x,Type:11,mbox=%d,let=%d\n",
			 dev_name(dev), io, id, did, mb, let);
#if defined(CONFIG_AXXIA_RIO_STAT)
			dme->desc_tmo_err_count++;
#endif
		}
	}
#if defined(CONFIG_AXXIA_RIO_STAT)
	if (dw0 & DME_DESC_DW0_DONE)
		dme->desc_done_count++;
#endif
}

#if defined(CONFIG_AXXIA_RIO_STAT)
static void reset_state_counters(struct rio_priv *priv)
{
	priv->rpio_compl_count = 0;
	priv->rpio_failed_count = 0;
	priv->apio_compl_count = 0;
	priv->apio_failed_count = 0;
	priv->rio_pw_count = 0;
	priv->rio_pw_msg_count = 0;
	priv->rio_pw_spurious = 0;
	memset(&priv->maint_local_read, 0, sizeof(struct rio_time_stat));
	memset(&priv->maint_local_write, 0, sizeof(struct rio_time_stat));
	memset(&priv->maint_remote_read, 0, sizeof(struct rio_time_stat));
	memset(&priv->maint_remote_write, 0, sizeof(struct rio_time_stat));
}
#endif /* defined(CONFIG_AXXIA_RIO_STAT) */

static void invalidate_dme_descriptors(struct rio_msg_dme *me)
{
	int i;
	u32 dw0;
	for (i = 0; i < me->entries; i++) {
		dw0 = *((u32 *)DESC_TABLE_W0_MEM(me, i));
		*((u32 *)DESC_TABLE_W0_MEM(me, i)) = (dw0 & 0xfe);
	}
}

static int dme_count_valid_desc(struct rio_msg_dme *me)
{
	u32 dw0;
	int count = 0;
	int calc = 0;
	int i;
	for (i = 0; i < me->entries; i++) {
		dw0 = *((u32 *)DESC_TABLE_W0_MEM(me, i));
		if (dw0 & DME_DESC_DW0_VALID)
			count++;
	}
	if (me->write_idx >= me->read_idx)
		calc = me->write_idx - me->read_idx;
	else
		calc = (me->write_idx + me->entries) - me->read_idx;
	if (count != calc)
		pr_warn("valid count %d calc %d widx %d ridx %d mismatch\n",
				count, calc, me->write_idx, me->read_idx);
	return count;
}

static void revalidate_dme_descriptors(struct rio_msg_dme *me, int cnt)
{
	u32 dw0, dw2_r, dw2;
	struct rio_msg_desc *desc;
	int i;
	for (i = 0; i < cnt; i++) {
		if (((me->write_idx + 1) % (me->entries)) ==
						 me->read_idx)
		pr_warn("R/W index mess up i = %d cnt = %d\n", i, cnt);
		dw0 = *((u32 *)DESC_TABLE_W0_MEM(me, me->write_idx));
		desc = &me->desc[me->write_idx];
		dw2_r = *((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx));
		dw2 = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
		dw2 = (dw2_r & 0xc0000000) | dw2;
		*((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx)) = dw2;

		dw0 |= DME_DESC_DW0_VALID;
		*((u32 *)DESC_TABLE_W0_MEM(me, me->write_idx)) = dw0;
		me->write_idx = (me->write_idx + 1) % (me->entries);
	}
}

/**
 * thrd_irq_handler - Threaded interrupt handler
 * @irq: Linux interrupt number
 * @data: Pointer to interrupt-specific data
 *
 */
static irqreturn_t thrd_irq_handler(int irq, void *data)
{
	struct rio_irq_handler *h = data;
	struct rio_priv *priv = h->data;

	/**
	 * Invoke handler callback
	 */
	h->thrd_irq_fn(h);
	if (h->irq_enab_reg_addr)
		axxia_local_config_write(priv,
					 h->irq_enab_reg_addr,
					 h->irq_state_mask);
	return IRQ_HANDLED;
}

/**
 * hw_irq_handler - RIO HW interrupt handler
 * @irq: Linux interrupt number
 * @data: Pointer to interrupt-specific data
 *
 */
static irqreturn_t hw_irq_handler(int irq, void *data)
{
	struct rio_irq_handler *h = data;
	struct rio_priv *priv = h->data;
	u32 state;
	/**
	 * Get current interrupt state and clear latched state
	 * for interrupts handled by current thread.
	 */
	axxia_local_config_read(priv, h->irq_state_reg_addr, &state);
	state &= h->irq_state_mask;

	if (state) {
		if (h->irq_enab_reg_addr)
			axxia_local_config_write(priv,
						 h->irq_enab_reg_addr,
						 0x0);
		return IRQ_WAKE_THREAD;
	}
	return IRQ_NONE;
}

static irqreturn_t hw_irq_dme_handler(int irq, void *data)
{
	struct rio_irq_handler *h;
	struct rio_priv *priv = data;

	h = &priv->ib_dme_irq;
	ib_dme_irq_handler(h);

	return IRQ_HANDLED;
}

/**
 * Caller must hold RAB lock
 */
int alloc_irq_handler(struct rio_irq_handler *h,
		     void *data,
		     const char *name)
{
	struct rio_priv *priv = data;/*mport->priv;*/
	u32 mask;
	int rc;

	if (test_and_set_bit(RIO_IRQ_ENABLED, &h->state))
		return -EBUSY;

	h->data = data;
	rc = request_threaded_irq(priv->irq_line,
				  hw_irq_handler,
				  thrd_irq_handler,
				  IRQF_TRIGGER_NONE | IRQF_SHARED | IRQF_ONESHOT,
				  name,
				  (void *)h);
	if (rc) {
		clear_bit(RIO_IRQ_ENABLED,  &h->state);
		h->data = NULL;
		return rc;
	}
	if (h->irq_enab_reg_addr) {
		axxia_local_config_read(priv, h->irq_enab_reg_addr, &mask);
		mask |= h->irq_state_mask;
		axxia_local_config_write(priv, h->irq_state_reg_addr, mask);
		axxia_local_config_write(priv, h->irq_enab_reg_addr, mask);
	}

	return rc;
}

/**
 * Caller must hold RAB lock
 */

void release_irq_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	u32 mask;

	if (test_and_clear_bit(RIO_IRQ_ENABLED, &h->state)) {
		if (h->irq_enab_reg_addr) {
			axxia_local_config_read(priv,
						h->irq_enab_reg_addr,
						&mask);
			mask &= ~h->irq_state_mask;
			axxia_local_config_write(priv,
						 h->irq_enab_reg_addr,
						 mask);
		}
		free_irq(priv->irq_line, h);
		if (h->release_fn)
			h->release_fn(h);
	}
}

/**
 * MISC Indications
 */
#if defined(CONFIG_RAPIDIO_HOTPLUG)
static void rio_port_down_notify(struct rio_mport *mport)
{
	unsigned long flags;
	struct rio_priv *priv = mport->priv;

	spin_lock_irqsave(&priv->port_lock, flags);
	if (priv->port_notify_cb)
		priv->port_notify_cb(mport);

	spin_unlock_irqrestore(&priv->port_lock, flags);
}
#else
#define rio_port_down_notify(mport)
#endif

/**
 * __port_fatal_err - Check port error state and clear latched
 *                    error state to enable detection of new events.
 *
 * @mport: Master port
 *
 * Returns:
 * 1 -- port fatal error state is detected
 * 0 -- port ok
 */
static inline void __misc_fatal(struct rio_mport *mport,
				u32 misc_state)
{
	struct rio_priv *priv = mport->priv;
	u32 amast = 0;
	u32 aslv_state = 0;
	u32 aslv_addr = 0;
	u32 escsr, iecsr;

	dev_err(priv->dev, "*************Fatal Error************\n");
	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &escsr);
	axxia_local_config_read(priv, EPC_IECSR(priv->port_ndx), &iecsr);

	/* clear latched state indications */
	/* Adding I2E to preserve idle sequence select bit which is R/w */
	axxia_local_config_write(priv, RIO_ESCSR(priv->port_ndx),
				(escsr & (RIO_ESCSR_I2E | RIO_EXCSR_WOLR)));
	dev_err(priv->dev, "port %d ESCSR(0x158) 0x%08x\n", priv->ndx, escsr);
	if (iecsr & EPC_IECSR_RETE) {
		dev_err(priv->dev, "Retry Error Threshold Exceeded\n");
		axxia_local_config_write(priv, EPC_IECSR(priv->port_ndx),
				(iecsr & EPC_IECSR_RETE));
	}
	if (misc_state & AMST_INT) {
		axxia_local_config_read(priv, RAB_AMAST_STAT, &amast);
		if (amast & RAB_AMAST_STAT_WRTO)
			dev_err(priv->dev, "AMST Write Response Timeout Error\n");
		if (amast & RAB_AMAST_STAT_RDTO)
			dev_err(priv->dev, "AMST Read Response Timeout Error\n");
		if (amast & RAB_AMAST_STAT_WRDE)
			dev_err(priv->dev, "AMST Write Decode Error\n");
		if (amast & RAB_AMAST_STAT_WRSE)
			dev_err(priv->dev, "AMST Write Slave Error\n");
		if (amast & RAB_AMAST_STAT_RDDE)
			dev_err(priv->dev, "AMST Read Decode Error\n");
		if (amast & RAB_AMAST_STAT_RDSE)
			dev_err(priv->dev, "AMST Read Slave Error\n");
		/* clear latched state */
		axxia_local_config_write(priv, RAB_AMAST_STAT, amast);
	}
	if (misc_state & ASLV_INT) {
		axxia_local_config_read(priv, RAB_ASLV_STAT_CMD,  &aslv_state);
		axxia_local_config_read(priv, RAB_ASLV_STAT_ADDR, &aslv_addr);
		if (aslv_state & RAB_ASLV_STAT_CMD_USUP) {
			dev_err(priv->dev, "AMBA Slave Unsupported Command\n");
			axxia_local_config_write(priv, RAB_ASLV_STAT_CMD,
					 (aslv_state & RAB_ASLV_STAT_CMD_USUP));
		}
	}
	if ((escsr & ESCSR_FATAL) ||
	    (iecsr & EPC_IECSR_RETE) ||
	    (misc_state & MISC_FATAL))
		rio_port_down_notify(mport);
}

/**
 * srio_sw_reset - Reset the SRIO (GRIO) module when it reaches a fatal
 *                 lockup state
 * @mport: Master port with triggered interrupt
 */
static void srio_sw_reset(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;

	/**
	 * Reset platform if port is broken
	 */
	if (priv->linkdown_reset.win) {
		uint32_t srio_control_0_reg = 0;
		uint32_t srio_read_test = 0;

		while (0 != apb2ser0_read_32((priv->linkdown_reset.win + PCIE_SRIO_PHY0_CFG_OFS),
									priv->linkdown_reset.reg_addr,
									&srio_control_0_reg))
			udelay(100);

		while (0 != apb2ser0_write_32((priv->linkdown_reset.win + PCIE_SRIO_PHY0_CFG_OFS),
									priv->linkdown_reset.reg_addr,
									(srio_control_0_reg | priv->linkdown_reset.reg_mask)))
			udelay(100);

		while (0 != apb2ser0_read_32((priv->linkdown_reset.win + PCIE_SRIO_PHY0_CFG_OFS),
									priv->linkdown_reset.reg_addr,
									&srio_read_test))
			udelay(100);

		while (0 != apb2ser0_write_32((priv->linkdown_reset.win + PCIE_SRIO_PHY0_CFG_OFS),
										priv->linkdown_reset.reg_addr,
										srio_control_0_reg))
			udelay(100);
	}
}

/**
 * pw_log_print - format string with discarded port write words
 */
static void format_pw_log(char * const dest, u32 index, u32 pw_word)
{
	char buff[12] = {'\0'};
	u8 const fmt_size = 12;

	if (0 == index) {
		strcat(dest, "\n");
		snprintf(buff, fmt_size, "0x%08x", pw_word);
	}
	else if (0 == (index % 4))
		snprintf(buff, fmt_size, "\n0x%08x", pw_word);
	else
		snprintf(buff, fmt_size, " 0x%08x", pw_word);
	strncat(dest, buff, fmt_size);
}

/**
 * PORT WRITE events
 */
/**
 * pw_irq_handler - AXXIA port write interrupt handler
 * @h: handler specific data
 * @state: PW Interrupt state
 *
 * Handles port write interrupts.
 */
static void pw_irq_handler(struct rio_irq_handler *h, u32 state)
{
	struct rio_priv *priv = h->data;
	struct rio_pw_irq *pw = priv->pw_data;
	struct rio_mport *mport = priv->mport;
	u32 pw0_mask = 0x0000ffff, pw0_val = mport->host_deviceid;
	u32 pw1_mask = 0x80817fc8, pw1_val = 0x80000000;
	int noofpw, discarded = 0;
	u32 csr;
	u32 msg_word;
	char disc_words[256] = {'\0'};

	axxia_local_config_read(priv, RAB_IB_PW_CSR, &csr);
	axxia_local_config_write(priv, RAB_INTR_STAT_MISC, PORT_WRITE_INT);
	if (pw == NULL)
		return;

	noofpw = RAB_IB_PW_NUMWORDS(csr);
	if (!(noofpw)) {
		dev_dbg(priv->dev, "PW Spurious Port Write\n");
#if defined(CONFIG_AXXIA_RIO_STAT)
		priv->rio_pw_spurious++;
#endif
		return;
	}
#if defined(CONFIG_AXXIA_RIO_STAT)
	priv->rio_pw_count++;
#endif
	while (noofpw) {
		axxia_local_config_read(priv, RAB_IB_PW_DATA, &msg_word);
		msg_word = BSWAP(msg_word);
resync:
		if ((pw->msg_wc == 0) && ((msg_word & pw0_mask) != pw0_val)) {
			format_pw_log(disc_words, discarded++, msg_word);
		} else if ((pw->msg_wc == 1) && ((msg_word & pw1_mask) != pw1_val)) {
			pw->msg_wc = 0;
			format_pw_log(disc_words, discarded++, msg_word);
			goto resync;
		} else {
			pw->msg_buffer[pw->msg_wc++] = msg_word;
			if (pw->msg_wc == 4) {
#if defined(CONFIG_AXXIA_RIO_STAT)
				priv->rio_pw_msg_count++;
#endif
				/* Pass the port-write message to RIO core for processing */
				rio_inb_pwrite_handler(mport,
						(union rio_pw_msg *)pw->msg_buffer);
				pw->msg_wc = 0;
			}
		}
		noofpw--;
	}
	if (discarded)
		pr_info_ratelimited("rio[%d]: PW FIFO: %d words discarded %s\n",
				mport->id, discarded, disc_words);
}

static void axxia_rio_flush_pw(struct rio_mport *mport, int noofpw,
			     struct rio_pw_irq *pw_data)
{
	struct rio_priv *priv = mport->priv;
	u32 dummy;
	int x;

	dev_dbg(priv->dev, "(%s): flush %d words from pwbuff\n",
		__func__, noofpw);
	for (x = 0; x < noofpw; x++) {
		axxia_local_config_read(priv, RAB_IB_PW_DATA, &dummy);
		pw_data->discard_count++;
	}
	pw_data->msg_wc = 0;
}

/**
 * enable_pw - enable port-write interface unit
 * @h: Interrupt handler specific data
 *
 * Caller must hold RAB lock
 */
static int enable_pw(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	struct rio_pw_irq *pw_data;
	u32 rval;
	int rc = 0;

	if (priv->pw_data)
		return -EBUSY;

	pw_data = kzalloc(sizeof(struct rio_pw_irq), GFP_KERNEL);
	if (!pw_data)
		return -ENOMEM;

	axxia_local_config_read(priv, RAB_IB_PW_CSR, &rval);
	rval |= RAB_IB_PW_EN;
	axxia_rio_flush_pw(mport, RAB_IB_PW_NUMWORDS(rval), pw_data);
	axxia_local_config_write(priv, RAB_IB_PW_CSR, rval);
	priv->pw_data = pw_data;

	dev_info(priv->dev, "enable port write %p\n", mport);

	return rc;
}

/**
 * disable_pw - Disable port-write interface unit
 * @mport: pointer to struct rio_mport
 *
 * Caller must hold RAB lock
 */
static void disable_pw(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	struct rio_pw_irq *pw_data = priv->pw_data;
	u32 rval;

	dev_info(priv->dev, "disable port write %p\n", mport);

	if (pw_data == NULL)
		return;

	axxia_local_config_read(priv, RAB_IB_PW_CSR, &rval);
	rval &= ~RAB_IB_PW_EN;
	axxia_local_config_write(priv, RAB_IB_PW_CSR, rval);
	kfree(pw_data);
	priv->pw_data = NULL;
}


/**
 * misc_irq_handler - MISC interrupt handler
 * @h: handler specific data
 * @state: Interrupt state
 * Handles the Error, doorbell, Link reset request Interrupts
 */
static void misc_irq_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;
	u32 misc_state;

	axxia_local_config_read(priv, RAB_INTR_STAT_MISC, &misc_state);
	/*
	 * Handle miscellaneous 'Link (IPG) Reset Request'
	 */
	if (misc_state & LINK_REQ_INT)
		link_reset_req(mport);

	if (misc_state & PORT_WRITE_INT) {
		misc_state &= ~PORT_WRITE_INT;
		pw_irq_handler(h, misc_state & PORT_WRITE_INT);
	}

	if (misc_state & (IB_DB_RCV_INT | OB_DB_DONE_INT))
		db_irq_handler(h,
			misc_state & (IB_DB_RCV_INT | OB_DB_DONE_INT));
	/**
	 * Notify platform if port is broken
	 */
	if (misc_state & MISC_FATAL)
		__misc_fatal(mport, misc_state);

	if (misc_state & GRIO_INT) {
		u32 reg_value = 0;

		/* clear retry threshold error indication */
		axxia_local_config_read(priv, EPC_IECSR(priv->port_ndx), &reg_value);
		axxia_local_config_write(priv, EPC_IECSR(priv->port_ndx), reg_value);

		dev_err(priv->dev, "GRIO Error Interrupt\n");
		msleep(500);
	}
		/* TODO Need further Handling */
	if (misc_state & LL_TL_INT)
		dev_err(priv->dev, "Logical Layer Error Interrupt\n");
		/* TODO Need further Handling */
	if (misc_state & UNSP_RIO_REQ_INT)
		dev_dbg(priv->dev, "Unsupported RIO Request Received\n");
		/* TODO Need further Handling */
	if (misc_state & UNEXP_MSG_INT)
		dev_dbg_ratelimited(priv->dev,
			"Unexpected Inbound Data Message Received\n");
		/* TODO Need further Handling */

	axxia_local_config_write(priv, RAB_INTR_STAT_MISC, misc_state);
}

static uint32_t const * const save_rio_reg_config(struct rio_mport const * const mport)
{
	uint32_t *rio_reg_config = NULL;
	uint32_t size = 0;
	int32_t i = 0, j = 0, k = 0;

	for (i = 0; i < ARRAY_SIZE(rio_regs_backup); i++)
		size += rio_regs_backup[i].access_count;

	rio_reg_config = kzalloc(size * sizeof(uint32_t), GFP_KERNEL);

	if (NULL == rio_reg_config)
		goto out;

	for (i = 0; i < ARRAY_SIZE(rio_regs_backup); i++) {
		for (j = 0; j < rio_regs_backup[i].access_count; j++) {
			if (0 != axxia_local_config_read(mport->priv, rio_regs_backup[i].start_addr + (j << 2), &rio_reg_config[k]))
				goto clean;
			k++;
		}
	}

out:
	return rio_reg_config;

clean:
	kfree(rio_reg_config);
	goto out;
}

static uint32_t restore_rio_reg_config(struct rio_mport const * const mport, uint32_t const * const rio_reg_config)
{
	int32_t i = 0, j = 0, k = 0;
	int ret_val = -(EINVAL);

	if (NULL == rio_reg_config)
		goto out;

	for (i = 0; i < ARRAY_SIZE(rio_regs_backup); i++) {
		for (j = 0; j < rio_regs_backup[i].access_count; j++) {
			if (0 != axxia_local_config_write(mport->priv, (rio_regs_backup[i].start_addr + (j << 2)), rio_reg_config[k]))
				goto clean;
			k++;
		}
	}
	ret_val = 0;

out:
	return ret_val;

clean:
	kfree(rio_reg_config);
	goto out;
}

static void link_reset_req(struct rio_mport *mport)
{
	if (NULL != mport) {
		uint32_t const *rio_reg_config_backup = NULL;
		uint32_t ccsr = 0;
		uint32_t escsr = 0;
		uint32_t i = 0;

		/* disable linkdown monitor */
		axxia_local_config_write(mport->priv, RAB_SRDS_CTRL2, 0);

		/* disable port */
		axxia_local_config_read(mport->priv, RIO_CCSR(((struct rio_priv *) mport->priv)->port_ndx), &ccsr);
		ccsr |= RIO_CCSR_PD;
		axxia_local_config_write(mport->priv, RIO_CCSR(((struct rio_priv *) mport->priv)->port_ndx), ccsr);

		/* save config */
		rio_reg_config_backup = save_rio_reg_config(mport);

		/* do srio software reset */
		srio_sw_reset(mport);

		mdelay(100);

		for (i = 0; i < 10; i++)	{
			axxia_local_config_read(mport->priv, RIO_ESCSR(((struct rio_priv *) mport->priv)->port_ndx), &escsr);

			if (RIO_ESCSR_PO == (escsr & RIO_ESCSR_PO))
				break;
			udelay(10);
		}

		/* restore config */
		(void) restore_rio_reg_config(mport, rio_reg_config_backup);

		/* restore linkdown monitor */
		axxia_local_config_write(mport->priv, RAB_SRDS_CTRL2, LINK_DOWN_TIMEOUT);
		axxia_local_config_write(mport->priv, RAB_SRDS_CTRL1, LINK_DOWN_IRQ_MASK);

		kfree(rio_reg_config_backup);
	}
}

static void misc_release_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;
	disable_pw(mport);
}
/**
 * linkdown_irq_handler - Link Down interrupt Status interrupt handler
 * @h: handler specific data
 * @state: Interrupt state
 */
static void linkdown_irq_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	u32 state = 0;
	u32 reg = 0;

	axxia_local_config_read(priv, RAB_SRDS_STAT1, &state);
	dev_err(priv->dev, "RIO LINK IS DOWN(0x20994): 0x%x\n", state);
	msleep(1000);

	do {
		/* try to clear linkdown interrupt status bit */
		axxia_local_config_read(priv, RAB_SRDS_CTRL1, &reg);
		reg |= RAB_SRDS_CTRL1_CLR_LNK;
		axxia_local_config_write(priv, RAB_SRDS_CTRL1, reg);
		reg &= ~RAB_SRDS_CTRL1_CLR_LNK;
		axxia_local_config_write(priv, RAB_SRDS_CTRL1, reg);

		axxia_local_config_read(priv, RAB_SRDS_STAT1, &state);
		if (0 == (state & 0x80000000)) {
			dev_err(priv->dev, "RIO LINK IS UP(0x20994): 0x%x\n", state);
			break;
		} else
			msleep(4000);
	} while (1);

	/* reconfigure linkdown handler */
	axxia_local_config_write(priv, RAB_SRDS_CTRL2, LINK_DOWN_TIMEOUT);
	axxia_local_config_write(priv, RAB_SRDS_CTRL1, LINK_DOWN_IRQ_MASK);
}

/**
 * rpio_irq_handler - RPIO interrupt handler.
 * Service Peripheral Bus bridge, RapidIO -> Peripheral bus interrupt
 *
 * @h: handler specific data
 * @state: Interrupt state
 *
 */
static void rpio_irq_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	u32 rstate;
	axxia_local_config_read(priv, RAB_INTR_STAT_RPIO, &rstate);
#if defined(CONFIG_AXXIA_RIO_STAT)
	if (rstate & RPIO_TRANS_COMPLETE)
		priv->rpio_compl_count++;
#endif
	if (rstate &  RPIO_TRANS_FAILED) {
		u32 rpio_stat;

		axxia_local_config_read(priv, RAB_RPIO_STAT, &rpio_stat);
		if (rpio_stat & RAB_RPIO_STAT_RSP_ERR)
			pr_debug("SRIO: %s:RPIO AXI Response Error\n",
						dev_name(priv->dev));
		if (rpio_stat & RAB_RPIO_STAT_ADDR_MAP)
			pr_debug("SRIO:%s:RPIO Invalid Address Mapping Error\n",
						dev_name(priv->dev));
		if (rpio_stat & RAB_RPIO_STAT_DISABLED)
			pr_debug("SRIO: %s:RPIO Engine Not Enabled\n",
						dev_name(priv->dev));

		axxia_local_config_write(priv, RAB_RPIO_STAT, rpio_stat);
#if defined(CONFIG_AXXIA_RIO_STAT)
		priv->rpio_failed_count++;
#endif
	}
	axxia_local_config_write(priv, RAB_INTR_STAT_RPIO, rstate);
}

/**
 * APIO
 */

/**
 * apio_irq_handler - APIO interrupt handler.
 * Service Peripheral Bus bridge, Peripheral bus -> RapidIO interrupt
 *
 * @h: handler specific data
 * @state: Interrupt state
 *
 */
static void apio_irq_handler(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	u32 astate;
	axxia_local_config_read(priv, RAB_INTR_STAT_APIO, &astate);
#if defined(CONFIG_AXXIA_RIO_STAT)
	if (astate & APIO_TRANS_COMPLETE)
		priv->apio_compl_count++;
#endif
	if (astate & APIO_TRANS_FAILED) {
		u32 apio_stat;

		axxia_local_config_read(priv, RAB_APIO_STAT, &apio_stat);
		if (apio_stat & RAB_APIO_STAT_RQ_ERR)
			pr_err("SRIO: %s:APIO AXI Request Format Error\n",
							dev_name(priv->dev));
		if (apio_stat & RAB_APIO_STAT_TO_ERR)
			pr_err("SRIO: %s:APIO RIO Timeout Error\n",
							dev_name(priv->dev));
		if (apio_stat & RAB_APIO_STAT_RSP_ERR)
			pr_err("SRIO: %s:APIO RIO Response Error\n",
							dev_name(priv->dev));
		if (apio_stat & RAB_APIO_STAT_MAINT_DIS)
			pr_err("SRIO: %s:APIO Maint Mapping Not Enabled\n",
							dev_name(priv->dev));
		if (apio_stat & RAB_APIO_STAT_MEM_DIS)
			pr_err("SRIO: %s:APIO Memory Mapping Not Enabled\n",
							dev_name(priv->dev));
		if (apio_stat & RAB_APIO_STAT_DISABLED)
			pr_err("SRIO: %s:APIO Engine Not Enabled\n",
							dev_name(priv->dev));

		/* show error message only if this is the only error bit set */
		if (RAB_APIO_STAT_MAP_ERR == apio_stat)
			pr_err("SRIO: %s:APIO Invalid Address Mapping Err\n",
					dev_name(priv->dev));

		axxia_local_config_write(priv, RAB_APIO_STAT, apio_stat);
#if defined(CONFIG_AXXIA_RIO_STAT)
		priv->apio_failed_count++;
#endif
	}
	axxia_local_config_write(priv, RAB_INTR_STAT_APIO, astate);
}

/**
 * DOORBELL events
 */

/**
 * axxia_rio_rx_db_int_handler - AXXIA inbound doorbell interrupt handler
 * @mport: Master port with triggered interrupt
 * @mask: Interrupt register data
 *
 * Handles inbound doorbell interrupts.  Executes a callback on received
 * doorbell. Now called from the misc_irq thread, rio-misc-db.
 */
void rx_db_handler(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	struct rio_dbell *dbell;
	u32 csr, info;
	u8 num_msg;
	u16 src_id, db_info;
	int found;

	axxia_local_config_read(priv, RAB_IB_DB_CSR, &csr);
	num_msg = IB_DB_CSR_NUM_MSG(csr);

	for (; num_msg; num_msg--) {
		axxia_local_config_read(priv, RAB_IB_DB_INFO, &info);
		src_id = DBELL_SID(info);
		db_info = DBELL_INF(info);

		found = 0;
		dev_dbg(priv->dev,
			 "Processing doorbell, sid %4.4x info %4.4x\n",
			src_id, db_info);

		list_for_each_entry(dbell, &mport->dbells, node) {
			if (dbell->res->start <= db_info &&
			    (dbell->res->end >= db_info)) {
				found = 1;
				break;
			}
		}
		if (found) {
			/**
			 * NOTE: dst is set to 0 since we don't have
			 *       that value in the ACP
			 */
			if (dbell->dinb)
				dbell->dinb(mport, dbell->dev_id, src_id,
						0, db_info);
		} else {
			dev_dbg(priv->dev,
				"Spurious doorbell, sid %4.4x info %4.4x\n",
				src_id, db_info);
		}
	}
}

void db_irq_handler(struct rio_irq_handler *h, u32 state)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;

	/**
	 * Handle RX doorbell events
	 */
	if (state & IB_DB_RCV_INT)
		rx_db_handler(mport);

	/**
	 * Check for outbound doorbell Error conditions.
	 */
	if (state & OB_DB_DONE_INT) {
		int db;
		u32 csr;
		for (db = 0; db < MAX_OB_DB; db++) {
			axxia_local_config_read(priv, RAB_OB_DB_CSR(db), &csr);

			if (OB_DB_STATUS(csr) == OB_DB_STATUS_RETRY)
				dev_dbg(priv->dev,
				  "Rio Doorbell Retry received\n");
			else if (OB_DB_STATUS(csr) == OB_DB_STATUS_ERROR)
				dev_dbg(priv->dev,
				  "Rio Doorbell send Error\n");
			else if (OB_DB_STATUS(csr) == OB_DB_STATUS_TIMEOUT)
				dev_dbg(priv->dev,
				  "Rio Doorbell send Timeout\n");
		}
	}
}

/**
 * OBDME Events/Outbound Messages
 */

static void release_dme(struct kref *kref)
{
	struct rio_msg_dme *me = container_of(kref, struct rio_msg_dme, kref);
	struct rio_priv *priv = me->priv;
	struct rio_msg_desc *desc;
	int i;

	if (me->desc) {
		for (i = 0, desc = me->desc; i < me->entries; i++, desc++)
			free_pages_exact(desc->msg_virt, RIO_OUTB_DME_TO_BUF_SIZE(priv, me->dme_no));
		kfree(me->desc);
	}

	kfree(me->descriptors);

	if (priv->intern_msg_desc) {
		if (me->dres.parent)
			release_resource(&me->dres);
	}

	kfree(me);
}

static inline struct rio_msg_dme *dme_get(struct rio_msg_dme *me)
{
	if (me)
		kref_get(&me->kref);
	return me;
}

static inline void dme_put(struct rio_msg_dme *me)
{
	if (me)
		kref_put(&me->kref, release_dme);
}

static inline int check_dme(int dme_no,
			    int *num_dmes,
			    int *dmes_in_use,
			    int *dmes)
{
	int i;
	for (i = 0; i < 2; i++) {
		if (dme_no < num_dmes[i]) {
			if (dmes[i] & (1 << dme_no)) {
				if (dmes_in_use[i] & (1 << dme_no))
					return -EBUSY;	/* Already allocated */
				return 0;
			}
		} else {
			dme_no -= num_dmes[i];
		}
	}

	return -ENXIO;	/* Not available */
}

/*
 * Enforce a DME 'choice' previously made
 */
static inline int select_dme(int dme_no,
			     int *num_dmes,
			     int *dmes_in_use,
			     int *dmes,
			     int value)
{
	int i;
	for (i = 0; i < 2; i++) {
		if (dme_no < num_dmes[i]) {
			dmes_in_use[i] &= ~(1 << dme_no);
			dmes_in_use[i] |= (value << dme_no);
			return 0;
		} else {
			dme_no -= num_dmes[i];
		}
	}

	return -ENXIO;	/* Not available */
}

/* Selects the DME for an Mbox
 * based on its occupancy. Two Outbound DMEs
 * are shared among mailboxes
 */
static inline int choose_ob_dme_static(
	struct rio_priv	*priv,
	int mbox_dest,
	int buf_sz,
	struct rio_msg_dme **ob_dme)
{
	int  i, ndx, sz, min_entries = 0;
	int  dme_no = 0, ret_dme_no = -ENXIO;
	struct rio_msg_dme *ret_dme = NULL;
	struct rio_tx_dme *dme_s;

	/* Multi-segment vs single-segment DMEs */
	ndx = RIO_MBOX_TO_IDX(mbox_dest);
	switch (ndx) {
	case 0:
		if ((priv->num_outb_dmes[0] == 0) || (priv->outb_dmes[0] == 0))
			return -ENXIO;
		break;
	case 1:
		if ((priv->num_outb_dmes[1] == 0) || (priv->outb_dmes[1] == 0))
			return -ENXIO;
		dme_no += priv->num_outb_dmes[0];
		break;
	default:
		dev_err(priv->dev, "Attempt to select unknown OB DME type!\n");
		return -ENXIO;
	}

	/* Find one with fewest entries, or sufficient free entries */
	for (i = 0; i < priv->num_outb_dmes[ndx]; i++, dme_no++) {
		sz = RIO_OUTB_DME_TO_BUF_SIZE(priv, dme_no);

		if (sz > buf_sz)
			continue;

		dme_s = &priv->ob_dme_shared[dme_no];

		if (dme_s->ring_size_free > min_entries) {
			min_entries = dme_s->ring_size_free;
			ret_dme = dme_s->me;
			ret_dme_no = dme_no;
		}
	}

	/* PR279747: use OBDME0 for all traffic */
	ret_dme_no = 0;
	dme_s = &priv->ob_dme_shared[ret_dme_no];
	(*ob_dme) = dme_s->me;
	return ret_dme_no;
}

static void release_mbox(struct kref *kref)
{
	struct rio_rx_mbox *mb = container_of(kref, struct rio_rx_mbox, kref);
	struct rio_priv *priv = mb->mport->priv;
	int letter;
	u32 dme_no;
	u32 dme;

	/* Quickly disable the engines */
	for (letter = 0; letter < RIO_MSG_MAX_LETTER; letter++) {
		for (dme = 0; dme < mb->num_dme[letter]; dme++) {
			if (mb->me[letter][dme])
				axxia_local_config_write(priv,
				   RAB_IB_DME_CTRL(mb->me[letter][dme]->dme_no),
									0);
		}
	}

	/* And then release the remaining resources */
	for (letter = 0; letter < RIO_MSG_MAX_LETTER; letter++) {
		for (dme = 0; dme < mb->num_dme[letter]; dme++) {
			if (mb->me[letter][dme]) {
				dme_no = mb->me[letter][dme]->dme_no;
				dme_put(mb->me[letter][dme]);
				select_dme(dme_no,
					&priv->num_inb_dmes[0],
					&priv->inb_dmes_in_use[0],
					&priv->inb_dmes[0], 0);
				priv->ib_dme[dme_no] = NULL;
			}
		}
	}

	for (letter = 0; letter < RIO_MSG_MAX_LETTER; letter++) {
		kfree(mb->virt_buffer[letter]);
		kfree(mb->cookie[letter]);
	}

	kfree(mb);
}

static inline struct rio_rx_mbox *mbox_get(struct rio_rx_mbox *mb)
{
	if (mb)
		kref_get(&mb->kref);
	return mb;
}

static inline void mbox_put(struct rio_rx_mbox *mb)
{
	if (mb)
		kref_put(&mb->kref, release_mbox);
}

static int alloc_msg_descriptors(struct rio_mport *mport,
				  struct resource *dres,
				  int buf_sz,
				  int entries,
				  int need_to_init,
				  struct rio_msg_desc **desc,
				  struct rio_desc **descriptors)
{
	struct rio_priv *priv = mport->priv;
	struct rio_msg_desc *rdesc = NULL, *idesc;
	struct rio_desc *rdescriptors = NULL;
	int i;

	if (priv->intern_msg_desc) {
		dres->name = "DME_DESC";
		dres->flags = ACP_RESOURCE_HW_DESC;
		if (allocate_resource(&priv->acpres[ACP_HW_DESC_RESOURCE],
				dres, entries,
				priv->acpres[ACP_HW_DESC_RESOURCE].start,
				priv->acpres[ACP_HW_DESC_RESOURCE].end,
				0x1, NULL, NULL)) {
			memset(dres, 0, sizeof(*dres));
			goto err;
		}
	} else {
		dres->start = 0;
	}

	rdesc = kzalloc(sizeof(struct rio_msg_desc) * entries, GFP_ATOMIC);
	if (rdesc == NULL)
		goto err;
	rdescriptors = kzalloc(sizeof(struct rio_desc) * entries, GFP_ATOMIC);
	if (rdescriptors == NULL)
		goto err;

	for (i = 0, idesc = rdesc; i < need_to_init; i++, idesc++) {
		idesc->msg_virt = alloc_pages_exact(buf_sz, GFP_ATOMIC | GFP_DMA | __GFP_ZERO);
		if (!idesc->msg_virt)
			goto err;
		idesc->msg_phys = virt_to_phys(idesc->msg_virt);
	}

	idesc--;
	idesc->last = DME_DESC_DW0_NXT_DESC_VALID;

	(*desc) = rdesc;
	(*descriptors) = rdescriptors;

	return 0;

err:
	kfree(rdesc);
	kfree(rdescriptors);
	return -ENOMEM;
}

static struct rio_msg_dme *alloc_message_engine(struct rio_mport *mport,
						int dme_no, void *dev_id,
						int buf_sz, int entries)
{
	struct rio_priv *priv = mport->priv;
	struct rio_msg_dme *me = kzalloc(sizeof(struct rio_msg_dme),
					 GFP_ATOMIC);
	int rc = 0;

	if (!me)
		return ERR_PTR(-ENOMEM);

	memset(me, 0, sizeof(struct rio_msg_dme));

	kref_init(&me->kref);
	spin_lock_init(&me->lock);
	me->priv = priv;
	me->sz = 0;/*buf_sz;*/

	rc = alloc_msg_descriptors(mport, &me->dres, buf_sz, entries,
				entries, &me->desc, &me->descriptors);
	if (rc < 0)
		goto err;

	me->entries = entries;
	me->dev_id = dev_id;
	me->write_idx = 0;
	me->read_idx = 0;
	me->tx_dme_tmo = 0;
	me->dme_no = dme_no;

	return me;

err:
	dme_put(me);
	return ERR_PTR(rc);
}

/**
 * ob_dme_msg_handler - Outbound Data message handler
 * --- Called from OB DME irq handler thread ---
 * @h: Pointer to interrupt-specific data
 *
 * Handles outbound message interrupts. Executes a callback,
 * if available.
 *
 * @note:
 * HW descriptor fetch and update may be out of order.
 * Check state of all used descriptors and take care to not fall into
 * any of the traps that come with this design:
 *
 * Due to this (possibly) out of order execution in the HW, SW ack of
 * descriptors must be done atomically, re-enabling descriptors with
 * completed transactions while processing finished transactions may
 * break the ring and leave the DMA engine in a state where it doesn't
 * process new inserted requests.
 */
static void ob_dme_msg_handler(struct rio_irq_handler *h, u32 dme_no)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;
	struct rio_msg_dme *dme = priv->ob_dme_shared[dme_no].me;
	struct rio_msg_desc *desc;
	u32 dw0;
	u32 dw1;
	int mbox;
	struct rio_tx_mbox *mb;
	unsigned int iteration = 0;
	void *ptr;

	/**
	 * Process all completed transactions
	 */
ob_dme_restart:
	while (dme->read_idx != dme->write_idx) {
		AXXIA_RIO_SYSMEM_BARRIER();
		dw0 = *((u32 *)DESC_TABLE_W0_MEM(dme, dme->read_idx));
		if ((dw0 & DME_DESC_DW0_VALID) &&
			(dw0 & DME_DESC_DW0_READY_MASK)) {
			*((u32 *)DESC_TABLE_W0_MEM(dme, dme->read_idx))
					= dw0 & DME_DESC_DW0_NXT_DESC_VALID;
			dw1 = *((u32 *)DESC_TABLE_W1_MEM(dme,
						dme->read_idx));
			__dme_dw_dbg(priv->dev, dme, 1, dw0, dw1);
			desc = &dme->desc[dme->read_idx];
			ptr = desc->kvirt;
			dme->read_idx = (dme->read_idx + 1) &
						(dme->entries - 1);
			mbox = (dw1 >> 2) & 0x3;
			mb = priv->ob_mbox[mbox];
			if (mb) {
				if (mport->outb_msg[mbox].mcback) {
					mb->tx_slot = (mb->tx_slot + 1)
							%(mb->ring_size);
					mport->outb_msg[mbox].mcback(mport,
							mb->dev_id,
							mbox,
							(dw0 & DME_DESC_DW0_ERROR_MASK) != 0,
							ptr);
				}
#ifdef CONFIG_AXXIA_RIO_STAT
				mb->compl_msg_count++;
#endif
			}
			iteration++;
		} else
			break;
	}
	if (iteration) {
		iteration = 0;
		goto ob_dme_restart;
	}
}

/**
 * ob_dme_irq_handler - Outbound message interrupt handler
 * --- Called in threaded irq handler ---
 * @h: Pointer to interrupt-specific data
 *
 * Handles outbound message interrupts. Calls the
 * msg handler if dscriptor xfer complete is set.
 * or reports the error
 */
enum hrtimer_restart ob_dme_tmr_handler(struct hrtimer *hr)
{
	struct rio_tx_dme *obd = container_of(hr, struct rio_tx_dme, tmr);
	struct rio_msg_dme *me = obd->me;
	struct rio_priv *priv = me->priv;
	struct rio_irq_handler *h = &priv->ob_dme_irq;
	u32 dme_stat, dme_no;

	dme_no = me->dme_no;
	axxia_local_config_read(priv, RAB_OB_DME_STAT(dme_no),
						&dme_stat);

	if (dme_stat & (OB_DME_STAT_DESC_FETCH_ERR |
				OB_DME_STAT_DESC_ERR |
				OB_DME_STAT_DESC_UPD_ERR))
		pr_err_ratelimited("SRIO: %s:OB DME%d: Descriptor Error\n",
						dev_name(priv->dev), dme_no);
	else {
		if (dme_stat & (OB_DME_STAT_DATA_TRANS_ERR |
				OB_DME_STAT_RESP_ERR |
				OB_DME_STAT_RESP_TO)) {
			if (dme_stat & OB_DME_STAT_DATA_TRANS_ERR)
				pr_err_ratelimited("SRIO: %s:OB DME%d: Transaction Error\n",
						dev_name(priv->dev), dme_no);
			if (dme_stat & OB_DME_STAT_RESP_ERR)
				pr_debug_ratelimited(
					"SRIO: %s:OB DME%d: Response Error\n",
						dev_name(priv->dev), dme_no);
			if (dme_stat & OB_DME_STAT_RESP_TO)
				pr_info_ratelimited(
				"SRIO: %s:OB DME%d: Response Timeout Err\n",
						dev_name(priv->dev), dme_no);
		}
		ob_dme_msg_handler(h, dme_no);
	}
	axxia_local_config_write(priv, RAB_OB_DME_STAT(dme_no),
							dme_stat);
	hrtimer_forward_now(&obd->tmr, ktime_set(0, axxia_hrtimer_delay));
	return HRTIMER_RESTART;
}

static int alloc_ob_dme_shared(struct rio_priv *priv,
			struct rio_tx_dme *dme_s, int dme_no)
{
	int rc = 0;
	int sz;
	struct rio_mport *mport = priv->mport;
	struct rio_msg_dme *me = NULL;
	struct rio_msg_desc *desc = NULL;
	u32 dw0, dw1, dw2, dw3;
	u64  desc_chn_start = 0;
	int entries = CONFIG_OB_DME_ENTRY_SIZE;
	int i;

	sz = RIO_OUTB_DME_TO_BUF_SIZE(priv, dme_no);
	entries = roundup_pow_of_two(entries);
	pr_info("RIO: Configuring DME %d with %d entries\n", dme_no, entries);
	me = alloc_message_engine(mport,
				dme_no, NULL, sz, entries);
	if (IS_ERR(me)) {
		rc = PTR_ERR(me);
		goto err;
	}

	for (i = 0, desc = me->desc; i < entries; i++, desc++) {
		dw0 = 0;
		if (!priv->intern_msg_desc) {
#ifdef AXM55XX_OUTB_DME_BBS
			dw1 = (u32)(desc->msg_phys >> 11) & 0x1fe00000;
			dw2 = (u32)(desc->msg_phys >>  0) & 0x3fffffff;
#else
			dw1 = 0;
			dw2 = (u32)(desc->msg_phys >>  8) & 0x3fffffff;
#endif
			if (axxia_rio_is_x9()) {
				dw0 = (u32)(desc->msg_phys >> 38) & 0x3;
				dw0 = (u32)(dw0 << 12);
			}

			*((u32 *)DESC_TABLE_W0_MEM(me, i)) = dw0;
			*((u32 *)DESC_TABLE_W1_MEM(me, i)) = dw1;
			*((u32 *)DESC_TABLE_W2_MEM(me, i)) = dw2;
			*((u32 *)DESC_TABLE_W3_MEM(me, i)) = 0;
		} else {
			dw1 = 0;
			dw2 = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
			axxia_local_config_write(priv,
				    DESC_TABLE_W0(me->dres.start+i), dw0);
			axxia_local_config_write(priv,
				    DESC_TABLE_W1(me->dres.start+i), dw1);
			axxia_local_config_write(priv,
				    DESC_TABLE_W2(me->dres.start+i), dw2);
			axxia_local_config_write(priv,
				    DESC_TABLE_W3(me->dres.start+i), 0);
		}
	}


	/**
	* Last descriptor - make ring.
	* Next desc table entry -> dw2.First desc address[37:36]
	*                       -> dw3.First desc address[35:4].
	* (desc_base + 0x10 * nr)
	*/
	desc--; i--;
	dw0 |= DME_DESC_DW0_NXT_DESC_VALID;
	if (!priv->intern_msg_desc) {
		desc_chn_start =
			(uintptr_t)virt_to_phys(me->descriptors);

		if (axxia_rio_is_x9()) {
			dw0  = *((u32 *)DESC_TABLE_W0_MEM(me, i));
			dw0 |= DME_DESC_DW0_NXT_DESC_VALID;
			dw3 = (u32)(desc_chn_start >> 38) & 0x3;
			dw0 |= (dw3 << 14);
		}

		dw2  = *((u32 *)DESC_TABLE_W2_MEM(me, i));
		if (axxia_rio_is_x9())
			dw2 |= (desc_chn_start >> 6) & 0xc0000000;
		else
			dw2 |= (desc_chn_start >> 4) & 0xc0000000;
		dw3  = desc_chn_start >> 4;
		*((u32 *)DESC_TABLE_W0_MEM(me, i)) = dw0;
		*((u32 *)DESC_TABLE_W2_MEM(me, i)) = dw2;
		*((u32 *)DESC_TABLE_W3_MEM(me, i)) = dw3;
	} else {
		desc_chn_start = DESC_TABLE_W0(me->dres.start);
		axxia_local_config_read(priv,
				DESC_TABLE_W2(me->dres.start+i), &dw2);
		dw2 |= ((desc_chn_start >> 8) & 0xc0000000);
		dw3  = 0;
		axxia_local_config_write(priv,
				DESC_TABLE_W0(me->dres.start+i), dw0);
		axxia_local_config_write(priv,
				DESC_TABLE_W2(me->dres.start+i), dw2);
		axxia_local_config_write(priv,
				DESC_TABLE_W3(me->dres.start+i), dw3);
	}
	test_and_set_bit(RIO_DME_OPEN, &me->state);
	dme_s->me = me;
	dme_s->ring_size = 0x0;
	dme_s->ring_size_free = entries;
err:
	return rc;
}

/**
 * open_outb_mbox_static - Initialize AXXIA outbound mailbox
 *			   using statically allocated DME descriptors.
 *
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox_id: Mailbox to open
 * @entries: Number of entries in the outbound mailbox ring for each letter
 * @prio: 0..3, higher number -> lower priority.
 *
 * Caller must hold RAB lock
 * If the specified mbox DME has already been opened/reserved, then we just
 * abort out of this operation with "busy", and without changing resource
 * allocation for the mbox DME.
 *
 * To increase efficiecny the Descriptors are allocated and initalized during
 * initialization time and then kept forever to be reused.
 *
 * Returns:
 * %0 if successful
 * %-EINVAL if an argument is invalid
 * %-ENOMEM if unable to allocate sufficient memory
 * %-ENODEV if unable to find a DME matching the input arguments
 */

static int open_outb_mbox_static(struct rio_mport *mport,
			void *dev_id, int mbox_id, int entries, int prio)
{
	int  rc = 0;
	int  dme_no, buf_sz = 0;
	struct rio_priv *priv = mport->priv;
	struct rio_tx_mbox *mb;/* = priv->ob_mbox[mbox_id];*/
	struct rio_msg_dme *me = NULL;
	u32 dme_ctrl, dme_stat, desc_addr, wait = 0;
	u64  desc_chn_start = 0;

	if ((mbox_id < 0) || (mbox_id > RIO_MAX_TX_MBOX) ||
	    (entries < 2) || (entries > priv->desc_max_entries))
		return -EINVAL;
	if (priv->ob_mbox[mbox_id])
		return -EINVAL;
	mb = kzalloc(sizeof(struct rio_tx_mbox), GFP_KERNEL);
	if (!mb)
		return -ENOMEM;
	mb->dme_no = 0xff;
#ifdef CONFIG_AXXIA_RIO_STAT
	mb->sent_msg_count = 0;
	mb->compl_msg_count = 0;
#endif

	if (test_bit(RIO_MB_OPEN, &mb->state)) {
		return -EINVAL;
	}

	/*
	** Pick the OB DME that we will use for this mailbox
	*/
		buf_sz = RIO_MBOX_TO_BUF_SIZE(mbox_id);

		dme_no = choose_ob_dme_static(priv, mbox_id, buf_sz, &me);
		if (dme_no < 0) {
			rc = dme_no;
			goto err;
		}
		if (IS_ERR_OR_NULL(me)) {
			rc = PTR_ERR(me);
			goto err;
		}

		if (!test_bit(RIO_DME_MAPPED, &me->state)) {
			do {
				axxia_local_config_read(priv,
					RAB_OB_DME_STAT(dme_no), &dme_stat);
				if (wait++ > 100) {
					rc = -EBUSY;
					goto err;
				}
			} while (dme_stat & OB_DME_STAT_TRANS_PEND);
			desc_chn_start =
				(uintptr_t)virt_to_phys(me->descriptors);

			dme_ctrl  = (prio & 0x3) << 4;
			dme_ctrl |= (u32)((desc_chn_start >> 6) & 0xc0000000);
			desc_addr  = (u32)desc_chn_start >> 4;
			axxia_local_config_write(priv,
				RAB_OB_DME_DESC_ADDR(dme_no), desc_addr);
			axxia_local_config_write(priv, RAB_OB_DME_CTRL(dme_no),
					dme_ctrl);
			me->dme_ctrl = dme_ctrl;
			me->dme_ctrl |= (DME_WAKEUP | DME_ENABLE);
			priv->ob_dme_irq.irq_state_mask |= (1 << dme_no);
			axxia_local_config_write(priv, RAB_INTR_STAT_ODME,
								1<<dme_no);
			axxia_local_config_write(priv, RAB_INTR_ENAB_ODME,
					priv->ob_dme_irq.irq_state_mask);
			test_and_set_bit(RIO_DME_MAPPED, &me->state);
		}


	mb->mport = mport;
	mb->mbox_no = mbox_id;
	mb->dme_no = dme_no;
	mb->me = me;
	mb->ring_size = entries;
	mb->tx_slot = 0;
	mb->dev_id = dev_id;
	me->sz++;
	msleep(500); /* Delay added to ensure completion of any pending TX
			before Transmission on this Mailbox */

	if (me->sz == 1) {
		hrtimer_init(&priv->ob_dme_shared[dme_no].tmr,
				 CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		priv->ob_dme_shared[dme_no].tmr.function = ob_dme_tmr_handler;
		hrtimer_start(&priv->ob_dme_shared[dme_no].tmr,
				ktime_set(0, (axxia_hrtimer_delay)),
					HRTIMER_MODE_REL_PINNED);
	}

	test_and_set_bit(RIO_MB_MAPPED, &mb->state);

	priv->ob_dme_shared[dme_no].ring_size += entries;
	priv->ob_dme_shared[dme_no].ring_size_free -= entries;

#ifdef CONFIG_AXXIA_RIO_STAT
	me->desc_done_count = 0;
	me->desc_error_count = 0;
	me->desc_rio_err_count = 0;
	me->desc_axi_err_count = 0;
	me->desc_tmo_err_count = 0;
#endif
	/**
	 * Finish updating the mailbox and DME state before we go
	 */
	test_and_set_bit(RIO_MB_OPEN, &mb->state);
	priv->ob_mbox[mbox_id] = mb;
	return 0;

err:
	kfree(mb);
	return rc;
}


/**
 * release_outb_dme - Close AXXIA outbound DME engine structures
 * @mport: Master port implementing the outbound message unit
 * @mbox: Mailbox to close
 *
 * Caller must hold RAB lock
 * Release all resources i.e. DMEs, descriptors, buffers, and so on.
 */

static void release_outb_dme(struct rio_irq_handler *h)
{
	struct rio_priv *priv = h->data;
	int i;
	struct rio_msg_dme *me;

	for (i = 0; i < DME_MAX_OB_ENGINES; i++) {
		me = priv->ob_dme_shared[i].me;
		if (me && test_bit(RIO_DME_OPEN, &me->state)) {
			if (test_bit(RIO_DME_MAPPED, &me->state)) {
				axxia_local_config_write(priv,
					RAB_OB_DME_CTRL(me->dme_no), 0);

				select_dme(me->dme_no,
					&priv->num_outb_dmes[0],
					&priv->outb_dmes_in_use[0],
					&priv->outb_dmes[0], 0);
			}

			dme_put(me);
		}
	}
	h->data = NULL;
}

/**
 * ib_dme_irq_handler - AXXIA inbound message interrupt handler
 * @mport: Master port with triggered interrupt
 * @mask: Interrupt register data
 *
 * Handles inbound message interrupts.  Executes a callback, if available,
 * on received message. Reports the Error.
 */
static void  ib_dme_irq_handler(struct rio_irq_handler *h/*, u32 state*/)
{
	struct rio_priv *priv = h->data;
	struct rio_mport *mport = priv->mport;
	int mbox_no;
	int letter;
	u32 dme_mask, mask;

ib_dme_restart:
	spin_lock(&rio_rx_mbox_lock);

	axxia_local_config_read(priv, RAB_INTR_STAT_IDME, &dme_mask);
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, dme_mask);
	axxia_local_config_read(priv, RAB_INTR_ENAB_IDME, &mask);

	dme_mask &= mask;
	if (!dme_mask) {
		spin_unlock(&rio_rx_mbox_lock);
		return;
	}
	/**
	 * Inbound mbox has 4 engines, 1 per letter.
	 * For each message engine that contributes to IRQ state,
	 * go through all descriptors in queue that have been
	 * written but not handled.
	 */
	while (dme_mask) {
		struct rio_msg_dme *me;
		u32 dme_stat;
		int dme_no = __fls(dme_mask);
		dme_mask ^= (1 << dme_no);
		me = priv->ib_dme[dme_no];
		/**
		 * Get and clear latched state
		 */
		axxia_local_config_read(priv,
					   RAB_IB_DME_STAT(dme_no), &dme_stat);
		axxia_local_config_write(priv,
					    RAB_IB_DME_STAT(dme_no), dme_stat);
		if (!me)
			continue;

		mbox_no = me->mbox;
		letter = me->letter;

		if (dme_stat & IB_DME_STAT_ERROR_MASK) {
			if (dme_stat & (IB_DME_STAT_DESC_UPDATE_ERR |
					IB_DME_STAT_DESC_ERR |
					IB_DME_STAT_DESC_FETCH_ERR))
				pr_err_ratelimited(
				"SRIO: %s:IB Mbx%d Let%d DME%d Descriptr Err\n",
				dev_name(priv->dev), mbox_no, letter, dme_no);

			if (dme_stat & IB_DME_STAT_DATA_TRANS_ERR)
				pr_err_ratelimited(
				"SRIO: %s:IB Mbx%d Let%d DME%d Transact Err\n",
				dev_name(priv->dev), mbox_no, letter, dme_no);

			if (dme_stat & IB_DME_STAT_MSG_ERR)
				pr_err_ratelimited(
				"SRIO: %s:IB Mbx%d Let%d DME%d Message Err\n",
				dev_name(priv->dev), mbox_no, letter, dme_no);

			if (dme_stat & (IB_DME_STAT_MSG_TIMEOUT))
				pr_warn_ratelimited(
				"SRIO: %s:IB Mbx%d Let%d DME%d SRIO Timeout\n",
				dev_name(priv->dev), mbox_no, letter, dme_no);
		}

		if (mport->inb_msg[mbox_no].mcback)
			mport->inb_msg[mbox_no].mcback(mport, me->dev_id,
						       mbox_no);

	}
	spin_unlock(&rio_rx_mbox_lock);
	goto ib_dme_restart;
}

enum hrtimer_restart ib_dme_tmr_handler(struct hrtimer *hr)
{
	struct rio_rx_mbox *mb = container_of(hr, struct rio_rx_mbox, tmr);
	struct rio_mport *mport = mb->mport;
	struct rio_priv *priv = mport->priv;
	int mbox_no;
	int letter;
	u32 dme_mask, mask;
ib_dme_restart:
	spin_lock(&rio_rx_mbox_lock);

	axxia_local_config_read(priv, RAB_INTR_STAT_IDME, &dme_mask);
	dme_mask &= mb->irq_state_mask;
	mask = dme_mask;
	if (!mask) {
		spin_unlock(&rio_rx_mbox_lock);
		hrtimer_forward_now(&mb->tmr,
				ktime_set(0, axxia_hrtimer_delay));
		return HRTIMER_RESTART;
	}
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, mask);
	/**
	 * Inbound mbox has 4 engines, 1 per letter.
	 * For each message engine that contributes to IRQ state,
	 * go through all descriptors in queue that have been
	 * written but not handled.
	 */
	while (dme_mask) {
		struct rio_msg_dme *me;
		u32 dme_stat;
		int dme_no = __fls(dme_mask);
		dme_mask ^= (1 << dme_no);
		me = priv->ib_dme[dme_no];
		/**
		 * Get and clear latched state
		 */
		axxia_local_config_read(priv,
					   RAB_IB_DME_STAT(dme_no), &dme_stat);
		axxia_local_config_write(priv,
					    RAB_IB_DME_STAT(dme_no), dme_stat);
		if (!me)
			continue;

		mbox_no = me->mbox;
		letter = me->letter;
		if (!(dme_stat & 0xff))
			continue;

		if (dme_stat & (IB_DME_STAT_DESC_XFER_CPLT |
				IB_DME_STAT_DESC_XFER_CPLT)) {
			if (mport->inb_msg[mbox_no].mcback)
				mport->inb_msg[mbox_no].mcback(mport, me->dev_id,
							       mbox_no);
		}

		if (dme_stat & IB_DME_STAT_ERROR_MASK) {
			if (dme_stat & (IB_DME_STAT_DESC_UPDATE_ERR |
					IB_DME_STAT_DESC_ERR |
					IB_DME_STAT_DESC_FETCH_ERR))
				pr_err_ratelimited(
				"SRIO: %s:IB Mbx%d Let%d DME%d Descriptr Err\n",
				dev_name(priv->dev), mbox_no, letter, dme_no);

			if (dme_stat & IB_DME_STAT_DATA_TRANS_ERR)
				pr_err_ratelimited(
				"SRIO: %s:IB Mbx%d Let%d DME%d Transact Err\n",
				dev_name(priv->dev), mbox_no, letter, dme_no);

			if (dme_stat & IB_DME_STAT_MSG_ERR)
				pr_err_ratelimited(
				"SRIO: %s:IB Mbx%d Let%d DME%d Message Err\n",
				dev_name(priv->dev), mbox_no, letter, dme_no);

			if (dme_stat & (IB_DME_STAT_MSG_TIMEOUT))
				pr_warn_ratelimited(
				"SRIO: %s:IB Mbx%d Let%d DME%d SRIO Timeout\n",
				dev_name(priv->dev), mbox_no, letter, dme_no);
		}

	}
	spin_unlock(&rio_rx_mbox_lock);
	goto ib_dme_restart;
}

/**
 * open_inb_mbox_let - Initialize AXXIA inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open 0..(MID-1),
 *            0..3 multi segment,
 *            4..(MID-1) single segment
 * @letter: the letter ID
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring.  Sets up desciptor ring and memory
 * for messages for a letter in the mailbox.
 *
 * Returns %0 on success and %-EINVAL or %-ENOMEM on failure.
 */

static int open_inb_mbox_let(struct rio_mport *mport, void *dev_id,
			 int mbox, int letter, int entries, int dme_per_letter)
{
	struct rio_priv *priv = mport->priv;
	struct rio_irq_handler *h = NULL;
	int i;
	int rc, buf_sz;
	u32 irq_state_mask = 0;
	u32 dme_ctrl;
	struct rio_rx_mbox *mb;
	int dme, entry_per_dme;
	int mbnew = 0;

	if ((mbox < 0) || (mbox >= RIO_MAX_RX_MBOX))
		return -EINVAL;

	if ((entries < 2) || (entries > priv->desc_max_entries))
		return -EINVAL;

	if (dme_per_letter <= 0) {
		pr_err("DME mapping invalid with %d DMEs\n", dme_per_letter);
		return -EINVAL;
	}
	h = &priv->ib_dme_irq;

	buf_sz = RIO_MBOX_TO_BUF_SIZE(mbox);
	/*
	 * Adding 1 to entries to ensure the presence of invalid descriptor
	 * in the circular buffer, to avoid the hardware getting into an
	 * indefinite loop
	 */
	entry_per_dme = (entries + (dme_per_letter - 1))/dme_per_letter;
	entry_per_dme += 1;
	entries = entry_per_dme * dme_per_letter;
	pr_debug("RIO %d - Opening inbound mbox %d with %d entries\n",
			priv->ndx, mbox, entries);

	mb = ib_mbox(priv, mbox);
	if (mb) {
		if (mb->me[letter][0] != NULL)
			return -EBUSY;
	} else {
		mb = kzalloc(sizeof(*mb), GFP_ATOMIC);
		if (!mb)
			return -ENOMEM;
		mbnew = 1;
		mb->mbox_no = mbox;

		kref_init(&mb->kref);
		mb->mport = mport;
	}
	mb->ring_size[letter] = entries;

	/* Initialize rx buffer ring */
	mb->virt_buffer[letter] = kcalloc(entries, sizeof(void *), GFP_ATOMIC);
	if (!mb->virt_buffer[letter])
		return -ENOMEM;

	mb->cookie[letter] = kcalloc(entries, sizeof(void *), GFP_ATOMIC);
	if (!mb->cookie[letter])
		return -ENOMEM;

	mb->last_rx_slot[letter] = 0;
	mb->next_rx_slot[letter] = 0;

	mb->num_dme[letter] = 0;
	for (dme = 0; dme < dme_per_letter ; dme++) {
		int dme_no = 0;
		struct rio_msg_dme *me = NULL;
		struct rio_msg_desc *desc;
		u32 dw0, dw1, dw2, dw3;
		u64 desc_chn_start, desc_addr;
		u32 dme_stat, wait = 0;
		u32 buffer_size = (buf_sz > 256 ? 3 : 0);

		/*
		 * Search for a free DME, so we can more efficiently map
		 * them to the all of the mbox||letter combinations.
		 */
		for (i = 0, rc = -1;
		     i < (priv->num_inb_dmes[0]+priv->num_inb_dmes[1]);
		     i++) {
			rc = check_dme(i, &priv->num_inb_dmes[0],
				&priv->inb_dmes_in_use[0],
				&priv->inb_dmes[0]);
			if (rc == 0) {
				dme_no = i;
				break;
			}
		}
		if (rc < 0) {
			dev_err(priv->dev,
				"No Free IB DME... exiting...\n");
			return rc;
		}

		dev_info(priv->dev, "%s: mb %d let %d dme %d ent %d\n",
			__func__, mbox, letter, dme_no, entries);
		me = alloc_message_engine(mport, dme_no, dev_id,
					  buf_sz, entry_per_dme);
		if (IS_ERR(me)) {
			rc = PTR_ERR(me);
			goto err;
		}
		irq_state_mask |= (1 << dme_no);

		do {
			axxia_local_config_read(priv,
					   RAB_IB_DME_STAT(me->dme_no),
					   &dme_stat);
			if (wait++ > 100) {
				rc = -EBUSY;
				goto err;
			}
		} while (dme_stat & IB_DME_STAT_TRANS_PEND);

		mb->me[letter][dme] = me;
		mb->num_dme[letter]++;

		dw0 = ((buffer_size & 0x3) << 4) |
		      DME_DESC_DW0_EN_INT;
		/*Valid bit will be set in add_inb_buffer*/

		dw1 = DME_DESC_DW1_XMBOX(mbox) |
		      DME_DESC_DW1_MBOX(mbox)  |
		      DME_DESC_DW1_LETTER(letter);
		dw3 = 0;	/* 0 means, next contiguous addr
					 * Also next desc valid bit in dw0
					 * must be zero. */
		for (i = 0, desc = me->desc; i < entry_per_dme;
						i++, desc++) {
			/* Reference AXX5500 Peripheral Subsystem
			 * Multicore Reference Manual, January 2013,
			 * Chapter 5, p. 584 */
			if (axxia_rio_is_x9()) {
				dw2 = (u32)(desc->msg_phys >> 8) &
							0xc0000000;
				dw2 = (dw2 >> 9);
				dw1 |= dw2;
			} else
				dw1 |= 0;

			dw2  = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
			*((u32 *)DESC_TABLE_W0_MEM(me,
						 i)) = dw0;
			*((u32 *)DESC_TABLE_W1_MEM(me,
						 i)) = dw1;
			*((u32 *)DESC_TABLE_W2_MEM(me,
						 i)) = dw2;
			*((u32 *)DESC_TABLE_W3_MEM(me,
						 i)) = dw3;
		}

		/**
		 * Last descriptor - make ring.
		 * Next desc table entry -> dw2.First desc address[37:36].
		 *                       -> dw3.First desc address[35:4].
		 * (desc_base + 0x10 * nr)
		 */
		desc--; i--;
		dw0 |= DME_DESC_DW0_NXT_DESC_VALID;
		dw0 &= ~DME_DESC_DW0_VALID;

		desc_chn_start =
			(uintptr_t)virt_to_phys(me->descriptors);
		if (axxia_rio_is_x9()) {
			dw1  = *((u32 *)DESC_TABLE_W1_MEM(me, i));
			dw2  = (u32)(desc_chn_start >> 8) & 0xc0000000;
			dw2  = (u32)(dw2 >> 11);
			dw1 |= dw2;
		}

		dw2  = *((u32 *)DESC_TABLE_W2_MEM(me, i));
		if (axxia_rio_is_x9())
			dw2 |= (desc_chn_start >> 6) & 0xc0000000;
		else
			dw2 |= (desc_chn_start >> 4) & 0xc0000000;
		dw3  = desc_chn_start >> 4;
		*((u32 *)DESC_TABLE_W0_MEM(me, i)) = dw0;
		if (axxia_rio_is_x9())
			*((u32 *)DESC_TABLE_W1_MEM(me, i)) = dw1;
		*((u32 *)DESC_TABLE_W2_MEM(me, i)) = dw2;
		*((u32 *)DESC_TABLE_W3_MEM(me, i)) = dw3;

		/**
		 * Setup the DME including descriptor chain start address
		 */
		dme_ctrl = RAB_IB_DME_CTRL_XMBOX(mbox)    |
			   RAB_IB_DME_CTRL_MBOX(mbox)     |
			   RAB_IB_DME_CTRL_LETTER(letter) |
			   DME_WAKEUP                     |
			   DME_ENABLE;
		dme_ctrl |= (u32)((desc_chn_start >> 6) & 0xc0000000);
		if (axxia_rio_is_x9())
			dme_ctrl |= (u32)((desc_chn_start >> 12) & 0x0c000000);
		desc_addr  = (u32)desc_chn_start >> 4;

		me->dme_ctrl = dme_ctrl;
		me->letter = letter;
		me->mbox = mbox;
		priv->ib_dme[dme_no] = me;
		axxia_local_config_write(priv,
				RAB_IB_DME_DESC_ADDR(dme_no),
				desc_addr);
		axxia_local_config_write(priv,
				RAB_IB_DME_CTRL(dme_no), dme_ctrl);

#ifdef CONFIG_AXXIA_RIO_STAT
		me->desc_done_count = 0;
		me->desc_error_count = 0;
		me->desc_rio_err_count = 0;
		me->desc_axi_err_count = 0;
		me->desc_tmo_err_count = 0;
#endif
		select_dme(dme_no, &priv->num_inb_dmes[0],
				&priv->inb_dmes_in_use[0],
				&priv->inb_dmes[0], 1);
	}

	/**
	* Create irq handler and enable MBOX irq
	*/
	mb->irq_letter_state[letter] = irq_state_mask;
	mb->irq_state_mask |= irq_state_mask;
	h->irq_state_mask |= irq_state_mask;
	rcu_assign_pointer(priv->ib_mbox[mbox], mb);
	AXXIA_RIO_SYSMEM_BARRIER();
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, irq_state_mask);

	if (mbnew && (priv->dme_mode == AXXIA_IBDME_TIMER_MODE)) {
		hrtimer_init(&mb->tmr, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		mb->tmr.function = ib_dme_tmr_handler;
		hrtimer_start(&mb->tmr, ktime_set(0, (axxia_hrtimer_delay)),
					HRTIMER_MODE_REL_PINNED);
	} else
		axxia_local_config_write(priv, RAB_INTR_ENAB_IDME,
						h->irq_state_mask);
	mb->num_letter++;
	return 0;

err:
	mbox_put(mb);
	return rc;
}

/**
 * release_inb_mbox - Close AXXIA inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Caller must hold RAB lock
 * Release all resources i.e. DMEs, descriptors, buffers, and so on.
 */

static void release_inb_mbox(struct rio_irq_handler *h)
{
	struct rio_rx_mbox *mb = h->data;
/*TODO*/
	h->data = NULL;
	mbox_put(mb);
}

void axxia_rio_port_get_state(struct rio_mport *mport, int cleanup)
{
	struct rio_priv *priv = mport->priv;
	u32 escsr, iecsr, state;

	if (cleanup) {
#if defined(CONFIG_AXXIA_RIO_STAT)
		reset_state_counters(priv);
#endif
		/**
		 * Clear latched state indications
		 */
		/* Miscellaneous Events */
		axxia_local_config_read(priv, RAB_INTR_STAT_MISC, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_MISC, state);
		/* Outbound Message Engine */
		axxia_local_config_read(priv, RAB_INTR_STAT_ODME, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_ODME , state);
		/* Inbound Message Engine */
		axxia_local_config_read(priv, RAB_INTR_STAT_IDME, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_IDME, state);
		/* Axxi Bus to RIO Events */
		axxia_local_config_read(priv, RAB_INTR_STAT_APIO, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_APIO, state);
		/* RIO to Axxia Bus Events */
		axxia_local_config_read(priv, RAB_INTR_STAT_RPIO, &state);
		axxia_local_config_write(priv, RAB_INTR_STAT_RPIO, state);
	}

	/* Master Port state */
	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &escsr);
	axxia_local_config_read(priv, EPC_IECSR(priv->port_ndx), &iecsr);

	/* Adding I2E to preserve idle sequence select bit which is R/w */
	axxia_local_config_write(priv, RIO_ESCSR(priv->port_ndx),
				(escsr & (RIO_ESCSR_I2E | RIO_EXCSR_WOLR)));
}

/**
 * RIO MPORT Driver API
 */

/**
 * axxia_rio_port_irq_enable - Register RIO interrupt handler
 *
 * @mport: master port
 * @irq: IRQ mapping from DTB
 *
 * Caller must hold RAB lock
 *
 * Returns:
 * 0        Success
 * <0       Failure
 */
int axxia_rio_port_irq_enable(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	int rc;

	priv->dme_mode = axxia_dme_tmr_mode[priv->ndx];
	/**
	 * Clean up history
	 * from port reset/restart
	 */
	axxia_rio_port_get_state(mport, 1);
	rc = alloc_irq_handler(&priv->misc_irq, priv, "rio-misc-db");
	if (rc)
		goto out;

#if 0
	rc = alloc_irq_handler(&priv->apio_irq, priv, "rio-apio");
	if (rc)
		goto err2;
#endif

	rc = alloc_irq_handler(&priv->rpio_irq, priv, "rio-rpio");
	if (rc)
		goto err3;
	priv->ib_dme_irq.data = priv;
	priv->ob_dme_irq.data = priv;

	if (priv->dme_mode == AXXIA_IBDME_INTERRUPT_MODE) {
		rc = request_threaded_irq(priv->irq_line,
				hw_irq_dme_handler, NULL,
				IRQF_TRIGGER_NONE | IRQF_SHARED | IRQF_ONESHOT,
				"rio-mb", (void *)priv);
		if (rc)
			goto err4;

		axxia_local_config_write(priv, RAB_INTR_ENAB_GNRL,
				    (RAB_INTR_ENAB_GNRL_SET | IB_DME_INT_EN));
	} else
		axxia_local_config_write(priv, RAB_INTR_ENAB_GNRL,
				    RAB_INTR_ENAB_GNRL_SET);

	rc = alloc_irq_handler(&priv->linkdown_irq, priv, "rio-linkdown");
	if (rc)
		goto err4;
	/* init linkdown timer */
	axxia_local_config_write(priv, RAB_SRDS_CTRL2, LINK_DOWN_TIMEOUT);
	axxia_local_config_write(priv, RAB_SRDS_CTRL1, LINK_DOWN_IRQ_MASK);

out:
	return rc;
err0:
	dev_warn(priv->dev, "RIO: unable to request irq.\n");
	goto out;
err4:
	release_irq_handler(&priv->rpio_irq);
err3:
#if 0
	release_irq_handler(&priv->apio_irq);
#endif
	release_irq_handler(&priv->misc_irq);
	goto err0;
}

void axxia_rio_port_irq_disable(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	/**
	 * Mask top level IRQs
	 */
	axxia_local_config_write(priv, RAB_INTR_ENAB_GNRL, 0);
	/**
	 * free registered handlers
	 */
	release_irq_handler(&priv->misc_irq);
	release_irq_handler(&priv->ob_dme_irq);
	release_irq_handler(&priv->ib_dme_irq);
	release_irq_handler(&priv->apio_irq);
	release_irq_handler(&priv->rpio_irq);
}

int axxia_rio_pw_enable(struct rio_mport *mport, int enable)
{
	struct rio_priv *priv = mport->priv;
	int rc = 0;

	mutex_lock(&priv->api_lock);
	if (enable)
		rc = enable_pw(mport);
	else
		disable_pw(mport);
	mutex_unlock(&priv->api_lock);

	return rc;
}

/**
 * axxia_rio_doorbell_send - Send a doorbell message
 *
 * @mport: RapidIO master port info
 * @index: ID of RapidIO interface
 * @destid: Destination ID of target device
 * @data: 16-bit info field of RapidIO doorbell message
 *
 * Sends a doorbell message.
 *
 * Returns %0 on success or %-EINVAL on failure.
 */
int axxia_rio_doorbell_send(struct rio_mport *mport,
			    int index, u16 destid, u16 data)
{
	struct rio_priv *priv = mport->priv;
	int db;
	u32 csr;
	unsigned long flags;

	spin_lock_irqsave(&rio_doorbell_lock, flags);

	for (db = 0; db < MAX_OB_DB; db++) {
		axxia_local_config_read(priv, RAB_OB_DB_CSR(db), &csr);
		if (OB_DB_STATUS(csr) == OB_DB_STATUS_DONE &&
		    OB_DB_STATUS(csr) != OB_DB_STATUS_RETRY) {

			csr = 0;
			csr |= OB_DB_DEST_ID(destid);
			csr |= OB_DB_PRIO(0x2); /* Good prio? */
			csr |= OB_DB_SEND;
			dev_dbg(priv->dev,
			   "Send doorbell 0x%04x to destid 0x%x\n",
				data, destid);
			axxia_local_config_write(priv, RAB_OB_DB_INFO(db),
						    OB_DB_INFO(data));
			axxia_local_config_write(priv, RAB_OB_DB_CSR(db),
						    csr);
			break;
		}
	}

	spin_unlock_irqrestore(&rio_doorbell_lock, flags);

	if (db == MAX_OB_DB)
		return -EBUSY;

	return 0;
}

/************/
/* OUTBOUND */
/************/
/**
 * axxia_open_outb_mbox - Initialize AXXIA outbound mailbox
 * @mport: Master port implementing the outbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox_dme: Mailbox to open
 * @entries: Number of entries in the outbound DME/mailbox ring for
 *           each letter
 *
 * Allocates and initializes descriptors.
 * We have N (e.g. 3) outbound mailboxes and M (e.g. 1024) message
 * descriptors.  The message descriptors are usable by inbound and
 * outbound message queues, at least until the point of binding.
 * Allocation/Distribution of message descriptors is flexible and
 * not restricted in any way other than that they must be uniquely
 * assigned/coherent to each mailbox/DME.
 *
 * Allocate memory for messages.
 * Each descriptor can hold a message of up to 4kB, though certain
 * DMEs or mailboxes may impose further limits on the size of the
 * messages.
 *
 * Returns %0 on success and %-EINVAL or %-ENOMEM on failure.
 */
int axxia_open_outb_mbox(
	struct rio_mport *mport,
	void *dev_id,
	int mbox_dme,
	int entries/*,
	int prio*/)
{
	struct rio_priv *priv = mport->priv;
	int rc = 0;

	mutex_lock(&priv->api_lock);
	rc = open_outb_mbox_static(mport, dev_id, mbox_dme,
					entries, 0x0/*prio*/);
	mutex_unlock(&priv->api_lock);

	return rc;
}

/**
 * axxia_close_outb_mbox - Shut down AXXIA outbound mailbox
 *
 * @mport: Master port implementing the outbound message unit
 * @mbox_id: Mailbox to close
 *
 * Disables the outbound message unit, frees all buffers, and
 * frees any other resources.
 */
void axxia_close_outb_mbox(struct rio_mport *mport, int mbox_id)
{
	struct rio_priv *priv = mport->priv;
	int dme_no;
	int wait_cnt = 0;
	struct rio_msg_dme *me;
	struct rio_tx_mbox *mb = NULL;


	if ((mbox_id < 0) ||
	    (mbox_id > RIO_MAX_TX_MBOX))
		return;
	mb = priv->ob_mbox[mbox_id];
	if ((!mb) ||
	    (!test_bit(RIO_MB_OPEN, &mb->state)))
		return;
	me = mb->me;

	mutex_lock(&priv->api_lock);
	clear_bit(RIO_MB_OPEN, &priv->ob_mbox[mbox_id]->state);
	while (me->write_idx != me->read_idx) {
		msleep(20);
		wait_cnt++;
		if (wait_cnt > 250)
			break;
	}
	if (wait_cnt > 250)
		pr_debug("RIO: Closed outb mbox%d while transaction pending\n",
								mbox_id);
	priv->ob_mbox[mbox_id] = NULL;
	dme_no = mb->dme_no;
	mb->dme_no = 0xff;

	priv->ob_dme_shared[dme_no].ring_size -=
		mb->ring_size;

	priv->ob_dme_shared[dme_no].ring_size_free +=
		mb->ring_size;
	mb->dev_id = NULL;
	clear_bit(RIO_MB_MAPPED, &mb->state);
	kfree(mb);
	me->sz--;
	if (!(me->sz))
		hrtimer_cancel(&priv->ob_dme_shared[dme_no].tmr);
	mutex_unlock(&priv->api_lock);
	return;
}

static inline struct rio_msg_desc *get_ob_desc(struct rio_mport *mport,
						struct rio_msg_dme *mb)
{
	int desc_num = mb->write_idx;
	struct rio_priv *priv = mport->priv;
	struct rio_msg_desc *desc = &mb->desc[desc_num];
	int nxt_write_idx = (mb->write_idx + 1) & (mb->entries - 1);
	u32 dw0;
	if (nxt_write_idx != mb->read_idx) {
		dw0 = *((u32 *)DESC_TABLE_W0_MEM(mb, desc_num));
		if (!(dw0 & DME_DESC_DW0_VALID))
			return desc;
		else
			pr_err_ratelimited("SRIO: %s:Tx Desc error %d\n",
					dev_name(priv->dev), mb->write_idx);
	}
	return NULL;
}

/**
 * axxia_add_outb_message - Add message to the AXXIA outbound message queue
 * --- Called in net core soft IRQ with local interrupts masked ---
 * --- And spin locked in master port net device handler        ---
 *
 * @mport: Master port with outbound message queue
 * @rdev: Target of outbound message
 * @mbox_dest: Destination mailbox
 * @letter: TID letter
 * @flags: 3 bit field,Critical Request Field[2] | Prio[1:0]
 * @buffer: Message to add to outbound queue
 * @len: Length of message
 * @cookie: Opaque pointer, must be passed to completion callback
 *
 * Adds the @buffer message to the AXXIA outbound message queue.
 * Returns %0 on success
 *         %-EBUSY  on temporarily unavailable resource failure e.g. such
 *                     as waiting for an open entry in the outbound DME
 *                     descriptor chain
 *         %-EAGAIN on another kind of temporarily unavailable resource
 *                     failure
 *         %-EINVAL on invalid argument failure.
 *         %-ENODEV on unavailable resource failure e.g. no outbound DME
 *                     open that matches the kind of destination mailbox
 *         %-ENXIO  on incompatible argument failure e.g. trying to open
 *                     a single-segment mbox when none are available on
 *                     the platform
 */
int axxia_add_outb_message(struct rio_mport *mport, /*struct rio_dev *rdev*/u16 destid,
			     int mbox_dest, int letter, int flags,
			     void *buffer, size_t len, void *cookie)
{
	int rc = 0;
	u32 dw0, dw1;
/*	u16 destid = (rdev ? rdev->destid : mport->host_deviceid);*/
	struct rio_priv *priv = mport->priv;
	struct rio_tx_mbox *mb = priv->ob_mbox[mbox_dest];
	struct rio_msg_dme *me;
	struct rio_msg_desc *desc;
	size_t orig_len;
	u32 dw2_r, dw2;
	u32 idx;
	u32 lock = 0;

	if (!mb)
		return -EINVAL;
	me = mb->me;
	if (me->sz > 1)
		lock = 1;

	/* Choose a free descriptor in a critical section */
	if (lock)
		spin_lock(&me->lock);
	desc = get_ob_desc(mport, me);
	if (!desc) {
		rc = -EBUSY;
		goto done;
	}

	/* Copy and clear rest of buffer */
	if ((uintptr_t)buffer > PAGE_OFFSET) {
		memcpy(desc->msg_virt, buffer, len);
	} else {
		if (copy_from_user(desc->msg_virt, buffer, len)) {
			rc = -ENXIO;
			goto done;
		}
	}

	/*
	 * AXM defect AX7-67: pad outbound messages to minimum 706B,
	 * copy and clear padding bytes to avoid sending out random
	 * sensitive kernel memory content
	 */
	orig_len = len;
	if (len < RIO_MSG_MIN_OB_MSG_SIZE)
		len = RIO_MSG_MIN_OB_MSG_SIZE;
	memset(desc->msg_virt + orig_len, 0, ALIGN(len, 8) - orig_len);

	dw0 = DME_DESC_DW0_SRC_DST_ID(destid) |
	/*	DME_DESC_DW0_EN_INT|*/
		DME_DESC_DW0_VALID;

#if 0
	if (!(me->write_idx % 4))
		dw0 |=	DME_DESC_DW0_EN_INT;
#endif
	dw0 |= desc->last;/*DME_DESC_DW0_NXT_DESC_VALID;*/
	dw1 = DME_DESC_DW1_PRIO(flags) |
		DME_DESC_DW1_CRF(flags) |
		DME_DESC_DW1_SEG_SIZE_256 |
		DME_DESC_DW1_MSGLEN(len) |
		DME_DESC_DW1_XMBOX(mbox_dest) |
		DME_DESC_DW1_MBOX(mbox_dest) |
		DME_DESC_DW1_LETTER(letter);
	idx = me->write_idx;
	dw2_r  = *((u32 *)DESC_TABLE_W2_MEM(me, idx));
	dw2 = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
	dw2 = (dw2_r & 0xc0000000) | dw2;
	desc->kvirt = cookie;
	me->write_idx = (me->write_idx+1) & (me->entries - 1);
	*((u32 *)DESC_TABLE_W2_MEM(me, idx)) = dw2;
	*((u32 *)DESC_TABLE_W1_MEM(me, idx)) = dw1;
	AXXIA_RIO_SYSMEM_BARRIER();
	*((u32 *)DESC_TABLE_W0_MEM(me, idx)) = dw0;

	if (lock)
		spin_unlock(&me->lock);
	else
		AXXIA_RIO_SYSMEM_BARRIER();
	/* Start / Wake up - the stored state is used to avoid a Read */
	axxia_local_config_write(priv, RAB_OB_DME_CTRL(me->dme_no),
							me->dme_ctrl);

#ifdef CONFIG_AXXIA_RIO_STAT
	priv->ob_mbox[mbox_dest]->sent_msg_count++;
#endif
	return rc;
done:
	if (lock)
		spin_unlock(&me->lock);
	return rc;
}
EXPORT_SYMBOL(axxia_add_outb_message);
int axxia_ml_add_outb_message(struct rio_mport *mport, struct rio_dev *rdev,
			      int mbox_dest, void *buffer, size_t len,
			      void *cookie)
{
	return axxia_add_outb_message(mport, rdev->destid, mbox_dest, 0, 0,
				      buffer, len, cookie);
}

int axm56xx_add_outb_message(struct rio_mport *mport, u16 destid,
			     int mbox_dest, int letter, int flags,
			     void *buffer, size_t len, void *cookie)
{
	int rc = 0;
	u32 dw0, dw1;
/*	u16 destid = (rdev ? rdev->destid : mport->host_deviceid);*/
	struct rio_priv *priv = mport->priv;
	struct rio_tx_mbox *mb = priv->ob_mbox[mbox_dest];
	struct rio_msg_dme *me;
	struct rio_msg_desc *desc;
	size_t orig_len;
	u32 dw2_r, dw2, dw0_r = 0, dw0_tmp;
	u32 idx;
	u32 lock = 0;

	if (!mb)
		return -EINVAL;
	me = mb->me;
	if (me->sz > 1)
		lock = 1;

	/* Choose a free descriptor in a critical section */
	if (lock)
		spin_lock(&me->lock);
	desc = get_ob_desc(mport, me);
	if (!desc) {
		rc = -EBUSY;
		goto done;
	}


	/* Copy and clear rest of buffer */
	if ((uintptr_t)buffer > PAGE_OFFSET) {
		memcpy(desc->msg_virt, buffer, len);
	} else {
		if (copy_from_user(desc->msg_virt, buffer, len)) {
			rc = -ENXIO;
			goto done;
		}
	}

	/*
	 * AXM defect AX7-67: pad outbound messages to minimum 706B,
	 * copy and clear padding bytes to avoid sending out random
	 * sensitive kernel memory content
	 */
	orig_len = len;
	if (len < RIO_MSG_MIN_OB_MSG_SIZE)
		len = RIO_MSG_MIN_OB_MSG_SIZE;
	memset(desc->msg_virt + orig_len, 0, ALIGN(len, 8) - orig_len);

	dw0 = DME_DESC_DW0_SRC_DST_ID(destid) |
	/*	DME_DESC_DW0_EN_INT|*/
		DME_DESC_DW0_VALID;

#if 0
	if (!(me->write_idx % 4))
		dw0 |=	DME_DESC_DW0_EN_INT;
#endif
	dw0 |= desc->last;/*DME_DESC_DW0_NXT_DESC_VALID;*/
	dw1 = DME_DESC_DW1_PRIO(flags) |
		DME_DESC_DW1_CRF(flags) |
		DME_DESC_DW1_SEG_SIZE_256 |
		DME_DESC_DW1_MSGLEN(len) |
		DME_DESC_DW1_XMBOX(mbox_dest) |
		DME_DESC_DW1_MBOX(mbox_dest) |
		DME_DESC_DW1_LETTER(letter);
	idx = me->write_idx;
	dw0_r  = *((u32 *)DESC_TABLE_W0_MEM(me, idx));
	dw2_r  = *((u32 *)DESC_TABLE_W2_MEM(me, idx));
	dw0_tmp = (u32)(desc->msg_phys >> 38) & 0x3;
	dw0 |= (dw0_tmp << 12);
	dw2 = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
	dw2 = (dw2_r & 0xc0000000) | dw2;
	dw0 |= (dw0_r & 0x0000c000);
	desc->kvirt = cookie;
	me->write_idx = (me->write_idx+1) & (me->entries - 1);
	*((u32 *)DESC_TABLE_W2_MEM(me, idx)) = dw2;
	*((u32 *)DESC_TABLE_W1_MEM(me, idx)) = dw1;
	AXXIA_RIO_SYSMEM_BARRIER();
	*((u32 *)DESC_TABLE_W0_MEM(me, idx)) = dw0;

	if (lock)
		spin_unlock(&me->lock);
	else
		AXXIA_RIO_SYSMEM_BARRIER();
	/* Start / Wake up - the stored state is used to avoid a Read */
	axxia_local_config_write(priv, RAB_OB_DME_CTRL(me->dme_no),
							me->dme_ctrl);

#ifdef CONFIG_AXXIA_RIO_STAT
	priv->ob_mbox[mbox_dest]->sent_msg_count++;
#endif
	return rc;
done:
	if (lock)
		spin_unlock(&me->lock);
	return rc;
}


/**
 * axxia_open_inb_mbox - Initialize AXXIA inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @dev_id: Device specific pointer to pass on event
 * @mbox: Mailbox to open
 * @entries: Number of entries in the inbound mailbox ring
 *
 * Initializes buffer ring.  Set up descriptor ring and memory
 * for messages for all letters in the mailbox.
 * Returns %0 on success and %-EINVAL or %-ENOMEM on failure.
 */
int axxia_open_inb_mbox(struct rio_mport *mport, void *dev_id,
			int mbox, int entries)
{
	struct rio_priv *priv = mport->priv;
	int rc = 0;
	int letter;
	int dme_per_letter = NUM_DME_PER_MBOX/4;

	mutex_lock(&priv->api_lock);
	for (letter = 0; letter < RIO_MSG_MAX_LETTER; letter++) {
		rc = open_inb_mbox_let(mport, dev_id, mbox, letter, entries,
							dme_per_letter);
		if (rc < 0) {
			dev_err(priv->dev, "Failed %s mb %d let %d\n",
					__func__, mbox, letter);
			break;
		}
	}
	mutex_unlock(&priv->api_lock);

	return rc;
}

int axxia_open_inb_mbox_let(struct rio_mport *mport, void *dev_id,
			int mbox, int letter_mask, int entries)
{
	struct rio_priv *priv = mport->priv;
	int rc = 0;
	int i;
	int num_letter = 0;
	int num_dmes[RIO_MSG_MAX_LETTER];

	for (i = 0; i < RIO_MSG_MAX_LETTER; i++) {
		if (letter_mask & (1 << i))
			num_letter++;
		num_dmes[i] = priv->inb_dmes_map[mbox][i];
	}
	if (!num_letter)
		return -EINVAL;
	mutex_lock(&priv->api_lock);
	for (i = 0; i < RIO_MSG_MAX_LETTER; i++) {
		if (letter_mask & (1 << i))
			rc = open_inb_mbox_let(mport, dev_id, mbox, i,
						entries, num_dmes[i]);
	}
	mutex_unlock(&priv->api_lock);

	return rc;
}

/**
 * axxia_open_inb_mbox_let_unsafe - Opens inbound mailbox
 * This function is not thread safe and must be called
 * from inside an atomic context.
 */
int axxia_open_inb_mbox_let_unsafe(struct rio_mport *mport, void *dev_id,
			int mbox, int letter_mask, int entries)
{
	struct rio_priv *priv = mport->priv;
	int rc = 0;
	int i;
	int num_letter = 0;
	int num_dmes[RIO_MSG_MAX_LETTER];

	for (i = 0; i < RIO_MSG_MAX_LETTER; i++) {
		if (letter_mask & (1 << i))
			num_letter++;
		num_dmes[i] = priv->inb_dmes_map[mbox][i];
	}

	if (!num_letter)
		return -EINVAL;
	for (i = 0; i < RIO_MSG_MAX_LETTER; i++) {
		if (letter_mask & (1 << i))
			rc = open_inb_mbox_let(mport, dev_id, mbox, i,
						entries, num_dmes[i]);
	}

	return rc;
}

static void close_inb_mbox_let(struct rio_priv *priv, struct rio_rx_mbox *mb,
		int letter, void (*release_cb)(void *cookie))
{
	u32 dme_stat;
	u32 num_dme = 0;
	u32 dme_no;
	u32 dme;
	unsigned long irq_flags;

	mb->irq_state_mask &= ~(mb->irq_letter_state[letter]);
	priv->ib_dme_irq.irq_state_mask &= ~(mb->irq_letter_state[letter]);

	spin_lock_irqsave(&rio_rx_mbox_lock, irq_flags);
	axxia_local_config_write(priv, RAB_INTR_ENAB_IDME,
					priv->ib_dme_irq.irq_state_mask);
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME,
					mb->irq_letter_state[letter]);
	num_dme = mb->num_dme[letter];
	mb->num_dme[letter] = 0;
	spin_unlock_irqrestore(&rio_rx_mbox_lock, irq_flags);

	for (dme = 0; dme < num_dme; dme++) {
		int wait = 0;
		if (mb->me[letter][dme]) {
			dme_no = mb->me[letter][dme]->dme_no;
			do {
				axxia_local_config_read(priv,
					RAB_IB_DME_STAT(dme_no),
							&dme_stat);
				if (wait++ > 10000)
					break;
			} while (dme_stat & IB_DME_STAT_TRANS_PEND);
			if (wait > 10000)
				dev_err(priv->dev,
				"Closing while Transaction pending\n");
			axxia_local_config_write(priv,
				RAB_IB_DME_CTRL(dme_no), 0);
			dme_put(mb->me[letter][dme]);
			select_dme(dme_no,
					&priv->num_inb_dmes[0],
					&priv->inb_dmes_in_use[0],
					&priv->inb_dmes[0], 0);
			priv->ib_dme[dme_no] = NULL;
			mb->me[letter][dme] = NULL;
		}
	}
	if (release_cb) {
		int i;

		for (i = 0; i < mb->ring_size[letter]; i++)
			release_cb(mb->cookie[letter][i]);
	}
	kfree(mb->virt_buffer[letter]);
	kfree(mb->cookie[letter]);
	mb->virt_buffer[letter] = NULL;
	mb->cookie[letter] = NULL;
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME,
					mb->irq_letter_state[letter]);
	mb->irq_letter_state[letter] = 0;
	mb->num_letter--;

	return;
}

void axxia_close_inb_mbox_let(struct rio_mport *mport, int mbox,
		int letter_mask, void (*release_cb)(void *cookie))
{
	struct rio_priv *priv = mport->priv;
	struct rio_rx_mbox *mb;
	int i;

	mutex_lock(&priv->api_lock);

	if ((mbox < 0) || (mbox >= RIO_MAX_RX_MBOX))
		goto unlock;

	mb = ib_mbox(priv, mbox);
	if (!mb)
		goto unlock;

	rcu_assign_pointer(priv->ib_mbox[mbox], NULL);
	synchronize_rcu();

	for (i = 0; i < RIO_MSG_MAX_LETTER; i++) {
		if (letter_mask & (1 << i))
			close_inb_mbox_let(priv, mb, i, release_cb);
	}

	if (priv->dme_mode == AXXIA_IBDME_TIMER_MODE)
		hrtimer_cancel(&mb->tmr);

	mbox_put(mb);
unlock:	mutex_unlock(&priv->api_lock);

	return;

}
/**
 * axxia_close_inb_mbox - Shut down AXXIA inbound mailbox
 * @mport: Master port implementing the inbound message unit
 * @mbox: Mailbox to close
 *
 * Disables the inbound message unit, free all buffers, and
 * frees resources.
 */
void axxia_close_inb_mbox(struct rio_mport *mport, int mbox)
{
	struct rio_priv *priv = mport->priv;
	struct rio_rx_mbox *mb;
	u32 letter;
	u32 dme_stat;
	u32 dme_no;
	u32 dme;
	unsigned long irq_flags;

	if ((mbox < 0) || (mbox >= RIO_MAX_RX_MBOX))
		return;
	mutex_lock(&priv->api_lock);
	mb = ib_mbox(priv, mbox);
	if (mb == NULL) {
		mutex_unlock(&priv->api_lock);
		return;
	}
	priv->ib_dme_irq.irq_state_mask &= ~(mb->irq_state_mask);

	spin_lock_irqsave(&rio_rx_mbox_lock, irq_flags);
	axxia_local_config_write(priv, RAB_INTR_ENAB_IDME,
					priv->ib_dme_irq.irq_state_mask);
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, mb->irq_state_mask);
	spin_unlock_irqrestore(&rio_rx_mbox_lock, irq_flags);

	rcu_assign_pointer(priv->ib_mbox[mbox], NULL);
	synchronize_rcu();
	msleep(500);
	for (letter = 0; letter < RIO_MSG_MAX_LETTER; letter++) {
		for (dme = 0; dme < mb->num_dme[letter]; dme++) {
			int wait = 0;
			if (mb->me[letter][dme]) {
				dme_no = mb->me[letter][dme]->dme_no;
				do {
					axxia_local_config_read(priv,
						RAB_IB_DME_STAT(dme_no),
								&dme_stat);
					if (wait++ > 10000)
						break;
				} while (dme_stat & IB_DME_STAT_TRANS_PEND);
				if (wait > 10000)
					dev_err(priv->dev,
					"Closing while Transaction pending\n");
				axxia_local_config_write(priv,
					RAB_IB_DME_CTRL(dme_no), 0);
			}
		}
	}
	axxia_local_config_write(priv, RAB_INTR_STAT_IDME, mb->irq_state_mask);
	mb->irq_state_mask = 0;
	msleep(100);
	if (priv->dme_mode == AXXIA_IBDME_TIMER_MODE)
		hrtimer_cancel(&mb->tmr);
	mbox_put(mb);
	mutex_unlock(&priv->api_lock);
	return;
}

/**
 * axxia_add_inb_buffer - Add buffer to the AXXIA inbound message queue
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @buf: Buffer to add to inbound queue
 *
 * Adds the @buf buffer to the AXXIA inbound message queue.
 *
 * Returns %0 on success
 *         %-EINVAL on invalid argument failure.
 *         %-EBUSY  on temporarily unavailable resource failure e.g. such
 *                     as waiting for a filled entry in the inbound DME
 *                     descriptor chain
 */
int axxia_add_inb_buffer(struct rio_mport *mport, int mbox,
					int letter, void *buf, void *cookie)
{
	struct rio_priv *priv = mport->priv;
	struct rio_rx_mbox *mb;
	int rc = -EBUSY;
	struct rio_msg_dme *me;
	struct rio_msg_desc *desc;
	u32 dw0, dw2, dw2_r;
	u32 itr;

	mb = ib_mbox(priv, mbox);
	if (!mb)
		return -EINVAL;
	for (itr = 0; itr < mb->num_dme[letter]; itr++) {
		me = mb->me[letter][itr];
		/* Lockless circular buffer scheme */
		if (((me->write_idx + 1) % (me->entries)) == me->read_idx)
			continue;
		if (mb->virt_buffer[letter][mb->next_rx_slot[letter]]) {
			/* TODO Need to handle when DME encounters error */
			pr_debug("SRIO: %s:%s: Buffer already valid at %d\n",
				dev_name(priv->dev), __func__, me->write_idx);
			break;
		}

		dw0 = *((u32 *)DESC_TABLE_W0_MEM(me, me->write_idx));
		if (dw0 & DME_DESC_DW0_VALID) {
			pr_debug("SRIO: %s:Filling invalid buffer %d %x\n",
				dev_name(priv->dev), me->write_idx, dw0);
			break;
		}
		mb->virt_buffer[letter][mb->next_rx_slot[letter]] = buf;
		mb->cookie[letter][mb->next_rx_slot[letter]] = cookie;
		if ((mb->num_dme[letter] == 1) && (!((uintptr_t)buf & 0xFF))) {
			dw2_r = *((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx));
			dw2 = (u32)(virt_to_phys(buf) >> 8) & 0x3fffffff;
			dw2 = (dw2_r & 0xc0000000) | dw2;
			*((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx)) = dw2;
		} else {
			desc = &me->desc[me->write_idx];
			dw2_r = *((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx));
			dw2 = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
			dw2 = (dw2_r & 0xc0000000) | dw2;
			*((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx)) = dw2;
		}

		AXXIA_RIO_SYSMEM_BARRIER();
		dw0 |= DME_DESC_DW0_VALID;
		*((u32 *)DESC_TABLE_W0_MEM(me, me->write_idx)) = dw0;
		AXXIA_RIO_SYSMEM_BARRIER();
		me->write_idx = (me->write_idx + 1) % (me->entries);
		mb->next_rx_slot[letter] = (mb->next_rx_slot[letter] + 1) %
							mb->ring_size[letter];
		axxia_local_config_write(priv,
			RAB_IB_DME_CTRL(me->dme_no), me->dme_ctrl);
		rc = 0;
		break;
	}

	return rc;
}


int axxia_ml_add_inb_buffer(struct rio_mport *mport, int mbox,
		void *buf, void *cookie)
{
	int letter = 0;
	return axxia_add_inb_buffer(mport, mbox, letter, buf, cookie);

}

int axm56xx_add_inb_buffer(struct rio_mport *mport, int mbox,
		int letter, void *buf, void *cookie)
{
	struct rio_priv *priv = mport->priv;
	struct rio_rx_mbox *mb;
	int rc = -EBUSY;
	struct rio_msg_dme *me;
	struct rio_msg_desc *desc;
	u32 dw0, dw2, dw2_r, dw1 = 0, dw1_r;
	u32 itr;

	mb = ib_mbox(priv, mbox);
	if (!mb)
		return -EINVAL;
	for (itr = 0; itr < mb->num_dme[letter]; itr++) {
		me = mb->me[letter][itr];
		/* Lockless circular buffer scheme */
		if (((me->write_idx + 1) % (me->entries)) == me->read_idx)
			continue;
		if (mb->virt_buffer[letter][mb->next_rx_slot[letter]]) {
			/* TODO Need to handle when DME encounters error */
			pr_debug("SRIO: %s:%s: Buffer already valid at %d\n",
				dev_name(priv->dev), __func__, me->write_idx);
			break;
		}

		dw0 = *((u32 *)DESC_TABLE_W0_MEM(me, me->write_idx));
		if (dw0 & DME_DESC_DW0_VALID) {
			pr_debug("SRIO: %s:Filling invalid buffer %d %x\n",
				dev_name(priv->dev), me->write_idx, dw0);
			break;
		}
		mb->virt_buffer[letter][mb->next_rx_slot[letter]] = buf;
		mb->cookie[letter][mb->next_rx_slot[letter]] = cookie;
		if ((mb->num_dme[letter] == 1) && (!((uintptr_t)buf & 0xFF))) {
			dw1_r = *((u32 *)DESC_TABLE_W1_MEM(me, me->write_idx));
			dw2 = (u32)(virt_to_phys(buf) >> 8) & 0xc0000000;
			dw2 = (dw2 >> 9);
			dw1 = dw1_r & 0xff9fffff;
			dw1 |= dw2;
			dw2_r = *((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx));
			dw2 = (u32)(virt_to_phys(buf) >> 8) & 0x3fffffff;
			dw2 = (dw2_r & 0xc0000000) | dw2;
			*((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx)) = dw2;
			*((u32 *)DESC_TABLE_W1_MEM(me, me->write_idx)) = dw1;
		} else {
			desc = &me->desc[me->write_idx];
			dw1_r = *((u32 *)DESC_TABLE_W1_MEM(me, me->write_idx));
			dw2 = (u32)(desc->msg_phys >> 8) & 0xc0000000;
			dw2 = (dw2 >> 9);
			dw1 = dw1_r & 0xff9fffff;
			dw1 |= dw2;
			dw2_r = *((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx));
			dw2 = (u32)(desc->msg_phys >> 8) & 0x3fffffff;
			dw2 = (dw2_r & 0xc0000000) | dw2;
			*((u32 *)DESC_TABLE_W2_MEM(me, me->write_idx)) = dw2;
			*((u32 *)DESC_TABLE_W1_MEM(me, me->write_idx)) = dw1;
		}

		AXXIA_RIO_SYSMEM_BARRIER();
		dw0 |= DME_DESC_DW0_VALID;
		*((u32 *)DESC_TABLE_W0_MEM(me, me->write_idx)) = dw0;
		AXXIA_RIO_SYSMEM_BARRIER();
		me->write_idx = (me->write_idx + 1) % (me->entries);
		mb->next_rx_slot[letter] = (mb->next_rx_slot[letter] + 1) %
							mb->ring_size[letter];
		axxia_local_config_write(priv,
			RAB_IB_DME_CTRL(me->dme_no), me->dme_ctrl);
		rc = 0;
		break;
	}

	return rc;
}


void axxia_inb_timeout_handler(struct rio_mport *mport, int mbox, int letter,
								int dme)
{
	struct rio_priv *priv = mport->priv;
	struct rio_rx_mbox *mb;
	struct rio_msg_dme *me;
	u32 idx;
	int i;
	void *buf = NULL;

	mb = ib_mbox(priv, mbox);
	if (!mb)
		return;

	me = mb->me[letter][dme];
	axxia_local_config_write(priv,
			RAB_IB_DME_CTRL(me->dme_no), 0);
	/* Reconfigure the DME Descriptors */
	idx = me->read_idx;
	if (mb->num_dme[letter] == 1) {
		void *my_buff[me->entries];
		void *cookies[me->entries];
		for (i = 0; i < me->entries; i++) {
			my_buff[i] = mb->virt_buffer[letter][idx];
			cookies[i] = mb->cookie[letter][idx];
			mb->virt_buffer[letter][idx] = NULL;
			mb->cookie[letter][idx] = NULL;
			idx = (idx + 1) % (me->entries);
		}
		invalidate_dme_descriptors(me);
		me->read_idx = 0;
		me->write_idx = 0;
		mb->next_rx_slot[letter] = 0;
		mb->last_rx_slot[letter] = 0;
		while (((me->write_idx + 1) % (me->entries)) !=
							 me->read_idx) {
			idx = me->write_idx;
			buf = my_buff[idx];
			if (!buf) {
				dev_dbg(priv->dev, "Null Buff %d\n",
							me->write_idx);
				break;
			} else {
				if (axxia_add_inb_buffer(mport, mbox, letter,
							buf, cookies[idx]) != 0)
					break;
			}
		}
	} else {
		i = dme_count_valid_desc(me);
		me->read_idx = 0;
		me->write_idx = 0;
		invalidate_dme_descriptors(me);
		revalidate_dme_descriptors(me, i);
	}
	AXXIA_RIO_SYSMEM_BARRIER();
	/* Enable the DME */
	axxia_local_config_write(priv,
			RAB_IB_DME_CTRL(me->dme_no), me->dme_ctrl);
}
/**
 * axxia_get_inb_message - Fetch an inbound message from the AXXIA
 *                         message unit
 * @mport: Master port implementing the inbound message unit
 * @mbox: Inbound mailbox number
 * @letter: Inbound mailbox letter
 * @sz: size of returned buffer
 *
 * Gets the next available inbound message from the inbound message queue.
 *
 * Returns pointer to the message on success
 *         NULL on nothing available
 *         IS_ERR(ptr) on failure with extra information
 */
void *axxia_get_inb_message(struct rio_mport *mport, int mbox, int letter,
			    int *sz,/* int *slot,*/ int *destid, void **cookie)
{
	struct rio_priv *priv = mport->priv;
	struct rio_rx_mbox *mb;
	struct rio_msg_dme *me;
	void *buf = NULL;
	u32 idx;
	u32 itr;

	mb = ib_mbox(priv, mbox);
	if (!mb)
		return NULL;
	for (itr = 0; itr < mb->num_dme[letter]; itr++) {
		struct rio_msg_desc *desc;
		u32 dw0, dw1;
		buf = NULL;
		*sz = 0;
		me = mb->me[letter][itr];
		idx = me->read_idx;
		desc = &me->desc[me->read_idx];
		dw0 = *((u32 *)DESC_TABLE_W0_MEM(me, idx));
		dw1 = *((u32 *)DESC_TABLE_W1_MEM(me, idx));
		if ((dw0 & DME_DESC_DW0_ERROR_MASK) &&
		    (dw0 & DME_DESC_DW0_VALID)) {
			__dme_dw_dbg(priv->dev, me, 0, dw0, dw1);
			pr_err_ratelimited("SRIO: Desc error %x\n", dw0);
			axxia_inb_timeout_handler(mport, mbox, letter, itr);
			goto done;
		} else if ((dw0 & DME_DESC_DW0_DONE) &&
			   (dw0 & DME_DESC_DW0_VALID)) {
			int seg, buf_sz, sid, rx_slot;
			__dme_dw_dbg(priv->dev, me, 0, dw0, dw1);
			AXXIA_RIO_SYSMEM_BARRIER();
			seg = DME_DESC_DW1_MSGLEN_F(dw1);
			buf_sz = DME_DESC_DW1_MSGLEN_B(seg);
			sid = DME_DESC_DW0_GET_DST_ID(dw0);
			rx_slot = mb->last_rx_slot[letter];
			buf = mb->virt_buffer[letter][rx_slot];
			if (cookie)
				*cookie = mb->cookie[letter][rx_slot];
			if (!buf) {
				pr_err_ratelimited(
				"SRIO: %s:Buffer Error mb %d idx %d %d\n",
				dev_name(priv->dev), mbox, me->write_idx,
				me->read_idx);
				goto err;
			}

			if ((mb->num_dme[letter] > 1) ||
				((uintptr_t)buf & 0xFF)) {
				AXXIA_RIO_SYSMEM_BARRIER();
				memcpy(buf, desc->msg_virt, buf_sz);
			}
			mb->virt_buffer[letter][rx_slot] = NULL;
			mb->cookie[letter][rx_slot] = NULL;
			*((u32 *)DESC_TABLE_W0_MEM(me, idx)) =
					(dw0 & 0xfe);/*DME_DESC_INVALIDATE*/
			*sz = buf_sz;
			*destid = sid;
			me->read_idx = (me->read_idx + 1) % (me->entries);
			mb->last_rx_slot[letter] = (mb->last_rx_slot[letter] +
						1) % mb->ring_size[letter];
			goto done;
		} else {
			continue;
		}
	}

done:
	return buf;
err:
	buf = NULL;
	goto done;
}
EXPORT_SYMBOL(axxia_get_inb_message);

void *axxia_ml_get_inb_message(struct rio_mport *mport, int mbox, void **cookie)
{
	int sz, did;
	return axxia_get_inb_message(mport, mbox, 0, &sz, &did, cookie);
}

void axxia_rio_port_irq_init(struct rio_mport *mport)
{
	struct rio_priv *priv = mport->priv;
	int i;

	/**
	 * Port general error indications
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->misc_irq.state);
	priv->misc_irq.irq_enab_reg_addr = RAB_INTR_ENAB_MISC;
	priv->misc_irq.irq_state_reg_addr = RAB_INTR_STAT_MISC;
	priv->misc_irq.irq_state_mask = AMST_INT | ASLV_INT |
					LINK_REQ_INT;
	priv->misc_irq.irq_state_mask |= IB_DB_RCV_INT |
					OB_DB_DONE_INT;
	priv->misc_irq.irq_state_mask |= PORT_WRITE_INT;
	priv->misc_irq.irq_state_mask |=
		GRIO_INT | LL_TL_INT |
		UNSP_RIO_REQ_INT | UNEXP_MSG_INT;

	priv->misc_irq.thrd_irq_fn = misc_irq_handler;
	priv->misc_irq.data = NULL;
	priv->misc_irq.release_fn = misc_release_handler;


	/**
	 * Deadman Monitor status interrupt
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->linkdown_irq.state);
	priv->linkdown_irq.irq_enab_reg_addr = 0;
	priv->linkdown_irq.irq_state_reg_addr = RAB_SRDS_STAT1;
	priv->linkdown_irq.irq_state_mask = RAB_SRDS_STAT1_LINKDOWN_INT;
	priv->linkdown_irq.thrd_irq_fn = linkdown_irq_handler;
	priv->linkdown_irq.data = NULL;
	priv->linkdown_irq.release_fn = NULL;

	/**
	 * Outbound messages
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->ob_dme_irq.state);
	priv->ob_dme_irq.irq_enab_reg_addr = RAB_INTR_ENAB_ODME;
	priv->ob_dme_irq.irq_state_reg_addr = RAB_INTR_STAT_ODME;
	priv->ob_dme_irq.irq_state_mask = 0;
/*	priv->ob_dme_irq.thrd_irq_fn = ob_dme_irq_handler;*/
	priv->ob_dme_irq.data = NULL;
	priv->ob_dme_irq.release_fn = release_outb_dme;

	for (i = 0; i < RIO_MAX_TX_MBOX; i++)
		priv->ob_mbox[i] = NULL;

/* Pre-Allocating the Outbound DME Descriptors*/
/* MultiSegment DME*/
	for (i = 0; i < priv->num_outb_dmes[0]; i++)
		alloc_ob_dme_shared(priv, &priv->ob_dme_shared[i], i);
/* SingleSegment DME*/
	for (i = priv->num_outb_dmes[0];
		i < priv->num_outb_dmes[0] + priv->num_outb_dmes[1]; i++) {
		alloc_ob_dme_shared(priv, &priv->ob_dme_shared[i], i);
	}

	/**
	 * Inbound messages
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->ib_dme_irq.state);
	priv->ib_dme_irq.irq_enab_reg_addr = RAB_INTR_ENAB_IDME;
	priv->ib_dme_irq.irq_state_reg_addr = RAB_INTR_STAT_IDME;
	priv->ib_dme_irq.irq_state_mask = 0x0;/*IB_DME_INT_EN;*/
	priv->ib_dme_irq.thrd_irq_fn = ib_dme_irq_handler;
	priv->ib_dme_irq.data = NULL;
	priv->ib_dme_irq.release_fn = release_inb_mbox;

	for (i = 0; i < DME_MAX_IB_ENGINES; i++)
		priv->ib_dme[i] = NULL;

	for (i = 0; i < RIO_MAX_RX_MBOX; i++)
		RCU_INIT_POINTER(priv->ib_mbox[i], NULL);

	/**
	 * PIO
	 * Only when debug config
	 */
	clear_bit(RIO_IRQ_ENABLED, &priv->apio_irq.state);
/*	priv->apio_irq.mport = mport;*/
	priv->apio_irq.irq_enab_reg_addr = RAB_INTR_ENAB_APIO;
	priv->apio_irq.irq_state_reg_addr = RAB_INTR_STAT_APIO;
	priv->apio_irq.irq_state_mask = APIO_TRANS_FAILED;
#ifdef CONFIG_AXXIA_RIO_STAT
	priv->apio_irq.irq_state_mask |= APIO_TRANS_COMPLETE;
#endif
	priv->apio_irq.thrd_irq_fn = apio_irq_handler;
	priv->apio_irq.data = NULL;
	priv->apio_irq.release_fn = NULL;

	clear_bit(RIO_IRQ_ENABLED, &priv->rpio_irq.state);
	priv->rpio_irq.irq_enab_reg_addr = RAB_INTR_ENAB_RPIO;
	priv->rpio_irq.irq_state_reg_addr = RAB_INTR_STAT_RPIO;
	priv->rpio_irq.irq_state_mask = RPIO_TRANS_FAILED;
#ifdef CONFIG_AXXIA_RIO_STAT
	priv->rpio_irq.irq_state_mask |= RPIO_TRANS_COMPLETE;
#endif
	priv->rpio_irq.irq_state_mask = 0;
	priv->rpio_irq.thrd_irq_fn = rpio_irq_handler;
	priv->rpio_irq.data = NULL;
	priv->rpio_irq.release_fn = NULL;

}

#if defined(CONFIG_RAPIDIO_HOTPLUG)
int axxia_rio_port_notify_cb(struct rio_mport *mport,
			       int enable,
			       void (*cb)(struct rio_mport *mport))
{
	struct rio_priv *priv = mport->priv;
	unsigned long flags;
	int rc = 0;

	spin_lock_irqsave(&priv->port_lock, flags);
	if (enable) {
		if (priv->port_notify_cb)
			rc = -EBUSY;
		else
			priv->port_notify_cb = cb;
	} else {
		if (priv->port_notify_cb != cb)
			rc = -EINVAL;
		else
			priv->port_notify_cb = NULL;
	}
	spin_unlock_irqrestore(&priv->port_lock, flags);

	return rc;
}

int axxia_rio_port_op_state(struct rio_mport *mport)
{
	u32 escsr;

	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &escsr);

	if (escsr & RIO_ESCSR_PO)
		return MPORT_STATE_OPERATIONAL;
	else
		return MPORT_STATE_DOWN;
}
#endif

static int apb2ser0_write_32(void __iomem *base_addr,
							 uint32_t serdes_reg_addr,
							 uint32_t value)
{
	void __iomem *ptr = (void __iomem *) (base_addr + APB2SER0_CMD0_REG_OFS);
	uint32_t reg_value = 0;
	int ret_val = -EINVAL;

	writel(value, ptr);

	ptr = (void __iomem *) (base_addr + APB2SER0_CMD1_REG_OFS);
	reg_value = readl(ptr);

	SET_FLD(reg_value, APB2SER0_CMD1_VALID_FLD,  1);
	SET_FLD(reg_value, APB2SER0_CMD1_HWRITE_FLD, 1);
	SET_FLD(reg_value, APB2SER0_CMD1_TSHIFT_FLD, 1);
	SET_FLD(reg_value, APB2SER0_CMD1_HSIZE_FLD,  2);
	SET_FLD(reg_value, APB2SER0_CMD1_HTRANS_FLD, 2);
	SET_FLD(reg_value, APB2SER0_CMD1_HADDR_FLD,  serdes_reg_addr);

	writel(reg_value, ptr); /* writing operation starts here */

	while (1) {
		reg_value = readl(ptr);
		reg_value = GET_FLD(reg_value, APB2SER0_CMD1_VALID_FLD);
		if (0 == reg_value)
			break;

		udelay(100);
	}

	ptr = (void __iomem *) (base_addr + APB2SER0_DATA1_REG_OFS);

	ret_val = readl(ptr);
	ret_val = GET_FLD(ret_val, APB2SER0_DATA1_HRESP_FLD);

	return ret_val;
}

static int apb2ser0_read_32(void __iomem *base_addr,
							uint32_t serdes_reg_addr,
							uint32_t *value)
{
	void __iomem *ptr = (void __iomem *) (base_addr + APB2SER0_CMD1_REG_OFS);
	uint32_t reg_value = 0;
	int ret_val = -EINVAL;

	reg_value = readl(ptr);

	SET_FLD(reg_value, APB2SER0_CMD1_VALID_FLD,  1);
	SET_FLD(reg_value, APB2SER0_CMD1_HWRITE_FLD, 0);
	SET_FLD(reg_value, APB2SER0_CMD1_TSHIFT_FLD, 1);
	SET_FLD(reg_value, APB2SER0_CMD1_HSIZE_FLD,  2);
	SET_FLD(reg_value, APB2SER0_CMD1_HTRANS_FLD, 2);
	SET_FLD(reg_value, APB2SER0_CMD1_HADDR_FLD,  serdes_reg_addr);

	writel(reg_value, ptr); /* read operation starts here */

	while (1) {
		reg_value = readl(ptr);
		reg_value = GET_FLD(reg_value, APB2SER0_CMD1_VALID_FLD);
		if (0 == reg_value)
			break;
		udelay(100);
	}

	ptr = (void __iomem *) (base_addr + APB2SER0_DATA1_REG_OFS);

	ret_val = readl(ptr);
	ret_val = GET_FLD(ret_val, APB2SER0_DATA1_HRESP_FLD);
	if (0 == ret_val) {
		ptr = (void __iomem *) (base_addr + APB2SER0_DATA0_REG_OFS);
		*value = readl(ptr);
	}

	return ret_val;
}
