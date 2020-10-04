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

/* #define DEBUG */
/* #define IO_OPERATIONS */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/slab.h>
#include <linux/platform_device.h>

#include "axxia-rio.h"
#include "axxia-rio-irq.h"

static ssize_t axxia_rio_stat_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;
	u32 reg_val = 0;

	if (priv->devid  == AXXIA_DEVID_AXM55XX) {
		str += sprintf(str, "AXM 55xx sRIO Controller");
		switch (priv->devrev) {
		case AXXIA_DEVREV_AXM55XX_V1_0:
			str += sprintf(str, "Revision 0\n");
			break;
		case AXXIA_DEVREV_AXM55XX_V1_1:
			str += sprintf(str, "Revision 1\n");
			break;
		case AXXIA_DEVREV_AXM55XX_V1_2:
			str += sprintf(str, "Revision 2\n");
			break;
		default:
			str += sprintf(str, "Revision Unknown\n");
			break;
		}
	}

	axxia_rio_port_get_state(mport, 0);
	str += sprintf(str, "Master Port state:\n");
	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &reg_val);
	str += sprintf(str, "ESCSR (0x158) : 0x%08x\n", reg_val);
	return str - buf;
}
static DEVICE_ATTR(stat, S_IRUGO, axxia_rio_stat_show, NULL);

ssize_t axxia_rio_misc_stat_show_timing(char *str,
		const char *name,
		struct rio_time_stat *st)
{
	unsigned long long accu = (unsigned long long)ktime_to_ns(st->accu);
	if (st->count)
		do_div(accu, st->count);
	else
		accu = 0;
	return sprintf(str, "\t %s: count=%llu avg=%lluns peak=%lluns\n",
						name,
						st->count,
						accu,
						(unsigned long long)ktime_to_ns(st->peak));
}

static ssize_t axxia_rio_misc_stat_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;

	str += sprintf(str, "RIO PIO Stat:\n");
	str += sprintf(str, "\t Successful Count: %d\n",
					priv->rpio_compl_count);
	str += sprintf(str, "\t Failed Count    : %d\n",
					priv->rpio_compl_count);

	str += sprintf(str, "AXI PIO Stat:\n");
	str += sprintf(str, "\t Successful Count: %d\n",
					priv->apio_compl_count);
	str += sprintf(str, "\t Failed Count    : %d\n",
					priv->apio_failed_count);

	str += sprintf(str, "Port Write Stat:\n");
	str += sprintf(str, "\t Interrupt Count : %d\n", priv->rio_pw_count);
	str += sprintf(str, "\t Message Count   : %d\n",
					priv->rio_pw_msg_count);
	str += sprintf(str, "\t Spurious Count   : %d\n",
					priv->rio_pw_spurious);

	str += sprintf(str, "Maint Stat:\n");

	str += axxia_rio_misc_stat_show_timing(str, "loc rd", &priv->maint_local_read);
	str += axxia_rio_misc_stat_show_timing(str, "loc wr", &priv->maint_local_write);
	str += axxia_rio_misc_stat_show_timing(str, "rem rd", &priv->maint_remote_read);
	str += axxia_rio_misc_stat_show_timing(str, "rem wr", &priv->maint_remote_write);

	return str - buf;

}
static DEVICE_ATTR(misc_stat, S_IRUGO,
		   axxia_rio_misc_stat_show, NULL);
static ssize_t axxia_rio_ib_dme_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;
	int e, j, k;
	struct rio_rx_mbox *mb;
	struct rio_msg_dme *me;
	str += sprintf(str, "Inbound Mailbox (DME) counters:\n");
	str += sprintf(str, "%-4s%-4s%-4s%-11s%-11s%-11s%-11s%-11s\n",
			"DME", "MBX", "LET",
			"DescDone", "DescErr", "RIOErr", "AXIErr", "TimeoutErr");

	rcu_read_lock();
	for (e = 0; e < RIO_MAX_RX_MBOX; e++) {
		mb = rcu_dereference(priv->ib_mbox[e]);
		if (!mb)
			continue;

		for (j = 0; j < RIO_MSG_MAX_LETTER; j++) {
			for (k = 0; k < mb->num_dme[j]; k++) {
				me = mb->me[j][k];
				str += sprintf(str, "%-4d%-4d%-4d%-11u%-11u%-11u%-11u%-11u\n",
						me->dme_no, mb->mbox_no, j,
						me->desc_done_count,
						me->desc_error_count,
						me->desc_rio_err_count,
						me->desc_axi_err_count,
						me->desc_tmo_err_count);
			}
		}
	}
	rcu_read_unlock();
	return str - buf;
}
static DEVICE_ATTR(ib_dme_stat, S_IRUGO,
		   axxia_rio_ib_dme_show, NULL);

#ifdef CONFIG_AXXIA_RIO_BURST_STAT


static void axxia_rio_ib_dme_burst_reset(struct rio_priv *priv)
{
	int e, j, k;

	rcu_read_lock();
	for (e = 0; e < RIO_MAX_RX_MBOX; e++) {
		struct rio_rx_mbox *mb = rcu_dereference(priv->ib_mbox[e]);
		if (!mb)
			continue;

		for (j = 0; j < RIO_MSG_MAX_LETTER; j++) {
			for (k = 0; k < mb->num_dme[j]; k++) {
				mb->me[j][k]->burst.count = 0;
				mb->me[j][k]->burst.time = ktime_set(0, 0);
			}
		}
	}
	rcu_read_unlock();

	priv->burst.count = 0;
	priv->burst.time = ktime_set(0, 0);

	for (e = 0; e < ARRAY_SIZE(priv->burst.hist); e++) {
		priv->burst.hist[e] = 0;
	}
}

static ssize_t axxia_rio_ib_dme_burst_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf,  size_t count)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	ssize_t rtn = count;
	unsigned long flags;

	spin_lock_irqsave(&priv->burst.lock, flags);
	if (!strncmp("reset\n", buf, count)) {
		axxia_rio_ib_dme_burst_reset(priv);
	} else if (!strncmp("reset_on_read\n", buf, count)) {
		priv->burst.reset_on_read = 1;
	} else if (!strncmp("no_reset_on_read\n", buf, count)) {
		priv->burst.reset_on_read = 0;
	} else {
		rtn = -EINVAL;
	}
	spin_unlock_irqrestore(&priv->burst.lock, flags);

	return rtn;
}

static ssize_t axxia_rio_ib_dme_burst_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	unsigned long flags;
	struct timeval tv;
	char *str = buf;
	int e, j, k;

	spin_lock_irqsave(&priv->burst.lock, flags);

	tv = ktime_to_timeval(priv->burst.time);
	str += sprintf(str, "period: %u\n", AXXIA_BURST_STAT_PERIOD);
	str += sprintf(str, "max: %u\n", priv->burst.count);
	str += sprintf(str, "time: %lu.%03lu\n\n", tv.tv_sec, tv.tv_usec/1000000);

	str += sprintf(str, "hist_0: %u\n", priv->burst.hist[0]);
	for (e = 1; e < ARRAY_SIZE(priv->burst.hist); e++) {
		str += sprintf(str, "hist_%d: %u\n", 1 << (e - 1),
				priv->burst.hist[e]);
	}
	str += sprintf(str, "\n");

	rcu_read_lock();
	for (e = 0; e < RIO_MAX_RX_MBOX; e++) {
		struct rio_rx_mbox *mb = rcu_dereference(priv->ib_mbox[e]);
		if (!mb)
			continue;

		for (j = 0; j < RIO_MSG_MAX_LETTER; j++) {
			for (k = 0; k < mb->num_dme[j]; k++) {
				struct rio_msg_dme *me = mb->me[j][k];
				tv = ktime_to_timeval(me->burst.time);
				str += sprintf(str, "dme_%02d_max: %u\n"
						"dme_%02d_time: %lu.%03lu\n",
						me->dme_no, me->burst.count,
						me->dme_no, tv.tv_sec,
						tv.tv_usec/1000000);
			}
		}
	}
	rcu_read_unlock();

	if (priv->burst.reset_on_read) {
		axxia_rio_ib_dme_burst_reset(priv);
	}

	spin_unlock_irqrestore(&priv->burst.lock, flags);

	return str - buf;
}

static DEVICE_ATTR(ib_dme_burst_stat, 0644, axxia_rio_ib_dme_burst_show,
		axxia_rio_ib_dme_burst_store);
#endif

static ssize_t axxia_rio_ob_dme_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	char *str = buf;
	int e;
	struct rio_msg_dme *me;
	struct rio_tx_mbox *mb;

	str += sprintf(str, "Outbound Message Engine Counters:\n");
	for (e = 0; e < DME_MAX_OB_ENGINES; e++) {
		me = priv->ob_dme_shared[e].me;
		if (me) {
			str += sprintf(str, "DME %d Enabled\n", e);
			str += sprintf(str, "\tNumber of Desc Done  : %d\n",
					me->desc_done_count);
			str += sprintf(str, "\tNumber of Desc Errors: %d\n",
					me->desc_error_count);
			str += sprintf(str, "\t\tRIO Error    : %d\n",
					me->desc_rio_err_count);
			str += sprintf(str, "\t\tAXI Error    : %d\n",
					me->desc_axi_err_count);
			str += sprintf(str, "\t\tTimeout Error: %d\n",
					me->desc_tmo_err_count);
		} else
			str += sprintf(str, "DME %d Disabled\n", e);
	}
	str += sprintf(str, "*********************************\n");
	str += sprintf(str, "Outbound Mbox stats\n");
	for (e = 0; e < RIO_MAX_TX_MBOX; e++) {
		mb = priv->ob_mbox[e];
		if (!mb)
			continue;
		if ((mb->sent_msg_count) || (mb->compl_msg_count)) {
			if (test_bit(RIO_DME_OPEN, &mb->state))
				str += sprintf(str, "Mailbox %d: DME %d\n",
							e, mb->dme_no);
			else
				str += sprintf(str, "Mailbox %d : Closed\n",
							e);
			str += sprintf(str, "\tMessages sent     : %d\n",
						mb->sent_msg_count);
			str += sprintf(str, "\tMessages Completed: %d\n",
						mb->compl_msg_count);
		}
	}

	return str - buf;
}
static DEVICE_ATTR(ob_dme_stat, S_IRUGO,
		   axxia_rio_ob_dme_show, NULL);

static ssize_t axxia_rio_irq_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	u32 stat;
	char *str = buf;

	str += sprintf(str, "Interrupt enable bits:\n");
	axxia_local_config_read(priv, RAB_INTR_ENAB_GNRL, &stat);
	str += sprintf(str, "General Interrupt Enable (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_GNRL, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_ODME, &stat);
	str += sprintf(str, "Outbound Message Engine  (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_ODME, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_IDME, &stat);
	str += sprintf(str, "Inbound Message Engine   (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_IDME, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_MISC, &stat);
	str += sprintf(str, "Miscellaneous Events     (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_MISC, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_APIO, &stat);
	str += sprintf(str, "Axxia Bus to RIO Events  (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_APIO, stat);
	axxia_local_config_read(priv, RAB_INTR_ENAB_RPIO, &stat);
	str += sprintf(str, "RIO to Axxia Bus Events  (%p)\t%8.8x\n",
		       (void *)RAB_INTR_ENAB_RPIO, stat);

	str += sprintf(str, "OBDME : in Timer Mode, Period %9.9d nanosecond\n",
			axxia_hrtimer_delay);
	str += sprintf(str, "IBDME : ");
	if (priv->dme_mode == AXXIA_IBDME_TIMER_MODE)
		str += sprintf(str, "in Timer Mode, Period %9.9d nanosecond\n",
			axxia_hrtimer_delay);
	else
		str += sprintf(str, "in Interrupt Mode\n");
	return str - buf;
}
static DEVICE_ATTR(irq, S_IRUGO, axxia_rio_irq_show, NULL);

static ssize_t axxia_rio_tmo_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	u32 stat;
	char *str = buf;

	str += sprintf(str, "Port Link Timeout Control Registers:\n");
	axxia_local_config_read(priv, RIO_PLTOCCSR, &stat);
	str += sprintf(str, "PLTOCCSR (%p)\t%8.8x\n",
		       (void *)RIO_PLTOCCSR, stat);
	axxia_local_config_read(priv, RIO_PRTOCCSR, &stat);
	str += sprintf(str, "PRTOCCSR (%p)\t%8.8x\n",
		       (void *)RIO_PRTOCCSR, stat);
	axxia_local_config_read(priv, RAB_STAT, &stat);
	str += sprintf(str, "RAB_STAT (%p)\t%8.8x\n",
		       (void *)RAB_STAT, stat);
	axxia_local_config_read(priv, RAB_APIO_STAT, &stat);
	str += sprintf(str, "RAB_APIO_STAT (%p)\t%8.8x\n",
		       (void *)RAB_APIO_STAT, stat);
	axxia_local_config_read(priv, RIO_ESCSR(priv->port_ndx), &stat);
	str += sprintf(str, "PNESCSR (%d)\t%8.8x\n",
		       RIO_ESCSR(priv->port_ndx), stat);

	return str - buf;
}
static DEVICE_ATTR(tmo, S_IRUGO, axxia_rio_tmo_show, NULL);

static ssize_t axxia_ib_dme_log_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct rio_mport *mport = dev_get_drvdata(dev);
	struct rio_priv *priv = mport->priv;
	u32 stat, log;
	char *str = buf;

	axxia_local_config_read(priv, RAB_INTR_STAT_MISC, &stat);
	log = (stat & UNEXP_MSG_LOG) >> 24;
	str += sprintf(str, "mbox[1:0]   %x\n", (log & 0xc0) >> 6);
	str += sprintf(str, "letter[1:0] %x\n", (log & 0x30) >> 4);
	str += sprintf(str, "xmbox[3:0] %x\n", log & 0x0f);

	return str - buf;
}
static DEVICE_ATTR(dme_log, S_IRUGO, axxia_ib_dme_log_show, NULL);

static struct attribute *rio_attributes[] = {
	&dev_attr_stat.attr,
	&dev_attr_irq.attr,
	&dev_attr_misc_stat.attr,
	&dev_attr_ob_dme_stat.attr,
	&dev_attr_ib_dme_stat.attr,
#ifdef CONFIG_AXXIA_RIO_BURST_STAT
	&dev_attr_ib_dme_burst_stat.attr,
#endif
	&dev_attr_tmo.attr,
	&dev_attr_dme_log.attr,
	NULL
};

static struct attribute_group rio_attribute_group = {
	.name = NULL,
	.attrs = rio_attributes,
};

int axxia_rio_init_sysfs(struct platform_device *dev)
{
	return sysfs_create_group(&dev->dev.kobj, &rio_attribute_group);
}
void axxia_rio_release_sysfs(struct platform_device *dev)
{
	sysfs_remove_group(&dev->dev.kobj, &rio_attribute_group);
}

