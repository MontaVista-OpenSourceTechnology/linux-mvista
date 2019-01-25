/*
 * Copyright (C) 2014 - 2019 MontaVista, Software, LLC.
 * Copyright (C) 2010 Wind River Systems, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/timex.h>

#include <mach/i2c.h>
#include <linux/slab.h>
#include <linux/of_platform.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include "../i2c-core.h"

/* ----- global defines ----------------------------------------------- */

#define PUMA_I2C_TIMEOUT	(1*HZ)
#define I2C_PUMA_INTR_ALL    (PUMA_I2C_IMR_AAS | \
				 PUMA_I2C_IMR_SCD | \
				 PUMA_I2C_IMR_ARDY | \
				 PUMA_I2C_IMR_NACK | \
				 PUMA_I2C_IMR_AL)

#define PUMA_I2C_OAR_REG	0x00
#define PUMA_I2C_IMR_REG	0x04
#define PUMA_I2C_STR_REG	0x08
#define PUMA_I2C_CLKL_REG	0x0c
#define PUMA_I2C_CLKH_REG	0x10
#define PUMA_I2C_CNT_REG	0x14
#define PUMA_I2C_DRR_REG	0x18
#define PUMA_I2C_SAR_REG	0x1c
#define PUMA_I2C_DXR_REG	0x20
#define PUMA_I2C_MDR_REG	0x24
#define PUMA_I2C_IVR_REG	0x28
#define PUMA_I2C_EMDR_REG	0x2c
#define PUMA_I2C_PSC_REG	0x30

#define PUMA_I2C_IVR_AAS	0x07
#define PUMA_I2C_IVR_SCD	0x06
#define PUMA_I2C_IVR_XRDY	0x05
#define PUMA_I2C_IVR_RDR	0x04
#define PUMA_I2C_IVR_ARDY	0x03
#define PUMA_I2C_IVR_NACK	0x02
#define PUMA_I2C_IVR_AL	0x01

#define PUMA_I2C_STR_BB	(1 << 12)
#define PUMA_I2C_STR_RSFULL	(1 << 11)
#define PUMA_I2C_STR_SCD	(1 << 5)
#define PUMA_I2C_STR_ARDY	(1 << 2)
#define PUMA_I2C_STR_NACK	(1 << 1)
#define PUMA_I2C_STR_AL	(1 << 0)

#define PUMA_I2C_MDR_NACK	(1 << 15)
#define PUMA_I2C_MDR_STT	(1 << 13)
#define PUMA_I2C_MDR_STP	(1 << 11)
#define PUMA_I2C_MDR_MST	(1 << 10)
#define PUMA_I2C_MDR_TRX	(1 << 9)
#define PUMA_I2C_MDR_XA	(1 << 8)
#define PUMA_I2C_MDR_RM	(1 << 7)
#define PUMA_I2C_MDR_IRS	(1 << 5)

#define PUMA_I2C_IMR_AAS	(1 << 6)
#define PUMA_I2C_IMR_SCD	(1 << 5)
#define PUMA_I2C_IMR_XRDY	(1 << 4)
#define PUMA_I2C_IMR_RRDY	(1 << 3)
#define PUMA_I2C_IMR_ARDY	(1 << 2)
#define PUMA_I2C_IMR_NACK	(1 << 1)
#define PUMA_I2C_IMR_AL	(1 << 0)

#define MOD_REG_BIT(val, mask, set) do { \
	if (set) { \
		val |= mask; \
	} else { \
		val &= ~mask; \
	} \
} while (0)

struct PUMA_i2c_dev {
	struct device           *dev;
	void __iomem		*base;
	struct completion	cmd_complete;
	struct clk              *clk;
	int			cmd_err;
	u8			*buf;
	size_t			buf_len;
	int			irq;
	u8			terminate;
	struct i2c_adapter	adapter;
};

/* default platform data to use if not supplied in the platform_device */
static struct PUMA_i2c_platform_data PUMA_i2c_platform_data_default = {
	.bus_freq	= 100,
	.bus_delay	= 0,
};

static inline void PUMA_i2c_write_reg(struct PUMA_i2c_dev *i2c_dev,
					 int reg, u16 val)
{
	__raw_writew(val, i2c_dev->base + reg);
}

static inline u16 PUMA_i2c_read_reg(struct PUMA_i2c_dev *i2c_dev, int reg)
{
	return __raw_readw(i2c_dev->base + reg);
}

/*
 * This functions configures I2C and brings I2C out of reset.
 * This function is called during I2C init function. This function
 * also gets called if I2C encounters any errors.
 */
static int i2c_PUMA_init(struct PUMA_i2c_dev *dev)
{
	struct PUMA_i2c_platform_data *pdata = dev->dev->platform_data;
	u16 psc;
	u32 clk;
	u32 d;
	u32 clkh;
	u32 clkl;
	u32 input_clock = CLOCK_TICK_RATE;
	u16 w;

	if (!pdata)
		pdata = &PUMA_i2c_platform_data_default;

	/* put I2C into reset */
	w = PUMA_i2c_read_reg(dev, PUMA_I2C_MDR_REG);
	MOD_REG_BIT(w, PUMA_I2C_MDR_IRS, 0);
	PUMA_i2c_write_reg(dev, PUMA_I2C_MDR_REG, w);

	/* NOTE: I2C Clock divider programming info
	 * As per I2C specs the following formulas provide prescaler
	 * and low/high divider values
	 * input clk --> PSC Div -----------> ICCL/H Div --> output clock
	 *                       module clk
	 *
	 * output clk = module clk / (PSC + 1) [ (ICCL + d) + (ICCH + d) ]
	 *
	 * Thus,
	 * (ICCL + ICCH) = clk = (input clk / ((psc +1) * output clk)) - 2d;
	 *
	 * where if PSC == 0, d = 7,
	 *       if PSC == 1, d = 6
	 *       if PSC > 1 , d = 5
	 */

	/* get minimum of 7 MHz clock, but max of 12 MHz */
	psc = (input_clock / 7000000) - 1;
	if ((input_clock / (psc + 1)) > 12000000)
		psc++;	/* better to run under spec than over */
	d = (psc >= 2) ? 5 : 7 - psc;

	clk = ((input_clock / (psc + 1)) / (pdata->bus_freq * 1000)) - (d << 1);
	clkh = clk >> 1;
	clkl = clk - clkh;

	PUMA_i2c_write_reg(dev, PUMA_I2C_PSC_REG, psc);
	PUMA_i2c_write_reg(dev, PUMA_I2C_CLKH_REG, clkh);
	PUMA_i2c_write_reg(dev, PUMA_I2C_CLKL_REG, clkl);

	PUMA_i2c_write_reg(dev, PUMA_I2C_OAR_REG, 0x08);

	dev_dbg(dev->dev, "input_clock = %d, CLK = %d\n", input_clock, clk);
	dev_dbg(dev->dev, "PSC  = %d\n",
		PUMA_i2c_read_reg(dev, PUMA_I2C_PSC_REG));
	dev_dbg(dev->dev, "CLKL = %d\n",
		PUMA_i2c_read_reg(dev, PUMA_I2C_CLKL_REG));
	dev_dbg(dev->dev, "CLKH = %d\n",
		PUMA_i2c_read_reg(dev, PUMA_I2C_CLKH_REG));
	dev_dbg(dev->dev, "bus_freq = %dkHz, bus_delay = %d\n",
		pdata->bus_freq, pdata->bus_delay);

	/* Take the I2C module out of reset: */
	w = PUMA_i2c_read_reg(dev, PUMA_I2C_MDR_REG);
	MOD_REG_BIT(w, PUMA_I2C_MDR_IRS, 1);
	PUMA_i2c_write_reg(dev, PUMA_I2C_MDR_REG, w);

	/* Enable interrupts */
	PUMA_i2c_write_reg(dev, PUMA_I2C_IMR_REG, I2C_PUMA_INTR_ALL);

	return 0;
}

/*
 * Waiting for bus not busy
 */
static int i2c_PUMA_wait_bus_not_busy(struct PUMA_i2c_dev *dev,
					 char allow_sleep)
{
	unsigned long timeout;

	timeout = jiffies + PUMA_I2C_TIMEOUT;
	while (PUMA_i2c_read_reg(dev, PUMA_I2C_STR_REG)
	       & PUMA_I2C_STR_BB) {
		if (time_after(jiffies, timeout)) {
			dev_warn(dev->dev,
				 "timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		if (allow_sleep)
			schedule_timeout(1);
	}

	return 0;
}

/*
 * Low level master read/write transaction. This function is called
 * from i2c_PUMA_xfer.
 */
static int
i2c_PUMA_xfer_msg(struct i2c_adapter *adap, struct i2c_msg *msg, int stop)
{
	struct PUMA_i2c_dev *dev = i2c_get_adapdata(adap);
	struct PUMA_i2c_platform_data *pdata = dev->dev->platform_data;
	u32 flag;
	u16 w;
	int r;

	if (msg->len == 0)
		return -EINVAL;

	if (!pdata)
		pdata = &PUMA_i2c_platform_data_default;
	/* Introduce a delay, required for some boards (e.g PUMA EVM) */
	if (pdata->bus_delay)
		udelay(pdata->bus_delay);

	/* set the slave address */
	PUMA_i2c_write_reg(dev, PUMA_I2C_SAR_REG, msg->addr);

	dev->buf = msg->buf;
	dev->buf_len = msg->len;

	PUMA_i2c_write_reg(dev, PUMA_I2C_CNT_REG, dev->buf_len);

	reinit_completion(&(dev->cmd_complete));
	dev->cmd_err = 0;

	/* Take I2C out of reset, configure it as master and set the
	 * start bit
	 */
	flag = PUMA_I2C_MDR_IRS | PUMA_I2C_MDR_MST | PUMA_I2C_MDR_STT;

	/* if the slave address is ten bit address, enable XA bit */
	if (msg->flags & I2C_M_TEN)
		flag |= PUMA_I2C_MDR_XA;
	if (!(msg->flags & I2C_M_RD))
		flag |= PUMA_I2C_MDR_TRX;
	if (stop)
		flag |= PUMA_I2C_MDR_STP;

	/* Enable receive or transmit interrupts */
	w = PUMA_i2c_read_reg(dev, PUMA_I2C_IMR_REG);
	if (msg->flags & I2C_M_RD)
		MOD_REG_BIT(w, PUMA_I2C_IMR_RRDY, 1);
	else
		MOD_REG_BIT(w, PUMA_I2C_IMR_XRDY, 1);
	PUMA_i2c_write_reg(dev, PUMA_I2C_IMR_REG, w);

	dev->terminate = 0;
	/* write the data into mode register */
	PUMA_i2c_write_reg(dev, PUMA_I2C_MDR_REG, flag);

	r = wait_for_completion_interruptible_timeout(&dev->cmd_complete,
						      PUMA_I2C_TIMEOUT);
	if (r == 0) {
		dev_err(dev->dev, "controller timed out\n");
		i2c_PUMA_init(dev);
		dev->buf_len = 0;
		return -ETIMEDOUT;
	}
	if (dev->buf_len) {
		/* This should be 0 if all bytes were transferred
		 * or dev->cmd_err denotes an error.
		 * A signal may have aborted the transfer.
		 */
		if (r >= 0) {
			dev_err(dev->dev, "abnormal termination buf_len=%i\n",
				dev->buf_len);
			r = -EREMOTEIO;
		}
		dev->terminate = 1;
		/* make sure it terminates */
		wmb();
		dev->buf_len = 0;
	}
	if (r < 0)
		return r;

	/* no error */
	if (likely(!dev->cmd_err))
		return msg->len;

	/* We have an error */
	if (dev->cmd_err & PUMA_I2C_STR_AL) {
		i2c_PUMA_init(dev);
		return -EIO;
	}

	if (dev->cmd_err & PUMA_I2C_STR_NACK) {
		if (msg->flags & I2C_M_IGNORE_NAK)
			return msg->len;
		if (stop) {
			w = PUMA_i2c_read_reg(dev, PUMA_I2C_MDR_REG);
			MOD_REG_BIT(w, PUMA_I2C_MDR_STP, 1);
			PUMA_i2c_write_reg(dev, PUMA_I2C_MDR_REG, w);
		}
		return -EREMOTEIO;
	}
	return -EIO;
}

/*
 * Prepare controller for a transaction and call i2c_PUMA_xfer_msg
 */
static int
i2c_PUMA_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	struct PUMA_i2c_dev *dev = i2c_get_adapdata(adap);
	int i;
	int ret;

	dev_dbg(dev->dev, "%s: msgs: %d\n", __func__, num);

	ret = i2c_PUMA_wait_bus_not_busy(dev, 1);
	if (ret < 0) {
		dev_warn(dev->dev, "timeout waiting for bus ready\n");
		return ret;
	}

	for (i = 0; i < num; i++) {
		ret = i2c_PUMA_xfer_msg(adap, &msgs[i], (i == (num - 1)));
		dev_dbg(dev->dev, "%s [%d/%d] ret: %d\n", __func__, i + 1, num,
			ret);
		if (ret < 0)
			return ret;
	}
	return num;
}

static u32 i2c_PUMA_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static void terminate_read(struct PUMA_i2c_dev *dev)
{
	u16 w = PUMA_i2c_read_reg(dev, PUMA_I2C_MDR_REG);

	w |= PUMA_I2C_MDR_NACK;
	PUMA_i2c_write_reg(dev, PUMA_I2C_MDR_REG, w);

	/* Throw away data */
	PUMA_i2c_read_reg(dev, PUMA_I2C_DRR_REG);
	if (!dev->terminate)
		dev_err(dev->dev, "RDR IRQ while no data requested\n");
}
static void terminate_write(struct PUMA_i2c_dev *dev)
{
	u16 w = PUMA_i2c_read_reg(dev, PUMA_I2C_MDR_REG);

	w |= PUMA_I2C_MDR_RM | PUMA_I2C_MDR_STP;
	PUMA_i2c_write_reg(dev, PUMA_I2C_MDR_REG, w);

	if (!dev->terminate)
		dev_err(dev->dev, "TDR IRQ while no data to send\n");
}

/*
 * Interrupt service routine. This gets called whenever an I2C interrupt
 * occurs.
 */
static irqreturn_t i2c_PUMA_isr(int this_irq, void *dev_id)
{
	struct PUMA_i2c_dev *dev = dev_id;
	u32 stat;
	int count = 0;
	u16 w;

	while ((stat = PUMA_i2c_read_reg(dev, PUMA_I2C_IVR_REG))) {
		dev_dbg(dev->dev, "%s: stat=0x%x\n", __func__, stat);
		if (count++ == 100) {
			dev_warn(dev->dev, "Too much work in one IRQ\n");
			break;
		}

		switch (stat) {
		case PUMA_I2C_IVR_AL:
			/* Arbitration lost, must retry */
			dev->cmd_err |= PUMA_I2C_STR_AL;
			dev->buf_len = 0;
			complete(&dev->cmd_complete);
			break;

		case PUMA_I2C_IVR_NACK:
			dev->cmd_err |= PUMA_I2C_STR_NACK;
			dev->buf_len = 0;
			complete(&dev->cmd_complete);
			break;

		case PUMA_I2C_IVR_ARDY:
			PUMA_i2c_write_reg(dev,
				PUMA_I2C_STR_REG, PUMA_I2C_STR_ARDY);
			complete(&dev->cmd_complete);
			break;

		case PUMA_I2C_IVR_RDR:
			if (dev->buf_len) {
				*dev->buf++ =
				    PUMA_i2c_read_reg(dev,
							 PUMA_I2C_DRR_REG);
				dev->buf_len--;
				if (dev->buf_len)
					continue;

				PUMA_i2c_write_reg(dev,
					PUMA_I2C_STR_REG,
					PUMA_I2C_IMR_RRDY);
			} else {
				/* signal can terminate transfer */
				terminate_read(dev);
			}
			break;

		case PUMA_I2C_IVR_XRDY:
			if (dev->buf_len) {
				PUMA_i2c_write_reg(dev, PUMA_I2C_DXR_REG,
						      *dev->buf++);
				dev->buf_len--;
				if (dev->buf_len)
					continue;

				w = PUMA_i2c_read_reg(dev,
							 PUMA_I2C_IMR_REG);
				MOD_REG_BIT(w, PUMA_I2C_IMR_XRDY, 0);
				PUMA_i2c_write_reg(dev,
						      PUMA_I2C_IMR_REG,
						      w);
			} else {
				/* signal can terminate transfer */
				terminate_write(dev);
			}
			break;

		case PUMA_I2C_IVR_SCD:
			PUMA_i2c_write_reg(dev,
				PUMA_I2C_STR_REG, PUMA_I2C_STR_SCD);
			complete(&dev->cmd_complete);
			break;

		case PUMA_I2C_IVR_AAS:
			dev_dbg(dev->dev, "Address as slave interrupt\n");
			break;

		default:
			dev_warn(dev->dev, "Unrecognized irq stat %d\n", stat);
			break;
		}
	}

	return count ? IRQ_HANDLED : IRQ_NONE;
}

static struct i2c_algorithm i2c_PUMA_algo = {
	.master_xfer	= i2c_PUMA_xfer,
	.functionality	= i2c_PUMA_func,
};

#ifdef CONFIG_OF
static const struct of_device_id puma_i2c_match[];
#endif
static int PUMA_i2c_probe(struct platform_device *pdev)
{
	struct PUMA_i2c_dev *dev;
	struct i2c_adapter *adap;
	struct resource *ioarea;
	int r;

#ifdef CONFIG_OF
	unsigned int  prop, i2c_irq;
	struct device_node *np;
	struct PUMA_i2c_platform_data *pdata =
		(struct PUMA_i2c_platform_data *)pdev->dev.platform_data;
	const struct of_device_id *match;

	match = of_match_device(puma_i2c_match, &pdev->dev);
	if (!match)
		return -EINVAL;
	np = pdev->dev.of_node;
	ioarea = of_iomap(np, 0);
	if (ioarea == NULL) {
		dev_err(&pdev->dev, "failed to iomap device\n");
		return -ENXIO;
	}

	i2c_irq = irq_of_parse_and_map(np, 0);
	if (!i2c_irq) {
		dev_err(&pdev->dev, "irq_of_parse_and_map failed\n");
		return -ENODEV;
	}
#else
	struct resource *irq, *mem;
	u32 size;

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!irq) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return -ENODEV;
	}

	size = mem->end - mem->start + 1;
	ioarea = ioremap(mem->start, size);

	printk(KERN_ERR "I2C :: IO REMAP PHYS: 0x%08x -> VIRT: 0x%08x SIZE:%d\n", mem->start, (u32) ioarea, size);

	if (!ioarea) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -EBUSY;
	}
#endif
	dev = kzalloc(sizeof(struct PUMA_i2c_dev), GFP_KERNEL);
	if (!dev) {
		r = -ENOMEM;
		goto err_release_region;
	}

	init_completion(&dev->cmd_complete);
	dev->dev = get_device(&pdev->dev);

#ifdef CONFIG_OF
	dev->irq = i2c_irq;
	if (!pdata && pdev->dev.of_node)
		pdata = kzalloc(sizeof(struct PUMA_i2c_platform_data),
					GFP_KERNEL);
	if (!pdata) {
		r = -ENOMEM;
		goto err_release_region;
	}
	if (!of_property_read_u32(np, "bus-frequency", &prop))
		pdata->bus_freq = prop / 1000;
	if (!of_property_read_u32(np, "bus-delay", &prop))
		pdata->bus_delay = prop;

	dev->dev->platform_data = (void *)pdata;
#else
	dev->irq = irq->start;
#endif
	platform_set_drvdata(pdev, dev);
	dev->base = (void __iomem *) ioarea;

	i2c_PUMA_init(dev);
	r = request_irq(dev->irq, i2c_PUMA_isr, 0, pdev->name, dev);
	if (r) {
		dev_err(&pdev->dev, "failure requesting irq %i\n", dev->irq);
		goto err_unuse_clocks;
	}
	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = I2C_CLASS_HWMON;
	strlcpy(adap->name, "PUMA I2C adapter", sizeof(adap->name));
	adap->algo = &i2c_PUMA_algo;
	adap->dev.parent = &pdev->dev;
#ifdef CONFIG_OF
	adap->dev.of_node = of_node_get(np);
#endif
	/* FIXME */
	adap->timeout = PUMA_I2C_TIMEOUT;

	adap->nr = pdev->id;
	r = i2c_add_numbered_adapter(adap);
	if (r) {
		dev_err(&pdev->dev, "failure adding adapter\n");
		goto err_free_irq;
	}

#ifdef CONFIG_OF
	of_i2c_register_devices(adap);
#endif

	return 0;

err_free_irq:
	free_irq(dev->irq, dev);
err_unuse_clocks:
	dev->clk = NULL;

	platform_set_drvdata(pdev, NULL);
	put_device(&pdev->dev);
	kfree(dev);
err_release_region:
#ifdef CONFIG_OF
	iounmap(ioarea);
#else
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
#endif

	return r;
}

static int PUMA_i2c_remove(struct platform_device *pdev)
{
	struct PUMA_i2c_dev *dev = platform_get_drvdata(pdev);
	struct resource *mem;

	platform_set_drvdata(pdev, NULL);
	i2c_del_adapter(&dev->adapter);
	put_device(&pdev->dev);

	dev->clk = NULL;

	PUMA_i2c_write_reg(dev, PUMA_I2C_MDR_REG, 0);
	free_irq(IRQ_I2C, dev);
	iounmap(dev->base); // ARNAUD
	kfree(dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(mem->start, (mem->end - mem->start) + 1);
	return 0;
}

/* work with hotplug and coldplug */
MODULE_ALIAS("platform:i2c_PUMA");

#ifdef CONFIG_OF
static const struct of_device_id puma_i2c_match[] = {
	{ .compatible = "puma,i2c_PUMA", },
	{}
};
MODULE_DEVICE_TABLE(of, puma_i2c_of_match);
#endif
static struct platform_driver PUMA_i2c_driver = {
	.probe		= PUMA_i2c_probe,
	.remove		= PUMA_i2c_remove,
	.driver		= {
		.name	= "i2c_PUMA",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = puma_i2c_match,
#endif
	},
};


#ifndef CONFIG_OF
/* I2C may be needed to bring up other drivers */
static int __init PUMA_i2c_init_driver(void)
{
	return platform_driver_register(&PUMA_i2c_driver);
}
subsys_initcall(PUMA_i2c_init_driver);

static void __exit PUMA_i2c_exit_driver(void)
{
	platform_driver_unregister(&PUMA_i2c_driver);
}
module_exit(PUMA_i2c_exit_driver);
#else
module_platform_driver(PUMA_i2c_driver);
#endif
MODULE_AUTHOR("Windriver");
MODULE_DESCRIPTION("TI PUMA I2C bus adapter");
MODULE_LICENSE("GPL");
