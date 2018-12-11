/*
 * Copyright (C) 2013 Broadcom Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/semaphore.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include "iproc_smbus_regs.h"

/* SMBUS protocol values defined in register 0x30 */
#define SMBUS_PROT_QUICK_CMD			0
#define SMBUS_PROT_SEND_BYTE			1
#define SMBUS_PROT_RECV_BYTE			2
#define SMBUS_PROT_WR_BYTE  			3
#define SMBUS_PROT_RD_BYTE  			4
#define SMBUS_PROT_WR_WORD  			5
#define SMBUS_PROT_RD_WORD  			6
#define SMBUS_PROT_BLK_WR   			7
#define SMBUS_PROT_BLK_RD   			8
#define SMBUS_PROT_PROC_CALL			9
#define SMBUS_PROT_BLK_WR_BLK_RD_PROC_CALL 	10

#define MSTR_STS_XACT_SUCCESS	0
#define DISABLE_INTR	0
#define ENABLE_INTR	1
#define SMB_MAX_DATA_SIZE	32
#define XACT_TIMEOUT	msecs_to_jiffies(100)
#define init_MUTEX(x)	sema_init(x,1)
#define IPROC_SMB_MAX_RETRIES   35

#define GETREGFLDVAL(regval, mask, startbit) (((regval) & (mask)) >> (startbit))

#define SETREGFLDVAL(regval, fldval, mask, startbit) regval = \
                                                      (regval & ~(mask)) | \
                                                      ((fldval) << (startbit))

typedef enum iproc_smb_clk_freq {
	I2C_SPEED_100KHz = 0,
	I2C_SPEED_400KHz = 1,
	I2C_SPEED_INVALID = 255
} smb_clk_freq_t;

/* Counters will be used mainly for testing and debugging */
struct iproc_smb_counters {
	unsigned int num_read_requests;
	unsigned int num_write_requests;
	unsigned int num_read_errors;
	unsigned int num_write_errors;
	unsigned int mstr_rx_evt_cnt; /* ISR counter to check recv event */
	unsigned int mstr_start_busy_cnt; /* ISR counter to checking xact sts */
	unsigned int mstr_rx_fifo_full_cnt; /* ISR counter to detect rx fifo full */
	unsigned int last_int_sts; /* last value of intr status reg */
};

/* This structure will be used internally by the driver to maintain its
 * configuration information as well as information programmed into the hardware
 */
struct iproc_smb_drv_int_data {
	struct device *dev;
	struct iproc_smb_drv_int_data *next;
	int irq;
	unsigned int drv_state_init; /* 1 = Initialized, 0 = not initialized */
	unsigned int drv_state_open; /* 1 = Accepting transaction requests,
	                                0 = Not accepting transaction requests */
	smb_clk_freq_t clk_speed;
	void __iomem *base; /* virtual base address for register access */
	struct i2c_adapter adapter;
	unsigned int i2c_slave_addr;
	struct semaphore xfer_lock; /* Lock for data transfer */
	struct completion ses_done; /* To signal the command completion */
	volatile int debug;
	unsigned int master_rx_fifo_thr; /* Master FIFO threshold */
	unsigned int slave_rx_fifo_thr; /* Slave FIFO threshold */
	unsigned int enable_evts;
	unsigned int evt_enable_bmap; /* Bit map of events enabled by the driver */
	struct iproc_smb_counters smb_counters; /* Statistics maintained by driver */
};

/* Structure used to pass information to read/write functions. */
struct iproc_xact_info {
	bool cmd_valid; /* true if command field below is valid. Otherwise, false */
	unsigned short command; /* Passed by caller to send SMBus command code */
	unsigned char *data; /* actual data pased by the caller */
	unsigned int size; /* Size of data buffer passed */
	unsigned short flags; /* Sent by caller specifying PEC, 10-bit addresses */
	unsigned char smb_proto; /* SMBus protocol to use to perform transaction */
};

static irqreturn_t iproc_smb_isr(int irq, void *devid)
{
	struct iproc_smb_drv_int_data *dev =
                                     (struct iproc_smb_drv_int_data *)devid;
	unsigned int intsts;
	unsigned int regval;

	intsts = readl(dev->base + CCB_SMB_EVTSTS_REG);
	dev->smb_counters.last_int_sts = intsts;

	if (!intsts)
		/* Likely received a spurious interrupt */
		return IRQ_NONE;

	/* Clear interrupts */
	writel(intsts, dev->base + CCB_SMB_EVTSTS_REG);

	/* Master read or write complete */
	if ((intsts & CCB_SMB_MSTRSTARTBUSYEN_MASK) ||
		(intsts & CCB_SMB_MSTRRXEVTSTS_MASK)) {

		if (intsts & CCB_SMB_MSTRSTARTBUSYEN_MASK)
			dev->smb_counters.mstr_start_busy_cnt++;

		if (intsts & CCB_SMB_MSTRRXEVTSTS_MASK)
			dev->smb_counters.mstr_rx_evt_cnt++;

		complete(&dev->ses_done);
	}

	/* If RX FIFO was full we can either read and then flush the FIFO.
	 * Or, only flush the FIFO, and then the client process can restart
	 * the transaction.
	 * For now, we will flush the later action.
	 */
	if (intsts & CCB_SMB_MSTRRXFIFOFULLSTS_MASK) {
		dev->smb_counters.mstr_rx_fifo_full_cnt++;
		regval = readl(dev->base + CCB_SMB_MSTRFIFOCTL_REG);
		regval |= CCB_SMB_MSTRRXFIFOFLSH_MASK;
		writel(regval, dev->base + CCB_SMB_MSTRFIFOCTL_REG);
		complete(&dev->ses_done);
	}

	return IRQ_HANDLED;
}

/*
 * This function set clock frequency for SMBus block. As per hardware
 * engineering, the clock frequency can be changed dynamically.
 */
static int iproc_smb_set_clk_freq(void __iomem *base_addr,
                                     smb_clk_freq_t freq)
{
	unsigned int regval;
	unsigned int val;

	switch (freq) {
	case I2C_SPEED_100KHz:
		val = 0;
		break;

	case I2C_SPEED_400KHz:
		val = 1;
		break;

	default:
		return -EINVAL;
		break;
	}

	regval = readl(base_addr + CCB_SMB_TIMGCFG_REG);
	SETREGFLDVAL(regval, val, CCB_SMB_TIMGCFG_MODE400_MASK,
			CCB_SMB_TIMGCFG_MODE400_SHIFT);
	writel(regval, base_addr + CCB_SMB_TIMGCFG_REG);

	return 0;
}

static int iproc_smbus_block_init(struct iproc_smb_drv_int_data *dev)
{

	void __iomem *base_addr = dev->base;
	unsigned int regval;
	u32 i2c_clk_freq;
	struct device_node *dn = dev->dev->of_node;

	/* Flush Tx, Rx FIFOs. Note we are setting the Rx FIFO threshold to 0.
	 * May be OK since we are setting RX_EVENT and RX_FIFO_FULL interrupts
	 */
	regval = CCB_SMB_MSTRRXFIFOFLSH_MASK | CCB_SMB_MSTRTXFIFOFLSH_MASK;

	writel(regval, base_addr + CCB_SMB_MSTRFIFOCTL_REG);

	/* Enable SMbus block. Note, we are setting MASTER_RETRY_COUNT to zero
	 * since there will be only one master
	 */
	regval = CCB_SMB_CFG_SMBEN_MASK;

	writel(regval, base_addr + CCB_SMB_CFG_REG);

	/* Wait a minimum of 50 Usec, as per SMB hw doc. But we wait longer */
	udelay(100);

	/* Set default clock frequency */
	if (of_property_read_u32(dn, "clock-frequency", &i2c_clk_freq))
		/*no property available, use default: 100KHz*/
		i2c_clk_freq = I2C_SPEED_100KHz;
	iproc_smb_set_clk_freq(base_addr, i2c_clk_freq);

	/* Disable intrs */
	regval = 0x0;
	writel(regval, base_addr + CCB_SMB_EVTEN_REG);

	/* Clear intrs (W1TC) */
	regval = readl(base_addr + CCB_SMB_EVTSTS_REG);
	writel(regval, base_addr + CCB_SMB_EVTSTS_REG);

	return(0);
}

/*
 * Function to ensure that the previous transaction was completed before
 * initiating a new transaction. It can also be used in polling mode to
 * check status of completion of a command
 */
static int iproc_smb_startbusy_wait(struct iproc_smb_drv_int_data *dev)
{
	unsigned int regval;

	regval = readl(dev->base + CCB_SMB_MSTRCMD_REG);

	/* Check if an operation is in progress. During probe it won't be.
	 * But when shutdown/remove was called we want to make sure that
	 * the transaction in progress completed
	 */
	if (regval & CCB_SMB_MSTRSTARTBUSYCMD_MASK) {
		unsigned int i = 0;

		do {
			msleep(1);
			i++;
			regval = readl(dev->base + CCB_SMB_MSTRCMD_REG);
		/* If start-busy bit cleared, exit the loop */
		} while ((regval & CCB_SMB_MSTRSTARTBUSYCMD_MASK) &&
				(i < IPROC_SMB_MAX_RETRIES));

		if (i >= IPROC_SMB_MAX_RETRIES) {
			dev_err(dev->dev, "START_BUSY bit didn't clear\n");
			return -ETIMEDOUT;
		}
	}

	return 0;
}


static u32 smbus0_sdaRecoveryCnt=0, smbus0_sdaFailedCnt=0, smbus0_startBusyCnt=0;
static u32 smbus1_sdaRecoveryCnt=0, smbus1_sdaFailedCnt=0, smbus1_startBusyCnt=0;

/*
 * Function to recover SMB hangs caused stuck master START_BUSY.
 *   Returns  0 if recovery procedure executed successfully.
 *   Returns -1 if recovery failed.
 */
static int iproc_smb_startbusy_recovery(struct iproc_smb_drv_int_data *dev)
{
	int rc = -1;
	unsigned int recoveryCnt;

	if (dev->adapter.nr == 0)
		recoveryCnt = ++smbus0_startBusyCnt;
	else
		recoveryCnt = ++smbus1_startBusyCnt;

	dev_info(dev->dev, "START_BUSY recovery #%d \n", recoveryCnt);

	/* reset the SMBus block, wait a minimum of 50 uSecs and then re-initialize */
	writel(CCB_SMB_CFG_RST_MASK, dev->base + CCB_SMB_CFG_REG);
	udelay(60);

	if (iproc_smbus_block_init(dev) == 0)
		rc = 0;

	return rc;
}


/*
 * Function to recover SMB hang caused by a slave device holding SDA low.
 *   Returns  0 if recovery procedure executed successfully.
 *   Returns -1 if recovery failed.
 */

static int iproc_smb_sda_low_recovery(struct iproc_smb_drv_int_data *dev)
{
	unsigned int bbReg, cfgReg, cfgSave, recoveryCnt, failedCnt, i;
	int rc = -1;

	/* enable bit-bang */
	cfgSave = readl(dev->base + CCB_SMB_CFG_REG);
	cfgReg  = cfgSave;
	cfgReg |= CCB_SMB_CFG_BITBANGEN_MASK;
	writel(cfgReg, dev->base + CCB_SMB_CFG_REG);
	udelay(50);

	/* start with clock and SDA set high */
	bbReg  = readl(dev->base + CCB_SMB_BITBANGCTL_REG);
	bbReg |= (CCB_SMB_SMBCLKOUTEN_MASK | CCB_SMB_SMBDATAOUTEN_MASK);
	writel(bbReg, dev->base + CCB_SMB_BITBANGCTL_REG);
	udelay(5);   /* should be sufficient for 100 KHz bus */

	/* set up to toggle the clock line with SDA out held high for 9 cycles */
	for (i = 0; i < 18; i++) {
		/* toggle CLK out */
		if ( (bbReg & CCB_SMB_SMBCLKOUTEN_MASK) == 0 )
			bbReg |= CCB_SMB_SMBCLKOUTEN_MASK;   /* set clock high */
		else
			bbReg &= ~CCB_SMB_SMBCLKOUTEN_MASK;  /* set clock low  */

		writel(bbReg, dev->base + CCB_SMB_BITBANGCTL_REG);
		udelay(5);
	}

	/* check bit 29 -- SMBDAT_IN and make sure SDA not being held low any more */
	for (i = 0; i < 10; i++) {
		bbReg  = readl(dev->base + CCB_SMB_BITBANGCTL_REG);
		bbReg &= CCB_SMB_SMBDATAIN_MASK;
		if (bbReg)
			break;
		udelay(1);
	}

	if (bbReg == 0) {
		/* SDA is still low */
		if (dev->adapter.nr == 0)
			failedCnt = ++smbus0_sdaFailedCnt;
		else
			failedCnt = ++smbus1_sdaFailedCnt;

		dev_info(dev->dev, "SDA release #%d FAILED.\n", failedCnt);
	} else {
		if (dev->adapter.nr == 0)
			recoveryCnt = ++smbus0_sdaRecoveryCnt;
		else
			recoveryCnt = ++smbus1_sdaRecoveryCnt;

		dev_info(dev->dev, "SDA release #%d SUCCESS.\n", recoveryCnt);
		rc = 0;
	}

	/* manually issue a stop by transitioning SDA from low to high with clock held high */
	bbReg  = readl(dev->base + CCB_SMB_BITBANGCTL_REG);
	bbReg &= ~CCB_SMB_SMBCLKOUTEN_MASK;                  /* set clock low */
	writel(bbReg, dev->base + CCB_SMB_BITBANGCTL_REG);
	udelay(2);

	bbReg &= ~CCB_SMB_SMBDATAOUTEN_MASK;                 /* drop SDA low */
	writel(bbReg, dev->base + CCB_SMB_BITBANGCTL_REG);
	udelay(2);

	bbReg |= CCB_SMB_SMBCLKOUTEN_MASK;                   /* set clock high */
	writel(bbReg, dev->base + CCB_SMB_BITBANGCTL_REG);
	udelay(5);

	bbReg |= CCB_SMB_SMBDATAOUTEN_MASK;                  /* pull SDA high */
	writel(bbReg, dev->base + CCB_SMB_BITBANGCTL_REG);
	udelay(2);

	/* disable bit-bang and then re-enable the SMB with the saved configuration */
	cfgReg  = readl(dev->base + CCB_SMB_CFG_REG);
	cfgReg &= ~CCB_SMB_CFG_BITBANGEN_MASK;
	writel(cfgReg, dev->base + CCB_SMB_CFG_REG);
	udelay(10);

	writel(cfgSave, dev->base + CCB_SMB_CFG_REG);

	return rc;
}


/*
 * Function to recover SMB hang caused by a slave device hold SDA low.
 *   Returns  0 if recovery procedure executed successfully.
 *   Returns -1 if recovery failed.
 */
static int iproc_smb_timeout_recovery(struct iproc_smb_drv_int_data *dev)
{
	unsigned int bbReg, mCmdReg;
	int rc = -1;

	/* read bit-bang control.  If SDA low, attempt SDA release recovery */
	bbReg = readl(dev->base + CCB_SMB_BITBANGCTL_REG);

	if ((bbReg & CCB_SMB_SMBDATAIN_MASK) == 0)
		if (iproc_smb_sda_low_recovery(dev) == 0)
			rc = 0;

	/* regardless of whether there was an SDA hang or not, see if START_BUSY stuck high */
	mCmdReg = readl( dev->base + CCB_SMB_MSTRCMD_REG );
	if (mCmdReg & CCB_SMB_MSTRSTARTBUSYCMD_MASK)
		/* attempt to recover the bus */
		if (iproc_smb_startbusy_recovery(dev) == 0)
			rc = 0;

	return rc;
}

/*
 * Copy data to SMBus's Tx FIFO. Valid for write transactions only
 *
 * base_addr: Mapped address of this SMBus instance
 * dev_addr: SMBus device address. Assuming 7-bit addresses initially
 * info: Data to copy in to Tx FIFO. For read commands, the size should be
 *       set to zero by the caller
 *
 */
static void iproc_smb_write_trans_data(void __iomem *base_addr,
                                       unsigned short dev_addr,
                                       struct iproc_xact_info *info)
{
	unsigned int regval;
	unsigned int i;
	unsigned int num_data_bytes = 0;

	/* Write SMBus device address first */
	/* We are assuming 7-bit addresses for now. For 10-bit addresses,
	 * we may have one more write to send the upper 3 bits of 10-bit addr
	 */
	writel(dev_addr, base_addr + CCB_SMB_MSTRDATAWR_REG);

	/* If the protocol needs command code, copy it */
	if (info->cmd_valid == true)
		writel(info->command, base_addr + CCB_SMB_MSTRDATAWR_REG);

	/*
	 * Depending on the SMBus protocol, we need to write additional
	 * transaction data into Tx FIFO.
	 * Refer to section 5.5 of SMBus spec for sequence for a transaction
	 */
	switch (info->smb_proto) {
	case SMBUS_PROT_RECV_BYTE:
		/* No additional data to be written */
		num_data_bytes = 0;
		break;

	case SMBUS_PROT_SEND_BYTE:
		num_data_bytes = info->size;
		break;

	case SMBUS_PROT_RD_BYTE:
	case SMBUS_PROT_RD_WORD:
	case SMBUS_PROT_BLK_RD:
		/* Write slave address with R/W~ set (bit #0) */
		writel(dev_addr | 0x1, base_addr + CCB_SMB_MSTRDATAWR_REG);
		num_data_bytes = 0;
		break;

	case SMBUS_PROT_WR_BYTE:
	case SMBUS_PROT_WR_WORD:
		/* No additional bytes to be written */
		/* Data portion is written in the 'for' loop below */
		num_data_bytes = info->size;
		break;

	case SMBUS_PROT_BLK_WR:
		/* 3rd byte is byte count */
		writel(info->size, base_addr + CCB_SMB_MSTRDATAWR_REG);
		num_data_bytes = info->size;
		break;

	case SMBUS_PROT_BLK_WR_BLK_RD_PROC_CALL:
		/* Write byte count */
		writel(info->size, base_addr + CCB_SMB_MSTRDATAWR_REG);
		num_data_bytes = info->size;
		break;

	default:
		break;
	}

	/* Copy actual data from caller */
	for (i = 0; num_data_bytes; --num_data_bytes, i++) {
        /* For the last byte, set MASTER_WR_STATUS bit. For block rd/wr process
         * call, we need to program slave addr after copying data byte(s), so
         * master status bit is set later, after the loop
         */
		if ((num_data_bytes == 1) &&
			(info->smb_proto != SMBUS_PROT_BLK_WR_BLK_RD_PROC_CALL))
			regval = info->data[i] | CCB_SMB_MSTRWRSTS_MASK;
		else
			regval =  info->data[i];

		writel(regval, base_addr + CCB_SMB_MSTRDATAWR_REG);
	}

	if (info->smb_proto == SMBUS_PROT_BLK_WR_BLK_RD_PROC_CALL)
		/* Write device address needed during repeat start condition */
		writel(CCB_SMB_MSTRWRSTS_MASK | dev_addr | 0x1,
				base_addr + CCB_SMB_MSTRDATAWR_REG);

	return;
}

static int iproc_smb_data_send(struct i2c_adapter *adapter,
                               unsigned short addr,
                               struct iproc_xact_info *info)
{
	int rc;
	unsigned int regval;
	struct iproc_smb_drv_int_data *dev = i2c_get_adapdata(adapter);
	unsigned long time_left;

	/* Make sure the previous transaction completed */
	rc = iproc_smb_startbusy_wait(dev);
	if (rc < 0) {
		dev_err(dev->dev, "Send: bus is busy, attempt recovery \n");
		/* attempt to recover the bus */
		if (iproc_smb_startbusy_recovery(dev) != 0)
			return rc;
	}

	if (dev->enable_evts == ENABLE_INTR) {
		/* Enable start_busy interrupt */
		regval = readl(dev->base + CCB_SMB_EVTEN_REG);
		regval |= CCB_SMB_MSTRSTARTBUSYEN_MASK;
		writel(regval, dev->base + CCB_SMB_EVTEN_REG);
		/* Mark as incomplete before sending the data */
		reinit_completion(&dev->ses_done);
	}

	/* Write transaction bytes to Tx FIFO */
	iproc_smb_write_trans_data(dev->base, addr, info);

	/* Program master command register (0x30) with protocol type and
	 * set start_busy_command bit to initiate the write transaction
	 */
	regval = (info->smb_proto << CCB_SMB_MSTRSMBUSPROTO_SHIFT) |
				CCB_SMB_MSTRSTARTBUSYCMD_MASK;
	writel(regval, dev->base + CCB_SMB_MSTRCMD_REG);

	if (dev->enable_evts == ENABLE_INTR) {
		/*
		 * Block waiting for the transaction to finish. When finished,
		 * we'll be signaled by an interrupt
		 */
		time_left = wait_for_completion_timeout(&dev->ses_done,
							XACT_TIMEOUT);
		/* Disable start_busy interrupt */
		regval = readl(dev->base + CCB_SMB_EVTEN_REG);
		regval &= ~CCB_SMB_MSTRSTARTBUSYEN_MASK;
		writel(regval, dev->base + CCB_SMB_EVTEN_REG);

		if (time_left == 0) {
			dev_info(dev->dev, "Send: timeout accessing device");
			/* attempt to recover the bus */
			rc = iproc_smb_timeout_recovery(dev);
			if (rc != 0)
				return -ETIMEDOUT;
			else
				return -ECOMM;
		}
	}

	regval = readl(dev->base + CCB_SMB_MSTRCMD_REG);

	/* If start_busy bit cleared, check if there are any errors */
	if (!(regval & CCB_SMB_MSTRSTARTBUSYCMD_MASK)) {
		/* start_busy bit cleared, check master_status field now */
		regval &= CCB_SMB_MSTRSTS_MASK;
		regval >>= CCB_SMB_MSTRSTS_SHIFT;

		if (regval != MSTR_STS_XACT_SUCCESS) {
			/* We can flush Tx FIFO here */
			dev_err(dev->dev, "Send: Error in transaction\n");
			return -EREMOTEIO;
		}
	}

	return 0;
}

static int iproc_smb_data_recv(struct i2c_adapter *adapter,
                               unsigned short addr,
                               struct iproc_xact_info *info,
                               unsigned int *num_bytes_read)
{
	int rc;
	unsigned int regval;
	struct iproc_smb_drv_int_data *dev = i2c_get_adapdata(adapter);
	unsigned long time_left;

	/* Make sure the previous transaction completed */
	rc = iproc_smb_startbusy_wait(dev);

	if (rc < 0) {
		dev_err(dev->dev, "Receive: bus is busy, attempt recovery \n");
		/* attempt to recover the bus */
		if (iproc_smb_startbusy_recovery(dev) != 0)
			return rc;
	}

	if (dev->enable_evts == ENABLE_INTR) {
		/* Enable start_busy interrupt */
		regval = readl(dev->base + CCB_SMB_EVTEN_REG);

		/* Set Rx_event_en bit for notification of reception event */
		regval |= (CCB_SMB_MSTRSTARTBUSYEN_MASK);

		writel(regval, dev->base + CCB_SMB_EVTEN_REG);

		/* Mark as incomplete before sending the data */
		reinit_completion(&dev->ses_done);
	}

	/* Program all transaction bytes into master Tx FIFO */
	iproc_smb_write_trans_data(dev->base, addr, info);

	/* Program master command register (0x30) with protocol type and set
	 * start_busy_command bit to initiate the write transaction
	 */
	regval = (info->smb_proto << CCB_SMB_MSTRSMBUSPROTO_SHIFT) |
				CCB_SMB_MSTRSTARTBUSYCMD_MASK | info->size;
	writel(regval, dev->base + CCB_SMB_MSTRCMD_REG);

	if (dev->enable_evts == ENABLE_INTR) {
		/*
		 * Block waiting for the transaction to finish. When finished,
		 * we'll be signaled by an interrupt
		 */
		time_left = wait_for_completion_timeout(&dev->ses_done,
							XACT_TIMEOUT);

		/* Disable start_busy and rx_event interrupts */
		regval = readl(dev->base + CCB_SMB_EVTEN_REG);
		regval &= ~(CCB_SMB_MSTRSTARTBUSYEN_MASK);
		writel(regval, dev->base + CCB_SMB_EVTEN_REG);

		if (time_left == 0) {
			dev_err(dev->dev,  "Receive: timeout accessing device\n");
			/* attempt to recover the bus */
			rc = iproc_smb_timeout_recovery(dev);
			if (rc != 0)
				return -ETIMEDOUT;
			else
				return -ECOMM;
		}
	}

	regval = readl(dev->base + CCB_SMB_MSTRCMD_REG);

	/* If start_busy bit cleared, check if there are any errors */
	if (!(regval & CCB_SMB_MSTRSTARTBUSYCMD_MASK)) {
		/* start_busy bit cleared, check master_status field now */
		regval &= CCB_SMB_MSTRSTS_MASK;
		regval >>= CCB_SMB_MSTRSTS_SHIFT;

		if (regval != MSTR_STS_XACT_SUCCESS) {
			/* We can flush Tx FIFO here */
			dev_info(dev->dev, "Error in transaction\n");
			return -EREMOTEIO;
		}
	}

	/* In the isr we will read the received byte, and also deal with
	 * rx fifo full event. The above check is for timeout error. If needed
	 * we may move it to rx isr
	 */

	/* For block read, protocol (hw) returns byte count, as the first byte */
	if ((info->smb_proto == SMBUS_PROT_BLK_RD) ||
		(info->smb_proto == SMBUS_PROT_BLK_WR_BLK_RD_PROC_CALL)) {
		int i, adj;

		/* Read received byte(s) */
		regval = readl(dev->base + CCB_SMB_MSTRDATARD_REG);
		*num_bytes_read = regval & CCB_SMB_MSTRRDDATA_MASK;

		adj = 0;

		/* SMBUS spec ver. 3 (2015) extends max block transfer byte count from 32 to 256 */
		/* Use SMB_MAX_DATA_SIZE (according to HW FIFO) instead of I2C_SMBUS_BLOCK_MAX (defined in Linux)*/
		/*
		 * Current SMBUS HW FIFO length is 64B. For block write xfer,
		 * the first three FIFO entries are for slave adress, register ofFset,
		 * and length count.
		 */
		for (i = 0; (i < *num_bytes_read) && (i < (SMB_MAX_DATA_SIZE - adj)); i++) {
			/* Read Rx FIFO for data bytes */
			regval = readl(dev->base + CCB_SMB_MSTRDATARD_REG);
			info->data[i + adj] = regval & CCB_SMB_MSTRRDDATA_MASK;
		}

		*num_bytes_read = i + adj;
	}
	else {
		regval = readl(dev->base + CCB_SMB_MSTRDATARD_REG);
		*info->data = regval & CCB_SMB_MSTRRDDATA_MASK;
		*num_bytes_read = 1;
		if (info->smb_proto == SMBUS_PROT_RD_WORD) {
			/* Read Rx FIFO for data bytes */
			regval = readl(dev->base + CCB_SMB_MSTRDATARD_REG);
			info->data[1] = regval & CCB_SMB_MSTRRDDATA_MASK;
			*num_bytes_read = 2;
		}
	}

	return 0;
}

static int iproc_smb_xfer(struct i2c_adapter *i2c_adap, u16 addr,
                          unsigned short flags, char read_write,
                          u8 command, int size, union i2c_smbus_data *data)
{
	int rc = 0;
	struct iproc_smb_drv_int_data *dev = i2c_get_adapdata(i2c_adap);
	struct iproc_xact_info info;
	unsigned int num_bytes_read = 0;
	int smb_xfer_size;

	down(&dev->xfer_lock);

	addr <<= 1;

	switch (size /* protocol */) {
        case I2C_SMBUS_BYTE:
		info.cmd_valid = false;
		info.command = command; /* not used */
		if (read_write == I2C_SMBUS_WRITE)
			info.data = &command;
		else
			info.data = &data->byte;
		info.size = 1;
		info.flags = flags;
		if (read_write == I2C_SMBUS_READ) {
			addr |= 0x1; /* Read operation */
			info.smb_proto = SMBUS_PROT_RECV_BYTE;
			info.data = &data->byte;
		}
		else {
		    info.smb_proto = SMBUS_PROT_SEND_BYTE;
		}
		break;

        case I2C_SMBUS_BYTE_DATA:
		info.cmd_valid = true;
		info.command = command;
		info.data = &data->byte;
		info.size = 1;
		info.flags = flags;
		if (read_write == I2C_SMBUS_READ)
			info.smb_proto = SMBUS_PROT_RD_BYTE;
		else
			info.smb_proto = SMBUS_PROT_WR_BYTE;
		break;

	case I2C_SMBUS_WORD_DATA:
		info.cmd_valid = true;
		info.command = command;
		info.data = (unsigned char *)(&data->word);
		info.size = 2;
		info.flags = flags;
		if (read_write == I2C_SMBUS_READ)
			info.smb_proto = SMBUS_PROT_RD_WORD;
		else
			info.smb_proto = SMBUS_PROT_WR_WORD;
		break;

        case I2C_SMBUS_BLOCK_DATA:
        case I2C_SMBUS_I2C_BLOCK_DATA:
		info.cmd_valid = true;
		info.command = command;
		info.data = &data->block[1];
		info.flags = flags;

		if (read_write == I2C_SMBUS_READ) {
			info.smb_proto = SMBUS_PROT_BLK_RD;
			/* Refer to RD_BYTE_COUNT in reg 0x30 about 'block read'
			 * If '0', protocol(hw) returns data byte count as part of
			 * response.
			 */
			info.size = 0;
		}
		else {
			info.smb_proto = SMBUS_PROT_BLK_WR;
			/* i2c-core passes the length in this field */
			info.size = data->block[0];
		}
		break;

	case I2C_SMBUS_BLOCK_PROC_CALL:
		info.cmd_valid = true;
		info.command = command;
		info.data = &data->block[1];
		info.flags = flags;
		info.size = data->block[0];
		info.smb_proto = SMBUS_PROT_BLK_WR_BLK_RD_PROC_CALL;
		break;

	default:
		dev_err(dev->dev, "Unsupported transaction\n");
		up(&dev->xfer_lock);
		return -EINVAL;
	}

	/* Handle large packet by spliting into SMB_MAX_DATA_SIZE packet */
	smb_xfer_size = (int)info.size;
	if ((info.smb_proto == SMBUS_PROT_BLK_RD) ||
		(info.smb_proto == SMBUS_PROT_BLK_WR_BLK_RD_PROC_CALL))
		data->block[0] = 0;

	while (smb_xfer_size) {
		if (info.size >= SMB_MAX_DATA_SIZE)
			info.size = SMB_MAX_DATA_SIZE;

		if (read_write == I2C_SMBUS_READ) {
			/* Refer to i2c_smbus_read_byte for params passed. */
			rc = iproc_smb_data_recv(i2c_adap, addr, &info,
						&num_bytes_read);
			/* if failed due to bus hang, but recovered, retry once */
			if (rc == -ECOMM)
				rc = iproc_smb_data_recv(i2c_adap, addr, &info,
							&num_bytes_read);
			/* pass the actual amount of data sent by slave */
			if ((info.smb_proto == SMBUS_PROT_BLK_RD) ||
			  (info.smb_proto == SMBUS_PROT_BLK_WR_BLK_RD_PROC_CALL))
				if (rc == 0)
					data->block[0] += num_bytes_read;
		} else {
			/* Refer to i2c_smbus_write_byte params passed. */
			rc = iproc_smb_data_send(i2c_adap, addr, &info);
			/* if failed due to bus hang, but recovered, retry */
			if (rc == -ECOMM)
				rc = iproc_smb_data_send(i2c_adap, addr, &info);
		}

		if (rc < 0) {
			dev_info(dev->dev, "%s error accessing\n",
			    (read_write == I2C_SMBUS_READ) ? "Read" : "Write");
			up(&dev->xfer_lock);
			return -EREMOTEIO;
		}
		if (info.size == SMB_MAX_DATA_SIZE) {
			smb_xfer_size -= SMB_MAX_DATA_SIZE;
			info.size = smb_xfer_size;
			info.data += SMB_MAX_DATA_SIZE;
			/* Adjust I2c device register offset */
			info.command += SMB_MAX_DATA_SIZE;
		} else {
			break;
		}
	}

	msleep(1);
	up(&dev->xfer_lock);

	return (rc);
}

static int iproc_intr_enable(struct iproc_smb_drv_int_data *dev, u32 bmap)
{
	void __iomem *base_addr = dev->base;
	unsigned int regval;

	regval = readl(base_addr + CCB_SMB_EVTEN_REG);
	regval |= bmap;
	writel(regval, base_addr + CCB_SMB_EVTEN_REG);

	/*
	 * Store all interrupts enabled so far. Note bmap can have only
	 * 'incremental' set of events
	 */
	dev->evt_enable_bmap = regval;

	return 0;
}

static int iproc_intr_disable(struct iproc_smb_drv_int_data *dev, u32 bmap)
{
	void __iomem *base_addr = dev->base;
	unsigned int regval;

	regval = readl(base_addr + CCB_SMB_EVTEN_REG);
	regval &= ~bmap;
	writel(regval, base_addr + CCB_SMB_EVTEN_REG);

	dev->evt_enable_bmap = regval;

	return 0;
}

/* Verify this sequence with hw engg */
static int iproc_smbus_block_deinit(struct iproc_smb_drv_int_data *dev)
{
	unsigned int regval;
	int rc;

	/* Disable all interrupts */
	regval = 0x0;

	writel(regval, dev->base + CCB_SMB_EVTEN_REG);

	/* Check if a transaction is in progress */
	rc = iproc_smb_startbusy_wait(dev);
	if (rc < 0) {
		dev_err(dev->dev, "A transaction is still in progress");
		/* Wait for some time */
		udelay(100);
	}

	/* Disable SMBus block */
	regval = readl(dev->base + CCB_SMB_CFG_REG);
	regval &= ~CCB_SMB_CFG_SMBEN_MASK;
	writel(regval, dev->base + CCB_SMB_CFG_REG);

	/* Wait for some time */
	udelay(100);

	/* Put the block under reset. Note the RESET bit in reg 0x0 is
	* self clearing
	*/
	regval = CCB_SMB_CFG_RST_MASK;
	writel(regval, dev->base + CCB_SMB_CFG_REG);

	return 0;
}

static u32 iproc_smb_funcs(struct i2c_adapter *adapter)
{
	/* I2C_FUNC_SMBUS_I2C_BLOCK */
	return (I2C_FUNC_SMBUS_BYTE | I2C_FUNC_SMBUS_BYTE_DATA |
		I2C_FUNC_SMBUS_WORD_DATA | I2C_FUNC_SMBUS_BLOCK_DATA);
}

static struct i2c_algorithm iproc_smb_algorithm = {
	/*.name           = "iproc-smb", */
	.smbus_xfer     = iproc_smb_xfer,
	.master_xfer    = NULL,
	.functionality  = iproc_smb_funcs,
};

static int iproc_smb_probe(struct platform_device *pdev)
{
	int rc=0, irq;
	struct iproc_smb_drv_int_data *dev;
	struct i2c_adapter *adap;
	struct resource *res;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->dev = &pdev->dev;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dev->base = devm_ioremap_resource(dev->dev, res);
	if (IS_ERR(dev->base))
		return PTR_ERR(dev->base);

	/* Get the interrupt number */
	irq = platform_get_irq(pdev, 0);
	if (irq <= 0) {
		dev_err(dev->dev, "no irq resource\n");
		return irq;
	}

	init_MUTEX(&dev->xfer_lock);
	init_completion(&dev->ses_done);
	dev->irq = irq;

	/* Default value, can be changed after initial testing */
	dev->enable_evts = ENABLE_INTR;

	platform_set_drvdata(pdev, dev);

	/*
	 * Init internal regs, disable intrs (and then clear intrs), set fifo
	 * thresholds, etc.
	 */
	iproc_smbus_block_init(dev);

	/* Register ISR handler */
	rc = devm_request_irq(dev->dev, dev->irq, iproc_smb_isr, 0,
				pdev->name, dev);
	if (rc) {
		dev_err(dev->dev, "unable to request irq %i\n", irq);
		goto err_smb_deinit;
	}

	adap = &dev->adapter;
	i2c_set_adapdata(adap, dev);
	adap->owner = THIS_MODULE;
	adap->class = UINT_MAX;
	adap->algo = &iproc_smb_algorithm;
	adap->dev.parent = &pdev->dev;
	adap->nr = pdev->id;
	adap->dev.of_node = pdev->dev.of_node;
	strlcpy(adap->name, "iproc-smbus", sizeof(adap->name));

	rc = i2c_add_numbered_adapter(adap);
	if (rc)
		goto err_free_irq;

	/* Turn on default set of interrupts */
	/*
	 * For Rx, enable RX fifo full, threshold hit interrupts. Other rx
	 * interrupts will be set in the read/recv transactions, as required
	 * For Tx, enable fifo under run intr. Other intrs will be set in send
	 * write access functions
	 */
	iproc_intr_enable(dev, CCB_SMB_MSTRRXFIFOFULLEN_MASK);

	return 0;

err_free_irq:
	free_irq(dev->irq, dev);

err_smb_deinit:
	iproc_smbus_block_deinit(dev);
	platform_set_drvdata(pdev, NULL);

	dev_err(dev->dev, "probe failed, error=%d", rc);
	return (rc);
}

static int iproc_smb_remove(struct platform_device *pdev)
{
	struct iproc_smb_drv_int_data *dev = platform_get_drvdata(pdev);
	unsigned int regval;

	/* Disable interrupts. */
	/* Verify: Should we wait for any in-progress xact to complete? */
	synchronize_irq(dev->irq);
	iproc_intr_disable(dev, ~0);
	free_irq(dev->irq, dev);

	/* Disable SMbus block */
	regval = readl(dev->base + CCB_SMB_CFG_REG);
	regval &= ~CCB_SMB_CFG_SMBEN_MASK;
	writel(regval, dev->base + CCB_SMB_CFG_REG);

	i2c_del_adapter(&dev->adapter);

	platform_set_drvdata(pdev, NULL);

	iproc_smbus_block_deinit(dev);

	devm_iounmap(&pdev->dev, dev->base);

	return 0;
}

static const struct of_device_id bcm_iproc_smb_of_match[] = {
	{ .compatible = "brcm,iproc-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, bcm_iproc_smb_of_match);

static struct platform_driver bcm_iproc_smbus_driver = {
	.driver = {
		.name = "bcm-iproc-smbus",
		.of_match_table = bcm_iproc_smb_of_match,
	},
	.probe = iproc_smb_probe,
	.remove = iproc_smb_remove,
};
module_platform_driver(bcm_iproc_smbus_driver);

MODULE_AUTHOR("Broadcom Corporation");
MODULE_DESCRIPTION("IPROC SMBus Bus Driver");
MODULE_LICENSE("GPL");
