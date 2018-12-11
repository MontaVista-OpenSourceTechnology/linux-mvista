/*
 * Copyright 2017 Broadcom Limited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

/****************************************************************************/
/**
*  @file    bcm_dwc_udc.c
*
*  @brief   Broadcom Linux driver for DWC USB 2.0 Device Controller (UDC)
*
*  This driver implements the Linux Gadget driver API as defined in usb_gadget.h
*
*  @note
*
*  This driver was written with the intent of being able to support any
*  variations on how this block is integrated into different Broadcom chips.
*
*  There is a requirement on how the DWC UDC is configured. In particular, this
*  driver requires that the following options be defined and enabled in the
*  UDC core.
*
*       UDC20AHB_CNAK_CLR_ENH_CC
*       UDC20AHB_STALL_SET_ENH_CC
*       UDC20AHB_SNAK_ENH_CC
*
*  Some other UDC attributes can be supported by setting compile time options
*  or with some minor modifications to the source code. Ideally these would
*  be run-time info that is provided by the device instance to the driver.
*  These attributes include the following.
*
*       IPROC_UDC_EP_CNT
*       IPROC_UDC_EP_MAX_PKG_SIZE
*       Type of each endpoint: Control, IN, OUT, or Bidirectional
*/
/****************************************************************************/
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/phy.h>
#include <linux/usb/iproc_usb.h>

#include "xgs_iproc_udc.h"

#define XGS_IPROC_UDC_NAME                  "xgs-iproc-udc"

#define DMA_ADDR_INVALID                    (~(dma_addr_t)0)
/*
 * FRAME_NUM_INVALID is used for ISOC IN transfers for frame alignment.
 * The device specifies the interval at which it wants to do transfers,
 * but the host initiates all transfers. If the interval is some multiple
 * number of frames, the device has no idea which frame in an interval
 * window the host is going to start transfers. This could even be at a
 * point many frames beyond the current window, as the starting point
 * can be very application dependant and subject to an indeterminate
 * amount of latency.
 */
#define FRAME_NUM_INVALID                   (~(u32)0)
#define ENOERROR                            0

#define DATA_EP_IN_INDEX                    3
#define DATA_EP_OUT_INDEX                   4

/* ---- Private Function Prototypes -------------------------------------- */
#ifdef IPROC_UDC_DEBUG
static void iproc_dbg_dma_dump(struct iproc_udc *udc);
static void iproc_dbg_dma_dump_desc(char *label, struct iproc_udc_dma_desc *virt,
					struct iproc_udc_dma_desc *phys);
static void iproc_dbg_dma_dump_ep(struct iproc_ep *ep);
#endif /* IPROC_UDC_DEBUG */

/* ---- Private Variables ------------------------------------------------ */
static const struct {
	const char *name;
	const int type;
	const int msize;
	const struct usb_ep_caps caps;
} xgs_iproc_ep_info[] = {
#define EP_INFO(_name, _type, _size, _caps) \
	{ \
		.name = _name, \
		.type = _type, \
		.msize = _size, \
		.caps = _caps, \
	}

	EP_INFO("ep0", USB_ENDPOINT_XFER_CONTROL, IPROC_UDC_CTRL_MAX_PKG_SIZE,
		USB_EP_CAPS(USB_EP_CAPS_TYPE_CONTROL, USB_EP_CAPS_DIR_ALL)),
	EP_INFO("ep1in", USB_ENDPOINT_XFER_ISOC, IPROC_UDC_EP_MAX_PKG_SIZE,
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ISO, USB_EP_CAPS_DIR_IN)),
	EP_INFO("ep2out", USB_ENDPOINT_XFER_ISOC, IPROC_UDC_EP_MAX_PKG_SIZE,
		USB_EP_CAPS(USB_EP_CAPS_TYPE_ISO, USB_EP_CAPS_DIR_OUT)),
	EP_INFO("ep3in", USB_ENDPOINT_XFER_BULK, IPROC_UDC_EP_MAX_PKG_SIZE,
		USB_EP_CAPS(USB_EP_CAPS_TYPE_BULK, USB_EP_CAPS_DIR_IN)),
	EP_INFO("ep4out", USB_ENDPOINT_XFER_BULK, IPROC_UDC_EP_MAX_PKG_SIZE,
		USB_EP_CAPS(USB_EP_CAPS_TYPE_BULK, USB_EP_CAPS_DIR_OUT)),
	EP_INFO("ep5in", USB_ENDPOINT_XFER_INT, IPROC_UDC_EP_MAX_PKG_SIZE,
		USB_EP_CAPS(USB_EP_CAPS_TYPE_INT, USB_EP_CAPS_DIR_IN)),
	EP_INFO("ep6out", USB_ENDPOINT_XFER_INT, IPROC_UDC_EP_MAX_PKG_SIZE,
		USB_EP_CAPS(USB_EP_CAPS_TYPE_INT, USB_EP_CAPS_DIR_OUT)),
#undef EP_INFO
};

/***********************************************************************
 * Convenience functions
 ***********************************************************************/
static inline struct iproc_udc *gadget_to_udc(struct usb_gadget *g)
{
	return container_of(g, struct iproc_udc, gadget);
}

static inline struct iproc_ep *our_ep(struct usb_ep *ep)
{
	return container_of(ep, struct iproc_ep, usb_ep);
}

static inline struct iproc_ep_req *our_req(struct usb_request *req)
{
	return container_of(req, struct iproc_ep_req, usb_req);
}

/****************************************************************************
 * DMA descriptor chain routines.
 *
 *  dma_desc_chain_reset - Initialize chain in preparation for transfer
 *  dma_desc_chain_full - Indicates if no descriptors in chain for available for use.
 *  dma_desc_chain_alloc - Get next free descriptor for use. Have to check if chain not full first.
 *  dma_desc_chain_empty - Indicates if no descriptors in the chain are being used.
 *  dma_desc_chain_head - Pointer to 1st entry in chain. Have to check if chain not empty first.
 *  dma_desc_chain_free - Frees up 1st entry for use. Only do this if DMA for this descriptor has completed.
 *
 ***************************************************************************/
static inline struct iproc_udc_dma_desc *dma_desc_chain_alloc(struct iproc_ep *ep)
{
	u32 idx;

	idx = ep->dma.add_idx++;

	return &ep->dma.vir_addr->desc[IPROC_EP_DMA_DESC_IDX(idx)];
}

static inline int dma_desc_chain_empty(struct iproc_ep *ep)
{
	return ep->dma.add_idx == ep->dma.rm_idx;
}

static inline void dma_desc_chain_free(struct iproc_ep *ep)
{
	ep->dma.rm_idx++;
}

static inline int dma_desc_chain_full(struct iproc_ep *ep)
{
	return (!dma_desc_chain_empty(ep) && (IPROC_EP_DMA_DESC_IDX(ep->dma.add_idx) == IPROC_EP_DMA_DESC_IDX(ep->dma.rm_idx)));
}

static inline struct iproc_udc_dma_desc *dma_desc_chain_head(struct iproc_ep *ep)
{
	return (&ep->dma.vir_addr->desc[IPROC_EP_DMA_DESC_IDX(ep->dma.rm_idx)]);
}

static inline void dma_desc_chain_reset(struct iproc_ep *ep)
{
	ep->dma.add_idx = 0;
	ep->dma.rm_idx = 0;
}

/****************************************************************************
 *
 * Platform device level alloc / free of memory used for DMA descriptors.
 * A single block of memory static in size is used for DMA descriptors.
 * Each endpoint has a small number of descriptors for its exclusive use.
 * These are chained in a loop. See bcm_udc_dwc.h and iproc_dma_ep_init()
 * for more details.
 *
 ***************************************************************************/
static int iproc_platform_dma_alloc(struct platform_device *platformDevP,
					struct iproc_udc *udc)
{
	udc->dma.vir_addr = dma_alloc_coherent(&platformDevP->dev,
				sizeof(struct iproc_udc_dma),
				(dma_addr_t *)&udc->dma.phy_addr, GFP_KERNEL);

	if (!udc->dma.vir_addr) {
		dev_err(udc->dev, "dma_alloc_coherent() failed\n");
		return -ENOMEM;
	}

	return ENOERROR;
}

static void iproc_platform_dma_free(struct platform_device *platformDevP,
					struct iproc_udc *udc)
{
	int idx;

	dma_free_coherent(&platformDevP->dev, sizeof(struct iproc_udc_dma),
			udc->dma.vir_addr, (dma_addr_t)udc->dma.phy_addr);

	for (idx = 0; idx < IPROC_UDC_EP_CNT; idx ++) {
		if (udc->ep[idx].dma.align_buff) {
			dma_free_coherent(NULL, udc->ep[idx].dma.align_len,
					udc->ep[idx].dma.align_buff,
					udc->ep[idx].dma.align_addr);
			udc->ep[idx].dma.align_buff = NULL;
		}
	}
}

/***************************************************************************
 * Routines for debug dump of DMA descriptors
 **************************************************************************/
#ifdef IPROC_UDC_DEBUG
static void iproc_dbg_dma_dump(struct iproc_udc *udc)
{
	int idx;

	for (idx = 0; idx < IPROC_UDC_EP_CNT; idx++)
		iproc_dbg_dma_dump_ep(&udc->ep[idx]);
}

static void iproc_dbg_dma_dump_desc(char *label, struct iproc_udc_dma_desc *virt,
					struct iproc_udc_dma_desc *phys)
{
	printk("%s virt=0x%p phys=0x%p: 0x%08x 0x%08x 0x%08x",
		label, virt, phys, virt->status, virt->reserved, virt->buf_addr);
}

static void iproc_dbg_dma_dump_ep(struct iproc_ep *ep)
{
	int idx;

	printk("EP %d DMA\n", ep->num);
	printk("   setup\n");
	iproc_dbg_dma_dump_desc("       ",
			(struct iproc_udc_dma_desc *)&ep->dma.vir_addr->setup,
			(struct iproc_udc_dma_desc *)&ep->dma.phy_addr->setup);
	printk("   desc\n");

	for (idx = 0; idx < IPROC_EP_DMA_DESC_CNT; idx++) {
		iproc_dbg_dma_dump_desc("       ", &ep->dma.vir_addr->desc[idx],
					&ep->dma.phy_addr->desc[idx]);

		/* Don't bother displaying entries beyond the last. */
		if (IPROC_USBD_READ(ep->dma.vir_addr->desc[idx].status) &
			REG_DMA_STAT_LAST_DESC)
			break;
	}
}
#endif /* IPROC_UDC_DEBUG */

/****************************************************************************
 * Initialization of DMA descriptors at the endpoint level.
 ***************************************************************************/
static void iproc_dma_ep_init(struct iproc_ep *ep)
{
	struct iproc_udc *udc = ep->udc;
	int idx;

	/** @todo shorten names to virtAddr physAddr?? */
	ep->dma.vir_addr = &udc->dma.vir_addr->ep[ep->num];
	ep->dma.phy_addr = &udc->dma.phy_addr->ep[ep->num];

	/*
	 * Control endpoints only do setup in the OUT direction, so only need to set the
	 * buffer address for that direction. The buffer is set, even if not a control
	 * endpoint, just to simplify things. There's no harm with this.
	 */
	ep->dma.vir_addr->setup.status = cpu_to_le32(REG_DMA_STAT_BUF_HOST_BUSY);
	wmb();
	iproc_usbd_ep_dma_buf_addr_set(udc->usbd_regs, ep->num, USB_DIR_OUT, &ep->dma.phy_addr->setup);

	/*
	 * Take ownership of the DMA descriptors, and chain them in a loop. This allows a small number
	 * descriptors to be used for requests. Need to have the DWC DMA Descriptor Update option enabled
	 * in the device control register in order to do this. When a transfer for a descriptor completes,
	 * the descriptor will get re-used if there's still data left in a request to transfer. See the
	 * iproc_dma_data_rm_done() and iproc_dma_data_add_ready() routines.
	 */
	 /** @todo Put these in endpoint context?? */
	for (idx = 0; idx < IPROC_EP_DMA_DESC_CNT; idx++) {
		ep->dma.vir_addr->desc[idx].status =
			cpu_to_le32(REG_DMA_STAT_BUF_HOST_BUSY);
		wmb();
		ep->dma.vir_addr->desc[idx].next_addr =
			cpu_to_le32((u32)&ep->dma.phy_addr->desc[idx+1]);
	}
	ep->dma.vir_addr->desc[(IPROC_EP_DMA_DESC_CNT - 1)].next_addr =
			cpu_to_le32((u32)&ep->dma.phy_addr->desc[0]);

	/*
	 * To simplify things, register the descriptor chain in both directions. Control endpoints are the
	 * only type that will be transferring in both directions, but they will only be transferring in one
	 * direction at a time, so should not be any issues with using the same descriptor set for both directions.
	 * For single direction endpoints, the other direction will not be used.
	 */

	iproc_usbd_ep_dma_desc_addr_set(udc->usbd_regs, ep->num, USB_DIR_OUT,
						&ep->dma.phy_addr->desc[0]);
	iproc_usbd_ep_dma_desc_addr_set(udc->usbd_regs, ep->num, USB_DIR_IN,
						&ep->dma.phy_addr->desc[0]);
}

/****************************************************************************
 * DMA data routines.
 *
 * A gadget usb_request buf is used for the data. The entire buf contents may
 * or may not fit into the descriptor chain at once. When the DMA transfer
 * associated with a descriptor completes, the descriptor is re-used to add
 * more segments of the usb_request to the chain as necessary.
 *
 *  iproc_dma_data_init - Initialization in preparation for DMA of usb_request.
 *  iproc_dma_data_add_ready - Adds usb_request segments into DMA chain until full or no segments left
 *  iproc_dma_data_rm_done - Removes usb_request segments from DMA chain that have completed transfer
 *  iproc_dma_data_finish - Final stage of DMA of the usb_request
 *
 ***************************************************************************/
static void iproc_dma_data_init(struct iproc_ep *ep)
{
	struct iproc_ep_req *req;
	struct iproc_udc *udc = ep->udc;

	req = list_first_entry(&ep->list_queue, struct iproc_ep_req, list_node);

	if (req->dma_aligned) {
		/*
		 * This buffer needs to be aligned in order to DMA. We do this by copying into a special buffer we
		 * have for this purpose. Save the original DMA physical address so it can be restored later.
		 * This may not be used, but we'll do it anyways. Then set the DMA address to the aligned buffer
		 * address. Only the DMA physical address is used for the transfers, so the original buffer virtual
		 * address does not need to be changed. Then copy the data into the aligned buffer.
		 */
		req->orig_dma_addr = req->usb_req.dma;
		req->usb_req.dma = ep->dma.align_addr;
		if (ep->dir == USB_DIR_IN) {
			memcpy(ep->dma.align_buff, req->usb_req.buf, req->usb_req.length);
		}
	}

	ep->dma.done = 0;
	ep->dma.done_len = 0;
	ep->dma.todo_len = ep->dma.usb_req->length;
	ep->dma.buf_addr = ep->dma.usb_req->dma;
	ep->dma.status = REG_DMA_STAT_RX_SUCCESS;

	if ((ep->dir == USB_DIR_IN) && (ep->type != USB_ENDPOINT_XFER_ISOC)) {
		/*
		 * For IN transfers, do not need to segment the buffer into max packet portions
		 * for the DMA descriptors. The hardware will automatically segment into max
		 * packet sizes as necessary.
		 */
		ep->dma.max_buf_len = ep->usb_ep.maxpacket;

		/*
		 * If the request is of zero length, then force the zero flag so iproc_dma_data_add_ready()
		 * will queue the request. Conversely, if the gadget has set the zero flag, leave
		 * it set only if it is needed (request length is a multiple of maxpacket)
		 */
		if (ep->dma.usb_req->length == 0)
			ep->dma.usb_req->zero = 1;
		else if (ep->dma.usb_req->zero)
			ep->dma.usb_req->zero =
				(ep->dma.usb_req->length % ep->usb_ep.maxpacket)? 0 : 1;
	} else {
		ep->dma.max_buf_len = ep->usb_ep.maxpacket;
	}

	dma_desc_chain_reset(ep);

	iproc_usbd_ep_irq_en(udc->usbd_regs, ep->num, ep->dir);
}

static void iproc_dma_data_finish(struct iproc_ep *ep)
{
	struct iproc_ep_req *req;
	struct iproc_udc *udc = ep->udc;

	if (ep->num == DATA_EP_OUT_INDEX) {
		iproc_usbd_ep_nak_en(udc->usbd_regs, DATA_EP_OUT_INDEX, USB_DIR_OUT);
	}

	iproc_usbd_ep_irq_dis(udc->usbd_regs, ep->num, ep->dir);
	iproc_usbd_ep_dma_dis(udc->usbd_regs, ep->num, ep->dir);

	req = list_first_entry(&ep->list_queue, struct iproc_ep_req, list_node);

	if (req->dma_aligned) {
		/*
		 * The original request buffer was not aligned properly, so a special buffer was used
		 * for the transfer. Copy the aligned buffer contents into the original. Also restore
		 * the original dma physical address.
		 */
		if (ep->dir == USB_DIR_OUT) {
			memcpy(req->usb_req.buf, ep->dma.align_buff, req->usb_req.length);
		}
		req->usb_req.dma = req->orig_dma_addr;
	}
}

static void iproc_dma_data_add_ready(struct iproc_ep *ep)
{
	struct iproc_udc *udc = ep->udc;
	volatile struct iproc_udc_dma_desc *dma_desc = NULL;
	u32 status;
	u32 len;
	int enable_rxdma = 0;

	/* No descriptor required for USB_DIR_OUT && "dma.todo_len == 0" */
	if ((!ep->dma.todo_len) && (ep->dir == USB_DIR_OUT)) {
		return;
	}

	/*
	 * DMA must be disabled while this loop is running, as multi-descriptor transfers
	 * will have the descriptor chain in an intermediate state until the last descriptor
	 * is written and the chain terminated.
	 */
	if ((ep->dir == USB_DIR_OUT) && iproc_usbd_rx_dma_status(udc->usbd_regs)) {
		enable_rxdma = 1;
		iproc_usbd_rx_dma_dis(udc->usbd_regs);
    }

	if (!ep->dma.todo_len)
		ep->dma.usb_req->zero = 1;

	/*
	 * Will only have one request in the chain at a time. Add request segments to the
	 * chain until all parts of the request have been put in the chain or the chain
	 * has no more room.
	 */
	while (!dma_desc_chain_full(ep) && (ep->dma.todo_len || ep->dma.usb_req->zero)) {
		/*
		 * Get the next descriptor in the chain, and then fill the descriptor contents as needed.
		 * Do not set the descriptor buffer status to ready until last to ensure there's no
		 * contention with the hardware.
		 */
		dma_desc = dma_desc_chain_alloc(ep);

		len = ep->dma.todo_len < ep->dma.max_buf_len ? ep->dma.todo_len : ep->dma.max_buf_len;
		ep->dma.todo_len -= len;

		status = 0;

		if (len < ep->dma.max_buf_len) {
			/*
			 * If this segment is less than the max, then it is the last segment. There's no need to
			 * send a closing ZLP, although this segment might be a ZLP. Regardless, clear the ZLP flag
			 * to ensure that the processing of this request finishes. Also set the end of the descriptor
			 * chain.
			 */
			ep->dma.usb_req->zero = 0;
			status |= REG_DMA_STAT_LAST_DESC;
		} else if ((ep->dma.todo_len == 0) && !ep->dma.usb_req->zero) {
			/*
			 * Segment is of the max packet length. Since there's nothing left, it has to also be the last
			 * last segment. No closing ZLP segment requested, just set the end of the descriptor chain.
			 */
			status |= REG_DMA_STAT_LAST_DESC;
		}

		if ((ep->dir == USB_DIR_IN) && (ep->type == USB_ENDPOINT_XFER_ISOC)) {
			/*
			 * Increment the frame number for transmit, then use it for the next packet. The frame number
			 * may get larger than its 13-bit size, but the mask will handle the wrap-around so we don't
			 * need to add checks for this condition. E.g. 0x7ff + 1 = 0x800. 0x800 & 0x7ff = 0 which
			 * is the next number in the sequence.
			 */
			/** @todo Handle ISOC PIDs and frame numbers used with HS high bandwidth transfers */
			/** @todo Might not need to set the last descriptor status. Currently restricting
			 * IN ISOC transfers to the max packet size.
			 */
			status |= REG_DMA_STAT_LAST_DESC;

			ep->dma.frame_num += ep->dma.frame_incr;
			status |= ((ep->dma.frame_num << REG_DMA_STAT_FRAME_NUM_SHIFT) &
					REG_DMA_STAT_FRAME_NUM_MASK);
		}

		IPROC_USBD_WRITE(dma_desc->buf_addr, ep->dma.buf_addr);
		status |= (len << REG_DMA_STAT_BYTE_CNT_SHIFT);
		IPROC_USBD_WRITE(dma_desc->status, status | REG_DMA_STAT_BUF_HOST_READY);
		wmb();
		ep->dma.buf_addr += len;

		if ((ep->dir == USB_DIR_IN) && (ep->type == USB_ENDPOINT_XFER_ISOC)) {
			/* With ISOC transfers, only enable one DMA descriptors at a time.
			 */
			/** @todo Determine if FIFO will overflow. If it does not, then can remove this check.
			 * This may not even be an issue if the buffer size is restricted to the max packet size
			 * when a request is submitted to the endpoint.
			 */
			break;
		}
	} /* while */

	/* Set LAST bit on last descriptor we've configured */
	if (dma_desc) {
		IPROC_USBD_BITS_SET(dma_desc->status, REG_DMA_STAT_LAST_DESC);
		wmb();
	}

	if (enable_rxdma)
		iproc_usbd_rx_dma_en(udc->usbd_regs);
}

static void iproc_dma_data_rm_done(struct iproc_ep *ep)
{
	struct iproc_udc *udc = ep->udc;
	volatile struct iproc_udc_dma_desc *dma_desc;
	u32 status;
    u32 len = 0x0badffff;

	/*
	 * Will only have one request in the chain at a time. Remove any completed
	 * request segments from the chain so any segments awaiting transfer can
	 * be put in the chain.
	 */
	while (!dma_desc_chain_empty(ep)) {
		/*
		 * Examine the first entry in the chain. If its status is not done, then there's
		 * nothing to remove.
		 */
		dma_desc = dma_desc_chain_head(ep);

		if ((IPROC_USBD_READ(dma_desc->status) & REG_DMA_STAT_BUF_MASK) !=
				REG_DMA_STAT_BUF_DMA_DONE) {
			if (ep->dir == USB_DIR_OUT)  {
				if ((len == ep->dma.max_buf_len) || (len == 0x0badffff))
					return;
				else {
					/* short packet: no more packet for this descriptor
					 * chain, free the descriptors.
					 */
					while (!dma_desc_chain_empty(ep)) {
						dma_desc = dma_desc_chain_head(ep);
						IPROC_USBD_WRITE(dma_desc->status, REG_DMA_STAT_BUF_HOST_BUSY);
						dma_desc_chain_free(ep);
					}
					ep->dma.todo_len = 0;
					wmb();
					break;
				}
			} else {
				return;
			}
		}
		/*
		 * The transfer of this request segment has completed. Save the status info and then
		 * take ownership of the descriptor. It is simpler to do this than modifying parts of
		 * the descriptor in order to take ownership. Don't put the descriptor back in the chain
		 * until all info affected by the status has been updated, just to be safe.
		 */
		status = IPROC_USBD_READ(dma_desc->status);
		IPROC_USBD_WRITE(dma_desc->status, REG_DMA_STAT_BUF_HOST_BUSY);
		wmb();

		len = (status & REG_DMA_STAT_NON_ISO_BYTE_CNT_MASK) >>
				REG_DMA_STAT_NON_ISO_BYTE_CNT_SHIFT;
		ep->dma.done_len += len;
		ep->dma.usb_req->actual += len;

		if ((status & REG_DMA_STAT_RX_MASK) != REG_DMA_STAT_RX_SUCCESS) {
			ep->dma.status = status & REG_DMA_STAT_RX_MASK;
			ep->dma.usb_req->status = -EIO;
			dev_warn(udc->dev, "%s: DMA error: desc=0x%p status=0x%x len=%d add=0x%x remove=0x%x\n",
			            ep->usb_ep.name, dma_desc, status, len, ep->dma.add_idx, ep->dma.rm_idx);
		}

		if ((ep->dir == USB_DIR_IN) && (ep->type == USB_ENDPOINT_XFER_ISOC)){
			/** @todo Determine if this special processing needs to be done. May not to do this if the
			 * buffer size is restricted to the max packet size when a request is submitted to the endpoint.
			 */
			if (ep->dma.usb_req->actual == ep->dma.usb_req->length)
				ep->dma.usb_req->status = ENOERROR;
			dma_desc_chain_reset(ep);
		} else {
			dma_desc_chain_free(ep);
		}
	}

	/* When last segment processed, update status if there has not been an error */
	if (!ep->dma.todo_len && (ep->dma.usb_req->status == -EINPROGRESS))
		ep->dma.usb_req->status = ENOERROR;
}

/****************************************************************************
 * Control Endpoint SETUP related routines.
 *
 * iproc_ep_setup_init - Prepares for next SETUP Rx. Status indicates if STALL req'd.
 * iproc_ep_setup_process - Handle Rx of a SETUP.
 ***************************************************************************/
static void iproc_ep_setup_init(struct iproc_ep *ep, int status)
{
	struct iproc_udc *udc = ep->udc;

	/* Re-enable transfers to the SETUP buffer, clear IN and OUT NAKs, and re-enable OUT interrupts. */
	ep->dma.vir_addr->setup.status = cpu_to_le32(REG_DMA_STAT_BUF_HOST_READY);
	ep->dir = USB_DIR_OUT;
	ep->stopped = 0;

	if (status == ENOERROR) {
		/* Handling of previous SETUP was OK. Just clear any NAKs. */
		iproc_usbd_ep_nak_clear(udc->usbd_regs, ep->num, USB_DIR_OUT);
		iproc_usbd_ep_nak_clear(udc->usbd_regs, ep->num, USB_DIR_IN);
	} else {
		/*
		 * Handling of previous SETUP failed. Set the STALL. This will get cleared
		 * when the next SETUP is rx'd.
		 */
		iproc_usbd_ep_stall_en(udc->usbd_regs, ep->num, USB_DIR_IN);
		iproc_usbd_ep_stall_en(udc->usbd_regs, ep->num, USB_DIR_OUT);
	}

	iproc_usbd_ep_irq_en(udc->usbd_regs, ep->num, USB_DIR_OUT);
	iproc_usbd_ep_dma_en(udc->usbd_regs, ep->num, USB_DIR_OUT);
}

void iproc_ep_setup_process(struct iproc_ep *ep, struct usb_ctrlrequest *setup)
{
	struct iproc_udc *udc = ep->udc;
	u32 value;
	u32 index;
	u32 length;
	int status;

	value = le16_to_cpu(setup->wValue);
	index = le16_to_cpu(setup->wIndex);
	length = le16_to_cpu(setup->wLength);

	/*
	 * Any SETUP packets appearing here need to be handled by the gadget driver. Some SETUPs may have
	 * already been silently handled and acknowledged by the DWC UDC. The exceptions to this rule are the
	 * USB_REQ_SET_CONFIGURATION and USB_REQ_SET_INTERFACE, which have been only partially handled with
	 * the expectation that some additional software processing is required in order to complete these requests.
	 * Thus, they have not been acknowledged by the DWC UDC. There is no DATA stage for these requests.
	 */

	/*
	 * Set the direction of the subsequent DATA stage of a control transfer. This is an
	 * optional stage. It may not exist for all control transfers. If there is a DATA
	 * stage, this info is used for DMA operations for any requests received from the
	 * Gadget driver.
	 */

	ep->dir = setup->bRequestType & USB_ENDPOINT_DIR_MASK;

	if (ep->num != 0) {
		/** @todo Make changes here if the Linux USB gadget ever supports a control endpoint other
		 * than endpoint 0. The DWC UDC supports multiple control endpoints, and this driver has
		 * been written with this in mind. To make things work, really need to change the Gadget
		 * setup() callback parameters to provide an endpoint context, or add something similar
		 * to the usb_ep structure, or possibly use a usb_request to hold a setup data packet.
		 */
		dev_err(udc->dev, "%s: control transfer not supported\n", ep->usb_ep.name);
		status = -EOPNOTSUPP;
	} else {
		/*
		 * Forward the SETUP to the gadget driver for processing. The appropriate directional
		 * interrupt and NAK clear will happen when the DATA stage request is queued.
		 */
		spin_unlock(&udc->lock);
		status = udc->gadget_driver->setup(&udc->gadget, setup);
		spin_lock(&udc->lock);
	}

	if (status < 0) {
		/*
		 * Error occurred during the processing of the SETUP, so enable STALL. This condition
		 * can only be cleared with the RX of another SETUP, so prepare for that event.
		 */
		dev_err(udc->dev, "%s: SETUP %02x.%02x STALL; status=%d\n",
		             ep->usb_ep.name, setup->bRequestType, setup->bRequest, status);

		iproc_ep_setup_init(ep, status);
	} else if (length == 0) {
		/* No DATA stage. Just need to prepare for the next SETUP. */
		iproc_ep_setup_init(ep, ENOERROR);
	} else {
		/*
		 * The SETUP stage processing has completed OK, and there may or may not be a request queued
		 * for the DATA stage. When the DATA stage completes, preparation for the RX of the next
		 * SETUP will be done.
		 */
		if (ep->dir == USB_DIR_IN) {
			if (iproc_usbd_ep_fifo_empty(udc->usbd_regs, ep->num, USB_DIR_OUT)) {
				iproc_usbd_ep_nak_en(udc->usbd_regs, DATA_EP_OUT_INDEX, USB_DIR_OUT);
				udc->waiting_zlp = 1;
			} else {
				dev_warn(udc->dev, "%s: %s rx fifo not empty\n", __func__, ep->usb_ep.name);
			}
		}
	}
}

static void iproc_udc_req_xfer_done(struct iproc_ep *ep, struct iproc_ep_req *req, int status)
{
	struct iproc_udc *udc = ep->udc;
	u32 stopped;

	list_del_init(&req->list_node);

	if (req->usb_req.status == -EINPROGRESS)
		req->usb_req.status = status;

	if (req->dma_aligned) {
		req->dma_aligned = 0;
	} else if (req->dma_mapped) {
		/*
		 * A physical address was not provided for the DMA buffer. Release any resources
		 * that were requested by the driver.
		 */
		dma_unmap_single(udc->gadget.dev.parent, req->usb_req.dma, req->usb_req.length,
		                    (ep->dir == USB_DIR_IN ? DMA_TO_DEVICE : DMA_FROM_DEVICE));

		req->dma_mapped = 0;
		req->usb_req.dma = DMA_ADDR_INVALID;
	}

	/*
	 * Disable DMA operations during completion callback. The callback may cause requests to be
	 * added to the queue, but we don't want to change the state of the queue head.
	 */
	stopped = ep->stopped;
	ep->stopped = 1;
	spin_unlock(&udc->lock);
	req->usb_req.complete(&ep->usb_ep, &req->usb_req);
	spin_lock(&udc->lock);
	ep->stopped = stopped;
}

static void iproc_udc_req_xfer_process(struct iproc_ep *ep)
{
	struct iproc_udc *udc = ep->udc;
	struct iproc_ep_req *req;

	/* @todo Current transfer is always the queue head.
	 * Do we need a separate pointer? Maybe just a pointer to usb_request
	 */
	if (!ep->dma.usb_req) {
		dev_err(udc->dev, "%s: No request being transferred\n", ep->usb_ep.name);
		return;
	}

	iproc_usbd_ep_dma_dis(udc->usbd_regs, ep->num, ep->dir);
	iproc_dma_data_rm_done(ep);

	if (ep->dma.usb_req->status != -EINPROGRESS) {
		/*
		 * Current transfer stage has finished. This may or may not be with error.
		 * Complete the transfer as needed before starting the next one, if any.
		 */
		iproc_dma_data_finish(ep);

		if ((ep->type == USB_ENDPOINT_XFER_CONTROL) && (ep->dir == USB_DIR_IN)
			&& (ep->dma.usb_req->status == ENOERROR)) {
			/*
			 * For the status phase of control IN transfers, the hardware requires that an OUT DMA transfer
			 * actually takes place. This should be just an OUT ZLP, and we will re-use the IN buffer that
			 * just completed transfer for this purpose. There should be no harm in doing this, even if the
			 * OUT status is more than a ZLP.
			 */
			ep->dir = USB_DIR_OUT;
			iproc_dma_data_init(ep);
		} else {
			/*
			 * All transfer stages have completed. Return the request to the gadget driver, and then
			 * setup for the next transfer.
			 */
			iproc_udc_req_xfer_done(ep, list_first_entry(&ep->list_queue, struct iproc_ep_req, list_node), ENOERROR);

			if (ep->type == USB_ENDPOINT_XFER_CONTROL) {
				iproc_ep_setup_init(ep, ENOERROR);
				if (udc->waiting_zlp) {
					udc->waiting_zlp = 0;
					if (udc->ep[DATA_EP_OUT_INDEX].dma.usb_req)
						iproc_usbd_ep_nak_clear(udc->usbd_regs, DATA_EP_OUT_INDEX, USB_DIR_OUT);
				}
			}

			if (list_empty(&ep->list_queue)) {
				/** @todo Probably should more closely bind this to iproc_dma_data_finish. */
				ep->dma.usb_req = NULL;
			} else {
				req = list_first_entry(&ep->list_queue, struct iproc_ep_req, list_node);
				ep->dma.usb_req = &req->usb_req;
				iproc_dma_data_init(ep);
			}
		}
	}

	if (ep->dma.usb_req != NULL) {
		iproc_dma_data_add_ready(ep);
		iproc_usbd_ep_dma_en(udc->usbd_regs, ep->num, ep->dir);
		iproc_usbd_ep_nak_clear(udc->usbd_regs, ep->num, ep->dir);
	}
}

static void iproc_udc_req_xfer_error(struct iproc_ep *ep, int status)
{
	struct iproc_udc *udc = ep->udc;

	if (!ep->dma.usb_req) {
		dev_err(udc->dev, "%s: No request being transferred\n", ep->usb_ep.name);
		return;
	}

	/** @todo abort current DMA, start next transfer if there is one. */
	ep->dma.usb_req->status = status;
	iproc_udc_req_xfer_process(ep);
}

static void iproc_udc_ops_disconnect(struct iproc_udc *udc)
{
	struct iproc_ep *ep;
	int idx;

	for (idx = 0; idx < IPROC_UDC_EP_CNT; idx++) {
		ep = &udc->ep[idx];

		if (ep->dma.usb_req) {
			/* Flush DMA, request still pending */
			iproc_usbd_ep_fifo_flush_en(udc->usbd_regs, 0, ep->dir);
			iproc_usbd_ep_fifo_flush_dis(udc->usbd_regs, 0, ep->dir);

			iproc_udc_req_xfer_process(ep);
		}
	}
}

static void iproc_udc_ops_shutdown(struct iproc_udc *udc)
{
	struct iproc_ep *ep;

	udc->ep[0].desc = NULL;
	list_for_each_entry(ep, &udc->gadget.ep_list, usb_ep.ep_list)
		ep->desc = NULL;

	udc->gadget.dev.driver = NULL;
	udc->gadget_driver = NULL;
}

/****************************************************************************
 * IRQ routines.
 *
 * xgs_iproc_udc_isr - top level entry point.
 * iproc_cfg_isr - device (endpoint 0) set config interrupt handler
 * iproc_inf_isr - device (endpoint 0) set interface interrupt handler
 * iproc_speed_isr - device speed enumeration done interrupt handler
 * iproc_ep_in_isr - top level IN endpoint related interrupt handler
 * iproc_ep_out_isr -  top level OUT endpoint related interrupt handler
 * iproc_ep_out_setup_isr - Control endpoint SETUP Rx handler. This may get
 *          called directly as the result of an endpoint OUT interrupt, or
 *          indirectly as the result of device SET_CFG or SET_INTF.
 ***************************************************************************/
static void iproc_cfg_isr(struct iproc_udc *udc)
{
	struct usb_ctrlrequest setup;
	int idx;
	u16 cfg;

	/*
	 * Device Configuration SETUP has been received. This is not placed in the SETUP
	 * DMA buffer. The packet has to be re-created here so it can be forwarded to the
	 * gadget driver to act upon.
	 */

	cfg = (u16) iproc_usbd_cfg_num(udc->usbd_regs);

	setup.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_DEVICE;
	setup.bRequest = USB_REQ_SET_CONFIGURATION;
	setup.wValue = cpu_to_le16(cfg);
	setup.wIndex = 0;
	setup.wLength = 0;

	/*
	 * Setting the configuration number before the gadget responds is a bit presumptious, but should
	 * not be fatal.
	 */
	/** @todo Do not set endpoint 0? Or is it a don't care? */
	for (idx = 0; idx < IPROC_UDC_EP_CNT; idx++)
		iproc_usbd_ep_cfg_set(udc->usbd_regs, idx, cfg);

	printk(KERN_INFO "SET CFG=%d\n", cfg);

	iproc_ep_setup_process(&udc->ep[0], &setup);
	iproc_usbd_setup_done(udc->usbd_regs);
}

static void iproc_inf_isr(struct iproc_udc *udc)
{
	struct usb_ctrlrequest setup;
	u32 idx;
	u16 intf;
	u16 alt;

	/*
	 * Device Interface SETUP has been received. This is not placed in the SETUP
	 * DMA buffer. The packet has to be re-created here so it can be forwarded to the
	 * gadget driver to act upon.
	 */
	intf = (u16)iproc_usbd_intf_num(udc->usbd_regs);
	alt =  (u16)iproc_usbd_alt_num(udc->usbd_regs);

	setup.bRequestType = USB_DIR_OUT | USB_TYPE_STANDARD | USB_RECIP_INTERFACE;
	setup.bRequest = USB_REQ_SET_INTERFACE;
	setup.wValue = cpu_to_le16(alt);
	setup.wIndex = cpu_to_le16(intf);
	setup.wLength = 0;

	/*
	 * Setting the interface numbers before the gadget responds is a bit
	 * presumptious, but should not be fatal.
	 */
	for (idx = 0; idx < IPROC_UDC_EP_CNT; idx++) {
		iproc_usbd_ep_alt_set(udc->usbd_regs, idx, alt);
		iproc_usbd_ep_intf_set(udc->usbd_regs, idx, intf);
	}

	iproc_ep_setup_process(&udc->ep[0], &setup);
	iproc_usbd_setup_done(udc->usbd_regs);
}

static void iproc_speed_isr(struct iproc_udc *udc)
{
	u32 speed;

	speed = udc->gadget.speed;

	switch(iproc_usbd_speed_get(udc->usbd_regs)) {
	case USB_SPEED_HIGH:
		printk(KERN_INFO "HIGH SPEED\n");
		udc->gadget.speed = USB_SPEED_HIGH;
		break;
	case USB_SPEED_FULL:
		printk(KERN_INFO "FULL SPEED\n");
		udc->gadget.speed = USB_SPEED_FULL;
		break;
	case USB_SPEED_LOW:
		dev_warn(udc->dev, "low speed not supported\n");
		udc->gadget.speed = USB_SPEED_LOW;
		break;
	default:
		dev_err(udc->dev, "unknown speed=0x%x\n", iproc_usbd_speed_get(udc->usbd_regs));
		break;
	}

	if ((speed == USB_SPEED_UNKNOWN) && (udc->gadget.speed != USB_SPEED_UNKNOWN)) {
		/*
		 * Speed has not been enumerated before, so now we can initialize transfers on endpoint 0.
		 * Also have to disable the NAKs at a global level, which has been in place while waiting
		 * for enumeration to complete.
		 */
		iproc_ep_setup_init(&udc->ep[0], ENOERROR);
		iproc_usbd_nak_response_dis(udc->usbd_regs);
	}
}

static void iproc_ep_in_isr(struct iproc_ep *ep)
{
	struct iproc_udc *udc = ep->udc;
	u32 status;

	status = iproc_usbd_ep_stat_active(udc->usbd_regs, ep->num, USB_DIR_IN);
	iproc_usbd_ep_stat_clear(udc->usbd_regs, ep->num, USB_DIR_IN, status);

	if (!status)
		return;

	/** @todo check might only be for direction... */
	if ((ep->dir != USB_DIR_IN) && (ep->type != USB_ENDPOINT_XFER_CONTROL)) {
		dev_err(udc->dev, "%s: unexpected IN interrupt\n", ep->usb_ep.name);
		return;
	}

	if (ep->dir != USB_DIR_IN)
		/* This probably should not be happening */
		dev_warn(udc->dev, "%s: CTRL dir OUT\n", ep->usb_ep.name);

	if ((ep->type == USB_ENDPOINT_XFER_ISOC) &&
	    (status & (USBD_EP_STAT_IN_XFER_DONE | USBD_EP_STAT_DMA_BUF_UNAVAIL)))
		dev_warn(udc->dev, "%s: ISOC IN unexpected status=0x%x\n", ep->usb_ep.name, status);

	if (status & USBD_EP_STAT_IN_TOKEN_RX) {
		/*
		 * If there's any IN requests, the DMA should be setup and ready to go if
		 * the endpoint is not an ISOC. Nothing to do in this case. However, if
		 * this is an ISOC endpoint, then this interrupt implies there was no
		 * data available for this frame number. This will happen if the gadget
		 * does not have any data queued to send in this frame, or we have been
		 * waiting for this event to occur so we can get alignment with the host
		 * for the interval. This alignment is necessary when the interval is
		 * greater than one frame / uframe. E.g. for an audio stream sending
		 * samples @ 5ms intervals on a FS link, this corresponds to a period
		 * of 5 frames. Samples with be queued for every 5th frame number after
		 * the frame number in which this interrupt occurred.
		 */
		status &= ~USBD_EP_STAT_IN_TOKEN_RX;

		if ((ep->type == USB_ENDPOINT_XFER_ISOC)) {
			/* Always align to the current frame number for subsequent transfers. */
			ep->dma.frame_num = iproc_usbd_last_rx_frame_num(udc->usbd_regs);
			if (ep->dma.usb_req != NULL) {
				/*
				 * Might have something queued when waiting for alignment. If something is queued,
				 * it is already too late for the current transfer point. It will also have been
				 * placed in the queue at some point before this interrupt, and it will be stale
				 * if we try to transmit at the next transfer point.
				 */
				ep->dma.usb_req->status = -EREMOTEIO;
				iproc_udc_req_xfer_process(ep);
			}
		}
	}

	if (status & USBD_EP_STAT_IN_DMA_DONE) {
		/*
		 * DMA has completed, but cannot start next transfer until USBD_EP_STAT_IN_XFER_DONE.
		 * To avoid race conditions and other issues, do not release the current transfer until both
		 * interrupts have arrived. Normally this interrupt will arrive at or before the IN_XFER_DONE,
		 * but there have been situations when the system is under load that this interrupt might
		 * arrive after the IN_XFER_DONE, in which case we will need to do the processing now.
		 * The exception to this rule is for ISOC endpoints. They will only get this interrupt to
		 * indicate that DMA has completed.
		 */
		status &= ~USBD_EP_STAT_IN_DMA_DONE;

		if ((ep->type == USB_ENDPOINT_XFER_ISOC)) {
			iproc_udc_req_xfer_process(ep);
		} else if (ep->dma.done & USBD_EP_STAT_IN_XFER_DONE) {
			/*
			 * Did not receive the IN_DMA_DONE interrupt for this request before or
			 * at the same time as the IN_XFER_DONE interrupt, so the request
			 * processing was postponed until the IN_DMA_DONE interrupt arrived.
			 * See handling of IN_XFER_DONE status below.
			 */
			iproc_udc_req_xfer_process(ep);
		} else {
			/*
			 * IN_DMA_DONE received. Save this info so request processing will be
			 * done when the IN_XFER_DONE interrupt is received. This may happen
			 * immediately, idx.e. both IN_DMA_DONE and IN_XFER_DONE status are
			 * set when the interrupt processing takes place.
			 */
			ep->dma.done = USBD_EP_STAT_IN_DMA_DONE;
		}
	}

	if (status & USBD_EP_STAT_IN_XFER_DONE) {
		status &= ~(USBD_EP_STAT_IN_XFER_DONE);
		status &= ~(USBD_EP_STAT_IN_FIFO_EMPTY);

		if (ep->dma.done & USBD_EP_STAT_IN_DMA_DONE) {
			/*
			 * Have received both the IN_DMA_DONE and IN_XFER_DONE interrupts
			 * for this request. OK to process the request (remove the request
			 * and start the next one).
			 */
			iproc_udc_req_xfer_process(ep);
		} else {
			/*
			 * Have not received the IN_DMA_DONE interrupt for this request.
			 * Need to postpone processing of the request until the IN_DMA_DONE
			 * interrupt occurs. See handling of IN_DMA_DONE status above.
			 */
			ep->dma.done = USBD_EP_STAT_IN_XFER_DONE;
		}
	}

	/* Clear the FIFO EMPTY bit, not to print error message */
	status &= ~(USBD_EP_STAT_IN_FIFO_EMPTY);

	if (status & USBD_EP_STAT_DMA_BUF_UNAVAIL) {
		dev_err(udc->dev, "%s: DMA BUF NOT AVAIL\n", ep->usb_ep.name);
		status &= ~(USBD_EP_STAT_DMA_BUF_UNAVAIL);
		iproc_udc_req_xfer_process(ep);
	}

	if (status & USBD_EP_STAT_DMA_ERROR) {
		status &= ~USBD_EP_STAT_DMA_ERROR;
		dev_err(udc->dev, "%s: DMA ERROR\n", ep->usb_ep.name);
		iproc_udc_req_xfer_error(ep, -EIO);
	}

	if (status)
		dev_err(udc->dev, "exit: %s %s: unknown status=0x%x\n",
				__func__, ep->usb_ep.name, status);
}

static void iproc_ep_out_setup_isr(struct iproc_ep *ep)
{
	struct iproc_udc *udc = ep->udc;
	struct iproc_udc_dma_setup *dma;

	dma = &ep->dma.vir_addr->setup;
	if ((IPROC_USBD_READ(dma->status) & REG_DMA_STAT_BUF_MASK) !=
			REG_DMA_STAT_BUF_DMA_DONE) {
		dev_err(udc->dev, "%s: unexpected DMA buf status=0x%x\n", ep->usb_ep.name,
			(IPROC_USBD_READ(dma->status) & REG_DMA_STAT_BUF_MASK));
		iproc_ep_setup_init(ep, ENOERROR);
	} else if ((IPROC_USBD_READ(dma->status) & REG_DMA_STAT_RX_MASK)
			 != REG_DMA_STAT_RX_SUCCESS) {
		dev_err(udc->dev, "%s: unexpected DMA rx status=0x%x\n", ep->usb_ep.name,
			(IPROC_USBD_READ(dma->status) & REG_DMA_STAT_RX_MASK));
		iproc_ep_setup_init(ep, ENOERROR);
	} else {
		if (ep->num != 0) {
			/** @todo Handle the cfg / intf / alt fields of the DMA status. This will only be any issue
			 * once the Linux Gadget driver framework supports control transfers on an endpoint other
			 * than 0.
			 */
			dev_warn(udc->dev, "%s: CTRL xfr support not complete\n", ep->usb_ep.name);
		}
		/*
		 * Take ownership of the descriptor while processing the request. Ownership will be released
		 * when ready to Rx SETUP again.
		 */
		IPROC_USBD_BITS_MODIFY(dma->status, REG_DMA_STAT_BUF_MASK,
					REG_DMA_STAT_BUF_HOST_BUSY);
		if (udc->waiting_zlp)
			dev_err(udc->dev, "%s:%s waiting_zlp\n", ep->usb_ep.name, __func__);
		iproc_ep_setup_process(ep, (struct usb_ctrlrequest *)&dma->data1);
	}
}

static void iproc_ep_out_isr(struct iproc_ep *ep)
{
	struct iproc_udc *udc = ep->udc;
	u32 status;

	status = iproc_usbd_ep_stat_active(udc->usbd_regs, ep->num, USB_DIR_OUT);
	iproc_usbd_ep_stat_clear(udc->usbd_regs, ep->num, USB_DIR_OUT, status);
	iproc_usbd_ep_irq_clear(udc->usbd_regs, ep->num, USB_DIR_OUT);

	/*
	 * Remove the Rx packet size field from the status. The datasheet states this field is not used
	 * in DMA mode, but that is not true.
	 */
	status &= USBD_EP_STAT_ALL;

	if (!status)
		return;

	if ((ep->dir != USB_DIR_OUT) && (ep->type != USB_ENDPOINT_XFER_CONTROL)) {
		dev_err(udc->dev, "%s: unexpected OUT interrupt\n", ep->usb_ep.name);
		return;
	}

	if (ep->dir != USB_DIR_OUT)
		/* This probably should not be happening */
		dev_err(udc->dev, "%s: CTRL dir IN\n", ep->usb_ep.name);

	if (status & USBD_EP_STAT_OUT_DMA_DATA_DONE) {
		status &= ~USBD_EP_STAT_OUT_DMA_DATA_DONE;
		iproc_udc_req_xfer_process(ep);
	}

	if (status & USBD_EP_STAT_OUT_DMA_SETUP_DONE) {
		status &= ~USBD_EP_STAT_OUT_DMA_SETUP_DONE;
		iproc_ep_out_setup_isr(ep);
	}

	if (status & USBD_EP_STAT_DMA_BUF_UNAVAIL) {
		/** @todo Verify under what situations this can happen. Should be when chain has emptied but last desc not reached  */
		/** @todo status for desc updates */

		status &= ~USBD_EP_STAT_DMA_BUF_UNAVAIL;
		dev_err(udc->dev, "%s: DMA BUF NOT AVAIL\n", ep->usb_ep.name);
		iproc_udc_req_xfer_process(ep);
	}

	if (status & USBD_EP_STAT_DMA_ERROR) {
		status &= ~USBD_EP_STAT_DMA_ERROR;
		dev_err(udc->dev, "%s: DMA ERROR\n", ep->usb_ep.name);
		/** @todo merge XferError and XferProcess?? */
		iproc_udc_req_xfer_error(ep, -EIO);
	}

	if (status)
		dev_err(udc->dev, "%s: unknown status=0x%x\n", ep->usb_ep.name, status);
}

irqreturn_t xgs_iproc_udc_isr(int irq, void *context)
{
	struct iproc_udc *udc = NULL;
	unsigned long flags;
	u32 stat, epin_stat, epout_stat;
	int idx;

	udc = (struct iproc_udc *)context;

	spin_lock_irqsave(&udc->lock, flags);

	if (!udc || !udc->gadget_driver) {
		dev_err(udc->dev, "Invalid context or no driver registered: irq dev=0x%x\n",
				iproc_usbd_irq_active(udc->usbd_regs));

		iproc_usbd_irq_clear(udc->usbd_regs, USBD_IRQ_ALL);
		iproc_usbd_ep_irq_list_clear(udc->usbd_regs, USB_DIR_IN, ~0);
		iproc_usbd_ep_irq_list_clear(udc->usbd_regs, USB_DIR_OUT, ~0);

		spin_unlock_irqrestore(&udc->lock, flags);
		return IRQ_HANDLED;
	}

	stat = iproc_usbd_irq_active(udc->usbd_regs);
	epin_stat = iproc_usbd_ep_irq_list_active(udc->usbd_regs, USB_DIR_IN);
	epout_stat = iproc_usbd_ep_irq_list_active(udc->usbd_regs, USB_DIR_OUT);

	if (!(stat || epin_stat || epout_stat))
		return IRQ_NONE;

	iproc_usbd_irq_clear(udc->usbd_regs, stat);
	iproc_usbd_ep_irq_list_clear(udc->usbd_regs, USB_DIR_IN, epin_stat);
	iproc_usbd_ep_irq_list_clear(udc->usbd_regs, USB_DIR_OUT, epout_stat);

	/*
	 * Handle the SET_CFG and SET_INTF interrupts after the endpoint and other device interrupts.
	 * There can be some race conditions where we have an endpoint 0 interrupt pending for the
	 * completion of a previous endpoint 0 transfer (e.g. a GET config) when a SETUP arrives
	 * corresponding to the SET_CFG and SET_INTF. Need to complete the processing of the previous
	 * transfer before handling the next one, idx.e. the SET_CFG or SET_INTF.
	 */
	if (stat & USBD_IRQ_BUS_RESET) {
		dev_info(udc->dev, "BUS reset\n");
		udc->waiting_zlp = 0;
	}

	if (stat & USBD_IRQ_BUS_SUSPEND)
		dev_dbg(udc->dev, "BUS suspend\n");

	if (stat & USBD_IRQ_BUS_IDLE) {
		dev_dbg(udc->dev, "BUS idle\n");
		if (udc->vbus_active) {
			if (udc->gadget_driver->disconnect) {
				spin_unlock(&udc->lock);
				udc->gadget_driver->disconnect(&udc->gadget);
				spin_lock(&udc->lock);
			}
			iproc_udc_ops_disconnect(udc);
			udc->vbus_active = 0;
		}
	}

	if (stat & USBD_IRQ_SPEED_ENUM_DONE) {
		dev_dbg(udc->dev, "BUS speed enum done\n");
		iproc_speed_isr(udc);
	}

	/* endpoint interrupts handler */
	for (idx = 0; idx < IPROC_UDC_EP_CNT; idx++) {
		if (epin_stat & (1 << idx))
			iproc_ep_in_isr(&udc->ep[idx]);
		if (epout_stat & (1 << idx))
			iproc_ep_out_isr(&udc->ep[idx]);
	}

	/* SET_CFG and SET_INTF interrupts handler */
	if (stat & USBD_IRQ_SET_CFG)
		iproc_cfg_isr(udc);
	if (stat & USBD_IRQ_SET_INTF) {
		iproc_inf_isr(udc);
		if (iproc_usbd_cfg_num(udc->usbd_regs)) {
			udc->vbus_active = 1;
		}
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return IRQ_HANDLED;
}

/***************************************************************************
* Endpoint request operations
***************************************************************************/
static void iproc_udc_req_queue_flush(struct iproc_ep *ep, int status)
{
	struct iproc_udc *udc = ep->udc;
	struct iproc_ep_req *req;

	ep->stopped = 1;
	iproc_usbd_ep_ops_finish(udc->usbd_regs, ep->num);

	while (!list_empty(&ep->list_queue)) {
		req = list_first_entry(&ep->list_queue, struct iproc_ep_req, list_node);
		iproc_udc_req_xfer_done(ep, req, status);
	}

	ep->dma.usb_req = NULL;
}


static void iproc_udc_req_xfer_add(struct iproc_ep *ep, struct iproc_ep_req *req)
{
	struct iproc_udc *udc = ep->udc;
	list_add_tail(&req->list_node, &ep->list_queue);

	/** @todo Is this necessary?? Stopped happens as a result of a halt, complete(), dequeue(), nuke().
	 * nuke() is called when ep disabled, during setup processing, and by udc_queisce(). The latter is
	 * called during vbus state change (cable insert/remove), USB reset interrupt, and gadget deregister.
	 */
	if (ep->stopped)
		return;

	if ((ep->dir == USB_DIR_IN) && (ep->type == USB_ENDPOINT_XFER_ISOC) &&
		ep->dma.usb_req && (ep->dma.frame_num == FRAME_NUM_INVALID)) {
		/*
		 * Gadget has a request already queued, but still have not received an IN token from the host
		 * and the interval window is not aligned. Queued packet is now very stale, so remove it.
		 */

		iproc_dma_data_finish(ep);
		/** @todo Move set of ep->dma.usb_req to iproc_dma_data_init() and iproc_dma_data_finish() routines. */
		ep->dma.usb_req = NULL;
		iproc_udc_req_xfer_done(ep, list_first_entry(&ep->list_queue, struct iproc_ep_req, list_node), -EREMOTEIO);
	}

	/** @todo Current transfer is always the queue head. Do we need a separate pointer? Maybe just a pointer to usb_request
	 * need to know if the queue head has already been loaded. Maybe that's the point of the "stopped".
	 */
	if (!ep->dma.usb_req) {
		if ((ep->dir == USB_DIR_IN) && (ep->type == USB_ENDPOINT_XFER_ISOC) &&
			(ep->dma.frame_num == FRAME_NUM_INVALID)) {
			/*
			 * Delay any ISOC IN DMA operations until it is known what frame number the host
			 * is going to start transfers with. Normally might just return requests until
			 * this event occurs. However, the zero gadget does not submit requests based on
			 * its own timer or similar, so if the request is returned right away things are
			 * going to thrash, as another request will be immediately submitted.
			 */
			ep->dma.usb_req = &(list_first_entry(&ep->list_queue,
					struct iproc_ep_req, list_node))->usb_req;
			iproc_dma_data_init(ep);
			iproc_usbd_ep_nak_clear(udc->usbd_regs, ep->num, ep->dir);
		} else {
			req = list_first_entry(&ep->list_queue,
						struct iproc_ep_req, list_node);
			ep->dma.usb_req = &req->usb_req;
			iproc_dma_data_init(ep);
			iproc_dma_data_add_ready(ep);
			iproc_usbd_ep_dma_en(udc->usbd_regs, ep->num, ep->dir);
			wmb();
			if ((ep->num == DATA_EP_OUT_INDEX) && udc->waiting_zlp) {
				dev_warn(udc->dev, "%s: %s waiting_zlp\n", __func__, ep->usb_ep.name);
			} else {
				iproc_usbd_ep_nak_clear(udc->usbd_regs, ep->num, ep->dir);
			}
		}
	}
}

/*
 * UDC Operations routines.
 * iproc_udc_ops_finish - Finish / terminate all UDC operations
 * iproc_udc_ops_start - Start UDC operations. Happens after a Gadget driver attaches.
 * iproc_udc_ops_stop - Stop UDC operations. Happens after a Gadget driver detaches.
 */
static void iproc_udc_ops_finish(struct iproc_udc *udc)
{
	/* do nothing */
	return;
}

static void iproc_udc_ops_start(struct iproc_udc *udc)
{
	int idx;

	/*
	 * Just enable interrupts for now. Endpoint 0 will get enabled once the speed enumeration
	 * has completed. The Device DMA enable is global in scope. There's endpoint specific
	 * DMA enables that will happen later.
	 */
	iproc_usbd_irq_en(udc->usbd_regs, (USBD_IRQ_SPEED_ENUM_DONE |
	                                   USBD_IRQ_BUS_SUSPEND |
	                                   USBD_IRQ_BUS_IDLE |
	                                   USBD_IRQ_BUS_RESET |
	                                   USBD_IRQ_SET_INTF |
	                                   USBD_IRQ_SET_CFG));
	iproc_usbd_dma_en(udc->usbd_regs);

	/* Enable interrupts for all configured endpoints */
	for (idx = 0; idx < IPROC_UDC_EP_CNT; ++idx) {
		if (udc->ep[idx].usb_ep.name) {
			if (idx !=0 && (udc->ep[idx].dir == USB_DIR_OUT)) {
				iproc_usbd_ep_nak_en(udc->usbd_regs, idx, USB_DIR_OUT);
			}
			iproc_usbd_ep_irq_en(udc->usbd_regs, udc->ep[idx].num, USB_DIR_OUT);
			if (idx == 0 )
				iproc_usbd_ep_irq_en(udc->usbd_regs, udc->ep[idx].num, USB_DIR_IN);
		}
	}
	iproc_usbd_nak_response_dis(udc->usbd_regs);
}

static void iproc_udc_ops_stop(struct iproc_udc *udc)
{
	struct iproc_ep *ep;

	iproc_usbd_dma_dis(udc->usbd_regs);
	iproc_usbd_irq_dis(udc->usbd_regs, USBD_IRQ_ALL);
	iproc_usbd_irq_clear(udc->usbd_regs, USBD_IRQ_ALL);

	udc->gadget.speed = USB_SPEED_UNKNOWN;

	iproc_udc_req_queue_flush(&udc->ep[0], -ESHUTDOWN);

	list_for_each_entry(ep, &udc->gadget.ep_list, usb_ep.ep_list)
	iproc_udc_req_queue_flush(ep, -ESHUTDOWN);
}

/*
 * APIs used by a Gadget driver to attach / detach from the UDC driver.
 */
static int xgs_iproc_udc_start(struct usb_gadget *gadget,
                               struct usb_gadget_driver *gadget_driver)
{
	struct iproc_udc *udc = gadget_to_udc(gadget);
	unsigned long flags;

	if (!udc) {
		dev_err(udc->dev, "UDC driver not initialized\n");
		return -ENODEV;
	}

	if (!gadget_driver || !gadget_driver->setup ||
		gadget_driver->max_speed < USB_SPEED_FULL) {
		dev_err(udc->dev, "invalid gadget driver\n" );
		return -EINVAL;
	}

	spin_lock_irqsave(&udc->lock, flags);

	if (udc->gadget_driver) {
		spin_unlock_irqrestore(&udc->lock, flags);
		dev_err(udc->dev, "UDC driver busy\n");
		return -EBUSY;
	}

	/* Hook up the gadget driver to the UDC controller driver */
	gadget_driver->driver.bus = NULL;
	udc->gadget_driver = gadget_driver;
	udc->gadget.dev.driver = &gadget_driver->driver;
	udc->pullup_on = 1;

	iproc_udc_ops_start(udc);
	/* un-stop the control endpoint */
	udc->ep[0].stopped = 0;
	iproc_usbd_bus_conn(udc->usbd_regs);

	iproc_usbd_setup_done(udc->usbd_regs);
	iproc_usbd_dma_en(udc->usbd_regs);

	spin_unlock_irqrestore(&udc->lock, flags);

	return ENOERROR;
}

static int xgs_iproc_udc_stop(struct usb_gadget *gadget)
{
	unsigned long flags;
	struct iproc_udc *udc = gadget_to_udc(gadget);

	if (!udc) {
		dev_err(udc->dev, "UDC driver not initialized\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&udc->lock, flags);

	udc->ep[0].stopped = 1;
	iproc_udc_ops_stop(udc);
	udelay(20);
	udc->pullup_on = 0;
	iproc_usbd_bus_disconn(udc->usbd_regs);
	iproc_udc_ops_shutdown(udc);
	spin_unlock_irqrestore(&udc->lock, flags);

	return ENOERROR;
}

/*
 * Linux Gadget endpoint operations. See usb_ep_ops in usb_gadget.h.
 */
static int xgs_iproc_ep_enable(struct usb_ep *usb_ep, const struct usb_endpoint_descriptor *desc)
{
	struct iproc_ep *ep = our_ep(usb_ep);
	struct iproc_udc *udc = ep->udc;
	unsigned long flags;
	u32 xferType;
	int ret = ENOERROR;

	if (!usb_ep || (ep->beq_addr != desc->bEndpointAddress)) {
		dev_err(udc->dev, "invalid endpoint (%p)\n", usb_ep);
		return -EINVAL;
	}

	if (!desc || (desc->bDescriptorType != USB_DT_ENDPOINT)) {
		dev_err(udc->dev, "ep%d: invalid descriptor=%p type=%d\n", ep->num, desc, desc ? desc->bDescriptorType : -1);
		return -EINVAL;
	}

	if (desc == ep->desc) {
		dev_warn(udc->dev, "ep%d: already enabled with same descriptor\n", ep->num);
		return -EEXIST;
	}

	if (ep->desc) {
		dev_warn(udc->dev, "ep%d: already enabled with another descriptor\n", ep->num);
		return -EBUSY;
	}

	if (!udc->gadget_driver || (udc->gadget.speed == USB_SPEED_UNKNOWN)) {
		dev_warn(udc->dev, "%s: invalid device state\n", ep->usb_ep.name);
		return -ESHUTDOWN;
	}

	xferType = desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	if ((ep->dir == USB_DIR_IN) && (xferType == USB_ENDPOINT_XFER_ISOC)) {
		if ((desc->bInterval < 1) || (desc->bInterval > 16)) {
			dev_err(udc->dev, "%s: invalid ISOC bInterval=%u\n", ep->usb_ep.name, desc->bInterval);
			return -ERANGE;
		}

		/*
		 * We don't know when the host will send the first ISO IN request, so we need to set up
		 * to capture that event so we can align subsequent transfers to that particular frame
		 * number. Also set the frame number increment. The endpoint descriptor specifies this
		 * as a power of 2 (2**(n-1)). Translate this into a specific number of frames.
		 */
		ep->dma.frame_num = FRAME_NUM_INVALID;
		ep->dma.frame_incr = 1 << (desc->bInterval - 1);
	}

	spin_lock_irqsave(&udc->lock, flags);

	ep->desc = desc;
	ep->stopped = 0;

	/** @todo Rework the UdcEpCfg() so it includes iproc_usbd_ep_cfg_set() ... */
	iproc_usbd_ep_cfg_set(udc->usbd_regs, ep->num, iproc_usbd_cfg_num(udc->usbd_regs));

	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

static int xgs_iproc_ep_disable(struct usb_ep *usb_ep)
{
	struct iproc_ep *ep = our_ep(usb_ep);
	struct iproc_udc *udc = ep->udc;
	unsigned long flags;
	int ret = ENOERROR;

	if (!usb_ep) {
		dev_err(udc->dev, "invalid endpoint\n");
		return -EINVAL;
	}

	if (!ep->desc) {
		dev_warn(udc->dev, "%s: already disabled\n", ep->usb_ep.name);
		return ENOERROR;
	}

	spin_lock_irqsave(&udc->lock, flags);

	iproc_udc_req_queue_flush(ep, -ESHUTDOWN);
	iproc_usbd_ep_irq_dis(udc->usbd_regs, ep->num, ep->dir);
	ep->desc = NULL;

	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

struct usb_request * xgs_iproc_ep_alloc_request(struct usb_ep *usb_ep, gfp_t gfp_flags)
{
	struct iproc_ep_req *req;

	if (!usb_ep)
		return NULL;

	if ((req = kzalloc(sizeof(*req), gfp_flags)) != NULL) {
		/*
		 * Set the usb_req.dma to DMA_ADDR_INVALID so it can be determined if the usb_req.buf needs
		 * to be mapped when the request is subsequently queued.
		 */
		INIT_LIST_HEAD(&req->list_node);
		req->usb_req.dma = DMA_ADDR_INVALID;

		return &req->usb_req;
	}

	return NULL;
}

static void xgs_iproc_ep_free_request(struct usb_ep *usb_ep, struct usb_request *usb_req)
{
	struct iproc_ep_req *req = our_req(usb_req);

	if (usb_req)
		kfree(req);
}

static int xgs_iproc_ep_queue(struct usb_ep *usb_ep, struct usb_request *usb_req, gfp_t gfp_flags)
{
	struct iproc_ep *ep = our_ep(usb_ep);
	struct iproc_udc *udc = ep->udc;
	struct iproc_ep_req *req = our_req(usb_req);
	unsigned long flags;
	int ret = ENOERROR;

	if (!usb_ep || !usb_req || !req->usb_req.complete || !req->usb_req.buf || !list_empty(&req->list_node)) {
		dev_err(udc->dev, "invalid request\n");
		return -EINVAL;
	}

	if (!ep->desc && (ep->num != 0)) {
		dev_err(udc->dev, "%s: invalid EP state\n", ep->usb_ep.name);
		return -EFAULT;
	}

	if ((ep->type == USB_ENDPOINT_XFER_CONTROL) && !list_empty(&ep->list_queue)) {
		dev_err(udc->dev, "%s: CTRL EP queue not empty\n", ep->usb_ep.name);
		return -EPERM;
	}

	if (usb_req->length > 16384 /* FSG_BUFLEN */) {
		dev_err(udc->dev, "%s: request too big, length=%u\n", ep->usb_ep.name, usb_req->length);
		return -E2BIG;
	}

	/*
	 * Restrict ISOC IN requests to the max packet size. Assumption is that it does not make
	 * much sense to have more than one interval's (scheduled bandwidth's) worth of data.
	 */
	if ((ep->type == USB_ENDPOINT_XFER_ISOC) && (ep->dir == USB_DIR_IN) && (usb_req->length > ep->usb_ep.maxpacket)) {
		dev_err(udc->dev, "%s: request > scheduled bandwidth, length=%u\n", ep->usb_ep.name, usb_req->length);
		return -EFBIG;
	}

	if (!udc->gadget_driver || (udc->gadget.speed == USB_SPEED_UNKNOWN)) {
		dev_warn(udc->dev, "%s: invalid device state\n", ep->usb_ep.name);
		return -ESHUTDOWN;
	}

	if (((u32)req->usb_req.buf) & 0x3UL) {
		/*
		 * The DMA buffer does not have the alignment required by the hardware. We keep an endpoint level
		 * buffer available to handle this situation if it arises. If we don't currently have one available
		 * for this purpose, or if the current one is not large enough, then allocate a new one. Since
		 * we only have one buffer, we won't copy into the buffer until we are ready to do the DMA transfer.
		 * Mark the request as needing this alignment (copy).
		 */
		if ((ep->dma.align_buff != NULL) && (ep->dma.align_len < req->usb_req.length)) {
			dma_free_coherent(NULL, ep->dma.align_len, ep->dma.align_buff, ep->dma.align_addr);
			ep->dma.align_buff = NULL;
		}

		if (ep->dma.align_buff == NULL) {
			ep->dma.align_len = req->usb_req.length;
			ep->dma.align_buff = dma_alloc_coherent(NULL, ep->dma.align_len, &(ep->dma.align_addr), GFP_ATOMIC);
		}

		if (ep->dma.align_buff == NULL) {
			dev_err(udc->dev, "%s: dma_alloc_coherent() failed, length=%u\n", ep->usb_ep.name, usb_req->length);
			return -ENOMEM;
		}
		req->dma_aligned = 1;
	} else if ((req->usb_req.dma == DMA_ADDR_INVALID) || (req->usb_req.dma == 0)) {
		/* A physical address was not provided for the DMA buffer, so request it. */
		req->dma_mapped = 1;
		req->usb_req.dma = dma_map_single(udc->gadget.dev.parent,
						req->usb_req.buf,
						req->usb_req.length,
						ep->dir == USB_DIR_IN ?
						DMA_TO_DEVICE : DMA_FROM_DEVICE);
	}

	spin_lock_irqsave(&udc->lock, flags);

	req->usb_req.status = -EINPROGRESS;
	req->usb_req.actual = 0;

	if ((ep->type == USB_ENDPOINT_XFER_CONTROL) && (ep->dir == USB_DIR_OUT) && (req->usb_req.length == 0)) {
		/*
		 * This might happen if gadget driver decides to send zero length packet (ZLP) during STATUS phase
		 * of a control transfer. This may happen for the cases where there is not a DATA phase. Just consider
		 * things complete. ZLP will be issued by hardware. See the handling of SETUP packets for more details
		 * on control transfer processing.
		 */
		iproc_udc_req_xfer_done(ep, req, ENOERROR);
	} else {
		if (req->usb_req.length == 0)
			req->usb_req.zero = 1;
		iproc_udc_req_xfer_add(ep, req);
	}

	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

static int xgs_iproc_ep_dequeue(struct usb_ep *usb_ep, struct usb_request *usb_req)
{
	struct iproc_ep *ep = our_ep(usb_ep);
	struct iproc_udc *udc = ep->udc;
	struct iproc_ep_req *req = our_req(usb_req);
	unsigned long flags;
	int ret = ENOERROR;

	if (!usb_ep || !usb_req) {
		dev_err(udc->dev, "invalid request\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&udc->lock, flags);

	/* Make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->list_queue, list_node) {
		if (&req->usb_req == usb_req)
			break;
	}

	if (&req->usb_req != usb_req) {
		spin_unlock_irqrestore(&udc->lock, flags);
		dev_err(udc->dev, "%s: request not queued\n", ep->usb_ep.name);
		return -ENOLINK;
	}

	/** @todo Handle case where the request is in progress, or completed but not dequeued */

	iproc_udc_req_xfer_done(ep, req, -ECONNRESET);
	spin_unlock_irqrestore(&udc->lock, flags);

	return ret;
}

static int xgs_iproc_ep_set_halt(struct usb_ep *usb_ep, int enable)
{
	struct iproc_ep *ep = our_ep(usb_ep);
	struct iproc_udc *udc = ep->udc;
	unsigned long flags;
	int ret = ENOERROR;

	if (!usb_ep) {
		dev_err(udc->dev, "invalid request\n");
		return -EINVAL;
	}

	if (ep->type == USB_ENDPOINT_XFER_ISOC) {
		dev_warn(udc->dev, "%s: ISO HALT operations not supported\n", ep->usb_ep.name);
		return -EOPNOTSUPP;
	}

	if (enable && (ep->dir == USB_DIR_IN) && !list_empty(&ep->list_queue)) {
		/* Only allow halt on an IN EP if its queue is empty */
		dev_err(udc->dev, "%s: IN queue not empty\n", ep->usb_ep.name);
		return -EAGAIN;
	}

	if (!enable && (ep->type == USB_ENDPOINT_XFER_CONTROL)) {
		/*
		 * Halt clear for a control EP should only be handled as part of the subsequent SETUP
		 * exchange that occurs after the Halt was set.
		 */
		dev_warn(udc->dev, "%s: CTRL HALT clear\n", ep->usb_ep.name);
		return -EPROTO;
	}

	spin_lock_irqsave(&udc->lock, flags);

	if (!enable) {
		iproc_usbd_ep_stall_dis(udc->usbd_regs, ep->num, ep->dir);
	} else if (ep->type != USB_ENDPOINT_XFER_CONTROL) {
		iproc_usbd_ep_stall_en(udc->usbd_regs, ep->num, ep->dir);
	} else {
		iproc_usbd_ep_stall_en(udc->usbd_regs, ep->num, USB_DIR_IN);
		iproc_usbd_ep_stall_en(udc->usbd_regs, ep->num, USB_DIR_OUT);
	}

	spin_unlock_irqrestore(&udc->lock, flags);
	mdelay(2);

	return ret;
}

static int xgs_iproc_ep_fifo_status(struct usb_ep *usb_ep)
{
	/*
	 * The DWC UDC core doesn't have a mechanism for determining the number of bytes
	 * currently in a FIFO. The best that can be done is determine whether or not a
	 * FIFO is empty. However, for the situation where a single Rx FIFO is being
	 * used for all endpoints, if cannot be determined which OUT and CTRL EP's are
	 * affected if the Rx FIFO is not empty.
	 */
	return -EOPNOTSUPP;
}

static void xgs_iproc_ep_fifo_flush(struct usb_ep *usb_ep)
{
	struct iproc_ep *ep = our_ep(usb_ep);
	struct iproc_udc *udc = ep->udc;
	unsigned long flags;

	if (!usb_ep) {
		dev_err(udc->dev, "invalid request\n");
		return;
	}

	/*
	 * FIFO flush for a control EP does not make any sense. The SETUP protocol
	 * should eliminate the need to flush.
	 */
	if (ep->type == USB_ENDPOINT_XFER_CONTROL) {
		dev_warn(udc->dev, "%s: CTRL FIFO flush\n", ep->usb_ep.name);
		return;
	}

	if (iproc_usbd_ep_fifo_empty(udc->usbd_regs, ep->num, ep->dir)) {
		dev_warn(udc->dev, "%s: FIFO empty\n", ep->usb_ep.name);
		return;
	}

	spin_lock_irqsave(&udc->lock, flags);

	iproc_usbd_ep_fifo_flush_en(udc->usbd_regs, ep->num, ep->dir);

	spin_unlock_irqrestore(&udc->lock, flags);
}


/***************************************************************************
 * Linux proc file system functions
 ***************************************************************************/
#ifdef CONFIG_USB_GADGET_DEBUG_FILES
#include <linux/seq_file.h>

static const char udc_proc_file_name[] = "driver/" XGS_IPROC_UDC_NAME;

static int proc_file_show(struct seq_file *s, void *_)
{
	return 0;
}

static int proc_file_open(struct inode *inode, struct file *file)
{
	return(single_open(file, proc_file_show, NULL));
}

static struct file_operations udc_proc_file_ops =
{
	.open       = proc_file_open,
	.read       = seq_read,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static void xgs_iproc_udc_proc_create(void)
{
	proc_create(udc_proc_file_name, 0, NULL, &udc_proc_file_ops);
}

static void xgs_iproc_udc_proc_remove(void)
{
	remove_proc_entry(udc_proc_file_name, NULL);
}

#else

static void xgs_iproc_udc_proc_create(void) {}
static void xgs_iproc_udc_proc_remove(void) {}

#endif

static struct usb_gadget_ops xgs_iproc_udc_ops = {
	.udc_start          = xgs_iproc_udc_start,
	.udc_stop           = xgs_iproc_udc_stop,
};

static struct usb_ep_ops xgs_iproc_udc_ep_ops = {
	.enable             = xgs_iproc_ep_enable,
	.disable            = xgs_iproc_ep_disable,
	.alloc_request      = xgs_iproc_ep_alloc_request,
	.free_request       = xgs_iproc_ep_free_request,
	.queue              = xgs_iproc_ep_queue,
	.dequeue            = xgs_iproc_ep_dequeue,
	.set_halt           = xgs_iproc_ep_set_halt,
	.fifo_status        = xgs_iproc_ep_fifo_status,
	.fifo_flush         = xgs_iproc_ep_fifo_flush,
};

static const struct of_device_id xgs_iproc_udc_ids[] = {
	{ .compatible = "brcm,usbd-xgs-iproc", },
	{ .compatible = "brcm,usbd-xgs-hx4", },
	{ }
};
MODULE_DEVICE_TABLE(of, xgs_iproc_udc_ids);

static void iproc_udc_ops_init(struct iproc_udc *udc)
{
	int idx;
	struct iproc_ep *ep;

	iproc_usbd_ops_init(udc->usbd_regs);

	/*
	 * See usb/gadget/epautoconf.c for endpoint naming conventions.
	 * Control endpoints are bi-directional, but initial transfer (SETUP stage) is always OUT.
	 */
	/** @todo Really should make the non endpoint 0 init attributes configurable by the chip specific part
	 * of the driver, idx.e. the device instantiation. The settings below are for a chip specific DWG UDC
	 * core configuration. Also should incorporate the DWG UDC endpoint type attribute as part of this,
	 * which can be control, IN, OUT, or bidirectional.
	 */
	INIT_LIST_HEAD(&udc->gadget.ep_list);
	for (idx = 0; idx < IPROC_UDC_EP_CNT; idx++) {
		ep = &udc->ep[idx];

		ep->udc = udc;
		ep->num = idx;

		ep->dir = (xgs_iproc_ep_info[idx].caps.dir_in) ? USB_DIR_IN : USB_DIR_OUT;
		ep->beq_addr = idx | ep->dir;
		ep->stopped = 0;
		ep->type = xgs_iproc_ep_info[idx].type;

		ep->usb_ep.name = xgs_iproc_ep_info[idx].name;
		ep->usb_ep.caps = xgs_iproc_ep_info[idx].caps;
		ep->usb_ep.ops = &xgs_iproc_udc_ep_ops;
		list_add_tail(&ep->usb_ep.ep_list, &udc->gadget.ep_list);
		usb_ep_set_maxpacket_limit(&ep->usb_ep, xgs_iproc_ep_info[idx].msize);
		ep->usb_ep.desc = NULL;
		INIT_LIST_HEAD(&ep->list_queue);

		iproc_usbd_ep_ops_init(udc->usbd_regs, ep->num, ep->type,
					ep->dir, xgs_iproc_ep_info[idx].msize);

		iproc_dma_ep_init(ep);

		if (idx != 0) {
			iproc_usbd_ep_nak_en(udc->usbd_regs, idx, ep->dir);
		}
	}

	udc->gadget.ep0 = &udc->ep[0].usb_ep;
	list_del(&udc->ep[0].usb_ep.ep_list);

	iproc_usbd_self_pwr_en(udc->usbd_regs);
}


/****************************************************************************
 ***************************************************************************/
static int xgs_iproc_udc_probe(struct platform_device *pdev)
{
	int ret = ENOERROR;
	struct device *dev = &pdev->dev;
	struct device_node *dn = dev->of_node;
	struct iproc_udc *udc = NULL;
	struct usb_phy *phy;
	struct resource *res;
	void __iomem *usbd_base;
	int irq;

	phy = devm_usb_get_phy_by_phandle(dev, "usb-phy", 0);
	if (IS_ERR(phy)) {
		dev_err(dev, "unable to find transceiver\n");
		return PTR_ERR(phy);
	}

	/* HX4 SVK always is identified as HOST, because GPIO pin 0 is always low,
	 * even the strap setting of JP1803 is device mode.
	 */
	if (!of_device_is_compatible(dn, "brcm,usbd-xgs-hx4"))
		if (phy->flags != IPROC_USB_MODE_DEVICE)
			return -ENODEV;

	irq = platform_get_irq(pdev, 0);

	udc = devm_kzalloc(dev, sizeof(*udc), GFP_KERNEL);
	if (!udc) {
		dev_err(dev, "devm_kzalloc() failed\n" );
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, udc);
	udc->dev = dev;
	udc->phy = phy;
	spin_lock_init(&udc->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	usbd_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(usbd_base)) {
		dev_err(dev, "can't ioremap USB2D base addr\n");
		return PTR_ERR(usbd_base);
	}

	udc->usbd_regs = (struct iproc_usbd_regs *)usbd_base;

	ret = usb_phy_init(phy);
	if (ret < 0) {
		dev_err(dev, "initial usb transceiver failed.\n");
		return ret;
	}

	ret = iproc_platform_dma_alloc(pdev, udc);
	if (ret < 0) {
		dev_err(dev, "iproc_platform_dma_alloc() failed\n");
		return ret;
	}

	/* gadget init */
	udc->gadget.name = XGS_IPROC_UDC_NAME;
	udc->gadget.speed = USB_SPEED_UNKNOWN;
	udc->gadget.max_speed = USB_SPEED_HIGH;
	udc->gadget.ops = &xgs_iproc_udc_ops;

	iproc_udc_ops_init(udc);

	iproc_usbd_irq_dis(udc->usbd_regs, USBD_IRQ_ALL);
	iproc_usbd_irq_clear(udc->usbd_regs, USBD_IRQ_ALL);

	ret = devm_request_irq(dev, irq, xgs_iproc_udc_isr, 0,
				XGS_IPROC_UDC_NAME, (void *)udc);
	if (ret < 0) {
		dev_err(dev, "error requesting IRQ #%d\n", irq);
		goto err1;
	}

	ret = usb_add_gadget_udc(dev, &udc->gadget);
	if (ret < 0) {
		dev_err(dev, "usb_add_gadget_udc() failed\n");
		goto err1;
	}

	udc->vbus_active = 0;
	xgs_iproc_udc_proc_create();

	return ENOERROR;

err1:
	iproc_platform_dma_free(pdev, udc);

	return ret;
}

static int xgs_iproc_udc_remove(struct platform_device *pdev)
{
	struct iproc_udc *udc = platform_get_drvdata(pdev);

	if (udc) {
		xgs_iproc_udc_proc_remove();

		usb_del_gadget_udc(&udc->gadget);
		iproc_udc_ops_finish(udc);

		platform_set_drvdata(pdev, NULL);
		iproc_platform_dma_free(pdev, udc);
	}

	return ENOERROR;
}

/*
 * Generic platform device driver definition.
 */
static struct platform_driver xgs_iproc_udc_driver =
{
	.probe      = xgs_iproc_udc_probe,
	.remove     = xgs_iproc_udc_remove,
	.driver = {
		.name   = XGS_IPROC_UDC_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = xgs_iproc_udc_ids,
	},
};

module_platform_driver(xgs_iproc_udc_driver);

MODULE_AUTHOR("Broadcom");
MODULE_DESCRIPTION("Broadcom USB Device Controller(UDC) driver");
MODULE_LICENSE("GPL");
