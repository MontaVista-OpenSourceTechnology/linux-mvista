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

#ifndef _XGS_IPROC_UDC_H_
#define _XGS_IPROC_UDC_H_

#include <linux/usb/gadget.h>
#include "xgs_iproc_usbd_regs.h"

#define IPROC_UDC_EP_CNT			7
#define IPROC_UDC_CTRL_MAX_PKG_SIZE		64
#define IPROC_UDC_EP_MAX_PKG_SIZE		512

/*
 * Some unsigned number trickery for indexing into DMA descriptor chain. If the
 * decriptor count is some power of 2, then we can use the mask to extract
 * an index and not worry about wrap around as the unsigned variables are
 * incremented. E.g. in following, IDX(0), IDX(4), IDX(8), ..., IDX(0xffffc)
 * all produce the same result, i.e. 0.
 */
#define IPROC_EP_DMA_DESC_CNT		16
#define IPROC_EP_DMA_DESC_IDX_MASK	(IPROC_EP_DMA_DESC_CNT - 1)
#define IPROC_EP_DMA_DESC_IDX(_idx)	((_idx) & IPROC_EP_DMA_DESC_IDX_MASK)

/* Some DWC UDC DMA descriptor layout definitions. See datasheet for details. */

struct iproc_udc_dma_setup {
	unsigned int status;
	unsigned int reserved;
	unsigned int data1;
	unsigned int data2;
};

struct iproc_udc_dma_desc {
	unsigned int status;
	unsigned int reserved;
	unsigned int buf_addr;
	unsigned int next_addr;
};

/*
 * Common DMA descriptor layout used for all endpoints. Only control endpoints
 * need the setup descriptor, but in order to simply things it is defined for
 * all. It may be possible to omit this altogether, and just use one of data
 * descriptors for setup instead. The control transfer protocol should allow
 * this to be done.
 */
struct iproc_ep_dma {
	struct iproc_udc_dma_setup setup;
	struct iproc_udc_dma_desc  desc[IPROC_EP_DMA_DESC_CNT];
};

/* Structure used for DMA descriptor allocation. Not really necessary but convenient. */
struct iproc_udc_dma {
	struct iproc_ep_dma ep[IPROC_UDC_EP_CNT];
};

/*
 * Structure used to hold endpoint specific information. There's one of these for
 * each endpoint.
 *
 * The Rx/Tx FIFO sizes are used for RAM allocation purposes. Each transfer
 * direction has its own RAM that is used for all the FIFOs in that direction.
 * The RAM gets segmented (allocated) as each endpoint gets enabled. This dynamic
 * allocation FIFO sizes gives flexibility, and does not require that an
 * endpoint's size be fixed at run-time or during compilation. If there's not
 * enough FIFO RAM as required by a gadget's endpoint definitions, then an
 * error will occur for the enabling of any endpoints after the FIFO RAM has
 * become exhausted.
 *
 * The DMA virtual address is used for all descriptor operations. The DMA
 * physical address is for convenience (setting hardware registers, obtaining
 * addresses for descriptor chaining, etc.). The DMA descriptors are not
 * allocated on a per-endpoint basis. These are just pointers into the
 * large block that was allocated for all endpoints.
 */
struct iproc_ep {
	struct usb_ep usb_ep;                           /* usb_gadget.h */
	const struct usb_endpoint_descriptor *desc;     /* usb/ch9.h */
	struct list_head list_queue;                    /* active BCM_UDC_EP_REQ's for the endpoint */
	struct iproc_udc *udc;                          /* endpoint owner (UDC controller) */
	unsigned int num;
	unsigned int dir;                               /* USB_DIR_xxx (direction) */
	unsigned int type;                              /* USB_ENDPOINT_XFER_xxx */
	unsigned int beq_addr;                          /* dirn | type */
	unsigned int stopped : 1;
	struct {
		struct iproc_ep_dma *vir_addr;
		struct iproc_ep_dma *phy_addr;
		struct usb_request *usb_req;                /* Current request being DMA'd */

		/** @todo Some of the below are duplicates of usb_request elements. Use usb_request instead. */
		unsigned int max_buf_len;                   /* Max buffer length to use with a descriptor */
		unsigned int done_len;                      /* Length of request DMA'd so far */
		unsigned int todo_len;                      /* Length of request left to DMA */
		unsigned int add_idx;                       /* descriptor chain index */
		unsigned int rm_idx;                        /* descriptor chain index */
		unsigned int buf_addr;                      /* Location in request to DMA */
		unsigned int frame_num;                     /* Frame number for ISOC transfers */
		unsigned int frame_incr;                    /* Frame number increment (period) */
		unsigned int status;
		unsigned int done;                          /* DMA and USB transfer completion indication (IN_DMA_DONE and IN_XFER_DONE) */
		void *align_buff;                           /* Aligned buffer. Only used if usb_req buffer not aligned properly. */
		dma_addr_t align_addr;                      /* Aligned buffer physical address */
		unsigned int align_len;                      /* Aligned buffer length */
	} dma;
};

/*
 * Structure used to hold controller information. There should be one of these
 * for each controller. Most likely there's only one.
 *
 * The Rx/Tx FIFO space are used for RAM allocation purposes. These track how
 * much RAM is available for use as a FIFO. When an endpoint is enabled, these
 * are check to see if there's enough RAM for a FIFO of the desired length as
 * implied by the max packet size.
 */
struct iproc_udc {
	struct usb_gadget gadget;                       /* usb_gadget.h */
	struct usb_gadget_driver *gadget_driver;        /* usb_gadget.h */
	struct completion *dev_release;                 /* Used for coordination during device removal */
	spinlock_t lock;
	struct device *dev;
	struct usb_phy *phy;
	unsigned int irq_num;
	struct iproc_ep ep[IPROC_UDC_EP_CNT];
	struct iproc_usbd_regs *usbd_regs;
	struct {
		struct iproc_udc_dma *vir_addr;
		struct iproc_udc_dma *phy_addr;
	} dma;
	/* operational flags */
	unsigned int sys_suspended : 1,
				 vbus_active    : 1,                /* Indicates if VBUS is present */
				 pullup_on      : 1,                /* Indicates if pull up is on */
				 waiting_zlp: 1,
				 connected;
};

/*
 * Structure used to hold an endpoint transfer request. Can be any number of
 * these for an endpoint.
 */
struct iproc_ep_req {
	struct usb_request usb_req;                     /* usb_gadget.h */
	struct list_head list_node;                     /* For linking in the BCM_UDC_EP request queue */
	dma_addr_t orig_dma_addr;                       /* Original buffer DMA address (physical). */
	unsigned dma_mapped     : 1;                    /* Indicates if address mapping req'd. See usb_gadget.h */
	unsigned dma_aligned    : 1;                    /* Indicates if buffer duplication done for alignment. */
};

#endif /* _XGS_IPROC_UDC_H_ */
