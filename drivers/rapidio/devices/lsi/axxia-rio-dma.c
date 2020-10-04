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

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/rio.h>
#include <linux/rio_drv.h>
#include <linux/dma-mapping.h>
#include "../../../dma/dmaengine.h"
#include <linux/uaccess.h>
#include <linux/delay.h>

#include <linux/highmem.h>

#include "axxia-rio.h"

#define RIO_NREAD 0x02
#define RIO_NWRITE 0x20
#define RIO_NWRITE_R 0x40
#define AXXIA_RIO_DMA_TX_QUEUE_SZ 1
#define MAX_LOOP_ITERATION (1000)

struct axxia_rio_gpdma_chan {
	struct dma_chan *txdmachan;
	enum dma_ctrl_flags flags;
	struct dma_device *txdmadev;
};

struct axxia_rio_dio_win {
	struct list_head node;
	struct axxia_rio_gpdma_chan *dio_channel;
	unsigned winid;
	resource_size_t win_size;
	struct rio_mport *mport;
};
struct rio_map_addr {
	void __iomem *va;
	resource_size_t phys;
};
struct axxia_rio_tx_desc {
	struct dma_async_tx_descriptor txd;
	u16 destid;
	u64 rio_addr;  /* low 64-bits of 66-bit RIO address */
	u8 rio_addr_u; /* upper 2-bits of 66-bit RIO address */
	u8 prio_lvl;   /* RIO packet priority level 0-7, msb (bit 2) is CRF */
	resource_size_t total_size;
	struct scatterlist *sg;
	unsigned int sg_len;
	enum dma_status status;
	enum dma_transfer_direction dir;
	struct rio_map_addr rio_res;
	u32 dio_type;
};

struct axxia_rio_dma_chan {
	int id;
	struct dma_chan dchan;
	struct axxia_rio_gpdma_chan *gpdma;
	bool active;
	struct axxia_rio_tx_desc axxia_txd;
	atomic_t txd_in_use;
	struct axxia_rio_tx_desc *active_tx;
	struct rio_mport *mport;
};

/*
 * Helper functions
 */

static inline struct axxia_rio_dma_chan *to_axxia_rio_chan(struct dma_chan *chan)
{
	return container_of(chan, struct axxia_rio_dma_chan, dchan);
}

static inline struct axxia_rio_tx_desc *to_axxia_rio_desc(struct dma_async_tx_descriptor *txd)
{
	return container_of(txd, struct axxia_rio_tx_desc, txd);
}

/*
 * GPDMA functions
 */

struct axxia_rio_dio_win *axxia_rio_dio_req_region(
	struct axxia_rio_gpdma_chan *dio_channel, resource_size_t size,
	struct rio_mport *mport, u32 transaction_type)
{
	struct axxia_rio_dio_win *io_win = kzalloc(sizeof(*io_win), GFP_KERNEL);

	if (!io_win) {
		dev_err(dio_channel->txdmadev->dev, "Out of memory\n");
		goto out_mem;
	}

	if(0 == is_power_of_2(size)) {
		size = __roundup_pow_of_two(size);
	}

	io_win->mport = mport;
	io_win->dio_channel = dio_channel;
	io_win->win_size = size;

	/* Setup outbound ATMU I/O window */
	if (axxia_rio_req_outb_region(mport, size, "rapidio_dio_win_axxia",
				transaction_type, &io_win->winid)) {
		dev_err(dio_channel->txdmadev->dev,
			  "Mapping RapidIO outbound ATMU window failed\n");
		goto out_map;
	}

	dev_dbg(dio_channel->txdmadev->dev, "%s: Alloc RapidIO ATMU window %u\n",
		__func__, io_win->winid);

	return io_win;

out_map:
	kfree(io_win);
out_mem:
	return NULL;
}

static struct axxia_rio_gpdma_chan *__gpdma_register(void)
{
	dma_cap_mask_t mask;
	struct axxia_rio_gpdma_chan *dma = kzalloc(sizeof(*dma),
					  GFP_KERNEL);
	if (!dma)
		return NULL;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	dma->txdmachan = dma_request_channel(mask, NULL, NULL);
	if (!dma->txdmachan) {
		kfree(dma);
		return NULL;
	}
	dma->flags = DMA_CTRL_ACK | DMA_PREP_INTERRUPT;
	dma->txdmadev = dma->txdmachan->device;

	dev_dbg(dma->txdmadev->dev, "%s: rio_dio Register DMA channel\n",
			__func__);

	return dma;
}

static void __gpdma_callback(void *completion)
{
#ifdef CONFIG_RAPIDIO_DEBUG
	printk(KERN_INFO "%s Enter\n", __func__);
#endif
	complete(completion);
}

static int __gpdma_cpy(struct axxia_rio_gpdma_chan *dio_dma,
		       struct scatterlist *sg,
		       int sg_len,
		       u32 sg_size,
		       struct rio_map_addr const * const addr,
		       enum dma_data_direction dir)
{
	struct dma_async_tx_descriptor *tx;
	dma_cookie_t cookie;
	unsigned long tmo = msecs_to_jiffies(1000);
	struct completion cmp;
	enum dma_status status = DMA_ERROR;
	u8 align;
	int rc = 0;
	struct dma_slave_config config = {0};

	dev_dbg(dio_dma->txdmadev->dev, "%s: Enter\n", __func__);

	align = dio_dma->txdmadev->copy_align;

	if (dir == DMA_TO_DEVICE) {
		config.dst_addr = addr->phys;
		config.direction = DMA_MEM_TO_DEV;
	} else {
		config.src_addr = addr->phys;
		config.direction = DMA_DEV_TO_MEM;
	}
	config.dst_addr_width = 16;
	config.dst_maxburst = 16;
	dmaengine_slave_config(dio_dma->txdmachan, &config);

	dev_dbg(dio_dma->txdmadev->dev, "%s: dev 0x%llx(%d) mem 0x%llx(%d) dir %d\n",
			__func__,
			addr->phys,
			sg_size,
			sg_dma_address(sg),
			sg_dma_len(sg),
			dir);

	tx = dio_dma->txdmadev->device_prep_slave_sg(dio_dma->txdmachan, sg, sg_len,
						     dir, dio_dma->flags, NULL);

	if (!tx) {
		dev_err(dio_dma->txdmadev->dev, "GPDMA channel prepare error\n");
		rc = -EFAULT;
		goto out_err;
	}

	init_completion(&cmp);
	tx->callback = __gpdma_callback;
	tx->callback_param = &cmp;
	cookie = tx->tx_submit(tx);
	if (dma_submit_error(cookie)) {
		dev_err(dio_dma->txdmadev->dev, "DMA channel submit error");
		rc = -EFAULT;
		goto out_err;
	}

	dma_async_issue_pending(dio_dma->txdmachan);
	tmo = wait_for_completion_timeout(&cmp, tmo);
	status = dma_async_is_tx_complete(dio_dma->txdmachan,
					  cookie, NULL, NULL);
	if (!tmo) {
		dev_err(dio_dma->txdmadev->dev, "GPDMA transfer timeout");
		rc = -ETIME;
		dmaengine_terminate_all(dio_dma->txdmachan);
		goto out_err;
	}
	if (status != DMA_COMPLETE)
		rc = -EFAULT;

out_err:
	dev_dbg(dio_dma->txdmadev->dev, "%s: Exit rc=%d\n", __func__, rc);
	return rc;

}

static inline size_t *realloc_split_table(size_t *table, u32 order)
{
	size_t *new_table = (size_t *) __get_free_pages(GFP_KERNEL, order + 1);

	if (IS_ERR_OR_NULL(new_table))
		return NULL;

	memset(new_table, 0, PAGE_SIZE  << (order + 1));
	memcpy(new_table, table, PAGE_SIZE << order);
	free_pages((unsigned long) table, order);

	return new_table;
}

/* the table size is limited to (PAGE_SIZE/size_t * 2^page_order) - 24
 * The 24 decrement is needed to avoid buffer overruns. */
#define SPLIT_TABLE_LIMIT(x) (((PAGE_SIZE/sizeof(size_t)) << (x)) - 24)

static s32 create_split_table(struct axxia_rio_tx_desc const * const desc,
			 size_t **split_table, u32 *order)
{
	u32 len = 0;
	u32 i = 0;
	u32 j = 0;
	u32 div_res = 0;
	struct scatterlist *sg = NULL;
	struct rio_map_addr addr = {0};
	u8 dw_offset = 0;
	bool last = false;
	size_t *split_sz = NULL;
	u32 page_order = 0;

	split_sz = (size_t *) get_zeroed_page(GFP_KERNEL);
	if (IS_ERR_OR_NULL(split_sz))
		return (-ENOMEM);

	addr.phys = desc->rio_res.phys;
	for_each_sg(desc->sg, sg, desc->sg_len, i) {
		/* realloc split size buffer if number of splits is too big  */
		if (unlikely(j >= SPLIT_TABLE_LIMIT(page_order))) {
			split_sz = realloc_split_table(split_sz, page_order++);
			if (IS_ERR_OR_NULL(split_sz))
				return (-ENOMEM);
		}

		len = sg_dma_len(sg);
		if (sg_is_last(sg))
		    last = true;

		dw_offset = addr.phys & 0x07ul;
                /* physical address is not double-word aligned */
                if (dw_offset) {
			if (split_sz[j]) /* if split not empty -> create new split */
				j++;

                        split_sz[j] = 8 - dw_offset;
			if (len > split_sz[j]) {
				len -= split_sz[j++];
			} else {
				split_sz[j] = len;
				len = 0;
				if (!last)
					j++;
			}
                }
		/* the current size generates segmentation and needs to be broken down
		 * in smaller chunks.
		 * */
                if ((len & 0x1ful) && (len != 16) && (len != 8)) {
			if (split_sz[j]) /* if split not empty -> create new split */
				j++;

			div_res = len & (~0x1ful);
			if (div_res)
				split_sz[j++] = div_res;
			len -= div_res;

                        while (len > 8) {
				split_sz[j] = 1 << (__fls(len) - 1);
                                len -= split_sz[j++];
                        }

                        if (len) {
                                split_sz[j] = len;
				if (!last)
					j++;
			}
                } else {
                        split_sz[j] += len;
		}
		addr.phys += sg_dma_len(sg);
	}

	*split_table = split_sz;
	*order = page_order;

	if (j) /* only one split so increment to pass to sg_split  */
		j++;

	return j;
}

static inline int __rio_dio(struct axxia_rio_gpdma_chan *dio_dma,
			    struct axxia_rio_tx_desc *desc)
{
	struct rio_map_addr addr = {0};
	struct scatterlist **sg_out = NULL;
	int *mapped_out = NULL;
	size_t *split_sz = NULL;
	int sg_list_count = 0;
	enum dma_data_direction dir = DMA_NONE;
	u32 i = 0;
	u32 order = 0;
	int rc = -EFAULT;

	sg_list_count = create_split_table(desc, &split_sz, &order);
	if (sg_list_count < 0) {
		dev_err(dio_dma->txdmadev->dev, "%s: split_sz allocation failed %d\n", __func__, rc);
		return -ENOMEM;
	}

	if (desc->dir == DMA_MEM_TO_DEV)
		dir = DMA_TO_DEVICE;
	else if (desc->dir == DMA_DEV_TO_MEM)
		dir = DMA_FROM_DEVICE;

	if (sg_list_count) {
		sg_out = (struct scatterlist **) __get_free_pages(GFP_KERNEL, order);
		if (IS_ERR_OR_NULL(sg_out)) {
			rc = -ENOMEM;
			goto alloc_err;
		}

		mapped_out = (int *) __get_free_pages(GFP_KERNEL, order);
		if (IS_ERR_OR_NULL(mapped_out)) {
			free_pages((unsigned long) sg_out, order);
			rc = -ENOMEM;
			goto alloc_err;
		}

		rc = sg_split(desc->sg, desc->sg_len, 0, sg_list_count, split_sz,
			      sg_out, mapped_out, GFP_KERNEL);

		if (rc) {
			dev_err(dio_dma->txdmadev->dev, "%s: sg_split failed -> %d\n", __func__, rc);
			goto sg_split_err;
		}

		addr.phys = desc->rio_res.phys;
		for (i = 0; i < sg_list_count; i++) {
			rc = __gpdma_cpy(dio_dma, sg_out[i], mapped_out[i], split_sz[i], &addr, dir);
			kfree(sg_out[i]);
			if (rc)
				break;
			addr.phys += split_sz[i];
		}

		for (i = i + 1; i < sg_list_count; i++)
			kfree(sg_out[i]);
sg_split_err:
		free_pages((unsigned long) sg_out, order);
		free_pages((unsigned long) mapped_out, order);
	} else {
		rc = __gpdma_cpy(dio_dma, desc->sg, desc->sg_len, desc->total_size, &desc->rio_res, dir);
	}

alloc_err:
	free_pages((unsigned long) split_sz, order);

	return rc;
}

static int __rio_dio_xfer(struct axxia_rio_dio_win *io_win,
	u32 offset, struct axxia_rio_tx_desc *desc, u32 mflags, u32 len)
{
	struct rio_priv *priv = (struct rio_priv *) io_win->mport->priv;
	u32 reg_val = 0;
	int rc = -EINVAL;
	char transfer_info[128] = {'\0'};

	dev_dbg(io_win->dio_channel->txdmadev->dev, "%s: Entry\n", __func__);

	mutex_lock(&priv->apio_mutex);

	rc = axxia_rio_map_outb_mem(io_win->mport,
			io_win->winid, desc->destid, offset, mflags,
			desc->prio_lvl, &desc->rio_res.va, &desc->rio_res.phys);
	if (rc) {
		dev_err(io_win->dio_channel->txdmadev->dev, "Map Failed\n");
		goto out;
	}

	dev_dbg(io_win->dio_channel->txdmadev->dev,
		"%s: DMA Case len = %d destid %d rioaddr 0x%08x\n",
		__func__,
		len, desc->destid, offset);
	dev_dbg(io_win->dio_channel->txdmadev->dev,
		"%s: res va = 0x%p pa = 0x%llx\n",
		__func__,
		desc->rio_res.va, desc->rio_res.phys);

	rc = __rio_dio(io_win->dio_channel, desc);

	/* reset the APIO engine to avoid generating spurious DMA interrupts for
	 * retried packets */
	if (0 != rc) {
		if (DMA_MEM_TO_DEV == desc->dir)
			snprintf(transfer_info, ARRAY_SIZE(transfer_info), "(NWRITE, DestId=0x%04x, Addr=0x%08x, Len=%u)", desc->destid, offset, len);
		else
			snprintf(transfer_info, ARRAY_SIZE(transfer_info), "(NREAD, DestId=0x%04x, Addr=0x%08x, Len=%u)", desc->destid, offset, len);
		apio_error_handler(priv, transfer_info);

		axxia_local_config_read(priv, RAB_APIO_CTRL, &reg_val);
		axxia_local_config_write(priv, RAB_PIO_RESET, RAB_PIO_RESET_APIO);
		axxia_local_config_write(priv, RAB_PIO_RESET, 0);
		axxia_local_config_write(priv, RAB_APIO_CTRL, reg_val);

		/* clear spurious errors caused by performing the APIO engine reset */
		axxia_local_config_read(priv, RAB_APIO_STAT, &reg_val);
		axxia_local_config_write(priv, RAB_APIO_STAT, reg_val);
		axxia_local_config_read(priv, RAB_APIO_STAT, &reg_val);
	}

	if (0 == axxia_local_config_read(priv, RAB_STAT, &reg_val)) {
		while (1 == (reg_val & 1)) {
			dev_err_ratelimited(priv->dev, "%s: In flight packets - 0x%x (0x%04x %d 0x%08x 0x%llx)\n", __func__,
				reg_val, desc->destid, len, offset,
				desc->rio_res.phys);
			msleep(20);
			axxia_local_config_read(priv, RAB_STAT, &reg_val);
		}
	}

out:
	reg_val = axxia_read_engine_status(priv, RAB_STAT_AXI_PIO_ENG, 0);
	if (0 != (reg_val & 0xff000000u)) {
		rc = -EIO;
		dev_err(priv->dev, "%s: DIO transfer failed! 0x%x\n", __func__, reg_val);
	}
	mutex_unlock(&priv->apio_mutex);

	dev_dbg(io_win->dio_channel->txdmadev->dev, "%s: Exit %d\n", __func__, rc);
	return rc;
}

/*
 * DMA engine API functions
 */

static dma_cookie_t axxia_rio_tx_submit(struct dma_async_tx_descriptor *txd)
{
	struct axxia_rio_tx_desc *desc = to_axxia_rio_desc(txd);
	struct axxia_rio_dma_chan *bdma_chan = to_axxia_rio_chan(txd->chan);
	struct dma_chan *dchan = txd->chan;
	dma_cookie_t cookie;

	dev_dbg(dchan->device->dev, "%s: for channel %d\n",
		__func__, bdma_chan->id);

	if (!bdma_chan->active) {
		return -ENODEV;
	}

	cookie = dma_cookie_assign(txd);
	desc->status = DMA_IN_PROGRESS;

	return cookie;
}

static int axxia_rio_alloc_chan_resources(struct dma_chan *dchan)
{
	struct axxia_rio_dma_chan *bdma_chan = to_axxia_rio_chan(dchan);
	struct axxia_rio_tx_desc *desc = NULL;

	dev_dbg(dchan->device->dev, "%s: for channel %d\n",
		__func__, bdma_chan->id);

	desc = &bdma_chan->axxia_txd;
	dma_async_tx_descriptor_init(&desc->txd, dchan);
	desc->txd.tx_submit = axxia_rio_tx_submit;
	desc->txd.flags = DMA_CTRL_ACK;
	dma_cookie_init(dchan);
	bdma_chan->active = true;

	return AXXIA_RIO_DMA_TX_QUEUE_SZ;
}

static void axxia_rio_free_chan_resources(struct dma_chan *dchan)
{
	struct axxia_rio_dma_chan *bdma_chan = to_axxia_rio_chan(dchan);
	dev_dbg(dchan->device->dev, "%s: for channel %d\n",
		__func__, bdma_chan->id);
}

static
enum dma_status axxia_rio_tx_status(struct dma_chan *dchan, dma_cookie_t cookie,
				 struct dma_tx_state *txstate)
{
	struct axxia_rio_dma_chan *bdma_chan = to_axxia_rio_chan(dchan);
	dev_dbg(dchan->device->dev, "%s: for channel %d\n",
		__func__, bdma_chan->id);
	return dma_cookie_status(dchan, cookie, txstate);
}

static bool axxia_rio_dma_is_idle(struct axxia_rio_dma_chan *bdma_chan)
{
	u32 sts;
	dev_dbg(bdma_chan->dchan.device->dev, "%s: for channel %d\n",
		__func__, bdma_chan->id);
	sts = 1;
	return sts;
}

static int axxia_rio_submit_sg(struct axxia_rio_tx_desc *desc)
{
	struct dma_chan *dchan = desc->txd.chan;
	struct axxia_rio_dma_chan *bdma_chan = to_axxia_rio_chan(dchan);
	u64 rio_addr;
	struct scatterlist *sg;
	unsigned int i;
	int err = 0;
	resource_size_t size = 0;
	struct axxia_rio_dio_win *rwin;
	struct dma_async_tx_descriptor *txd = &desc->txd;
	dma_async_tx_callback callback = txd->callback;
	void *param = txd->callback_param;

	if (!axxia_rio_dma_is_idle(bdma_chan)) {
		dev_err(bdma_chan->dchan.device->dev,
			"BUG: Attempt to use non-idle channel\n");
		err = -EIO;
		goto unlock_out;
	}

	rio_addr = desc->rio_addr;

	for_each_sg(desc->sg, sg, desc->sg_len, i) {
		size += sg_dma_len(sg);

		dev_dbg(bdma_chan->dchan.device->dev,
			"%s: sg%d/%d addr: 0x%llx 0x%p len: %d\n",
			__func__,
			i, desc->sg_len,
			sg_dma_address(sg),
			sg_virt(sg),
			sg_dma_len(sg));
		dev_dbg(bdma_chan->dchan.device->dev,
			"%s: size = %lld\n", __func__, size);
	}
	desc->total_size = size;

	bdma_chan->gpdma = __gpdma_register();
	if (!bdma_chan->gpdma) {
                dev_err(bdma_chan->dchan.device->dev,
                        "Failed to register GPDMA resources\n");
		err = -EFAULT;
		goto unlock_out;
	}
	dev_dbg(bdma_chan->dchan.device->dev,
		"%s: Allocated a GPDMA Channel\n", __func__);

	rwin = axxia_rio_dio_req_region(bdma_chan->gpdma, size,
		dma_to_mport(bdma_chan->dchan.device), desc->dio_type);
	if (!rwin) {
		dev_err(bdma_chan->dchan.device->dev,
			"Cannot request axxia dio region\n");
		err = -EFAULT;
		goto dma_release_out;
	}
	dev_dbg(bdma_chan->dchan.device->dev,
		"destid 0x%04x rio_addr %llx\n", desc->destid, rio_addr);

	err = __rio_dio_xfer(rwin, rio_addr, desc, desc->dio_type,
					size);
	if (err) {
		dev_info(bdma_chan->dchan.device->dev,
			"DirectIO DMA transfer failed with rc=%d\n", err);
	}

	dev_dbg(bdma_chan->dchan.device->dev,
		"%s: Return ATMU window %u\n", __func__, rwin->winid);
	axxia_rio_release_outb_region(dma_to_mport(bdma_chan->dchan.device),
				 rwin->winid);
	kfree(rwin);
dma_release_out:
	dev_dbg(bdma_chan->dchan.device->dev,
		"%s: rio_dio Release DMA channel\n", __func__);
	dma_release_channel(bdma_chan->gpdma->txdmachan);
	kfree(bdma_chan->gpdma);

	bdma_chan->active_tx = NULL;
	desc->status = DMA_ERROR;
        if ((!err) && (callback)) {
		desc->status = DMA_COMPLETE;
                dma_cookie_complete(&desc->txd);
		dev_dbg(bdma_chan->dchan.device->dev, "%s: Original Callback\n", __func__);
                callback(param);
	}

unlock_out:
	atomic_set(&bdma_chan->txd_in_use, 0);
	return err;
}

static void axxia_rio_advance_work(struct axxia_rio_dma_chan *bdma_chan,
                                struct axxia_rio_tx_desc *desc)
{
	dev_dbg(bdma_chan->dchan.device->dev, "%s: Enter\n", __func__);

	if (!axxia_rio_dma_is_idle(bdma_chan))
		return;

	if (desc == NULL && bdma_chan->active_tx == NULL) {
		desc = &bdma_chan->axxia_txd;
		bdma_chan->active_tx = desc;
	}

	if (desc) {
		if (!axxia_rio_submit_sg(desc)) {
			dev_dbg(bdma_chan->dchan.device->dev,
				"submit_sg: DMA done\n");
		}
	}

	dev_dbg(bdma_chan->dchan.device->dev, "%s: Exit\n", __func__);
}

static void axxia_rio_issue_pending(struct dma_chan *dchan)
{
	struct axxia_rio_dma_chan *bdma_chan = to_axxia_rio_chan(dchan);

	dev_dbg(dchan->device->dev, "%s: Enter\n", __func__);

	if (axxia_rio_dma_is_idle(bdma_chan) && bdma_chan->active) {
		axxia_rio_advance_work(bdma_chan, NULL);
	}
}

static
struct dma_async_tx_descriptor *axxia_rio_prep_rio_sg(struct dma_chan *dchan,
			struct scatterlist *sgl, unsigned int sg_len,
			enum dma_transfer_direction dir, unsigned long flags,
			void *tinfo)
{
	struct axxia_rio_dma_chan *bdma_chan = to_axxia_rio_chan(dchan);
	struct axxia_rio_tx_desc *desc;
	struct rio_dma_ext *rext = tinfo;
	struct dma_async_tx_descriptor *txd = NULL;
	u32 dio_type;
	u32 iteration = 0;

	if (!sgl || !sg_len) {
		dev_err(dchan->device->dev, "%s: No SG list\n", __func__);
		return NULL;
	}

	dev_dbg(dchan->device->dev, "%s: %s\n", __func__,
		(dir == DMA_DEV_TO_MEM)?"READ":"WRITE");

	if (dir == DMA_DEV_TO_MEM) {
/*		dio_type = RIO_NREAD; */
		/* The intention would be to use NREAD here, but the value is used
		for the DIO window mapping. This uses the same config bits for
		NREAD and NWRITE, so it decodes only the NWRITE as valid data and
		the NREAD is not implemented and causes an error response. */
		dio_type = RIO_NWRITE;
	} else if (dir == DMA_MEM_TO_DEV) {
		switch (rext->wr_type) {
		case RDW_ALL_NWRITE:
			dio_type = RIO_NWRITE;
			break;
		case RDW_LAST_NWRITE_R:
			dev_warn(dchan->device->dev,
			"%s: RDW_LAST_NWRITE_R not supported, fall back to RDW_ALL_NWRITE_R\n", __func__);
		case RDW_ALL_NWRITE_R:
		default:
			dio_type = RIO_NWRITE_R;
			break;
		}
	} else {
		dev_err(dchan->device->dev,
			"%s: Unsupported DMA direction option\n", __func__);
		return NULL;
	}

	while (1) {
		if (atomic_cmpxchg(&bdma_chan->txd_in_use, 0, 1) == 0) {
			desc = &bdma_chan->axxia_txd;
			desc->destid = rext->destid;
			desc->rio_addr = rext->rio_addr;
			desc->rio_addr_u = 0;
			desc->prio_lvl = rext->prio_lvl;
			desc->dio_type = dio_type;
			desc->dir = dir;
			desc->sg_len = sg_len;
			desc->sg = sgl;
			txd = &desc->txd;
			txd->flags = flags;
			break;
		}
		msleep(20);
		iteration++;
		if (iteration > MAX_LOOP_ITERATION) {
			dev_warn(dchan->device->dev,
				"DMA loop, waited too long, exiting\n");
			break;
		}
	}

	return txd;
}

static int axxia_rio_terminate_all(struct dma_chan *dchan)
{
	dev_dbg(dchan->device->dev, "%s: Entry\n", __func__);

	return 0;
}

void axxia_rio_dma_stop_all(struct rio_priv *priv)
{
#ifdef CONFIG_RAPIDIO_DEBUG
	printk(KERN_INFO "%s called\n", __func__);
#endif
}

int axxia_rio_register_dma(struct rio_priv *priv)
{
	int err;
	struct rio_mport *mport = priv->mport;
	struct axxia_rio_dma_chan *bdma_chan = NULL;

	dev_dbg(&mport->dev, "%s: register DMA device for mport\n", __func__);

	INIT_LIST_HEAD(&mport->dma.channels);
	bdma_chan = kzalloc(sizeof(struct axxia_rio_dma_chan), GFP_KERNEL);
	bdma_chan->dchan.device = &mport->dma;
	bdma_chan->dchan.cookie = 1;
	bdma_chan->active = false;
	bdma_chan->active_tx = NULL;
	bdma_chan->mport = mport;
	atomic_set(&bdma_chan->txd_in_use, 0);
	list_add_tail(&bdma_chan->dchan.device_node, &mport->dma.channels);

	mport->dma.chancnt = 1;
	dma_cap_zero(mport->dma.cap_mask);
	dma_cap_set(DMA_PRIVATE, mport->dma.cap_mask);
	dma_cap_set(DMA_SLAVE, mport->dma.cap_mask);

	mport->dma.dev = priv->dev;
	mport->dma.device_alloc_chan_resources = axxia_rio_alloc_chan_resources;
	mport->dma.device_free_chan_resources = axxia_rio_free_chan_resources;
	mport->dma.device_tx_status = axxia_rio_tx_status;
	mport->dma.device_issue_pending = axxia_rio_issue_pending;
	mport->dma.device_prep_slave_sg = axxia_rio_prep_rio_sg;
	mport->dma.device_terminate_all = axxia_rio_terminate_all;

	err = dma_async_device_register(&mport->dma);
	if (err)
		dev_err(&mport->dev, "Failed to register DMA device\n");

	return err;
}

void axxia_rio_unregister_dma(struct rio_priv *priv)
{
	struct rio_mport *mport = priv->mport;
	struct dma_chan *chan, *_c;
	struct axxia_rio_dma_chan *bdma_chan;

	axxia_rio_dma_stop_all(priv);
	dma_async_device_unregister(&mport->dma);

	list_for_each_entry_safe(chan, _c, &mport->dma.channels,
					device_node) {
		bdma_chan = to_axxia_rio_chan(chan);
		if (bdma_chan->active) {
			bdma_chan->active = false;
		}

		list_del(&chan->device_node);
		kfree(bdma_chan);
	}
}
