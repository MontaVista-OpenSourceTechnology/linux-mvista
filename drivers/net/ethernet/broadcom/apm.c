/*
 * Copyright (C) 2016 Broadcom
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
#include <linux/bcma/bcma.h>
#include <linux/etherdevice.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include "apm.h"
#include "pm.h"

extern void __iomem *get_iproc_wrap_ctrl_base(void);
static bool apm_clk_enabled(struct apm *apm);

static u32 apm_read(struct apm *apm, u16 offset)
{
	return readl(apm->plat.base + offset);
}

static void apm_write(struct apm *apm, u16 offset, u32 value)
{
	writel(value, apm->plat.base + offset);
}

static u32 apm_idm_read(struct apm *apm, u16 offset)
{
	return readl(apm->plat.idm_base + offset);
}

static void apm_idm_write(struct apm *apm, u16 offset, u32 value)
{
	return writel(value, apm->plat.idm_base + offset);
}

static bool apm_wait_value(struct apm *apm, u16 reg, u32 mask,
			     u32 value, int timeout)
{
	u32 val;
	int i;

	for (i = 0; i < timeout / 10; i++) {
		val = apm_read(apm, reg);
		if ((val & mask) == value)
			return true;
		udelay(10);
	}
	dev_err(apm->dev, "Timeout waiting for reg 0x%X\n", reg);
	return false;
}

/**************************************************
 * DMA
 **************************************************/
static void apm_dma_tx_reset(struct apm *apm, struct apm_dma_ring *tx_ring)
{
	u32 val;
	int i;

	if (!tx_ring->mmio_base)
		return;

	/* Suspend DMA TX ring first.
	 * apm_wait_value doesn't support waiting for any of few values, so
	 * implement whole loop here.
	 */
	val = apm_read(apm, tx_ring->mmio_base + APM_DMA_TX_CTL);
	val &= ~APM_DMA_TX_ENABLE;
	val |= APM_DMA_TX_SUSPEND;
	apm_write(apm, tx_ring->mmio_base + APM_DMA_TX_CTL, val);
	for (i = 0; i < 10000 / 10; i++) {
		val = apm_read(apm, tx_ring->mmio_base + APM_DMA_TX_STATUS);
		val &= APM_DMA_TX_STAT;
		if (val == APM_DMA_TX_STAT_DISABLED ||
		    val == APM_DMA_TX_STAT_IDLEWAIT ||
		    val == APM_DMA_TX_STAT_STOPPED) {
			i = 0;
			break;
		}
		udelay(10);
	}
	if (i) {
		dev_err(apm->dev, "Timeout suspending DMA TX ring 0x%X (APM_DMA_TX_STAT: 0x%08X)\n",
			tx_ring->mmio_base, val);
	}

	/* Disable the transmit channel */
	val = apm_read(apm, tx_ring->mmio_base + APM_DMA_TX_CTL);
	val &= ~APM_DMA_TX_SUSPEND;
	apm_write(apm, tx_ring->mmio_base + APM_DMA_TX_CTL, val);
	if (!apm_wait_value(apm, tx_ring->mmio_base + APM_DMA_TX_STATUS,
				APM_DMA_TX_STAT, APM_DMA_TX_STAT_DISABLED, 10000)) {
		dev_warn(apm->dev, "DMA TX ring 0x%X wasn't disabled on time, waiting additional 300us\n",
			 tx_ring->mmio_base);
		udelay(300);

		val = apm_read(apm, tx_ring->mmio_base + APM_DMA_TX_STATUS);
		if ((val & APM_DMA_TX_STAT) != APM_DMA_TX_STAT_DISABLED)
			dev_err(apm->dev, "Reset of DMA TX ring 0x%X failed\n",
				tx_ring->mmio_base);
	}
}

static void apm_dma_tx_enable(struct apm *apm,
				struct apm_dma_ring *tx_ring)
{
	u32 ctl;

	ctl = apm_read(apm, tx_ring->mmio_base + APM_DMA_TX_CTL);
	if (apm->feature_flags & APM_FEAT_TX_MASK_SETUP) {
		ctl &= ~APM_DMA_TX_BL_MASK;
		ctl |= APM_DMA_TX_BL_128 << APM_DMA_TX_BL_SHIFT;

		ctl &= ~APM_DMA_TX_PC_MASK;
		ctl |= APM_DMA_TX_PC_16 << APM_DMA_TX_PC_SHIFT;

		ctl &= ~APM_DMA_TX_PT_MASK;
		ctl |= APM_DMA_TX_PT_8 << APM_DMA_TX_PT_SHIFT;
	}
	ctl |= APM_DMA_TX_ENABLE;
	apm_write(apm, tx_ring->mmio_base + APM_DMA_TX_CTL, ctl);
}

static void
apm_dma_tx_add_buf(struct apm *apm, struct apm_dma_ring *tx_ring,
		     int i, int len, u32 ctl0)
{
	struct apm_slot_info *slot;
	struct apm_dma_desc *dma_desc;
	u32 ctl1;

	if (i == tx_ring->desc_num - 1)
		ctl0 |= APM_DESC_CTL0_EOT;

	ctl1 = len & APM_DESC_CTL1_LEN;

	slot = &tx_ring->slots[i];
	dma_desc = &tx_ring->desc_base[i];
	dma_desc->addr_low = cpu_to_le32(lower_32_bits(slot->dma_addr));
	dma_desc->addr_high = cpu_to_le32(upper_32_bits(slot->dma_addr));
	dma_desc->ctl0 = cpu_to_le32(ctl0);
	dma_desc->ctl1 = cpu_to_le32(ctl1);
}

static netdev_tx_t apm_dma_tx_add(struct apm *apm,
				    struct apm_dma_ring *tx_ring, struct sk_buff *skb)
{
	struct device *dma_dev = apm->dma_dev;
	struct net_device *net_dev = apm->net_dev;
	int index = tx_ring->end % tx_ring->desc_num;
	struct apm_slot_info *slot = &tx_ring->slots[index];
	int nr_frags;
	u32 flags;
	int i;

	if (skb->len > APM_DESC_CTL1_LEN) {
		netdev_err(apm->net_dev, "Too long skb (%d)\n", skb->len);
		goto err_drop;
	}

	if (skb->ip_summed == CHECKSUM_PARTIAL)
		skb_checksum_help(skb);

	nr_frags = skb_shinfo(skb)->nr_frags;

	/* ring->end - ring->start will return the number of valid slots,
	 * even when tx_ring->end overflows
	 */
	if (tx_ring->end - tx_ring->start + nr_frags + 1 >= tx_ring->desc_num) {
		netdev_err(apm->net_dev, "TX ring is full, queue should be stopped!\n");
		netif_stop_queue(net_dev);
		return NETDEV_TX_BUSY;
	}

	slot->dma_addr = dma_map_single(dma_dev, skb->data, skb_headlen(skb),
					DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dma_dev, slot->dma_addr)))
		goto err_dma_head;

	flags = APM_DESC_CTL0_SOF;
	if (!nr_frags)
		flags |= APM_DESC_CTL0_EOF | APM_DESC_CTL0_IOC;

	apm_dma_tx_add_buf(apm, tx_ring, index, skb_headlen(skb), flags);
	flags = 0;

	for (i = 0; i < nr_frags; i++) {
		struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[i];
		int len = skb_frag_size(frag);

		index = (index + 1) % tx_ring->desc_num;
		slot = &tx_ring->slots[index];
		slot->dma_addr = skb_frag_dma_map(dma_dev, frag, 0,
						  len, DMA_TO_DEVICE);
		if (unlikely(dma_mapping_error(dma_dev, slot->dma_addr)))
			goto err_dma;

		if (i == nr_frags - 1)
			flags |= APM_DESC_CTL0_EOF | APM_DESC_CTL0_IOC;

		apm_dma_tx_add_buf(apm, tx_ring, index, len, flags);
	}

	slot->skb = skb;
	tx_ring->end += nr_frags + 1;
	netdev_sent_queue(net_dev, skb->len);

	wmb();

	/* Increase tx_ring->end to point empty slot. We tell hardware the first
	 * slot it should *not* read.
	 */
	apm_write(apm, tx_ring->mmio_base + APM_DMA_TX_INDEX,
		    tx_ring->index_base +
		    (tx_ring->end % tx_ring->desc_num) *
		    sizeof(struct apm_dma_desc));

	if (tx_ring->end - tx_ring->start >= tx_ring->desc_num - 8)
		netif_stop_queue(net_dev);

	return NETDEV_TX_OK;

err_dma:
	dma_unmap_single(dma_dev, slot->dma_addr, skb_headlen(skb),
			 DMA_TO_DEVICE);

	while (i-- > 0) {
		int index = (tx_ring->end + i) % tx_ring->desc_num;
		struct apm_slot_info *slot = &tx_ring->slots[index];
		u32 ctl1 = le32_to_cpu(tx_ring->desc_base[index].ctl1);
		int len = ctl1 & APM_DESC_CTL1_LEN;

		dma_unmap_page(dma_dev, slot->dma_addr, len, DMA_TO_DEVICE);
	}

err_dma_head:
	netdev_err(apm->net_dev, "Mapping error of skb on TX ring 0x%X\n",
		   tx_ring->mmio_base);

err_drop:
	dev_kfree_skb(skb);
	net_dev->stats.tx_dropped++;
	net_dev->stats.tx_errors++;
	return NETDEV_TX_OK;
}

/* Free transmitted packets */
static void apm_dma_tx_free(struct apm *apm, struct apm_dma_ring *tx_ring)
{
	struct device *dma_dev = apm->dma_dev;
	int empty_slot;
	bool freed = false;
	unsigned bytes_compl = 0, pkts_compl = 0;

	/* The last slot that hardware didn't consume yet */
	empty_slot = apm_read(apm, tx_ring->mmio_base + APM_DMA_TX_STATUS);
	empty_slot &= APM_DMA_TX_STATDPTR;
	empty_slot -= tx_ring->index_base;
	empty_slot &= APM_DMA_TX_STATDPTR;
	empty_slot /= sizeof(struct apm_dma_desc);

	while (tx_ring->start != tx_ring->end) {
		int index = tx_ring->start % tx_ring->desc_num;
		struct apm_slot_info *slot = &tx_ring->slots[index];
		u32 ctl0, ctl1;
		int len;

		if (index == empty_slot)
			break;

		ctl0 = le32_to_cpu(tx_ring->desc_base[index].ctl0);
		ctl1 = le32_to_cpu(tx_ring->desc_base[index].ctl1);
		len = ctl1 & APM_DESC_CTL1_LEN;
		if (ctl0 & APM_DESC_CTL0_SOF)
			/* Unmap no longer used buffer */
			dma_unmap_single(dma_dev, slot->dma_addr, len,
					 DMA_TO_DEVICE);
		else
			dma_unmap_page(dma_dev, slot->dma_addr, len,
				       DMA_TO_DEVICE);

		if (slot->skb) {
			apm->net_dev->stats.tx_bytes += slot->skb->len;
			apm->net_dev->stats.tx_packets++;
			bytes_compl += slot->skb->len;
			pkts_compl++;

			/* Free memory! :) */
			dev_kfree_skb(slot->skb);
			slot->skb = NULL;
		}

		slot->dma_addr = 0;
		tx_ring->start++;
		freed = true;
	}

	if (!pkts_compl)
		return;

	netdev_completed_queue(apm->net_dev, pkts_compl, bytes_compl);

	if (netif_queue_stopped(apm->net_dev))
		netif_wake_queue(apm->net_dev);
}

static void apm_dma_rx_reset(struct apm *apm, struct apm_dma_ring *rx_ring)
{
	if (!rx_ring->mmio_base)
		return;

	apm_write(apm, rx_ring->mmio_base + APM_DMA_RX_CTL, 0);
	if (!apm_wait_value(apm,
			      rx_ring->mmio_base + APM_DMA_RX_STATUS,
			      APM_DMA_RX_STAT, APM_DMA_RX_STAT_DISABLED,
			      10000))
		dev_err(apm->dev, "Reset of RX ring 0x%X RX failed\n",
			rx_ring->mmio_base);
}

static void apm_dma_rx_enable(struct apm *apm,
				struct apm_dma_ring *rx_ring)
{
	u32 ctl;

	ctl = apm_read(apm, rx_ring->mmio_base + APM_DMA_RX_CTL);

	if (apm->feature_flags & APM_FEAT_RX_MASK_SETUP) {
		ctl &= ~APM_DMA_RX_BL_MASK;
		ctl |= APM_DMA_RX_BL_128 << APM_DMA_RX_BL_SHIFT;

		ctl &= ~APM_DMA_RX_PC_MASK;
		ctl |= APM_DMA_RX_PC_8 << APM_DMA_RX_PC_SHIFT;

		ctl &= ~APM_DMA_RX_PT_MASK;
		ctl |= APM_DMA_RX_PT_1 << APM_DMA_RX_PT_SHIFT;
	}
	ctl |= APM_DMA_RX_ENABLE;
	ctl |= APM_DMA_RX_OVERFLOW_CONT;
	ctl |= APM_RX_FRAME_OFFSET << APM_DMA_RX_FRAME_OFFSET_SHIFT;
	apm_write(apm, rx_ring->mmio_base + APM_DMA_RX_CTL, ctl);
}

static int apm_dma_rx_skb_for_slot(struct apm *apm,
				     struct apm_slot_info *slot)
{
	struct device *dma_dev = apm->dma_dev;
	dma_addr_t dma_addr;
	struct apm_rx_header *rx;
	void *buf;

	/* Alloc skb */
	buf = netdev_alloc_frag(APM_RX_ALLOC_SIZE);
	if (!buf)
		return -ENOMEM;

	/* Poison - if everything goes fine, hardware will overwrite it */
	rx = buf + APM_RX_BUF_OFFSET;
	rx->len = cpu_to_le16(0xdead);
	rx->flags = cpu_to_le16(0xbeef);

	/* Map skb for the DMA */
	dma_addr = dma_map_single(dma_dev, buf + APM_RX_BUF_OFFSET,
				  APM_RX_BUF_SIZE, DMA_FROM_DEVICE);
	if (dma_mapping_error(dma_dev, dma_addr)) {
		netdev_err(apm->net_dev, "DMA mapping error\n");
		put_page(virt_to_head_page(buf));
		return -ENOMEM;
	}

	/* Update the slot */
	slot->buf = buf;
	slot->dma_addr = dma_addr;

	return 0;
}

static void apm_dma_rx_update_index(struct apm *apm,
				      struct apm_dma_ring *rx_ring)
{
	dma_wmb();

	apm_write(apm, rx_ring->mmio_base + APM_DMA_RX_INDEX,
		    rx_ring->index_base +
		    rx_ring->end * sizeof(struct apm_dma_desc));
}

static void apm_dma_rx_setup_desc(struct apm *apm,
				    struct apm_dma_ring *rx_ring, int index)
{
	struct apm_dma_desc *dma_desc = rx_ring->desc_base + index;
	u32 ctl0 = 0, ctl1 = 0;

	if (index == rx_ring->desc_num - 1)
		ctl0 |= APM_DESC_CTL0_EOT;
	ctl1 |= APM_RX_BUF_SIZE & APM_DESC_CTL1_LEN;
	/* Is there any APM device that requires extension? */
	/* ctl1 |= (addrext << B43_DMA64_DCTL1_ADDREXT_SHIFT) &
	 * B43_DMA64_DCTL1_ADDREXT_MASK;
	 */

	dma_desc->addr_low = cpu_to_le32(lower_32_bits(rx_ring->slots[index].dma_addr));
	dma_desc->addr_high = cpu_to_le32(upper_32_bits(rx_ring->slots[index].dma_addr));
	dma_desc->ctl0 = cpu_to_le32(ctl0);
	dma_desc->ctl1 = cpu_to_le32(ctl1);

	rx_ring->end = index;
}

static void apm_dma_rx_poison_buf(struct device *dma_dev,
				    struct apm_slot_info *slot)
{
	struct apm_rx_header *rx = slot->buf + APM_RX_BUF_OFFSET;

	dma_sync_single_for_cpu(dma_dev, slot->dma_addr, APM_RX_BUF_SIZE,
				DMA_FROM_DEVICE);
	rx->len = cpu_to_le16(0xdead);
	rx->flags = cpu_to_le16(0xbeef);
	dma_sync_single_for_device(dma_dev, slot->dma_addr, APM_RX_BUF_SIZE,
				   DMA_FROM_DEVICE);
}

static int apm_dma_rx_read(struct apm *apm, struct apm_dma_ring *rx_ring,
			     int weight)
{
	u32 end_slot;
	int handled = 0;

	end_slot = apm_read(apm, rx_ring->mmio_base + APM_DMA_RX_STATUS);
	end_slot &= APM_DMA_RX_STATDPTR;
	end_slot -= rx_ring->index_base;
	end_slot &= APM_DMA_RX_STATDPTR;
	end_slot /= sizeof(struct apm_dma_desc);

	while (rx_ring->start != end_slot) {
		struct device *dma_dev = apm->dma_dev;
		struct apm_slot_info *slot = &rx_ring->slots[rx_ring->start];
		struct apm_rx_header *rx = slot->buf + APM_RX_BUF_OFFSET;
		struct sk_buff *skb;
		void *buf = slot->buf;
		dma_addr_t dma_addr = slot->dma_addr;
		u16 len, flags;

		do {
			/* Prepare new skb as replacement */
			if (apm_dma_rx_skb_for_slot(apm, slot)) {
				apm_dma_rx_poison_buf(dma_dev, slot);
				break;
			}

			/* Unmap buffer to make it accessible to the CPU */
			dma_unmap_single(dma_dev, dma_addr,
					 APM_RX_BUF_SIZE, DMA_FROM_DEVICE);

			/* Get info from the header */
			len = le16_to_cpu(rx->len);
			flags = le16_to_cpu(rx->flags);

			/* Check for poison and drop or pass the packet */
			if (len == 0xdead && flags == 0xbeef) {
				netdev_err(apm->net_dev, "Found poisoned packet at slot %d, DMA issue!\n",
					   rx_ring->start);
				put_page(virt_to_head_page(buf));
				apm->net_dev->stats.rx_errors++;
				break;
			}

			if (len > APM_RX_ALLOC_SIZE) {
				netdev_err(apm->net_dev, "Found oversized packet at slot %d, DMA issue!\n",
					   rx_ring->start);
				put_page(virt_to_head_page(buf));
				apm->net_dev->stats.rx_length_errors++;
				apm->net_dev->stats.rx_errors++;
				break;
			}

			/* Omit CRC. */
			len -= ETH_FCS_LEN;

			skb = build_skb(buf, APM_RX_ALLOC_SIZE);
			if (unlikely(!skb)) {
				netdev_err(apm->net_dev, "build_skb failed\n");
				put_page(virt_to_head_page(buf));
				apm->net_dev->stats.rx_errors++;
				break;
			}
			skb_put(skb, APM_RX_FRAME_OFFSET +
				APM_RX_BUF_OFFSET + len);
			skb_pull(skb, APM_RX_FRAME_OFFSET +
				 APM_RX_BUF_OFFSET);

			skb_checksum_none_assert(skb);
			skb->protocol = eth_type_trans(skb, apm->net_dev);
			apm->net_dev->stats.rx_bytes += len;
			apm->net_dev->stats.rx_packets++;
			napi_gro_receive(&apm->napi, skb);
			handled++;
		} while (0);

		apm_dma_rx_setup_desc(apm, rx_ring, rx_ring->start);

		if (++rx_ring->start >= rx_ring->desc_num)
			rx_ring->start = 0;

		if (handled >= weight) /* Should never be greater */
			break;
	}

	apm_dma_rx_update_index(apm, rx_ring);

	return handled;
}

/* Does ring support unaligned addressing? */
static bool apm_dma_unaligned(struct apm *apm,
				struct apm_dma_ring *ring,
				enum apm_dma_ring_type ring_type)
{
	switch (ring_type) {
		case APM_DMA_RING_TYPE_TX:
			apm_write(apm, ring->mmio_base + APM_DMA_TX_RINGLO, 0xff0);
			if (apm_read(apm, ring->mmio_base + APM_DMA_TX_RINGLO))
				return true;
			break;
		case APM_DMA_RING_TYPE_RX:
			apm_write(apm, ring->mmio_base + APM_DMA_RX_RINGLO, 0xff0);
			if (apm_read(apm, ring->mmio_base + APM_DMA_RX_RINGLO))
				return true;
			break;
		default:
			return false;
	}
	return false;
}

static void apm_dma_tx_ring_free(struct apm *apm,
				   struct apm_dma_ring *tx_ring)
{
	struct device *dma_dev = apm->dma_dev;
	struct apm_dma_desc *dma_desc = tx_ring->desc_base;
	struct apm_slot_info *slot;
	int i;

	for (i = 0; i < tx_ring->desc_num; i++) {
		int len = dma_desc[i].ctl1 & APM_DESC_CTL1_LEN;

		slot = &tx_ring->slots[i];
		dev_kfree_skb(slot->skb);
		if (!slot->dma_addr)
			continue;

		if (slot->skb)
			dma_unmap_single(dma_dev, slot->dma_addr,
					 len, DMA_TO_DEVICE);
		else
			dma_unmap_page(dma_dev, slot->dma_addr,
				       len, DMA_TO_DEVICE);
	}
}

static void apm_dma_rx_ring_free(struct apm *apm,
				   struct apm_dma_ring *rx_ring)
{
	struct device *dma_dev = apm->dma_dev;
	struct apm_slot_info *slot;
	int i;

	for (i = 0; i < rx_ring->desc_num; i++) {
		slot = &rx_ring->slots[i];
		if (!slot->dma_addr)
			continue;

		dma_unmap_single(dma_dev, slot->dma_addr,
				 APM_RX_BUF_SIZE,
				 DMA_FROM_DEVICE);
		put_page(virt_to_head_page(slot->buf));
		slot->dma_addr = 0;
	}
}

static void apm_dma_cleanup(struct apm *apm)
{
	int i;

	for (i = 0; i < apm->tx_channel; i++) {
		apm_dma_tx_ring_free(apm, &apm->tx_ring[i]);
	}

	apm_dma_rx_ring_free(apm, &apm->rx_ring[0]);
}

static void apm_dma_free(struct apm *apm)
{
	struct device *dma_dev = apm->dma_dev;
	int size;

	if (apm->desc_buf) {
		size = (APM_TX_MAX_DESCS + APM_RX_MAX_DESCS) *
						sizeof(struct apm_dma_desc);
		dma_free_coherent(dma_dev, size, apm->desc_buf, apm->dma_addr);
	}

	if (apm->slot_buf) {
		kfree(apm->slot_buf);
	}
}

static int apm_dma_alloc(struct apm *apm)
{
	struct device *dma_dev = apm->dma_dev;
	int size;

	size = (APM_TX_MAX_DESCS + APM_RX_MAX_DESCS) *
						sizeof(struct apm_dma_desc);
	apm->desc_buf = dma_zalloc_coherent(dma_dev, size,
						&apm->dma_addr, GFP_KERNEL);
	if (!apm->desc_buf) {
		dev_err(apm->dev, "Descriptor buffer allocation failed\n");
		goto err_dma_free;
	}
	memset(apm->desc_buf, 0, size);

	size = (APM_TX_MAX_DESCS + APM_RX_MAX_DESCS) *
						sizeof(struct apm_slot_info);
	apm->slot_buf = kmalloc(size, GFP_KERNEL);
	if (!apm->slot_buf) {
		dev_err(apm->dev, "Data buffer allocation failed\n");
		goto err_dma_free;
	}
	memset(apm->slot_buf, 0, size);
	return 0;

err_dma_free:
	apm_dma_free(apm);
	return -ENOMEM;
}

static int apm_tx_dma_init(struct apm *apm,
					struct apm_dma_ring *tx_ring, int channel)
{
	const u16 ring_base[] = { APM_DMA_BASE0, APM_DMA_BASE1,
							  APM_DMA_BASE2, APM_DMA_BASE3,};
	int desc_num = APM_TX_MAX_DESCS / apm->tx_channel;
	int offset = channel * desc_num;

	BUILD_BUG_ON(APM_MAX_TX_RINGS > ARRAY_SIZE(ring_base));

	tx_ring->mmio_base = ring_base[channel];
	if ((apm->tx_channel == 2) && (channel == 1)) {
		tx_ring->mmio_base = ring_base[2];
	}

	tx_ring->desc_num = desc_num;
	tx_ring->desc_base = apm->desc_buf + offset;
	tx_ring->dma_base = (dma_addr_t)(apm->dma_addr +
						offset * sizeof(struct apm_dma_desc));
	tx_ring->slots = apm->slot_buf + offset;

	tx_ring->index_base = 0;
	tx_ring->unaligned = apm_dma_unaligned(apm,
						tx_ring, APM_DMA_RING_TYPE_TX);
	if (tx_ring->unaligned) {
		tx_ring->index_base = lower_32_bits(tx_ring->dma_base);
	}

	tx_ring->start = 0;
	tx_ring->end = 0;	/* Points the slot that should *not* be read */

	return 0;
}

static int apm_rx_dma_init(struct apm *apm, struct apm_dma_ring *rx_ring)
{
	int offset = APM_TX_MAX_DESCS;

	rx_ring->desc_num = APM_RX_MAX_DESCS;
	rx_ring->mmio_base = APM_DMA_BASE0;
	rx_ring->desc_base = apm->desc_buf + offset;
	rx_ring->dma_base = (dma_addr_t)(apm->dma_addr +
						offset * sizeof(struct apm_dma_desc));
	rx_ring->slots = apm->slot_buf + offset;

	rx_ring->index_base = 0;
	rx_ring->unaligned = apm_dma_unaligned(apm,
						rx_ring, APM_DMA_RING_TYPE_RX);
	if (rx_ring->unaligned) {
		rx_ring->index_base = lower_32_bits(rx_ring->dma_base);
	}

	rx_ring->start = 0;
	rx_ring->end = 0;	/* Points the slot that should *not* be read */

	return 0;
}

static int apm_dma_init(struct apm *apm)
{
	struct apm_dma_ring *tx_ring, *rx_ring;
	int i, err;

	/* TX DMA init */
	for (i = 0; i < apm->tx_channel; i++) {
		tx_ring = &apm->tx_ring[i];
		apm_tx_dma_init(apm, tx_ring, i);

		if (!tx_ring->unaligned) {
			apm_dma_tx_enable(apm, tx_ring);
		}

		apm_write(apm, tx_ring->mmio_base + APM_DMA_TX_RINGLO,
			    lower_32_bits(tx_ring->dma_base));
		apm_write(apm, tx_ring->mmio_base + APM_DMA_TX_RINGHI,
			    upper_32_bits(tx_ring->dma_base));

		if (tx_ring->unaligned) {
			apm_dma_tx_enable(apm, tx_ring);
		}
	}

	/* RX DMA init */
	rx_ring = &apm->rx_ring[0];
	apm_rx_dma_init(apm, rx_ring);

	if (!rx_ring->unaligned) {
		apm_dma_rx_enable(apm, rx_ring);
	}

	apm_write(apm, rx_ring->mmio_base + APM_DMA_RX_RINGLO,
		    lower_32_bits(rx_ring->dma_base));
	apm_write(apm, rx_ring->mmio_base + APM_DMA_RX_RINGHI,
		    upper_32_bits(rx_ring->dma_base));

	if (rx_ring->unaligned) {
		apm_dma_rx_enable(apm, rx_ring);
	}

	for (i = 0; i < rx_ring->desc_num; i++) {
		err = apm_dma_rx_skb_for_slot(apm, &rx_ring->slots[i]);
		if (err) {
			goto error;
		}
		apm_dma_rx_setup_desc(apm, rx_ring, i);
	}
	apm_dma_rx_update_index(apm, rx_ring);

	return 0;

error:
	apm_dma_cleanup(apm);
	return err;
}


/**************************************************
 * Chip ops
 **************************************************/
static int apm_port_loopback(struct apm *apm, int lb_type)
{
	struct iproc_pm_ops *pm_ops = apm->pm_ops;

	if (!pm_ops) {
		dev_err(apm->dev, "(%s) PM does not exist\n", __func__);
		return -EINVAL;
	}

	if (lb_type == APM_LOOPBACK_TYPE_NONE) {
		pm_ops->port_loopback(apm->land_idx, pmLoopbackMac, 0);
		pm_ops->port_loopback(apm->land_idx, pmLoopbackPhy, 0);
	} else if (lb_type == APM_LOOPBACK_TYPE_MAC) {
		pm_ops->port_loopback(apm->land_idx, pmLoopbackMac, 1);
	} else if (lb_type == APM_LOOPBACK_TYPE_PHY) {
		pm_ops->port_loopback(apm->land_idx, pmLoopbackPhy, 1);
	}
	return 0;
}

static int apm_port_mac_address_set(struct apm *apm, u8 *addr)
{
	struct iproc_pm_ops *pm_ops = apm->pm_ops;

	if (!pm_ops) {
		dev_err(apm->dev, "(%s) PM does not exist\n", __func__);
		return -EINVAL;
	}

	return pm_ops->port_mac_addr(apm->land_idx, addr);
}

static int apm_port_speed(struct apm *apm)
{
	struct iproc_pm_ops *pm_ops = apm->pm_ops;

	if (!pm_ops) {
		dev_err(apm->dev, "(%s) PM does not exist\n", __func__);
		return -EINVAL;
	}

	switch (apm->mac_speed) {
		case SPEED_10:
		case SPEED_100:
		case SPEED_1000:
			return pm_ops->port_speed(apm->land_idx, apm->mac_speed);
		default:
			dev_err(apm->dev, "Unsupported speed: %d\n", apm->mac_speed);
	}

	return -EINVAL;
}

static int apm_port_enable(struct apm *apm, int enable)
{
	struct iproc_pm_ops *pm_ops = apm->pm_ops;

	if (!pm_ops) {
		dev_err(apm->dev, "(%s) PM does not exist\n", __func__);
		return -EINVAL;
	}

	return pm_ops->port_enable(apm->land_idx, enable);
}

static int apm_port_stats_clear(struct apm *apm)
{
	struct iproc_pm_ops *pm_ops = apm->pm_ops;

	if (!pm_ops) {
		dev_err(apm->dev, "(%s) PM does not exist\n", __func__);
		return -EINVAL;
	}

	return pm_ops->port_stats_clear(apm->land_idx);
}

static void apm_chip_init(struct apm *apm)
{
	u32 dev_ctl;

	/* 1 interrupt per received frame */
	apm_write(apm, APM_INT_RECV_LAZY, 1 << APM_IRL_FC_SHIFT);

	/* TX QoS mode */
	dev_ctl = apm_read(apm, APM_DEV_CTL);
	if (apm->strict_mode) {
		dev_ctl |= APM_DC_TSM;
	} else {
		dev_ctl &= ~APM_DC_TSM;
	}
	apm_write(apm, APM_DEV_CTL, dev_ctl);

	apm_port_mac_address_set(apm, apm->net_dev->dev_addr);

	/* Enable the pm port */
	apm_port_enable(apm, 1);
}

static void apm_chip_reset(struct apm *apm)
{
	int i;

	if (apm_clk_enabled(apm)) {
		for (i = 0; i < apm->tx_channel; i++) {
			apm_dma_tx_reset(apm, &apm->tx_ring[i]);
		}

		apm_port_loopback (apm, APM_LOOPBACK_TYPE_NONE);
		udelay(1);

		apm_dma_rx_reset(apm, &apm->rx_ring[0]);

		/* TODO: Clear software multicast filter list */
	}

	/* Disable the pm port */
	apm_port_enable(apm, 0);

	/* Clear the MIB */
	apm_port_stats_clear(apm);

	apm->mac_speed = SPEED_1000;
	apm->mac_duplex = DUPLEX_FULL;
	apm_port_speed(apm);

	if (apm->mii_bus) {
		apm->mii_bus->reset(apm->mii_bus);
	}

	netdev_reset_queue(apm->net_dev);
}


static void apm_intrs_on(struct apm *apm)
{
	apm_write(apm, APM_INT_MASK, apm->int_mask);
}

static void apm_intrs_off(struct apm *apm)
{
	apm_write(apm, APM_INT_MASK, 0);
	apm_read(apm, APM_INT_MASK);
}

static bool apm_clk_enabled(struct apm *apm)
{
	if ((apm_idm_read(apm, BCMA_IOCTL) &
	     (BCMA_IOCTL_CLK | BCMA_IOCTL_FGC)) != BCMA_IOCTL_CLK)
		return false;
	if (apm_idm_read(apm, BCMA_RESET_CTL) & BCMA_RESET_CTL_RESET)
		return false;
	return true;
}

static void apm_clk_enable(struct apm *apm, u32 flags)
{
	apm_idm_write(apm, BCMA_IOCTL,
			(BCMA_IOCTL_CLK | BCMA_IOCTL_FGC | flags));
	apm_idm_read(apm, BCMA_IOCTL);

	apm_idm_write(apm, BCMA_RESET_CTL, 0);
	apm_idm_read(apm, BCMA_RESET_CTL);
	udelay(1);

	apm_idm_write(apm, BCMA_IOCTL, (BCMA_IOCTL_CLK | flags));
	apm_idm_read(apm, BCMA_IOCTL);
	udelay(1);
}

static irqreturn_t apm_interrupt(int irq, void *dev_id)
{
	struct apm *apm = netdev_priv(dev_id);
	u32 int_status = apm_read(apm, APM_INT_STATUS);
	int_status &= apm->int_mask;

	if (!int_status)
		return IRQ_NONE;

	int_status &= ~(APM_IS_TX0 | APM_IS_RX);
	if (int_status)
		dev_err(apm->dev, "Unknown IRQs: 0x%08X\n", int_status);

	/* Disable new interrupts until handling existing ones */
	apm_intrs_off(apm);

	napi_schedule(&apm->napi);

	return IRQ_HANDLED;
}

static int apm_poll(struct napi_struct *napi, int weight)
{
	struct apm *apm = container_of(napi, struct apm, napi);
	int handled = 0;

	/* Ack */
	apm_write(apm, APM_INT_STATUS, ~0);

	apm_dma_tx_free(apm, &apm->tx_ring[0]);
	handled += apm_dma_rx_read(apm, &apm->rx_ring[0], weight);

	/* Poll again if more events arrived in the meantime */
	if (apm_read(apm, APM_INT_STATUS) & (APM_IS_TX0 | APM_IS_RX))
		return weight;

	if (handled < weight) {
		napi_complete(napi);
		apm_intrs_on(apm);
	}

	return handled;
}

/**************************************************
 * net_device_ops
 **************************************************/
static int apm_open(struct net_device *net_dev)
{
	struct apm *apm = netdev_priv(net_dev);
	int err = 0;

	apm_chip_reset(apm);

	err = apm_dma_init(apm);
	if (err)
		return err;

	/* Specs say about reclaiming rings here, but we do that in DMA init */
	apm_chip_init(apm);


	err = request_irq(apm->irq0, apm_interrupt, IRQF_SHARED,
			  KBUILD_MODNAME, net_dev);
	if (err < 0) {
		dev_err(apm->dev, "IRQ 0 request error: %d!\n", err);
		apm_dma_cleanup(apm);
		return err;
	}
	err = request_irq(apm->irq1, apm_interrupt, IRQF_SHARED,
			  KBUILD_MODNAME, net_dev);
	if (err < 0) {
		dev_err(apm->dev, "IRQ 1 request error: %d!\n", err);
		apm_dma_cleanup(apm);
		return err;
	}
	err = request_irq(apm->irq2, apm_interrupt, IRQF_SHARED,
			  KBUILD_MODNAME, net_dev);
	if (err < 0) {
		dev_err(apm->dev, "IRQ 2 request error: %d!\n", err);
		apm_dma_cleanup(apm);
		return err;
	}
	napi_enable(&apm->napi);

	phy_start(net_dev->phydev);

	netif_start_queue(net_dev);

	apm_intrs_on(apm);

	return 0;
}

static int apm_stop(struct net_device *net_dev)
{
	struct apm *apm = netdev_priv(net_dev);

	netif_carrier_off(net_dev);

	phy_stop(net_dev->phydev);

	napi_disable(&apm->napi);
	apm_intrs_off(apm);

	free_irq(apm->irq0, net_dev);
	free_irq(apm->irq1, net_dev);
	free_irq(apm->irq2, net_dev);

	apm_chip_reset(apm);
	apm_dma_cleanup(apm);

	return 0;
}

static netdev_tx_t apm_start_xmit(struct sk_buff *skb,
				    struct net_device *net_dev)
{
	struct apm *apm = netdev_priv(net_dev);
	struct apm_dma_ring *tx_ring;
	u32 channel;

	/* Remap the priority to 8 priorities first and transmit the packet
	 * to corresponding tx channel.
	 */
	channel = (skb->priority % 8) / (8 / apm->tx_channel);
	channel = 0; // FIXME, GH2 doesn't support multiple channel

	tx_ring = &apm->tx_ring[channel];
	return apm_dma_tx_add(apm, tx_ring, skb);
}

static int apm_set_mac_address(struct net_device *net_dev, void *addr)
{
	struct apm *apm = netdev_priv(net_dev);
	int ret;

	ret = eth_prepare_mac_addr_change(net_dev, addr);
	if (ret < 0)
		return ret;
	apm_port_mac_address_set(apm, (u8 *)addr);
	eth_commit_mac_addr_change(net_dev, addr);

	return 0;
}

static int apm_ioctl(struct net_device *net_dev, struct ifreq *ifr, int cmd)
{
	if (!netif_running(net_dev))
		return -EINVAL;

	return phy_mii_ioctl(net_dev->phydev, ifr, cmd);
}

static const struct net_device_ops apm_netdev_ops = {
	.ndo_open		= apm_open,
	.ndo_stop		= apm_stop,
	.ndo_start_xmit		= apm_start_xmit,
	.ndo_set_mac_address	= apm_set_mac_address,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl           = apm_ioctl,
};

/**************************************************
 * MII
 **************************************************/
static void apm_adjust_link(struct net_device *net_dev)
{
	struct apm *apm = netdev_priv(net_dev);
	struct phy_device *phy_dev = net_dev->phydev;
	bool update = false;

	if (phy_dev->link) {
		if (phy_dev->speed != apm->mac_speed) {
			apm->mac_speed = phy_dev->speed;
			update = true;
		}

		if (phy_dev->duplex != apm->mac_duplex) {
			apm->mac_duplex = phy_dev->duplex;
			update = true;
		}
	}

	if (update) {
		apm_port_speed(apm);
		phy_print_status(phy_dev);
	}
}

static int apm_enet_probe(struct apm *info)
{
	struct net_device *net_dev;
	struct apm *apm;
	struct phy_device *phy_dev;
	int err;

	/* Allocation and references */
	net_dev = alloc_etherdev(sizeof(*apm));
	if (!net_dev)
		return -ENOMEM;

	net_dev->netdev_ops = &apm_netdev_ops;

	apm = netdev_priv(net_dev);
	memcpy(apm, info, sizeof(*apm));
	apm->net_dev = net_dev;
	net_dev->irq = apm->irq0;  /* irq1, irq2 ?? */
	SET_NETDEV_DEV(net_dev, apm->dev);

	if (!is_valid_ether_addr(apm->mac_addr)) {
		dev_err(apm->dev, "Invalid MAC addr: %pM\n", apm->mac_addr);
		eth_random_addr(apm->mac_addr);
		dev_warn(apm->dev, "Using random MAC: %pM\n", apm->mac_addr);
	}
	ether_addr_copy(net_dev->dev_addr, apm->mac_addr);

	/* This (reset &) enable is not preset in specs or reference driver but
	 * Broadcom does it in arch PCI code when enabling fake PCI device.
	 */
	apm_clk_enable(apm, 0);

	apm_chip_reset(apm);

	err = apm_ethtool_init(net_dev);
	if (err) {
		dev_err(apm->dev, "Init ethtool failed\n");
		goto err_netdev_free;
	}

	err = apm_dma_alloc(apm);
	if (err) {
		dev_err(apm->dev, "Unable to alloc memory for DMA\n");
		goto err_netdev_free;
	}

	apm->int_mask = APM_IS_ERRMASK | APM_IS_RX | APM_IS_TX_MASK;

	netif_napi_add(net_dev, &apm->napi, apm_poll, APM_WEIGHT);

	/* phy init; serdes init is already done in pm.c */
	phy_dev = of_phy_get_and_connect(apm->net_dev, apm->dev->of_node,
						&apm_adjust_link);
	if (!phy_dev) {
		dev_warn(apm->dev, "No phy available in DT");
	}

	net_dev->features = NETIF_F_SG | NETIF_F_IP_CSUM | NETIF_F_IPV6_CSUM;
	net_dev->hw_features = net_dev->features;
	net_dev->vlan_features = net_dev->features;

	err = register_netdev(apm->net_dev);
	if (err) {
		dev_err(apm->dev, "Cannot register net device\n");
		goto err_phy_disconnect;
	}

	netif_carrier_off(net_dev);

	return 0;

err_phy_disconnect:
	phy_disconnect(net_dev->phydev);
err_netdev_free:
	free_netdev(net_dev);

	return err;
}

static void apm_enet_remove(struct apm *apm)
{
	unregister_netdev(apm->net_dev);
	phy_disconnect(apm->net_dev->phydev);
	netif_napi_del(&apm->napi);
	apm_dma_free(apm);
	free_netdev(apm->net_dev);
}

/**************************************************
 * Platform related code
 **************************************************/
static int apm_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	struct apm *apm;
	struct resource *regs;
	const u8 *mac_addr;
	const char *pm_type;
	u32 value;

	apm = devm_kzalloc(&pdev->dev, sizeof(*apm), GFP_KERNEL);
	if (!apm) {
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, apm);

	/* Set the features */
	apm->feature_flags |= APM_FEAT_TX_MASK_SETUP;
	apm->feature_flags |= APM_FEAT_RX_MASK_SETUP;

	apm->dev = &pdev->dev;
	apm->dma_dev = &pdev->dev;

	mac_addr = of_get_mac_address(dn);
	if (mac_addr)
		ether_addr_copy(apm->mac_addr, mac_addr);
	else
		dev_warn(&pdev->dev, "MAC address not present in device tree\n");

	apm->irq0 = platform_get_irq(pdev, 0);
	if (apm->irq0 < 0) {
		dev_err(&pdev->dev, "Unable to obtain IRQ 0\n");
		return apm->irq0;
	}
	apm->irq1 = platform_get_irq(pdev, 1);
	if (apm->irq1 < 0) {
		dev_err(&pdev->dev, "Unable to obtain IRQ 1\n");
		return apm->irq1;
	}
	apm->irq2 = platform_get_irq(pdev, 2);
	if (apm->irq2 < 0) {
		dev_err(&pdev->dev, "Unable to obtain IRQ 2\n");
		return apm->irq2;
	}

	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apm_base");
	if (!regs) {
		dev_err(&pdev->dev, "Unable to obtain base resource\n");
		return -EINVAL;
	}

	apm->plat.base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(apm->plat.base))
		return PTR_ERR(apm->plat.base);

	regs = platform_get_resource_byname(pdev, IORESOURCE_MEM, "idm_base");
	if (!regs) {
		dev_err(&pdev->dev, "Unable to obtain idm resource\n");
		return -EINVAL;
	}

	apm->plat.idm_base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(apm->plat.idm_base)) {
		return PTR_ERR(apm->plat.idm_base);
	}

	apm->plat.wrap_base = get_iproc_wrap_ctrl_base();
	if (IS_ERR(apm->plat.wrap_base)) {
		return PTR_ERR(apm->plat.wrap_base);
	}

	if (of_device_is_compatible(dn, "brcm,xgs-iproc-apm,hx5")) {
		apm->apm_device = XGS_IPROC_APM_HX5;
	} else if (of_device_is_compatible(dn, "brcm,xgs-iproc-apm,hr4")) {
		apm->apm_device = XGS_IPROC_APM_HR4;
	} else {
		return -ENODEV;
	}

	/* Get TX queue number and QoS mode from DTS file */
	if (of_property_read_u32(dn, "tx-channels", &value)) {
		/* Set the default TX channel number */
		apm->tx_channel = 1;
	} else {
		apm->tx_channel = value;
		if (value == 0) {
			apm->tx_channel = 1;
		} else if (value > APM_MAX_TX_RINGS) {
			apm->tx_channel = APM_MAX_TX_RINGS;
		}
	}

	if (of_property_read_u32(dn, "strict-mode", &value)) {
		/* Set the default strict mode */
		apm->strict_mode = true;
	} else {
		apm->strict_mode = true;
		if (value == 0) {
			apm->strict_mode = false;
		}
	}

	/* Get lane index in PM */
	if (of_property_read_u32(dn, "land-idx", &value)) {
		dev_err(&pdev->dev, "Unable to get the PM land index\n");
		return -EINVAL;
	}
	apm->land_idx = value;

	/* Get the PM type */
	if (of_property_read_string(dn, "pm-type", &pm_type)) {
		dev_err(&pdev->dev, "Unable to get the PM type\n");
		return -EINVAL;
	}

	if (!strcmp(pm_type, "pm4x10")) {
		apm->pm_ops = kmalloc(sizeof(struct iproc_pm_ops), GFP_KERNEL);
		apm->pm_ops->port_enable= pm4x10_pm_xlport_port_config;
		apm->pm_ops->port_speed = pm4x10_xlport_speed_set;
		apm->pm_ops->port_loopback = pm4x10_xlport_loopback_set;
		apm->pm_ops->port_mac_addr = pm4x10_xlport_mac_addr_set;
		apm->pm_ops->port_stats = pm4x10_xlport_stats_get;
		apm->pm_ops->port_stats_clear = pm4x10_xlport_mib_reset;
		pm4x10_pm_init(apm);
	} else {
		dev_err(&pdev->dev, "Unknown the PM type - %s\n", pm_type);
		return -EINVAL;
	}

	if (dma_set_mask_and_coherent(apm->dma_dev, DMA_BIT_MASK(36))) {
		dev_err(&pdev->dev, "Unable to set 36-bit dma mask\n");
	}

	return apm_enet_probe(apm);
}

static int apm_remove(struct platform_device *pdev)
{
	struct apm *apm = platform_get_drvdata(pdev);

	kfree(apm->pm_ops);
	apm->pm_ops = NULL;
	pm4x10_pm_deinit(apm->pm_ops);
	apm_enet_remove(apm);

	return 0;
}

static const struct of_device_id apm_of_enet_match[] = {
	{.compatible = "brcm,xgs-iproc-apm",},
	{.compatible = "brcm,xgs-iproc-apm,hx5",},
	{.compatible = "brcm,xgs-iproc-apm,hr4",},
	{},
};
MODULE_DEVICE_TABLE(of, apm_of_enet_match);

static struct platform_driver apm_enet_driver = {
	.driver = {
		.name  = "apm-enet",
		.of_match_table = apm_of_enet_match,
	},
	.probe = apm_probe,
	.remove = apm_remove,
};

module_platform_driver(apm_enet_driver);
MODULE_LICENSE("GPL");
