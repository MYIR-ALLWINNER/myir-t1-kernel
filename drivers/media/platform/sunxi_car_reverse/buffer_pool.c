/*
 * Fast car reverse image preview module
 *
 * Copyright (C) 2015-2018 AllwinnerTech, Inc.
 *
 * Contacts:
 * Zeng.Yajian <zengyajian@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/version.h>
#include <linux/slab.h>

#include <linux/kernel.h>
#include <linux/ion.h>
#include <uapi/linux/ion.h>
#include <linux/spinlock.h>
#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>

#include "car_reverse.h"

#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
// #include "../../../video/fbdev/sunxi/disp2/disp/dev_disp.h"
#include "../../../char/sunxi_g2d/g2d_rcq/g2d_top.h"
extern struct device *g_g2d_drv;
// extern struct disp_drv_info g_disp_drv;
#endif

#define SUNXI_MEM

#ifdef SUNXI_MEM
#include <linux/ion.h>
#include <uapi/linux/ion.h>
#include <linux/dma-mapping.h>

struct ion_private {
	char *client_name;
	struct ion_client *client;
	struct list_head handle_list;
};

struct ion_memory {
	struct buffer_node node;
	struct list_head list;
	int alloc_size;
	void *vir_address;
	unsigned long phy_address;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
	struct dma_buf_attachment *attachment_de;
	struct sg_table *sgt_de;
	unsigned long phy_address_de;
#endif
};

struct buffer_node *__buffer_node_alloc(struct device *dev, int size, int flags)
{
	struct ion_memory *mem = NULL;
	struct ion_buffer *ion_buf = NULL;
	unsigned int heap_id_mask;
	int alloc_size = PAGE_ALIGN(size);

	if (!size)
		return NULL;

	mem = kzalloc(sizeof(struct ion_memory), GFP_KERNEL);
	if (!mem) {
		logerror("%s: alloc failed\n", __func__);
		goto ion_private_err;
	}

	heap_id_mask = 1 << ION_HEAP_TYPE_SYSTEM;
	mem->dmabuf = ion_alloc(alloc_size, heap_id_mask, 0);
	if (IS_ERR_OR_NULL(mem->dmabuf)) {
		logerror("%s: ion alloc failed\n", __func__);
		goto ion_alloc_err;
	}

	mem->vir_address = ion_heap_map_kernel(NULL, mem->dmabuf->priv);
	if (IS_ERR_OR_NULL(mem->vir_address)) {
		logerror("%s: ion_map_kernel failed\n", __func__);
		goto ion_map_err;
	}

	ion_buf = (struct ion_buffer *)mem->dmabuf->priv;
	ion_buf->vaddr = mem->vir_address;

	mem->attachment = dma_buf_attach(mem->dmabuf, dev);
	if (IS_ERR_OR_NULL(mem->attachment)) {
		logerror("%s: dma_buf_attach failed\n", __func__);
		goto ion_buf_umap_kernel;
	}

	mem->sgt = dma_buf_map_attachment(mem->attachment, DMA_FROM_DEVICE);
	if (IS_ERR_OR_NULL(mem->sgt)) {
		logerror("%s: dma_buf_map_attachment failed\n", __func__);
		goto ion_buf_detach;
	}

	mem->phy_address = sg_dma_address(mem->sgt->sgl);

#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
	mem->attachment_de = dma_buf_attach(mem->dmabuf, g_g2d_drv);
	if (IS_ERR_OR_NULL(mem->attachment_de)) {
		logerror("%s:tvd dma_buf_attach failed\n", __func__);
		goto ion_buf_detach;
	}

	mem->sgt_de = dma_buf_map_attachment(mem->attachment_de, DMA_FROM_DEVICE);
	if (IS_ERR_OR_NULL(mem->sgt_de)) {
		logerror("%s:tvd dma_buf_map_attachment failed\n", __func__);
		goto ion_buf_detach_de;
	}

	mem->phy_address_de = sg_dma_address(mem->sgt_de->sgl);

	mem->node.phy_address_de = (void *)mem->phy_address_de;
#endif

	mem->alloc_size = alloc_size;

	mem->node.vir_address = mem->vir_address;
	mem->node.phy_address = (void *)mem->phy_address;
	mem->node.size = alloc_size;

	return &mem->node;

#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
ion_buf_detach_de:
	dma_buf_detach(mem->dmabuf, mem->attachment_de);
#endif
ion_buf_detach:
	dma_buf_detach(mem->dmabuf, mem->attachment);
ion_buf_umap_kernel:
	ion_heap_unmap_kernel(NULL, mem->dmabuf->priv);
ion_map_err:
	dma_buf_put(mem->dmabuf);
ion_alloc_err:
	kfree(mem);
ion_private_err:
	return NULL;
}

void __buffer_node_free(struct device *dev, struct buffer_node *node)
{
	struct ion_memory *mem = container_of(node, struct ion_memory, node);

#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
	dma_buf_unmap_attachment(mem->attachment_de, mem->sgt_de, DMA_FROM_DEVICE);
#endif
	dma_buf_unmap_attachment(mem->attachment, mem->sgt, DMA_FROM_DEVICE);
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
	dma_buf_detach(mem->dmabuf, mem->attachment_de);
#endif
	dma_buf_detach(mem->dmabuf, mem->attachment);
	ion_heap_unmap_kernel(NULL, mem->dmabuf->priv);
	dma_buf_put(mem->dmabuf);
	kfree(mem);
}

#else

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 1, 0)
#include <linux/ion_sunxi.h>
struct buffer_node *__buffer_node_alloc(struct device *dev, int size, int flags)
{
	struct buffer_node *node = NULL;
	unsigned int phyaddr;

	node = kzalloc(sizeof(struct buffer_node), GFP_KERNEL);
	if (!node) {
		logerror("%s: alloc failed\n", __func__);
		return NULL;
	}

	node->size = PAGE_ALIGN(size);
	node->vir_address = sunxi_buf_alloc(node->size, &phyaddr);
	node->phy_address = (void *)phyaddr;
	if (!node->vir_address) {
		logerror("%s: sunxi_buf_alloc failed!\n", __func__);
		kfree(node);
		return NULL;
	}
	return node;
}

void __buffer_node_free(struct device *dev, struct buffer_node *node)
{
	if (node && node->vir_address) {
		sunxi_buf_free(node->vir_address,
			(unsigned int)node->phy_address, node->size);
		kfree(node);
	}
}
#else
struct buffer_node *__buffer_node_alloc(struct device *dev, int size, int flags)
{
	struct buffer_node *node = NULL;
	unsigned long phyaddr;

	node = kzalloc(sizeof(struct buffer_node), GFP_KERNEL);
	if (!node) {
		logerror("%s: alloc failed\n", __func__);
		return NULL;
	}
	node->size = PAGE_ALIGN(size);
	node->vir_address = dma_alloc_coherent(dev, node->size,
				(dma_addr_t *)&phyaddr, GFP_KERNEL);
	node->phy_address = (void *)phyaddr;
	if (!node->vir_address) {
		logerror("%s: sunxi_buf_alloc failed!\n", __func__);
		kfree(node);
		return NULL;
	}
	return node;
}

void __buffer_node_free(struct device *dev, struct buffer_node *node, int flags)
{
	if (node && node->vir_address && node->phy_address) {
		dma_free_coherent(dev, node->size,
			node->vir_address, (dma_addr_t)node->phy_address);
		kfree(node);
	}
}

#endif
#endif

static struct buffer_node *buffer_pool_dequeue(struct buffer_pool *bp)
{
	struct buffer_node *retval = NULL;

	spin_lock(&bp->lock);
	if (!list_empty(&bp->head)) {
		retval = list_entry(bp->head.next, struct buffer_node, list);
		list_del(&retval->list);
	}
	spin_unlock(&bp->lock);
	return retval;
}
static void
buffer_pool_queue(struct buffer_pool *bp, struct buffer_node *node)
{
	spin_lock(&bp->lock);
	list_add_tail(&node->list, &bp->head);
	spin_unlock(&bp->lock);
}

struct buffer_pool *
alloc_buffer_pool(struct device *dev, int depth, int buf_size)
{
	int i;
	struct buffer_pool *bp;
	struct buffer_node *node;

	bp = kzalloc(sizeof(struct buffer_pool), GFP_KERNEL);
	if (!bp) {
		logerror("%s: alloc failed\n", __func__);
		goto _out;
	}

	bp->depth = depth;
	bp->pool = kzalloc(sizeof(struct buffer_pool *)*depth, GFP_KERNEL);
	if (!bp->pool) {
		logerror("%s: alloc failed\n", __func__);
		kfree(bp);
		bp = NULL;
		goto _out;
	}
	spin_lock_init(&bp->lock);

	bp->queue_buffer   = buffer_pool_queue;
	bp->dequeue_buffer = buffer_pool_dequeue;

	/* alloc memory for buffer node */
	INIT_LIST_HEAD(&bp->head);
	for (i = 0; i < depth; i++) {
		node = __buffer_node_alloc(dev, buf_size, 1);
		if (node) {
			list_add(&node->list, &bp->head);
			bp->pool[i] = node;
#ifdef CONFIG_VIDEO_SUNXI_TVD_SPECIAL
			logdebug("%s:[%d] vaddr:%px, paddr:%px, paddr_de:%px\n", __func__, i,
				node->vir_address, node->phy_address, node->phy_address_de);
#else
			logdebug("%s:[%d] vaddr:%px, paddr:%px\n", __func__, i,
				node->vir_address, node->phy_address);
#endif
		}
	}

_out:
	return bp;
}

void free_buffer_pool(struct device *dev, struct buffer_pool *bp)
{
	struct buffer_node *node;

	spin_lock(&bp->lock);
	while (!list_empty(&bp->head)) {
		node = list_entry(bp->head.next, struct buffer_node, list);
		list_del(&node->list);
		logdebug("%s: free %p\n", __func__, node->phy_address);
		__buffer_node_free(dev, node);
	}

	spin_unlock(&bp->lock);

	kfree(bp->pool);
	kfree(bp);
}

void rest_buffer_pool(struct device *dev, struct buffer_pool *bp)
{
	int i;
	struct buffer_node *node;

	INIT_LIST_HEAD(&bp->head);
	for (i = 0; i < bp->depth; i++) {
		node = bp->pool[i];
		if (node)
			list_add(&node->list, &bp->head);
	}
}

void dump_buffer_pool(struct device *dev, struct buffer_pool *bp)
{
	int i = 0;
	struct buffer_node *node;

	spin_lock(&bp->lock);
	list_for_each_entry(node, &bp->head, list) {
		logdebug("%s: [%d] %p\n", __func__, i++, node->phy_address);
	}
	spin_unlock(&bp->lock);
}
