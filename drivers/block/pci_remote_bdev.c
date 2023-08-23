// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCI Remote Block Device Driver
 *
 * Copyright (C) 2023 Continental Automotive GmbH
 * Copyright 2023 Continental Automotive GmbH
 */

#include <linux/major.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/blk-mq.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/workqueue.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/random.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/pci_regs.h>
#include <linux/hdreg.h>
#include <linux/kthread.h>

#define NUM_DESRIPTORS 512

#define COMMAND_SET_QUEUE BIT(6)
#define COMMAND_WRITE_TO_QUEUE BIT(4)
#define COMMAND_READ_FROM_QUEUE BIT(5)
#define COMMAND_ADD_DRV_ENTRY BIT(7)
#define COMMAND_START BIT(8)

#define PBI_EPF_BIO_F_LAST BIT(0)
#define PBI_EPF_BIO_F_USED BIT(1)

#define PBI_EPF_BIO_F_DIR_READ BIT(2)

struct pci_epf_bio_reg {
	u32 magic;
	u32 command;
	u32 status;
	u32 queue_size; /* number of struct pci_epf_bio_descr */
	u32 drv_offset;
	u32 dev_offset;
	u32 irq_type;
	u32 irq_number;
	u32 flags;
	u32 flags2;
	u64 queue_addr; /* start of struct pci_epf_bio_descr*/
	u64 num_sectors;
	char dev_name[64];
} __packed;

struct pci_epf_bio_descr {
	sector_t s_sector; /* start sector of the request */
	u64 addr; /* where the data is  */
	u64 flags;
	u64 opf;
	u64 tag;
	u32 len; /* bytes to put at addr + s_offset*/
	u32 offset; /* offset from addr */
};

struct pci_bio_driver_ring_entry {
	u16 index;
	u16 flags;
};

struct pci_bio_device_ring_entry {
	u16 index;
	u16 flags;
};

struct pci_bio_driver_ring {
	u16 flags;
	u16 idx;
	struct pci_bio_driver_ring_entry ring[]; /* queue size*/
};

struct pci_bio_device_ring {
	u16 flags;
	u16 idx;
	u32 len;
	struct pci_bio_device_ring_entry ring[]; /* queue size*/
};

#define PCI_REMOTE_BLOCK_MINORS 16
#define PCI_REMOTE_BLKDEV_NAME "remote-bd"
#define DRV_MODULE_NAME "pci-remote-bdev"

#define PCI_RBD_INLINE_SG_CNT 2

enum pci_barno {
	BAR_0,
	BAR_1,
	BAR_2,
	BAR_3,
	BAR_4,
	BAR_5,
};

struct pci_remote_bd_data {
	enum pci_barno reg_bar;
	size_t alignment;
};

struct pci_remote_block_device {
	struct gendisk *gd;
	struct pci_dev *pdev;
	struct blk_mq_tag_set tag_set;
	struct bio_set bset;
	struct pci_epf_bio_reg __iomem *base;
	void __iomem *bar[PCI_STD_NUM_BARS];
	int num_irqs;
	spinlock_t lock;
	struct pci_epf_bio_descr __iomem *descr_ring;
	u32 descr_size;
	u32 drv_offset;
	u32 dev_offset;
	u32 drv_idx;
	u32 dev_idx;
	bool armed;
	struct task_struct *dp_thr;
	const struct blk_mq_queue_data *bd;
};

struct pci_remote_bd_request {
	struct pci_remote_block_device *rdev;
	struct bio *bio;
	u8 status;
	struct page *pg;
	int order;
	int num_bios;
	struct work_struct bio_work;
	struct sg_table sg_table;
	struct scatterlist sg[];
};

static const struct pci_remote_bd_data rbd_default_data = {
	.reg_bar = BAR_0,
	.alignment = SZ_4K,
};

#define REM_BD_PCI_DEVICE(device_id, gen)                                      \
	{                                                                      \
		.vendor = 0x1234, .device = device_id,                         \
		.subvendor = PCI_ANY_ID, .subdevice = PCI_ANY_ID,              \
		.class = (PCI_CLASS_OTHERS << 8), .class_mask = 0xFFFFFFFF,    \
		.driver_data = gen,                                            \
	}

static const struct pci_device_id pci_remote_bd_tbl[] = {
	{
		PCI_DEVICE(0x0, 0xc402),
		.driver_data = (kernel_ulong_t)&rbd_default_data,
	},

	{ 0 }
};

static int major;
static struct workqueue_struct *deferred_bio_add_workqueue;

static struct pci_bio_driver_ring *
get_driver_ring(struct pci_remote_block_device *rdev)
{
	return (struct pci_bio_driver_ring *)((u64)rdev->descr_ring +
					      rdev->drv_offset);
}

static struct pci_bio_device_ring *
get_device_ring(struct pci_remote_block_device *rdev)
{
	return (struct pci_bio_device_ring *)((u64)rdev->descr_ring +
					      rdev->dev_offset);
}

static int get_and_mark_free_descriptor(struct pci_remote_block_device *rdev)
{
	int i;
	struct device *dev = &rdev->pdev->dev;
	spin_lock(&rdev->lock);
	for (i = 0; i < NUM_DESRIPTORS; ++i) {
		struct pci_epf_bio_descr __iomem *de = &rdev->descr_ring[i];
		if (!(de->flags & PBI_EPF_BIO_F_USED)) {
			dev_dbg(dev, "Found free descriptor at idx %i\n", i);
			de->flags |= PBI_EPF_BIO_F_USED;
			spin_unlock(&rdev->lock);
			return i;
		}
	}

	spin_unlock(&rdev->lock);
	dev_err(dev, "%s Unable to find free descriptor\n", __FUNCTION__);
	return -ENOSPC;
}

static blk_status_t pci_rbd_queue_rq(struct blk_mq_hw_ctx *hctx,
				     const struct blk_mq_queue_data *bd)
{
	struct pci_remote_block_device *rdev = hctx->queue->queuedata;
	struct pci_remote_bd_request *rb_req = blk_mq_rq_to_pdu(bd->rq);
	int descr_idx;
	struct device *dev = &rdev->pdev->dev;
	struct pci_epf_bio_descr __iomem *dtu;
	struct pci_bio_driver_ring *drv_ring = get_driver_ring(rdev);
	dma_addr_t dma_addr;

	rb_req->rdev = rdev;
	dev_dbg(dev, "%s called\n", __FUNCTION__);

	descr_idx = get_and_mark_free_descriptor(rdev);
	if (unlikely(descr_idx < 0))
		return BLK_STS_RESOURCE;

	dtu = &rdev->descr_ring[descr_idx];
	rb_req->order = get_order(blk_rq_bytes(bd->rq));
	rb_req->pg = alloc_pages(GFP_KERNEL, rb_req->order);

	if (!rb_req->pg) {
		dev_err(dev, "%s: OOM\n", __FUNCTION__);
		return BLK_STS_RESOURCE;
	}

	dma_addr = dma_map_single(dev, page_address(rb_req->pg),
				  blk_rq_bytes(bd->rq), DMA_FROM_DEVICE);
	dtu->addr = dma_addr;
	dtu->len = blk_rq_bytes(bd->rq);
	dtu->offset = 0;
	dtu->opf = rq_data_dir(bd->rq);
	dtu->s_sector = blk_rq_pos(bd->rq);
	dtu->tag = (u64)rb_req;
	spin_lock(&rdev->lock);
	drv_ring->ring[rdev->drv_idx].index = descr_idx;
	drv_ring->idx = rdev->drv_idx = (rdev->drv_idx + 1) % NUM_DESRIPTORS;
	spin_unlock(&rdev->lock);
	dev_dbg(dev, "(DIR: %s): Adding desc %i (%i). sector: 0x%llX, len: 0x%x, offset: 0x%x\n",
		 (rq_data_dir(bd->rq) == WRITE) ? "WRITE" : "READ", descr_idx, rdev->drv_idx,
		 dtu->s_sector, dtu->len, dtu->offset);

	blk_mq_start_request(bd->rq);
	wake_up_process(rdev->dp_thr);
	return BLK_STS_OK;
}

static void pci_rbd_end_rq(struct request *rq)
{
	struct pci_remote_bd_request *rb_req = blk_mq_rq_to_pdu(rq);
	
	__free_pages(rb_req->pg, rb_req->order);
	dev_dbg(&rb_req->rdev->pdev->dev, "%s called\n", __FUNCTION__);
	blk_mq_end_request(rq, BLK_STS_OK);
}

static enum blk_eh_timer_return pci_rbd_timeout_rq(struct request *rq, bool res)
{
	struct pci_remote_bd_request *rb_req = blk_mq_rq_to_pdu(rq);
	dev_dbg(&rb_req->rdev->pdev->dev, "%s called\n", __FUNCTION__);
	blk_mq_complete_request(rq);
	return BLK_EH_DONE;
}

static const struct blk_mq_ops pci_rbd_mq_ops = { .queue_rq = pci_rbd_queue_rq,
						  .complete = pci_rbd_end_rq,
						  .timeout =
							  pci_rbd_timeout_rq };

static int pci_rbd_open(struct block_device *bdev, fmode_t mode)
{
	struct pci_remote_block_device *rdev = bdev->bd_disk->private_data;
	dev_dbg(&rdev->pdev->dev, "%s called\n", __FUNCTION__);
	return 0;
}

static void pci_rbd_release(struct gendisk *disk, fmode_t mode)
{
}

static int pci_rbd_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct pci_remote_block_device *rdev = bdev->bd_disk->private_data;
	dev_dbg(&rdev->pdev->dev, "%s called\n", __FUNCTION__);
	geo->heads = 4;
	geo->sectors = 16;
	geo->cylinders =
		get_capacity(bdev->bd_disk) / (geo->heads * geo->sectors);
	return 0;
}

static int pci_rbd_ioctl(struct block_device *bdev, fmode_t mode,
			 unsigned int cmd, unsigned long arg)
{
	/* int dir = _IOC_DIR(cmd); */
	/* int tp = _IOC_TYPE(cmd); */
	/* int nr = _IOC_NR(cmd); */
	/* int sz = _IOC_SIZE(cmd); */

	/* printk(KERN_DEBUG */
	/*        "-------> IOCTL received %d. R/WR: 0x%x, TYPE: 0x%x, NR: 0x%x, SIZE: 0x%x\n", */
	/*        cmd, dir, tp, nr, sz); */
	return -EINVAL;
}

#ifdef CONFIG_COMPAT
static int pci_rbd_compat_ioctl(struct block_device *bdev, fmode_t mode,
				unsigned int cmd, unsigned long arg)
{
	return pci_rbd_ioctl(bdev, mode, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct block_device_operations pci_rbd_ops = {
	.open = pci_rbd_open,
	.release = pci_rbd_release,
	.getgeo = pci_rbd_getgeo,
	.owner = THIS_MODULE,
	.ioctl = pci_rbd_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pci_rbd_compat_ioctl,
#endif
};

static irqreturn_t pci_rbd_irqhandler(int irq, void *dev_id)
{
	struct pci_remote_block_device *rdev = dev_id;
	dev_err(&rdev->pdev->dev, "%s\n", __FUNCTION__);
	return IRQ_HANDLED;
}

static void pci_rbd_clear_descriptor(struct pci_remote_block_device *rdev, struct pci_epf_bio_descr *descr)
{
    spin_lock(&rdev->lock);
    memset(descr, 0, sizeof(*descr));
    spin_unlock(&rdev->lock);
}

static int pci_remote_bd_dispatch(void *cookie)
{
	struct pci_remote_block_device *rdev =
		(struct pci_remote_block_device *)cookie;
	struct device *dev = &rdev->pdev->dev;
	struct pci_bio_device_ring *dev_ring = get_device_ring(rdev);
	struct req_iterator iter;
	struct bio_vec bv;

	allow_signal(SIGINT);
	allow_signal(SIGTERM);
	allow_signal(SIGKILL);
	allow_signal(SIGUSR1);

	while (!kthread_should_stop()) {
		while (dev_ring->idx != rdev->dev_idx) {
			int descr_idx = dev_ring->ring[rdev->dev_idx].index;
			struct pci_epf_bio_descr *desc =
				&rdev->descr_ring[descr_idx];
			struct pci_remote_bd_request *rb_req =
				(struct pci_remote_bd_request *)desc->tag;
			struct request *rq = blk_mq_rq_from_pdu(rb_req);
			void *buf = kmap(rb_req->pg);
			int i = 0;

			dev_dbg(dev,"Dev Ring: Internal: %i, Device: %i. sec: 0x%llX, addr: 0x%llX, opf: %s, len: 0x%x, offset: 0x%x\n",
				rdev->dev_idx, dev_ring->idx, desc->s_sector,
				desc->addr,
				(desc->opf == WRITE) ? "WRITE" : "READ",
				desc->len, desc->offset);
			/* print_hex_dump(KERN_WARNING, */
			/* 	       "raw: ", DUMP_PREFIX_OFFSET, 16, 16, buf, */
			/* 	       0x50, true); */
			dev_dbg(dev, "Digest Req: sec: 0x%llX, len: 0x%x\n", desc->s_sector, desc->len);
			rq_for_each_segment (bv, rq, iter) {
				memcpy_to_bvec(&bv, buf);
				dev_dbg(dev,
					"%i: bv.len = 0x%x, bv.offset = 0x%x, page: 0x%llX. Buf: 0x%llX\n",
					i, bv.bv_len, bv.bv_offset, page_address(bv.bv_page), buf);
				buf += bv.bv_len;
				i++;
			}

			kunmap(rb_req->pg);
			dma_unmap_single(dev, desc->addr, desc->len,
					 DMA_FROM_DEVICE);
			pci_rbd_clear_descriptor(rdev, desc);
			rdev->dev_idx = (rdev->dev_idx + 1) % NUM_DESRIPTORS;
			blk_mq_complete_request(rq);
		}
		usleep_range(500, 1000);
	}

	return 0;
}

static int pci_remote_bd_probe(struct pci_dev *pdev,
			       const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	int err;
	enum pci_barno bar;
	int i;
	size_t offset = 0;
	enum pci_barno def_reg_bar = BAR_0;
	dma_addr_t orig_phys_addr;
	dma_addr_t phys_addr;
	void __iomem *base;
	unsigned long timeout = 0;
	/* struct pci_epf_bio_descr __iomem * or ; */
	size_t alignment = SZ_4K;
	struct pci_remote_block_device *rdev =
		devm_kzalloc(dev, sizeof(*rdev), GFP_KERNEL);
	if (!rdev)
		return -ENOMEM;

	rdev->dp_thr =
		kthread_create(pci_remote_bd_dispatch, rdev, "pci_rbd_thread");
	if (IS_ERR(rdev->dp_thr)) {
		dev_err(dev, "Cannot create kernel thread\n");
		err = PTR_ERR(rdev->dp_thr);
		goto out_free_dev;
	}

	rdev->descr_size =
		ALIGN(NUM_DESRIPTORS * sizeof(*rdev->descr_ring), sizeof(u64)) +
		ALIGN(sizeof(struct pci_bio_driver_ring) +
			      (NUM_DESRIPTORS *
			       sizeof(struct pci_bio_driver_ring_entry)),
		      sizeof(u64)) +
		ALIGN(sizeof(struct pci_bio_device_ring) +
			      (NUM_DESRIPTORS *
			       sizeof(struct pci_bio_device_ring_entry)),
		      sizeof(u64)) +
		alignment;
	rdev->descr_ring = devm_kzalloc(dev, rdev->descr_size, GFP_KERNEL);

	if (!rdev->descr_ring) {
		dev_err(dev, "Cannot alloc %i Bytes for descriptor\n",
			rdev->descr_size);
		err = -ENOMEM;
		goto out_free_dev;
	}

	rdev->pdev = pdev;
	if ((dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(48)) != 0) &&
	    dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)) != 0) {
		err = -ENODEV;
		dev_err(dev, "Cannot set DMA mask\n");
		goto out_free_descr;
	}

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Cannot enable PCI device\n");
		goto out_free_descr;
	}

	err = pci_request_regions(pdev, DRV_MODULE_NAME);
	if (err) {
		dev_err(dev, "Cannot obtain PCI resources\n");
		goto err_disable_pdev;
	}

	pci_set_master(pdev);

	rdev->num_irqs = pci_alloc_irq_vectors(pdev, 1, 32, PCI_IRQ_MSI);
	if (rdev->num_irqs < 0)
		dev_err(dev, "Failed to get MSI interrupts\n");

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (pci_resource_flags(pdev, bar) & IORESOURCE_MEM) {
			base = pci_ioremap_bar(pdev, bar);
			if (!base) {
				dev_err(dev, "Failed to read BAR%d\n", bar);
				WARN_ON(bar == def_reg_bar);
			}
			rdev->bar[bar] = base;
		}
	}

	rdev->base = rdev->bar[def_reg_bar];

	if (!rdev->base) {
		err = -ENOMEM;
		dev_err(dev, "Cannot perform PCI communictaion without BAR%d\n",
			def_reg_bar);
		goto err_iounmap;
	}

	/* dev_err(dev, "-----> READING EP MAGIC at (0x%llX) offset 0  = 0x%x\n", */
	/* 	(u64)rdev->base, rdev->base->magic); */
	/* dev_err(dev, "-----> READING EP SECTOR_COUNT at (0x%llX ) = 0x%llX\n", */
	/* 	(u64)rdev->base, rdev->base->num_sectors); */
	/* dev_err(dev, "-----> READING EP DEVICE PATH at (0x%llX ) = %s\n", */
	/* 	(u64)rdev->base, rdev->base->dev_name); */

	orig_phys_addr =
		dma_map_single(dev, rdev->descr_ring,
			       rdev->descr_size + alignment, DMA_FROM_DEVICE);
	if (dma_mapping_error(dev, orig_phys_addr)) {
		dev_err(dev, "failed to map descriptor address\n");
		err = -ENODEV;
		goto err_iounmap;
	}

	if (!IS_ALIGNED(orig_phys_addr, alignment)) {
		phys_addr = PTR_ALIGN(orig_phys_addr, alignment);
		offset = phys_addr - orig_phys_addr;
		rdev->descr_ring = (struct pci_epf_bio_descr __iomem
					    *)((u64)rdev->descr_ring + offset);
	} else {
		phys_addr = orig_phys_addr;
	}

	rdev->drv_offset =
		ALIGN(NUM_DESRIPTORS * sizeof(*rdev->descr_ring), sizeof(u64));
	rdev->dev_offset =
		rdev->drv_offset + ALIGN(sizeof(struct pci_bio_driver_ring) +
						 (NUM_DESRIPTORS * sizeof(u16)),
					 sizeof(u64));

	rdev->base->drv_offset = rdev->drv_offset;
	rdev->base->dev_offset = rdev->dev_offset;

	dev_info(
		dev,
		" Setting queue addr @ 0x%llX -> 0x%llx. Queue Size for %i descriptors = %i. Offset = %i\n",
		rdev->descr_ring, phys_addr, NUM_DESRIPTORS, rdev->descr_size,
		offset);

	rdev->base->queue_addr = phys_addr;
	rdev->base->queue_size = rdev->descr_size;
	__iomb();
	rdev->base->command = COMMAND_SET_QUEUE;

	/* timeout = jiffies + 500; */
	/* while (!time_after(jiffies, timeout)) { */
	/* } */

	/* rdev->base->command = COMMAND_WRITE_TO_QUEUE; */
	/* dev_info(dev, "-----> First Timeout \n"); */
	/* timeout = jiffies + 500; */
	/* while (!time_after(jiffies, timeout)) { */
	/* } */
	/* dev_info(dev, "-----> Second Timeout \n"); */
	for (i = 0; i < rdev->num_irqs; i++) {
		err = devm_request_irq(dev, pci_irq_vector(pdev, i),
				       pci_rbd_irqhandler, IRQF_SHARED,
				       "RBD IRQ", rdev);
		if (err) {
			dev_err(dev, "Unable to register irq %i\n", i);
			break;
		}
	}
	/* for (i = 0; i < 16; ++i) { */
	/* 	or = &rdev->descr_ring[i]; */
	/* 	dev_info( */
	/* 		dev, */
	/* 		"%i: Lets look what was written: addr: 0x%llX, s_sector: 0x%llX, s_offset = 0x%x, num_sectors = 0x%x", */
	/* 		i, or->addr, or->s_sector, or->offset, or->len); */
	/* } */

	timeout = jiffies + 10;
	while (!time_after(jiffies, timeout)) {
	}

	rdev->base->command = COMMAND_START;

	timeout = jiffies + 10;
	while (!time_after(jiffies, timeout)) {
	}

	deferred_bio_add_workqueue =
		alloc_workqueue("pci_rbd_bio_add", WQ_UNBOUND, 1);
	if (!deferred_bio_add_workqueue) {
		err = -ENOMEM;
		goto err_free_irq;
	}

	pci_set_drvdata(pdev, rdev);
	major = register_blkdev(0, PCI_REMOTE_BLKDEV_NAME);
	if (major < 0) {
		dev_err(dev, "unable to register remote block device\n");
		err = -EBUSY;
		goto out_workqueue;
	}

	if (bioset_init(&rdev->bset, 2, 0, 0)) {
		dev_err(dev, "Could not init bioset\n");
		err = -ENODEV;
		goto out_workqueue;
	}

	spin_lock_init(&rdev->lock);
	rdev->tag_set.ops = &pci_rbd_mq_ops;
	rdev->tag_set.queue_depth = 128;
	rdev->tag_set.numa_node = NUMA_NO_NODE;
	rdev->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
	rdev->tag_set.nr_hw_queues = 1; //num_present_cpus();
	rdev->tag_set.cmd_size =
		sizeof(struct pci_remote_bd_request) +
		sizeof(struct scatterlist) * PCI_RBD_INLINE_SG_CNT;
	rdev->tag_set.driver_data = rdev;

	err = blk_mq_alloc_tag_set(&rdev->tag_set);
	if (err)
		goto out_workqueue;

	rdev->gd = blk_mq_alloc_disk(&rdev->tag_set, rdev);
	if (IS_ERR(rdev->gd)) {
		err = -ENODEV;
		goto out_tag_set;
	}

	rdev->gd->major = major;
	rdev->gd->minors = PCI_REMOTE_BLOCK_MINORS;
	rdev->gd->first_minor = 1;
	rdev->gd->fops = &pci_rbd_ops;
	rdev->gd->private_data = rdev;
	rdev->gd->queue->queuedata = rdev;

	snprintf(rdev->gd->disk_name, 32, "pci_rbd-%s",
		 &rdev->base->dev_name[5]);
	set_capacity(rdev->gd, rdev->base->num_sectors);
	device_add_disk(dev, rdev->gd, NULL);
	wake_up_process(rdev->dp_thr);
	return 0;
out_tag_set:
	blk_mq_free_tag_set(&rdev->tag_set);
out_workqueue:
	destroy_workqueue(deferred_bio_add_workqueue);
err_free_irq:
	for (i = 0; i < rdev->num_irqs; i++) {
		devm_free_irq(dev, pci_irq_vector(pdev, i), rdev);
	}
	rdev->num_irqs = 0;

err_iounmap:
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (rdev->bar[bar])
			pci_iounmap(pdev, rdev->bar[bar]);
	}

	pci_free_irq_vectors(pdev);
	pci_release_regions(pdev);
err_disable_pdev:
	pci_disable_device(pdev);
out_free_descr:
	devm_kfree(dev, rdev->descr_ring);
out_free_dev:
	devm_kfree(dev, rdev);
	return err;
}

static void pci_remote_bd_remove(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_remote_block_device *rdev = pci_get_drvdata(pdev);

	del_gendisk(rdev->gd);
	blk_cleanup_disk(rdev->gd);
	blk_mq_free_tag_set(&rdev->tag_set);
	devm_kfree(dev, rdev);
	unregister_blkdev(major, PCI_REMOTE_BLKDEV_NAME);
	destroy_workqueue(deferred_bio_add_workqueue);
}

MODULE_DEVICE_TABLE(pci, pci_remote_bd_tbl);

static struct pci_driver pci_remote_bdev_driver = {
	.name = DRV_MODULE_NAME,
	.id_table = pci_remote_bd_tbl,
	.probe = pci_remote_bd_probe,
	.remove = pci_remote_bd_remove,
	.sriov_configure = pci_sriov_configure_simple,
};

module_pci_driver(pci_remote_bdev_driver);

MODULE_AUTHOR("Wadim Mueller <wadim.mueller@continental.com>");
MODULE_DESCRIPTION("Remote PCI Endpoint Block Device driver");
MODULE_LICENSE("GPL v2");
