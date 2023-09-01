// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCI Remote Block Device Driver
 *
 * Copyright (C) 2023 Continental Automotive GmbH
 * Wadim Mueller <wadim.mueller@continental-corporation.com>
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


#define RBD_MAGIC 0x636f6e74
#define NUM_DESRIPTORS 4096

#define RBD_STATUS_SUCCESS	                BIT(0)
#define RD_STATUS_TIMEOUT_COUNT                 (1e6)
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
	u32 num_desc;
	u64 queue_addr; /* start of struct pci_epf_bio_descr*/
	u64 num_sectors;
	char dev_name[64];
} __packed;

struct pci_epf_bio_descr {
	sector_t s_sector; /* start sector of the request */
	u64 addr; /* where the data is  */
	u64 opf;
	u64 tag;
	u32 len; /* bytes to put at addr + s_offset*/
	u32 offset; /* offset from addr */
        u32 status;
    	u32 flags;
        u32 res0;
        u32 res1;
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


#define DRV_MODULE_NAME "pci-remote-disk"
#define PCI_RBD_INLINE_SG_CNT 2

enum pci_barno {
	BAR_0,
	BAR_1,
	BAR_2,
	BAR_3,
	BAR_4,
	BAR_5,
};


struct pci_remote_disk {
	struct gendisk *gd;
	struct pci_dev *pdev;
	struct blk_mq_tag_set tag_set;
	struct pci_epf_bio_reg __iomem *base;
	void __iomem *bar[PCI_STD_NUM_BARS];
	int num_irqs;
	spinlock_t lock;
	struct pci_epf_bio_descr __iomem *descr_ring;
        struct page *desc_page;
        u32 dp_order;
	u32 descr_size;
	u32 drv_offset;
	u32 dev_offset;
	u32 drv_idx;
	u32 dev_idx;
	struct task_struct *dp_thr;
	const struct blk_mq_queue_data *bd;
};

struct pci_remote_disk_request {
	struct pci_remote_disk *rdev;
	struct bio *bio;
	u8 status;
	struct page *pg;
	int order;
	int num_bios;
        int descr_idx;
        struct pci_epf_bio_descr *descr;
	struct work_struct bio_work;
	struct sg_table sg_table;
	struct scatterlist sg[];
};


static const struct pci_device_id pci_remote_bd_tbl[] = {
	{
		PCI_DEVICE(0x0, 0xc402),
	},
	{ 0 }
};

static struct pci_bio_driver_ring * pci_rd_get_driver_ring(struct pci_remote_disk *rdev)
{
	return (struct pci_bio_driver_ring *)((u64)rdev->descr_ring +
					      rdev->drv_offset);
}

static struct pci_bio_device_ring * pci_rd_get_device_ring(struct pci_remote_disk *rdev)
{
	return (struct pci_bio_device_ring *)((u64)rdev->descr_ring +
					      rdev->dev_offset);
}

static int pci_rd_get_and_mark_free_descriptor(struct pci_remote_disk *rdev)
{
	int i;
	struct device *dev = &rdev->pdev->dev;
	spin_lock(&rdev->lock);
	for (i = 0; i < NUM_DESRIPTORS; ++i) {
		struct pci_epf_bio_descr __iomem *de = &rdev->descr_ring[i];
		u32 flags = READ_ONCE(de->flags);
		if (!(flags & PBI_EPF_BIO_F_USED)) {
			dev_dbg(dev, "Found free descriptor at idx %i\n", i);
			WRITE_ONCE(de->flags, flags | PBI_EPF_BIO_F_USED);
			spin_unlock(&rdev->lock);
			return i;
		}
	}

	spin_unlock(&rdev->lock);
	dev_err(dev, "%s Unable to find free descriptor\n", __FUNCTION__);
	return -ENOSPC;
}

static bool is_valid_request(unsigned int op)
{
    return (op == REQ_OP_READ) || (op == REQ_OP_WRITE);
}

static blk_status_t pci_rd_queue_rq(struct blk_mq_hw_ctx *hctx,
				     const struct blk_mq_queue_data *bd)
{
        struct req_iterator iter;
	struct bio_vec bv;

	struct pci_remote_disk *rdev = hctx->queue->queuedata;
	struct pci_remote_disk_request *rb_req = blk_mq_rq_to_pdu(bd->rq);
	int descr_idx;
	struct device *dev = &rdev->pdev->dev;
	struct pci_epf_bio_descr __iomem *dtu;
	struct pci_bio_driver_ring __iomem *drv_ring = pci_rd_get_driver_ring(rdev);
	dma_addr_t dma_addr;
	char *buf;
	int err;
	int i = 0;

	rb_req->rdev = rdev;
	if (!is_valid_request(req_op(bd->rq))) {
	    dev_err(dev, "Unsupported Request: %i\n", req_op(bd->rq));
	    return BLK_STS_NOTSUPP;
	}

	descr_idx = pci_rd_get_and_mark_free_descriptor(rdev);
	if (unlikely(descr_idx < 0))
	    return BLK_STS_AGAIN;

	dtu = &rdev->descr_ring[descr_idx];
	rb_req->order = get_order(blk_rq_bytes(bd->rq));
	rb_req->pg = alloc_pages(GFP_ATOMIC | GFP_DMA, rb_req->order);
	if (unlikely(!rb_req->pg)) {
	    dev_err(dev, "cannot alloc %i page(s)\n", (1 << rb_req->order));
	    err = BLK_STS_AGAIN;
	    goto free_descr;
	}

	rb_req->descr = dtu;
	rb_req->descr_idx = descr_idx;
	
	buf = page_address(rb_req->pg);
	dma_addr = dma_map_single(dev, buf, blk_rq_bytes(bd->rq), rq_dma_dir(bd->rq));
	if (dma_mapping_error(dev, dma_addr)) {
		dev_err(dev, "failed to map page for descriptor\n");
		err = BLK_STS_AGAIN;
		goto free_pages;
	}
	
	dtu->addr = dma_addr;
	dtu->len = blk_rq_bytes(bd->rq);
	dtu->offset = 0;
	dtu->opf = rq_data_dir(bd->rq);
	if (dtu->opf == WRITE) {
	    rq_for_each_segment (bv, bd->rq, iter) {
		memcpy_from_bvec(buf, &bv);
		dev_dbg(dev,"WRITE: %i: bv.len = 0x%x, bv.offset = 0x%x, page: 0x%llX. Buf: 0x%llX. sec: 0x%llX, len: 0x%x\n",
					 i, bv.bv_len, bv.bv_offset, page_address(bv.bv_page), buf, blk_rq_pos(bd->rq), blk_rq_bytes(bd->rq));
		buf += bv.bv_len;
		i++;
	    }
	}
	dtu->s_sector = blk_rq_pos(bd->rq);
	dtu->tag = (u64)rb_req;
	spin_lock(&rdev->lock);
	iowrite16(descr_idx, &drv_ring->ring[rdev->drv_idx].index);
	rdev->drv_idx = (rdev->drv_idx + 1) % NUM_DESRIPTORS;
	smp_wmb();
	iowrite16(rdev->drv_idx, &drv_ring->idx);
	spin_unlock(&rdev->lock);
	dev_dbg(dev, "(DIR: %s): Adding desc %i (%i). sector: 0x%llX, len: 0x%x, offset: 0x%x\n",
		 (rq_data_dir(bd->rq) == WRITE) ? "WRITE" : "READ", descr_idx, rdev->drv_idx,
		 dtu->s_sector, dtu->len, dtu->offset);

	blk_mq_start_request(bd->rq);
	wake_up_process(rdev->dp_thr);
	return BLK_STS_OK;
free_pages:
	__free_pages(rb_req->pg, rb_req->order);
free_descr:
	memset(dtu, 0, sizeof(*dtu));
	return err;
}

static void pci_rd_end_rq(struct request *rq)
{
	struct pci_remote_disk_request *rb_req = blk_mq_rq_to_pdu(rq);
	blk_mq_end_request(rq, rb_req->descr->status);
}

static enum blk_eh_timer_return pci_rd_timeout_rq(struct request *rq, bool res)
{
	struct pci_remote_disk_request *rb_req = blk_mq_rq_to_pdu(rq);
	dev_err(&rb_req->rdev->pdev->dev, "%s : Timeout waiting for request descriptor: %i\n", __FUNCTION__, rb_req->descr_idx);
	return BLK_EH_DONE;
}

static const struct blk_mq_ops pci_rd_mq_ops = { .queue_rq = pci_rd_queue_rq,
						  .complete = pci_rd_end_rq,
						  .timeout =
							  pci_rd_timeout_rq };

static int pci_rd_open(struct block_device *bdev, fmode_t mode)
{
	struct pci_remote_disk *rdev = bdev->bd_disk->private_data;
	dev_dbg(&rdev->pdev->dev, "%s called\n", __FUNCTION__);
	return 0;
}

static void pci_rd_release(struct gendisk *disk, fmode_t mode)
{
    struct pci_remote_disk *rdev = disk->private_data;
    dev_dbg(&rdev->pdev->dev, "%s called\n", __FUNCTION__);
}

static int pci_rd_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct pci_remote_disk *rdev = bdev->bd_disk->private_data;
	dev_dbg(&rdev->pdev->dev, "%s called\n", __FUNCTION__);
	geo->heads = 4;
	geo->sectors = 16;
	geo->cylinders =
		get_capacity(bdev->bd_disk) / (geo->heads * geo->sectors);
	return 0;
}

static int pci_rd_ioctl(struct block_device *bdev, fmode_t mode,
			 unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

#ifdef CONFIG_COMPAT
static int pci_rd_compat_ioctl(struct block_device *bdev, fmode_t mode,
				unsigned int cmd, unsigned long arg)
{
	return pci_rd_ioctl(bdev, mode, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct block_device_operations pci_rd_ops = {
	.open = pci_rd_open,
	.release = pci_rd_release,
	.getgeo = pci_rd_getgeo,
	.owner = THIS_MODULE,
	.ioctl = pci_rd_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pci_rd_compat_ioctl,
#endif
};

static irqreturn_t pci_rd_irqhandler(int irq, void *dev_id)
{
	struct pci_remote_disk *rdev = dev_id;
	dev_err(&rdev->pdev->dev, "%s\n", __FUNCTION__);
	return IRQ_HANDLED;
}

static void pci_rd_clear_descriptor(struct pci_remote_disk *rdev, struct pci_epf_bio_descr *descr)
{
    unsigned long flags;
    spin_lock_irqsave(&rdev->lock, flags);
    /* descr->flags &= ~PBI_EPF_BIO_F_USED; */
    memset(descr, 0, sizeof(*descr));
    spin_unlock_irqrestore(&rdev->lock, flags);
}

static int pci_remote_bd_dispatch(void *cookie)
{
    struct pci_remote_disk *rdev =
	(struct pci_remote_disk *)cookie;
    struct device *dev = &rdev->pdev->dev;
    struct pci_bio_device_ring *dev_ring = pci_rd_get_device_ring(rdev);
    struct req_iterator iter;
    struct bio_vec bv;

    allow_signal(SIGINT);
    allow_signal(SIGTERM);
    allow_signal(SIGKILL);
    allow_signal(SIGUSR1);

    while (!kthread_should_stop()) {
	while (ioread16(&dev_ring->idx) != rdev->dev_idx) {
	    int descr_idx = dev_ring->ring[rdev->dev_idx].index;
	    struct pci_epf_bio_descr *desc = &rdev->descr_ring[descr_idx];
	    struct pci_remote_disk_request *rb_req = (struct pci_remote_disk_request *)desc->tag;
	    struct request *rq = blk_mq_rq_from_pdu(rb_req);
	    void *buf;
	    int i = 0;

	    BUG_ON(rb_req == NULL);
	    BUG_ON(!(READ_ONCE(desc->flags) & PBI_EPF_BIO_F_USED));
	    
	    dev_dbg(dev, "Digest Req: sec: 0x%llX, len: 0x%x\n", desc->s_sector, desc->len);
	    if (rq_data_dir(rq) == READ) {
		buf = kmap(rb_req->pg);
		rq_for_each_segment (bv, rq, iter) {
		    memcpy_to_bvec(&bv, buf);
		    dev_dbg(dev,
			    "%i: bv.len = 0x%x, bv.offset = 0x%x, page: 0x%llX. Buf: 0x%llX\n",
			    i, bv.bv_len, bv.bv_offset,
			    page_address(bv.bv_page), buf);
		    buf += bv.bv_len;
		    i++;
		}
		kunmap(rb_req->pg);
	    }
			
	    dma_unmap_single(dev, desc->addr, desc->len,
			     rq_dma_dir(rq));
	    pci_rd_clear_descriptor(rdev, desc);
	    __free_pages(rb_req->pg, rb_req->order);
	    WRITE_ONCE(rdev->dev_idx, (rdev->dev_idx + 1) % NUM_DESRIPTORS);
	    blk_mq_complete_request(rq);
	}
	usleep_range(500, 1000);
    }

    return 0;
}

static int pci_remote_bd_send_command(struct pci_remote_disk *rdev, u32 cmd)
{
    int timeout = 0;
    struct device *dev = &rdev->pdev->dev;
    
    iowrite32(cmd, &rdev->base->command);
    while(++timeout < RD_STATUS_TIMEOUT_COUNT && ioread32(&rdev->base->status) != RBD_STATUS_SUCCESS);

    if (ioread32(&rdev->base->status) != RBD_STATUS_SUCCESS) {
	dev_err(dev, "cannot set queue address\n");
	return -ENODEV;
    }
	
    iowrite32(0, &rdev->base->status);
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
	dma_addr_t phys_addr;
	void __iomem *base;
	struct pci_remote_disk *rdev =
		devm_kzalloc(dev, sizeof(*rdev), GFP_KERNEL);
	if (!rdev)
		return -ENOMEM;

	rdev->dp_thr =
		kthread_create(pci_remote_bd_dispatch, rdev, "pci_rd_thread");
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
		      sizeof(u64));

	rdev->dp_order = get_order(rdev->descr_size);
	rdev->desc_page = alloc_pages(GFP_KERNEL | GFP_DMA, rdev->dp_order);
	if (!rdev->desc_page) {
	    dev_err(dev, "could not alloc memory\n", rdev->descr_size);
	    err = -ENOSPC;
	    goto out_free_dev;
	}
	
	rdev->descr_ring = kmap(rdev->desc_page);
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

	if (rdev->base->magic != RBD_MAGIC) {
	    dev_err(dev, "Invalid magic at BAR0 detected. Expected 0x%x. Read 0x%x\n", RBD_MAGIC, rdev->base->magic);
	    err = -ENODEV;
	    goto err_iounmap;
	}
	
	phys_addr = dma_map_single(dev, rdev->descr_ring,
			       rdev->descr_size, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, phys_addr)) {
		dev_err(dev, "failed to map descriptor address\n");
		err = -ENODEV;
		goto err_iounmap;
	}

	rdev->drv_offset =
		ALIGN(NUM_DESRIPTORS * sizeof(*rdev->descr_ring), sizeof(u64));
	rdev->dev_offset =
		rdev->drv_offset + ALIGN(sizeof(struct pci_bio_driver_ring) +
						 (NUM_DESRIPTORS * sizeof(u16)),
					 sizeof(u64));

	iowrite32(rdev->drv_offset, &rdev->base->drv_offset);
	iowrite32(rdev->dev_offset, &rdev->base->dev_offset);
	dev_info(
		dev,
		" Setting queue addr @ 0x%llX -> 0x%llx. Queue Size for %i descriptors = %i. Offset = %i\n",
		rdev->descr_ring, phys_addr, NUM_DESRIPTORS, rdev->descr_size,
		offset);

	iowrite64(phys_addr, &rdev->base->queue_addr);
	iowrite32(rdev->descr_size, &rdev->base->queue_size);
	iowrite32(NUM_DESRIPTORS, &rdev->base->queue_size);
	smp_wmb();
	err = pci_remote_bd_send_command(rdev, COMMAND_SET_QUEUE);
	if (err) {
	    dev_err(dev, "cannot set queue\n");
	    goto err_iounmap;
   	}
	
	err = pci_remote_bd_send_command(rdev, COMMAND_START);
	if (err) {
	    dev_err(dev, "cannot start device\n");
	    goto err_iounmap;
   	}

	for (i = 0; i < rdev->num_irqs; i++) {
	    err = devm_request_irq(dev, pci_irq_vector(pdev, i),
				   pci_rd_irqhandler, IRQF_SHARED,
				   "rd-irq", rdev);
	    if (err) {
		dev_err(dev, "Unable to register irq %i\n", i);
		break;
	    }
	}

	pci_set_drvdata(pdev, rdev);
	spin_lock_init(&rdev->lock);
	rdev->tag_set.ops = &pci_rd_mq_ops;
	rdev->tag_set.queue_depth = 32;
	rdev->tag_set.numa_node = NUMA_NO_NODE;
	rdev->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
	rdev->tag_set.nr_hw_queues = num_present_cpus();
	rdev->tag_set.cmd_size =
		sizeof(struct pci_remote_disk_request) +
		sizeof(struct scatterlist) * PCI_RBD_INLINE_SG_CNT;
	rdev->tag_set.driver_data = rdev;

	err = blk_mq_alloc_tag_set(&rdev->tag_set);
	if (err)
		goto err_iounmap;

	rdev->gd = blk_mq_alloc_disk(&rdev->tag_set, rdev);
	if (IS_ERR(rdev->gd)) {
		err = -ENODEV;
		goto out_tag_set;
	}

	rdev->gd->fops = &pci_rd_ops;
	rdev->gd->private_data = rdev;
	rdev->gd->queue->queuedata = rdev;

	/* skip the /dev/ part, and export it as pci-rd-<disk-name>*/
	snprintf(rdev->gd->disk_name, sizeof(rdev->gd->disk_name), "pci-rd-%s",
		 &rdev->base->dev_name[5]);
	set_capacity(rdev->gd, rdev->base->num_sectors);
	device_add_disk(dev, rdev->gd, NULL);
	wake_up_process(rdev->dp_thr);
	return 0;
out_tag_set:
	blk_mq_free_tag_set(&rdev->tag_set);
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
	kunmap(rdev->desc_page);
	__free_pages(rdev->desc_page, rdev->dp_order);
out_free_dev:
	devm_kfree(dev, rdev);
	return err;
}

static void pci_remote_bd_remove(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	struct pci_remote_disk *rdev = pci_get_drvdata(pdev);
	
	blk_cleanup_disk(rdev->gd);
	del_gendisk(rdev->gd);

	blk_mq_free_tag_set(&rdev->tag_set);
	kunmap(rdev->desc_page);
	__free_pages(rdev->desc_page, rdev->dp_order);
	devm_kfree(dev, rdev);
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
MODULE_DESCRIPTION("Remote PCI Endpoint Disk driver");
MODULE_LICENSE("GPL v2");
