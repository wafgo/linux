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
#include <linux/configfs.h>

#define RBD_MAGIC 0x636f6e74
#define NUM_DESRIPTORS 1024

#define RBD_STATUS_SUCCESS	                BIT(0)
#define RD_STATUS_TIMEOUT_COUNT                 (100)

#define COMMAND_RAISE_LEGACY_IRQ	BIT(0)
#define COMMAND_RAISE_MSI_IRQ		BIT(1)
#define COMMAND_RAISE_MSIX_IRQ		BIT(2)
#define COMMAND_WRITE_TO_QUEUE          BIT(4)
#define COMMAND_READ_FROM_QUEUE         BIT(5)
#define COMMAND_SET_QUEUE               BIT(6)
#define COMMAND_GET_DEVICES             BIT(7)
#define COMMAND_START                   BIT(8)
#define COMMAND_GET_NUM_SECTORS         BIT(9)

#define PBI_EPF_BIO_F_LAST BIT(0)
#define PBI_EPF_BIO_F_USED BIT(1)

#define PBI_EPF_BIO_F_DIR_READ BIT(2)

struct pci_epf_blockpt_reg {
	u32	magic;
	u32	command;
	u32	status;
        u32     queue_size;  /* number of struct pci_epf_blockpt_descr */
        u32     drv_offset;
        u32     dev_offset;
	u32	flags;
        u32	num_desc;
        u32     max_devs;
        u8      dev_idx;
        u8      res0;
        u8      res1;
        u8      res2;
        u64	queue_addr;  /* start of struct pci_epf_blockpt_descr*/
        u64     num_sectors;
        char    dev_name[];
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


struct pci_bio_driver_ring {
	u16 idx;
	u16 ring[]; /* queue size*/
};

struct pci_bio_device_ring {
	u16 idx;
	u16 ring[]; /* queue size*/
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

struct pci_remote_disk_common;

struct pci_remote_disk_device {
    struct list_head node;
    struct pci_remote_disk_common *rcom;
    struct blk_mq_tag_set tag_set;
    struct config_group cfs_group;
    struct gendisk *gd;
    struct pci_epf_bio_descr __iomem *descr_ring;
    struct page *desc_page;
    sector_t capacity;
    char *r_name;
    char *npr_name;
    char *l_name;
    u32 dp_order;
    u32 descr_size;
    u32 drv_offset;
    u32 dev_offset;
    u32 drv_idx;
    u32 dev_idx;
    u8 id;
    u16 ns_idx;
    bool attached;
    spinlock_t lock;
    struct task_struct *dp_thr;
    const struct blk_mq_queue_data *bd;
};


struct pci_remote_disk_common {
    struct list_head bd_list;
    struct pci_dev *pdev;
    struct pci_epf_blockpt_reg __iomem *base;
    void __iomem *bar[PCI_STD_NUM_BARS];
    int num_irqs;
};

struct pci_remote_disk_request {
	struct pci_remote_disk_device *rdd;
	struct bio *bio;
	blk_status_t status;
	struct page *pg;
	int order;
	int num_bios;
        int descr_idx;
        struct pci_epf_bio_descr *descr;
	struct work_struct bio_work;
	struct sg_table sg_table;
	struct scatterlist sg[];
};

static LIST_HEAD(available_remote_disks);

static blk_status_t pci_rd_queue_rq(struct blk_mq_hw_ctx *hctx,
				    const struct blk_mq_queue_data *bd);
static void pci_rd_end_rq(struct request *rq);
static enum blk_eh_timer_return pci_rd_timeout_rq(struct request *rq, bool res);

static const struct blk_mq_ops pci_rd_mq_ops = { .queue_rq = pci_rd_queue_rq,
						  .complete = pci_rd_end_rq,
						  .timeout =
							  pci_rd_timeout_rq };

static int pci_rd_open(struct block_device *bdev, fmode_t mode);
static void pci_rd_release(struct gendisk *disk, fmode_t mode);
static int pci_rd_getgeo(struct block_device *bdev, struct hd_geometry *geo);
static int pci_rd_ioctl(struct block_device *bdev, fmode_t mode,
			unsigned int cmd, unsigned long arg);
static int pci_rd_compat_ioctl(struct block_device *bdev, fmode_t mode,
			       unsigned int cmd, unsigned long arg);

static int pci_remote_bd_dispatch(void *cookie);

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

static int pci_remote_bd_send_command(struct pci_remote_disk_common *rcom, u32 cmd)
{
    int timeout = 0;
    
    writel(cmd, &rcom->base->command);
    while(++timeout < RD_STATUS_TIMEOUT_COUNT && readl(&rcom->base->status) != RBD_STATUS_SUCCESS)   {
	usleep_range(100, 200);
    }

    if (readl(&rcom->base->status) != RBD_STATUS_SUCCESS) {
	return -ENODEV;
    }
	
    writel(0, &rcom->base->status);
    return 0;
}

static inline struct pci_remote_disk_device *to_remote_disk_dev(struct config_item *item)
{
	return container_of(to_config_group(item), struct pci_remote_disk_device, cfs_group);
}

static ssize_t pci_remote_disk_group_remote_name_show(struct config_item *item,
					   char *page)
{
    struct pci_remote_disk_device *rdd = to_remote_disk_dev(item);
    return sprintf(page, "%s", rdd->r_name);
}

CONFIGFS_ATTR_RO(pci_remote_disk_group_, remote_name);

static ssize_t pci_remote_disk_group_local_name_show(struct config_item *item,
					   char *page)
{
    struct pci_remote_disk_device *rdd = to_remote_disk_dev(item);
    return sprintf(page, "%s", rdd->l_name);
}

static ssize_t pci_remote_disk_group_local_name_store(struct config_item *item,
						     const char *page, size_t len)
{
    struct pci_remote_disk_device *rdd = to_remote_disk_dev(item);
    if (rdd->l_name)
	kfree(rdd->l_name);
    rdd->l_name = kasprintf(GFP_KERNEL, "%s", page);
    return len;
}

CONFIGFS_ATTR(pci_remote_disk_group_, local_name);

static ssize_t pci_remote_disk_group_attach_show(struct config_item *item,
					   char *page)
{
    struct pci_remote_disk_device *rdd = to_remote_disk_dev(item);
    return sprintf(page, "%s\n", rdd->attached ? "yes" : "no");
}

static ssize_t pci_remote_disk_group_attach_store(struct config_item *item,
						     const char *page, size_t len)
{
    bool attach;
    dma_addr_t phys_addr;
    struct pci_remote_disk_device *rdd = to_remote_disk_dev(item);
    struct device *dev = &rdd->rcom->pdev->dev;
    struct pci_epf_blockpt_reg __iomem *base = rdd->rcom->base;
    int ret = kstrtobool(page, &attach);
    
    if (ret)
	return ret;

    if (!rdd->attached && attach) {
	rdd->descr_size = ALIGN(NUM_DESRIPTORS * sizeof(*rdd->descr_ring), sizeof(u64)) + ALIGN(sizeof(struct pci_bio_driver_ring) + (NUM_DESRIPTORS * sizeof(u16)), sizeof(u64)) + ALIGN(sizeof(struct pci_bio_device_ring) + (NUM_DESRIPTORS * sizeof(u16)), sizeof(u64));

	rdd->dp_order = get_order(rdd->descr_size);
	/* we need page aligned memory for the descriptors, use alloc_pages() to not waste any memory */
	rdd->desc_page = alloc_pages(GFP_KERNEL | GFP_DMA, rdd->dp_order);
	if (!rdd->desc_page) {
	    dev_err(dev, "could not alloc memory\n", rdd->descr_size);
	    ret = -ENOSPC;
	    goto out_err;
	}
	
	rdd->descr_ring = kmap(rdd->desc_page);
	phys_addr = dma_map_single(dev, rdd->descr_ring,
			       rdd->descr_size, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dev, phys_addr)) {
		dev_err(dev, "failed to map descriptor address\n");
		ret = -ENODEV;
		goto err_desc_unmap;
	}

	rdd->drv_offset =
		ALIGN(NUM_DESRIPTORS * sizeof(*rdd->descr_ring), sizeof(u64));
	rdd->dev_offset =
		rdd->drv_offset + ALIGN(sizeof(struct pci_bio_driver_ring) +
						 (NUM_DESRIPTORS * sizeof(u16)),
					 sizeof(u64));
	writeb(rdd->id, &base->dev_idx);
	writel(rdd->drv_offset, &base->drv_offset);
	writel(rdd->dev_offset, &base->dev_offset);
	dev_info(
		dev,
		"%s: Setting queue addr @ 0x%llX -> 0x%llx. Queue Size for %i descriptors = %i\n", rdd->npr_name,
		rdd->descr_ring, phys_addr, NUM_DESRIPTORS, rdd->descr_size);
	writeq(phys_addr, &base->queue_addr);
	writel(rdd->descr_size, &base->queue_size);
	writel(NUM_DESRIPTORS, &base->num_desc);
	smp_wmb();
	ret = pci_remote_bd_send_command(rdd->rcom, COMMAND_SET_QUEUE);
	if (ret) {
	    dev_err(dev, "%s: cannot set queue\n", rdd->npr_name);
	    goto err_desc_unmap;
   	}
	
	ret = pci_remote_bd_send_command(rdd->rcom, COMMAND_GET_NUM_SECTORS);
	if (ret) {
	    dev_err(dev, "%s: cannot get number of sectors\n", rdd->npr_name);
	    goto err_desc_unmap;
   	}
	
	rdd->capacity = readq(&base->num_sectors);
	dev_info(dev, "%s has capacity 0x%lx\n", rdd->r_name, rdd->capacity);
	/* FIXME */
	ret = pci_remote_bd_send_command(rdd->rcom, COMMAND_START);
	if (ret) {
	    dev_err(dev, "%s: cannot start device\n", rdd->npr_name);
	    goto err_desc_unmap;
   	}

	rdd->tag_set.ops = &pci_rd_mq_ops;
	rdd->tag_set.queue_depth = 32;
	rdd->tag_set.numa_node = NUMA_NO_NODE;
	rdd->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
	rdd->tag_set.nr_hw_queues = num_present_cpus();
	rdd->tag_set.cmd_size =
		sizeof(struct pci_remote_disk_request) +
		sizeof(struct scatterlist) * PCI_RBD_INLINE_SG_CNT;
	rdd->tag_set.driver_data = rdd;
	ret = blk_mq_alloc_tag_set(&rdd->tag_set);
	if (ret) {
	    dev_err(dev, "%s: Could not allocate tag set\n", rdd->npr_name);
	    goto err_desc_unmap;
	}

	rdd->gd = blk_mq_alloc_disk(&rdd->tag_set, rdd);
	if (IS_ERR(rdd->gd)) {
		ret = -ENODEV;
		goto out_free_tag_set;
	}

	rdd->gd->fops = &pci_rd_ops;
	rdd->gd->private_data = rdd;
	rdd->gd->queue->queuedata = rdd;
	snprintf(rdd->gd->disk_name, sizeof(rdd->gd->disk_name), "%s", rdd->l_name);
	set_capacity(rdd->gd, rdd->capacity);
	rdd->dp_thr = kthread_create(pci_remote_bd_dispatch, rdd, "rdt-%s", rdd->npr_name);
	if (IS_ERR(rdd->dp_thr)) {
		dev_err(dev, "Cannot create kernel dispatcher thread\n");
		ret = PTR_ERR(rdd->dp_thr);
		goto out_free_tag_set;
	}

	device_add_disk(dev, rdd->gd, NULL);
	rdd->attached = true;
    } else if (rdd->attached && !attach) {
	rdd->attached = false;
    }
    
    return len;
    
out_free_tag_set:
    blk_mq_free_tag_set(&rdd->tag_set);
err_desc_unmap:
  kunmap(rdd->desc_page);    
out_err:
    return ret;
}

CONFIGFS_ATTR(pci_remote_disk_group_, attach);

static struct configfs_attribute *pci_remote_disk_group_attrs[] = {
	&pci_remote_disk_group_attr_remote_name,
	&pci_remote_disk_group_attr_local_name,
	&pci_remote_disk_group_attr_attach,
	NULL,
};


static const struct config_item_type pci_remote_disk_group_type = {
	.ct_owner	= THIS_MODULE,
	.ct_attrs	= pci_remote_disk_group_attrs,
};

static const struct config_item_type pci_remote_disk_type = {
	.ct_owner	= THIS_MODULE,
};

static struct configfs_subsystem pci_remote_disk_subsys = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "pci_remote_disk",
			.ci_type = &pci_remote_disk_type,
		},
	},
	.su_mutex = __MUTEX_INITIALIZER(pci_remote_disk_subsys.su_mutex),
};



static const struct pci_device_id pci_remote_bd_tbl[] = {
	{
		PCI_DEVICE(0x0, 0xc402),
	},
	{ 0 }
};

static struct pci_bio_driver_ring * pci_rd_get_driver_ring(struct pci_remote_disk_device *rdd)
{
	return (struct pci_bio_driver_ring *)((u64)rdd->descr_ring + rdd->drv_offset);
}

static struct pci_bio_device_ring * pci_rd_get_device_ring(struct pci_remote_disk_device *rdd)
{
	return (struct pci_bio_device_ring *)((u64)rdd->descr_ring + rdd->dev_offset);
}

static int pci_rd_alloc_free_descriptor(struct pci_remote_disk_device *rdd)
{
        int i;
	int ret = -ENOSPC;
	struct device *dev = &rdd->rcom->pdev->dev;
	spin_lock(&rdd->lock);
	for (i = 0; i < NUM_DESRIPTORS; ++i) {
		struct pci_epf_bio_descr __iomem *de = &rdd->descr_ring[rdd->ns_idx];
		u32 flags = READ_ONCE(de->flags);
		if (!(flags & PBI_EPF_BIO_F_USED)) {
			dev_dbg(dev, "Found free descriptor at idx %i\n", rdd->ns_idx);
			WRITE_ONCE(de->flags, flags | PBI_EPF_BIO_F_USED);
			ret = rdd->ns_idx;
			rdd->ns_idx = (rdd->ns_idx + 1) % NUM_DESRIPTORS;
			goto unlock_return;
		}
		rdd->ns_idx = (rdd->ns_idx + 1) % NUM_DESRIPTORS;
	}
unlock_return:
	spin_unlock(&rdd->lock);
	if (ret == -ENOSPC)
	    dev_err(dev, "No free descriptor\n");
	return ret;
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
	int descr_idx;
	struct pci_remote_disk_device *rdd = hctx->queue->queuedata;
	struct pci_remote_disk_request *rb_req = blk_mq_rq_to_pdu(bd->rq);

	struct device *dev = &rdd->rcom->pdev->dev;
	struct pci_epf_bio_descr __iomem *dtu;
	struct pci_bio_driver_ring __iomem *drv_ring = pci_rd_get_driver_ring(rdd);
	dma_addr_t dma_addr;
	char *buf;
	int err;

	rb_req->rdd = rdd;
	if (!is_valid_request(req_op(bd->rq))) {
	    dev_err(dev, "Unsupported Request: %i\n", req_op(bd->rq));
	    return BLK_STS_NOTSUPP;
	}

	descr_idx = pci_rd_alloc_free_descriptor(rdd);
	if (unlikely(descr_idx < 0))
	    return BLK_STS_AGAIN;

	dtu = &rdd->descr_ring[descr_idx];
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
		buf += bv.bv_len;
	    }
	}
	dtu->s_sector = blk_rq_pos(bd->rq);
	dtu->tag = (u64)rb_req;
	spin_lock(&rdd->lock);
	writew(descr_idx, &drv_ring->ring[rdd->drv_idx]);
	rdd->drv_idx = (rdd->drv_idx + 1) % NUM_DESRIPTORS;
	smp_wmb();
	writew(rdd->drv_idx, &drv_ring->idx);
	spin_unlock(&rdd->lock);
	dev_dbg(dev, "(DIR: %s): Adding desc %i (%i). sector: 0x%llX, len: 0x%x, offset: 0x%x\n",
		 (rq_data_dir(bd->rq) == WRITE) ? "WRITE" : "READ", descr_idx, rdd->drv_idx,
		 dtu->s_sector, dtu->len, dtu->offset);

	blk_mq_start_request(bd->rq);
	wake_up_process(rdd->dp_thr);
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
	blk_mq_end_request(rq, rb_req->status);
}

static enum blk_eh_timer_return pci_rd_timeout_rq(struct request *rq, bool res)
{
	struct pci_remote_disk_request *rb_req = blk_mq_rq_to_pdu(rq);
	dev_err(&rb_req->rdd->rcom->pdev->dev, "%s : Timeout waiting for request descriptor: %i\n", __FUNCTION__, rb_req->descr_idx);
	return BLK_EH_DONE;
}


static int pci_rd_open(struct block_device *bdev, fmode_t mode)
{
	struct pci_remote_disk_common *rcom = bdev->bd_disk->private_data;
	dev_dbg(&rcom->pdev->dev, "%s called\n", __FUNCTION__);
	return 0;
}

static void pci_rd_release(struct gendisk *disk, fmode_t mode)
{
    struct pci_remote_disk_common *rcom = disk->private_data;
    dev_dbg(&rcom->pdev->dev, "%s called\n", __FUNCTION__);
}

static int pci_rd_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	struct pci_remote_disk_common *rcom = bdev->bd_disk->private_data;
	dev_dbg(&rcom->pdev->dev, "%s called\n", __FUNCTION__);
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


static irqreturn_t pci_rd_irqhandler(int irq, void *dev_id)
{
	struct pci_remote_disk_common *rcom = dev_id;
	dev_err(&rcom->pdev->dev, "%s\n", __FUNCTION__);
	return IRQ_HANDLED;
}

static void pci_rd_clear_descriptor(struct pci_remote_disk_device *rdd, struct pci_epf_bio_descr *descr)
{
    unsigned long flags;
    spin_lock_irqsave(&rdd->lock, flags);
    memset(descr, 0, sizeof(*descr));
    spin_unlock_irqrestore(&rdd->lock, flags);
}

static int pci_remote_bd_dispatch(void *cookie)
{
    struct pci_remote_disk_device *rdd = cookie;
    struct device *dev = &rdd->rcom->pdev->dev;
    struct pci_bio_device_ring *dev_ring = pci_rd_get_device_ring(rdd);
    struct req_iterator iter;
    struct bio_vec bv;

    while (!kthread_should_stop()) {
	while (readw(&dev_ring->idx) != rdd->dev_idx) {
	    u16 descr_idx = readw(&dev_ring->ring[rdd->dev_idx]);
	    struct pci_epf_bio_descr *desc = &rdd->descr_ring[descr_idx];
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
	    rb_req->status = (blk_status_t) readl(&desc->status);
	    pci_rd_clear_descriptor(rdd, desc);
	    __free_pages(rb_req->pg, rb_req->order);
	    WRITE_ONCE(rdd->dev_idx, (rdd->dev_idx + 1) % NUM_DESRIPTORS);
	    blk_mq_complete_request(rq);
	}
	usleep_range(500, 1000);
    }

    return 0;
}

static int pci_remote_parse_disks(struct pci_remote_disk_common *rcom)
{
    struct pci_remote_disk_device *rdd;
    struct list_head *lh, *lhtmp;
    char *sbd, *ebd;
    int count = 0;
    int err;
    char *loc_st;
    struct device *dev = &rcom->pdev->dev;

    loc_st = kasprintf(GFP_KERNEL, "%s", rcom->base->dev_name);
    sbd = ebd = loc_st;

    while ((ebd = strchr(sbd, ';')) != NULL) {
	rdd = kzalloc(sizeof(*rdd), GFP_KERNEL);
	if (!rdd) {
	    dev_err(dev, "Could not allocate rd struct\n");
	    err = -ENOMEM;
	    goto err_free;
	}
	
	INIT_LIST_HEAD(&rdd->node);
	list_add_tail(&rdd->node, &available_remote_disks);
	rdd->r_name = kmemdup_nul(sbd, ebd - sbd, GFP_KERNEL);
	if (!rdd->r_name) {
	    dev_err(dev, "Could not allocate memory for remote device name\n");
	    err = -ENOMEM;
	    goto err_free;
	}
	
	rdd->l_name = kasprintf(GFP_KERNEL, "pci-rd-%s", rdd->r_name);
	if (!rdd->l_name) {
	    dev_err(dev, "Could not allocate memory for local device name\n");
	    err = -ENOMEM;
	    goto err_free;
	}
	
	spin_lock_init(&rdd->lock);
	rdd->rcom = rcom;
	rdd->id = count;
	/* get rid of all path seperators  */
	rdd->npr_name = strrchr(rdd->r_name, '/');
	rdd->npr_name = (rdd->npr_name == NULL) ? rdd->r_name : (rdd->npr_name + 1);
	config_group_init_type_name(&rdd->cfs_group, rdd->npr_name, &pci_remote_disk_group_type);
	err = configfs_register_group(&pci_remote_disk_subsys.su_group, &rdd->cfs_group);
	if (err) {
	    dev_err(dev, "Cannot register configfs group for %s\n", rdd->npr_name);
	    err = -ENODEV;
	    goto err_free;
	}
	
	dev_info(dev, "Found: %s\n", rdd->r_name);
	sbd = ebd + 1;
	count++;
    }
    
    kfree(loc_st);
    return count;
    
err_free:
    kfree(loc_st);
    list_for_each_safe(lh, lhtmp, &available_remote_disks) {
	rdd = list_entry(lh, struct pci_remote_disk_device, node);
	if (rdd->r_name) {
	    kfree(rdd->r_name);
	    configfs_unregister_group(&rdd->cfs_group);
	}
	if (rdd->l_name)
	    kfree(rdd->l_name);
	list_del(lh);
	kfree(rdd);
    }
    return err;
}

static int pci_remote_bd_probe(struct pci_dev *pdev,
			       const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	int err;
	enum pci_barno bar;
	int i;
	enum pci_barno def_reg_bar = BAR_0;
	void __iomem *base;
	struct pci_remote_disk_common *rcom =
		devm_kzalloc(dev, sizeof(*rcom), GFP_KERNEL);
	if (!rcom)
		return -ENOMEM;

	rcom->pdev = pdev;
	if ((dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(48)) != 0) &&
	    dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32)) != 0) {
		err = -ENODEV;
		dev_err(dev, "Cannot set DMA mask\n");
		goto out_free_dev;
	}

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Cannot enable PCI device\n");
		goto out_free_dev;
	}

	err = pci_request_regions(pdev, DRV_MODULE_NAME);
	if (err) {
		dev_err(dev, "Cannot obtain PCI resources\n");
		goto err_disable_pdev;
	}

	pci_set_master(pdev);
	rcom->num_irqs = pci_alloc_irq_vectors(pdev, 1, 32, PCI_IRQ_MSI);
	if (rcom->num_irqs < 0)
		dev_err(dev, "Failed to get MSI interrupts\n");

	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (pci_resource_flags(pdev, bar) & IORESOURCE_MEM) {
			base = pci_ioremap_bar(pdev, bar);
			if (!base) {
				dev_err(dev, "Failed to read BAR%d\n", bar);
				WARN_ON(bar == def_reg_bar);
			}
			rcom->bar[bar] = base;
		}
	}

	rcom->base = rcom->bar[def_reg_bar];
	if (!rcom->base) {
		err = -ENOMEM;
		dev_err(dev, "Cannot perform PCI communictaion without BAR%d\n",
			def_reg_bar);
		goto err_iounmap;
	}

	if (rcom->base->magic != RBD_MAGIC) {
	    dev_err(dev, "Invalid magic at BAR0 detected. Expected 0x%x. Read 0x%x\n", RBD_MAGIC, rcom->base->magic);
	    err = -ENODEV;
	    goto err_iounmap;
	}

	err = pci_remote_bd_send_command(rcom, COMMAND_GET_DEVICES);
	if (err) {
	    dev_err(dev, "Cannot get devices\n");
	    goto err_iounmap;
   	}
	
	dev_dbg(dev, "Following block devices are available: %s", rcom->base->dev_name);
	config_group_init(&pci_remote_disk_subsys.su_group);
	err = configfs_register_subsystem(&pci_remote_disk_subsys);
	if (err) {
	    dev_err(dev, "Error %d while registering subsystem %s\n",
		       err, pci_remote_disk_subsys.su_group.cg_item.ci_namebuf);
	    goto err_iounmap;
	}
	
	INIT_LIST_HEAD(&available_remote_disks);
	err = pci_remote_parse_disks(rcom);
	if (err <= 0) {
	    dev_err(dev, "Unable to parse any valid disk\n");
	    err = -ENODEV;
	    goto err_iounmap;
	}

	dev_info(dev, "Found %i devices\n", err);
	pci_set_drvdata(pdev, rcom);
	for (i = 0; i < rcom->num_irqs; i++) {
	    err = devm_request_irq(dev, pci_irq_vector(pdev, i),
				   pci_rd_irqhandler, IRQF_SHARED,
				   "rd-irq", rcom);
	    if (err) {
		dev_err(dev, "Unable to register irq %i\n", i);
		goto out_free_irq;
		
	    }
	}

	return 0;
	
out_free_irq:
	for (i = 0; i < rcom->num_irqs; i++) {
		devm_free_irq(dev, pci_irq_vector(pdev, i), rcom);
	}
	rcom->num_irqs = 0;
err_iounmap:
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		if (rcom->bar[bar])
			pci_iounmap(pdev, rcom->bar[bar]);
	}

	pci_free_irq_vectors(pdev);
	pci_release_regions(pdev);
err_disable_pdev:
	pci_disable_device(pdev);
out_free_dev:
	devm_kfree(dev, rcom);
	return err;
}

static void pci_remote_bd_remove(struct pci_dev *pdev)
{
    struct device *dev = &pdev->dev;
    struct pci_remote_disk_common *rcom = pci_get_drvdata(pdev);
    struct pci_remote_disk_device *rdd, *tmp_rdd;
    int i;
    
    list_for_each_entry_safe(rdd, tmp_rdd, &available_remote_disks, node) {
	if (rdd->attached) {
	    kthread_stop(rdd->dp_thr);
	    
	    del_gendisk(rdd->gd);
	    blk_mq_free_tag_set(&rdd->tag_set);
	    
	    kunmap(rdd->desc_page);
	    __free_pages(rdd->desc_page, rdd->dp_order);
	}
	
	kfree(rdd->r_name);
	kfree(rdd->l_name);
	configfs_unregister_group(&rdd->cfs_group);

	list_del(&rdd->node);
	kfree(rdd);
    }

    configfs_unregister_subsystem(&pci_remote_disk_subsys);
    for (i = 0; i < rcom->num_irqs; i++) {
	devm_free_irq(dev, pci_irq_vector(pdev, i), rcom);
    }
    
    rcom->num_irqs = 0;
    for (i = 0; i < PCI_STD_NUM_BARS; i++) {
	if (rcom->bar[i])
	    pci_iounmap(pdev, rcom->bar[i]);
    }
    
    pci_free_irq_vectors(pdev);
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    devm_kfree(dev, rcom);
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
