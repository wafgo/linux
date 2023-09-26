// SPDX-License-Identifier: GPL-2.0-only
/*
 * PCI Remote Block Device Driver
 *
 * Copyright (C) 2023 Continental Automotive GmbH
 * Wadim Mueller <wadim.mueller@continental.com>
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
#include <linux/pci-epf.h>
#include <linux/pci-epf-block-passthru.h>

#define NUM_DESRIPTORS                          1024

#define RD_STATUS_TIMEOUT_COUNT                 (100)

#define DRV_MODULE_NAME                         "pci-remote-disk"
#define PCI_RBD_INLINE_SG_CNT                   2

struct pci_remote_disk_common;

struct pci_remote_disk_device {
    struct list_head node;
    struct pci_remote_disk_common *rcom;
    struct blk_mq_tag_set tag_set;
    struct config_group cfs_group;
    struct gendisk *gd;
    struct pci_epf_blockpt_descr __iomem *descr_ring;
    struct pci_blockpt_driver_ring __iomem *drv_ring;
    struct pci_blockpt_device_ring __iomem *dev_ring;
    struct page *desc_page;
    u64 *descr_tags;
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
    int *irqs;
    int num_irqs;
    struct semaphore dig_sem;
    char irq_name[32];
    u8 id;
    u16 ns_idx;
    bool attached;
    bool read_only;
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
        struct pci_epf_blockpt_descr *descr;
	struct work_struct bio_work;
	struct sg_table sg_table;
	struct scatterlist sg[];
};

static LIST_HEAD(available_remote_disks);

static irqreturn_t pci_rd_irqhandler(int irq, void *dev_id);
static blk_status_t pci_rd_queue_rq(struct blk_mq_hw_ctx *hctx,
				    const struct blk_mq_queue_data *bd);
static void pci_rd_end_rq(struct request *rq);
static enum blk_eh_timer_return pci_rd_timeout_rq(struct request *rq, bool res);

static const struct blk_mq_ops pci_rd_mq_ops = { .queue_rq = pci_rd_queue_rq,
						 .complete = pci_rd_end_rq,
						 .timeout = pci_rd_timeout_rq };

static int pci_rd_open(struct block_device *bdev, fmode_t mode);
static void pci_rd_release(struct gendisk *disk, fmode_t mode);
static int pci_rd_getgeo(struct block_device *bdev, struct hd_geometry *geo);
static int pci_rd_ioctl(struct block_device *bdev, fmode_t mode,
			unsigned int cmd, unsigned long arg);
static int pci_rd_compat_ioctl(struct block_device *bdev, fmode_t mode,
			       unsigned int cmd, unsigned long arg);

static int pci_remote_disk_dispatch(void *cookie);

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

static int pci_remote_disk_send_command(struct pci_remote_disk_common *rcom, u32 cmd)
{
    int timeout = 0;
    
    smp_wmb();
    writel(cmd, &rcom->base->command);
    while(++timeout < RD_STATUS_TIMEOUT_COUNT && readl(&rcom->base->status) != BPT_STATUS_SUCCESS)   {
	usleep_range(100, 200);
    }

    if (readl(&rcom->base->status) != BPT_STATUS_SUCCESS) {
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

static int pci_remote_disk_attach_device(struct pci_remote_disk_device *rdd)
{
    dma_addr_t phys_addr;
    int ret, i;
    struct device *dev = &rdd->rcom->pdev->dev;
    struct pci_epf_blockpt_reg __iomem *base = rdd->rcom->base;
    
    rdd->descr_size = ALIGN(NUM_DESRIPTORS * sizeof(*rdd->descr_ring), sizeof(u64)) + ALIGN(sizeof(struct pci_blockpt_driver_ring) + (NUM_DESRIPTORS * sizeof(u16)), sizeof(u64)) + ALIGN(sizeof(struct pci_blockpt_device_ring) + (NUM_DESRIPTORS * sizeof(u16)), sizeof(u64));

    rdd->dp_order = get_order(rdd->descr_size);
    /* we need page aligned memory for the descriptors, which is implicit for alloc_pages() */
    rdd->desc_page = alloc_pages(GFP_KERNEL | GFP_DMA, rdd->dp_order);
    if (!rdd->desc_page) {
	dev_err(dev, "could not alloc memory\n", rdd->descr_size);
	ret = -ENOSPC;
	goto out_err;
    }

    sema_init(&rdd->dig_sem, 0);
    rdd->descr_ring = kmap(rdd->desc_page);
    memset(rdd->descr_ring, 0, rdd->descr_size);
    rdd->dev_idx = rdd->drv_idx = rdd->ns_idx = 0;

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
	rdd->drv_offset + ALIGN(sizeof(struct pci_blockpt_driver_ring) +
				(NUM_DESRIPTORS * sizeof(u16)),
				sizeof(u64));
    rdd->drv_ring = (struct pci_blockpt_driver_ring *)((u64)rdd->descr_ring + rdd->drv_offset);
    rdd->dev_ring = (struct pci_blockpt_device_ring *)((u64)rdd->descr_ring + rdd->dev_offset);

    writeb(rdd->id, &base->dev_idx);
    writel(rdd->drv_offset, &base->drv_offset);
    writel(rdd->dev_offset, &base->dev_offset);
    
    dev_info(dev, "%s: Setting queue addr. #Descriptors %i (%i Bytes)\n", rdd->npr_name, NUM_DESRIPTORS, rdd->descr_size);
    
    writeq(phys_addr, &base->queue_addr);
    writel(rdd->descr_size, &base->queue_size);
    writel(NUM_DESRIPTORS, &base->num_desc);
    
    snprintf(rdd->irq_name, sizeof(rdd->irq_name), "rdd-%s-irq", rdd->npr_name);

    for (i = 0; i < rdd->num_irqs; ++i) {
	int *irq = &rdd->irqs[i];
	*irq = pci_irq_vector(rdd->rcom->pdev, (rdd->id * rdd->num_irqs) + i);
	dev_info(dev, "%i: request irq %i -> %i\n", i, (rdd->id * rdd->num_irqs) + i, *irq);
	ret = devm_request_irq(dev, *irq, pci_rd_irqhandler, IRQF_SHARED,
			       rdd->irq_name, rdd);
	if (ret) {
	    dev_err(dev, "Can't register %s IRQ. Id %i.\n", rdd->irq_name, *irq);
	    goto err_desc_unmap;
	}
    }

    dev_info(dev, "Setting first IRQ of %s to %i\n", rdd->l_name, (rdd->id * rdd->num_irqs) + 1);
    writel((rdd->id * rdd->num_irqs) + 1, &base->start_irq);
    ret = pci_remote_disk_send_command(rdd->rcom, BPT_COMMAND_SET_QUEUE);
    if (ret) {
	dev_err(dev, "%s: cannot set queue\n", rdd->npr_name);
	goto err_free_irq;
    }

    ret = pci_remote_disk_send_command(rdd->rcom, BPT_COMMAND_SET_IRQ);
    if (ret) {
	dev_err(dev, "%s: cannot set irq\n", rdd->npr_name);
	goto err_free_irq;
    }

    
    ret = pci_remote_disk_send_command(rdd->rcom, BPT_COMMAND_GET_NUM_SECTORS);
    if (ret) {
	dev_err(dev, "%s: cannot get number of sectors\n", rdd->npr_name);
	goto err_free_irq;
    }

    ret = pci_remote_disk_send_command(rdd->rcom, BPT_COMMAND_GET_PERMISSION);
    if (ret) {
	dev_err(dev, "%s: cannot get permission, assume RO\n", rdd->npr_name);
	rdd->read_only = true;
    } else {
	rdd->read_only = readb(&base->perm) & BPT_PERMISSION_RO;
    }
    
    rdd->capacity = readq(&base->num_sectors);
    dev_info(dev, "%s capacity 0x%lx\n", rdd->r_name, rdd->capacity);
    
    ret = pci_remote_disk_send_command(rdd->rcom, BPT_COMMAND_START);
    if (ret) {
	dev_err(dev, "%s: cannot start device\n", rdd->npr_name);
	goto err_free_irq;
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
	goto err_free_irq;
    }

    rdd->gd = blk_mq_alloc_disk(&rdd->tag_set, rdd);
    if (IS_ERR(rdd->gd)) {
	ret = -ENODEV;
	goto err_blk_mq_free;
    }

    rdd->gd->fops = &pci_rd_ops;
    rdd->gd->private_data = rdd->gd->queue->queuedata = rdd;
    snprintf(rdd->gd->disk_name, sizeof(rdd->gd->disk_name), "%s", rdd->l_name);
    set_capacity(rdd->gd, rdd->capacity);
    rdd->dp_thr = kthread_create(pci_remote_disk_dispatch, rdd, "rdt-%s", rdd->npr_name);
    if (IS_ERR(rdd->dp_thr)) {
	dev_err(dev, "Cannot create kernel dispatcher thread\n");
	ret = PTR_ERR(rdd->dp_thr);
	goto err_blk_mq_free;
    }

    wake_up_process(rdd->dp_thr);
    if (rdd->read_only)
	dev_info(dev, "%s attached in RO mode\n", rdd->npr_name);

    rdd->attached = true;
    set_disk_ro(rdd->gd, rdd->read_only);
    device_add_disk(dev, rdd->gd, NULL);

    return 0;

  err_blk_mq_free:
    blk_mq_free_tag_set(&rdd->tag_set);
  err_free_irq:
    for (i = 0; i < rdd->num_irqs; ++i) 
	devm_free_irq(dev, rdd->irqs[i], rdd);
  err_desc_unmap:
    kunmap(rdd->desc_page);    
  out_err:
    return ret;
}

static int pci_remote_disk_detach_device(struct pci_remote_disk_device *rdd)
{
        struct device *dev = &rdd->rcom->pdev->dev;
	struct pci_epf_blockpt_reg __iomem *base = rdd->rcom->base;
	int ret, i;
	
	writeb(rdd->id, &base->dev_idx);
	ret = pci_remote_disk_send_command(rdd->rcom, BPT_COMMAND_STOP);
	
	if (ret) {
	    dev_err(dev, "%s: cannot stop device\n", rdd->npr_name);
	    return ret;
	}

	kthread_stop(rdd->dp_thr);
	del_gendisk(rdd->gd);
	blk_mq_free_tag_set(&rdd->tag_set);
	for (i = 0; i < rdd->num_irqs; ++i) {
	    devm_free_irq(dev, rdd->irqs[i], rdd);
	    rdd->irqs[i] = -EINVAL;
	}
	kunmap(rdd->desc_page);
	__free_pages(rdd->desc_page, rdd->dp_order);

	put_disk(rdd->gd);
	rdd->attached = false;
	
	return 0;
}

static ssize_t pci_remote_disk_group_attach_store(struct config_item *item,
						     const char *page, size_t len)
{
    bool attach;
    struct pci_remote_disk_device *rdd = to_remote_disk_dev(item);
    int ret = kstrtobool(page, &attach);
    
    if (ret)
	return ret;

    if (!rdd->attached && attach)
	ret = pci_remote_disk_attach_device(rdd);
    else if (rdd->attached && !attach)
	ret = pci_remote_disk_detach_device(rdd);
    else
	ret = -EINVAL;
    
    if (ret < 0)
	return ret;
    
    return len;
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



static const struct pci_device_id pci_remote_disk_tbl[] = {
	{
		PCI_DEVICE(0x0, 0xc402),
	},
	{ 0 }
};

static int pci_rd_alloc_descriptor(struct pci_remote_disk_device *rdd)
{
        int i;
	int ret = -ENOSPC;
	struct device *dev = &rdd->rcom->pdev->dev;
	spin_lock(&rdd->lock);
	for (i = 0; i < NUM_DESRIPTORS; ++i) {
		struct pci_epf_blockpt_descr __iomem *de = &rdd->descr_ring[rdd->ns_idx];
		u32 flags = READ_ONCE(de->si.flags);
		if (!(flags & PBI_EPF_BLOCKPT_F_USED)) {
			dev_dbg(dev, "Found free descriptor at idx %i\n", rdd->ns_idx);
			WRITE_ONCE(de->si.flags, flags | PBI_EPF_BLOCKPT_F_USED);
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
	struct pci_epf_blockpt_descr __iomem *dtu;
	struct pci_blockpt_driver_ring __iomem *drv_ring = rdd->drv_ring;
	dma_addr_t dma_addr;
	char *buf;
	int err;

	rb_req->rdd = rdd;
	if (!is_valid_request(req_op(bd->rq))) {
	    dev_err(dev, "Unsupported Request: %i\n", req_op(bd->rq));
	    return BLK_STS_NOTSUPP;
	}

	descr_idx = pci_rd_alloc_descriptor(rdd);
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
	dtu->si.opf = rq_data_dir(bd->rq);
	if (dtu->si.opf == WRITE) {
	    rq_for_each_segment (bv, bd->rq, iter) {
		memcpy_from_bvec(buf, &bv);
		buf += bv.bv_len;
	    }
	}
	dtu->s_sector = blk_rq_pos(bd->rq);
	rdd->descr_tags[descr_idx] = (u64)rb_req;
	spin_lock(&rdd->lock);
	writew(descr_idx, &drv_ring->ring[rdd->drv_idx]);
	rdd->drv_idx = (rdd->drv_idx + 1) % NUM_DESRIPTORS;
	smp_wmb();
	writew(rdd->drv_idx, &drv_ring->idx);
	spin_unlock(&rdd->lock);
	dev_dbg(dev, "(DIR: %s): Adding desc %i (%i). sector: 0x%llX, len: 0x%x\n",
		 (rq_data_dir(bd->rq) == WRITE) ? "WRITE" : "READ", descr_idx, rdd->drv_idx,
		 dtu->s_sector, dtu->len);

	blk_mq_start_request(bd->rq);
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
	struct pci_remote_disk_device *rdd = dev_id;
	
	BUG_ON(!rdd->attached);

	/* wakeup the process to digest the processed request*/
	up(&rdd->dig_sem);
	
	return IRQ_HANDLED;
}

static void pci_rd_clear_descriptor(struct pci_remote_disk_device *rdd, struct pci_epf_blockpt_descr *descr, u16 descr_idx)
{
    unsigned long flags;
    
    spin_lock_irqsave(&rdd->lock, flags);
    rdd->descr_tags[descr_idx] = 0;
    memset(descr, 0, sizeof(*descr));
    spin_unlock_irqrestore(&rdd->lock, flags);
}

static int pci_remote_disk_dispatch(void *cookie)
{
    struct pci_remote_disk_device *rdd = cookie;
    struct device *dev = &rdd->rcom->pdev->dev;
    struct pci_blockpt_device_ring __iomem *dev_ring = rdd->dev_ring;
    struct req_iterator iter;
    struct bio_vec bv;
    u16 descr_idx;
    struct pci_epf_blockpt_descr *desc;
    struct pci_remote_disk_request *rb_req;
    struct request *rq;
    void *buf;
    
    while (!kthread_should_stop()) {
	down(&rdd->dig_sem);
	
	BUG_ON(readw(&dev_ring->idx) == rdd->dev_idx);
	
	descr_idx = readw(&dev_ring->ring[rdd->dev_idx]);
	desc = &rdd->descr_ring[descr_idx];
	
	BUG_ON(!(READ_ONCE(desc->si.flags) & PBI_EPF_BLOCKPT_F_USED));
	
	rb_req = (struct pci_remote_disk_request *)rdd->descr_tags[descr_idx];
	BUG_ON(rb_req == NULL);

	rq = blk_mq_rq_from_pdu(rb_req);
	    
	dev_dbg(dev, "Digest Req: sec: 0x%llX, len: 0x%x\n", desc->s_sector, desc->len);
	if (rq_data_dir(rq) == READ) {
	    buf = kmap(rb_req->pg);
	    rq_for_each_segment (bv, rq, iter) {
		memcpy_to_bvec(&bv, buf);
		dev_dbg(dev,
			" bv.len = 0x%x, bv.offset = 0x%x, page: 0x%llX. Buf: 0x%llX\n",
			bv.bv_len, bv.bv_offset,
			page_address(bv.bv_page), buf);
		buf += bv.bv_len;
	    }
	    kunmap(rb_req->pg);
	}
			
	dma_unmap_single(dev, desc->addr, desc->len,
			 rq_dma_dir(rq));
	rb_req->status = (blk_status_t) readl(&desc->si.status);
	    
	pci_rd_clear_descriptor(rdd, desc, descr_idx);
	__free_pages(rb_req->pg, rb_req->order);
	WRITE_ONCE(rdd->dev_idx, (rdd->dev_idx + 1) % NUM_DESRIPTORS);
	blk_mq_complete_request(rq);
    }

    return 0;
}

static int pci_remote_disk_parse(struct pci_remote_disk_common *rcom)
{
    struct pci_remote_disk_device *rdd;
    struct list_head *lh, *lhtmp;
    char *sbd, *ebd;
    int count = 0;
    int err, i;
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

	rdd->descr_tags = kzalloc((sizeof(u64) * NUM_DESRIPTORS), GFP_KERNEL);
	if (!rdd->descr_tags) {
	    dev_err(dev, "Could not allocate descriptor tags\n");
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
	
	spin_lock_init(&rdd->lock);
	rdd->rcom = rcom;
	rdd->id = count;

	/* get rid of all path seperators  */
	rdd->npr_name = strrchr(rdd->r_name, '/');
	rdd->npr_name = (rdd->npr_name == NULL) ? rdd->r_name : (rdd->npr_name + 1);
	rdd->l_name = kasprintf(GFP_KERNEL, "pci-rd-%s", rdd->npr_name);
	if (!rdd->l_name) {
	    dev_err(dev, "Could not allocate memory for local device name\n");
	    err = -ENOMEM;
	    goto err_free;
	}

	rdd->num_irqs = readl(&rcom->base->irqs_per_device);
	rdd->irqs = devm_kzalloc(dev, rdd->num_irqs * sizeof(int), GFP_KERNEL);
	if (rdd->irqs == NULL) {
	    dev_err(dev, "Can't alloc %i IRQs for %s\n", rdd->num_irqs, rdd->l_name);
	    goto err_free;
	}
	
	for (i = 0; i < rdd->num_irqs; ++i)
	    rdd->irqs[i] = -EINVAL;
	
	config_group_init_type_name(&rdd->cfs_group, rdd->npr_name, &pci_remote_disk_group_type);
	err = configfs_register_group(&pci_remote_disk_subsys.su_group, &rdd->cfs_group);
	if (err) {
	    dev_err(dev, "Cannot register configfs group for %s\n", rdd->npr_name);
	    err = -ENODEV;
	    goto err_free;
	}
	
	dev_info(dev, "Found %s\n", rdd->r_name);
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
	kfree(rdd->descr_tags);
	kfree(rdd);
    }
    return err;
}

static int pci_remote_disk_probe(struct pci_dev *pdev,
			       const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	int err, num, num_irqs;
	enum pci_barno bar;
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

	if (rcom->base->magic != BLOCKPT_MAGIC) {
	    dev_err(dev, "Invalid magic at BAR0 detected. Expected 0x%x. Read 0x%x\n", BLOCKPT_MAGIC, rcom->base->magic);
	    err = -ENODEV;
	    goto err_iounmap;
	}

	err = pci_remote_disk_send_command(rcom, BPT_COMMAND_GET_DEVICES);
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
	num = pci_remote_disk_parse(rcom);
	if (num <= 0) {
	    dev_err(dev, "Unable to parse any valid disk\n");
	    err = -ENODEV;
	    goto err_iounmap;
	}

	dev_info(dev, "Found %i %s\n", num, (num == 1) ? "device" : "devices");
	num_irqs = num * readl(&rcom->base->irqs_per_device);
	/* alloc one vector for each possible device */
	rcom->num_irqs = pci_alloc_irq_vectors(pdev, 1, num_irqs, PCI_IRQ_MSIX | PCI_IRQ_MSI);
	if (rcom->num_irqs < num_irqs)
	    dev_err(dev, "Failed to get %i MSI-X interrupts: Returned %i\n", num_irqs, rcom->num_irqs);

	dev_info(dev, "Allocated %i IRQ Vectors\n", rcom->num_irqs);
	pci_set_drvdata(pdev, rcom);
	return 0;

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

static void pci_remote_disk_remove(struct pci_dev *pdev)
{
    struct device *dev = &pdev->dev;
    struct pci_remote_disk_common *rcom = pci_get_drvdata(pdev);
    struct pci_epf_blockpt_reg __iomem *base = rcom->base;
    struct pci_remote_disk_device *rdd, *tmp_rdd;
    int i, ret;
    
    list_for_each_entry_safe(rdd, tmp_rdd, &available_remote_disks, node) {
	if (rdd->attached) {
	    writeb(rdd->id, &base->dev_idx);
	    ret = pci_remote_disk_send_command(rdd->rcom, BPT_COMMAND_STOP);
	    if (ret) 
		dev_err(dev, "%s: cannot stop device\n", rdd->npr_name);
	    kthread_stop(rdd->dp_thr);
	    
	    del_gendisk(rdd->gd);
	    blk_mq_free_tag_set(&rdd->tag_set);
	    
	    kunmap(rdd->desc_page);
	    __free_pages(rdd->desc_page, rdd->dp_order);
	}
	
	kfree(rdd->r_name);
	kfree(rdd->l_name);
	configfs_unregister_group(&rdd->cfs_group);
	for (i = 0; i < rdd->num_irqs; ++i) {
	    if (rdd->irqs[i] != -EINVAL)
		devm_free_irq(dev, rdd->irqs[i], rdd);
	}
	list_del(&rdd->node);
	kfree(rdd->descr_tags);
	kfree(rdd);
    }

    configfs_unregister_subsystem(&pci_remote_disk_subsys);
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

MODULE_DEVICE_TABLE(pci, pci_remote_disk_tbl);

static struct pci_driver pci_remote_disk_driver = {
	.name = DRV_MODULE_NAME,
	.id_table = pci_remote_disk_tbl,
	.probe = pci_remote_disk_probe,
	.remove = pci_remote_disk_remove,
	.sriov_configure = pci_sriov_configure_simple,
};

module_pci_driver(pci_remote_disk_driver);

MODULE_AUTHOR("Wadim Mueller <wadim.mueller@continental.com>");
MODULE_DESCRIPTION("Remote PCI Endpoint Disk driver");
MODULE_LICENSE("GPL v2");
