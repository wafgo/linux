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

static char *block_dev_name = "/dev/vdb";
module_param(block_dev_name, charp, 0444);

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
	struct block_device *real_bd;
	char *device_path;
	spinlock_t lock;
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

static const struct pci_remote_bd_data default_data = {
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
	REM_BD_PCI_DEVICE(0x0005, (kernel_ulong_t)&default_data),
	{ 0 }
};

static void pci_rbd_unmap_data(struct pci_remote_bd_request *rdr);
static int pci_rbd_map_data(struct blk_mq_hw_ctx *hctx, struct request *req,
			    struct pci_remote_bd_request *rdr);

static int major;
static struct workqueue_struct *deferred_bio_add_workqueue;

static void read_complete(struct bio *bio);

static void pci_rbd_unmap_data(struct pci_remote_bd_request *rdr);

static void deferred_bio_work_func(struct work_struct *work)
{
	struct pci_remote_bd_request *rb_req =
		container_of(work, struct pci_remote_bd_request, bio_work);
	struct request *rq = blk_mq_rq_from_pdu(rb_req);
	struct pci_remote_block_device *rdev = rb_req->rdev;

	struct bio *bio_src;
	rb_req->num_bios = 0;

	__rq_for_each_bio (bio_src, rq) {
		struct bio *bt = bio_alloc(GFP_KERNEL, 1);
		if (bt) {
			bt = bio_clone_fast(bio_src, GFP_NOIO, &rdev->bset);
			bio_set_dev(bt, rdev->real_bd);
			bt->bi_end_io = read_complete;
			bt->bi_private = rb_req;
			rb_req->num_bios++;
			bio_get(bt);
			submit_bio(bt);
		} else {
			// FIXME
			dev_err(&rdev->pdev->dev, "Could not alloc bio\n");
			break;
		}
	}

	return;
}

static void read_complete(struct bio *bio)
{
	struct pci_remote_bd_request *rbd = bio->bi_private;

	struct request *req = blk_mq_rq_from_pdu(rbd);
	if (--rbd->num_bios == 0) {
		pci_rbd_unmap_data(rbd);
		blk_mq_complete_request(req);
	}
	bio_put(bio);
}

static void pci_rbd_unmap_data(struct pci_remote_bd_request *rdr)
{
	struct request *req = blk_mq_rq_from_pdu(rdr);
	if (blk_rq_nr_phys_segments(req))
		sg_free_table_chained(&rdr->sg_table, PCI_RBD_INLINE_SG_CNT);
}

static int pci_rbd_map_data(struct blk_mq_hw_ctx *hctx, struct request *req,
			    struct pci_remote_bd_request *rdr)
{
	int err;
	if (!blk_rq_nr_phys_segments(req))
		return 0;
	rdr->sg_table.sgl = rdr->sg;
	err = sg_alloc_table_chained(&rdr->sg_table,
				     blk_rq_nr_phys_segments(req),
				     rdr->sg_table.sgl, PCI_RBD_INLINE_SG_CNT);
	if (unlikely(err))
		return -ENOMEM;

	return blk_rq_map_sg(hctx->queue, req, rdr->sg_table.sgl);
}

static blk_status_t pci_rbd_queue_rq(struct blk_mq_hw_ctx *hctx,
				     const struct blk_mq_queue_data *bd)
{
	struct pci_remote_block_device *rdev = hctx->queue->queuedata;
	struct pci_remote_bd_request *rb_req = blk_mq_rq_to_pdu(bd->rq);
	int num;

	rb_req->rdev = rdev;
	blk_mq_start_request(bd->rq);
	num = pci_rbd_map_data(hctx, bd->rq, rb_req);

	if (unlikely(num < 0))
		return BLK_STS_RESOURCE;

	INIT_WORK(&rb_req->bio_work, deferred_bio_work_func);
	queue_work(deferred_bio_add_workqueue, &rb_req->bio_work);

	return BLK_STS_OK;
}

static void pci_rbd_end_rq(struct request *rq)
{
	blk_mq_end_request(rq, BLK_STS_OK);
}

static enum blk_eh_timer_return pci_rbd_timeout_rq(struct request *rq, bool res)
{
	blk_mq_complete_request(rq);
	return BLK_EH_DONE;
}

static const struct blk_mq_ops pci_rbd_mq_ops = { .queue_rq = pci_rbd_queue_rq,
						  .complete = pci_rbd_end_rq,
						  .timeout =
							  pci_rbd_timeout_rq };

static int pci_rbd_open(struct block_device *bdev, fmode_t mode)
{
	int ret = 0;
	struct pci_remote_block_device *rdev = bdev->bd_disk->private_data;

	rdev->device_path = block_dev_name;
	rdev->real_bd = blkdev_get_by_path(rdev->device_path,
					   FMODE_READ | FMODE_WRITE, NULL);
	if (IS_ERR(rdev->real_bd)) {
		ret = PTR_ERR(rdev->real_bd);
		if (ret != -ENOTBLK) {
			dev_err(&rdev->pdev->dev,
				"failed to open block device %s: (%ld)\n",
				rdev->device_path, PTR_ERR(rdev->real_bd));
		}
		return -ENXIO;
	}
	return 0;
}

static void pci_rbd_release(struct gendisk *disk, fmode_t mode)
{
}

static int pci_rbd_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	geo->heads = 4;
	geo->sectors = 16;
	geo->cylinders =
		get_capacity(bdev->bd_disk) / (geo->heads * geo->sectors);
	return 0;
}

static int pci_rbd_ioctl(struct block_device *bdev, fmode_t mode,
			 unsigned int cmd, unsigned long arg)
{
	int dir = _IOC_DIR(cmd);
	int tp = _IOC_TYPE(cmd);
	int nr = _IOC_NR(cmd);
	int sz = _IOC_SIZE(cmd);

	printk(KERN_ERR
	       "-------> IOCTL received %d. R/WR: 0x%x, TYPE: 0x%x, NR: 0x%x, SIZE: 0x%x\n",
	       cmd, dir, tp, nr, sz);
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

static int pci_remote_bd_probe(struct pci_dev *pdev,
			       const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	int err;
	struct pci_remote_block_device *rdev =
		devm_kzalloc(dev, sizeof(*rdev), GFP_KERNEL);
	if (!rdev)
		return -ENOMEM;

	rdev->device_path = block_dev_name;
	rdev->real_bd = blkdev_get_by_path(rdev->device_path,
					   FMODE_READ | FMODE_WRITE, NULL);
	if (IS_ERR(rdev->real_bd)) {
		err = PTR_ERR(rdev->real_bd);
		if (err != -ENOTBLK) {
			dev_err(&rdev->pdev->dev,
				"failed to open block device %s: (%ld)\n",
				rdev->device_path, PTR_ERR(rdev->real_bd));
		}
		goto out_free_dev;
	}

	deferred_bio_add_workqueue =
		alloc_workqueue("pci_rbd_bio_add", WQ_UNBOUND, 1);
	if (!deferred_bio_add_workqueue) {
		err = -ENOMEM;
		goto out_free_dev;
	}

	rdev->pdev = pdev;
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

	snprintf(rdev->gd->disk_name, 32, "pci_rbd");
	set_capacity(rdev->gd,
		     get_capacity(bdev_whole(rdev->real_bd)->bd_disk));
	device_add_disk(dev, rdev->gd, NULL);
	return 0;
out_tag_set:
	blk_mq_free_tag_set(&rdev->tag_set);
out_workqueue:
	destroy_workqueue(deferred_bio_add_workqueue);
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
