#include <linux/ceph/osd_client.h>
#include <linux/ceph/mon_client.h>
#include <linux/ceph/cls_lock_client.h>
#include <linux/ceph/striper.h>
#include <linux/ceph/decode.h>
#include <linux/fs_parser.h>
#include <linux/bsearch.h>

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

#define PCI_REMOTE_BLOCK_MINORS 16
#define PCI_REMOTE_BLKDEV_MAJOR 290
#define PCI_REMOTE_BLKDEV_NAME "remote-bd"
#define DRV_MODULE_NAME "pci-remote-bdev"

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
	/* Block layer tags. */
	struct blk_mq_tag_set tag_set;
	spinlock_t lock;
};

struct pci_remote_bd_request {
	int dummy;
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

static blk_status_t pci_rbd_queue_rq(struct blk_mq_hw_ctx *hctx,
				     const struct blk_mq_queue_data *bd)
{
	/* struct pci_remote_block_device *rbd = hctx->queue->queuedata; */
	/* struct req_iterator iter; */
	/* struct bio_vec bvec; */
	/* unsigned int len; */
	/* int err; */
	/* struct request *rq = bd->rq; */
	/* sector_t sector = blk_rq_pos(rq); */
	/* void *dst; */
	/* size_t temp, count = 0; */
	/* unsigned int offset; */
	/* /\* struct rbd_img_request *img_req = blk_mq_rq_to_pdu(bd->rq); *\/ */
	/* struct device *dev = &rbd->pdev->dev; */

	blk_mq_start_request(bd->rq);
	switch (req_op(bd->rq)) {
	case REQ_OP_DISCARD:
		printk(KERN_ERR "----> DISCARD received\n");
		break;
	case REQ_OP_WRITE_ZEROES:
		printk(KERN_ERR "----> REQ_OP_WRITE_ZEROES received\n");
		break;
	case REQ_OP_WRITE:
		printk(KERN_ERR "----> WRITE received, %d\n",
		       blk_rq_bytes(bd->rq));
		break;
	case REQ_OP_READ:
		printk(KERN_ERR "----> READ received %d\n",
		       blk_rq_bytes(bd->rq));
		/* spin_lock_irq(&rbd->lock); */
		/* rq_for_each_segment (bvec, rq, iter) { */
		/* 	len = bvec.bv_len; */
		/* 	while(count < len) { */
		/* 	    size_t temp = min_t(size_t, 512, len - count); */
		/* 	    offset = (sector & SECTOR_MASK) << SECTOR_SHIFT; */
		/* 	    dst = kmap_atomic(bvec.bv_page); */
		/* 	    memset(dst + bvec.bv_offset + count, 0xab, temp); */
		/* 	    kunmap_atomic(dst); */
		/* 	    count += temp; */
		/* 	    sector += temp >> SECTOR_SHIFT; */
		/* 	} */
		/* 	sector += len >> SECTOR_SHIFT; */
		/* } */
		/* spin_unlock_irq(&rbd->lock); */

		break;
	default:
		printk(KERN_ERR "unknown req_op %d", req_op(bd->rq));
		return BLK_STS_IOERR;
	}

	blk_mq_complete_request(bd->rq);
	return BLK_STS_OK;
}

static void pci_rbd_end_rq(struct request *rq)
{
	printk(KERN_ERR "--------COMPLETE\n");
	blk_mq_end_request(rq, BLK_STS_OK);
}

static enum blk_eh_timer_return pci_rbd_timeout_rq(struct request *rq, bool res)
{
	printk(KERN_ERR "--------TIMEOUT\n");
	blk_mq_complete_request(rq);
	return BLK_EH_DONE;
}

static int pci_rbd_init_hctx(struct blk_mq_hw_ctx *hctx, void *driver_data,
			     unsigned int hctx_idx)
{
	printk(KERN_ERR "--------INIT\n");
	return 0;
}

static void pci_rbd_exit_hctx(struct blk_mq_hw_ctx *hctx, unsigned int hctx_idx)
{
	printk(KERN_ERR "--------EXIT\n");
}

static const struct blk_mq_ops pci_rbd_mq_ops = {
	.queue_rq = pci_rbd_queue_rq,
	.complete = pci_rbd_end_rq,
	.init_hctx = pci_rbd_init_hctx,
	.exit_hctx = pci_rbd_exit_hctx,
	.timeout = pci_rbd_timeout_rq
};

/* static const struct attribute_group pci_rbdisk_attr_group = { */
/* 	.attrs = mmc_disk_attrs, */
/* }; */

/* static const struct attribute_group *pci_rbdisk_attr_groups[] = { */
/* 	&pci_rbdisk_attr_group, */
/* 	NULL, */
/* }; */

static int pci_rbd_open(struct block_device *bdev, fmode_t mode)
{
	struct pci_remote_block_device *rdev = bdev->bd_disk->private_data;
	/* struct device *dev = &rdev->pdev->dev; */
	/* int ret = -ENXIO; */

	printk(KERN_ERR "-----------> OPEN CALLED\n");

	return 0;
}

static void pci_rbd_release(struct gendisk *disk, fmode_t mode)
{
	/* struct pci_remote_block_device *rdev = disk->private_data; */
	/* struct device *dev = &rdev->pdev->dev; */
	printk(KERN_ERR "-----------> RELEASE CALLED\n");
	//dev_err(dev, "-----------> RELEASE CALLED\n");
}

static int pci_rbd_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	/* struct pci_remote_block_device *rdev = bdev->bd_disk->private_data; */
	/* struct device *dev = &rdev->pdev->dev; */
	printk(KERN_ERR "-----------> GET GEO CALLED\n");
	geo->cylinders = get_capacity(bdev->bd_disk) / (4 * 16);
	geo->heads = 4;
	geo->sectors = 16;
	return -EINVAL;
}

static int pci_rbd_ioctl(struct block_device *bdev, fmode_t mode,
			 unsigned int cmd, unsigned long arg)
{
	struct pci_remote_block_device *rdev = bdev->bd_disk->private_data;

	int dir = _IOC_DIR(cmd);
	int tp = _IOC_TYPE(cmd);
	int nr = _IOC_NR(cmd);
	int sz = _IOC_SIZE(cmd);

	printk(KERN_ERR "-------> IOCTL received %d. R/WR: 0x%x, TYPE: 0x%x, NR: 0x%x, SIZE: 0x%x\n", cmd, dir, tp, nr, sz);
	return -EINVAL;
}

#ifdef CONFIG_COMPAT
static int pci_rbd_compat_ioctl(struct block_device *bdev, fmode_t mode,
				unsigned int cmd, unsigned long arg)
{
	printk(KERN_ERR "-------> COMPAT IOCTL received %d\n", cmd);
	return pci_rbd_ioctl(bdev, mode, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct block_device_operations pci_rbd_bdops = {
	.open = pci_rbd_open,
	.release = pci_rbd_release,
	/* .getgeo = pci_rbd_getgeo, */
	.owner = THIS_MODULE,
	.ioctl = pci_rbd_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pci_rbd_compat_ioctl,
#endif
};

static int pci_remote_bd_probe(struct pci_dev *pdev,
			       const struct pci_device_id *ent)
{
	int status;
	struct device *dev = &pdev->dev;
	int err;
	struct pci_remote_block_device *rdev =
		devm_kzalloc(dev, sizeof(*rdev), GFP_KERNEL);
	if (!rdev)
		return -ENOMEM;

	rdev->pdev = pdev;

	dev_err(dev, "Probe from %s\n", __FUNCTION__);
	status = register_blkdev(PCI_REMOTE_BLKDEV_MAJOR,
				 PCI_REMOTE_BLKDEV_NAME);
	if (status < 0) {
		dev_err(dev, "unable to register mybdev block device\n");
		return -EBUSY;
	}

	spin_lock_init(&rdev->lock);
	rdev->tag_set.ops = &pci_rbd_mq_ops;
	// FIXME
	rdev->tag_set.queue_depth = 128;
	rdev->tag_set.numa_node = NUMA_NO_NODE;
	rdev->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
	rdev->tag_set.nr_hw_queues = 1; //num_present_cpus();
	rdev->tag_set.cmd_size = sizeof(struct pci_remote_bd_request);
	rdev->tag_set.driver_data = rdev;

	err = blk_mq_alloc_tag_set(&rdev->tag_set);
	if (err)
		goto out_free_dev;

	rdev->gd = blk_mq_alloc_disk(&rdev->tag_set, rdev);
	if (IS_ERR(rdev->gd)) {
		err = -ENODEV;
		goto out_tag_set;
	}

	rdev->gd->major = PCI_REMOTE_BLKDEV_MAJOR;
	rdev->gd->minors = PCI_REMOTE_BLOCK_MINORS;
	rdev->gd->first_minor = 1;
	rdev->gd->fops = &pci_rbd_bdops;
	rdev->gd->private_data = rdev;
	rdev->gd->queue->queuedata = rdev;

	snprintf(rdev->gd->disk_name, 32, "pci_rbd");
	set_capacity(rdev->gd, (1024 * 1024 * 1024) / SECTOR_SIZE);
	device_add_disk(dev, rdev->gd, NULL);
	return 0;
out_tag_set:
	blk_mq_free_tag_set(&rdev->tag_set);
out_free_dev:
	devm_kfree(dev, rdev);
	return err;
}

static void pci_remote_bd_remove(struct pci_dev *pdev)
{
	struct device *dev = &pdev->dev;
	dev_err(dev, "Remove from %s\n", __FUNCTION__);
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
