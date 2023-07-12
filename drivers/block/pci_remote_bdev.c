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
	struct block_device *real_bd;
	char *device_path;
	spinlock_t lock;
	const struct blk_mq_queue_data *bd;
};

struct pci_remote_bd_request {
        struct pci_remote_block_device *rdev;
	struct bio *bio;
	u8 status;
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

static void pci_rbd_unmap_data(struct request *req, struct pci_remote_bd_request *rdr);
static int pci_rbd_map_data(struct blk_mq_hw_ctx *hctx, struct request *req,struct pci_remote_bd_request *rdr);

static int major;

static char test_array[8 * PAGE_SIZE];
static void read_complete(struct bio *bio)
{
	struct pci_remote_bd_request *rbd = bio->bi_private;
	struct request *req = blk_mq_rq_from_pdu(rbd);
	struct device *dev = &rbd->rdev->pdev->dev;
	int nbytes = sg_copy_to_buffer(rbd->sg_table.sgl, sg_nents(rbd->sg_table.sgl),
				       test_array, sizeof(test_array));
	/* struct bio_vec bv; */
	/* struct bvec_iter iter; */
	/* int i = 0; */
	dev_err(dev, "Finished reading bio %d bytes\n", nbytes);
	/* void *kaddr = kmap(&pg[1]); */
	/* printk(KERN_ALERT "--NUM OF VECS: %d\n", bio_sectors(bio)); */
	/* /\* bio_for_each_segment(bv, bio, iter) { *\/ */
	/* /\* 	printk(KERN_ALERT "--READING VEC\n"); *\/ */
	/* /\* 	memcpy_from_bvec(my_local_read_array, &bv); *\/ */
	/* /\* } *\/ */
	print_hex_dump(KERN_ALERT, "", DUMP_PREFIX_OFFSET, 16, 1,
			       test_array, nbytes, true);
	/* kunmap(&pg[1]); */

	/* printk(KERN_ALERT "-----------> FIN READING PAGE \n"); */
	//    free_page((unsigned long) page_address(&pg[1]));
	//bio_free_pages(bio);
	pci_rbd_unmap_data(req, rbd);
	blk_mq_complete_request(req);
	dev_err(dev, "Finished reading bio END\n");
}

#define PCI_RBD_INLINE_SG_CNT 2

static void pci_rbd_unmap_data(struct request *req, struct pci_remote_bd_request *rdr)
{
	if (blk_rq_nr_phys_segments(req))
		sg_free_table_chained(&rdr->sg_table,PCI_RBD_INLINE_SG_CNT);
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
	int pages_order;
	struct page *pg;
	rb_req->rdev = rdev;
	blk_mq_start_request(bd->rq);
	num = pci_rbd_map_data(hctx, bd->rq, rb_req);
	
	if (unlikely(num < 0)) 
		return BLK_STS_RESOURCE;

	switch (req_op(bd->rq)) {
	case REQ_OP_DISCARD:
		dev_err(&rdev->pdev->dev, "----> DISCARD received\n");
		break;
	case REQ_OP_WRITE_ZEROES:
		dev_err(&rdev->pdev->dev, "----> REQ_OP_WRITE_ZEROES received\n");
		break;
	case REQ_OP_WRITE:
		printk(KERN_ERR "----> WRITE received, %d\n", blk_rq_bytes(bd->rq));
		break;
	case REQ_OP_READ:
		printk(KERN_ERR "----> READ received %d --- current sector: %d\n",
		       blk_rq_bytes(bd->rq), blk_rq_pos(bd->rq));
		pages_order = get_order(blk_rq_bytes(bd->rq));
		dev_err(&rdev->pdev->dev, "Order for %d bytes is %d\n",
			blk_rq_bytes(bd->rq), pages_order);
		pg = alloc_pages(GFP_KERNEL, pages_order);
		if (pg) {
			dev_err(&rdev->pdev->dev,
				"Successfully allocated %d order pages\n",
				pages_order);
			// WHAT is the no_of_vecs?
			rb_req->bio = bio_alloc(GFP_NOIO, 1);
			if (!rb_req->bio) {
				dev_err(&rdev->pdev->dev,
					"Cannot allocate bio %d\n",
					bio_sectors(rb_req->bio));
				free_pages((unsigned long)page_address(pg), 1);
				return BLK_STS_IOERR;
			} else {
				rb_req->bio->bi_private = rb_req;
				bio_set_dev(rb_req->bio, rdev->real_bd);
				rb_req->bio->bi_iter.bi_sector = blk_rq_pos(bd->rq);
				rb_req->bio->bi_opf = REQ_OP_READ;
				bio_add_page(rb_req->bio, pg, blk_rq_bytes(bd->rq), 0);
				rb_req->bio->bi_end_io = read_complete;
				rb_req->bio->bi_ioprio = bd->rq->bio->bi_ioprio + 10;
				submit_bio(rb_req->bio);
			}
		} else {
			dev_err(&rdev->pdev->dev,
				"Could not get Pages for read \n",
				__FUNCTION__);
		}
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
	//blk_mq_complete_request(bd->rq);

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
	int ret = 0;
	struct pci_remote_block_device *rdev = bdev->bd_disk->private_data;

	rdev->device_path = kzalloc(64, GFP_KERNEL);
	snprintf(rdev->device_path, 64, "/dev/vdb");
	if (!rdev->device_path) {
		dev_err(&rdev->pdev->dev, "OOM in string allocation\n");
		return -ENXIO;
	}

	rdev->real_bd = blkdev_get_by_path(rdev->device_path,
					   FMODE_READ | FMODE_WRITE, NULL);
	if (IS_ERR(rdev->real_bd)) {
		ret = PTR_ERR(rdev->real_bd);
		if (ret != -ENOTBLK) {
			dev_err(&rdev->pdev->dev,
				"failed to open block device %s: (%ld)\n",
				rdev->device_path, PTR_ERR(rdev->real_bd));
		}
		kfree(rdev->device_path);
		return -ENXIO;
	}
	dev_err(&rdev->pdev->dev, "%s: Got reference to %s\n", __FUNCTION__,
		rdev->device_path);
	return 0;
}

static void pci_rbd_release(struct gendisk *disk, fmode_t mode)
{
	struct pci_remote_block_device *rdev =
		(struct pci_remote_block_device *)disk->private_data;
	if (rdev->device_path)
		kfree(rdev->device_path);
	dev_err(&rdev->pdev->dev, "%s: Releasing %s\n", __FUNCTION__,
		rdev->device_path);
	/* struct pci_remote_block_device *rdev = disk->private_data; */
	/* struct device *dev = &rdev->pdev->dev; */
	// printk(KERN_ERR "-----------> RELEASE CALLED\n");
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

	printk(KERN_ERR
	       "-------> IOCTL received %d. R/WR: 0x%x, TYPE: 0x%x, NR: 0x%x, SIZE: 0x%x\n",
	       cmd, dir, tp, nr, sz);
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
	struct device *dev = &pdev->dev;
	int err, ret;
	struct pci_remote_block_device *rdev =
		devm_kzalloc(dev, sizeof(*rdev), GFP_KERNEL);
	if (!rdev)
		return -ENOMEM;

	rdev->pdev = pdev;
	pci_set_drvdata(pdev, rdev);
	dev_err(dev, "Probe from %s\n", __FUNCTION__);
	major = register_blkdev(0, PCI_REMOTE_BLKDEV_NAME);
	if (major < 0) {
		dev_err(dev, "unable to register remote block device\n");
		return -EBUSY;
	}

	spin_lock_init(&rdev->lock);
	rdev->tag_set.ops = &pci_rbd_mq_ops;
	// FIXME
	rdev->tag_set.queue_depth = 128;
	rdev->tag_set.numa_node = NUMA_NO_NODE;
	rdev->tag_set.flags = BLK_MQ_F_SHOULD_MERGE;
	rdev->tag_set.nr_hw_queues = 1; //num_present_cpus();
	rdev->tag_set.cmd_size = sizeof(struct pci_remote_bd_request) + sizeof(struct scatterlist) * PCI_RBD_INLINE_SG_CNT;
	rdev->tag_set.driver_data = rdev;

	err = blk_mq_alloc_tag_set(&rdev->tag_set);
	if (err)
		goto out_free_dev;

	rdev->gd = blk_mq_alloc_disk(&rdev->tag_set, rdev);
	if (IS_ERR(rdev->gd)) {
		err = -ENODEV;
		goto out_tag_set;
	}

	rdev->gd->major = major;
	rdev->gd->minors = PCI_REMOTE_BLOCK_MINORS;
	rdev->gd->first_minor = 1;
	rdev->gd->fops = &pci_rbd_bdops;
	rdev->gd->private_data = rdev;
	rdev->gd->queue->queuedata = rdev;

	snprintf(rdev->gd->disk_name, 32, "pci_rbd");
	set_capacity(rdev->gd, (2024 * 1024 * 1024) / SECTOR_SIZE);
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
	struct pci_remote_block_device *rdev = pci_get_drvdata(pdev);

	del_gendisk(rdev->gd);
	blk_cleanup_disk(rdev->gd);
	blk_mq_free_tag_set(&rdev->tag_set);
	dev_err(dev, "Remove from %s\n", __FUNCTION__);
	devm_kfree(dev, rdev);
	unregister_blkdev(major, PCI_REMOTE_BLKDEV_NAME);
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
