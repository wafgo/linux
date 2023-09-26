// SPDX-License-Identifier: GPL-2.0
/*
 * Block Device Passthrough as an Endpoint Function driver
 *
 * Copyright (C) 2023 Continental Automotive Technologies
 * Author: Wadim Mueller <wadim.mueller@continental.com>
 */

/*
 * PCI Block Device Passthrough allows one Linux Device to expose its Block devices to the PCI(e) host.
 * The device can export either the full disk or just certain partitions. 
 * The PCI Block Passthrough function driver is the part running on SoC2 from the diagram below. For more details refer
 * to Documentation/PCI/endpoint/pci-endpoint-block-passthru-function.rst
 *
 *                                                   +-------------+  
 *                                                   |             |
 *                                                   |   SD Card   |  
 *                                                   |             |  
 *                                                   +------^------+  
 *                                                          |                                                            
 *		                                            |
 *    +---------------------+                +--------------v------------+       +---------+
 *    |                     |                |                           |       |         |
 *    |      SoC1 (RC)      |<-------------->|        SoC2 (EP)          |<----->|  eMMC   |
 *    |  (pci-remote-disk)  |                | (pci-epf-block-passthru)  |       |         |
 *    |                     |                |                           |       +---------+
 *    +---------------------+                +--------------^------------+       
 *                                                          |
 *                                                          |
 *                                                   +------v------+  
 *                                                   |             |
 *                                                   |    NVMe     |  
 *                                                   |             |  
 *                                                   +-------------+
 *
 */

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pci_ids.h>
#include <linux/pci-epc.h>
#include <linux/pci-epf.h>
#include <linux/pci_regs.h>
#include <linux/bvec.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/blk-mq.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pci.h>
#include <linux/hdreg.h>
#include <linux/kthread.h>
#include <linux/pci-epf-block-passthru.h>


/* #define USE_DMAENGINE */

#define DRV_RING_POLL_INTERVAL_MS (1)

static struct workqueue_struct *kpciblockpt_wq;

struct pci_blockpt_device_common;

struct pci_blockt_dispatch_descriptor {
    struct list_head node;
    u16 desc_idx;
};

struct pci_epf_blockpt_device {
        struct list_head node;
        struct pci_blockpt_device_common *dcommon;
        struct config_group cfg_grp;
        char *cfs_disk_name;
        struct pci_epf_blockpt_descr __iomem *descr;
	struct block_device *bd;
        int dev_tag;
        dma_addr_t descr_addr;
        u32 descr_size;
        int max_cpu;
        struct pci_blockpt_driver_ring __iomem *driver_ring;
        struct pci_blockpt_device_ring __iomem *device_ring;
        u32 drv_idx;
        u32 dev_idx;
        u32 num_desc;
	char *device_path;
        char *dev_name;
        bool read_only;
        bool attached;

#ifdef USE_DMAENGINE	
        struct dma_chan *dma_chan;
#endif
        struct task_struct *digest_thr[NR_CPUS];
        struct task_struct *produce_thr[NR_CPUS];
        struct task_struct *dispatch_thr;
    
        struct list_head __percpu *proc_list;
        struct list_head __percpu *disp_list;
    
	spinlock_t __percpu *proc_lock;
        spinlock_t __percpu *disp_lock;

        struct semaphore __percpu *proc_sem;
        struct semaphore __percpu *disp_sem;

        int __percpu *irq;
        spinlock_t dev_lock;
        spinlock_t nm_lock; 
};

struct pci_blockpt_device_common {
        struct pci_epf_blockpt_reg __iomem *bpt_regs;
	struct pci_epf		*epf;
	enum pci_barno		blockpt_reg_bar;
	size_t			msix_table_offset;
	struct delayed_work	cmd_handler;
        struct list_head        devices;
	const struct pci_epc_features *epc_features;
        int next_disc_idx;
};

static LIST_HEAD(exportable_bds);

static struct pci_epf_header pci_blockpt_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

struct pci_epf_blockpt_info {
    struct list_head node;
    struct pci_epf_blockpt_device *bpt_dev;
    struct page *page;
    size_t page_order;
    size_t size;
    struct bio *bio;
    dma_addr_t dma_addr;
    struct completion	dma_transfer_complete;
    struct pci_epf_blockpt_descr __iomem *descr;
    int descr_idx;
    void __iomem *addr;
    phys_addr_t phys_addr;
    enum dma_data_direction dma_dir;
    int cpu;
};
#define blockpt_retry_delay()  usleep_range(100, 500)

static int pci_blockpt_digest_descriptor_kthread(void *);
static int pci_blockpt_bio_submit_kthread(void *);
static int pci_blockpt_dispatch_desc_on_cpu_kthread(void *);

static int pci_epf_blockpt_map_queue(struct pci_epf_blockpt_device *bpt_dev, struct pci_epf_blockpt_reg *reg) {
        int ret;
	void __iomem *addr;
	phys_addr_t phys_addr;
	struct pci_epf *epf = bpt_dev->dcommon->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;

	addr = pci_epc_mem_alloc_addr(epc, &phys_addr, bpt_dev->descr_size);
	if (!addr) {
		dev_err(dev, "Failed to allocate queue address\n");
		return -ENOMEM;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, phys_addr,
			       bpt_dev->descr_addr, bpt_dev->descr_size);
	if (ret) {
		dev_err(dev, "Failed to map queue address\n");
		ret = -EINVAL;
		goto err_queue_addr;
	}

	bpt_dev->descr = addr;
	bpt_dev->driver_ring = (struct pci_blockpt_driver_ring *)((u64)addr + readl(&reg->drv_offset));
	bpt_dev->device_ring = (struct pci_blockpt_device_ring *)((u64)addr + readl(&reg->dev_offset));

	dev_dbg(dev, "Queue => Queue Address physical: 0x%llX\t Queue Address virt: 0x%llX\t Queue PCI Addr: 0x%llX\n",
		phys_addr, addr, bpt_dev->descr_addr);
	return 0;
	
err_queue_addr:
	pci_epc_mem_free_addr(epc, phys_addr, addr, bpt_dev->descr_size);
	return ret;
}


static void pci_epf_blockpt_set_invalid_id_error(struct pci_blockpt_device_common *dcommon, struct pci_epf_blockpt_reg *reg)
{
    struct pci_epf *epf = dcommon->epf;
    struct device *dev = &epf->dev;
    
    dev_err(dev, "Could not find device with id: %i\n", readb(&reg->dev_idx));
    writel(BPT_STATUS_ERROR, &reg->status);
}

static struct pci_epf_blockpt_device *pci_epf_blockpt_get_device_by_id(struct pci_blockpt_device_common *dcom, u8 id)
{
    struct list_head *lh;
    struct pci_epf_blockpt_device *bpt_dev;
    
    list_for_each(lh, &exportable_bds) {
	bpt_dev = list_entry(lh, struct pci_epf_blockpt_device, node);
	if (bpt_dev->dev_tag == id)
	    return bpt_dev;
    }

    list_for_each(lh, &dcom->devices) {
	bpt_dev = list_entry(lh, struct pci_epf_blockpt_device, node);
	if (bpt_dev->dev_tag == id)
	    return bpt_dev;
    }

    return NULL;
}

static void move_bpt_device_to_active_list(struct pci_epf_blockpt_device *bpt_dev)
{
    spin_lock(&bpt_dev->nm_lock);
    list_del(&bpt_dev->node);
    INIT_LIST_HEAD(&bpt_dev->node);
    list_add_tail(&bpt_dev->node, &bpt_dev->dcommon->devices);
    spin_unlock(&bpt_dev->nm_lock);
}

static void move_bpt_device_to_exportable_list(struct pci_epf_blockpt_device *bpt_dev)
{
    spin_lock(&bpt_dev->nm_lock);
    list_del(&bpt_dev->node);
    INIT_LIST_HEAD(&bpt_dev->node);
    list_add_tail(&bpt_dev->node, &exportable_bds);
    spin_unlock(&bpt_dev->nm_lock);
}

static void free_pci_blockpt_info(struct pci_epf_blockpt_info *info)
{
    struct pci_blockpt_device_common *dcommon = info->bpt_dev->dcommon;
    struct device *dev = &dcommon->epf->dev;
    struct device *dma_dev = dcommon->epf->epc->dev.parent;
    spinlock_t *lock = this_cpu_ptr(info->bpt_dev->proc_lock);

    dma_unmap_single(dma_dev, info->dma_addr, info->size, info->dma_dir);
    if (info->bio->bi_opf == REQ_OP_READ) {
	pci_epc_unmap_addr(dcommon->epf->epc, dcommon->epf->func_no,
			   dcommon->epf->vfunc_no, info->phys_addr);
	pci_epc_mem_free_addr(dcommon->epf->epc, info->phys_addr,
			      info->addr, info->size);
    }
    
    __free_pages(info->page, info->page_order);
    
    spin_lock_irq(lock);
    list_del(&info->node);
    spin_unlock_irq(lock);
    
    bio_put(info->bio);    
    devm_kfree(dev, info);
}

struct pci_epf_blockpt_info *alloc_pci_epf_blockpt_info(struct pci_epf_blockpt_device *bpt_dev, size_t size, struct pci_epf_blockpt_descr __iomem *descr, int descr_idx, enum dma_data_direction dma_dir)
{
    struct pci_epf_blockpt_info *binfo;
    struct pci_blockpt_device_common *dcommon = bpt_dev->dcommon;
    struct bio *bio;
    struct device *dev = &dcommon->epf->dev;
    struct page *page;
    struct device *dma_dev = dcommon->epf->epc->dev.parent;
    dma_addr_t dma_addr;
    gfp_t alloc_flags = GFP_KERNEL;
    
    binfo = devm_kzalloc(dev, sizeof(*binfo), alloc_flags);
    if (unlikely(!binfo)) {
	dev_err(dev, "Could not allocate BIO INFO\n");
	return NULL;
    }

    INIT_LIST_HEAD(&binfo->node);
    bio = bio_alloc(alloc_flags, 1);
    if (unlikely(!bio)) {
	dev_err(dev, "Could not allocate BIO\n");
	goto free_binfo;
    }
    
    binfo->size = size;
    binfo->page_order = get_order(size);
    page = alloc_pages(alloc_flags | GFP_DMA, binfo->page_order);
    if (unlikely(!page)) {
	dev_err(dev, "Could not allocate %i page(s) for BIO\n", 1 << binfo->page_order);
	goto put_bio;
    }

    binfo->addr = pci_epc_mem_alloc_addr(dcommon->epf->epc, &binfo->phys_addr, size);
    if (!binfo->addr) {
	dev_err(dev, "Failed to allocate PCI address slot for transfer\n");
	goto release_page;
    }

    dma_addr = dma_map_single(dma_dev, page_address(page), size, dma_dir);
    if (dma_mapping_error(dma_dev, dma_addr)) {
	dev_err(dev, "Failed to map buffer addr\n");
	goto free_epc_mem;
    }

    init_completion(&binfo->dma_transfer_complete);
    binfo->bio = bio;
    binfo->dma_addr = dma_addr;
    binfo->bpt_dev = bpt_dev;
    binfo->page = page;
    binfo->descr = descr;
    binfo->descr_idx = descr_idx;
    binfo->dma_dir = dma_dir;
    return binfo;
free_epc_mem:
    pci_epc_mem_free_addr(dcommon->epf->epc, binfo->phys_addr, binfo->addr, size);
release_page:
    __free_pages(page, binfo->page_order);
put_bio:
    bio_put(bio);
free_binfo:
    devm_kfree(dev, binfo);
    return NULL;
}


static void pci_epf_blockpt_transfer_complete(struct bio *bio)
{
    struct pci_epf_blockpt_info *binfo = bio->bi_private;
    struct device *dev = &binfo->bpt_dev->dcommon->epf->dev;
    struct list_head *cpu_list = per_cpu_ptr(binfo->bpt_dev->proc_list, binfo->cpu);
    struct semaphore *sem = per_cpu_ptr(binfo->bpt_dev->proc_sem, binfo->cpu);
    spinlock_t *lock = per_cpu_ptr(binfo->bpt_dev->proc_lock, binfo->cpu);
    
    if (bio->bi_status != BLK_STS_OK) {
	dev_err(dev, "BIO error %i\n", bio->bi_status);
    }

    spin_lock(lock);
    list_add_tail(&binfo->node, cpu_list);
    spin_unlock(lock);

    up(sem);
}


static void destroy_all_worker_threads(struct pci_epf_blockpt_device *bpt_dev)
{
    int cpu;

    if (bpt_dev->dispatch_thr) {
	kthread_stop(bpt_dev->dispatch_thr);
	bpt_dev->dispatch_thr = NULL;
    }
    
    for_each_present_cpu (cpu) {
	if (bpt_dev->produce_thr[cpu]) {
	    up(per_cpu_ptr(bpt_dev->proc_sem, cpu));
	    bpt_dev->produce_thr[cpu] = NULL;
	}
	if (bpt_dev->digest_thr[cpu]) {
	    up(per_cpu_ptr(bpt_dev->disp_sem, cpu));
	    bpt_dev->digest_thr[cpu] = NULL;
	}
    }

}

static int start_bpt_worker_threads(struct pci_epf_blockpt_device *bpt_dev)
{
    int cpu, ret = 0;
    char tname[64];
    struct device *dev = &bpt_dev->dcommon->epf->dev;
    
    for_each_present_cpu(cpu) {
	if (cpu >= bpt_dev->max_cpu)
	    break;
	snprintf(tname, sizeof(tname), "bpt-dig-%s/%d", bpt_dev->dev_name, cpu);
	dev_dbg(dev, "Creating thread %s\n", tname);
	bpt_dev->digest_thr[cpu] = kthread_create_on_cpu(pci_blockpt_digest_descriptor_kthread, bpt_dev, cpu, tname);
	if (IS_ERR(bpt_dev->digest_thr[cpu])) {
	    ret = PTR_ERR(bpt_dev->digest_thr[cpu]);
	    dev_err(dev, "%s Could not create digest kernel thread: %i\n", bpt_dev->device_path, ret);
	    goto check_start_errors;
	}
	/* we can wake up the kthread here, because it will wait for its percpu samaphore  */
	wake_up_process(bpt_dev->digest_thr[cpu]);
    }

    for_each_present_cpu (cpu) {
	if (cpu >= bpt_dev->max_cpu)
	    break;
	snprintf(tname, sizeof(tname), "bpt-sub-%s/%d", bpt_dev->dev_name, cpu);
	dev_dbg(dev, "Creating thread %s\n", tname);
	bpt_dev->produce_thr[cpu] = kthread_create_on_cpu(pci_blockpt_bio_submit_kthread, bpt_dev, cpu, tname);
	if (IS_ERR(bpt_dev->produce_thr[cpu])) {
	    ret = PTR_ERR(bpt_dev->produce_thr[cpu]);
	    dev_err(dev, "%s Could not create bio submit kernel thread: %i\n", bpt_dev->device_path, ret);
	    goto check_start_errors;
	}
	/* ditto  */
	wake_up_process(bpt_dev->produce_thr[cpu]);
    }

    bpt_dev->dispatch_thr = kthread_create(pci_blockpt_dispatch_desc_on_cpu_kthread, bpt_dev, "bpt-disp/%s" , bpt_dev->dev_name);
    if (IS_ERR(bpt_dev->dispatch_thr)) {
	ret = PTR_ERR(bpt_dev->dispatch_thr);
	dev_err(dev, "%s: Could not create dispatcher kernel thread %i\n", bpt_dev->device_path, ret);
	goto check_start_errors;
    }

#ifdef USE_DMAENGINE	
    dma_cap_zero(mask);
    dma_cap_set(DMA_MEMCPY, mask);

    bpt_dev->dma_chan = dma_request_chan_by_mask(&mask);
    if (IS_ERR(bpt_dev->dma_chan)) {
	ret = PTR_ERR(bpt_dev->dma_chan);
	if (ret != -EPROBE_DEFER)
	    dev_err(dev, "Failed to get DMA channel: %i\n", ret);
	bpt_dev->dma_chan = NULL;
    }
#endif
	
check_start_errors:
    if (ret) {
	destroy_all_worker_threads(bpt_dev);
    } else {
	wake_up_process(bpt_dev->dispatch_thr);
	dev_info(dev, "%s started\n", bpt_dev->device_path);
    }
	
    return ret;
}

static int set_device_descriptor_queue(struct pci_epf_blockpt_device *bpt_dev)
{
    struct device *dev = &bpt_dev->dcommon->epf->dev;
    struct pci_epf_blockpt_reg __iomem *bpt_regs = bpt_dev->dcommon->bpt_regs;
    int ret = 0;

    bpt_dev->num_desc = readl(&bpt_regs->num_desc);
    /* everything below 16 descriptors is by default a bug */
    BUG_ON(bpt_dev->num_desc <= 16);
    
    bpt_dev->descr_addr = readq(&bpt_regs->queue_addr);
    bpt_dev->descr_size = readl(&bpt_regs->queue_size);
    dev_dbg(dev, "%s: mapping Queue to bus address: 0x%llX. Size = 0x%llX\n", bpt_dev->device_path, bpt_dev->descr_addr, bpt_dev->descr_size);

    ret = pci_epf_blockpt_map_queue(bpt_dev, bpt_regs);
    if (ret) {
	dev_err(dev, "Could not set queue: %i\n", ret);
    }

    return ret;
}

static void pci_epf_blockpt_cmd_handler(struct work_struct *work)
{
    struct pci_blockpt_device_common *dcommon = container_of(work, struct pci_blockpt_device_common,
							 cmd_handler.work);
    u32 command;
    int ret, cpu;
    struct pci_epf *epf = dcommon->epf;
    struct pci_epf_blockpt_reg *reg = dcommon->bpt_regs;
    struct pci_epf_blockpt_device *bpt_dev;
    struct device *dev = &epf->dev;
    struct list_head *lh;
    
    command = readl(&reg->command);

    if (!command)
	goto reset_handler;

    writel(0, &reg->command);
    writel(0, &reg->status);

    if (command != 0 && list_empty(&exportable_bds) && list_empty(&dcommon->devices)) {
	dev_err_ratelimited(dev, "Available Devices must be configured first through ConfigFS, before remote partner can send any command\n");
	goto reset_handler;
    }

    bpt_dev = pci_epf_blockpt_get_device_by_id(dcommon, readb(&reg->dev_idx));
    if (!bpt_dev) {
	pci_epf_blockpt_set_invalid_id_error(dcommon, reg);
	goto reset_handler;
    }

    if (command & BPT_COMMAND_GET_DEVICES) {
	int nidx = 0;
	dev_info(dev, "Request for available devices received\n");
	list_for_each(lh, &exportable_bds) {
	    struct pci_epf_blockpt_device *bpt_dev = list_entry(lh, struct pci_epf_blockpt_device, node);
	    nidx += snprintf(&reg->dev_name[nidx], 64, "%s%s", (nidx == 0) ? "" : ";", bpt_dev->device_path);
	}
	
	sprintf(&reg->dev_name[nidx] , "%s", ";");
    }

    if (command & BPT_COMMAND_SET_IRQ) {
	dev_info(dev, "%s setting IRQ %i\n", bpt_dev->device_path, readl(&reg->start_irq));

	for_each_possible_cpu (cpu) {
	    *per_cpu_ptr(bpt_dev->irq, cpu) = readl(&reg->start_irq) + cpu;
	}
    }

    if (command & BPT_COMMAND_GET_NUM_SECTORS) {
	dev_info(dev, "Request for %s number of sectors received\n", bpt_dev->device_path);
	writeq(bdev_nr_sectors(bpt_dev->bd), &reg->num_sectors);
    }

    if (command & BPT_COMMAND_SET_QUEUE) {
	ret = set_device_descriptor_queue(bpt_dev);
	if (ret) {
	    writel(BPT_STATUS_ERROR, &reg->status);
	    goto reset_handler;
	}
	/* if the queue was (re)set, we need to reset the device and driver indices */
	bpt_dev->dev_idx = bpt_dev->drv_idx = 0;
    }

    if (command & BPT_COMMAND_GET_PERMISSION) {
	writel(bpt_dev->read_only ? BPT_PERMISSION_RO : 0, &reg->perm);
    }

    if (command & BPT_COMMAND_START) {
	ret = start_bpt_worker_threads(bpt_dev);
	if (ret) {
	    writel(BPT_STATUS_ERROR, &reg->status);
	    goto reset_handler;
	}
	/* move the device from the exportable_devices to the active ones */
	move_bpt_device_to_active_list(bpt_dev);
	bpt_dev->attached = true;
    }

    if (command & BPT_COMMAND_STOP) {
	if (bpt_dev->attached) {
	    destroy_all_worker_threads(bpt_dev);
	    move_bpt_device_to_exportable_list(bpt_dev);
	    dev_info(dev, "%s stopped\n", bpt_dev->dev_name);
	    bpt_dev->attached = false;
	} else {
	    dev_err(dev, "%s try to stop a device which was not started. \n", bpt_dev->dev_name);
	    writel(BPT_STATUS_ERROR, &reg->status);
	    goto reset_handler;
	}
    }
    writel(BPT_STATUS_SUCCESS, &reg->status);
    
reset_handler:
    queue_delayed_work(kpciblockpt_wq, &dcommon->cmd_handler,
		       msecs_to_jiffies(1));
}

static void pci_epf_blockpt_unbind(struct pci_epf *epf)
{
	struct pci_blockpt_device_common *bpt = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;

	cancel_delayed_work(&bpt->cmd_handler);
	pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no, &epf->bar[bpt->blockpt_reg_bar]);
	pci_epf_free_space(epf, bpt->bpt_regs, bpt->blockpt_reg_bar, PRIMARY_INTERFACE);
}

static int pci_epf_blockpt_set_bar(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_bar *epf_bar;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct pci_blockpt_device_common *dcommon = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;

	epc_features = dcommon->epc_features;

	epf_bar = &epf->bar[dcommon->blockpt_reg_bar];
	ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no, epf_bar);
	if (ret) {
	    pci_epf_free_space(epf, dcommon->bpt_regs, dcommon->blockpt_reg_bar,
			       PRIMARY_INTERFACE);
	    dev_err(dev, "Failed to set BAR%d\n", dcommon->blockpt_reg_bar);
	    return ret;
	}

	return 0;
}

static int pci_epf_blockpt_core_init(struct pci_epf *epf)
{
	struct pci_blockpt_device_common *bpt = epf_get_drvdata(epf);
	struct pci_epf_header *header = epf->header;
	const struct pci_epc_features *epc_features;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	bool msix_capable = false;
	bool msi_capable = true;
	int ret;

	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (epc_features) {
		msix_capable = epc_features->msix_capable;
		msi_capable = epc_features->msi_capable;
	}

	if (epf->vfunc_no <= 1) {
		ret = pci_epc_write_header(epc, epf->func_no, epf->vfunc_no, header);
		if (ret) {
			dev_err(dev, "Configuration header write failed\n");
			return ret;
		}
	}

	ret = pci_epf_blockpt_set_bar(epf);
	if (ret)
		return ret;

	/* MSIs and MSI-Xs are mutually exclusive; MSI-Xs will not work if the
	 * configuration is done for both, simultaneously.
	 */
	if (msi_capable && !msix_capable) {
		dev_info(dev, "Configuring MSIs\n");
		ret = pci_epc_set_msi(epc, epf->func_no, epf->vfunc_no,
				      epf->msi_interrupts);
		if (ret) {
			dev_err(dev, "MSI configuration failed\n");
			return ret;
		}
	}

	if (msix_capable) {
		dev_info(dev, "Configuring MSI-Xs\n");
		ret = pci_epc_set_msix(epc, epf->func_no, epf->vfunc_no,
				       epf->msix_interrupts,
				       bpt->blockpt_reg_bar,
				       bpt->msix_table_offset);
		if (ret) {
			dev_err(dev, "MSI-X configuration failed\n");
			return ret;
		}
	}

	return 0;
}

static int pci_epf_blockpt_notifier(struct notifier_block *nb, unsigned long val,
				 void *data)
{
	struct pci_epf *epf = container_of(nb, struct pci_epf, nb);
	struct pci_blockpt_device_common *dcommon = epf_get_drvdata(epf);
	int ret;

	switch (val) {
	case CORE_INIT:
		ret = pci_epf_blockpt_core_init(epf);
		if (ret)
			return NOTIFY_BAD;
		break;

	case LINK_UP:
		queue_delayed_work(kpciblockpt_wq, &dcommon->cmd_handler,
				   msecs_to_jiffies(1));
		break;

	default:
		dev_err(&epf->dev, "Invalid EPF blockpt notifier event\n");
		return NOTIFY_BAD;
	}

	return NOTIFY_OK;
}

static int pci_epf_blockpt_alloc_space(struct pci_epf *epf)
{
	struct pci_blockpt_device_common *dcommon = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	size_t msix_table_size = 0;
	size_t bio_reg_bar_size;
	size_t pba_size = 0;
	bool msix_capable;
	void *base;
	enum pci_barno reg_bar = dcommon->blockpt_reg_bar;
	const struct pci_epc_features *epc_features;
	size_t bar_reg_size;

	epc_features = dcommon->epc_features;
	bio_reg_bar_size = ALIGN(sizeof(struct pci_epf_blockpt_reg), 128);
	msix_capable = epc_features->msix_capable;
	if (msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		dcommon->msix_table_offset = bio_reg_bar_size;
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);
	}
	
	bar_reg_size = bio_reg_bar_size + msix_table_size + pba_size;
	if (epc_features->bar_fixed_size[reg_bar]) {
		if (bar_reg_size > epc_features->bar_fixed_size[reg_bar])
			return -ENOMEM;
		
		bar_reg_size = epc_features->bar_fixed_size[reg_bar];
	}

	base = pci_epf_alloc_space(epf, bar_reg_size, reg_bar,
				   epc_features->align, PRIMARY_INTERFACE);
	if (!base) {
		dev_err(dev, "Failed to allocated register space\n");
		return -ENOMEM;
	}

	dcommon->bpt_regs = base;
	return 0;
}

static void pci_epf_blockpt_configure_bar(struct pci_epf *epf,
					  const struct pci_epc_features *epc_features, enum pci_barno bar_no)
{
	struct pci_epf_bar *epf_bar = &epf->bar[bar_no];;

	if (!!(epc_features->bar_fixed_64bit & (1 << bar_no)))
	    epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
}


static int pci_epf_blockpt_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_blockpt_device_common *dcommon = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno reg_bar = BAR_0;
	struct pci_epc *epc = epf->epc;
	bool linkup_notifier = false;
	bool core_init_notifier = false;
	struct pci_epf_blockpt_reg *breg;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	linkup_notifier = epc_features->linkup_notifier;
	core_init_notifier = epc_features->core_init_notifier;
	reg_bar = pci_epc_get_first_free_bar(epc_features);
	if (reg_bar < 0)
		return -EINVAL;
	
	pci_epf_blockpt_configure_bar(epf, epc_features, reg_bar);
	dcommon->blockpt_reg_bar = reg_bar;
	dcommon->epc_features = epc_features;
	ret = pci_epf_blockpt_alloc_space(epf);
	if (ret)
		return ret;

	breg = (struct pci_epf_blockpt_reg *) dcommon->bpt_regs;
	breg->magic = BLOCKPT_MAGIC;
	breg->irqs_per_device = num_present_cpus();
	breg->max_devs = MAX_BLOCK_DEVS;
	if (!core_init_notifier) {
		ret = pci_epf_blockpt_core_init(epf);
		if (ret)
			return ret;
	}

	if (linkup_notifier) {
		epf->nb.notifier_call = pci_epf_blockpt_notifier;
		pci_epc_register_notifier(epc, &epf->nb);
	} else {
		queue_work(kpciblockpt_wq, &dcommon->cmd_handler.work);
	}
	return 0;
}

static const struct pci_epf_device_id pci_epf_blockpt_ids[] = {
	{
		.name = "pci_epf_blockpt",
	},
	{},
};

#ifdef USE_DMAENGINE
static void pci_epf_blockpt_dma_callback(void *param)
{
	struct pci_epf_blockpt_info *bio_info = param;
	complete(&bio_info->dma_transfer_complete);
}
#endif


static int pci_blockpt_dispatch_desc_on_cpu_kthread(void *__bpt_dev)
{
    struct pci_blockt_dispatch_descriptor *bpt_dd;
    struct pci_epf_blockpt_device *bpt_dev = __bpt_dev;
    struct device *dev = &bpt_dev->dcommon->epf->dev;
    static int cpu = 0;
    struct list_head *cpu_list;
    struct semaphore *sem;
    spinlock_t *lock;
    
    while (!kthread_should_stop()) {
	while (bpt_dev->drv_idx != readw(&bpt_dev->driver_ring->idx)) {
	    bpt_dd = devm_kzalloc(dev, sizeof(*bpt_dd), GFP_KERNEL);
	    if (bpt_dd == NULL) {
		dev_err(dev, "Could not allocate bpt dispatch descriptor\n");
		break;
	    }

	    cpu_list = per_cpu_ptr(bpt_dev->disp_list, cpu);
	    lock = per_cpu_ptr(bpt_dev->disp_lock, cpu);
	    sem = per_cpu_ptr(bpt_dev->disp_sem, cpu);
	    bpt_dd->desc_idx = readw(&bpt_dev->driver_ring->ring[bpt_dev->drv_idx]);
	    INIT_LIST_HEAD(&bpt_dd->node);

	    spin_lock(lock);
	    list_add_tail(&bpt_dd->node, cpu_list);
	    spin_unlock(lock);

	    cpu = (cpu + 1) % bpt_dev->max_cpu;
	    
	    bpt_dev->drv_idx = (bpt_dev->drv_idx + 1) % bpt_dev->num_desc;
	    up(sem);
	}
	msleep(DRV_RING_POLL_INTERVAL_MS);
    }

    dev_info(dev, "Dispatcher thread stopped\n");
    return 0;
}

static int pci_blockpt_bio_submit_kthread(void *__bpt_dev)
{
    struct pci_blockt_dispatch_descriptor *bpt_dd;
    struct pci_epf_blockpt_device *bpt_dev = __bpt_dev;
    struct device *dev = &bpt_dev->dcommon->epf->dev;
    struct pci_epf *epf = bpt_dev->dcommon->epf;
    struct pci_epc *epc = epf->epc;
    struct pci_epf_blockpt_info *bio_info;
    struct pci_epf_blockpt_descr loc_descr;
    struct pci_epf_blockpt_descr __iomem *descr;
    __maybe_unused struct dma_async_tx_descriptor *dma_txd;
    __maybe_unused dma_cookie_t dma_cookie;
    u16 de;
    int ret = 0;
    struct list_head *cpu_list = this_cpu_ptr(bpt_dev->disp_list);
    spinlock_t *lock = this_cpu_ptr(bpt_dev->disp_lock);
    struct semaphore *sem = this_cpu_ptr(bpt_dev->disp_sem);
    
    while (!kthread_should_stop()) {
	down(sem);
	/* Here no lock is needed as the first element should always be there, guaranteed by the semaphore and cannot be removed underneath us*/
	bpt_dd = list_first_entry_or_null(cpu_list, struct pci_blockt_dispatch_descriptor, node);

	/* This can only happen if the thread supposed to be stopped */
	if (bpt_dd == NULL) {
	    dev_info(dev, "submit thread on cpu %u stopped\n", smp_processor_id());
	    return 0;
	}

	de = bpt_dd->desc_idx;
	descr = &bpt_dev->descr[de];
	memcpy_fromio(&loc_descr, descr, sizeof(loc_descr));

	BUG_ON(!(loc_descr.si.flags & PBI_EPF_BLOCKPT_F_USED));

	bio_info = alloc_pci_epf_blockpt_info(bpt_dev, loc_descr.len, descr, de, (loc_descr.si.opf == WRITE) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
	if (unlikely(!bio_info)) {
	    dev_err(dev, "Unable to allocate bio_info\n");
	    /* the descriptor was _not_ processed, we need to increment the semaphore again */
	    up(sem);
	    blockpt_retry_delay();
	    continue;
	}

	bio_set_dev(bio_info->bio, bpt_dev->bd);
	bio_info->bio->bi_iter.bi_sector = loc_descr.s_sector;
	bio_info->bio->bi_opf = loc_descr.si.opf == WRITE ? REQ_OP_WRITE : REQ_OP_READ;
	if (loc_descr.si.opf == WRITE) {
	    ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, bio_info->phys_addr, loc_descr.addr, loc_descr.len);
	    if (ret) {
		/* This is not an error. Some PCI Controllers have very few translation windows, and as we run this on all available cores
		   it is not unusual that the translation windows are all used for a short period of time. Instead of giving up and panic here, just wait an retry. It will usually
		   be available on the next few retries */
		dev_info(dev, "Mapping descriptor failed with %i. Retry\n", ret);
		goto err_retry;
	    }
#ifdef USE_DMAENGINE
	    if (bpt_dev->dma_chan) {
		dma_txd = dmaengine_prep_dma_memcpy(bpt_dev->dma_chan, bio_info->dma_addr, bio_info->phys_addr, loc_descr.len, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
		if (!dma_txd) {
		    ret = -ENODEV;
		    dev_err(dev, "Failed to prepare DMA memcpy\n");
		    break;
		}
		    
		dma_txd->callback = pci_epf_blockpt_dma_callback;
		dma_txd->callback_param = &bio_info;
		dma_cookie = dma_txd->tx_submit(dma_txd);
		ret = dma_submit_error(dma_cookie);
		if (ret) {
		    dev_err(dev, "Failed to do DMA tx_submit %d\n", dma_cookie);
		    break;
		}
		    
		dma_async_issue_pending(bpt_dev->dma_chan);
		ret = wait_for_completion_interruptible_timeout(&bio_info->dma_transfer_complete);
		if (ret <= 0) {
		    ret = -ETIMEDOUT;
		    dev_err(dev, "DMA wait_for_completion timeout\n");
		    dmaengine_terminate_sync(bpt_dev->dma_chan);
		    break;
		}
	    } else {
		memcpy_fromio(page_address(bio_info->page), bio_info->addr, loc_descr.len);
	    }

#else
	    ret = pci_epc_start_single_dma(epf->epc, epf->func_no, epf->vfunc_no, 1, bio_info->phys_addr, bio_info->dma_addr, loc_descr.len, &bio_info->dma_transfer_complete);
	    if (ret) {
		dev_err(dev, "Failed to start single DMA read\n");
		goto err_retry;
	    } else {
		ret = wait_for_completion_interruptible_timeout(&bio_info->dma_transfer_complete, msecs_to_jiffies(10));
		if (ret <= 0) {
		    ret = -ETIMEDOUT;
		    dev_err(dev, "DMA wait_for_completion timeout\n");
		    goto err_retry;
		}
	    }
	    pci_epc_unmap_addr(epf->epc, epf->func_no, epf->vfunc_no,
			       bio_info->phys_addr);
	    pci_epc_mem_free_addr(epf->epc, bio_info->phys_addr, bio_info->addr,
				  bio_info->size);
#endif
	}

	bio_info->cpu = smp_processor_id();
	bio_info->bio->bi_end_io = pci_epf_blockpt_transfer_complete;
	bio_info->bio->bi_private = bio_info;
	bio_add_page(bio_info->bio, bio_info->page, loc_descr.len, 0);
	submit_bio(bio_info->bio);
	    
	spin_lock(lock);
	list_del(&bpt_dd->node);
	spin_unlock(lock);
	    
	devm_kfree(dev, bpt_dd);
	continue;
err_retry:
	if (loc_descr.si.opf == WRITE) {
	    pci_epc_unmap_addr(epf->epc, epf->func_no, epf->vfunc_no,
			       bio_info->phys_addr);
	    pci_epc_mem_free_addr(epf->epc, bio_info->phys_addr, bio_info->addr,
				  bio_info->size);
	}
	free_pci_blockpt_info(bio_info);
	blockpt_retry_delay();
	up(sem);
    }
    
    return 0;
}


static int pci_blockpt_digest_descriptor_kthread(void *__bpt_dev)
{
	struct pci_epf_blockpt_device *bpt_dev = __bpt_dev;
	struct device *dev = &bpt_dev->dcommon->epf->dev;
	struct pci_epf *epf = bpt_dev->dcommon->epf;
	struct pci_epf_blockpt_info *bi;
	struct pci_epf_blockpt_descr loc_descr;
	int ret;
	int irq = *this_cpu_ptr(bpt_dev->irq);
	struct list_head *cpu_list = this_cpu_ptr(bpt_dev->proc_list);
	struct semaphore *sem = this_cpu_ptr(bpt_dev->proc_sem);
	__maybe_unused struct dma_async_tx_descriptor *dma_rxd;
	__maybe_unused dma_cookie_t dma_cookie;
	__maybe_unused char *buf;

	while (!kthread_should_stop()) {
	    down(sem);
	    /* the semaphore assures that the first element is always there, so no lock needed here */
	    bi = list_first_entry_or_null(cpu_list, struct pci_epf_blockpt_info, node);

	   /* This can only happen if the thread supposed to be stopped */
	    if (bi == NULL) {
		dev_info(dev, "digest thread on cpu %u stopped\n", smp_processor_id());
		return 0;
	    }
	    
	    /* using a local copy of the descriptor avoids unneccessary pci bus accesses */
	    memcpy_fromio(&loc_descr, bi->descr, sizeof(loc_descr));
	    BUG_ON(!(loc_descr.si.flags & PBI_EPF_BLOCKPT_F_USED));
	    if (loc_descr.si.opf == READ) {
		ret = pci_epc_map_addr(epf->epc, epf->func_no, epf->vfunc_no, bi->phys_addr, loc_descr.addr, loc_descr.len);
		if (ret) {
		    /* don't panic. simply retry. A window will be available sooner or later */
		    dev_info(dev, "Could not map read descriptor. Retry\n");
		    up(sem);
		    blockpt_retry_delay();
		    continue;
		}
#ifdef USE_DMAENGINE
		if (bpt_dev->dma_chan) {
		    dma_rxd = dmaengine_prep_dma_memcpy(bpt_dev->dma_chan, bi->phys_addr, bi->dma_addr, loc_descr.len, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
		    if (!dma_rxd) {
			dev_err(dev, "Failed to prepare DMA memcpy\n");
			continue;
		    }
		    
		    dma_rxd->callback = pci_epf_blockpt_dma_callback;
		    dma_rxd->callback_param = &bi;
		    dma_cookie = dma_rxd->tx_submit(dma_rxd);
		    ret = dma_submit_error(dma_cookie);
		    if (ret) {
			dev_err(dev, "Failed to do DMA rx_submit %d\n", dma_cookie);
			continue;
		    }
		    
		    dma_async_issue_pending(bpt_dev->dma_chan);
		    ret = wait_for_completion_interruptible_timeout(&bi->dma_transfer_complete, msecs_to_jiffies(10));
		    if (ret <= 0) {
			dev_err(dev, "DMA wait_for_completion timeout\n");
			dmaengine_terminate_sync(bpt_dev->dma_chan);
			continue;
		    }
		} else {
		    buf = kmap_atomic(bi->page);	
		    memcpy_toio(bi->addr, buf, bi->descr->len);
		    kunmap_atomic(buf);
		}
#else
		ret = pci_epc_start_single_dma(epf->epc, epf->func_no, epf->vfunc_no, 0, bi->dma_addr, bi->phys_addr, loc_descr.len, &bi->dma_transfer_complete);
		if (ret == -EBUSY) {
		    dev_err(dev, "Use PIO for pci transers\n");
		    buf = kmap_atomic(bi->page);	
		    memcpy_toio(bi->addr, buf, loc_descr.len);
		    kunmap_atomic(buf);
		} else if (ret == 0) {
		    ret = wait_for_completion_interruptible_timeout(&bi->dma_transfer_complete, msecs_to_jiffies(10));
		    if (ret <= 0) {
			dev_err(dev, "DMA wait_for_completion timeout\n");
			goto err_retry;
		    } 
		} else {
		    dev_err(dev, "Failed to start single DMA read: %i\n", ret);
		    goto err_retry;
		}
#endif		
	    }
	    
	    spin_lock(&bpt_dev->dev_lock);
	    writew(bi->descr_idx, &bpt_dev->device_ring->ring[bpt_dev->dev_idx]);
	    bpt_dev->dev_idx = (bpt_dev->dev_idx + 1) % bpt_dev->num_desc;
	    writew(bpt_dev->dev_idx, &bpt_dev->device_ring->idx);
	    spin_unlock(&bpt_dev->dev_lock);
	    pci_epc_raise_irq(epf->epc, epf->func_no, epf->vfunc_no, PCI_EPC_IRQ_MSIX, irq);
	    free_pci_blockpt_info(bi);
	    continue;
err_retry:
	    pci_epc_unmap_addr(epf->epc, epf->func_no,
			   epf->vfunc_no, bi->phys_addr);
	    blockpt_retry_delay();
	    up(sem);
	}

	return 0;
}

static int pci_epf_blockpt_probe(struct pci_epf *epf)
{
	struct pci_blockpt_device_common *dcommon;
	struct device *dev = &epf->dev;
	
	dcommon = devm_kzalloc(dev, sizeof(*dcommon), GFP_KERNEL);
	if (!dcommon)
		return -ENOMEM;

	epf->header = &pci_blockpt_header;
	dcommon->epf = epf;
	INIT_LIST_HEAD(&dcommon->devices);
	INIT_LIST_HEAD(&exportable_bds);
	INIT_DELAYED_WORK(&dcommon->cmd_handler, pci_epf_blockpt_cmd_handler);
	epf_set_drvdata(epf, dcommon);
	return 0;
}

static void blockpt_free_per_cpu_data(struct pci_epf_blockpt_device *bpt_dev)
{
    if (bpt_dev->proc_list) {
	free_percpu(bpt_dev->proc_list);
	bpt_dev->proc_list = NULL;
    }
    if (bpt_dev->disp_list) {
	free_percpu(bpt_dev->disp_list);
	bpt_dev->disp_list = NULL;
    }
    if (bpt_dev->proc_lock) {
	free_percpu(bpt_dev->proc_lock);
	bpt_dev->proc_lock = NULL;
    }
    if (bpt_dev->disp_lock) {
	free_percpu(bpt_dev->disp_lock);
	bpt_dev->disp_lock = NULL;
    }
    if (bpt_dev->proc_sem) {
	free_percpu(bpt_dev->proc_sem);
	bpt_dev->proc_sem = NULL;
    }
    if (bpt_dev->disp_sem) {
	free_percpu(bpt_dev->disp_sem);
	bpt_dev->disp_sem = NULL;
    }
    if (bpt_dev->irq) {
	free_percpu(bpt_dev->irq);
	bpt_dev->irq = NULL;
    }
}

static void pci_epf_blockpt_remove(struct pci_epf *epf)
{
    struct pci_blockpt_device_common *dcommon = epf_get_drvdata(epf);
    struct pci_epf_blockpt_device *bpt_dev, *dntmp;
    unsigned long flags;
    struct pci_epf_blockpt_info *bio_info, *bntmp;
    int cpu;
    struct device *dev = &dcommon->epf->dev;
    
    list_for_each_entry_safe(bpt_dev, dntmp, &dcommon->devices, node) {
	destroy_all_worker_threads(bpt_dev);
	blkdev_put(bpt_dev->bd, bpt_dev->read_only ? FMODE_READ : (FMODE_READ | FMODE_WRITE));
	
	spin_lock_irqsave(&bpt_dev->nm_lock, flags);
	list_del(&bpt_dev->node);
	spin_unlock_irqrestore(&bpt_dev->nm_lock, flags);

	for_each_present_cpu (cpu) {
	    list_for_each_entry_safe (bio_info, bntmp, per_cpu_ptr(bpt_dev->proc_list, cpu),
				      node) {
		    free_pci_blockpt_info(bio_info);
	    }
	}

	blockpt_free_per_cpu_data(bpt_dev);
	kfree(bpt_dev->cfs_disk_name);
	kfree(bpt_dev->device_path);
	devm_kfree(dev, bpt_dev);
    }
}

static inline struct pci_epf_blockpt_device *to_blockpt_dev(struct config_item *item)
{
	return container_of(to_config_group(item), struct pci_epf_blockpt_device,
			cfg_grp);
}

static ssize_t pci_blockpt_disc_name_show(struct config_item *item,
					   char *page)
{
    struct pci_epf_blockpt_device *bpt_dev = to_blockpt_dev(item);
    return sprintf(page, "%s\n", (bpt_dev->device_path != NULL) ? bpt_dev->device_path : "");
}

static ssize_t pci_blockpt_disc_name_store(struct config_item *item,
				       const char *page, size_t len)
{
    int ret;
    struct pci_epf_blockpt_device *bpt_dev = to_blockpt_dev(item);
    struct device *dev = &bpt_dev->dcommon->epf->dev;
    struct block_device *blkdev = blkdev_get_by_path(page, bpt_dev->read_only ? FMODE_READ : (FMODE_READ | FMODE_WRITE), NULL);
    unsigned long flags;
    
    if (IS_ERR(blkdev)) {
		ret = PTR_ERR(blkdev);
		if (ret != -ENOTBLK) {
		    dev_err(dev, "Failed to get block device %s: (%ld)\n",
					page, ret);
		}
		return ret;
    }
    
    if (bpt_dev->device_path) 
	kfree(bpt_dev->device_path);

    bpt_dev->bd = blkdev;
    bpt_dev->device_path = kasprintf(GFP_KERNEL, "%s", page);
    if (unlikely(!bpt_dev->device_path)) {
	dev_err(dev, "Unable to allocate memory for device path\n");
	return 0;
    }
    
    bpt_dev->dev_name = strrchr(bpt_dev->device_path, '/');
    if (unlikely(!bpt_dev->dev_name))
	bpt_dev->dev_name = bpt_dev->device_path;
    else
	bpt_dev->dev_name++;
	
    
    spin_lock_irqsave(&bpt_dev->nm_lock, flags);
    list_add_tail(&bpt_dev->node, &exportable_bds);
    spin_unlock_irqrestore(&bpt_dev->nm_lock, flags);
    return len;
}

CONFIGFS_ATTR(pci_blockpt_, disc_name);

static ssize_t pci_blockpt_num_descriptors_show(struct config_item *item,
					  char *page)
{
        struct pci_epf_blockpt_device *bpt_dev = to_blockpt_dev(item);
	return sprintf(page, "%i\n", bpt_dev->num_desc);
}

CONFIGFS_ATTR_RO(pci_blockpt_, num_descriptors);

static ssize_t pci_blockpt_attached_show(struct config_item *item,
					  char *page)
{
        struct pci_epf_blockpt_device *bpt_dev = to_blockpt_dev(item);
	return sprintf(page, "%i\n", bpt_dev->attached);
}

CONFIGFS_ATTR_RO(pci_blockpt_, attached);

static ssize_t pci_blockpt_max_number_of_cpus_show(struct config_item *item,
					   char *page)
{
    struct pci_epf_blockpt_device *bpt_dev = to_blockpt_dev(item);
    return sprintf(page, "%i\n", bpt_dev->max_cpu);
}

static ssize_t pci_blockpt_max_number_of_cpus_store(struct config_item *item,
				       const char *page, size_t len)
{
    struct pci_epf_blockpt_device *bpt_dev = to_blockpt_dev(item);
    u32 mcpu;
    int err;
    
    err = kstrtou32(page, 10, &mcpu);
    if (err || mcpu > num_present_cpus() || mcpu == 0)
	return -EINVAL;

    bpt_dev->max_cpu = mcpu;
    return len;
}

CONFIGFS_ATTR(pci_blockpt_, max_number_of_cpus);

static ssize_t pci_blockpt_read_only_show(struct config_item *item,
					   char *page)
{
    struct pci_epf_blockpt_device *bpt_dev = to_blockpt_dev(item);
    return sprintf(page, "%i\n", bpt_dev->read_only);
}

static ssize_t pci_blockpt_read_only_store(struct config_item *item,
				       const char *page, size_t len)
{
    bool ro;
    struct pci_epf_blockpt_device *bpt_dev = to_blockpt_dev(item);
    int ret = kstrtobool(page, &ro);
    if (ret)
	return ret;
    
    bpt_dev->read_only = ro;
    return len;
}

CONFIGFS_ATTR(pci_blockpt_, read_only);

static struct configfs_attribute *blockpt_attrs[] = {
	&pci_blockpt_attr_disc_name,
	&pci_blockpt_attr_read_only,
	&pci_blockpt_attr_num_descriptors,
	&pci_blockpt_attr_max_number_of_cpus,
	&pci_blockpt_attr_attached,
	NULL,
};

static const struct config_item_type blockpt_disk_type = {
	.ct_attrs	= blockpt_attrs,
	.ct_owner	= THIS_MODULE,
};


static int blockpt_alloc_per_cpu_data(struct pci_epf_blockpt_device *bpt_dev)
{

    struct device *dev = &bpt_dev->dcommon->epf->dev;
    int cpu;
    
    bpt_dev->proc_lock = alloc_percpu(spinlock_t);
    if (bpt_dev->proc_lock == NULL) {
	dev_err(dev, "Could not alloc percpu spinlock\n");
	goto err_dealloc;
    }
    
    for_each_possible_cpu (cpu) {
	spin_lock_init(per_cpu_ptr(bpt_dev->proc_lock, cpu));
    }

    bpt_dev->disp_lock = alloc_percpu(spinlock_t);
    if (bpt_dev->disp_lock == NULL) {
	dev_err(dev, "Could not alloc percpu spinlock\n");
	goto err_dealloc;
    }

    for_each_possible_cpu (cpu) {
	spin_lock_init(per_cpu_ptr(bpt_dev->disp_lock, cpu));
    }

    bpt_dev->proc_sem = alloc_percpu(struct semaphore);
    if (bpt_dev->proc_sem == NULL) {
	dev_err(dev, "Could not alloc percpu semaphore\n");
	goto err_dealloc;
    }

    for_each_possible_cpu (cpu) {
	sema_init(per_cpu_ptr(bpt_dev->proc_sem, cpu), 0);
    }

    bpt_dev->disp_sem = alloc_percpu(struct semaphore);
    if (bpt_dev->disp_sem == NULL) {
	dev_err(dev, "Could not alloc percpu semaphore\n");
	goto err_dealloc;
    }

    for_each_possible_cpu (cpu) {
	sema_init(per_cpu_ptr(bpt_dev->disp_sem, cpu), 0);
    }

    bpt_dev->proc_list = alloc_percpu(struct list_head);
    if (bpt_dev->proc_list == NULL) {
	dev_err(dev, "Could not alloc percpu list\n");
	goto err_dealloc;
    }

    for_each_possible_cpu (cpu) {
	INIT_LIST_HEAD(per_cpu_ptr(bpt_dev->proc_list, cpu));
    }

    bpt_dev->disp_list = alloc_percpu(struct list_head);
    if (bpt_dev->disp_list == NULL) {
	dev_err(dev, "Could not alloc percpu list\n");
	goto err_dealloc;
    }

    for_each_possible_cpu (cpu) {
	INIT_LIST_HEAD(per_cpu_ptr(bpt_dev->disp_list, cpu));
    }

    bpt_dev->irq = alloc_percpu(int);
    for_each_possible_cpu (cpu) {
	*per_cpu_ptr(bpt_dev->irq, cpu) = -EINVAL;
    }

    return 0;
    
err_dealloc:
    blockpt_free_per_cpu_data(bpt_dev);
    return -ENOMEM;
}

static struct config_group *pci_epf_blockpt_add_cfs(struct pci_epf *epf, struct config_group *group)
{
    struct pci_epf_blockpt_device *bpt_dev;
    struct pci_blockpt_device_common *dcommon = epf_get_drvdata(epf);
    struct device *dev = &epf->dev;
    int ret;
    
    bpt_dev = devm_kzalloc(dev, sizeof(*bpt_dev), GFP_KERNEL);
    if (!bpt_dev) {
	dev_err(dev, "Could not alloc bpt device\n");
	return ERR_PTR(-ENOMEM);
    }

    bpt_dev->max_cpu = num_present_cpus();
    bpt_dev->cfs_disk_name = kasprintf(GFP_KERNEL, "disc%i", dcommon->next_disc_idx);
    if (bpt_dev->cfs_disk_name == NULL) {
	dev_err(dev, "Could not alloc cfs disk name\n");
	goto free_bpt_dev;
    }
    
    bpt_dev->dcommon = dcommon;
    ret = blockpt_alloc_per_cpu_data(bpt_dev);
    if (ret)
	goto free_bpt_dev;
    
    spin_lock_init(&bpt_dev->dev_lock);
    spin_lock_init(&bpt_dev->nm_lock);
    INIT_LIST_HEAD(&bpt_dev->node);
    config_group_init_type_name(&bpt_dev->cfg_grp, bpt_dev->cfs_disk_name, &blockpt_disk_type);
    bpt_dev->dev_tag = dcommon->next_disc_idx++;
    return &bpt_dev->cfg_grp;
    
free_bpt_dev:
    devm_kfree(dev, bpt_dev);
    return NULL;
}

static struct pci_epf_ops blockpt_ops = {
	.unbind	= pci_epf_blockpt_unbind,
	.bind	= pci_epf_blockpt_bind,
	.add_cfs = pci_epf_blockpt_add_cfs,
};

static struct pci_epf_driver blockpt_driver = {
	.driver.name	= "pci_epf_blockpt",
	.probe		= pci_epf_blockpt_probe,
	.remove		= pci_epf_blockpt_remove,
	.id_table	= pci_epf_blockpt_ids,
	.ops		= &blockpt_ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_blockpt_init(void)
{
	int ret;

	kpciblockpt_wq = alloc_workqueue("kpciblockpt_wq",
					     WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!kpciblockpt_wq) {
		pr_err("Failed to allocate the kpciblockpt work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&blockpt_driver);
	if (ret) {
		destroy_workqueue(kpciblockpt_wq);
		pr_err("Failed to register pci epf blockpt driver\n");
		return ret;
	}

	return 0;
}
module_init(pci_epf_blockpt_init);

static void __exit pci_epf_blockpt_exit(void)
{
	if (kpciblockpt_wq)
		destroy_workqueue(kpciblockpt_wq);
	pci_epf_unregister_driver(&blockpt_driver);
}
module_exit(pci_epf_blockpt_exit);

MODULE_DESCRIPTION("PCI Endpoint Function Driver for Block Device Passthrough");
MODULE_AUTHOR("Wadim Mueller <wadim.mueller@continental.com>");
MODULE_LICENSE("GPL v2");
