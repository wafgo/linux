// SPDX-License-Identifier: GPL-2.0
/*
 * Block Device Passthrough as an Endpoint Function driver
 *
 * Copyright (C) 2023 Continental Automotive Technologies
 * Author: Wadim Mueller <wadim.mueller@continental.com>
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

#define MAX_BLOCK_DEVS  (16UL)

static struct workqueue_struct *kpciblockpt_wq;

struct pci_blockpt_device_common;

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
        struct pci_blockpt_driver_ring __iomem *driver_ring;
        struct pci_blockpt_device_ring __iomem *device_ring;
        atomic_t dig_cpu;
        u32 drv_idx;
        u32 dev_idx;
        u32 num_desc;
	char *device_path;
        char *dev_name;
        bool read_only;
#ifdef USE_DMAENGINE	
        struct dma_chan *dma_chan;
#endif
        struct list_head __percpu *proc_list;
        struct task_struct *digest_thr[NR_CPUS];
        struct task_struct      *produce_thr;
	spinlock_t __percpu *lock;
        spinlock_t dev_lock;
        spinlock_t nm_lock; /* node move lock */
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
        char    dev_name[64 * MAX_BLOCK_DEVS + 1];
} __packed;

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
};

static int pci_blockpt_digest(void *);
static int pci_blockpt_bio_submit(void *);

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

static struct pci_epf_blockpt_device *pci_epf_blockpt_get_device_by_id(u8 id)
{
    struct list_head *lh;
    struct pci_epf_blockpt_device *bpt_dev;
    list_for_each_rcu(lh, &exportable_bds) {
	bpt_dev = list_entry(lh, struct pci_epf_blockpt_device, node);
	if (bpt_dev->dev_tag == id)
	    return bpt_dev;
    }
    
    return NULL;
}

static void move_bpt_device_to_active_list(struct pci_epf_blockpt_device *bpt_dev)
{
    spin_lock(&bpt_dev->nm_lock);
    list_del_rcu(&bpt_dev->node);
    INIT_LIST_HEAD(&bpt_dev->node);
    list_add_tail_rcu(&bpt_dev->node, &bpt_dev->dcommon->devices);
    spin_unlock(&bpt_dev->nm_lock);
}

static void free_pci_blockpt_info(struct pci_epf_blockpt_info *info)
{
    struct pci_blockpt_device_common *dcommon = info->bpt_dev->dcommon;
    struct device *dev = &dcommon->epf->dev;
    struct device *dma_dev = dcommon->epf->epc->dev.parent;

    dma_unmap_single(dma_dev, info->dma_addr, info->size, info->dma_dir);
    if (info->bio->bi_opf == REQ_OP_READ) {
	pci_epc_unmap_addr(dcommon->epf->epc, dcommon->epf->func_no,
			   dcommon->epf->vfunc_no, info->phys_addr);
	pci_epc_mem_free_addr(dcommon->epf->epc, info->phys_addr,
			      info->addr, info->size);
    }
    
    __free_pages(info->page, info->page_order);
    spin_lock_irq(this_cpu_ptr(info->bpt_dev->lock));
    list_del(&info->node);
    spin_unlock_irq(this_cpu_ptr(info->bpt_dev->lock));
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
    /* FIXME  */
    int cpu = atomic_read(&binfo->bpt_dev->dig_cpu) % 4;
    struct list_head *cpu_list = per_cpu_ptr(binfo->bpt_dev->proc_list, cpu);

    /* atomic_inc(&binfo->bpt_dev->dig_cpu); */
    if (bio->bi_status != BLK_STS_OK) {
	dev_err(dev, "BIO error %i\n", bio->bi_status);
    }

    spin_lock(per_cpu_ptr(binfo->bpt_dev->lock, cpu));
    list_add_tail(&binfo->node, cpu_list);
    spin_unlock(per_cpu_ptr(binfo->bpt_dev->lock, cpu));
    wake_up_process(binfo->bpt_dev->digest_thr[cpu]);
    atomic_inc(&binfo->bpt_dev->dig_cpu);
}


static void pci_epf_blockpt_cmd_handler(struct work_struct *work)
{
    struct pci_blockpt_device_common *dcommon = container_of(work, struct pci_blockpt_device_common,
							 cmd_handler.work);
    u32 command;
    int ret;
    struct pci_epf *epf = dcommon->epf;
    struct pci_epf_blockpt_reg *reg = dcommon->bpt_regs;
    struct pci_epf_blockpt_device *bpt_dev;
    struct device *dev = &epf->dev;
    struct list_head *lh;
    unsigned int cpu;
    char tname[64];
    __maybe_unused dma_cap_mask_t mask;
    
    command = readl(&reg->command);

    if (!command)
	goto reset_handler;

    writel(0, &reg->command);
    writel(0, &reg->status);

    if (command != 0 && list_empty(&exportable_bds)) {
	dev_err_ratelimited(dev, "Available Devices must be configured first through ConfigFS, before remote partner can send any command\n");
	goto reset_handler;
    }

    bpt_dev = pci_epf_blockpt_get_device_by_id(readb(&reg->dev_idx));
    if (!bpt_dev) {
	pci_epf_blockpt_set_invalid_id_error(dcommon, reg);
	goto reset_handler;
    }

    if (command & COMMAND_GET_DEVICES) {
	int nidx = 0;
	dev_info(dev, "Request for available devices received\n");
	list_for_each_rcu(lh, &exportable_bds) {
	    struct pci_epf_blockpt_device *bpt_dev = list_entry(lh, struct pci_epf_blockpt_device, node);
	    nidx += snprintf(&reg->dev_name[nidx], 64, "%s%s", (nidx == 0) ? "" : ";", bpt_dev->device_path);
	}
	
	sprintf(&reg->dev_name[nidx] , "%s", ";");
    }

    if (command & COMMAND_GET_NUM_SECTORS) {
	dev_info(dev, "Request for %s number of sectors received\n", bpt_dev->device_path);
	writeq(bdev_nr_sectors(bpt_dev->bd), &reg->num_sectors);
    }

    if (command & COMMAND_SET_QUEUE) {
	dev_dbg(dev, "%s: mapping Queue to physical address: 0x%llX. Size = 0x%llX\n", bpt_dev->device_path, reg->queue_addr, reg->queue_size);
	if (bpt_dev->descr_addr != 0) {
	    dev_err(dev, "%s: Queue is already mapped: 0x%llX\n", bpt_dev->device_path, bpt_dev->descr_addr);
	    goto reset_handler;
	}

	bpt_dev->num_desc = readl(&reg->num_desc);
	/* everything below 16 descriptors is by default a bug */
	BUG_ON(bpt_dev->num_desc <= 16);
	bpt_dev->descr_addr = readq(&reg->queue_addr);
	bpt_dev->descr_size = readl(&reg->queue_size);
	ret = pci_epf_blockpt_map_queue(bpt_dev, reg);
	if (ret) {
	    dev_err(dev, "Could not set queue: %i\n", ret);
	    writel(BPT_STATUS_QUEUE_ADDR_INVALID, &reg->status);
	    goto reset_handler;
	} 
    }

    if (command & COMMAND_START) {
	for_each_present_cpu(cpu) {
	    snprintf(tname, sizeof(tname), "bptd-%s/%d", bpt_dev->dev_name, cpu);
	    dev_info(dev, "Starting thread %s\n", tname);
	    bpt_dev->digest_thr[cpu] = kthread_create_on_cpu(pci_blockpt_digest, bpt_dev, cpu, tname);
	    if (IS_ERR(bpt_dev->digest_thr[cpu])) {
		ret = PTR_ERR(bpt_dev->digest_thr[cpu]);
		dev_err(dev, "%s Could not create digest kernel thread: %i\n", bpt_dev->device_path, ret);
		writel(BPT_STATUS_ERROR, &reg->status);
		goto reset_handler;
	    }
	}
		
	bpt_dev->produce_thr = kthread_create(pci_blockpt_bio_submit, bpt_dev, "bpt-bio/%s", bpt_dev->dev_name);
	if (IS_ERR(bpt_dev->produce_thr)) {
	    ret = PTR_ERR(bpt_dev->produce_thr);
	    dev_err(dev, "%s: Could not create bio producer kernel thread %i\n", bpt_dev->device_path, ret);
	    writel(BPT_STATUS_ERROR, &reg->status);
	    goto reset_handler;
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
	dev_info(dev, "%s started\n", bpt_dev->device_path);
	/* move the device from the exportable_devices to the active ones */
	move_bpt_device_to_active_list(bpt_dev);
	wake_up_process(bpt_dev->produce_thr);
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
		dev_err(&epf->dev, "Invalid EPF test notifier event\n");
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

static int pci_blockpt_bio_submit(void *__bpt_dev)
{
    struct pci_epf_blockpt_device *bpt_dev = __bpt_dev;
    struct device *dev = &bpt_dev->dcommon->epf->dev;
    struct pci_epf *epf = bpt_dev->dcommon->epf;
    struct pci_epc *epc = epf->epc;
    struct pci_epf_blockpt_info *bio_info;
    struct pci_epf_blockpt_descr_remote loc_descr;
    struct pci_epf_blockpt_descr __iomem *descr;
    __maybe_unused struct dma_async_tx_descriptor *dma_txd;
    __maybe_unused dma_cookie_t dma_cookie;
    u16 de;
    int ret = 0;

    while (!kthread_should_stop()) {
	while (bpt_dev->drv_idx != readw(&bpt_dev->driver_ring->idx)) {
	    de = readw(&bpt_dev->driver_ring->ring[bpt_dev->drv_idx]);
	    descr = &bpt_dev->descr[de];
	    memcpy_fromio(&loc_descr, &descr->rdata, sizeof(loc_descr));

	    BUG_ON(!(loc_descr.si.flags & PBI_EPF_BLOCKPT_F_USED));

	    bio_info = alloc_pci_epf_blockpt_info(bpt_dev, loc_descr.len, descr, de, (loc_descr.si.opf == WRITE) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
	    if (unlikely(!bio_info)) {
		dev_err(dev, "Unable to allocate bio_info\n");
		break;
	    }

	    bio_set_dev(bio_info->bio, bpt_dev->bd);
	    bio_info->bio->bi_iter.bi_sector = loc_descr.s_sector;
	    bio_info->bio->bi_opf = loc_descr.si.opf == WRITE ? REQ_OP_WRITE : REQ_OP_READ;
	    if (loc_descr.si.opf == WRITE) {
		ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, bio_info->phys_addr, loc_descr.addr, loc_descr.len);
		if (ret) {
		    dev_err(dev, "Failed to map descriptor: %i\n", ret);
		    break;
		}
#ifdef USE_DMAENGINE
		if (bpt_dev->dma_chan) {
		    dma_txd = dmaengine_prep_dma_memcpy(bpt_dev->dma_chan, page_to_phys(bio_info->page), bio_info->phys_addr, loc_descr.len, DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
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
		ret = pci_epc_start_single_dma(epf->epc, epf->func_no, epf->vfunc_no, 1, bio_info->phys_addr, page_to_phys(bio_info->page), loc_descr.len, &bio_info->dma_transfer_complete);
		if (ret) {
		    dev_err(dev, "Failed to start single DMA read\n");
		    break;
		} else {
		    ret = wait_for_completion_interruptible_timeout(&bio_info->dma_transfer_complete, msecs_to_jiffies(10));
		    if (ret <= 0) {
			ret = -ETIMEDOUT;
			dev_err(dev, "DMA wait_for_completion timeout\n");
			break;
		    }
		}
		pci_epc_unmap_addr(epf->epc, epf->func_no, epf->vfunc_no, bio_info->phys_addr);
		pci_epc_mem_free_addr(epf->epc, bio_info->phys_addr, bio_info->addr, bio_info->size);
#endif		
	    }

	    bpt_dev->drv_idx = (bpt_dev->drv_idx + 1) % bpt_dev->num_desc;
	    bio_info->bio->bi_end_io = pci_epf_blockpt_transfer_complete;
	    bio_info->bio->bi_private = bio_info;
	    bio_add_page(bio_info->bio, bio_info->page, loc_descr.len, 0);
	    submit_bio(bio_info->bio);
	}

	if (ret) {
	    free_pci_blockpt_info(bio_info);
	    ret = 0;
	}
	
	usleep_range(500, 1000);
    }
    
    return 0;
}

static int pci_blockpt_digest(void *__bpt_dev)
{
	struct pci_epf_blockpt_device *bpt_dev = __bpt_dev;
	struct device *dev = &bpt_dev->dcommon->epf->dev;
	struct pci_epf *epf = bpt_dev->dcommon->epf;
	struct pci_epf_blockpt_info *bi;
	struct pci_epf_blockpt_descr_remote loc_descr;
	int ret;
	__maybe_unused struct dma_async_tx_descriptor *dma_rxd;
	__maybe_unused dma_cookie_t dma_cookie;
	__maybe_unused char *buf;

	while (!kthread_should_stop()) {
	    spin_lock_irq(this_cpu_ptr(bpt_dev->lock));
	    bi = list_first_entry_or_null(this_cpu_ptr(bpt_dev->proc_list), struct pci_epf_blockpt_info, node);
	    spin_unlock_irq(this_cpu_ptr(bpt_dev->lock));
	    if (bi == NULL) {
		usleep_range(1000, 5000);
		continue;
	    }

	    memcpy_fromio(&loc_descr, &bi->descr->rdata, sizeof(loc_descr));
	    BUG_ON(!(loc_descr.si.flags & PBI_EPF_BLOCKPT_F_USED));
	    if (loc_descr.si.opf == READ) {
		ret = pci_epc_map_addr(epf->epc, epf->func_no, epf->vfunc_no, bi->phys_addr, loc_descr.addr, loc_descr.len);

		if (ret) {
		    dev_err(dev, "Failed to map buffer address\n");
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
			continue;
		    } 
		} else {
		    dev_err(dev, "Failed to start single DMA read: %i\n", ret);
		    pci_epc_unmap_addr(epf->epc, epf->func_no, epf->vfunc_no, bi->phys_addr);
		    continue;
		}
		
#endif		

	    }
	    spin_lock(&bi->bpt_dev->dev_lock);
	    writew(bi->descr_idx, &bi->bpt_dev->device_ring->ring[bi->bpt_dev->dev_idx]);
	    bi->bpt_dev->dev_idx = (bi->bpt_dev->dev_idx + 1) % bi->bpt_dev->num_desc;
	    writew(bi->bpt_dev->dev_idx, &bi->bpt_dev->device_ring->idx);
	    spin_unlock(&bi->bpt_dev->dev_lock);
	    free_pci_blockpt_info(bi);
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

static void pci_epf_blockpt_remove(struct pci_epf *epf)
{
    struct pci_blockpt_device_common *dcommon = epf_get_drvdata(epf);
    struct pci_epf_blockpt_device *bpt_dev, *dntmp;
    unsigned long flags;
    struct pci_epf_blockpt_info *bio_info, *bntmp;
    int cpu;
    struct device *dev = &dcommon->epf->dev;
    
    list_for_each_entry_safe(bpt_dev, dntmp, &dcommon->devices, node) {
	kthread_stop(bpt_dev->produce_thr);

	for_each_present_cpu (cpu) {
	    kthread_stop(bpt_dev->digest_thr[cpu]);
	}
	
	blkdev_put(bpt_dev->bd, bpt_dev->read_only ? FMODE_READ : (FMODE_READ | FMODE_WRITE));
	
	spin_lock_irqsave(&bpt_dev->nm_lock, flags);
	list_del_rcu(&bpt_dev->node);
	spin_unlock_irqrestore(&bpt_dev->nm_lock, flags);
	
	synchronize_rcu();

	for_each_present_cpu (cpu) {
	    list_for_each_entry_safe (bio_info, bntmp, per_cpu_ptr(bpt_dev->proc_list, cpu),
				      node) {
		    free_pci_blockpt_info(bio_info);
	    }
	}

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
    return sprintf(page, "%s", (bpt_dev->device_path != NULL) ? bpt_dev->device_path : "");
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
    list_add_tail_rcu(&bpt_dev->node, &exportable_bds);
    spin_unlock_irqrestore(&bpt_dev->nm_lock, flags);
    return len;
}

CONFIGFS_ATTR(pci_blockpt_, disc_name);

static ssize_t pci_blockpt_read_only_show(struct config_item *item,
					   char *page)
{
    struct pci_epf_blockpt_device *bpt_dev = to_blockpt_dev(item);
    return sprintf(page, "%s", bpt_dev->read_only ? "read_only" : "rw");
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
	NULL,
};

static const struct config_item_type blockpt_disk_type = {
	.ct_attrs	= blockpt_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *pci_epf_blockpt_add_cfs(struct pci_epf *epf, struct config_group *group)
{
    struct pci_epf_blockpt_device *bpt_dev;
    struct pci_blockpt_device_common *dcommon = epf_get_drvdata(epf);
    struct device *dev = &epf->dev;
    int cpu;
    
    bpt_dev = devm_kzalloc(dev, sizeof(*bpt_dev), GFP_KERNEL);
    if (!bpt_dev) {
	dev_err(dev, "Could not alloc bpt device\n");
	return ERR_PTR(-ENOMEM);
    }
    
    bpt_dev->cfs_disk_name = kasprintf(GFP_KERNEL, "disc%i", dcommon->next_disc_idx);
    bpt_dev->dcommon = dcommon;
    bpt_dev->lock = alloc_percpu(spinlock_t);
    for_each_possible_cpu (cpu) {
	spin_lock_init(per_cpu_ptr(bpt_dev->lock, cpu));
    }

    spin_lock_init(&bpt_dev->dev_lock);
    spin_lock_init(&bpt_dev->nm_lock);
    INIT_LIST_HEAD(&bpt_dev->node);
    bpt_dev->proc_list = alloc_percpu(struct list_head);
    for_each_possible_cpu (cpu) {
	INIT_LIST_HEAD(per_cpu_ptr(bpt_dev->proc_list, cpu));
    }
    
    /* INIT_LIST_HEAD(&bpt_dev->proc_list); */
    config_group_init_type_name(&bpt_dev->cfg_grp, bpt_dev->cfs_disk_name, &blockpt_disk_type);
    bpt_dev->dev_tag = dcommon->next_disc_idx++;
    
    return &bpt_dev->cfg_grp;
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
		pr_err("Failed to allocate the kpcitest work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&blockpt_driver);
	if (ret) {
		destroy_workqueue(kpciblockpt_wq);
		pr_err("Failed to register pci epf test driver\n");
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
