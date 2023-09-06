// SPDX-License-Identifier: GPL-2.0
/*
 * Block Device Passthrough as an Endpoint Function driver
 *
 * Copyright (C) 2023 Continental Automotive Technologies
 * Author: Wadim Mueller <wadim.mueller@continental.com>
 */

#include <linux/crc32.h>
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
#include <linux/major.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/blk-mq.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <linux/idr.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/hdreg.h>
#include <linux/kthread.h>

#define USE_DMA

//static char *pt_block_dev_name = "/dev/mmcblk0";
static char *pt_block_dev_name = "/dev/nvme0n1";

#define BLOCKPT_MAGIC 0x636f6e74

#define PBI_EPF_BLOCKPT_F_LAST BIT(0)
#define PBI_EPF_BLOCKPT_F_USED BIT(1)
#define PBI_EPF_BLOCKPT_F_DIR_READ BIT(2)

#define IRQ_TYPE_LEGACY			0
#define IRQ_TYPE_MSI			1
#define IRQ_TYPE_MSIX			2

#define COMMAND_RAISE_LEGACY_IRQ	BIT(0)
#define COMMAND_RAISE_MSI_IRQ		BIT(1)
#define COMMAND_RAISE_MSIX_IRQ		BIT(2)
#define COMMAND_WRITE_TO_QUEUE          BIT(4)
#define COMMAND_READ_FROM_QUEUE         BIT(5)
#define COMMAND_SET_QUEUE               BIT(6)
#define COMMAND_ADD_DRV_ENTRY           BIT(7)
#define COMMAND_START                   BIT(8)

#define BPT_STATUS_SUCCESS	        BIT(0)
#define BPT_STATUS_QUEUE_ADDR_INVALID	BIT(9)

#define DEFAULT_WORK_DELAY_MS           (1)
#define LONG_WORK_DELAY_MS              (100)

static struct workqueue_struct *kpciblockpt_wq;

struct pci_epf_blockpt_descr {
    sector_t s_sector; /* start sector of the request */
    u64 addr; /* where the data is  */
    u64 opf;
    u64 tag;
    u32 len; /* bytes to pu at addr + s_offset*/
    u32 offset;  /* offset from addr */
    u32 status;
    u32 flags;
    u32 res0;
    u32 res1;
};

struct pci_blockpt_driver_ring {
    u16 idx;
    u16 ring[]; /* queue size*/
};

struct pci_blockpt_device_ring {
    u16 idx;
    u16 ring[]; /* queue size*/
};

enum cmd_state {
    PCI_BLOCKPT_SETUP = 0,
    PCI_BLOCKPT_RUNNING,
};

struct pci_epf_blockpt {
	void			*reg[PCI_STD_NUM_BARS];
	struct pci_epf		*epf;
	enum pci_barno		bio_reg_bar;
	size_t			msix_table_offset;
	struct delayed_work	cmd_handler;
        struct list_head        proc_list;
        struct task_struct      *digest_thr;
        struct task_struct      *produce_thr;
	const struct pci_epc_features *epc_features;
        struct pci_epf_blockpt_descr __iomem *descr;
	struct block_device *real_bd;
        dma_addr_t descr_addr;
        u32 descr_size;
        struct pci_blockpt_driver_ring __iomem *driver_ring;
        struct pci_blockpt_device_ring __iomem *device_ring;
        u32 drv_idx;
        u32 dev_idx;
        u32 num_desc;
	char *device_path;
	spinlock_t lock;
        enum cmd_state state;
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
        u64	queue_addr;  /* start of struct pci_epf_blockpt_descr*/
        u64     num_sectors;
        char    dev_name[64];
} __packed;

static struct pci_epf_header pci_blockpt_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

module_param(pt_block_dev_name, charp, 0444);

#define PCI_REMOTE_BLOCK_MINORS 16
#define DRV_MODULE_NAME "pci-remote-bdev"

#define PCI_RBD_INLINE_SG_CNT 2

struct pci_epf_blockpt_info {
    struct list_head node;
    struct pci_epf_blockpt *bpt;
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

static size_t bar_size[] = { 512, 512, 1024, 16384, 131072, 1048576};

static int pci_epf_blockpt_map_queue(struct pci_epf_blockpt *bpt, struct pci_epf_blockpt_reg *reg) {
        int ret;
	void __iomem *addr;
	phys_addr_t phys_addr;
	struct pci_epf *epf = bpt->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;

	addr = pci_epc_mem_alloc_addr(epc, &phys_addr, bpt->descr_size);
	if (!addr) {
		dev_err(dev, "Failed to allocate queue address\n");
		reg->status = BPT_STATUS_QUEUE_ADDR_INVALID;
		return -ENOMEM;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, phys_addr,
			       bpt->descr_addr, bpt->descr_size);
	if (ret) {
		dev_err(dev, "Failed to map queue address\n");
		reg->status = BPT_STATUS_QUEUE_ADDR_INVALID;
		ret = -EINVAL;
		goto err_queue_addr;
	}

	bpt->descr = addr;
	bpt->driver_ring = (struct pci_blockpt_driver_ring *)((u64)addr + readl(&reg->drv_offset));
	bpt->device_ring = (struct pci_blockpt_device_ring *)((u64)addr + readl(&reg->dev_offset));

	dev_dbg(dev, "\tQueue => Queue Address physical: 0x%llX\t Queue Address virt: 0x%llX\t Queue PCI Addr: 0x%llX\n",
		phys_addr, addr, bpt->descr_addr);
	return 0;
err_queue_addr:
	pci_epc_mem_free_addr(epc, phys_addr, addr, bpt->descr_size);
	return ret;
}


static void pci_epf_setup(struct pci_epf_blockpt *bpt)
{
    u32 command;
    enum pci_barno bio_reg_bar = bpt->bio_reg_bar;
    struct pci_epf *epf = bpt->epf;
    struct pci_epf_blockpt_reg *reg = bpt->reg[bio_reg_bar];
    struct device *dev = &epf->dev;
    
    command = readl(&reg->command);

    if (!command)
	goto reset_handler;

    writel(0, &reg->command);
    writel(0, &reg->status);

    if (command & COMMAND_SET_QUEUE) {
	dev_dbg(dev, "mapping Queue to physical address: 0x%llX. Size = 0x%llX\n", reg->queue_addr, reg->queue_size);
	if (bpt->descr_addr != 0) {
	    dev_err(dev, "Queue is already mapped: 0x%llX\n", bpt->descr_addr);
	    goto reset_handler;
	}
	
	bpt->num_desc = readl(&reg->num_desc);
	
	BUG_ON(bpt->num_desc <= 0);
	
	bpt->descr_addr = readq(&reg->queue_addr);
	bpt->descr_size = readl(&reg->queue_size);
	pci_epf_blockpt_map_queue(bpt, reg);
	writel(BPT_STATUS_SUCCESS, &reg->status);
    }

    if (command & COMMAND_START) {
	if (bpt->descr_addr == 0) {
	    dev_err(dev, "Queue is not mapped, cannot start anything\n");
	    goto reset_handler;
	}
	
	dev_info(dev, "Started\n");
	writel(BPT_STATUS_SUCCESS, &reg->status);
	bpt->state = PCI_BLOCKPT_RUNNING;
	wake_up_process(bpt->produce_thr);
	return;
    }

reset_handler:
    queue_delayed_work(kpciblockpt_wq, &bpt->cmd_handler,
		       msecs_to_jiffies(1));
}

static void free_bpt_info(struct pci_epf_blockpt_info *info)
{
    struct device *dev = &info->bpt->epf->dev;
    struct device *dma_dev = info->bpt->epf->epc->dev.parent;

    dma_unmap_single(dma_dev, info->dma_addr, info->size, info->dma_dir);

    if (info->bio->bi_opf == REQ_OP_READ) {
	pci_epc_unmap_addr(info->bpt->epf->epc, info->bpt->epf->func_no,
			   info->bpt->epf->vfunc_no, info->phys_addr);
	pci_epc_mem_free_addr(info->bpt->epf->epc, info->phys_addr,
			      info->addr, info->size);
    }
    
    __free_pages(info->page, info->page_order);
    
    spin_lock_irq(&info->bpt->lock);
    list_del(&info->node);
    spin_unlock_irq(&info->bpt->lock);
    bio_put(info->bio);    
    devm_kfree(dev, info);
}

struct pci_epf_blockpt_info *alloc_pci_epf_blockpt_info(struct pci_epf_blockpt *bpt, size_t size, struct pci_epf_blockpt_descr __iomem *descr, int descr_idx, enum dma_data_direction dma_dir)
{
    struct pci_epf_blockpt_info *binfo;
    struct bio *bio;
    struct device *dev = &bpt->epf->dev;
    struct page *page;
    struct device *dma_dev = bpt->epf->epc->dev.parent;
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

    binfo->addr = pci_epc_mem_alloc_addr(bpt->epf->epc, &binfo->phys_addr, size);
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
    binfo->bpt = bpt;
    binfo->page = page;
    binfo->descr = descr;
    binfo->descr_idx = descr_idx;
    binfo->dma_dir = dma_dir;
    return binfo;
free_epc_mem:
    pci_epc_mem_free_addr(binfo->bpt->epf->epc, binfo->phys_addr, binfo->addr, size);
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
    struct device *dev = &binfo->bpt->epf->dev;
    if (bio->bi_status != BLK_STS_OK) {
	dev_err(dev, "BIO error %i\n", bio->bi_status);
    }

    spin_lock(&binfo->bpt->lock);
    list_add_tail(&binfo->node, &binfo->bpt->proc_list);
    spin_unlock(&binfo->bpt->lock);
    
    wake_up_process(binfo->bpt->digest_thr);
}


static void pci_epf_blockpt_cmd_handler(struct work_struct *work)
{
	struct pci_epf_blockpt *bpt = container_of(work, struct pci_epf_blockpt,
						     cmd_handler.work);
	if (bpt->state == PCI_BLOCKPT_SETUP)
	    return pci_epf_setup(bpt);
}

static void pci_epf_blockpt_unbind(struct pci_epf *epf)
{
	struct pci_epf_blockpt *bpt = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct pci_epf_bar *epf_bar;
	int bar;

	cancel_delayed_work(&bpt->cmd_handler);
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar = &epf->bar[bar];

		if (bpt->reg[bar]) {
			pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,
					  epf_bar);
			pci_epf_free_space(epf, bpt->reg[bar], bar,
					   PRIMARY_INTERFACE);
		}
	}
}

static int pci_epf_blockpt_set_bar(struct pci_epf *epf)
{
	int bar, add;
	int ret;
	struct pci_epf_bar *epf_bar;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct pci_epf_blockpt *bpt = epf_get_drvdata(epf);
	enum pci_barno bio_reg_bar = bpt->bio_reg_bar;
	const struct pci_epc_features *epc_features;

	epc_features = bpt->epc_features;
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		/*
		 * pci_epc_set_bar() sets PCI_BASE_ADDRESS_MEM_TYPE_64
		 * if the specific implementation required a 64-bit BAR,
		 * even if we only requested a 32-bit BAR.
		 */
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;
		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no, epf_bar);
		if (ret) {
			pci_epf_free_space(epf, bpt->reg[bar], bar,
					   PRIMARY_INTERFACE);
			dev_err(dev, "Failed to set BAR%d\n", bar);
			if (bar == bio_reg_bar)
				return ret;
		}
	}

	return 0;
}

static int pci_epf_blockpt_core_init(struct pci_epf *epf)
{
	struct pci_epf_blockpt *bpt = epf_get_drvdata(epf);
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
				       bpt->bio_reg_bar,
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
	struct pci_epf_blockpt *bpt = epf_get_drvdata(epf);
	int ret;

	switch (val) {
	case CORE_INIT:
		ret = pci_epf_blockpt_core_init(epf);
		if (ret)
			return NOTIFY_BAD;
		break;

	case LINK_UP:
		queue_delayed_work(kpciblockpt_wq, &bpt->cmd_handler,
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
	struct pci_epf_blockpt *bpt = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	struct pci_epf_bar *epf_bar;
	size_t msix_table_size = 0;
	size_t bio_reg_bar_size;
	size_t pba_size = 0;
	bool msix_capable;
	void *base;
	int bar, add;
	enum pci_barno bio_reg_bar = bpt->bio_reg_bar;
	const struct pci_epc_features *epc_features;
	size_t bar_reg_size;

	epc_features = bpt->epc_features;
	bio_reg_bar_size = ALIGN(sizeof(struct pci_epf_blockpt_reg), 128);
	msix_capable = epc_features->msix_capable;
	if (msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		bpt->msix_table_offset = bio_reg_bar_size;
		/* Align to QWORD or 8 Bytes */
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);
	}
	
	bar_reg_size = bio_reg_bar_size + msix_table_size + pba_size;
	if (epc_features->bar_fixed_size[bio_reg_bar]) {
		if (bar_reg_size > bar_size[bio_reg_bar])
			return -ENOMEM;
		
		bar_reg_size = bar_size[bio_reg_bar];
	}

	base = pci_epf_alloc_space(epf, bar_reg_size, bio_reg_bar,
				   epc_features->align, PRIMARY_INTERFACE);
	if (!base) {
		dev_err(dev, "Failed to allocated register space\n");
		return -ENOMEM;
	}
	
	bpt->reg[bio_reg_bar] = base;
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar += add) {
		epf_bar = &epf->bar[bar];
		add = (epf_bar->flags & PCI_BASE_ADDRESS_MEM_TYPE_64) ? 2 : 1;
		if (bar == bio_reg_bar)
			continue;

		if (!!(epc_features->reserved_bar & (1 << bar)))
			continue;

		base = pci_epf_alloc_space(epf, bar_size[bar], bar,
					   epc_features->align,
					   PRIMARY_INTERFACE);
		if (!base)
			dev_err(dev, "Failed to allocate space for BAR%d\n",
				bar);
		bpt->reg[bar] = base;
	}

	return 0;
}

static void pci_epf_configure_bar(struct pci_epf *epf,
				  const struct pci_epc_features *epc_features)
{
	struct pci_epf_bar *epf_bar;
	bool bar_fixed_64bit;
	int i;

	for (i = 0; i < PCI_STD_NUM_BARS; i++) {
		epf_bar = &epf->bar[i];
		bar_fixed_64bit = !!(epc_features->bar_fixed_64bit & (1 << i));
		if (bar_fixed_64bit)
			epf_bar->flags |= PCI_BASE_ADDRESS_MEM_TYPE_64;
		if (epc_features->bar_fixed_size[i])
			bar_size[i] = epc_features->bar_fixed_size[i];
	}
}

static int pci_epf_blockpt_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_blockpt *bpt = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno bio_reg_bar = BAR_0;
	struct pci_epc *epc = epf->epc;
	bool linkup_notifier = false;
	bool core_init_notifier = false;
	struct pci_epf_blockpt_reg *breg;

	if (WARN_ON_ONCE(!epc))
		return -EINVAL;

	if (WARN_ON_ONCE(!bpt))
	    return -EINVAL;

	epc_features = pci_epc_get_features(epc, epf->func_no, epf->vfunc_no);
	if (!epc_features) {
		dev_err(&epf->dev, "epc_features not implemented\n");
		return -EOPNOTSUPP;
	}

	linkup_notifier = epc_features->linkup_notifier;
	core_init_notifier = epc_features->core_init_notifier;
	bio_reg_bar = pci_epc_get_first_free_bar(epc_features);
	if (bio_reg_bar < 0)
		return -EINVAL;
	
	pci_epf_configure_bar(epf, epc_features);

	bpt->bio_reg_bar = bio_reg_bar;
	bpt->epc_features = epc_features;

	ret = pci_epf_blockpt_alloc_space(epf);
	if (ret)
		return ret;

	breg = (struct pci_epf_blockpt_reg *) bpt->reg[bio_reg_bar];
	breg->magic = BLOCKPT_MAGIC;
	breg->num_sectors = get_capacity(bdev_whole(bpt->real_bd)->bd_disk);
	strncpy(breg->dev_name, pt_block_dev_name, sizeof(breg->dev_name));
	if (!core_init_notifier) {
		ret = pci_epf_blockpt_core_init(epf);
		if (ret)
			return ret;
	}

	if (linkup_notifier) {
		epf->nb.notifier_call = pci_epf_blockpt_notifier;
		pci_epc_register_notifier(epc, &epf->nb);
	} else {
		queue_work(kpciblockpt_wq, &bpt->cmd_handler.work);
	}
	return 0;
}

static const struct pci_epf_device_id pci_epf_blockpt_ids[] = {
	{
		.name = "pci_epf_blockpt",
	},
	{},
};


static int setup_block_device(struct pci_epf_blockpt *bpt)
{
        int err;
	struct device *dev = &bpt->epf->dev;
    	bpt->device_path = pt_block_dev_name;
	bpt->real_bd = blkdev_get_by_path(bpt->device_path,
					   FMODE_READ | FMODE_WRITE, NULL);
	if (IS_ERR(bpt->real_bd)) {
		err = PTR_ERR(bpt->real_bd);
		if (err != -ENOTBLK) {
			dev_err(dev,
				"failed to open block device %s: (%ld)\n",
				bpt->device_path, err);
		}
		goto out_err;
	}
	return 0;
out_err:
	return err;
}

static int pci_blockpt_produce(void *cookie)
{
    struct pci_epf_blockpt *bpt = (struct pci_epf_blockpt *)cookie;
    struct device *dev = &bpt->epf->dev;
    struct pci_blockpt_driver_ring __iomem *drv_ring = bpt->driver_ring;
    struct pci_epf *epf = bpt->epf;
    struct pci_epc *epc = epf->epc;
    u16 de;
    struct pci_epf_blockpt_info *bio_info;
    struct pci_epf_blockpt_descr loc_descr;
    struct pci_epf_blockpt_descr __iomem *descr;
    int ret;

    while (!kthread_should_stop()) {
	while (bpt->drv_idx != readw(&drv_ring->idx)) {
	    de = readw(&drv_ring->ring[bpt->drv_idx]);
	    descr = &bpt->descr[de];
	    memcpy_fromio(&loc_descr, descr, sizeof(loc_descr));

	    BUG_ON(!(loc_descr.flags & PBI_EPF_BLOCKPT_F_USED));

	    bio_info = alloc_pci_epf_blockpt_info(bpt, loc_descr.len, descr, de, (loc_descr.opf == WRITE) ? DMA_FROM_DEVICE : DMA_TO_DEVICE);
	    if (unlikely(!bio_info)) {
		dev_err(dev, "Unable to allocate bio_info\n");
		goto end;
	    }

	    bio_set_dev(bio_info->bio, bpt->real_bd);
	    bio_info->bio->bi_iter.bi_sector = loc_descr.s_sector;
	    bio_info->bio->bi_opf = loc_descr.opf == WRITE ? REQ_OP_WRITE : REQ_OP_READ;
	    if (loc_descr.opf == WRITE) {
		ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, bio_info->phys_addr, loc_descr.addr, loc_descr.len);
		if (ret) {
		    dev_err(dev, "Failed to map descriptor: %i\n", ret);
		    free_bpt_info(bio_info);
		    goto end;
		}
#ifdef USE_DMA
		ret = pci_epc_start_single_dma(epf->epc, epf->func_no, epf->vfunc_no, 1, bio_info->phys_addr, page_to_phys(bio_info->page), descr->len, &bio_info->dma_transfer_complete);
		if (ret) {
		    dev_err(dev, "Failed to start single DMA read\n");
		} else {
		    ret = wait_for_completion_interruptible(&bio_info->dma_transfer_complete);
		    if (ret < 0)
			dev_err(dev, "DMA wait_for_completion Timed out\n");
		}
		pci_epc_unmap_addr(epf->epc, epf->func_no, epf->vfunc_no, bio_info->phys_addr);
		pci_epc_mem_free_addr(epf->epc, bio_info->phys_addr, bio_info->addr, bio_info->size);
		if (ret < 0) {
		    free_bpt_info(bio_info);
		    goto end;
		}

#else
		memcpy_fromio(page_address(bio_info->page), bio_info->addr, loc_descr.len);
#endif
	    }

	    bpt->drv_idx = (bpt->drv_idx + 1) % bpt->num_desc;
	    bio_info->bio->bi_end_io = pci_epf_blockpt_transfer_complete;
	    bio_info->bio->bi_private = bio_info;
	    bio_add_page(bio_info->bio, bio_info->page, loc_descr.len, loc_descr.offset);
	    submit_bio(bio_info->bio);
	}
      end:
	usleep_range(500, 1000);
    }
    return 0;
}

static int pci_blockpt_digest(void *cookie)
{
	struct pci_epf_blockpt *bpt = (struct pci_epf_blockpt *)cookie;
	struct device *dev = &bpt->epf->dev;
	struct pci_blockpt_device_ring __iomem *dev_ring = bpt->device_ring;
	struct pci_epf *epf = bpt->epf;
	struct pci_epf_blockpt_info *bi;
	__maybe_unused char *buf;
	struct pci_epf_blockpt_descr loc_descr;
	int ret;
	
	allow_signal(SIGINT);
	allow_signal(SIGTERM);
	allow_signal(SIGKILL);
	allow_signal(SIGUSR1);

	while (!kthread_should_stop()) {
	    spin_lock_irq(&bpt->lock);
	    bi = list_first_entry_or_null(&bpt->proc_list, struct pci_epf_blockpt_info, node);
	    spin_unlock_irq(&bpt->lock);
	    if (bi == NULL) {
		usleep_range(1000, 5000);
		continue;
	    }

	    memcpy_fromio(&loc_descr, bi->descr, sizeof(loc_descr));
	    BUG_ON(!(loc_descr.flags & PBI_EPF_BLOCKPT_F_USED));
	    if (loc_descr.opf == READ) {
		ret = pci_epc_map_addr(epf->epc, epf->func_no, epf->vfunc_no, bi->phys_addr, loc_descr.addr, loc_descr.len);

		if (ret) {
		    dev_err(dev, "Failed to map buffer address\n");
		    continue;
		}

#ifdef USE_DMA
		ret = pci_epc_start_single_dma(epf->epc, epf->func_no, epf->vfunc_no, 0, bi->dma_addr, bi->phys_addr, loc_descr.len, &bi->dma_transfer_complete);
		if (ret) {
		    dev_err(dev, "Failed to start single DMA read: %i\n", ret);
		    pci_epc_unmap_addr(epf->epc, epf->func_no, epf->vfunc_no, bi->phys_addr);
		    continue;
		} else {
		    ret = wait_for_completion_interruptible_timeout(&bi->dma_transfer_complete, msecs_to_jiffies(10));
		    if (ret < 0) {
			dev_err(dev, "DMA wait_for_completion Timed out\n");
		    } 
		}

#else
		buf = kmap_atomic(bi->page);	
		dev_dbg(dev, "memcpy_toio() dest: 0x%llX, src: 0x%llX, len: 0x%x. dev_idx = %i, descr_idx = %i\n", (u64)bi->addr, (u64)buf, bi->descr->len, bpt->dev_idx, bi->descr_idx);
		memcpy_toio(bi->addr, buf, bi->descr->len);
		kunmap_atomic(buf);
#endif

	    }
	    writew(bi->descr_idx, &dev_ring->ring[bpt->dev_idx]);
	    bpt->dev_idx = (bpt->dev_idx + 1) % bpt->num_desc;
	    writew(bpt->dev_idx, &dev_ring->idx);
	    free_bpt_info(bi);
	}

	return 0;
}

static int pci_epf_blockpt_probe(struct pci_epf *epf)
{
	struct pci_epf_blockpt *bpt;
	struct device *dev = &epf->dev;
	int err;
	
	bpt = devm_kzalloc(dev, sizeof(*bpt), GFP_KERNEL);
	if (!bpt)
		return -ENOMEM;

	epf->header = &pci_blockpt_header;
	bpt->epf = epf;
	spin_lock_init(&bpt->lock);

	INIT_LIST_HEAD(&bpt->proc_list);
	INIT_DELAYED_WORK(&bpt->cmd_handler, pci_epf_blockpt_cmd_handler);

	epf_set_drvdata(epf, bpt);
	err = setup_block_device(bpt);
	if (err) {
	    dev_err(dev, "Could not get block device %s (%i)\n", pt_block_dev_name, err);
	    goto free_dev;
	}
	bpt->digest_thr = kthread_create(pci_blockpt_digest, bpt, "kpci-blockpt-digest");

	if (IS_ERR(bpt->digest_thr)) {
	    err = PTR_ERR(bpt->digest_thr);
	    dev_err(dev, "Could not create kernel thread %i\n", err);
	    goto free_bd;
	}

	bpt->produce_thr = kthread_create(pci_blockpt_produce, bpt, "kpci-blockpt-produce");

	if (IS_ERR(bpt->produce_thr)) {
	    err = PTR_ERR(bpt->produce_thr);
	    dev_err(dev, "Could not create kernel thread %i\n", err);
	    goto free_bd;
	}
	
	return 0;
free_bd:
	blkdev_put(bpt->real_bd, FMODE_READ | FMODE_WRITE);
free_dev:
	devm_kfree(dev, bpt);
	return err;
	
}

static void pci_epf_blockpt_remove(struct pci_epf *epf)
{
    struct pci_epf_blockpt *bpt = epf_get_drvdata(epf);
    dev_err(&epf->dev,"%s called\n", __FUNCTION__);
    kthread_stop(bpt->digest_thr);
    put_task_struct(bpt->digest_thr);
    blkdev_put(bpt->real_bd, FMODE_READ | FMODE_WRITE);
}

static struct pci_epf_ops blockpt_ops = {
	.unbind	= pci_epf_blockpt_unbind,
	.bind	= pci_epf_blockpt_bind,
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
MODULE_AUTHOR("Wadim Mueller <wafgo01@gmail.com>");
MODULE_LICENSE("GPL v2");
