// SPDX-License-Identifier: GPL-2.0
/*
 * Block Device Passthrough as an Endpoint Function driver
 *
 * Copyright (C) 2023 Continental Automotive Technologies
 * Author: Wadim Mueller <wadim.mueller@continental.com>
 */

/* #define DEBUG 1 */
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

//#define USE_DMA

static char *pt_block_dev_name = "/dev/mmcblk0";
//static char *pt_block_dev_name = "/dev/nvme0n1";

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

#define STATUS_IRQ_RAISED		BIT(6)
#define STATUS_QUEUE_ADDR_INVALID	BIT(9)

#define DEFAULT_WORK_DELAY_MS           (1)
#define LONG_WORK_DELAY_MS              (100)

static struct workqueue_struct *kpciblockpt_wq;

struct pci_epf_blockpt_descr {
    sector_t s_sector; /* start sector of the request */
    u64 addr; /* where the data is  */
    u64 flags;
    u64 opf;
    u64 tag;
    u32 len; /* bytes to pu at addr + s_offset*/
    u32 offset;  /* offset from addr */
    u32 status;
    u32 res0;
};

struct pci_blockpt_driver_ring_entry {
    u16 index;
    u16 flags;
};

struct pci_blockpt_device_ring_entry {
    u16 index;
    u16 flags;
};

struct pci_blockpt_driver_ring {
    u16 flags;
    u16 idx;
    struct pci_blockpt_driver_ring_entry ring[]; /* queue size*/
};

struct pci_blockpt_device_ring {
    u16 flags;
    u16 idx;
    u32 len;
    struct pci_blockpt_device_ring_entry ring[]; /* queue size*/
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
        u32 drv_offset;
        u32 dev_offset;
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
	u32	irq_type;
	u32	irq_number;
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
    struct pci_epf_blockpt *epf_bio;
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
};

static size_t bar_size[] = { 512, 512, 1024, 16384, 131072, 1048576};

static int pci_epf_blockpt_map_queue(struct pci_epf_blockpt *epf_bio, struct pci_epf_blockpt_reg *reg) {
        int ret;
	void __iomem *addr;
	phys_addr_t phys_addr;
	struct pci_epf *epf = epf_bio->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;

	addr = pci_epc_mem_alloc_addr(epc, &phys_addr, epf_bio->descr_size);
	dev_dbg(dev, "pci_epc_mem_alloc_addr() phys_addr = 0x%llX, size = 0x%x, virt_addr = 0x%llX\n", phys_addr, epf_bio->descr_size, addr);
	if (!addr) {
		dev_err(dev, "Failed to allocate queue address\n");
		reg->status = STATUS_QUEUE_ADDR_INVALID;
		return -ENOMEM;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, phys_addr,
			       epf_bio->descr_addr, epf_bio->descr_size);
	dev_dbg(dev, "pci_epc_map_addr() func_no = %i, vfunc_no = %i, descr_addr = 0x%llX size = 0x%x\n", epf->func_no, epf->vfunc_no, epf_bio->descr_addr, epf_bio->descr_size);
	
	if (ret) {
		dev_err(dev, "Failed to map queue address\n");
		reg->status = STATUS_QUEUE_ADDR_INVALID;
		ret = -EINVAL;
		goto err_queue_addr;
	}

	epf_bio->descr = addr;
	dev_dbg(dev, "\tQUEUE => Queue Address physical: 0x%llX\t Queue Address virt: 0x%llX\t Queue Addr PCI: 0x%llX\n",
		phys_addr, addr, epf_bio->descr_addr);
	return 0;
err_queue_addr:
	pci_epc_mem_free_addr(epc, phys_addr, addr, epf_bio->descr_size);
	return ret;
}

static struct pci_blockpt_driver_ring *get_driver_ring(struct pci_epf_blockpt *bdev)
{
    return (struct pci_blockpt_driver_ring *)((u64)bdev->descr + bdev->drv_offset);
}

static struct pci_blockpt_device_ring *get_device_ring(struct pci_epf_blockpt *bdev)
{
    return (struct pci_blockpt_device_ring *)((u64)bdev->descr + bdev->dev_offset);
}

static void reset_cmd_handler(struct pci_epf_blockpt *epf_bio)
{
    queue_delayed_work(kpciblockpt_wq, &epf_bio->cmd_handler,
		       msecs_to_jiffies(1));
}

static void pci_epf_setup(struct pci_epf_blockpt *epf_bio)
{
    u32 command;
    enum pci_barno bio_reg_bar = epf_bio->bio_reg_bar;
    struct pci_epf *epf = epf_bio->epf;
    struct pci_epf_blockpt_reg *reg = epf_bio->reg[bio_reg_bar];
    struct device *dev = &epf->dev;
    struct pci_blockpt_driver_ring *drv_ring;
    struct pci_epf_blockpt_descr __iomem *descr;
    int i;
    
    command = reg->command;

    if (!command)
	goto reset_handler;

    reg->command = 0;
    reg->status = 0;

    if (command & COMMAND_ADD_DRV_ENTRY) {
	dev_dbg(dev, "-> Received new driver entry\n");
	drv_ring = get_driver_ring(epf_bio);
	dev_dbg(dev, "Received new index = %i\n", drv_ring->idx);
    }

    	
    if (command & COMMAND_WRITE_TO_QUEUE) {
	for (i = 0; i < 16; ++i) {
	    descr = &epf_bio->descr[i];
	    descr->addr = 0xa5a5a5a5 + i;
	    descr->s_sector = 0xa5;
	    descr->offset = 0x5a;
	    descr->len = 0x5a;
	}
    }
	
    if (command & COMMAND_SET_QUEUE) {
	dev_dbg(dev, "mapping Queue to physical address: 0x%llX with size = 0x%llX\n", reg->queue_addr, reg->queue_size);
	if (epf_bio->descr_addr != 0) {
	    dev_err(dev, "QUEUE is already mapped: 0x%llX\n", epf_bio->descr_addr);
	    goto reset_handler;
	}
	
	epf_bio->num_desc = reg->num_desc;
	if (epf_bio->num_desc <= 0) {
	    dev_emerg(dev, "number of descriptors is invalid: %i\n", epf_bio->num_desc);
	    goto reset_handler;
	}
	epf_bio->descr_addr = reg->queue_addr;
	epf_bio->descr_size = reg->queue_size;
	epf_bio->drv_offset = reg->drv_offset;
	epf_bio->dev_offset = reg->dev_offset;
	pci_epf_blockpt_map_queue(epf_bio, reg);
    }

    if (command & COMMAND_START) {
	dev_dbg(dev, "Starting Up\n");
	if (epf_bio->descr_addr == 0) {
	    dev_err(dev, "Queue is not mapped, cannot start anything\n");
	    goto reset_handler;
	}
	epf_bio->state = PCI_BLOCKPT_RUNNING;
	wake_up_process(epf_bio->produce_thr);
    }

reset_handler:
    reset_cmd_handler(epf_bio);
}

static void free_epf_bio_info(struct pci_epf_blockpt_info *info)
{
    struct device *dev = &info->epf_bio->epf->dev;
    struct device *dma_dev = info->epf_bio->epf->epc->dev.parent;

    dma_unmap_single(dma_dev, info->dma_addr, info->size, DMA_BIDIRECTIONAL);

    pci_epc_unmap_addr(info->epf_bio->epf->epc, info->epf_bio->epf->func_no, info->epf_bio->epf->vfunc_no, info->phys_addr);
    pci_epc_mem_free_addr(info->epf_bio->epf->epc, info->phys_addr, info->addr, info->size);
    __free_pages(info->page, info->page_order);
    
    spin_lock_bh(&info->epf_bio->lock);
    list_del(&info->node);
    spin_unlock_bh(&info->epf_bio->lock);
    bio_put(info->bio);    
    devm_kfree(dev, info);
}

struct pci_epf_blockpt_info *alloc_pci_epf_blockpt_info(struct pci_epf_blockpt *epf_bio, size_t size, struct pci_epf_blockpt_descr __iomem *descr, int descr_idx)
{
    struct pci_epf_blockpt_info *binfo;
    struct bio *bio;
    struct device *dev = &epf_bio->epf->dev;
    struct page *page;
    struct device *dma_dev = epf_bio->epf->epc->dev.parent;
    dma_addr_t dma_addr;
    gfp_t alloc_flags = GFP_ATOMIC;
    
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
	dev_err(dev, "Could not allocate pages for BIO\n");
	goto free_bio;
    }

    binfo->addr = pci_epc_mem_alloc_addr(epf_bio->epf->epc, &binfo->phys_addr, size);
    if (!binfo->addr) {
	dev_err(dev, "Failed to allocate bio pci addr address\n");
	goto release_page;
    }

    dma_addr = dma_map_single(dma_dev, page_address(page), descr->len, DMA_BIDIRECTIONAL);
    if (dma_mapping_error(dma_dev, dma_addr)) {
	dev_err(dev, "Failed to map buffer addr\n");
	goto free_epc_mem;
    }

    dev_dbg(dev, "%s: pci_epc_mem_alloc_addr() phys_addr = 0x%llX, size = 0x%x, virt_addr = 0x%llX\n", __FUNCTION__, binfo->phys_addr, size, binfo->addr);
    init_completion(&binfo->dma_transfer_complete);
    binfo->bio = bio;
    binfo->dma_addr = dma_addr;
    binfo->epf_bio = epf_bio;
    binfo->page = page;
    binfo->descr = descr;
    binfo->descr_idx = descr_idx;
    return binfo;
free_epc_mem:
    pci_epc_mem_free_addr(binfo->epf_bio->epf->epc, binfo->phys_addr, binfo->addr, size);
release_page:
    __free_pages(page, binfo->page_order);
free_bio:
    bio_put(bio);
free_binfo:
    devm_kfree(dev, binfo);
    return NULL;
}


static void pci_epf_blockpt_transfer_complete(struct bio *bio)
{
    
    struct pci_epf_blockpt_info *binfo = bio->bi_private;
    struct device *dev = &binfo->epf_bio->epf->dev;
    
    if (bio->bi_status != BLK_STS_OK) {
	dev_err(dev, "BIO error %i\n", bio->bi_status);
    }

    dev_info(dev, "BIO %i\n", binfo->descr_idx);
    
    spin_lock(&binfo->epf_bio->lock);
    list_add_tail(&binfo->node, &binfo->epf_bio->proc_list);
    spin_unlock(&binfo->epf_bio->lock);
    /* wakeup task*/
    wake_up_process(binfo->epf_bio->digest_thr);
}


static void pci_epf_blockpt_cmd_handler(struct work_struct *work)
{
	int count;
	u32 command;
	struct pci_epf_blockpt *epf_bio = container_of(work, struct pci_epf_blockpt,
						     cmd_handler.work);
	struct pci_epf *epf = epf_bio->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno bio_reg_bar = epf_bio->bio_reg_bar;
	struct pci_epf_blockpt_reg *reg = epf_bio->reg[bio_reg_bar];
	struct pci_epf_blockpt_descr __iomem *descr;
	struct pci_blockpt_driver_ring *drv_ring;
	int ret;
	struct pci_blockpt_driver_ring_entry *de;
	struct pci_epf_blockpt_info *bio_info;
	int delay_ms = DEFAULT_WORK_DELAY_MS;
	
	if (epf_bio->state == PCI_BLOCKPT_SETUP)
	    return pci_epf_setup(epf_bio);
	
/* 	command = reg->command; */

/* 	drv_ring = get_driver_ring(epf_bio);	 */

/* 	reg->command = 0; */
/* 	reg->status = 0; */

/* 	if (command & COMMAND_RAISE_MSI_IRQ) { */
/* 		count = pci_epc_get_msi(epc, epf->func_no, epf->vfunc_no); */
/* 		if (reg->irq_number > count || count <= 0) */
/* 			goto reset_handler; */
/* 		reg->status = STATUS_IRQ_RAISED; */
/* 		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no, */
/* 				  PCI_EPC_IRQ_MSI, reg->irq_number); */
/* 		goto reset_handler; */
/* 	} */

/* 	if (command & COMMAND_RAISE_MSIX_IRQ) { */
/* 		count = pci_epc_get_msix(epc, epf->func_no, epf->vfunc_no); */
/* 		if (reg->irq_number > count || count <= 0) */
/* 			goto reset_handler; */
/* 		reg->status = STATUS_IRQ_RAISED; */
/* 		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no, */
/* 				  PCI_EPC_IRQ_MSIX, reg->irq_number); */
/* 		goto reset_handler; */
/* 	} */

/* 	if (epf_bio->drv_idx != drv_ring->idx) { */
/* 	    dev_dbg(dev, "New entry in driver ring to process. Internal counter = %i, driver counter = %i\n", epf_bio->drv_idx, drv_ring->idx); */
/* 	    while (epf_bio->drv_idx != drv_ring->idx) { */
/* 		de = &drv_ring->ring[epf_bio->drv_idx]; */
/* 		descr = &epf_bio->descr[de->index]; */
/* 		dev_dbg(dev, "Driver ring Index to process: %i, flags: %i\n", de->index, de->flags); */
/* 		dev_dbg(dev, "DescrInfo:  sector: 0x%llX, len: 0x%x, dir: %s\n", descr->s_sector, descr->len, (descr->opf == WRITE) ? "WRITE" : "READ"); */
/* 		bio_info = alloc_pci_epf_blockpt_info(epf_bio, descr->len, descr, de->index); */
/* 		if (unlikely(!bio_info)) { */
/* 		    dev_err(dev, "Unable to allocate bio_info\n"); */
/* 		    delay_ms = LONG_WORK_DELAY_MS; */
/* 		    goto reset_handler; */
/* 		} */

/* 		bio_set_dev(bio_info->bio, epf_bio->real_bd); */
/* 		bio_info->bio->bi_iter.bi_sector = descr->s_sector; */
/* 		/\*FIXME: HACK*\/ */
/* 		bio_info->bio->bi_opf = (descr->opf == WRITE) ? REQ_OP_WRITE : REQ_OP_READ; */
/* 		if (descr->opf == WRITE) { */
/* 		    ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, bio_info->phys_addr, descr->addr, descr->len); */
		    
/* 		    if (ret) { */
/* 			dev_err(dev, "Failed to map queue address\n"); */
/* 			free_epf_bio_info(bio_info); */
/* 			delay_ms = LONG_WORK_DELAY_MS; */
/* 			goto reset_handler; */
/* 		    } */
/* //#ifdef USE_DMA */
/* #if 0		     */
/* 		    ret = pci_epc_start_single_dma(epf->epc, epf->func_no, epf->vfunc_no, 1, bio_info->phys_addr, page_to_phys(bio_info->page), descr->len, &bio_info->dma_transfer_complete); */
/* 		    if (ret) { */
/* 			dev_err(dev, "Failed to start single DMA read\n"); */
/* 		    } else { */
/* 			ret = wait_for_completion_interruptible(&bio_info->dma_transfer_complete); */
/* 			if (ret < 0) */
/* 			    dev_err(dev, "DMA wait_for_completion Timed out\n"); */
/* 		    } */
/* #else */
/* 		    memcpy_fromio(page_address(bio_info->page), bio_info->addr, descr->len); */
/* #endif */
/* 		} */
		
/* 		epf_bio->drv_idx = (epf_bio->drv_idx + 1) % epf_bio->num_desc; */
/* 		bio_info->bio->bi_end_io = pci_epf_blockpt_transfer_complete; */
/* 		bio_info->bio->bi_private = bio_info; */
/* 		bio_add_page(bio_info->bio, bio_info->page, descr->len, descr->offset); */
/* 		submit_bio(bio_info->bio); */
/* 	    } */
/* 	} */

/* reset_handler: */
/* 	queue_delayed_work(kpciblockpt_wq, &epf_bio->cmd_handler, */
/* 			   msecs_to_jiffies(delay_ms)); */
}

static void pci_epf_blockpt_unbind(struct pci_epf *epf)
{
	struct pci_epf_blockpt *epf_bio = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct pci_epf_bar *epf_bar;
	int bar;

	dev_dbg(&epf->dev, "%s called\n", __FUNCTION__);
	cancel_delayed_work(&epf_bio->cmd_handler);
	for (bar = 0; bar < PCI_STD_NUM_BARS; bar++) {
		epf_bar = &epf->bar[bar];

		if (epf_bio->reg[bar]) {
			pci_epc_clear_bar(epc, epf->func_no, epf->vfunc_no,
					  epf_bar);
			pci_epf_free_space(epf, epf_bio->reg[bar], bar,
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
	struct pci_epf_blockpt *epf_bio = epf_get_drvdata(epf);
	enum pci_barno bio_reg_bar = epf_bio->bio_reg_bar;
	const struct pci_epc_features *epc_features;

	epc_features = epf_bio->epc_features;

	dev_dbg(dev, "Setting BIO BAR%d\n", bio_reg_bar);

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

		ret = pci_epc_set_bar(epc, epf->func_no, epf->vfunc_no,
				      epf_bar);
		if (ret) {
			pci_epf_free_space(epf, epf_bio->reg[bar], bar,
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
	struct pci_epf_blockpt *epf_bio = epf_get_drvdata(epf);
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
				       epf_bio->bio_reg_bar,
				       epf_bio->msix_table_offset);
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
	struct pci_epf_blockpt *epf_bio = epf_get_drvdata(epf);
	int ret;

	switch (val) {
	case CORE_INIT:
		ret = pci_epf_blockpt_core_init(epf);
		if (ret)
			return NOTIFY_BAD;
		break;

	case LINK_UP:
		queue_delayed_work(kpciblockpt_wq, &epf_bio->cmd_handler,
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
	struct pci_epf_blockpt *epf_bio = epf_get_drvdata(epf);
	struct device *dev = &epf->dev;
	struct pci_epf_bar *epf_bar;
	size_t msix_table_size = 0;
	size_t bio_reg_bar_size;
	size_t pba_size = 0;
	bool msix_capable;
	void *base;
	int bar, add;
	enum pci_barno bio_reg_bar = epf_bio->bio_reg_bar;
	const struct pci_epc_features *epc_features;
	size_t bar_reg_size;

	epc_features = epf_bio->epc_features;

	bio_reg_bar_size = ALIGN(sizeof(struct pci_epf_blockpt_reg), 128);

	msix_capable = epc_features->msix_capable;
	if (msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_bio->msix_table_offset = bio_reg_bar_size;
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
	epf_bio->reg[bio_reg_bar] = base;
	
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
		epf_bio->reg[bar] = base;
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
	struct pci_epf_blockpt *epf_bio = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno bio_reg_bar = BAR_0;
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
	bio_reg_bar = pci_epc_get_first_free_bar(epc_features);
	if (bio_reg_bar < 0)
		return -EINVAL;
	
	pci_epf_configure_bar(epf, epc_features);

	epf_bio->bio_reg_bar = bio_reg_bar;
	epf_bio->epc_features = epc_features;

	ret = pci_epf_blockpt_alloc_space(epf);
	if (ret)
		return ret;

	breg = (struct pci_epf_blockpt_reg *) epf_bio->reg[bio_reg_bar];
	breg->magic = BLOCKPT_MAGIC;
	breg->num_sectors = get_capacity(bdev_whole(epf_bio->real_bd)->bd_disk);
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
		queue_work(kpciblockpt_wq, &epf_bio->cmd_handler.work);
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
		goto out_free_dev;
	}

	spin_lock_init(&bpt->lock);
	return 0;
out_free_dev:
	return err;
}

static int pci_blockpt_produce(void *cookie)
{
    struct pci_epf_blockpt *epf_bio = (struct pci_epf_blockpt *)cookie;
    struct device *dev = &epf_bio->epf->dev;
    struct pci_blockpt_driver_ring __iomem *drv_ring = get_driver_ring(epf_bio);
    struct pci_epf *epf = epf_bio->epf;
    struct pci_epc *epc = epf->epc;
    struct pci_blockpt_driver_ring_entry *de;
    struct pci_epf_blockpt_info *bio_info;
    struct pci_epf_blockpt_descr __iomem *descr;
    int ret;

    while (!kthread_should_stop()) {
	/* dev_info_ratelimited(dev, "%s\n", __FUNCTION__); */
	if (epf_bio->drv_idx != drv_ring->idx) {
	    dev_dbg(dev, "New entry in driver ring to process. Internal counter = %i, driver counter = %i\n", epf_bio->drv_idx, drv_ring->idx);
	    while (epf_bio->drv_idx != drv_ring->idx) {
		de = &drv_ring->ring[epf_bio->drv_idx];
		descr = &epf_bio->descr[de->index];
		WARN(!(descr->flags & PBI_EPF_BLOCKPT_F_USED), "Process unused descr %i", epf_bio->drv_idx);
		dev_dbg(dev, "Driver ring Index to process: %i, flags: %i\n", de->index, de->flags);
		dev_dbg(dev, "DescrInfo:  sector: 0x%llX, len: 0x%x, dir: %s\n", descr->s_sector, descr->len, (descr->opf == WRITE) ? "WRITE" : "READ");
		bio_info = alloc_pci_epf_blockpt_info(epf_bio, descr->len, descr, de->index);
		if (unlikely(!bio_info)) {
		    dev_err(dev, "Unable to allocate bio_info\n");
		    goto end;
		}

		bio_set_dev(bio_info->bio, epf_bio->real_bd);
		bio_info->bio->bi_iter.bi_sector = descr->s_sector;
		/*FIXME: HACK*/
		bio_info->bio->bi_opf = (descr->opf == WRITE) ? REQ_OP_WRITE : REQ_OP_READ;
		if (descr->opf == WRITE) {
		    ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, bio_info->phys_addr, descr->addr, descr->len);
		    
		    if (ret) {
			dev_err(dev, "Failed to map queue address\n");
			free_epf_bio_info(bio_info);
			goto end;
		    }
//#ifdef USE_DMA
#if 0		    
		    ret = pci_epc_start_single_dma(epf->epc, epf->func_no, epf->vfunc_no, 1, bio_info->phys_addr, page_to_phys(bio_info->page), descr->len, &bio_info->dma_transfer_complete);
		    if (ret) {
			dev_err(dev, "Failed to start single DMA read\n");
		    } else {
			ret = wait_for_completion_interruptible(&bio_info->dma_transfer_complete);
			if (ret < 0)
			    dev_err(dev, "DMA wait_for_completion Timed out\n");
		    }
#else
		    memcpy_fromio(page_address(bio_info->page), bio_info->addr,
				  descr->len);
#endif
		}

		dev_info(dev, "Prod %i (%i)\n", epf_bio->drv_idx, de->index);
		epf_bio->drv_idx = (epf_bio->drv_idx + 1) % epf_bio->num_desc;
		bio_info->bio->bi_end_io = pci_epf_blockpt_transfer_complete;
		bio_info->bio->bi_private = bio_info;
		bio_add_page(bio_info->bio, bio_info->page, descr->len,
			     descr->offset);
		bio_get(bio_info->bio);
		ret = submit_bio(bio_info->bio);
	    }
end:
	    usleep_range(5000, 10000);
	} else {
	    usleep_range(5000, 10000);
	}
    }
    return 0;
}

static int pci_blockpt_digest(void *cookie)
{
	struct pci_epf_blockpt *ebio = (struct pci_epf_blockpt *)cookie;
	struct device *dev = &ebio->epf->dev;
	struct pci_blockpt_device_ring __iomem *dev_ring = get_device_ring(ebio);
	struct pci_epf *epf = ebio->epf;
	struct pci_epf_blockpt_info *bi;
	char *buf;
	int ret;
	
	allow_signal(SIGINT);
	allow_signal(SIGTERM);
	allow_signal(SIGKILL);
	allow_signal(SIGUSR1);

	while (!kthread_should_stop()) {
	    /* dev_info_ratelimited(dev, "%s\n", __FUNCTION__); */

	    /* rcu_read_lock(); */
	    spin_lock_bh(&ebio->lock);
	    bi = list_first_entry_or_null(&ebio->proc_list, struct pci_epf_blockpt_info, node);
	    spin_unlock_bh(&ebio->lock);
	    /* rcu_read_unlock(); */

	    if (bi == NULL) {
		usleep_range(1000, 5000);
		continue;
	    }

	    if (bi->descr->opf == READ) {
		ret = pci_epc_map_addr(epf->epc, epf->func_no, epf->vfunc_no, bi->phys_addr, bi->descr->addr, bi->descr->len);

		if (ret) {
		    dev_err(dev, "Failed to map buffer address\n");
		    continue;
		}
		buf = kmap_atomic(bi->page);	
#ifdef USE_DMA
		dev_dbg(dev, "Trying DMA src: 0x%llX, dst: 0x%llX. PHYS_ADDR 0x%llX (dma_addr: 0x%llX, phys_address: 0x%llX, virt_address: 0x%llX) is mapped to 0x%llX. Length: 0x%x\n", bi->dma_addr, bi->phys_addr, bi->phys_addr, bi->dma_addr, page_to_phys(bi->page), buf, bi->descr->addr, bi->descr->len);
		
		ret = pci_epc_start_single_dma(epf->epc, epf->func_no, epf->vfunc_no, 0, bi->dma_addr, bi->phys_addr, bi->descr->len, &bi->dma_transfer_complete);
		if (ret) {
		    dev_err(dev, "Failed to start single DMA read: %i\n", ret);
		} else {
		    ret = wait_for_completion_interruptible_timeout(&bi->dma_transfer_complete, msecs_to_jiffies(10));
		    if (ret < 0) {
			dev_err(dev, "DMA wait_for_completion Timed out\n");
		    } 
		}

#else
		dev_dbg(dev, "memcpy_toio() dest: 0x%llX, src: 0x%llX, len: 0x%x. dev_idx = %i, descr_idx = %i\n", (u64)bi->addr, (u64)buf, bi->descr->len, ebio->dev_idx, bi->descr_idx);
		memcpy_toio(bi->addr, buf, bi->descr->len);
#endif
		kunmap_atomic(buf);
	    }
	    dev_info(dev, "Consume (%i)\n", bi->descr_idx);
	    /* spin_lock(&ebio->lock); */
	    dev_ring->ring[ebio->dev_idx].index = bi->descr_idx;
	    dev_ring->idx = ebio->dev_idx = (ebio->dev_idx + 1) % ebio->num_desc;
	    /* spin_unlock(&ebio->lock); */
	    free_epf_bio_info(bi);
	    	    
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
