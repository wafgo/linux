// SPDX-License-Identifier: GPL-2.0
/*
 * Remote Blockdevice as an Endpoint Function driver
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
#include <linux/random.h>

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
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/hdreg.h>


static int major;
static struct workqueue_struct *deferred_bio_add_workqueue;

#define PBI_EPF_BIO_F_LAST BIT(0)
#define PBI_EPF_BIO_F_USED BIT(1)
#define PBI_EPF_BIO_F_DIR_READ BIT(2)

#define NUM_DESRIPTORS                  512

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


#define STATUS_READ_SUCCESS		BIT(0)
#define STATUS_READ_FAIL		BIT(1)
#define STATUS_WRITE_SUCCESS		BIT(2)
#define STATUS_WRITE_FAIL		BIT(3)
#define STATUS_COPY_SUCCESS		BIT(4)
#define STATUS_COPY_FAIL		BIT(5)
#define STATUS_IRQ_RAISED		BIT(6)
#define STATUS_SRC_ADDR_INVALID		BIT(7)
#define STATUS_DST_ADDR_INVALID		BIT(8)
#define STATUS_QUEUE_ADDR_INVALID	BIT(9)

#define FLAG_USE_DMA			BIT(0)
#define FLAG_USE_SINGLE_DMA		BIT(1)

#define TIMER_RESOLUTION		1

static struct workqueue_struct *kpcibio_workqueue;

struct pci_epf_bio_descr {
    sector_t s_sector; /* start sector of the request */
    u64 addr; /* where the data is  */
    u64 flags;
    u64 opf;
    u64 tag;
    u32 len; /* bytes to pu at addr + s_offset*/
    u32 offset;  /* offset from addr */
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

enum cmd_state {
    PCI_BIO_PARKED = 0,
    PCI_BIO_RUNNING,
};

struct pci_epf_bio {
	void			*reg[PCI_STD_NUM_BARS];
	struct pci_epf		*epf;
	enum pci_barno		bio_reg_bar;
	size_t			msix_table_offset;
	struct delayed_work	cmd_handler;
	struct dma_chan		*dma_chan;
	struct completion	transfer_complete;
	bool			dma_supported;
	const struct pci_epc_features *epc_features;
        struct pci_epf_bio_descr __iomem *descr;
    	struct gendisk *gd;
	struct blk_mq_tag_set tag_set;
	struct bio_set bset;
	struct block_device *real_bd;
        dma_addr_t descr_addr;
        u32 descr_size;
        u32 drv_offset;
        u32 dev_offset;
        u32 drv_idx;
        u32 dev_idx;
	char *device_path;
	spinlock_t lock;
    enum cmd_state state;
	const struct blk_mq_queue_data *bd;

};

struct pci_epf_bio_reg {
	u32	magic;
	u32	command;
	u32	status;
        u32     queue_size;  /* number of struct pci_epf_bio_descr */
        u32     drv_offset;
        u32     dev_offset;
	u32	irq_type;
	u32	irq_number;
	u32	flags;
        u32	flags2;
        u64	queue_addr;  /* start of struct pci_epf_bio_descr*/
        u64     num_sectors;
        char    dev_name[64];
} __packed;

static struct pci_epf_header test_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
};

static char *block_dev_name = "/dev/mmcblk0";
module_param(block_dev_name, charp, 0444);

#define PCI_REMOTE_BLOCK_MINORS 16
#define PCI_REMOTE_BLKDEV_NAME "remote-bd"
#define DRV_MODULE_NAME "pci-remote-bdev"

#define PCI_RBD_INLINE_SG_CNT 2

struct pci_remote_bd_data {
	enum pci_barno reg_bar;
	size_t alignment;
};


struct pci_remote_bd_request {
	struct pci_epf_bio *rdev;
	struct bio *bio;
	u8 status;
	struct page *pg;
	int order;
	int num_bios;
	struct work_struct bio_work;
	struct sg_table sg_table;
	struct scatterlist sg[];
};

static const struct pci_remote_bd_data rbd_default_data = {
	.reg_bar = BAR_0,
	.alignment = SZ_4K,
};

static size_t bar_size[] = { 512, 512, 1024, 16384, 131072, 1048576 };

/**
 * pci_epf_bio_init_dma_chan() - Function to initialize EPF test DMA channel
 * @epf_bio: the EPF test device that performs data transfer operation
 *
 * Function to initialize EPF test DMA channel.
 */
static int pci_epf_bio_init_dma_chan(struct pci_epf_bio *epf_bio)
{
	struct pci_epf *epf = epf_bio->epf;
	struct device *dev = &epf->dev;
	struct dma_chan *dma_chan;
	dma_cap_mask_t mask;
	int ret;

	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);

	dma_chan = dma_request_chan_by_mask(&mask);
	if (IS_ERR(dma_chan)) {
		ret = PTR_ERR(dma_chan);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "Failed to get DMA channel\n");
		return ret;
	}
	init_completion(&epf_bio->transfer_complete);

	epf_bio->dma_chan = dma_chan;

	return 0;
}

/**
 * pci_epf_bio_clean_dma_chan() - Function to cleanup EPF test DMA channel
 * @epf_bio: the EPF test device that performs data transfer operation
 *
 * Helper to cleanup EPF test DMA channel.
 */
static void pci_epf_bio_clean_dma_chan(struct pci_epf_bio *epf_bio)
{
	if (!epf_bio->dma_supported)
		return;

	dma_release_channel(epf_bio->dma_chan);
	epf_bio->dma_chan = NULL;
}

static int pci_epf_bio_map_queue(struct pci_epf_bio *epf_bio, struct pci_epf_bio_reg *reg) {
        int ret;
	void __iomem *addr;
	phys_addr_t phys_addr;
	struct pci_epf *epf = epf_bio->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;

	addr = pci_epc_mem_alloc_addr(epc, &phys_addr, epf_bio->descr_size);
	dev_info(dev, "pci_epc_mem_alloc_addr() phys_addr = 0x%llX, size = 0x%x, virt_addr = 0x%llX\n", phys_addr, epf_bio->descr_size, addr);
	if (!addr) {
		dev_err(dev, "Failed to allocate queue address\n");
		reg->status = STATUS_QUEUE_ADDR_INVALID;
		return -ENOMEM;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, phys_addr,
			       epf_bio->descr_addr, epf_bio->descr_size);
	dev_info(dev, "pci_epc_map_addr() func_no = %i, vfunc_no = %i, descr_addr = 0x%llX size = 0x%x\n", epf->func_no, epf->vfunc_no, epf_bio->descr_addr, epf_bio->descr_size);
	
	if (ret) {
		dev_err(dev, "Failed to map queue address\n");
		reg->status = STATUS_QUEUE_ADDR_INVALID;
		ret = -EINVAL;
		goto err_queue_addr;
	}

	epf_bio->descr = addr;
	dev_err(dev, "\tQUEUE => Queue Address physical: 0x%llX\t Queue Address virt: 0x%llX\t Queue Addr PCI: 0x%llX\n",
		phys_addr, addr, epf_bio->descr_addr);
	return 0;
err_queue_addr:
	pci_epc_mem_free_addr(epc, phys_addr, addr, epf_bio->descr_size);
	return ret;
}

static struct pci_bio_driver_ring *get_driver_ring(struct pci_epf_bio *bdev)
{
    return (struct pci_bio_driver_ring *)((u64)bdev->descr + bdev->drv_offset);
}

static struct pci_bio_device_ring *get_device_ring(struct pci_epf_bio *bdev)
{
    return (struct pci_bio_device_ring *)((u64)bdev->descr + bdev->dev_offset);
}

static void reset_cmd_handler(struct pci_epf_bio *epf_bio)
{
    queue_delayed_work(kpcibio_workqueue, &epf_bio->cmd_handler,
		       msecs_to_jiffies(1));
}

static void pci_epf_parked(struct pci_epf_bio *epf_bio)
{
    u32 command;
    enum pci_barno bio_reg_bar = epf_bio->bio_reg_bar;
    struct pci_epf *epf = epf_bio->epf;
    struct pci_epf_bio_reg *reg = epf_bio->reg[bio_reg_bar];
    struct device *dev = &epf->dev;
    struct pci_bio_driver_ring *drv_ring;
    struct pci_epf_bio_descr __iomem *descr;
    int i;
    
    command = reg->command;

    if (!command)
	goto reset_handler;

    reg->command = 0;
    reg->status = 0;

    if (command & COMMAND_ADD_DRV_ENTRY) {
	dev_info(dev, "-> Received new driver entry\n");
	drv_ring = get_driver_ring(epf_bio);
	dev_info(dev, "Received new index = %i\n", drv_ring->idx);
    }

    	
    if (command & COMMAND_WRITE_TO_QUEUE) {
	dev_info(dev, "-> WRITING to QUEUE to physical address: 0x%llX with size = 0x%llX\n", reg->queue_addr, reg->queue_size);
	for (i = 0; i < 16; ++i) {
	    descr = &epf_bio->descr[i];
	    descr->addr = 0xa5a5a5a5 + i;
	    descr->s_sector = 0xa5;
	    descr->offset = 0x5a;
	    descr->len = 0x5a;
	    dev_info(dev, "---> written to descr @ addr 0x%llX\n", (u64)descr);
	}
    }
	
    if (command & COMMAND_SET_QUEUE) {
	dev_err(dev, "-> Mapping QUEUE to physical address: 0x%llX with size = 0x%llX\n", reg->queue_addr, reg->queue_size);
	if (epf_bio->descr_addr != 0) {
	    dev_err(dev, "QUEUE is already mapped... 0x%llX\n", epf_bio->descr_addr);
	    goto reset_handler;
	}
	epf_bio->descr_addr = reg->queue_addr;
	epf_bio->descr_size = reg->queue_size;
	epf_bio->drv_offset = reg->drv_offset;
	epf_bio->dev_offset = reg->dev_offset;
	pci_epf_bio_map_queue(epf_bio, reg);
    }

    if (command & COMMAND_START) {
	dev_err(dev, "-> STARTING UP\n");
	if (epf_bio->descr_addr == 0) {
	    dev_err(dev, "QUEUE is not mapped, cannot start anything\n");
	    goto reset_handler;
	}
	epf_bio->state = PCI_BIO_RUNNING;
    }

  reset_handler:
    reset_cmd_handler(epf_bio);
}

struct pci_epf_bio_info {
    struct pci_epf_bio *epf_bio;
    struct page *page;
    size_t page_order;
    struct bio *bio;
    struct pci_epf_bio_descr __iomem *descr;
    int descr_idx;
    void __iomem *addr;
    phys_addr_t phys_addr;

};


void free_epf_bio_info(struct pci_epf_bio_info *info)
{
    __free_pages(info->page, info->page_order);
    /* bio_put() needed?*/
    kfree(info);
}

struct pci_epf_bio_info *alloc_pci_epf_bio_info(struct pci_epf_bio *epf_bio, size_t size, struct pci_epf_bio_descr __iomem *descr, int descr_idx)
{
    struct pci_epf_bio_info *binfo;
    struct bio *bio;
    struct device *dev = &epf_bio->epf->dev;
    struct page *page;
    
    binfo = devm_kzalloc(dev, sizeof(*binfo), GFP_KERNEL);
    if (unlikely(!binfo)) {
	dev_err(dev, "Could not allocate BIO INFO\n");
	return NULL;
    }

    bio = bio_alloc(GFP_KERNEL, 1);
    if (unlikely(!bio)) {
	dev_err(dev, "Could not allocate BIO\n");
	return NULL;
    }

    binfo->page_order = get_order(size);
    page = alloc_pages(GFP_KERNEL, binfo->page_order);
    if (unlikely(!page)) {
	dev_err(dev, "Could not allocate pages for BIO\n");
	return NULL;
    }

    binfo->addr = pci_epc_mem_alloc_addr(epf_bio->epf->epc, &binfo->phys_addr, size);
    if (!binfo->addr) {
	dev_err(dev, "Failed to allocate bio pci addr address\n");
	return NULL;
    }

    dev_info(dev, "%s: pci_epc_mem_alloc_addr() phys_addr = 0x%llX, size = 0x%x, virt_addr = 0x%llX\n", __FUNCTION__, binfo->phys_addr, size, binfo->addr);

    binfo->bio = bio;
    binfo->epf_bio = epf_bio;
    binfo->page = page;
    binfo->descr = descr;
    binfo->descr_idx = descr_idx;
    return binfo;
}

static void pci_epf_bio_transfer_complete(struct bio *bio)
{
    
    struct pci_epf_bio_info *epf_bio_info = bio->bi_private;
    struct device *dev = &epf_bio_info->epf_bio->epf->dev;
    static struct pci_bio_device_ring *dev_ring;
    int dev_idx = epf_bio_info->epf_bio->dev_idx;
    int ret;
    
    if (bio->bi_status == BLK_STS_OK) {
	char *buffer = kmap_atomic(epf_bio_info->page);
	ret = pci_epc_map_addr(epf_bio_info->epf_bio->epf->epc, epf_bio_info->epf_bio->epf->func_no, epf_bio_info->epf_bio->epf->vfunc_no, epf_bio_info->phys_addr, epf_bio_info->descr->addr, epf_bio_info->descr->len);
	
	if (ret) {
		dev_err(dev, "Failed to map buffer address\n");
		return;
	}

	dev_err(dev, "memcpy_toio() dest: 0x%llX, src: 0x%llX, len: 0x%x\n", (u64)epf_bio_info->addr, (u64)buffer, epf_bio_info->descr->len);
	memcpy_toio(epf_bio_info->addr, buffer, epf_bio_info->descr->len);
	dev_ring = get_device_ring(epf_bio_info->epf_bio);
	dev_ring->ring[dev_idx].index = epf_bio_info->descr_idx;
	dev_ring->idx = (dev_idx + 1) % NUM_DESRIPTORS;
	/* pci_epc_unmap_addr(epf_bio_info->epf_bio->epf->epc, epf_bio_info->epf_bio->epf->func_no, epf_bio_info->epf_bio->epf->vfunc_no, epf_bio_info->phys_addr); */
	/* pci_epc_mem_free_addr(epf_bio_info->epf_bio->epf->epc, epf_bio_info->phys_addr, epf_bio_info->addr, epf_bio_info->descr->len); */
	print_hex_dump(KERN_WARNING, "raw: ", DUMP_PREFIX_OFFSET, 16, 16, buffer, 0x50, true);
	/* kunmap(epf_bio_info->page); */
	/* free_epf_bio_info(epf_bio_info); */
    } else {
	dev_err(dev, "BIO error %i\n", bio->bi_status);
    }
	
}

static void pci_epf_bio_cmd_handler(struct work_struct *work)
{
	int count;
	u32 command;
	struct pci_epf_bio *epf_bio = container_of(work, struct pci_epf_bio,
						     cmd_handler.work);
	struct pci_epf *epf = epf_bio->epf;
	struct device *dev = &epf->dev;
	struct pci_epc *epc = epf->epc;
	enum pci_barno bio_reg_bar = epf_bio->bio_reg_bar;
	struct pci_epf_bio_reg *reg = epf_bio->reg[bio_reg_bar];
	struct pci_epf_bio_descr __iomem *descr;
	struct pci_bio_driver_ring *drv_ring;
	int i, ridx, desc_idx, len;
	struct pci_bio_driver_ring_entry *de;
	struct pci_epf_bio_info *bio_info;

	if (epf_bio->state == PCI_BIO_PARKED)
	    return pci_epf_parked(epf_bio);
	
	command = reg->command;

	/* if (!command) */
	/* 	goto reset_handler; */

	drv_ring = get_driver_ring(epf_bio);	

	/* dev_info(dev, "-> reg: magic: 0x%x\n", reg->magic); */
	/* dev_info(dev, "-> reg: command: 0x%x\n", reg->command); */
	/* dev_info(dev, "-> reg: status: 0x%x\n", reg->status); */

	/* reg->command = 0; */
	/* reg->status = 0; */

	if (command & COMMAND_RAISE_MSI_IRQ) {
		count = pci_epc_get_msi(epc, epf->func_no, epf->vfunc_no);
		if (reg->irq_number > count || count <= 0)
			goto reset_handler;
		reg->status = STATUS_IRQ_RAISED;
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				  PCI_EPC_IRQ_MSI, reg->irq_number);
		goto reset_handler;
	}

	if (command & COMMAND_RAISE_MSIX_IRQ) {
		count = pci_epc_get_msix(epc, epf->func_no, epf->vfunc_no);
		if (reg->irq_number > count || count <= 0)
			goto reset_handler;
		reg->status = STATUS_IRQ_RAISED;
		pci_epc_raise_irq(epc, epf->func_no, epf->vfunc_no,
				  PCI_EPC_IRQ_MSIX, reg->irq_number);
		goto reset_handler;
	}

	if (epf_bio->drv_idx != drv_ring->idx) {
	    dev_info(dev, "New entry in driver ring to process. Internal counter = %i, driver counter = %i\n", epf_bio->drv_idx, drv_ring->idx);
	    while (epf_bio->drv_idx != drv_ring->idx) {
		de = &drv_ring->ring[epf_bio->drv_idx];
		descr = &epf_bio->descr[de->index];
		dev_info(dev, "Driver ring Index to process: %i, flags: %i\n", de->index, de->flags);
		dev_info(dev, "DescrInfo: addr: 0x%llX, sector: 0x%llX, len: 0x%x, offset: 0x%x, dir: %s\n", descr->addr, descr->s_sector, descr->len, descr->offset, (descr->opf == WRITE) ? "WRITE" : "READ");
		bio_info = alloc_pci_epf_bio_info(epf_bio, descr->len, descr, de->index);
		if (unlikely(!bio_info)) {
		    goto reset_handler;
		}
		epf_bio->drv_idx = (epf_bio->drv_idx + 1) % NUM_DESRIPTORS;
		
		bio_set_dev(bio_info->bio, epf_bio->real_bd);
		bio_info->bio->bi_iter.bi_sector = descr->s_sector;
		bio_info->bio->bi_opf = REQ_OP_READ;
		bio_info->bio->bi_end_io = pci_epf_bio_transfer_complete;
		bio_info->bio->bi_private = bio_info;
		len = bio_add_page(bio_info->bio, bio_info->page, descr->len, descr->offset);
		dev_info(dev, "Added pages : %i, offset: %i\n", len, descr->offset);
		submit_bio(bio_info->bio);
	    }
	    /* Process all from epf_bio->drv_idx -> drv_ring->idx*/
	    
	}

reset_handler:
	queue_delayed_work(kpcibio_workqueue, &epf_bio->cmd_handler,
			   msecs_to_jiffies(1));
}

static void pci_epf_bio_unbind(struct pci_epf *epf)
{
	struct pci_epf_bio *epf_bio = epf_get_drvdata(epf);
	struct pci_epc *epc = epf->epc;
	struct pci_epf_bar *epf_bar;
	int bar;

	dev_err(&epf->dev, "%s called\n", __FUNCTION__);
	cancel_delayed_work(&epf_bio->cmd_handler);
	pci_epf_bio_clean_dma_chan(epf_bio);
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

static int pci_epf_bio_set_bar(struct pci_epf *epf)
{
	int bar, add;
	int ret;
	struct pci_epf_bar *epf_bar;
	struct pci_epc *epc = epf->epc;
	struct device *dev = &epf->dev;
	struct pci_epf_bio *epf_bio = epf_get_drvdata(epf);
	enum pci_barno bio_reg_bar = epf_bio->bio_reg_bar;
	const struct pci_epc_features *epc_features;

	epc_features = epf_bio->epc_features;

	dev_info(dev, "Setting test BAR%d\n", bio_reg_bar);

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

static int pci_epf_bio_core_init(struct pci_epf *epf)
{
	struct pci_epf_bio *epf_bio = epf_get_drvdata(epf);
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

	ret = pci_epf_bio_set_bar(epf);
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

static int pci_epf_bio_notifier(struct notifier_block *nb, unsigned long val,
				 void *data)
{
	struct pci_epf *epf = container_of(nb, struct pci_epf, nb);
	struct pci_epf_bio *epf_bio = epf_get_drvdata(epf);
	int ret;

	switch (val) {
	case CORE_INIT:
		ret = pci_epf_bio_core_init(epf);
		if (ret)
			return NOTIFY_BAD;
		break;

	case LINK_UP:
		queue_delayed_work(kpcibio_workqueue, &epf_bio->cmd_handler,
				   msecs_to_jiffies(1));
		break;

	default:
		dev_err(&epf->dev, "Invalid EPF test notifier event\n");
		return NOTIFY_BAD;
	}

	return NOTIFY_OK;
}

static int pci_epf_bio_alloc_space(struct pci_epf *epf)
{
	struct pci_epf_bio *epf_bio = epf_get_drvdata(epf);
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
	size_t test_reg_size;

	epc_features = epf_bio->epc_features;

	bio_reg_bar_size = ALIGN(sizeof(struct pci_epf_bio_reg), 128);

	msix_capable = epc_features->msix_capable;
	if (msix_capable) {
		msix_table_size = PCI_MSIX_ENTRY_SIZE * epf->msix_interrupts;
		epf_bio->msix_table_offset = bio_reg_bar_size;
		/* Align to QWORD or 8 Bytes */
		pba_size = ALIGN(DIV_ROUND_UP(epf->msix_interrupts, 8), 8);
	}
	test_reg_size = bio_reg_bar_size + msix_table_size + pba_size;

	if (epc_features->bar_fixed_size[bio_reg_bar]) {
		if (test_reg_size > bar_size[bio_reg_bar])
			return -ENOMEM;
		test_reg_size = bar_size[bio_reg_bar];
	}

	base = pci_epf_alloc_space(epf, test_reg_size, bio_reg_bar,
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

static int pci_epf_bio_bind(struct pci_epf *epf)
{
	int ret;
	struct pci_epf_bio *epf_bio = epf_get_drvdata(epf);
	const struct pci_epc_features *epc_features;
	enum pci_barno bio_reg_bar = BAR_0;
	struct pci_epc *epc = epf->epc;
	bool linkup_notifier = false;
	bool core_init_notifier = false;
	struct pci_epf_bio_reg *breg;

	dev_err(&epf->dev, "%s called\n", __FUNCTION__);
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

	ret = pci_epf_bio_alloc_space(epf);
	if (ret)
		return ret;

	breg = (struct pci_epf_bio_reg *) epf_bio->reg[bio_reg_bar];
	breg->magic = 0x636f6e74;
	breg->num_sectors = get_capacity(bdev_whole(epf_bio->real_bd)->bd_disk);
	strncpy(breg->dev_name, block_dev_name, sizeof(breg->dev_name));
	
	if (!core_init_notifier) {
		ret = pci_epf_bio_core_init(epf);
		if (ret)
			return ret;
	}

	epf_bio->dma_supported = true;


	ret = pci_epf_bio_init_dma_chan(epf_bio);
	if (ret)
		epf_bio->dma_supported = false;

	if (linkup_notifier) {
		epf->nb.notifier_call = pci_epf_bio_notifier;
		pci_epc_register_notifier(epc, &epf->nb);
	} else {
		queue_work(kpcibio_workqueue, &epf_bio->cmd_handler.work);
	}
	device_add_disk(&epf_bio->epf->dev, epf_bio->gd, NULL);

	return 0;
}

static const struct pci_epf_device_id pci_epf_bio_ids[] = {
	{
		.name = "pci_epf_bio",
	},
	{},
};

static void pci_rbd_transfer_complete(struct bio *bio)
{
	struct pci_remote_bd_request *rbd = bio->bi_private;

	struct request *req = blk_mq_rq_from_pdu(rbd);

	if (--rbd->num_bios == 0) {
		/* pci_rbd_unmap_data(rbd); */
		blk_mq_complete_request(req);
	}
	bio_put(bio);
}


static void deferred_bio_work_func(struct work_struct *work)
{
	struct pci_remote_bd_request *rb_req =
		container_of(work, struct pci_remote_bd_request, bio_work);
	struct request *rq = blk_mq_rq_from_pdu(rb_req);
	struct pci_epf_bio *rdev = rb_req->rdev;

	struct bio *bio_src;

	rb_req->num_bios = 0;

	__rq_for_each_bio (bio_src, rq) {
		struct bio *bt = bio_alloc(GFP_KERNEL, 1);
		if (bt) {
			bt = bio_clone_fast(bio_src, GFP_NOIO, &rdev->bset);
			bio_set_dev(bt, rdev->real_bd);
			bt->bi_end_io = pci_rbd_transfer_complete;
			bt->bi_private = rb_req;
			rb_req->num_bios++;
			bio_get(bt);
			submit_bio(bt);
		} else {
			// FIXME
			dev_err(&rdev->epf->dev, "Could not alloc bio\n");
			break;
		}
	}

	return;
}

static enum blk_eh_timer_return pci_epf_bio_timeout_rq(struct request *rq, bool res)
{
	blk_mq_complete_request(rq);
	return BLK_EH_DONE;
}

static void pci_epf_bio_end_rq(struct request *rq)
{
	blk_mq_end_request(rq, BLK_STS_OK);
}

static blk_status_t pci_epf_bio_queue_rq(struct blk_mq_hw_ctx *hctx,
				     const struct blk_mq_queue_data *bd)
{
	struct pci_epf_bio *rdev = hctx->queue->queuedata;
	struct pci_remote_bd_request *rb_req = blk_mq_rq_to_pdu(bd->rq);
	/* int num; */
	
	rb_req->rdev = rdev;
	blk_mq_start_request(bd->rq);
	/* num = pci_rbd_map_data(hctx, bd->rq, rb_req); */

	/* if (unlikely(num < 0)) */
	/* 	return BLK_STS_RESOURCE; */

	INIT_WORK(&rb_req->bio_work, deferred_bio_work_func);
	queue_work(deferred_bio_add_workqueue, &rb_req->bio_work);

	return BLK_STS_OK;
}

static const struct blk_mq_ops pci_rbd_mq_ops = { .queue_rq = pci_epf_bio_queue_rq,
						  .complete = pci_epf_bio_end_rq,
						  .timeout = pci_epf_bio_timeout_rq };


static void pci_epf_release(struct gendisk *disk, fmode_t mode)
{
}

static int pci_epf_open(struct block_device *bdev, fmode_t mode)
{
	int ret = 0;
	struct pci_epf_bio *rdev = bdev->bd_disk->private_data;

	rdev->device_path = block_dev_name;
	rdev->real_bd = blkdev_get_by_path(rdev->device_path,
					   FMODE_READ | FMODE_WRITE, NULL);
	if (IS_ERR(rdev->real_bd)) {
		ret = PTR_ERR(rdev->real_bd);
		if (ret != -ENOTBLK) {
			dev_err(&rdev->epf->dev,
				"failed to open block device %s: (%ld)\n",
				rdev->device_path, PTR_ERR(rdev->real_bd));
		}
		return -ENXIO;
	}
	return 0;
}

static int pci_epf_getgeo(struct block_device *bdev, struct hd_geometry *geo)
{
	geo->heads = 4;
	geo->sectors = 16;
	geo->cylinders =
		get_capacity(bdev->bd_disk) / (geo->heads * geo->sectors);
	return 0;
}

static int pci_epf_ioctl(struct block_device *bdev, fmode_t mode,
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
static int pci_epf_compat_ioctl(struct block_device *bdev, fmode_t mode,
				unsigned int cmd, unsigned long arg)
{
	return pci_epf_ioctl(bdev, mode, cmd, (unsigned long)compat_ptr(arg));
}
#endif

static const struct block_device_operations pci_epf_bio_ops = {
	.open = pci_epf_open,
	.release = pci_epf_release,
	.getgeo = pci_epf_getgeo,
	.owner = THIS_MODULE,
	.ioctl = pci_epf_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = pci_epf_compat_ioctl,
#endif
};

static int setup_block_device(struct pci_epf_bio *rdev)
{
        int err;
    	rdev->device_path = block_dev_name;
	rdev->real_bd = blkdev_get_by_path(rdev->device_path,
					   FMODE_READ | FMODE_WRITE, NULL);
	if (IS_ERR(rdev->real_bd)) {
		err = PTR_ERR(rdev->real_bd);
		if (err != -ENOTBLK) {
			dev_err(&rdev->epf->dev,
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

	major = register_blkdev(0, PCI_REMOTE_BLKDEV_NAME);
	if (major < 0) {
		dev_err(&rdev->epf->dev, "unable to register remote block device\n");
		err = -EBUSY;
		goto out_workqueue;
	}

	if (bioset_init(&rdev->bset, 2, 0, 0)) {
		dev_err(&rdev->epf->dev, "Could not init bioset\n");
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
	rdev->gd->fops = &pci_epf_bio_ops;
	rdev->gd->private_data = rdev;
	rdev->gd->queue->queuedata = rdev;

	snprintf(rdev->gd->disk_name, 32, "pci_rbd");
	set_capacity(rdev->gd,
		     get_capacity(bdev_whole(rdev->real_bd)->bd_disk));
	return 0;
out_tag_set:
	blk_mq_free_tag_set(&rdev->tag_set);
out_workqueue:
	destroy_workqueue(deferred_bio_add_workqueue);
out_free_dev:
	devm_kfree(&rdev->epf->dev, rdev);
	return err;
}

static int pci_epf_bio_probe(struct pci_epf *epf)
{
	struct pci_epf_bio *epf_bio;
	struct device *dev = &epf->dev;
	epf_bio = devm_kzalloc(dev, sizeof(*epf_bio), GFP_KERNEL);
	if (!epf_bio)
		return -ENOMEM;

	epf->header = &test_header;
	epf_bio->epf = epf;
	
	INIT_DELAYED_WORK(&epf_bio->cmd_handler, pci_epf_bio_cmd_handler);

	epf_set_drvdata(epf, epf_bio);
	setup_block_device(epf_bio);
	dev_err(dev, "%s called\n", __FUNCTION__);
	return 0;
}

static struct pci_epf_ops ops = {
	.unbind	= pci_epf_bio_unbind,
	.bind	= pci_epf_bio_bind,
};

static struct pci_epf_driver bio_driver = {
	.driver.name	= "pci_epf_bio",
	.probe		= pci_epf_bio_probe,
	.id_table	= pci_epf_bio_ids,
	.ops		= &ops,
	.owner		= THIS_MODULE,
};

static int __init pci_epf_bio_init(void)
{
	int ret;

	kpcibio_workqueue = alloc_workqueue("kpcibio",
					     WQ_MEM_RECLAIM | WQ_HIGHPRI, 0);
	if (!kpcibio_workqueue) {
		pr_err("Failed to allocate the kpcitest work queue\n");
		return -ENOMEM;
	}

	ret = pci_epf_register_driver(&bio_driver);
	if (ret) {
		destroy_workqueue(kpcibio_workqueue);
		pr_err("Failed to register pci epf test driver --> %d\n", ret);
		return ret;
	}

	return 0;
}
module_init(pci_epf_bio_init);

static void __exit pci_epf_bio_exit(void)
{
	if (kpcibio_workqueue)
		destroy_workqueue(kpcibio_workqueue);
	pci_epf_unregister_driver(&bio_driver);
}
module_exit(pci_epf_bio_exit);

MODULE_DESCRIPTION("PCI BIO Block Driver");
MODULE_AUTHOR("Wadim Mueller <wafgo01@gmx.com>");
MODULE_LICENSE("GPL v2");
