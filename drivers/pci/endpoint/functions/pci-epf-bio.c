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
    u32 num_sectors; /* num of sectors*/
    u32 s_offset;  /* offset from the s_sector */
};

struct pci_bio_driver_ring {
    u16 flags;
    u16 idx;
    u16 ring[]; /* queue size*/
};

struct pci_bio_device_ring {
    u16 flags;
    u32 len;
    u16 idx;
    u16 ring[]; /* queue size*/
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
        struct pci_epf_bio_descr *descr;
        size_t  descr_size;
        u32 drv_offset;
        u32 dev_offset;
};

struct pci_epf_bio_reg {
	u32	magic;
	u32	command;
	u32	status;
        u64	queue_addr;  /* start of struct pci_epf_bio_descr*/
        u32     queue_size;  /* number of struct pci_epf_bio_descr */
        u32     drv_offset;
        u32     dev_offset;
	u32	irq_type;
	u32	irq_number;
	u32	flags;
} __packed;

static struct pci_epf_header test_header = {
	.vendorid	= PCI_ANY_ID,
	.deviceid	= PCI_ANY_ID,
	.baseclass_code = PCI_CLASS_OTHERS,
	.interrupt_pin	= PCI_INTERRUPT_INTA,
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
	if (!addr) {
		dev_err(dev, "Failed to allocate queue address\n");
		reg->status = STATUS_QUEUE_ADDR_INVALID;
		return -ENOMEM;
	}

	ret = pci_epc_map_addr(epc, epf->func_no, epf->vfunc_no, phys_addr,
			       reg->queue_addr, epf_bio->descr_size);
	if (ret) {
		dev_err(dev, "Failed to map queue address\n");
		reg->status = STATUS_QUEUE_ADDR_INVALID;
		ret = -EINVAL;
		goto err_queue_addr;
	}

	epf_bio->descr = addr;
	dev_err(dev, "\tQUEUE => Queue Address physical: 0x%llX\t Queue Address virt: 0x%llX\t Queue Addr PCI: 0x%llX\n",
		phys_addr, addr, reg->queue_addr);
	return 0;
err_queue_addr:
	pci_epc_mem_free_addr(epc, phys_addr, addr, epf_bio->descr_size);
	return ret;
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
	struct pci_epf_bio_descr *descr;
	command = reg->command;
	if (!command)
		goto reset_handler;


	dev_err(dev, "-> reg: magic: 0x%x\n", reg->magic);
	dev_err(dev, "-> reg: command: 0x%x\n", reg->command);
	dev_err(dev, "-> reg: status: 0x%x\n", reg->status);

	reg->command = 0;
	reg->status = 0;
	reg->magic = 0xd00dfeed;

	if (reg->irq_type > IRQ_TYPE_MSIX) {
		dev_err(dev, "Failed to detect IRQ type\n");
		goto reset_handler;
	}

	if (command & COMMAND_SET_QUEUE) {
	    dev_err(dev, "-> Mapping QUEUE to physical address: 0x%llX with size = 0x%llX\n", reg->queue_addr, epf_bio->descr_size);
	    pci_epf_bio_map_queue(epf_bio, reg);
	}

	if (command & COMMAND_WRITE_TO_QUEUE) {
	    dev_err(dev, "-> WRITING to QUEUE to physical address: 0x%llX with size = 0x%llX\n", reg->queue_addr, epf_bio->descr_size);
	    descr = epf_bio->descr;
	    descr->num_sectors = 0xa0;
	    descr->addr = 0xa5a5a5a5;
	    descr->s_offset = 0x5a;
	    descr->s_sector = 0xa5;
	}
	
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
	breg->queue_size = NUM_DESRIPTORS;
	breg->dev_offset = epf_bio->dev_offset;
	breg->drv_offset = epf_bio->drv_offset;
	
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

	return 0;
}

static const struct pci_epf_device_id pci_epf_bio_ids[] = {
	{
		.name = "pci_epf_bio",
	},
	{},
};

static int pci_epf_bio_probe(struct pci_epf *epf)
{
	struct pci_epf_bio *epf_bio;
	struct device *dev = &epf->dev;
	epf_bio = devm_kzalloc(dev, sizeof(*epf_bio), GFP_KERNEL);
	if (!epf_bio)
		return -ENOMEM;

	epf_bio->descr_size = ALIGN(NUM_DESRIPTORS * sizeof(*epf_bio->descr), 8) +
				    ALIGN(sizeof(struct pci_bio_driver_ring) + (NUM_DESRIPTORS * sizeof(u16)), 8) +
	                            ALIGN(sizeof(struct pci_bio_device_ring) + (NUM_DESRIPTORS * sizeof(u16)), 8);
	
	epf_bio->drv_offset = ALIGN(NUM_DESRIPTORS * sizeof(*epf_bio->descr), 8);
	epf_bio->dev_offset = epf_bio->drv_offset + ALIGN(sizeof(struct pci_bio_driver_ring) + (NUM_DESRIPTORS * sizeof(u16)), 8);
	epf->header = &test_header;
	epf_bio->epf = epf;

	INIT_DELAYED_WORK(&epf_bio->cmd_handler, pci_epf_bio_cmd_handler);

	epf_set_drvdata(epf, epf_bio);
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
