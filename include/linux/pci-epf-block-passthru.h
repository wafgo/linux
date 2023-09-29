/* SPDX-License-Identifier: GPL-2.0 */
/*
 * PCI Endpoint Function for Blockdevice passthrough header 
 *
 * Copyright (C) 2023 Continental Automotive GmbH
 * Author: Wadim Mueller <wadim.mueller@continental.com>
 */

#ifndef __LINUX_PCI_EPF_BLOCKPT_H
#define __LINUX_PCI_EPF_BLOCKPT_H

#include <linux/types.h>

#define MAX_BLOCK_DEVS                  (16UL)

#define BLOCKPT_MAGIC                   0x636f6e74

#define PBI_EPF_BLOCKPT_F_USED          BIT(1)

#define BPT_COMMAND_SET_QUEUE               BIT(6)
#define BPT_COMMAND_GET_DEVICES             BIT(7)
#define BPT_COMMAND_START                   BIT(8)
#define BPT_COMMAND_GET_NUM_SECTORS         BIT(9)
#define BPT_COMMAND_STOP                    BIT(10)
#define BPT_COMMAND_SET_IRQ                 BIT(11)
#define BPT_COMMAND_GET_PERMISSION          BIT(12)

#define BPT_STATUS_SUCCESS	        BIT(0)
#define BPT_STATUS_ERROR	        BIT(8)
#define BPT_STATUS_QUEUE_ADDR_INVALID	BIT(9)

#define BPT_PERMISSION_RO               BIT(0)

struct pci_epf_blockpt_reg {
	u32	magic;
	u32	command;
	u32	status;
        u32     queue_bar_offset;  
        u32     drv_offset;
        u32     dev_offset;
        u32	num_desc;
        u32     max_devs;
        u32     irq;
        u32     qsize;
        u32     num_queues;
        u32     queue_offset;
        u32     available_qsize;
        u8      dev_idx;
        u8      perm;
        u8      qidx;
        u8      bres0;
        u64     num_sectors;
        char    dev_name[64 * MAX_BLOCK_DEVS + 1];
} __packed;

struct pci_epf_blockpt_descr {
    u64 s_sector; /* start sector of the request */
    u64 addr; /* where the data is  */
    u32 len; /* bytes to pu at addr + s_offset*/
    struct blockpt_si {
	u8 opf;
	u8 status;
	u8 flags;
	u8 res0;
    } si;
};

struct pci_blockpt_driver_ring {
    u16 idx;
    u16 ring[]; /* queue size*/
};

struct pci_blockpt_device_ring {
    u16 idx;
    u16 ring[]; /* queue size*/
};


#endif /* __LINUX_PCI_EPF_BLOCKPT_H */
