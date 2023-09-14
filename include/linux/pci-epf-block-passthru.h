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

#define BLOCKPT_MAGIC                   0x636f6e74

#define PBI_EPF_BLOCKPT_F_USED          BIT(1)

#define COMMAND_SET_QUEUE               BIT(6)
#define COMMAND_GET_DEVICES             BIT(7)
#define COMMAND_START                   BIT(8)
#define COMMAND_GET_NUM_SECTORS         BIT(9)

#define BPT_STATUS_SUCCESS	        BIT(0)
#define BPT_STATUS_ERROR	        BIT(8)
#define BPT_STATUS_QUEUE_ADDR_INVALID	BIT(9)


struct pci_epf_blockpt_descr_remote {
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

struct pci_epf_blockpt_descr {
    struct pci_epf_blockpt_descr_remote rdata;
    u64 tag;
};

struct pci_blockpt_driver_ring {
    u16 idx;
    u16 ring[]; /* queue size*/
};

struct pci_blockpt_device_ring {
    u16 idx;
    u16 ring[]; /* queue size*/
};


#endif /* __LINUX_PCI_EPC_H */
