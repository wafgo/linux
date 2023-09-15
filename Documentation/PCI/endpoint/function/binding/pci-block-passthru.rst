.. SPDX-License-Identifier: GPL-2.0

==========================
PCI Test Endpoint Function
==========================

name: Should be "pci_epf_blockpt" to bind to the pci_epf_blockpt driver.

Configurable Fields:

================   ===========================================================
vendorid	   should be 0x0000
deviceid	   should be 0xc402 for S32CC
revid		   don't care
progif_code	   don't care
subclass_code	   don't care
baseclass_code	   should be 0xff
cache_line_size	   don't care
subsys_vendor_id   don't care
subsys_id	   don't care
interrupt_pin	   don't care
msi_interrupts	   don't care
msix_interrupts	   don't care
================   ===========================================================
