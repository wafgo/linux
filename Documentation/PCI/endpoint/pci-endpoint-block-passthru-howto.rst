.. SPDX-License-Identifier: GPL-2.0

================================
PCI Block Passthrough User Guide
================================

:Author: Wadim Mueller <wadim.mueller@continental.com>

This document is a guide to help users use pci-epf-block-passthru function driver
and pci-remote-disk host driver for accessing remote block-devices which are exported on the Endpoint from the Host. The list of steps to be followed on the host side and EP side is given below.

Endpoint Device
===============

Endpoint Controller Devices
---------------------------

To find the list of endpoint controller devices in the system::

	# ls /sys/class/pci_epc/
	  44100000.pcie

If PCI_ENDPOINT_CONFIGFS is enabled::

	# ls /sys/kernel/config/pci_ep/controllers
	  44100000.pcie


Endpoint Function Drivers
-------------------------

To find the list of endpoint function drivers in the system::

	# ls /sys/bus/pci-epf/drivers
	  pci_epf_blockpt

If PCI_ENDPOINT_CONFIGFS is enabled::

	# ls /sys/kernel/config/pci_ep/functions
	  pci_epf_blockpt


Creating pci-epf-blockpt Device
-------------------------------

PCI endpoint function device can be created using the configfs. To create
pci-epf-blockpt device, the following commands can be used::

	# mount -t configfs none /sys/kernel/config
	# cd /sys/kernel/config/pci_ep/
	# mkdir functions/pci_epf_blockpt/func1

The "mkdir func1" above creates the pci-epf-blockpt function device that will
be probed by pci_epf_blockpt driver.

The PCI endpoint framework populates the directory with the following
configurable fields::

	# ls functions/pci_epf_blockpt/func1
	  baseclass_code	interrupt_pin	progif_code	subsys_id
	  cache_line_size	msi_interrupts	revid		subsys_vendorid
	  deviceid          	msix_interrupts	subclass_code	vendorid


Configuring pci-epf-blockpt Device
----------------------------------

The user can configure the pci-epf-blockpt device using configfs entry. In order
to change the vendorid the following commands can be used::

	# echo 0x0000 > functions/pci_epf_blockpt/func1/vendorid
	# echo 0xc402 > functions/pci_epf_blockpt/func1/deviceid


Binding pci-epf-blockpt Device to EP Controller
-----------------------------------------------

In order for the endpoint function device to be useful, it has to be bound to
a PCI endpoint controller driver. Use the configfs to bind the function
device to one of the controller driver present in the system::

	# ln -s functions/pci_epf_blockpt/func1 controllers/44100000.pcie/

Once the above step is completed, the PCI endpoint is ready to establish a link
with the host.


Export the Block Devices
------------------------

In order for the Block Passthrough function driver to be useful you first need to export
some of the block devices to the Host. For this a new folder for each exported Block device has
to be created inside of the blockpt folder. The following example shows how the full mmc device can be exported::

	# cd /sys/kernel/config/pci_ep/functions/pci_epf_blockpt/func1
	# mkdir mmc0
	# echo -n /dev/mmcblk0 > mmc0/disc_name

If you also have e.g. an nvme which you want to export you can continue like in the following::

	# mkdir nvme
	# echo -n /dev/nvme0n1 > nvme/disc_name

Start the Link
--------------

In order for the endpoint device to establish a link with the host, the _start_
field should be populated with '1'::

	# echo 1 > controllers/44100000.pcie/start



	
Thats it from the EP side. If you now load the pci-remote-disk driver on the RC side you should already see that /dev/mmcblk0 and /dev/nvme0n1 can be attached


RootComplex Device
==================

lspci Output
------------

Note that the devices listed here correspond to the value populated in 1.4
above::

	0001:00:00.0 PCI bridge: Qualcomm Device 0115
	0001:01:00.0 Unassigned class [ff00]: Device 0000:c402

PCI driver
----------

If the driver was not loaded automatically after `Start the Link`_, you can load it manually by running e.g::

         # insmod pci-remote-disk.ko
           pci-remote-disk 0001:01:00.0: Found /dev/mmcblk0
           pci-remote-disk 0001:01:00.0: Found /dev/nvme0n1
           pci-remote-disk 0001:01:00.0: Found 2 devices

This just shows you which Block devices are exported by the EP. You are not attached to any of them yet. If you e.g. want to attach to the nvme device. Run the following::

         # echo 1 > /sys/kernel/config/pci_remote_disk/nvme0n1/attach 
           pci-remote-disk 0001:01:00.0: nvme0n1: Setting queue addr. #Descriptors 1024 (28688 Bytes)
           pci-remote-disk 0001:01:00.0: /dev/nvme0n1 capacity 0x3a386030

After this the device is attached and can be used. By default the devices are exported by the original names with an **pci-rd-** prepended (this can be changed by using the */sys/kernel/config/pci_remote_disk/<DEVICE>/local_name* node). So in this case the output of 'lsblk' would look like the following::

        # lsblk
	  ...
	  ...
          pci-rd-nvme0n1 259:30   0 465.8G  0 disk 

Thats it, the device should now be usable. You can try to mount it through::

        # mount /dev/pci-rd-nvme0n1 <SOME_DIR> 


