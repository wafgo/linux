.. SPDX-License-Identifier: GPL-2.0

=====================================
PCI Block Device Passthrough Function
=====================================

:Author: Wadim Mueller <wadim.mueller@continental.com>

PCI Block Device Passthrough allows one Linux Device to expose its Block devices to the PCI(e) host. The device can export either the full disk or just certain partitions. Also an export in readonly mode is possible.

This feature is useful if you have a direct connection between two PCI capable SoC's, one running as Root Complex and the other in Endpoint mode, and you want to provide the RC device access to some (or all) Block devices attached to the SoC running is EP mode. This is to a certain extent a similar functionality which NTB exposes over Network, but on the PCI(e) bus utilizing the EPC/EPF Kernel Framework.

The below diagram shows a possible setup with two SoCs, SoC1 working in RC mode, SoC2 in EP mode.
SoC2 can now export the NVMe, eMMC and the SD Card attached to it (full Disks or some Partitions). For this
the *pci-epf-block-passthru* driver (located at **drivers/pci/endpoint/functions/pci-epf-block-passthru.c**)
must be loaded on SoC2. SoC1 requires the PCI Driver *pci-remote-disk* (located at **drivers/block/pci-remote-disk.c**)

After both drivers are loaded SoC2 can configure which devices it wants to expose using ConfigFS.
SoC1 can afterwards configure (also utilizing ConfigFS) on his side which exported devices it wants attach to.
After attaching to it, the device will register a disk on SoC1 which can be accessed as a local disk.


.. code-block:: text


                                                   +-------------+  
                                                   |             |
                                                   |   SD Card   |  
                                                   |             |  
                                                   +------^------+  
                                                          |                                                            
		                                          |
    +---------------------+                +--------------v------------+       +---------+
    |                     |                |                           |       |         |
    |      SoC1 (RC)      |<-------------->|        SoC2 (EP)          |<----->|  eMMC   |
    |  (pci-remote-disk)  |                | (pci-epf-block-passthru)  |       |         |
    |                     |                |                           |       +---------+
    +---------------------+                +--------------^------------+       
                                                          |
                                                          |
                                                   +------v------+  
                                                   |             |
                                                   |    NVMe     |  
                                                   |             |  
                                                   +-------------+
						   
 

Registers
---------

The PCI Block Device Passthrough has the following registers:

1) PCI_BLOCKPT_MAGIC         (offset 0x00)
2) PCI_BLOCKPT_COMMAND       (offset 0x04)
3) PCI_BLOCKPT_STATUS        (offset 0x08)
4) PCI_BLOCKPT_QUEUE_SIZE    (offset 0x0C)
5) PCI_BLOCKPT_DRV_OFFSET    (offset 0x10)
6) PCI_BLOCKPT_DEV_OFFSET    (offset 0x14)
7) PCI_BLOCKPT_NUM_DESC      (offset 0x1C)
8) PCI_BLOCKPT_DEV_IDX       (offset 0x24)
9) PCI_BLOCKPT_QUEUE_ADDR    (offset 0x28)
10) PCI_BLOCKPT_NUM_SECTORS  (offset 0x2C)
11) PCI_BLOCKPT_DEV_NAME     (offset 0x30)

Registers Description
---------------------

* **PCI_BLOCKPT_MAGIC**

This register is used to identify itself at the Host Driver as a BlockPT device. This 32-bit register must contain the value 0x636f6e74. Any other value will be rejected by the host driver.

* **PCI_BLOCKPT_COMMAND**
  
This register will be used by the host driver to setup the EP device to export the desired block device. Any operation the Host will do in the ConfigFS will be translated to corresponding command values in this register.

.. _command bitfield description:

========	================================================================
Bitfield	Description
========	================================================================
Bit 0		unused
Bit 1		unused
Bit 2		unused
Bit 3		unused
Bit 4		unused
Bit 5		unused
Bit 6		**SET_QUEUE**: This tells the Endpoint at which bus address the Queue
                is on the host side. This information is used by the EP to map
		the Descriptor queue.
Bit 7           **GET_DEVICES**: Through this command bit the host requests from their
                EP all the available devices the EP Device want to export. Their
		answer to this request is placed into Register PCI_BLOCKPT_DEV_NAME
		where all exported devices are placed in a ';' seperated list
		of device names
Bit 8           **START**: After configuring the corresponding device, this command
                is used by the driver to attach to the device. On EP side worker
		threads are generated to process the descriptors from the host
		side
Bit 9		**NUM_SECTORS**: Get number of sectors. The host issues this command to get the
                size of the block device in number of 512 Byte sectors

Bit 10          **STOP**: Send to detach from the block device. On reception all
                worker threads are terminated.
========	================================================================

  
* **PCI_BLOCKPT_STATUS**

This register reflects the status of the PCI Block Passthrough device.

========	==============================
Bitfield	Description
========	==============================
Bit 0		success
Bit 1		unused
Bit 2		unused
Bit 3		unused
Bit 4		unused
Bit 5		unused
Bit 6		unused
Bit 7		unused
Bit 8		error
Bit 9		invalid queue address (queue could not be mapped)
========	==============================

* **PCI_BLOCKPT_QUEUE_SIZE**

When the descriptor queue is mapped through the SET_QUEUE command, this
register must contain the number of bytes used by the queue.

* **PCI_BLOCKPT_DRV_OFFSET**

The descriptor queue which is provided by the host and mapped by the EP has
the layout as described in `descriptor queue layout`_ . The Entry in this register contains the **Driver Offset**
value from this diagram.

* **PCI_BLOCKPT_DRV_OFFSET**

The descriptor queue which is provided by the host and mapped by the EP has
the layout as described in `descriptor queue layout`_ . The Entry in this register contains the **Device Offset**
value from this diagram.

* **PCI_BLOCKPT_NUM_DESC**
  
This register contains the number of Descriptors in the Descriptor Queue. The minimum number which must be provided
by the host is 16. Anything below will be rejected by the device

.. _blockpt_selector_idx:

* **PCI_BLOCKPT_DEV_IDX**

This register selects which device from the provided list which was requested with a command from `command bitfield description`_ 
this request for. E.g. if you want to set the queue of the device /dev/mmcblk0 and the list which was delivered with
from the command GET_DEVICES from `command bitfield description`_ is the following "/dev/nvme0n1p1;/dev/mmcblk0", than you
set this register to 1 when issues the SET_QUEUE command. If you configure /dev/nvme0n1p1 than this register should be 0.

* **PCI_BLOCKPT_QUEUE_ADDR**

This Register contains the Queue Address of the Queue as shown in `descriptor queue layout`_ .

* **PCI_BLOCKPT_NUM_SECTORS**

The device puts the number of 512 Byte sectors of the device selected with blockpt_selector_idx_ if the command NUM_SECTORS from
`command bitfield description`_ is send from the host.

* **PCI_BLOCKPT_DEV_NAME**
  
The device puts the names of all devices it wants to export into this register when it receives the GET_DEVICES command from `command bitfield description`_.
This field is currently limited to (64 * 16 + 1) bytes.


Data Transfer
-------------

The Data Transfer from the EP to the Host is using a fixed sized Descriptor Queue. This approach is inspired by the VirtIO Specification.

A Descriptor Queue is allocated on the Host side with a fixed size. The Descriptor Queue has a Layout as depicted in `descriptor queue layout`_.
When the host wants to access data from the EP Disk, it first looks for a free descriptor in the Descriptor Ring. When one is found it
sets up the Fields in this descriptor as shown in `descriptor layout`_, with the following description:

 * **s_sector** containing the start sector from which the host wants to read from or write to
 * **len** containing the number of bytes it wants to transfer
 * **addr** field containing the bus address it wants the data transferred to or from (if you have an IOMMU on your SoC1 than this will be an IOVA, without an IOMMU it will usually be a PA).
 * **opf** field tells about the operation (READ or WRITE),
 * **status** field is written to by the EP to tell whether the transfer was successful or not.

After those field are filled in by the Host driver it puts this descriptor index into the driver ring with the layout shown in `driver entry layout`_, and increments
the **idx** field (using modulo NUM_DESCRIPTORS to implement the ring buffer functionality). When the EP detects that the **idx** field in the driver entry has changed
it will pick up this descriptor, setup a Block-IO Request and submit it to the Block-IO layer. After the Block-IO layer has processed this request the Descriptor index will be transferred into
the **Device Ring** as depicted in `device entry layout`_ and the **idx** field incremented there.  From there, the Host driver will know that the Request has been finished and will
deliver it to whoever did the request on the Host side before it will free this descriptor for new transfers.




.. _descriptor layout:

Descriptor Layout
-----------------------
.. code-block:: text

		         	+------------------------+
                         	|        s_sector        |
				|                        | 
                         	+------------------------+
                         	|          addr          |
				|                        |
                         	+------------------------+
                         	|          len           |
                         	+------------------------+
                         	| opf | stat|flags | res |
                         	+------------------------+


.. _driver entry layout: 		

Driver Entry Layout
-----------------------
.. code-block:: text

		         	+------------------------+
                         	|          idx           |
                         	+------------------------+
                         	|     descriptor idx 0   |
                         	+------------------------+
                         	|     descriptor idx 1   |
                         	+------------------------+
                         	|            :           |
                         	+------------------------+
                         	|            :           |
                         	+------------------------+
                         	|descriptor idx NUM_DESC |
                         	+------------------------+


.. _device entry layout:				

Device Entry Layout
-----------------------
.. code-block:: text

		         	+------------------------+
                         	|          idx           |
                         	+------------------------+
                         	|     descriptor idx 0   |
                         	+------------------------+
                         	|     descriptor idx 1   |
                         	+------------------------+
                         	|            :           |
                         	+------------------------+
                         	|            :           |
                         	+------------------------+
                         	|descriptor idx NUM_DESC |
                         	+------------------------+
				
		
.. _descriptor queue layout:

Descriptor Queue Layout
-----------------------

.. code-block:: text

     Queue Address ----->	+------------------------+
                         	|      1. Descriptor     |
                         	+------------------------+
                         	|      2. Descriptor     |
                         	+------------------------+
                         	|            :           |
                         	+------------------------+
                         	|            :           |
                         	+------------------------+
                         	|     Last Descriptor    |
                         	+------------------------+
     Driver Offset ----->       +------------------------+
                         	|     Driver Ring        |
     			        |           :            |
                         	|           :            |
                         	+------------------------+
     Device Offset ----->       +------------------------+
                         	|     Driver Ring        |
     			        |           :            |
                         	|           :            |
                         	+------------------------+
     

			

