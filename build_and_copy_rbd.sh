#!/bin/sh


make -j25 ARCH=arm64 LLVM=1 all
#cp /home/uia67865/devel/git/linux-s32/drivers/block/pci_remote_bdev.ko  /home/uia67865/devel/nfs
cp arch/arm64/boot/Image /home/uia67865/devel/nfs/flash-images/
cp arch/arm64/boot/Image /home/uia67865/devel/tftp/
cp arch/arm64/boot/dts/freescale/continental_hdk11.dtb /home/uia67865/devel/tftp/
