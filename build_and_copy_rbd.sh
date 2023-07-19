#!/bin/sh


make -j25 ARCH=arm64 LLVM=1 all
cp /home/uia67865/devel/git/linux-s32/drivers/block/pci_remote_bdev_new.ko  /home/uia67865/devel/nfs
