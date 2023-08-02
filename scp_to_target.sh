#/bin/sh

IP=${1:-"192.168.0.144"}
echo "IP is ${IP}"
scp arch/arm64/boot/Image  arch/arm64/boot/dts/freescale/continental_hdk11.dtb root@$IP:/home/root/
ssh root@$IP sync
