#!/bin/bash

scp arch/arm/boot/zImage root@$1:/root/
scp arch/arm/boot/dts/exynos3250-kitra520.dtb root@$1:/root/exynos3250_artik5.dtb
scp usr/modules.img root@$1:/root/

