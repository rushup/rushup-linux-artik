# Linux Kernel for KITRA710C and KitraGTI
## Contents
1. [Introduction](#1-introduction)
2. [Build guide](#2-build-guide)
3. [Update guide](#3-update-guide)

## 1. Introduction
Fork from 'linux-artik' repository adding KITRA710C and KitraGTI support.

## 2. Build guide
### 2.1 Install cross compiler

You'll need an arm64 cross compiler
```
sudo apt-get install gcc-aarch64-linux-gnu
```
If you can't install the above toolchain, you can use linaro toolchain.
```
wget https://releases.linaro.org/components/toolchain/binaries/5.4-2017.05/aarch64-linux-gnu/gcc-linaro-5.4.1-2017.05-x86_64_aarch64-linux-gnu.tar.xz
tar xf gcc-linaro-5.4.1-2017.05-x86_64_aarch64-linux-gnu.tar.xz
export PATH=~/gcc-linaro-5.4.1-2017.05-x86_64_aarch64-linux-gnu/bin:$PATH
```

You can the path permernently through adding it into ~/.bashrc

### 2.2 Install android-fs-tools
To generate modules.img which contains kernel modules, you can use the make_ext4fs.
```
sudo apt-get install android-tools-fsutils
```

### 2.2 Build the kernel

```
make ARCH=arm64 kitra710C_defconfig
or
make ARCH=arm64 kitragti_defconfig
```
If you want to change kernel configurations,
```
make ARCH=arm64 menuconfig
```
Run:

```
./mk_kernel.sh
./mk_dtb.sh
./mk_modules.sh
```

## 3. Update Guide

```
mount -o remount,rw /boot
cp /root/Image /boot
cp /root/*.dtb /boot
dd if=/root/modules.img of=/dev/mmcblk0p5
sync
reboot
```

