obj-m += iio_lis3mdl_spi.o

KDIR ?= "/home/luca/code/drone/Buildroot-PREEMPT_RT/buildroot/output/build/linux-af8289b5267c26cfb74b69dbaaedfbe4d4c5690c"
# KDIR ?= "/lib/modules/$(shell uname -r)/build"

all: 
	make -C $(KDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=/opt/arm-buildroot-linux-gnueabihf_sdk-buildroot/bin/arm-buildroot-linux-gnueabihf- modules

clean:
	make -C $(KDIR) M=$(PWD) clean