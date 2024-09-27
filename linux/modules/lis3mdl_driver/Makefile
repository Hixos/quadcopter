obj-m += iio_lis3mdl_spi.o

KDIR ?= "/home/luca/code/drone/Buildroot-PREEMPT_RT/buildroot/output/build/linux-a7479ae20b3725616e345e8c6423f5d56d89624b"
# KDIR ?= "/lib/modules/$(shell uname -r)/build"
# CFLAGS_iio_lis3mdl_spi.o := -DDEBUG
all: 
	make -C $(KDIR) M=$(PWD) ARCH=arm CROSS_COMPILE=/opt/armv8-rpi2w-linux-gnueabihf/bin/armv8-rpi2w-linux-gnueabihf- modules

clean:
	make -C $(KDIR) M=$(PWD) clean