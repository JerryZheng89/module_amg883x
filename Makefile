obj-m:= amg883x.o

PWD  := $(shell pwd)

KERNEL_SRC ?= /opt/jerryzheng/yocto_linux/build-condor-adas/tmp/work/condor-poky-linux/linux-renesas/5.4.72+gitAUTOINC+902a351a25-r1/linux-condor-standard-build/

default:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

check-syntax:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(PWD) modules clean

