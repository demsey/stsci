NAME=stsci
ifneq ($(KERNELRELEASE),)
# kbuild part of makefile

obj-m		+= ${NAME}.o

EXTRA_CFLAGS    := -DDEBUG

else
# Normal Makefile

KERNEL_STM22 = ~/devel/opensh4stm23/cuberevo-mini2/build/tmp/work/sh4-linux/linux-cuberevo-mini2-2.6.17.14+stm22+0041-r0.0/stblinux-2.6.17.14
KERNEL_STM23 = ~/devel/openE2/cuberevo-mini2/build/tmp/work/sh4-linux/linux-cuberevo-mini2-2.6.23.17+stm23+0123-r0.0/stblinux-2.6.23.17
KERNELDIR := $(KERNEL_STM23)

export ARCH=sh
CROSS_COMPILE_STM22=~/devel/opensh4stm23/cuberevo-mini2/build/tmp/cross/sh4/bin/sh4-linux-
CROSS_COMPILE_STM23=~/devel/openE2/cuberevo-mini2/build/tmp/cross/sh4/bin/sh4-linux-
export CROSS_COMPILE=$(CROSS_COMPILE_STM23)

all:
#	$(MAKE) -C $(KERNELDIR) M=`pwd` $@
#	$(MAKE) -C $(KERNELDIR) M=`pwd` modules
	$(MAKE) -C $(KERNELDIR) M=`pwd`

# Module specific targets
#genbin:
#	echo "X" > 8123_bin.o_shipped
#
endif

clean:
	rm -rf .tmp_versions
	rm -f Module*.symvers built-in.o .built-in.o.cmd
	rm -f .${NAME}*
	rm -f ${NAME}.mod* ${NAME}.o
