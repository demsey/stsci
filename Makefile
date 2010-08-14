NAME=stsci
ifneq ($(KERNELRELEASE),)
# kbuild part of makefile

obj-m		+= ${NAME}.o

EXTRA_CFLAGS    += -DDEBUG

else
# Normal Makefile

export ARCH=sh

ifneq (,$(findstring STM22,$(EXTRA_CFLAGS)))
STM22=1
endif

ifndef KDIR
ifdef STM22
export KDIR=~/devel/opensh4stm23/cuberevo-mini2/build/tmp/work/sh4-linux/linux-cuberevo-mini2-2.6.17.14+stm22+0041-r0.0/stblinux-2.6.17.14
else
export KDIR=~/devel/openE2/cuberevo-mini2/build/tmp/work/sh4-linux/linux-cuberevo-mini2-2.6.23.17+stm23+0123-r0.0/stblinux-2.6.23.17
endif # STM22
endif # KDIR

ifndef CROSS_COMPILE
ifdef STM22
export CROSS_COMPILE=~/devel/opensh4stm23/cuberevo-mini2/build/tmp/cross/sh4/bin/sh4-linux-
else
export CROSS_COMPILE=~/devel/openE2/cuberevo-mini2/build/tmp/cross/sh4/bin/sh4-linux-
endif # STM22
endif # CROSS_COMPILE

all:
#	$(MAKE) -C $(KDIR) M=`pwd` $@
#	$(MAKE) -C $(KDIR) M=`pwd` modules
	$(MAKE) -C $(KDIR) M=`pwd`

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
