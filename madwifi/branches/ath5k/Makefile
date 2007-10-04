KDIR=/lib/modules/$(shell uname -r)/build
KBUILD=$(MAKE) -C $(KDIR) M=$(PWD)

EXTRA_CFLAGS += -DCONFIG_ATHEROS_AR5K_AR5210 -DCONFIG_ATHEROS_AR5K_AR5211 \
		-DCONFIG_ATHEROS_AR5K_AR5212

ath5k-objs	:= ath5k_base.o ath5k_hw.o ath5k_regdom.o ath5k_phy.o \
		   ath5k_inivals.o

obj-m		+= ath5k.o

all:
	$(KBUILD) modules

clean:
	$(KBUILD) clean

install:
	$(KBUILD) modules_install

cscope:
	cscope -b `find $(src) -type f -name '*.[ch]'`

.PHONY: all clean cscope
