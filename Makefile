# Makefile for the snd-marc2 ALSA driver

# Define KDIR, but allow it to be overridden from the command line.
# The `?=` operator means "assign if not already set".
KDIR ?= /lib/modules/$(shell uname -r)/build

# Our target object
obj-m := snd-marc2.o

# The source file for our object
snd-marc2-objs := marc2_driver.o

# Standard kernel module build rules
all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

install: all
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install

.PHONY: all clean install
