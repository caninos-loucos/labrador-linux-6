CURDIR=$(shell pwd)
CPUS=$$(($(shell cat /sys/devices/system/cpu/present | awk -F- '{ print $$2 }')+1))
KERNEL=$(CURDIR)/linux-source-6.1
BUILD=$(CURDIR)/build
BUILD32=$(CURDIR)/build32
#
COMPILER=aarch64-linux-gnu-
COMPILER32=arm-linux-gnueabihf-
#
SCRIPTS_DIR=$(CURDIR)/scripts
#
# version 0.1
#
RELEASE_VER=v01
RELEASE_DIR=$(CURDIR)/release
RELEASE_NAME=labrador-kernel6

.PHONY: all32 config32 menuconfig32 dtbs32 image32 kernel32 clean32 install32

all32: clean32 config32 kernel32

config32:
	$(Q)mkdir $(BUILD32)
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD32) CROSS_COMPILE=$(COMPILER32) ARCH=arm caninos5_defconfig

menuconfig32:
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD32) CROSS_COMPILE=$(COMPILER32) ARCH=arm menuconfig
	
dtbs32:
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD32) CROSS_COMPILE=$(COMPILER32) ARCH=arm dtbs
	
image32:
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD32) CROSS_COMPILE=$(COMPILER32) ARCH=arm -j$(CPUS) uImage modules LOADADDR=0x00008000
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD32) CROSS_COMPILE=$(COMPILER32) ARCH=arm -j$(CPUS) INSTALL_MOD_PATH=$(BUILD32) modules_install
	
kernel32: dtbs32 image32
	$(Q)$(SCRIPTS_DIR)/pack.sh $(BUILD32) $(RELEASE_DIR) armhf $(RELEASE_NAME) $(RELEASE_VER)
	
install32:
	$(Q)$(SCRIPTS_DIR)/install32.sh $(RELEASE_DIR) $(RELEASE_NAME) $(RELEASE_VER)
	
clean32:
	$(Q)rm -rf $(BUILD32)

.PHONY: all config menuconfig dtbs image kernel clean install

all: clean config kernel

config:
	$(Q)mkdir $(BUILD)
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD) CROSS_COMPILE=$(COMPILER) ARCH=arm64 caninos7_defconfig
	
menuconfig:
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD) CROSS_COMPILE=$(COMPILER) ARCH=arm64 menuconfig
	
dtbs:
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD) CROSS_COMPILE=$(COMPILER) ARCH=arm64 dtbs
	
image:
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD) CROSS_COMPILE=$(COMPILER) ARCH=arm64 -j$(CPUS) Image modules
	$(Q)$(MAKE) -C $(KERNEL) O=$(BUILD) CROSS_COMPILE=$(COMPILER) ARCH=arm64 -j$(CPUS) INSTALL_MOD_PATH=$(BUILD) modules_install
	
kernel: dtbs image
	$(Q)$(SCRIPTS_DIR)/pack.sh $(BUILD) $(RELEASE_DIR) aarch64 $(RELEASE_NAME) $(RELEASE_VER)
	
install:
	$(Q)$(SCRIPTS_DIR)/install.sh $(RELEASE_DIR) $(RELEASE_NAME) $(RELEASE_VER)
	
clean:
	$(Q)rm -rf $(BUILD)
	
