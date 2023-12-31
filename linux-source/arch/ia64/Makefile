#
# ia64/Makefile
#
# This file is included by the global makefile so that you can add your own
# architecture-specific flags and dependencies.
#
# This file is subject to the terms and conditions of the GNU General Public
# License.  See the file "COPYING" in the main directory of this archive
# for more details.
#
# Copyright (C) 1998-2004 by David Mosberger-Tang <davidm@hpl.hp.com>
#

KBUILD_DEFCONFIG := generic_defconfig

NM := $(CROSS_COMPILE)nm -B

CHECKFLAGS	+= -D__ia64=1 -D__ia64__=1 -D_LP64 -D__LP64__

OBJCOPYFLAGS	:= --strip-all
LDFLAGS_vmlinux	:= -static
KBUILD_AFLAGS_KERNEL := -mconstant-gp
EXTRA		:=

cflags-y	:= -pipe $(EXTRA) -ffixed-r13 -mfixed-range=f12-f15,f32-f127 \
		   -falign-functions=32 -frename-registers -fno-optimize-sibling-calls
KBUILD_CFLAGS_KERNEL := -mconstant-gp

KBUILD_CPPFLAGS += -DHAVE_WORKING_TEXT_ALIGN -DHAVE_MODEL_SMALL_ATTRIBUTE -DHAVE_SERIALIZE_DIRECTIVE

quiet_cmd_gzip = GZIP    $@
cmd_gzip = cat $(real-prereqs) | $(KGZIP) -n -f -9 > $@

quiet_cmd_objcopy = OBJCOPY $@
cmd_objcopy = $(OBJCOPY) $(OBJCOPYFLAGS) $(OBJCOPYFLAGS_$(@F)) $< $@

KBUILD_CFLAGS += $(cflags-y)

libs-y				+= arch/ia64/lib/

drivers-y			+= arch/ia64/pci/ arch/ia64/hp/common/

PHONY += compressed check

all: compressed

compressed: vmlinux.gz

vmlinuz: vmlinux.gz

vmlinux.gz: vmlinux.bin FORCE
	$(call if_changed,gzip)

vmlinux.bin: vmlinux FORCE
	$(call if_changed,objcopy)

archheaders:
	$(Q)$(MAKE) $(build)=arch/ia64/kernel/syscalls all

CLEAN_FILES += vmlinux.gz

install: KBUILD_IMAGE := vmlinux.gz
install:
	$(call cmd,install)

define archhelp
  echo '* compressed	- Build compressed kernel image'
  echo '  install	- Install compressed kernel image'
endef
