# SPDX-License-Identifier: GPL-2.0-only
#
# RTL819x image recipes.

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

DTS_DIR := ../dts
# The boot code copies the cvimg payload to 0x80a00000 and jumps there, so the
# loader must be linked for that address. The kernel load base and entry are
# read from the built ELF at image time (see Build/loader-kernel).
LOADER_LOADADDR := 0x80a00000
CVIMG_BURN_ADDR := 0x30000
RAMSIZE := 0x08000000
# The generic lzma-loader decompresses to KERNEL_ENTRY and jumps to KERNEL_ENTRY,
# which assumes the kernel's entry symbol sits at its load base. On MIPS,
# kernel_entry lives in .ref.text and the linker places it inside .text, so the
# two differ (_text=0x80000000, kernel_entry=0x8066xxxx). Read both from the
# built ELF and decompress to the load base while jumping to the real entry.
define Build/loader-kernel
	rm -rf $@.src
	mkdir -p $@.src
	$(CP) ../../generic/image/lzma-loader/src/* $@.src/
	cp "$@" "$@.src/vmlinux.lzma"
	$(SED) 's/unsigned char \*\*buffer/const unsigned char **buffer/' \
		-e 's/unsigned char \*buffer;/const unsigned char *buffer;/' \
		$@.src/decompress.c
	$(SED) 's|(unsigned char\*)KERNEL_ENTRY|(unsigned char*)KERNEL_LOAD|' \
		$@.src/decompress.c
	# Progress markers so a stop after "Jump to image start" is diagnosable.
	cp ./loader-dbg.h $@.src/loader-dbg.h
	$(SED) 's|#include "LzmaDecode.h"|#include "LzmaDecode.h"\n#include "loader-dbg.h"|' \
		$@.src/decompress.c
	$(SED) 's|\tdata = lzma_start;|\tdbg_report_uart_divisor();\n\tdbg_dump_uart();\n\tdbg_puts("loader: decompressing kernel\\n");\n\tdata = lzma_start;|' \
		$@.src/decompress.c
	$(SED) 's|\t\tblast_dcache(dcache_size, dcache_lsize);|\t\tdbg_puts("loader: done, jumping to kernel\\n");\n\t\tblast_dcache(dcache_size, dcache_lsize);|' \
		$@.src/decompress.c
	# The boot code jumps to the first byte of the image, so _start must be at
	# offset 0. The generic linker script puts *(.text.entry) first, and with
	# -ffunction-sections that section holds decompress.c's entry() -- jumping
	# there would skip start.S's cache setup and pass it garbage arguments. Park
	# _start in its own section and place that section first instead.
	$(SED) 's/^LEAF(_start)/.section .text.start,"ax",@progbits\nLEAF(_start)/' \
		$@.src/start.S
	$(SED) 's/*(.text.entry)/KEEP(*(.text.start))\n    *(.text.entry)/' \
		$@.src/lzma.lds.in
	set -e; \
	kload=`python3 ./kernel-addrs.py --load $(KDIR)/vmlinux.elf`; \
	kentry=`python3 ./kernel-addrs.py --entry $(KDIR)/vmlinux.elf`; \
	echo "kernel load=$$kload entry=$$kentry loader=$(LOADER_LOADADDR)"; \
	echo "CFLAGS += -DKERNEL_LOAD=$$kload" >> $@.src/Makefile; \
	$(MAKE) -C $@.src \
		CC="$(TARGET_CC) -EL" \
		LD="$(TARGET_CROSS)ld -EL" \
		OBJDUMP="$(TARGET_CROSS)objdump -EL" \
		O_FORMAT=elf32-tradlittlemips \
		CROSS_COMPILE="$(TARGET_CROSS)" \
		LOADADDR=$(LOADER_LOADADDR) \
		KERNEL_ENTRY=$$kentry \
		RAMSIZE=$(RAMSIZE) \
		all
	mv "$@.src/lzma.bin" "$@"
	rm -rf $@.src
endef

# Wrap the loader+kernel payload in the CVIMG container the stock stage-2
# bootcode expects at flash offset 0x30000 ("cr6c": data=firmware, cpu=new).
define Build/cvimg
	$(STAGING_DIR_HOST)/bin/cvimg -i "$@" -o "$@.new" \
		-e $(LOADER_LOADADDR) -b $(CVIMG_BURN_ADDR) -c new -t fw
	mv "$@.new" "$@"
endef

define Device/Default
  PROFILES := Default
  KERNEL_DEPENDS = $$(wildcard $$(DTS_DIR)/$$(DEVICE_DTS).dts)
endef

define Device/knup_rw407
  DEVICE_VENDOR := Knup
  DEVICE_MODEL := KP-RW407/AX
  DEVICE_DTS := rtl8197f-knup-rw407
  IMAGE_SIZE := 8000k
  KERNEL_SIZE := 2240k
  KERNEL := kernel-bin | append-dtb | lzma | loader-kernel | cvimg
  KERNEL_INITRAMFS := kernel-bin | append-dtb | lzma | loader-kernel
  DEVICE_PACKAGES := kmod-rtw88-8822be rtl8822be-firmware wpad-basic-mbedtls
  IMAGES := sysupgrade.bin
  IMAGE/sysupgrade.bin := append-kernel | pad-to $$(KERNEL_SIZE) | \
	append-rootfs | pad-rootfs | check-size | append-metadata
endef
TARGET_DEVICES += knup_rw407

$(eval $(call BuildImage))
