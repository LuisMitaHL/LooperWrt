# SPDX-License-Identifier: GPL-2.0-only
#
# RTL819x image recipes.

include $(TOPDIR)/rules.mk
include $(INCLUDE_DIR)/image.mk

DTS_DIR := ../dts
# The stock boot code copies the cvimg payload to 0x80a00000 and jumps there,
# so the flashed image links its loader at that address. The decompressed
# kernel ends at 0x8065e2e0, well below the loader.
#
# The initramfs image is linked higher: it is uploaded to RAM and jumped to
# explicitly, and its decompressed image reaches 0x81041b97, which would
# overwrite a loader sitting at 0x80a00000.
LOADER_LOADADDR := 0x80a00000
LOADER_LOADADDR_INITRAMFS := 0x85000000
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
	$(SED) 's/^LEAF(_start)/.section .text.start,"ax",@progbits\nLEAF(_start)\n\tmtc0\tzero, $$12/' \
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
		LOADADDR=$(if $(1),$(1),$(LOADER_LOADADDR)) \
		KERNEL_ENTRY=$$kentry \
		RAMSIZE=$(RAMSIZE) \
		all
	mv "$@.src/lzma.bin" "$@"
	rm -rf $@.src
endef


# The RW407 container: a small loader, the compressed kernel split around the
# fixed rootfs at 0x260000, and the real rootfs.  See mkcontainer.py.
define Build/loader-rw407
	$(MAKE) -C ./loader clean all \
		TOPDIR=$(TOPDIR) CROSS_COMPILE="$(TARGET_CROSS)"
	$(CP) ./loader/loader.bin $(KDIR)/rw407-loader.bin
endef

define Build/rw407-container
	$(call Build/loader-rw407)
	python3 ./mkcontainer.py \
		--loader $(KDIR)/rw407-loader.bin \
		--kernel $(KDIR)/vmlinux \
		--dtb $(KDIR)/image-$(firstword $(DEVICE_DTS)).dtb \
		--rootfs $(KDIR)/root.squashfs \
		--out $@ \
		--load $$(python3 ./kernel-addrs.py --load $(KDIR)/vmlinux.elf) \
		--entry $$(python3 ./kernel-addrs.py --entry $(KDIR)/vmlinux.elf)
endef

# Wrap the loader+kernel payload in the CVIMG container the stock stage-2
# bootcode expects at flash offset 0x30000 ("cr6c": data=firmware, cpu=new).
define Build/cvimg
	$(STAGING_DIR_HOST)/bin/cvimg -i "$@" -o "$@.new" \
		-e $(if $(1),$(1),$(LOADER_LOADADDR)) -b $(CVIMG_BURN_ADDR) -c new -t fw
	mv "$@.new" "$@"
endef

# Realtek stage-2 boot code treats the word at squashfs offset 8 (normally
# the mkfs timestamp) as the rootfs length and verifies a 16-bit big-endian
# checksum over len + 0x282 bytes, so wrap the rootfs before check-size.
define Build/rw407-rootfs
	python3 ./rw407-rootfs.py "$@"
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
  KERNEL := kernel-bin | append-dtb
  KERNEL_INITRAMFS := kernel-bin | append-dtb | lzma | \
	loader-kernel $(LOADER_LOADADDR_INITRAMFS)
  DEVICE_PACKAGES := kmod-rtw88-8822be rtl8822be-firmware wpad-basic-mbedtls
  IMAGES := sysupgrade.bin
  IMAGE/sysupgrade.bin := rw407-container | append-metadata
endef
TARGET_DEVICES += knup_rw407

$(eval $(call BuildImage))
