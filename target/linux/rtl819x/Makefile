# SPDX-License-Identifier: GPL-2.0-only

include $(TOPDIR)/rules.mk

ARCH:=mipsel
BOARD:=rtl819x
BOARDNAME:=Realtek RTL819x-class routers
DEVICE_TYPE:=router
FEATURES:=ramdisk squashfs pci
SUBTARGETS:=rtl8197f

KERNEL_PATCHVER:=6.18

include $(INCLUDE_DIR)/target.mk

# The vendor boot slot is 0x230000, so build the kernel for size.  This has
# to run after target.mk: kernel.mk assigns KERNEL_MAKE_FLAGS with '=' and
# would drop an earlier '+=', and KCFLAGS='...' can only be overridden by
# appending to the same word (a second KCFLAGS= would drop the existing
# -fmacro-prefix-map flags).
KERNEL_MAKE_FLAGS := $(subst KCFLAGS=",KCFLAGS="-Os ,$(KERNEL_MAKE_FLAGS))

DEFAULT_PACKAGES += dropbear mtd netifd uci ubus

$(eval $(call BuildTarget))
