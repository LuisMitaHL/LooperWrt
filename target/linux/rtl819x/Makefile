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

DEFAULT_PACKAGES += dropbear mtd netifd uci ubus

$(eval $(call BuildTarget))
