#
# Copyright (C) 2026 OpenWrt.org
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#
# cvimg wraps a loadable payload in the Realtek CVIMG container that the
# stock RTL819x stage-2 bootcode expects ("cr6c" = data=firmware, cpu=new).
# Source: Weijie Gao's tool as carried by the out-of-tree RTL8197F ports.
#
include $(TOPDIR)/rules.mk

PKG_NAME:=cvimg
PKG_VERSION:=1.0

include $(INCLUDE_DIR)/host-build.mk

define Host/Prepare
	mkdir -p $(HOST_BUILD_DIR)
endef

define Host/Compile
	$(HOSTCC) $(HOST_CFLAGS) -o $(HOST_BUILD_DIR)/cvimg $(CURDIR)/src/cvimg.c
endef

define Host/Install
	$(INSTALL_BIN) $(HOST_BUILD_DIR)/cvimg $(STAGING_DIR_HOST)/bin/cvimg
endef

define Host/Clean
	rm -f $(STAGING_DIR_HOST)/bin/cvimg
	rm -rf $(HOST_BUILD_DIR)
endef

$(eval $(call HostBuild))
