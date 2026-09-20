include $(TOPDIR)/rules.mk

PKG_NAME:=aurora-branding
PKG_VERSION:=1.0
PKG_RELEASE:=2

PKG_LICENSE:=Apache-2.0
PKGARCH:=all

include $(INCLUDE_DIR)/package.mk

define Package/aurora-branding
  SECTION:=luci
  CATEGORY:=LaOtraRed
  TITLE:=LaOtraRed Aurora branding for the footstrap LuCI theme
  DEPENDS:=+luci-theme-footstrap
endef

define Package/aurora-branding/description
  Ships the LaOtraRed Aurora logo (dark/light variants) for the
  footstrap LuCI theme: the header brand mark is swapped at runtime
  through the theme's companion-plugin hook, and the favicons are
  replaced with the Aurora mark. No theme files are modified and no
  file is owned by two packages (apk rejects that): the favicons
  travel as payloads and are copied into place by uci-defaults.
endef

define Build/Compile
endef

define Package/aurora-branding/install
	$(INSTALL_DIR) $(1)/www/luci-static/footstrap/img
	$(INSTALL_DATA) ./files/www/luci-static/footstrap/img/aurora-dark.svg ./files/www/luci-static/footstrap/img/aurora-light.svg $(1)/www/luci-static/footstrap/img/
	$(INSTALL_DIR) $(1)/usr/share/aurora-branding
	$(INSTALL_DATA) ./files/usr/share/aurora-branding/logo.svg ./files/usr/share/aurora-branding/logo_48.png ./files/usr/share/aurora-branding/app-icon-192.png $(1)/usr/share/aurora-branding/
	$(INSTALL_DIR) $(1)/www/luci-static/resources
	$(INSTALL_DATA) ./files/www/luci-static/resources/fs-brand-aurora.js $(1)/www/luci-static/resources/
	$(INSTALL_DIR) $(1)/etc/uci-defaults
	$(INSTALL_BIN) ./files/etc/uci-defaults/50_aurora-branding $(1)/etc/uci-defaults/
	$(INSTALL_DIR) $(1)/lib/upgrade/keep.d
	$(INSTALL_DATA) ./files/lib/upgrade/keep.d/aurora-branding $(1)/lib/upgrade/keep.d/
endef

$(eval $(call BuildPackage,aurora-branding))
