include $(TOPDIR)/rules.mk

PKG_NAME:=aurora-branding
PKG_VERSION:=1.0
PKG_RELEASE:=1

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
  replaced with the Aurora mark. No theme files are modified.
endef

define Build/Compile
endef

define Package/aurora-branding/install
	$(INSTALL_DIR) $(1)/www/luci-static/footstrap/img
	$(INSTALL_DATA) ./files/www/luci-static/footstrap/img/aurora-dark.svg ./files/www/luci-static/footstrap/img/aurora-light.svg $(1)/www/luci-static/footstrap/img/
	$(INSTALL_DATA) ./files/www/luci-static/footstrap/logo.svg ./files/www/luci-static/footstrap/logo_48.png ./files/www/luci-static/footstrap/app-icon-192.png $(1)/www/luci-static/footstrap/
	$(INSTALL_DIR) $(1)/www/luci-static/resources
	$(INSTALL_DATA) ./files/www/luci-static/resources/fs-brand-aurora.js $(1)/www/luci-static/resources/
	$(INSTALL_DIR) $(1)/etc/uci-defaults
	$(INSTALL_BIN) ./files/etc/uci-defaults/50_aurora-branding $(1)/etc/uci-defaults/
endef

$(eval $(call BuildPackage,aurora-branding))
