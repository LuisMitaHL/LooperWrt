# SPDX-License-Identifier: GPL-2.0-only

ARCH:=mipsel
SUBTARGET:=rtl8197f
CPU_TYPE:=24kc
BOARDNAME:=Realtek RTL8197F-class SoCs

FEATURES := $(filter-out mips16,$(FEATURES))

CPU_CFLAGS_mips32r2 := -mips32r2 -mtune=24kc

define Target/Description
	Build firmware for Realtek RTL8197F-class router SoCs.
endef
