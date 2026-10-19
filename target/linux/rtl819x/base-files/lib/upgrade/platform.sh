# SPDX-License-Identifier: GPL-2.0-only
#
# Knup KP-RW407/AX fixed flash layout:
#   mtd0 bootloader 0x000000-0x030000  (read-only: bootcode + factory + apmib)
#   mtd1 kernel     0x030000-0x260000  (cvimg container)
#   mtd2 rootfs     0x260000-0x800000
#
# Sysupgrade is intentionally not implemented yet: the image has to be written
# as kernel + rootfs with the cvimg container left at offset 0, and the stock
# stage-2 bootcode has not been verified against an OpenWrt image on hardware.
# Flashing until then is done over the UART boot shell (FLW) after a full
# 8 MiB backup. Never write mtd0.

REQUIRE_IMAGE_METADATA=1

platform_check_image() {
	case "$(board_name)" in
	knup,rw407)
		echo "Knup KP-RW407/AX: sysupgrade is not enabled yet (UART flash only)." >&2
		return 1
		;;
	esac

	echo "Sysupgrade is not supported on $(board_name)." >&2
	return 1
}

platform_do_upgrade() {
	echo "Knup KP-RW407/AX: sysupgrade is not enabled yet (UART flash only)." >&2
	return 1
}
