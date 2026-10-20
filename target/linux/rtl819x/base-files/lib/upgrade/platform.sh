# SPDX-License-Identifier: GPL-2.0-only
#
# Knup KP-RW407/AX single-firmware container.
#
# Flash layout:
#   mtd0 bootloader 0x000000-0x030000  (bootcode + factory/apmib; never write)
#   mtd1 firmware   0x030000-0x800000  (container: loader + kernel + rootfs)
#   mtd2 rootfs0    dynamic           (real squashfs, starts at flash 0x260000)
#   mtd3 rootfs_data dynamic          (JFFS2 after the kernel)
#
# The stock stage-2 still checks the cvimg at 0x30000 and the real rootfs at
# 0x260000.  mtdsplit_rw407 exposes the rootfs and the JFFS2 area.

PART_NAME=firmware
REQUIRE_IMAGE_METADATA=1

read_le32() {
	dd if="$1" bs=1 skip="$2" count=4 2>/dev/null | \
		hexdump -v -n 4 -e '1/4 "%u"' | tr -d ' \n'
}

read_hex4() {
	dd if="$1" bs=1 skip="$2" count=4 2>/dev/null | \
		hexdump -v -n 4 -e '4/1 "%02x"' | tr -d ' \n'
}

platform_check_image() {
	local img="$1"
	local size hdr_off marker

	[ "$(board_name)" = "knup,rw407" ] || {
		echo "Unsupported board: $(board_name)" >&2
		return 1
	}

	[ -f "$img" ] || {
		echo "Image not found: $img" >&2
		return 1
	}

	size=$(wc -c < "$img")
	if [ "$size" -gt $((0x7d0000)) ]; then
		echo "Image too large: $size bytes (firmware region is 0x7d0000)" >&2
		return 1
	fi

	[ "$(read_hex4 "$img" 0)" = "63723663" ] || {
		echo "Not a cvimg container (bad signature)" >&2
		return 1
	}
	[ "$(read_hex4 "$img" 4)" = "85000000" ] || {
		echo "Unexpected cvimg start address" >&2
		return 1
	}
	[ "$(read_hex4 "$img" $((0x10010)))" = "4b4e5752" ] || {
		echo "Missing RW407 container header" >&2
		return 1
	}
	[ "$(read_hex4 "$img" $((0x230000)))" = "68737173" ] || {
		echo "Missing squashfs at the bootloader rootfs offset" >&2
		return 1
	}

	hdr_off=$(read_le32 "$img" $((0x1003c)))
	marker=$((0x10 + hdr_off))
	if [ "$marker" -ge "$size" ] || \
	   [ "$(read_hex4 "$img" "$marker")" != "deadc0de" ]; then
		echo "Missing JFFS2 marker at the container rootfs_data offset" >&2
		return 1
	fi

	return 0
}

platform_do_upgrade() {
	# Erase the whole firmware region: this removes the old loader, kernel,
	# rootfs and overlay.  The stock bootloader/factory region is untouched.
	mtd erase firmware || {
		echo "Failed to erase firmware" >&2
		return 1
	}
	default_do_upgrade "$1"
}
