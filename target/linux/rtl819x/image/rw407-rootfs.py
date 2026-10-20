#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
#
# Make the sysupgrade image acceptable to the stock Realtek stage-2 boot code.
#
# The boot code checks the rootfs like this:
#   - first word must be "hsqs"
#   - length is the big-endian 32-bit word at offset 8
#   - a 16-bit big-endian sum over (length + 0x282) bytes must be zero
#
# A plain OpenWrt squashfs has the mkfs timestamp at offset 8, so the boot
# code reads a huge length and never finishes. Reuse that field for the length
# (mkfs time is cosmetic), pad the image so the two-byte checksum starts on a
# 4 KiB sector boundary -- as the vendor images do -- and append it.

import struct
import sys

KERNEL_SIZE = 0x230000          # sysupgrade kernel slot (2240k)
TRAILER = 0x280                 # zero bytes between squashfs end and checksum
SECTOR = 0x1000
ROOTFS_MAX = 0x5A0000           # 0x260000 .. 0x800000


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: rw407-rootfs.py <sysupgrade image>")

    path = sys.argv[1]
    with open(path, "rb") as f:
        data = f.read()
    if len(data) < KERNEL_SIZE:
        sys.exit("image is shorter than the kernel slot")

    rootfs = data[KERNEL_SIZE:]
    if not rootfs.startswith(b"hsqs"):
        sys.exit("rootfs does not start with the squashfs magic")

    size = len(rootfs)
    length = ((size + TRAILER + SECTOR - 1) // SECTOR) * SECTOR - TRAILER
    if length + TRAILER + 2 > ROOTFS_MAX:
        sys.exit("rootfs too large: %#x bytes" % (length + TRAILER + 2))

    out = bytearray(length + TRAILER + 2)
    out[:size] = rootfs
    struct.pack_into(">I", out, 8, length)

    total = 0
    for i in range(0, length + TRAILER, 2):
        total += (out[i] << 8) | out[i + 1]
    checksum = (-total) & 0xFFFF
    struct.pack_into(">H", out, length + TRAILER, checksum)

    with open(path, "wb") as f:
        f.write(data[:KERNEL_SIZE])
        f.write(out)

    print("rw407-rootfs: squashfs=%#x len=%#x checksum=%04x" % (size, length, checksum))


if __name__ == "__main__":
    main()
