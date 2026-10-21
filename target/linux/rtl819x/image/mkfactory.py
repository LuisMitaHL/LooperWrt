#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Build the stock-web-UI factory image for the Knup RW407.

The stock Realtek SDK web uploader (/boafrm/formUpload) does not accept a
plain OpenWrt image.  It walks a sequence of 16-byte sections:

    magic[4] field4[4] dst[4] len[4] payload[len]

and writes:
  * cr6c/cs6c : the whole section (header + payload) to the
                "boot+cfg+linux" mtdblock at "dst" (big-endian);
  * r6cr      : the payload only to the "rootfs" mtdblock at offset 0.

This tool emits that container for the RW407 stock layout:

  kernel slot   flash 0x030000 .. 0x260000  (0x230000 bytes)
  rootfs        flash 0x260000 .. 0x800000  (0x5a0000 bytes)
  rootfs_data   flash 0x650000 (deadc0de marker)

The kernel section is a short cvimg: the normal RW407 loader, its KNWR
header, and kernel fragment A.  The KNWR header still describes the full
firmware layout so the OpenWrt mtdsplit parser can find rootfs0/rootfs_data.

The outer section format is the generic Realtek SDK update format, so the
same wrapper can be reused on other RTL8197F boards with a different kernel
slot / rootfs offset.  Those are command-line parameters.
"""

from __future__ import annotations

import argparse
import lzma
import struct
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mkcontainer import (  # noqa: E402
    CVIMG_HEADER,
    HDR,
    HEADER_OFF,
    HEADER_SIZE,
    align,
    squashfs_bytes_used,
    sum16_be,
    wrap_rootfs,
)

LOADER_BASE = 0x85000000
KERNEL_BURN = 0x30000
KERNEL_SLOT = 0x230000
ROOTFS_FLASH = 0x260000
ROOTFS_DATA_FLASH = 0x650000
FLASH_END = 0x800000
SECTOR = 0x1000
MAX_UPLOAD = 0x800000
DEADC0DE = b"\xde\xad\xc0\xde"


def body_offset(flash_addr: int) -> int:
    return flash_addr - KERNEL_BURN - CVIMG_HEADER


def build_kernel_cvimg(loader: bytes, kernel: bytes, dtb: bytes,
                       loader_base: int, kernel_load: int, entry: int,
                       rootfs_used: int) -> bytes:
    raw = kernel + dtb
    lz = lzma.compress(raw, format=lzma.FORMAT_ALONE)

    kernel_off = align(len(loader), SECTOR)
    body_need = kernel_off + len(lz)
    if kernel_off < HEADER_OFF + HEADER_SIZE:
        sys.exit("loader is shorter than its KNWR header slot")
    if CVIMG_HEADER + body_need + 2 > KERNEL_SLOT:
        sys.exit(
            "kernel cvimg %#x does not fit the %#x kernel slot"
            % (CVIMG_HEADER + body_need + 2, KERNEL_SLOT)
        )
    if loader[HEADER_OFF:HEADER_OFF + 4] != b"\0\0\0\0":
        sys.exit("loader KNWR header slot is not empty")

    body = bytearray(b"\xff" * body_need)
    body[:len(loader)] = loader
    body[kernel_off:kernel_off + len(lz)] = lz
    if len(body) & 1:
        body += b"\xff"

    # Logical layout of the full firmware region, used by mtdsplit_rw407.
    full_body_len = body_offset(ROOTFS_DATA_FLASH)
    header = HDR.pack(
        0x52574E4B,              # KNWR
        1,
        HEADER_SIZE,
        kernel_off,
        len(lz),
        0,
        0,
        kernel_load,
        entry,
        body_offset(ROOTFS_FLASH),
        rootfs_used,
        full_body_len,
        full_body_len,
        0, 0, 0,
    )
    body[HEADER_OFF:HEADER_OFF + len(header)] = header

    csum = (-sum16_be(body)) & 0xFFFF
    body = bytes(body) + struct.pack(">H", csum)
    if sum16_be(body) != 0:
        sys.exit("internal: kernel cvimg checksum")

    return b"cr6c" + struct.pack(">III", loader_base, KERNEL_BURN, len(body)) + body


def build_rootfs_payload(rootfs: bytes) -> bytes:
    part_size = FLASH_END - ROOTFS_FLASH
    if len(rootfs) > part_size:
        sys.exit("rootfs too large: %#x > %#x" % (len(rootfs), part_size))

    payload = bytearray(b"\xff" * part_size)
    payload[:len(rootfs)] = rootfs
    marker = ROOTFS_DATA_FLASH - ROOTFS_FLASH
    payload[marker:marker + 4] = DEADC0DE
    payload[-2:] = b"\0\0"
    csum = (-sum16_be(payload)) & 0xFFFF
    payload[-2:] = struct.pack(">H", csum)
    if sum16_be(payload) != 0:
        sys.exit("internal: rootfs section checksum")
    return bytes(payload)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--loader", type=Path, required=True)
    ap.add_argument("--kernel", type=Path, required=True, help="raw vmlinux")
    ap.add_argument("--dtb", type=Path)
    ap.add_argument("--rootfs", type=Path, required=True, help="root.squashfs")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--loader-base", type=lambda s: int(s, 0), default=LOADER_BASE)
    ap.add_argument("--kernel-load", type=lambda s: int(s, 0), default=0x80000000)
    ap.add_argument("--entry", type=lambda s: int(s, 0), required=True)
    args = ap.parse_args()

    loader = args.loader.read_bytes()
    rootfs_used, rootfs_raw = squashfs_bytes_used(args.rootfs)
    rootfs = wrap_rootfs(rootfs_raw)
    dtb = args.dtb.read_bytes() if args.dtb else b""

    kernel_cvimg = build_kernel_cvimg(
        loader, args.kernel.read_bytes(), dtb, args.loader_base,
        args.kernel_load, args.entry, rootfs_used
    )
    rootfs_payload = build_rootfs_payload(rootfs)
    rootfs_section = b"r6cr" + struct.pack(">III", 0, 0, len(rootfs_payload)) + rootfs_payload

    image = kernel_cvimg + rootfs_section
    if len(kernel_cvimg) > KERNEL_SLOT:
        sys.exit("kernel section does not fit the kernel slot")
    if len(image) > MAX_UPLOAD:
        sys.exit("factory image too large for stock upload: %#x" % len(image))

    args.out.write_bytes(image)
    print("kernel cvimg=%#x (slot %#x)" % (len(kernel_cvimg), KERNEL_SLOT))
    print("rootfs used=%#x wrapped=%#x section=%#x" %
          (rootfs_used, len(rootfs), len(rootfs_payload)))
    print("factory=%#x (upload limit %#x)" % (len(image), MAX_UPLOAD))
    return 0


if __name__ == "__main__":
    sys.exit(main())
