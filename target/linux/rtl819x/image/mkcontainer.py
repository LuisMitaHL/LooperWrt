#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Build the RW407 single-firmware container.

Layout (firmware-partition relative, partition starts at flash 0x30000):

  0x00000  cvimg header (16 bytes)
  0x00010  loader body (code + header placeholder + checksum)
  ...      kernel fragment A (compressed; up to the rootfs at 0x230000)
  0x230000 real squashfs + Realtek length/checksum trailer
  ...      kernel fragment B (only if A overflowed)
  ...      JFFS2 EOF marker -> rootfs_data

The stock stage-2 checks the cvimg at 0x30000 and the rootfs at 0x260000,
copies the cvimg body to 0x85000000, and jumps.  The loader reads fragment A/B
from that RAM copy and decompresses the kernel.  The kernel mtdsplit parser
reads the same header from flash.
"""

from __future__ import annotations

import argparse
import lzma
import struct
import sys
from pathlib import Path

MAGIC = b"KNWR"
VERSION = 1
HEADER_OFF = 0x10000
HEADER_SIZE = 0x40
CVIMG_HEADER = 16
LOADER_BASE = 0x85000000
ROOTFS_FLASH_OFF = 0x230000          # firmware-partition relative
ROOTFS_BODY_OFF = ROOTFS_FLASH_OFF - CVIMG_HEADER
ROOTFS_MAX = 0x5A0000
IMAGE_MAX = 0x7D0000
TRAILER = 0x280
SECTOR = 0x1000
BLOCK = 0x10000

HDR = struct.Struct("<I I I I I I I I I I I I I I I I")


def align(v: int, a: int) -> int:
    return (v + a - 1) // a * a


def sum16_be(data: bytes) -> int:
    total = 0
    for i in range(0, len(data) - 1, 2):
        total = (total + struct.unpack_from(">H", data, i)[0]) & 0xFFFF
    if len(data) & 1:
        total = (total + (data[-1] << 8)) & 0xFFFF
    return total


def squashfs_bytes_used(path: Path) -> int:
    data = path.read_bytes()
    if data[:4] != b"hsqs":
        sys.exit(f"{path}: not squashfs")
    used = struct.unpack_from("<Q", data, 40)[0]
    if not used or used > len(data):
        sys.exit(f"{path}: bad bytes_used {used:#x} (file {len(data):#x})")
    return used, data


def wrap_rootfs(data: bytes) -> bytes:
    """Pad and stamp the Realtek rootfs length/checksum."""
    size = len(data)
    length = align(size + TRAILER, SECTOR) - TRAILER
    total = length + TRAILER + 2
    if total > ROOTFS_MAX:
        sys.exit(f"rootfs too large: {total:#x} > {ROOTFS_MAX:#x}")
    out = bytearray(b"\xff" * total)
    out[:size] = data
    struct.pack_into(">I", out, 8, length)
    csum = (-sum16_be(out[:length + TRAILER])) & 0xFFFF
    struct.pack_into(">H", out, length + TRAILER, csum)
    assert sum16_be(out) == 0
    return bytes(out)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--loader", type=Path, required=True)
    ap.add_argument("--kernel", type=Path, required=True, help="raw kernel")
    ap.add_argument("--dtb", type=Path, help="DTB appended to the kernel")
    ap.add_argument("--rootfs", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--load", type=lambda s: int(s, 0), required=True)
    ap.add_argument("--entry", type=lambda s: int(s, 0), required=True)
    args = ap.parse_args()

    loader = args.loader.read_bytes()
    if len(loader) < HEADER_OFF + HEADER_SIZE:
        sys.exit("loader is shorter than its header slot")
    if loader[HEADER_OFF:HEADER_OFF + 4] != b"\0\0\0\0":
        # The section is zero-initialised at build time; allow a patched loader
        # only if it already carries our magic.
        if loader[HEADER_OFF:HEADER_OFF + 4] != MAGIC:
            sys.exit("loader header slot is not empty")

    raw = args.kernel.read_bytes()
    if args.dtb:
        raw += args.dtb.read_bytes()
    kernel = lzma.compress(raw, format=lzma.FORMAT_ALONE)
    rootfs_used, rootfs_raw = squashfs_bytes_used(args.rootfs)
    rootfs = wrap_rootfs(rootfs_raw)

    # Kernel A starts after the loader, 4K aligned.  The rootfs begins at the
    # bootloader-mandated 0x230000 (firmware relative).  If the kernel does not
    # fit before it, the rest goes after the rootfs trailer.
    kernel_a_off = align(len(loader), SECTOR)
    capacity = ROOTFS_BODY_OFF - kernel_a_off
    if capacity <= 0:
        sys.exit("loader overlaps the rootfs")
    kernel_a_len = min(len(kernel), capacity)
    kernel_b_len = len(kernel) - kernel_a_len
    kernel_b_off = 0
    if kernel_b_len:
        kernel_b_off = align(ROOTFS_BODY_OFF + len(rootfs), SECTOR)

    needed_end = ROOTFS_BODY_OFF + len(rootfs)
    if kernel_b_len:
        needed_end = max(needed_end, kernel_b_off + kernel_b_len)
    marker_fw = align(CVIMG_HEADER + needed_end + 2, BLOCK)
    body_len = marker_fw - CVIMG_HEADER - 2
    rootfs_data_off_body = body_len + 2

    body = bytearray(b"\xff" * body_len)
    body[:len(loader)] = loader
    body[kernel_a_off:kernel_a_off + kernel_a_len] = kernel[:kernel_a_len]
    body[ROOTFS_BODY_OFF:ROOTFS_BODY_OFF + len(rootfs)] = rootfs
    if kernel_b_len:
        body[kernel_b_off:kernel_b_off + kernel_b_len] = kernel[kernel_a_len:]

    header = HDR.pack(
        0x52574E4B,          # 'KNWR' little-endian
        VERSION,
        HEADER_SIZE,
        kernel_a_off,
        kernel_a_len,
        kernel_b_off,
        kernel_b_len,
        args.load,
        args.entry,
        ROOTFS_BODY_OFF,
        rootfs_used,
        rootfs_data_off_body,
        body_len,
        0, 0, 0,
    )
    body[HEADER_OFF:HEADER_OFF + len(header)] = header

    # cvimg: header + body + 16-bit checksum, sum over body+checksum == 0.
    body_sum = sum16_be(body)
    csum = (-body_sum) & 0xFFFF
    body = bytes(body) + struct.pack(">H", csum)
    if sum16_be(body) != 0:
        sys.exit("internal: cvimg checksum")

    cvimg = b"cr6c" + struct.pack(">III", LOADER_BASE, 0x30000, len(body))
    image = bytearray(cvimg + body)
    if len(image) < marker_fw:
        image += b"\xff" * (marker_fw - len(image))
    image += b"\xde\xad\xc0\xde"

    if len(image) > IMAGE_MAX:
        sys.exit(f"image too large: {len(image):#x} > {IMAGE_MAX:#x}")

    args.out.write_bytes(image)
    print(f"loader={len(loader):#x} kernel={len(kernel):#x} A={kernel_a_len:#x} B={kernel_b_len:#x}")
    print(f"kernel_a_off={kernel_a_off:#x} kernel_b_off={kernel_b_off:#x} rootfs={len(rootfs):#x}")
    print(f"rootfs_used={rootfs_used:#x} marker={marker_fw:#x} body={body_len:#x} image={len(image):#x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
