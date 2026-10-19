#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Print the kernel's load base or entry point from a vmlinux ELF.

The generic lzma-loader uses a single KERNEL_ENTRY as both the decompression
destination and the jump target. That only works when the kernel's entry symbol
sits at its load base. On MIPS, kernel_entry lives in .ref.text and the linker
places it well inside .text, so the two addresses differ and must be read from
the ELF rather than hardcoded (they move on every kernel rebuild).
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path


PT_LOAD = 1


def read_elf(path: Path) -> tuple[int, int]:
    data = path.read_bytes()
    if data[:4] != b"\x7fELF":
        raise ValueError(f"{path} is not an ELF file")
    if data[4] != 1:
        raise ValueError("only 32-bit ELF is supported")
    endian = "<" if data[5] == 1 else ">"

    entry = struct.unpack_from(endian + "I", data, 0x18)[0]
    phoff = struct.unpack_from(endian + "I", data, 0x1C)[0]
    phentsize = struct.unpack_from(endian + "H", data, 0x2A)[0]
    phnum = struct.unpack_from(endian + "H", data, 0x2C)[0]

    load_base = None
    for i in range(phnum):
        off = phoff + i * phentsize
        p_type, _, p_vaddr = struct.unpack_from(endian + "3I", data, off)
        if p_type == PT_LOAD:
            load_base = p_vaddr if load_base is None else min(load_base, p_vaddr)

    if load_base is None:
        raise ValueError(f"{path} has no PT_LOAD segment")
    return load_base, entry


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("vmlinux", type=Path)
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--load", action="store_true", help="lowest PT_LOAD vaddr")
    group.add_argument("--entry", action="store_true", help="ELF entry point")
    args = parser.parse_args()

    load_base, entry = read_elf(args.vmlinux)
    print(f"0x{load_base if args.load else entry:08x}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError) as exc:
        print(f"kernel-addrs: {exc}", file=sys.stderr)
        sys.exit(1)
