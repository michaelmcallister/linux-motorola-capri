#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Align an arm64 Image for the Moto G10 stock bootloader."""

import argparse
import struct
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    if args.image.resolve() == args.output.resolve():
        parser.error("input and output must be different files")
    data = args.image.read_bytes()
    if len(data) < 64 or data[56:60] != b"ARM\x64":
        parser.error("input must be an uncompressed arm64 Image")
    offset, size, flags = struct.unpack_from("<QQQ", data, 8)
    if offset != 0 or size == 0:
        parser.error("expected an unwrapped Image with zero text offset")
    if b"motorola,capri\0" not in data:
        parser.error("Image does not contain the Moto G10 device tree")

    # The bootloader loads at base + 0x80000. Put the unchanged kernel at
    # base + 0x200000 and branch to it without changing the boot arguments.
    prefix_size = 0x180000
    prefix = bytearray(prefix_size)
    nop = 0xd503201f
    branch = 0x14000000 | ((prefix_size - 4) // 4)
    struct.pack_into("<IIQQQ", prefix, 0, nop, branch,
                     0x80000, size + prefix_size, flags)
    prefix[56:60] = b"ARM\x64"
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(prefix + data)
    print(f"Wrote {args.output}: {prefix_size} byte prefix, {len(data)} byte Image")


if __name__ == "__main__":
    main()
