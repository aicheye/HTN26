#!/usr/bin/env python3
"""Reads files straight off a QNX6 (power-safe) filesystem, such as the Pi's /data partition on its SD card.

Linux on this laptop has no QNX6 driver, so the card cannot be mounted. This reads the on-disk structures
itself. It only ever opens the device for reading.

    python3 pi/qnx6-extract.py /dev/sda3 ls /home/qnxuser/recordings
    python3 pi/qnx6-extract.py /dev/sda3 copy /home/qnxuser/recordings recordings

The device must be readable: sudo chmod o+r /dev/sda3 (lasts until the card is unplugged).
Layout follows the Linux kernel's fs/qnx6: an 8 KB boot block, a 4 KB superblock area, then the blocks.
A second superblock sits after the last block, and the one with the higher serial number is the current one.
"""
import os
import struct
import sys

BOOT_BLOCK = 0x2000
SUPERBLOCK_AREA = 0x1000
MAGIC = 0x68191122
UNUSED = 0xFFFFFFFF
INODE_SIZE = 128
ROOT_INODE = 1
S_IFMT, S_IFDIR, S_IFREG = 0o170000, 0o040000, 0o100000


class Tree:
    """A file's blocks: 16 top pointers, with `levels` layers of pointer blocks below them."""

    def __init__(self, fs, size, pointers, levels):
        self.fs, self.size, self.pointers, self.levels = fs, size, pointers, levels

    def block_number(self, index):
        per_block = self.fs.block_size // 4
        span = per_block ** self.levels  # data blocks covered by one top pointer
        pointer = self.pointers[index // span] if index // span < 16 else UNUSED
        index %= span
        for _ in range(self.levels):
            if pointer == UNUSED:
                return UNUSED
            span //= per_block
            table = self.fs.read_block(pointer)
            pointer = struct.unpack_from("<I", table, (index // span) * 4)[0]
            index %= span
        return pointer

    def read(self, offset=0, length=None):
        length = self.size - offset if length is None else min(length, self.size - offset)
        out = bytearray()
        while length > 0:
            index, within = divmod(offset, self.fs.block_size)
            take = min(self.fs.block_size - within, length)
            number = self.block_number(index)
            block = b"\0" * self.fs.block_size if number == UNUSED else self.fs.read_block(number)
            out += block[within:within + take]
            offset += take
            length -= take
        return bytes(out)


class Qnx6:
    def __init__(self, path):
        self.device = open(path, "rb")
        first = self.read_superblock(BOOT_BLOCK)
        if first is None:
            raise SystemExit(f"{path}: no QNX6 superblock found at offset {BOOT_BLOCK:#x}")
        self.block_size = first["block_size"]
        second = self.read_superblock(BOOT_BLOCK + SUPERBLOCK_AREA + first["blocks"] * first["block_size"])
        current = second if second and second["serial"] > first["serial"] else first
        self.block_size = current["block_size"]
        self.cache = {}
        self.inodes = Tree(self, *current["inode_root"])
        self.long_names = Tree(self, *current["longfile_root"])
        self.info = current

    def read_superblock(self, offset):
        self.device.seek(offset)
        raw = self.device.read(512)
        if len(raw) < 312 or struct.unpack_from("<I", raw, 0)[0] != MAGIC:
            return None
        root = lambda at: (struct.unpack_from("<Q", raw, at)[0], struct.unpack_from("<16I", raw, at + 8), raw[at + 72])
        return {
            "serial": struct.unpack_from("<Q", raw, 8)[0],
            "block_size": struct.unpack_from("<I", raw, 48)[0],
            "blocks": struct.unpack_from("<I", raw, 60)[0],
            "inode_root": root(72),
            "longfile_root": root(232),
        }

    def read_block(self, number):
        if number not in self.cache:
            if len(self.cache) > 4096:
                self.cache.clear()
            self.device.seek(BOOT_BLOCK + SUPERBLOCK_AREA + number * self.block_size)
            self.cache[number] = self.device.read(self.block_size)
        return self.cache[number]

    def inode(self, number):
        raw = self.inodes.read((number - 1) * INODE_SIZE, INODE_SIZE)
        size, = struct.unpack_from("<Q", raw, 0)
        # size 8, uid 4, gid 4, four timestamps 16, then mode at 32, 16 block pointers at 36, tree depth at 100.
        # Checked against the card: inode 1 reads as mode 040755, a directory.
        mode, = struct.unpack_from("<H", raw, 32)
        return {"mode": mode, "tree": Tree(self, size, struct.unpack_from("<16I", raw, 36), raw[100])}

    def list_dir(self, inode):
        data = inode["tree"].read()
        for at in range(0, len(data) - 31, 32):
            number, length = struct.unpack_from("<IB", data, at)
            if number == 0:
                continue
            if length <= 27:
                name = data[at + 5:at + 5 + length]
            else:  # long name: the entry points at a block in the long-name file
                block, = struct.unpack_from("<I", data, at + 8)
                long_entry = self.long_names.read(block * self.block_size, self.block_size)
                name = long_entry[2:2 + struct.unpack_from("<H", long_entry, 0)[0]]
            name = name.decode("utf-8", "replace")
            if name not in (".", ".."):
                yield name, number

    def find(self, path):
        inode = self.inode(ROOT_INODE)
        for part in [p for p in path.split("/") if p]:
            entries = dict(self.list_dir(inode))
            if part not in entries:
                raise SystemExit(f"{path}: '{part}' not found. Present: {', '.join(sorted(entries)) or 'nothing'}")
            inode = self.inode(entries[part])
        return inode

    def copy(self, inode, target):
        """Copies a directory tree. Returns (files, bytes)."""
        os.makedirs(target, exist_ok=True)
        files = size = 0
        for name, number in sorted(self.list_dir(inode)):
            child = self.inode(number)
            kind = child["mode"] & S_IFMT
            if kind == S_IFDIR:
                f, s = self.copy(child, os.path.join(target, name))
                files, size = files + f, size + s
            elif kind == S_IFREG:
                with open(os.path.join(target, name), "wb") as out:
                    out.write(child["tree"].read())
                files, size = files + 1, size + child["tree"].size
        return files, size


def main():
    if len(sys.argv) < 4 or sys.argv[2] not in ("ls", "copy") or (sys.argv[2] == "copy" and len(sys.argv) < 5):
        raise SystemExit(__doc__)
    fs = Qnx6(sys.argv[1])
    print(f"QNX6 filesystem: block size {fs.block_size}, {fs.info['blocks']} blocks, superblock serial {fs.info['serial']}")
    source = fs.find(sys.argv[3])
    if sys.argv[2] == "ls":
        for name, number in sorted(fs.list_dir(source)):
            child = fs.inode(number)
            is_dir = child["mode"] & S_IFMT == S_IFDIR
            count = f"{sum(1 for _ in fs.list_dir(child))} entries" if is_dir else f"{child['tree'].size} bytes"
            print(f"  {name}{'/' if is_dir else ''}  {count}")
        return
    for name, number in sorted(fs.list_dir(source)):
        child = fs.inode(number)
        if child["mode"] & S_IFMT != S_IFDIR:
            continue
        files, size = fs.copy(child, os.path.join(sys.argv[4], name))
        print(f"  {name}: {files} files, {size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
