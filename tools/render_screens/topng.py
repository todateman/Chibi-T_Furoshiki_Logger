#!/usr/bin/env python3
"""PPM(P6) を2倍に拡大したPNGにする"""
import struct, sys, zlib
for path in sys.argv[1:]:
    d = open(path, "rb").read()
    parts = d.split(b"\n", 3)
    w, h = map(int, parts[1].split())
    px = parts[3]
    rows = []
    for y in range(h):
        row = px[y*w*3:(y+1)*w*3]
        big = b"".join(row[i:i+3] * 2 for i in range(0, len(row), 3))
        rows += [b"\0" + big] * 2
    raw = zlib.compress(b"".join(rows), 9)
    def chunk(t, b): return struct.pack(">I", len(b)) + t + b + struct.pack(">I", zlib.crc32(t + b) & 0xffffffff)
    png = b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w*2, h*2, 8, 2, 0, 0, 0)) + chunk(b"IDAT", raw) + chunk(b"IEND", b"")
    open(path[:-4] + ".png", "wb").write(png)
