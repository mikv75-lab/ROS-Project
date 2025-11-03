#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Einfacher STL-Rec center:
- Sucht NUR im Ordner, in dem dieses Script liegt (nicht rekursiv)
- Korrigiert alle *.stl: XY auf Mitte, Boden auf z=0
- Schreibt Ergebnis mit GLEICHEM Dateinamen in Unterordner "output"
- Keine Argumente nötig (Doppelklick-freundlich)

Windows: Rechstklick -> Öffnen mit -> Python
Linux/Mac: python stl_center_folder.py
"""

import os, struct, sys

def is_binary_stl(data: bytes) -> bool:
    if len(data) < 84:
        return False
    tri_count = struct.unpack('<I', data[80:84])[0]
    expected = 84 + 50 * tri_count
    return expected == len(data)

def read_stl(path):
    with open(path, 'rb') as f:
        data = f.read()
    tris = []
    if is_binary_stl(data):
        tri_count = struct.unpack('<I', data[80:84])[0]
        off = 84
        for _ in range(tri_count):
            floats = struct.unpack('<12f', data[off:off+48])
            v1 = floats[3:6]; v2 = floats[6:9]; v3 = floats[9:12]
            tris.append((list(v1), list(v2), list(v3)))
            off += 50
        return tris
    # ASCII
    text = data.decode('utf-8', errors='ignore').splitlines()
    buf = []
    for line in text:
        s = line.strip()
        if s.lower().startswith('vertex'):
            parts = s.split()
            if len(parts) >= 4:
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                buf.append([x, y, z])
                if len(buf) == 3:
                    tris.append((buf[0], buf[1], buf[2]))
                    buf = []
    return tris

def bbox(tris):
    xs = [v[0] for tri in tris for v in tri]
    ys = [v[1] for tri in tris for v in tri]
    zs = [v[2] for tri in tris for v in tri]
    return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

def recenter(tris):
    xmin, xmax, ymin, ymax, zmin, zmax = bbox(tris)
    cx = 0.5 * (xmin + xmax)
    cy = 0.5 * (ymin + ymax)
    shift_x = -cx
    shift_y = -cy
    shift_z = -zmin  # Boden -> z=0
    out = []
    for v1, v2, v3 in tris:
        a = [v1[0] + shift_x, v1[1] + shift_y, v1[2] + shift_z]
        b = [v2[0] + shift_x, v2[1] + shift_y, v2[2] + shift_z]
        c = [v3[0] + shift_x, v3[1] + shift_y, v3[2] + shift_z]
        out.append((a, b, c))
    return out

def normal(a, b, c):
    ax, ay, az = a; bx, by, bz = b; cx, cy, cz = c
    ux, uy, uz = (bx-ax, by-ay, bz-az)
    vx, vy, vz = (cx-ax, cy-ay, cz-az)
    nx = uy*vz - uz*vy
    ny = uz*vx - ux*vz
    nz = ux*vy - uy*vx
    l = (nx*nx + ny*ny + nz*nz) ** 0.5
    if l == 0.0:
        return (0.0, 0.0, 0.0)
    return (nx/l, ny/l, nz/l)

def write_binary_stl(path, tris, name=b'centered'):
    header = bytearray(80)
    header[:min(len(name), 80)] = name[:min(len(name), 80)]
    with open(path, 'wb') as f:
        f.write(header)
        f.write(struct.pack('<I', len(tris)))
        for a, b, c in tris:
            n = normal(a, b, c)
            f.write(struct.pack('<12f', n[0], n[1], n[2],
                                       a[0], a[1], a[2],
                                       b[0], b[1], b[2],
                                       c[0], c[1], c[2]))
            f.write(struct.pack('<H', 0))

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(base_dir, 'output')
    os.makedirs(out_dir, exist_ok=True)

    stls = [fn for fn in os.listdir(base_dir) if fn.lower().endswith('.stl')]
    if not stls:
        print("Keine STL-Dateien im Script-Ordner gefunden.")
        return

    print(f"Verarbeite {len(stls)} Datei(en) in: {base_dir}")
    for fn in stls:
        src = os.path.join(base_dir, fn)
        try:
            tris = read_stl(src)
            if not tris:
                print(f"[SKIP] {fn}: keine Dreiecke erkannt.")
                continue
            tris2 = recenter(tris)
            dst = os.path.join(out_dir, fn)  # gleicher Name im 'output'
            write_binary_stl(dst, tris2, name=fn.encode('ascii', 'ignore')[:80])
            print(f"[OK] {fn} -> output/{fn}")
        except Exception as e:
            print(f"[ERR] {fn}: {e}")

    print("Fertig. Ergebnisse im 'output'-Ordner.")

if __name__ == '__main__':
    main()
