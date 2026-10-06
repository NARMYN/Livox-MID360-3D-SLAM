#!/usr/bin/env python3
"""Render a PCD point-cloud map to a PNG with a top view and a 3D view, coloured by height.

Usage:
    python3 tools/render_map.py maps/fastlio_map.pcd docs/images/fastlio_map.png

Requires numpy and matplotlib. Reads ASCII and binary PCD files.
"""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

BG, FG, MUTED, EDGE = '#0d1117', 'white', '#8b949e', '#30363d'


def read_pcd(path):
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            key, *vals = f.readline().decode('ascii', 'ignore').split()
            hdr[key] = vals
            if key == 'DATA':
                break
        fields, n = hdr['FIELDS'], int(hdr['POINTS'][0])
        if hdr['DATA'][0] == 'ascii':
            pts = np.loadtxt(f, dtype=np.float32, max_rows=n)
        else:
            kinds = {'F': 'f', 'U': 'u', 'I': 'i'}
            dt = np.dtype([(name, kinds[t] + s) for name, s, t in zip(fields, hdr['SIZE'], hdr['TYPE'])])
            rec = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
            pts = np.stack([rec[name].astype(np.float32) for name in fields], 1)
    xyz = pts[:, [fields.index('x'), fields.index('y'), fields.index('z')]]
    return xyz[np.isfinite(xyz).all(1)]


def render(src, dst):
    xyz = read_pcd(src)
    # Keep the dense core of the map so a few far-away returns don't shrink the plot
    centre = np.median(xyz[:, :2], 0)
    dist = np.linalg.norm(xyz[:, :2] - centre, axis=1)
    radius = np.percentile(dist, 85) * 1.15
    zlo, zhi = np.percentile(xyz[:, 2], [0.5, 99.5])
    xyz = xyz[(dist <= radius) & (xyz[:, 2] >= zlo) & (xyz[:, 2] <= zhi)]
    z = xyz[:, 2]

    fig = plt.figure(figsize=(13, 6), facecolor=BG)
    top = fig.add_subplot(1, 2, 1, facecolor=BG)
    order = np.argsort(z)
    top.scatter(xyz[order, 0], xyz[order, 1], c=z[order], s=0.7, cmap='turbo', linewidths=0, vmin=zlo, vmax=zhi)
    top.set_aspect('equal')
    top.set_title('Top view', color=FG, fontsize=13)
    top.set_xlabel('x [m]', color=MUTED)
    top.set_ylabel('y [m]', color=MUTED)
    top.tick_params(colors=MUTED)
    for spine in top.spines.values():
        spine.set_color(EDGE)

    view = fig.add_subplot(1, 2, 2, projection='3d', facecolor=BG)
    sc = view.scatter(xyz[:, 0], xyz[:, 1], z, c=z, s=0.6, cmap='turbo', linewidths=0, vmin=zlo, vmax=zhi)
    view.set_xlim(centre[0] - radius, centre[0] + radius)
    view.set_ylim(centre[1] - radius, centre[1] + radius)
    view.set_zlim(zlo, zhi)
    view.set_box_aspect((1, 1, 0.35))
    view.view_init(elev=32, azim=-60)
    view.set_axis_off()
    view.set_title('3D view', color=FG, fontsize=13)

    bar = fig.colorbar(sc, ax=[top, view], shrink=0.6, pad=0.02)
    bar.set_label('height z [m]', color=MUTED)
    bar.ax.tick_params(colors=MUTED)
    plt.savefig(dst, dpi=110, facecolor=BG, bbox_inches='tight')
    print(f'{src}: {len(xyz)} points rendered -> {dst}')


if __name__ == '__main__':
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    render(sys.argv[1], sys.argv[2])
