#!/usr/bin/env python3
"""Generate convex-hull collision meshes for the JAKA S12 links.

Why: the URDF used the full visual STLs (5k-30k triangles per link) for collision.
Under Gazebo's DART+Bullet physics, a triangle mesh near the table is tested
triangle-by-triangle every 1 ms step, which dropped the real-time factor to ~0.12
whenever the wrist worked near the table. A convex hull is a few hundred triangles,
always encloses the original link (so MoveIt planning only becomes slightly more
conservative), and is cheap for Bullet.

Usage:
    python3 tools/make_convex_collision_meshes.py
Writes jaka_description/meshes/jaka_s12_meshes/collision/<Link>.STL (binary STL,
same units/frame as the source). Source meshes are never modified.
"""
import struct
import sys
from pathlib import Path

import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT / "jaka_ros_ws/src/jaka_description/meshes/jaka_s12_meshes"
OUT_DIR = SRC_DIR / "collision"
LINKS = ["base_link", "Link_01", "Link_02", "Link_03", "Link_04", "Link_05", "Link_06"]


def read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    if 84 + count * 50 == len(data):  # binary STL
        rec = np.frombuffer(data[84:84 + count * 50],
                            dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
        return rec["v"].reshape(-1, 3).astype(np.float64)
    verts = [list(map(float, line.split()[1:4]))
             for line in data.decode(errors="ignore").splitlines() if line.strip().startswith("vertex")]
    return np.asarray(verts, dtype=np.float64)


def write_stl(path: Path, points: np.ndarray, hull: ConvexHull) -> None:
    centroid = points[hull.vertices].mean(axis=0)
    with path.open("wb") as fh:
        fh.write(b"convex hull collision mesh".ljust(80, b" "))
        fh.write(struct.pack("<I", len(hull.simplices)))
        for tri in hull.simplices:
            a, b, c = points[tri]
            normal = np.cross(b - a, c - a)
            if np.dot(normal, a - centroid) < 0:  # make winding/normal point outward
                b, c = c, b
                normal = -normal
            norm = np.linalg.norm(normal)
            normal = normal / norm if norm > 0 else normal
            fh.write(struct.pack("<12fH", *normal, *a, *b, *c, 0))


def main() -> int:
    OUT_DIR.mkdir(exist_ok=True)
    for link in LINKS:
        src = SRC_DIR / f"{link}.STL"
        points = np.unique(read_stl(src).round(7), axis=0)
        hull = ConvexHull(points)
        write_stl(OUT_DIR / f"{link}.STL", points, hull)
        print(f"{link:10s} {len(points):6d} verts -> hull {len(hull.simplices):4d} triangles "
              f"(extent {np.ptp(points, axis=0).round(3)})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
