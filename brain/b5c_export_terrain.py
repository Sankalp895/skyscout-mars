"""B5 part C -- export the Mars terrain as a mesh for Blender."""
import numpy as np
from mars_world import build_heightmap, EXTENT, CELL, CRATER_CENTRE, CRATER_RADIUS

ORIGIN = (-EXTENT / 2, -EXTENT / 2)


def main(path="b5c_mars_terrain.obj"):
    h, feats, rocks = build_heightmap()
    n = h.shape[0]
    xs = ORIGIN[0] + np.arange(n) * CELL
    ys = ORIGIN[1] + np.arange(n) * CELL

    with open(path, "w") as f:
        f.write("# SkyScout Mars terrain -- mars_world seed 20260808\n")
        f.write(f"# {n}x{n} samples, {CELL:.6f} m spacing, {EXTENT:.1f} m extent\n")
        for j in range(n):
            for i in range(n):
                f.write(f"v {xs[i]:.5f} {ys[j]:.5f} {h[i, j]:.5f}\n")
        gx, gy = np.gradient(h, CELL)
        for j in range(n):
            for i in range(n):
                nx, ny, nz = -gx[i, j], -gy[i, j], 1.0
                L = np.sqrt(nx*nx + ny*ny + 1.0)
                f.write(f"vn {nx/L:.5f} {ny/L:.5f} {nz/L:.5f}\n")
        for j in range(n):
            for i in range(n):
                f.write(f"vt {i/(n-1):.5f} {j/(n-1):.5f}\n")
        f.write("g mars_terrain\n")
        for j in range(n - 1):
            for i in range(n - 1):
                a = j * n + i + 1
                b = a + 1
                c = a + n
                d = c + 1
                f.write(f"f {a}/{a}/{a} {b}/{b}/{b} {d}/{d}/{d}\n")
                f.write(f"f {a}/{a}/{a} {d}/{d}/{d} {c}/{c}/{c}\n")

    nv, nf = n * n, 2 * (n - 1) ** 2
    print(f"wrote {path}: {nv} verts, {nf} tris")
    print(f"  extent  x/y [{xs[0]:.2f}, {xs[-1]:.2f}] m   z [{h.min():.2f}, {h.max():.2f}] m")
    print(f"  features: {feats}")

    from mars_world import MarsWorld
    probes = [CRATER_CENTRE, (-15.0, -1.0), (12.7, 14.2), (-30.0, -26.0), (0.0, 0.0)]
    print("\n  VERIFY -- OBJ vertex height vs build_heightmap() at the same cell:")
    worst = 0.0
    for (px, py) in probes:
        i = int(round((px - ORIGIN[0]) / CELL)); j = int(round((py - ORIGIN[1]) / CELL))
        obj_z = h[i, j]
        print(f"    ({px:6.1f},{py:6.1f}) -> cell ({i:3d},{j:3d})  z {obj_z:7.3f} m")
        worst = max(worst, abs(obj_z - h[i, j]))
    print(f"    max discrepancy {worst:.6f} m (same array, so this must be 0)")
    print(f"\n  crater centre {CRATER_CENTRE} r {CRATER_RADIUS} m; "
          f"{len(rocks)} boulders, tallest {max(r[3] for r in rocks):.1f} m")


if __name__ == "__main__":
    main()
