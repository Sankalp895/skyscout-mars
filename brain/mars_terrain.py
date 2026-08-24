"""SkyScout B1.5 -- procedural Mars terrain as a PyBullet heightfield."""
import numpy as np
import pybullet as p

SEED = 20260808
N = 128
CELL = 0.625
EXTENT = N * CELL

CRATER_CENTRE = (-15.0, 10.0)
CRATER_RADIUS = 12.0
CRATER_DEPTH = 2.5
CRATER_RIM = 1.0

RIDGE_HEIGHT = 4.0
N_ROCKS = 12
ROCK_RADIUS = (1.0, 3.0)
ROCK_HEIGHT = (0.8, 2.5)


def _grid(n=N):
    """World-coordinate meshgrid. EXTENT is fixed, so cell size follows n."""
    ax = np.linspace(-EXTENT / 2, EXTENT / 2, n)
    return np.meshgrid(ax, ax, indexing="ij")


def build_heightmap(seed=SEED, n=N):
    """Return (h, features) with h in METRES, shape (N, N), indexed [ix, iy]."""
    rng = np.random.default_rng(seed)
    xx, yy = _grid(n)
    features = []

    h = np.zeros_like(xx)
    for wx, wy, amp in ((0.035, 0.021, 0.9), (0.017, 0.041, 0.6), (0.061, 0.055, 0.3)):
        ph = rng.uniform(0, 2 * np.pi, 2)
        h += amp * np.sin(wx * 2 * np.pi * xx + ph[0]) * np.cos(wy * 2 * np.pi * yy + ph[1])
    features.append(f"rolling base, amplitude ~{np.ptp(h):.1f} m")

    d_ridge = np.abs(0.6 * xx + 0.8 * yy - 14.0)
    ridge = RIDGE_HEIGHT * np.exp(-(d_ridge / 5.0) ** 2)
    h += ridge
    features.append(f"ridge, peak {RIDGE_HEIGHT:.1f} m, running diagonally")

    r = np.hypot(xx - CRATER_CENTRE[0], yy - CRATER_CENTRE[1])
    bowl = -CRATER_DEPTH * np.clip(1.0 - (r / CRATER_RADIUS) ** 2, 0, None)
    rim = CRATER_RIM * np.exp(-((r - CRATER_RADIUS) / 2.2) ** 2)
    h += bowl + rim
    features.append(f"crater at {CRATER_CENTRE}, r {CRATER_RADIUS:.0f} m, "
                    f"depth {CRATER_DEPTH:.1f} m, rim +{CRATER_RIM:.1f} m")

    rocks = []
    for _ in range(N_ROCKS):
        while True:
            rx, ry = rng.uniform(-EXTENT / 2 + 5, EXTENT / 2 - 5, 2)
            if np.hypot(rx - CRATER_CENTRE[0], ry - CRATER_CENTRE[1]) > CRATER_RADIUS + 3:
                break
        rr = rng.uniform(*ROCK_RADIUS)
        rh = rng.uniform(*ROCK_HEIGHT)
        h += rh * np.exp(-(np.hypot(xx - rx, yy - ry) / rr) ** 2)
        rocks.append((round(rx, 1), round(ry, 1), round(rr, 1), round(rh, 1)))
    features.append(f"{N_ROCKS} boulders, r {ROCK_RADIUS[0]}-{ROCK_RADIUS[1]} m, "
                    f"height {ROCK_HEIGHT[0]}-{ROCK_HEIGHT[1]} m")

    return h, features, rocks


class MarsTerrain:
    """Heightfield ground + a ground-truth height query."""

    def __init__(self, seed=SEED, n=N):
        self.n = n
        self.cell = EXTENT / (n - 1)
        self.h, self.features, self.rocks = build_heightmap(seed, n)
        self.seed = seed
        self.min_h = float(self.h.min())
        self.max_h = float(self.h.max())

        shape = p.createCollisionShape(
            p.GEOM_HEIGHTFIELD,
            meshScale=[self.cell, self.cell, 1.0],
            # the differential constrains theta_port + theta_stbd = 0, leaving the difference free
            heightfieldData=self.h.T.reshape(-1).astype(np.float64).tolist(),
            numHeightfieldRows=n,
            numHeightfieldColumns=n,
        )
        self.body = p.createMultiBody(0, shape)
        p.changeVisualShape(self.body, -1, rgbaColor=[0.62, 0.42, 0.31, 1.0])
        p.changeDynamics(self.body, -1, lateralFriction=0.9)

        self.z_offset = 0.5 * (self.min_h + self.max_h)
        p.resetBasePositionAndOrientation(self.body, [0, 0, self.z_offset], [0, 0, 0, 1])

    def terrain_height(self, x, y):
        """Ground elevation (m) at a world (x, y), bilinear on the heightmap."""
        n = self.n
        fx = np.clip(x / self.cell + (n - 1) / 2.0, 0, n - 1)
        fy = np.clip(y / self.cell + (n - 1) / 2.0, 0, n - 1)
        i0, j0 = int(np.floor(fx)), int(np.floor(fy))
        i1, j1 = min(i0 + 1, n - 1), min(j0 + 1, n - 1)
        tx, ty = fx - i0, fy - j0
        return float(
            self.h[i0, j0] * (1 - tx) * (1 - ty) + self.h[i1, j0] * tx * (1 - ty)
            + self.h[i0, j1] * (1 - tx) * ty + self.h[i1, j1] * tx * ty
        )

    def verify_against_raycast(self, n=40, seed=3):
        """Assert terrain_height() matches the real collision surface."""
        rng = np.random.default_rng(seed)
        half = EXTENT / 2 - 6
        worst, pts = 0.0, []
        for _ in range(n):
            x, y = rng.uniform(-half, half, 2)
            hit = p.rayTest([x, y, 80], [x, y, -80])[0]
            if hit[0] < 0:
                continue
            d = abs(self.terrain_height(x, y) - hit[3][2])
            worst = max(worst, d)
            pts.append(d)
        return worst, len(pts)

    def summary(self):
        return {
            "seed": self.seed,
            "extent_m": EXTENT,
            "samples": f"{self.n}x{self.n}",
            "cell_m": round(self.cell, 3),
            "height_range_m": (round(self.min_h, 2), round(self.max_h, 2)),
            "features": self.features,
        }
