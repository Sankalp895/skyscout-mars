"""SkyScout Track B / B1 -- procedural Mars terrain for Genesis."""
import numpy as np

SEED = 20260808
N = 128
EXTENT = 80.0
CELL = EXTENT / (N - 1)

CRATER_CENTRE = (-15.0, 10.0)
CRATER_RADIUS = 12.0
CRATER_DEPTH = 2.5
CRATER_RIM = 1.0

RIDGE_HEIGHT = 4.0
N_ROCKS = 12
ROCK_RADIUS = (1.0, 3.0)
ROCK_HEIGHT = (0.8, 2.5)


def build_heightmap(seed=SEED, n=N):
    """(h, features, rocks). h is METRES, shape (n, n), indexed [ix, iy]."""
    rng = np.random.default_rng(seed)
    ax = np.linspace(-EXTENT / 2, EXTENT / 2, n)
    xx, yy = np.meshgrid(ax, ax, indexing="ij")
    features = []

    h = np.zeros_like(xx)
    for wx, wy, amp in ((0.035, 0.021, 0.9), (0.017, 0.041, 0.6), (0.061, 0.055, 0.3)):
        ph = rng.uniform(0, 2 * np.pi, 2)
        h += amp * np.sin(wx * 2 * np.pi * xx + ph[0]) * np.cos(wy * 2 * np.pi * yy + ph[1])
    features.append(f"rolling base, amplitude ~{np.ptp(h):.1f} m")

    d_ridge = np.abs(0.6 * xx + 0.8 * yy - 14.0)
    h += RIDGE_HEIGHT * np.exp(-(d_ridge / 5.0) ** 2)
    features.append(f"ridge, peak {RIDGE_HEIGHT:.1f} m, diagonal")

    r = np.hypot(xx - CRATER_CENTRE[0], yy - CRATER_CENTRE[1])
    h += -CRATER_DEPTH * np.clip(1.0 - (r / CRATER_RADIUS) ** 2, 0, None)
    h += CRATER_RIM * np.exp(-((r - CRATER_RADIUS) / 2.2) ** 2)
    features.append(f"crater at {CRATER_CENTRE}, r {CRATER_RADIUS:.0f} m, "
                    f"depth {CRATER_DEPTH:.1f} m, rim +{CRATER_RIM:.1f} m")

    rocks = []
    for _ in range(N_ROCKS):
        while True:
            rx, ry = rng.uniform(-EXTENT / 2 + 5, EXTENT / 2 - 5, 2)
            if np.hypot(rx - CRATER_CENTRE[0], ry - CRATER_CENTRE[1]) > CRATER_RADIUS + 3:
                break
        rr, rh = rng.uniform(*ROCK_RADIUS), rng.uniform(*ROCK_HEIGHT)
        h += rh * np.exp(-(np.hypot(xx - rx, yy - ry) / rr) ** 2)
        rocks.append((round(rx, 1), round(ry, 1), round(rr, 1), round(rh, 1)))
    features.append(f"{N_ROCKS} boulders, r {ROCK_RADIUS[0]}-{ROCK_RADIUS[1]} m, "
                    f"h {ROCK_HEIGHT[0]}-{ROCK_HEIGHT[1]} m")

    return h, features, rocks


class MarsWorld:
    """Genesis terrain entity + a ground-truth height query."""

    def __init__(self, scene, seed=SEED, n=N):
        import genesis as gs
        self.seed, self.n, self.cell = seed, n, EXTENT / (n - 1)
        self.h, self.features, self.rocks = build_heightmap(seed, n)
        self.min_h, self.max_h = float(self.h.min()), float(self.h.max())

        # the differential constrains theta_port + theta_stbd = 0, leaving the difference free
        self.origin = np.array([-EXTENT / 2, -EXTENT / 2, 0.0])
        self.entity = scene.add_entity(
            gs.morphs.Terrain(
                height_field=self.h,
                horizontal_scale=self.cell,
                vertical_scale=1.0,
                pos=tuple(self.origin),
            ),
            surface=gs.surfaces.Default(color=(0.55, 0.34, 0.24), roughness=0.95),
        )

    def terrain_height(self, x, y):
        """Ground elevation (m) at world (x, y)."""
        n = self.n
        fx = np.clip((x - self.origin[0]) / self.cell, 0, n - 1)
        fy = np.clip((y - self.origin[1]) / self.cell, 0, n - 1)
        i0, j0 = int(np.floor(fx)), int(np.floor(fy))
        i1, j1 = min(i0 + 1, n - 1), min(j0 + 1, n - 1)
        tx, ty = fx - i0, fy - j0
        return float(
            self.h[i0, j0] * (1 - tx) * (1 - ty) + self.h[i1, j0] * tx * (1 - ty)
            + self.h[i0, j1] * (1 - tx) * ty + self.h[i1, j1] * tx * ty
        )

    def summary(self):
        return {
            "seed": self.seed,
            "extent_m": EXTENT,
            "samples": f"{self.n}x{self.n}",
            "cell_m": round(self.cell, 3),
            "elev_min_max_m": (round(self.min_h, 2), round(self.max_h, 2)),
            "features": self.features,
        }
