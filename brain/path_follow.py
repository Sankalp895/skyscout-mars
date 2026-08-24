"""B3 steps 3-4 -- waypoint / carrot path follower."""
import numpy as np


class PathFollower:
    def __init__(self, path_xyz, speed=1.5, lookahead=2.5, arrive_radius=1.0,
                 dt=0.02, max_lag=2.0):
        self.path = np.asarray(path_xyz, float)
        seg = np.linalg.norm(np.diff(self.path, axis=0), axis=1)
        self.s = np.r_[0.0, np.cumsum(seg)]
        self.total = float(self.s[-1])
        self.speed = speed
        self.lookahead = lookahead
        self.arrive_radius = arrive_radius
        self._s_prev = 0.0
        self.dt = dt
        self.max_lag = max_lag
        self._s_ref = 0.0

    def _at(self, s):
        s = float(np.clip(s, 0.0, self.total))
        k = int(np.searchsorted(self.s, s) - 1)
        k = int(np.clip(k, 0, len(self.path) - 2))
        seg = self.s[k + 1] - self.s[k]
        t = 0.0 if seg <= 0 else (s - self.s[k]) / seg
        p = self.path[k] + t * (self.path[k + 1] - self.path[k])
        d = self.path[k + 1] - self.path[k]
        n = np.linalg.norm(d)
        return p, (d / n if n > 1e-9 else np.array([1.0, 0, 0]))

    def project(self, pos):
        """Arc length of the closest point on the path, and the 3D distance to it."""
        pos = np.asarray(pos, float)
        lo = max(0.0, self._s_prev - 1.0)
        hi = min(self.total, self._s_prev + 8.0)
        ss = np.linspace(lo, hi, 160)
        pts = np.array([self._at(s)[0] for s in ss])
        d = np.linalg.norm(pts - pos, axis=1)
        k = int(np.argmin(d))
        self._s_prev = float(ss[k])
        return self._s_prev, float(d[k])

    def target(self, pos):
        """(target_pos, target_vel, progress_fraction, done)."""
        s, _ = self.project(pos)
        self._s_ref = min(self._s_ref + self.speed * self.dt, s + self.max_lag)
        self._s_ref = float(np.clip(self._s_ref, 0.0, self.total))
        ref, tangent = self._at(self._s_ref)
        remaining = self.total - s
        speed = self.speed * float(np.clip(remaining / 2.0, 0.0, 1.0))
        done = (remaining < self.arrive_radius
                and np.linalg.norm(np.asarray(pos) - self.path[-1]) < self.arrive_radius)
        return ref, tangent * speed, s / max(self.total, 1e-9), done


def cross_track(path_xyz, pos, horizontal=True):
    """Shortest distance from pos to the planned polyline."""
    path = np.asarray(path_xyz, float)
    p = np.asarray(pos, float)
    if horizontal:
        path, p = path[:, :2], p[:2]
    a, b = path[:-1], path[1:]
    ab = b - a
    denom = np.einsum("ij,ij->i", ab, ab)
    t = np.clip(np.einsum("ij,ij->i", p - a, ab) / np.where(denom > 1e-12, denom, 1), 0, 1)
    closest = a + t[:, None] * ab
    return float(np.linalg.norm(closest - p, axis=1).min())
