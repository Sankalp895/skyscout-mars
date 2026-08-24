"""Offline frame rendering for SkyScout Track B."""
import numpy as np
from PIL import Image, ImageDraw

TRAIL_RGB = (0, 235, 255)
HUD_BG = (0, 0, 0)
HUD_FG = (255, 255, 255)


def look_at(eye, target, up=(0, 0, 1)):
    """Right-handed view matrix, camera looking down -Z."""
    eye, target, up = map(lambda v: np.asarray(v, float), (eye, target, up))
    f = target - eye
    f /= np.linalg.norm(f)
    s = np.cross(f, up)
    if np.linalg.norm(s) < 1e-9:
        s = np.cross(f, np.array([0.0, 1.0, 0.0]))
    s /= np.linalg.norm(s)
    u = np.cross(s, f)
    M = np.eye(4)
    M[0, :3], M[1, :3], M[2, :3] = s, u, -f
    M[:3, 3] = -M[:3, :3] @ eye
    return M


def perspective(fov_deg, aspect, near=0.1, far=400.0):
    t = 1.0 / np.tan(np.radians(fov_deg) / 2.0)
    P = np.zeros((4, 4))
    P[0, 0] = t / aspect
    P[1, 1] = t
    P[2, 2] = (far + near) / (near - far)
    P[2, 3] = 2 * far * near / (near - far)
    P[3, 2] = -1.0
    return P


def project(pt, V, P, w, h):
    clip = P @ (V @ np.array([pt[0], pt[1], pt[2], 1.0]))
    if clip[3] <= 1e-6:
        return None
    n = clip[:3] / clip[3]
    if not (-1.3 < n[0] < 1.3 and -1.3 < n[1] < 1.3):
        return None
    return ((n[0] * 0.5 + 0.5) * w, (1.0 - (n[1] * 0.5 + 0.5)) * h)


class FollowRenderer:
    """Chase camera + overlay, writing PNGs from a headless run."""

    def __init__(self, sim, res=(960, 600), fov=55.0,
                 back=7.0, up=3.0, side=5.0, lerp=0.15):
        if sim.camera is None:
            raise RuntimeError("build SkyScoutSim with camera_res=... "
                               "(cameras must be added before scene.build())")
        self.sim, self.res, self.fov = sim, res, fov
        self.back, self.up, self.side, self.lerp = back, up, side, lerp
        self.cam = sim.camera
        self.trail = []
        self._target = None
        self._warmed = False

    def _pose_for(self, pos):
        if self._target is None:
            self._target = np.asarray(pos, float).copy()
        self._target += self.lerp * (np.asarray(pos, float) - self._target)
        eye = self._target + np.array([-self.back, -self.side, self.up])
        return eye, self._target.copy()

    def warm_up(self):
        """Render one THROWAWAY frame before any timed capture."""
        import time
        t0 = time.time()
        s = self.sim.get_true_state()
        eye, tgt = self._pose_for(s["pos"])
        self.cam.set_pose(pos=tuple(eye), lookat=tuple(tgt))
        self.cam.render(rgb=True)
        self._warmed = True
        return time.time() - t0

    def capture(self, path, phase="", extra=()):
        if not self._warmed:
            raise RuntimeError("call warm_up() before capture(); see docstring")
        s = self.sim.get_true_state()
        pos = s["pos"]
        self.trail.append(pos.copy())

        eye, tgt = self._pose_for(pos)
        self.cam.set_pose(pos=tuple(eye), lookat=tuple(tgt))
        rgb = np.asarray(self.cam.render(rgb=True)[0], dtype=np.uint8)[:, :, :3]

        w, h = self.res
        V = look_at(eye, tgt)
        P = perspective(self.fov, w / h)
        im = Image.fromarray(rgb)
        d = ImageDraw.Draw(im)

        z0 = self.sim.terrain_height(0, 0)
        o = project([0, 0, z0], V, P, w, h)
        for vec, col in (((8, 0, 0), (255, 70, 70)), ((0, 8, 0), (70, 255, 70)),
                         ((0, 0, 8), (90, 150, 255))):
            e = project([vec[0], vec[1], z0 + vec[2]], V, P, w, h)
            if o and e:
                d.line([o, e], fill=col, width=3)

        pix = [q for q in (project(t, V, P, w, h) for t in self.trail) if q]
        if len(pix) > 1:
            d.line(pix, fill=TRAIL_RGB, width=3)

        ground = self.sim.terrain_height(pos[0], pos[1])
        lines = [f"PHASE  {phase.upper()}" if phase else "PHASE  --",
                 f"POS    x{pos[0]:+7.2f}  y{pos[1]:+7.2f}  z{pos[2]:+7.2f} m",
                 f"AGL    {pos[2] - ground:6.2f} m",
                 f"SPEED  {np.linalg.norm(s['lin_vel']):6.2f} m/s",
                 f"T      {s['t']:6.2f} s"]
        lines.extend(extra)
        d.rectangle([0, 0, 340, 12 + 15 * len(lines)], fill=HUD_BG)
        for i, ln in enumerate(lines):
            d.text((8, 6 + 15 * i), ln, fill=HUD_FG)
        im.save(path)
        return path
