"""Visual clarity layer for the PyBullet view (SKYSCOUT_VISUAL_CLARITY)."""
import collections
import math
import os

import numpy as np
import pybullet as p

BODY = (0.88, 0.92, 1.00, 1)
WING = (0.33, 0.48, 0.72, 1)
NACELLE = (0.20, 0.22, 0.26, 1)
ROTOR = (0.10, 0.10, 0.12, 1)
ELEVON = (0.24, 0.36, 0.56, 1)
SKID = (0.30, 0.30, 0.33, 1)
NOSE = (1.00, 0.45, 0.05, 1)
PORT_TIP = (0.90, 0.15, 0.15, 1)
STBD_TIP = (0.15, 0.80, 0.25, 1)

TRAIL_COLOR = (0.0, 1.0, 1.0)
TEXT_COLOR = (1.0, 1.0, 1.0)


def _link_color(name):
    if name.startswith("rotor"):
        return ROTOR
    if name.startswith("nacelle"):
        return NACELLE
    if name.startswith("elevon"):
        return ELEVON
    if name.startswith("skid"):
        return SKID
    if name.startswith("sensor"):
        return NOSE
    if name.startswith("wing_panel_b_port"):
        return PORT_TIP
    if name.startswith("wing_panel_b_stbd"):
        return STBD_TIP
    if name.startswith("wing"):
        return WING
    return BODY


class VisualLayer:
    def __init__(self, sim, follow=True, trail=True, text=True,
                 cam_distance=6.0, cam_yaw=40.0, cam_pitch=-25.0,
                 follow_lerp=0.12, max_trail=400):
        self.sim = sim
        self.follow, self.trail_on, self.text_on = follow, trail, text
        self.cam_distance, self.cam_yaw, self.cam_pitch = cam_distance, cam_yaw, cam_pitch
        self.follow_lerp = follow_lerp
        self.max_trail = max_trail

        self._trail = collections.deque()
        self.trail_pts = collections.deque(maxlen=max_trail + 1)
        self._prev_pos = None
        self._text_ids = []
        self._cam_target = np.array(sim.get_true_state()["pos"], float)

        self.colorize()
        self.draw_world_axes()
        if sim.terrain is not None:
            self.draw_range_rings()
        p.resetDebugVisualizerCamera(cam_distance, cam_yaw, cam_pitch,
                                     self._cam_target.tolist())

    def colorize(self):
        d = self.sim.drone
        p.changeVisualShape(d, -1, rgbaColor=BODY)
        for j in range(self.sim.n_joints):
            name = p.getJointInfo(d, j)[12].decode()
            p.changeVisualShape(d, j, rgbaColor=_link_color(name))
        if self.sim.terrain is not None:
            p.changeVisualShape(self.sim.plane, -1, rgbaColor=(0.52, 0.36, 0.28, 1))

    def draw_world_axes(self, length=6.0):
        z = self.sim.terrain_height(0, 0) + 0.05
        for vec, col in (((length, 0, 0), (1, 0, 0)),
                         ((0, length, 0), (0, 1, 0)),
                         ((0, 0, length), (0, 0.4, 1))):
            p.addUserDebugLine([0, 0, z], [vec[0], vec[1], z + vec[2]],
                               col, 2.0, 0)
        p.addUserDebugText("X", [length + 0.6, 0, z], (1, 0, 0), 1.2)
        p.addUserDebugText("Y", [0, length + 0.6, z], (0, 1, 0), 1.2)

    def draw_range_rings(self, radii=(10, 20, 30), segments=36):
        """Distance rings on the terrain so scale and motion are judgeable."""
        for r in radii:
            pts = []
            for k in range(segments + 1):
                a = 2 * math.pi * k / segments
                x, y = r * math.cos(a), r * math.sin(a)
                pts.append([x, y, self.sim.terrain_height(x, y) + 0.08])
            for a, b in zip(pts[:-1], pts[1:]):
                p.addUserDebugLine(a, b, (0.45, 0.35, 0.30), 1.0, 0)
            p.addUserDebugText(f"{r} m", pts[0], (0.75, 0.6, 0.5), 0.9)

    def update(self, phase="", rt_factor=None):
        s = self.sim.get_true_state()
        pos = s["pos"]

        if self.trail_on and self._prev_pos is not None:
            if np.linalg.norm(pos - self._prev_pos) > 0.02:
                self._trail.append(
                    p.addUserDebugLine(self._prev_pos.tolist(), pos.tolist(),
                                       TRAIL_COLOR, 2.5, 0))
                self.trail_pts.append(pos.copy())
                self._prev_pos = pos.copy()
                while len(self._trail) > self.max_trail:
                    p.removeUserDebugItem(self._trail.popleft())
        elif self._prev_pos is None:
            self._prev_pos = pos.copy()
            self.trail_pts.append(pos.copy())

        if self.follow:
            cam = p.getDebugVisualizerCamera()
            yaw, pitch, dist = cam[8], cam[9], cam[10]
            self._cam_target += self.follow_lerp * (pos - self._cam_target)
            p.resetDebugVisualizerCamera(dist, yaw, pitch,
                                         self._cam_target.tolist())

        if self.text_on:
            ground = self.sim.terrain_height(pos[0], pos[1])
            speed = float(np.linalg.norm(s["lin_vel"]))
            lines = [
                f"PHASE  {phase.upper()}",
                f"POS    x{pos[0]:+6.2f}  y{pos[1]:+6.2f}  z{pos[2]:+6.2f} m",
                f"AGL    {pos[2] - ground:5.2f} m      SPEED {speed:5.2f} m/s",
                f"T      {s['t']:6.2f} s" + (f"      {rt_factor:.1f}x realtime"
                                             if rt_factor else ""),
            ]
            self._draw_text(lines, pos)

    def _draw_text(self, lines, anchor):
        """Text anchored above the drone, replaced in place each tick."""
        for i, line in enumerate(lines):
            where = [anchor[0], anchor[1], anchor[2] + 2.4 - 0.45 * i]
            if i < len(self._text_ids):
                self._text_ids[i] = p.addUserDebugText(
                    line, where, TEXT_COLOR, 1.1,
                    replaceItemUniqueId=self._text_ids[i])
            else:
                self._text_ids.append(
                    p.addUserDebugText(line, where, TEXT_COLOR, 1.1))

    def clear_trail(self):
        while self._trail:
            p.removeUserDebugItem(self._trail.popleft())
        self._prev_pos = None


def _project(pt, view, proj, width, height):
    """World point -> pixel. view/proj come back from PyBullet column-major."""
    V = np.array(view, dtype=float).reshape(4, 4, order="F")
    P = np.array(proj, dtype=float).reshape(4, 4, order="F")
    clip = P @ (V @ np.array([pt[0], pt[1], pt[2], 1.0]))
    if clip[3] <= 1e-6:
        return None
    ndc = clip[:3] / clip[3]
    if not (-1.2 < ndc[0] < 1.2 and -1.2 < ndc[1] < 1.2):
        return None
    return ((ndc[0] * 0.5 + 0.5) * width, (1.0 - (ndc[1] * 0.5 + 0.5)) * height)


def capture_frame(sim, path, width=960, height=600, distance=4.0,
                  yaw=40.0, pitch=-22.0, vis=None, phase="", rt_factor=None):
    """Crisp still, decoupled from live fps (V.6)."""
    from PIL import Image, ImageDraw
    s = sim.get_true_state()
    pos = s["pos"].tolist()
    view = p.computeViewMatrixFromYawPitchRoll(pos, distance, yaw, pitch, 0, 2)
    proj = p.computeProjectionMatrixFOV(60, width / height, 0.1, 250)
    img = p.getCameraImage(width, height, view, proj,
                           renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgb = np.reshape(np.array(img[2], dtype=np.uint8), (height, width, 4))[:, :, :3]
    im = Image.fromarray(rgb)
    d = ImageDraw.Draw(im)

    z0 = sim.terrain_height(0, 0)
    o = _project([0, 0, z0], view, proj, width, height)
    for vec, col in (((6, 0, 0), (255, 60, 60)), ((0, 6, 0), (60, 255, 60)),
                     ((0, 0, 6), (80, 140, 255))):
        e = _project([vec[0], vec[1], z0 + vec[2]], view, proj, width, height)
        if o and e:
            d.line([o, e], fill=col, width=3)

    if vis is not None and len(vis.trail_pts) > 1:
        pix = [_project(q, view, proj, width, height) for q in vis.trail_pts]
        pix = [q for q in pix if q]
        if len(pix) > 1:
            d.line(pix, fill=(0, 235, 255), width=3)

    ground = sim.terrain_height(pos[0], pos[1])
    speed = float(np.linalg.norm(s["lin_vel"]))
    lines = [f"PHASE  {phase.upper()}" if phase else "PHASE  --",
             f"POS    x{pos[0]:+6.2f}  y{pos[1]:+6.2f}  z{pos[2]:+6.2f} m",
             f"AGL    {pos[2] - ground:5.2f} m",
             f"SPEED  {speed:5.2f} m/s",
             f"T      {s['t']:6.2f} s" + (f"   {rt_factor:.1f}x realtime"
                                          if rt_factor else "")]
    d.rectangle([0, 0, 330, 14 + 15 * len(lines)], fill=(0, 0, 0))
    for i, line in enumerate(lines):
        d.text((8, 7 + 15 * i), line, fill=(255, 255, 255))
    im.save(path)
    return path
