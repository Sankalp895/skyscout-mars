"""B5C animation -- build a per-frame pose timeline from the RECORDED logs."""
import os
import json
import numpy as np

from urdf_fk import URDF
from mars_world import build_heightmap, EXTENT, CELL

REPO = os.environ.get(
    "SKYSCOUT_MODELS",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "models") + os.sep)
ORIGIN = (-EXTENT / 2, -EXTENT / 2)
FPS = 24
BEATS = [("reveal", 72), ("flight", 120), ("drive", 120)]
WHEEL_R = 0.29

ROVER_MAST = {"mast_pan": 0.0, "mast_tilt": -0.261799}
ARM_STOWED = {"arm_shoulder_az": 0.0, "arm_shoulder_el": 1.74533,
              "arm_elbow": -2.44346, "arm_wrist": -1.74533, "arm_turret": 0.0}
D_STOWED = {"wing_fold_a_port": -1.5708, "wing_fold_b_port": -1.5708,
            "wing_fold_a_stbd": 1.5708, "wing_fold_b_stbd": 1.5708,
            "sensor_retract": 0.08, "sensor_pod": -1.5708,
            "skid_port_fold": 0.0, "skid_stbd_fold": 0.0}
D_HOVER = {"wing_fold_a_port": 0.0, "wing_fold_b_port": 0.0,
           "wing_fold_a_stbd": 0.0, "wing_fold_b_stbd": 0.0,
           "sensor_retract": 0.0, "sensor_pod": 0.0,
           "skid_port_fold": 1.5708, "skid_stbd_fold": 1.5708}


def ramp(t, a, b):
    """Smoothstep from 0 at t=a to 1 at t=b."""
    if b <= a:
        return 1.0 if t >= b else 0.0
    x = min(1.0, max(0.0, (t - a) / (b - a)))
    return x * x * (3 - 2 * x)


def lerp(a, b, t):
    keys = set(a) | set(b)
    return {k: (1 - t) * a.get(k, 0.0) + t * b.get(k, 0.0) for k in keys}


def yawT(x, y, z, yaw=0.0):
    c, s = np.cos(yaw), np.sin(yaw)
    return [[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1]]


def tz(h, x, y):
    n = h.shape[0]
    fx = np.clip((x - ORIGIN[0]) / CELL, 0, n - 1)
    fy = np.clip((y - ORIGIN[1]) / CELL, 0, n - 1)
    i0, j0 = int(np.floor(fx)), int(np.floor(fy))
    i1, j1 = min(i0 + 1, n - 1), min(j0 + 1, n - 1)
    a, b = fx - i0, fy - j0
    return float(h[i0, j0]*(1-a)*(1-b) + h[i1, j0]*a*(1-b)
                 + h[i0, j1]*(1-a)*b + h[i1, j1]*a*b)


def main():
    h, _, _ = build_heightmap()
    rover = URDF(REPO + "skyscout_rover/skyscout_rover.urdf")
    hud = np.load("b5_hud_timeline.npz")
    route = np.load("b5_rover_route.npz")["drive_xy"]
    D = hud["drone_true"]; HT = hud["t"]

    frames = []
    n_rev, n_fly, n_drv = [n for _, n in BEATS]

    rx, ry = -6.0, -4.0
    rbase = yawT(rx, ry, tz(h, rx, ry), np.radians(35))
    pad_static = rover.fk(dict(ROVER_MAST, **ARM_STOWED))["pad_mount"]
    for i in range(n_rev):
        t = i / (n_rev - 1)
        open_ = ramp(t, 0.00, 0.34)
        rise = ramp(t, 0.28, 0.56)
        climb = ramp(t, 0.52, 0.74)
        unfold = ramp(t, 0.70, 1.00)
        rq = dict(ROVER_MAST, **ARM_STOWED)
        rq.update({"pad_panel_port": -3.14159 * open_,
                   "pad_panel_stbd": 3.14159 * open_,
                   "pad_riser": 0.44 * rise,
                   "latch_fore": 1.39626 * ramp(t, 0.0, 0.12),
                   "latch_aft": -1.39626 * ramp(t, 0.0, 0.12)})
        pad = np.array(rbase) @ rover.fk(rq)["pad_mount"]
        dq = lerp(D_STOWED, D_HOVER, unfold)
        spin = 38.0 * (i / FPS) * ramp(t, 0.40, 0.60)
        for r_ in ("rotor_fwd_port", "rotor_aft_port", "rotor_fwd_stbd", "rotor_aft_stbd"):
            dq[r_] = spin
        db = pad.copy()
        db[2, 3] += 0.02 + 0.55 * climb
        frames.append({
            "beat": "reveal", "label": "DECK OPEN -- DRONE DEPLOY",
            "source": "AUTHORED from SRDF named poses",
            "rover": {"base": rbase, "q": rq},
            "drone": {"base": db.tolist(), "q": dq},
            "cam": {"focus": [float(pad[0, 3]), float(pad[1, 3]),
                              float(pad[2, 3]) + 0.30 + 0.35 * climb],
                    "off": [4.9 - 1.1 * t, -6.2 + 1.3 * t, 2.9 - 0.5 * t],
                    "lens": 42, "dof": True},
        })

    d_c = np.hypot(D[:, 0] + 15.0, D[:, 1] - 10.0)
    k0 = max(0, int(np.argmin(d_c)) - 6)
    k1 = min(len(D) - 1, k0 + 13)
    span_s = float(HT[k1] - HT[k0])
    speed_up = span_s / (n_fly / FPS)
    ks = np.linspace(k0, k1, n_fly)
    for i, kf in enumerate(ks):
        j = int(np.floor(kf)); f = kf - j
        j2 = min(j + 1, len(D) - 1)
        p = D[j] * (1 - f) + D[j2] * f
        nxt = D[min(j2 + 1, len(D) - 1)]
        yaw = float(np.arctan2(nxt[1] - D[j][1], nxt[0] - D[j][0]))
        dq = dict(D_HOVER)
        spin = 46.0 * (i / FPS)
        for r_ in ("rotor_fwd_port", "rotor_aft_port", "rotor_fwd_stbd", "rotor_aft_stbd"):
            dq[r_] = spin
        frames.append({
            "beat": "flight", "label": "SURVEY -- MAPPING THE CRATER",
            "source": f"LOGGED survey track (b5_hud_timeline), {speed_up:.1f}x speed",
            "rover": None,
            "drone": {"base": yawT(p[0], p[1], p[2], yaw), "q": dq},
            "cam": {"focus": [float(p[0]), float(p[1]), float(p[2])],
                    "off": [-7.5 + 3.0 * (i / n_fly), -9.0, 3.0 + 1.5 * (i / n_fly)],
                    "lens": 40, "dof": False},
        })

    a, b = 30, 90
    seg = route[a:b + 1]
    seglen = float(np.sum(np.linalg.norm(np.diff(seg, axis=0), axis=1)))
    idx = np.linspace(0, len(seg) - 1, n_drv)
    travelled = 0.0
    prev = seg[0]
    for i, ii in enumerate(idx):
        j = int(np.floor(ii)); f = ii - j
        j2 = min(j + 1, len(seg) - 1)
        xy = seg[j] * (1 - f) + seg[j2] * f
        travelled += float(np.linalg.norm(xy - prev)); prev = xy
        nx = seg[min(j2 + 1, len(seg) - 1)]
        yaw = float(np.arctan2(nx[1] - seg[j][1], nx[0] - seg[j][0]))
        rq = dict(ROVER_MAST, **ARM_STOWED)
        wa = travelled / WHEEL_R
        for w in ("drive_fl", "drive_ml", "drive_rl", "drive_fr", "drive_mr", "drive_rr"):
            rq[w] = wa
        frames.append({
            "beat": "drive", "label": "ROVER DRIVING THE SCOUTED ROUTE",
            "source": f"LOGGED route (b5_rover_route), waypoints {a}-{b}",
            "rover": {"base": yawT(xy[0], xy[1], tz(h, xy[0], xy[1]), yaw), "q": rq},
            "drone": None,
            "cam": {"focus": [float(xy[0]), float(xy[1]), tz(h, xy[0], xy[1]) + 0.9],
                    "off": [5.2, -6.4, 2.6 + 0.8 * (i / n_drv)],
                    "lens": 44, "dof": True},
        })

    out = {"fps": FPS, "n_frames": len(frames),
           "mesh_roots": {"drone": REPO + "skyscout_drone/",
                          "rover": REPO + "skyscout_rover/"},
           "terrain_obj": "b5c_mars_terrain.obj",
           "urdf": {"drone": REPO + "skyscout_drone/skyscout_drone.urdf",
                    "rover": REPO + "skyscout_rover/skyscout_rover.urdf"},
           "frames": frames}
    with open("b5c_anim.json", "w") as f:
        json.dump(out, f)

    print(f"wrote b5c_anim.json: {len(frames)} frames @ {FPS} fps = "
          f"{len(frames)/FPS:.1f} s")
    for nm, n in BEATS:
        print(f"  {nm:8s} {n:3d} frames ({n/FPS:.1f} s)")
    print(f"  flight beat: HUD snapshots {k0}-{k1} spanning {span_s:.1f} s of "
          f"mission -> {speed_up:.1f}x speed-up")
    print(f"  drive beat : route waypoints {a}-{b}, {seglen:.1f} m of the 85 m route")
    print(f"  closest approach to crater centre in the flight beat: "
          f"{d_c[k0:k1+1].min():.1f} m")


if __name__ == "__main__":
    main()
