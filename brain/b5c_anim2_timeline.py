"""B5C v2 -- the full scout mission: deploy, depart, SCAN, return, drive."""
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
N_ESTAB, N_DEPLOY, N_DEPART, N_SCOUT, N_RETURN, N_LAND, N_DRIVE = 48, 72, 60, 600, 60, 96, 264
WHEEL_R = 0.29
ROVER_XY = (-6.0, -4.0)
ROVER_YAW = np.radians(35)

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
ROTORS = ("rotor_fwd_port", "rotor_aft_port", "rotor_fwd_stbd", "rotor_aft_stbd")
WHEELS = ("drive_fl", "drive_ml", "drive_rl", "drive_fr", "drive_mr", "drive_rr")


def ramp(t, a, b):
    if b <= a:
        return 1.0 if t >= b else 0.0
    x = min(1.0, max(0.0, (t - a) / (b - a)))
    return x * x * (3 - 2 * x)


def lerp(a, b, t):
    return {k: (1 - t) * a.get(k, 0.0) + t * b.get(k, 0.0) for k in set(a) | set(b)}


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
    D, HT = hud["drone_true"], hud["t"]
    route = np.load("b5_rover_route.npz")["drive_xy"]

    rbase = yawT(ROVER_XY[0], ROVER_XY[1], tz(h, *ROVER_XY), ROVER_YAW)
    frames = []
    spin = 0.0

    est_q = dict(ROVER_MAST, **ARM_STOWED)
    for i in range(N_ESTAB):
        t = i / (N_ESTAB - 1)
        frames.append({
            "beat": "ESTABLISH", "label": "SKYSCOUT -- MARS SURFACE OPERATIONS",
            "prov": "AUTHORED", "source": "establishing shot, vehicle at rest",
            "rover": {"base": rbase, "q": est_q},
            "drone": None, "scan_idx": -1,
            "cam": {"focus": [ROVER_XY[0], ROVER_XY[1], tz(h, *ROVER_XY) + 1.2],
                    "off": [10.0 - 3.4 * t, -13.0 + 4.2 * t, 5.2 - 1.6 * t],
                    "lens": 40, "dof": False},
        })

    for i in range(N_DEPLOY):
        t = i / (N_DEPLOY - 1)
        open_, rise = ramp(t, 0.00, 0.34), ramp(t, 0.28, 0.56)
        climb, unfold = ramp(t, 0.52, 0.76), ramp(t, 0.70, 1.00)
        rq = dict(ROVER_MAST, **ARM_STOWED)
        rq.update({"pad_panel_port": -3.14159 * open_, "pad_panel_stbd": 3.14159 * open_,
                   "pad_riser": 0.44 * rise,
                   "latch_fore": 1.39626 * ramp(t, 0, 0.12),
                   "latch_aft": -1.39626 * ramp(t, 0, 0.12)})
        pad = np.array(rbase) @ rover.fk(rq)["pad_mount"]
        spin += 38.0 / FPS * ramp(t, 0.40, 0.60)
        dq = lerp(D_STOWED, D_HOVER, unfold)
        for r_ in ROTORS:
            dq[r_] = spin
        db = pad.copy(); db[2, 3] += 0.02 + 0.60 * climb
        frames.append({
            "beat": "DEPLOY", "label": "DRONE DEPLOY", "prov": "AUTHORED",
            "source": "SRDF named poses (no log holds a joint angle)",
            "rover": {"base": rbase, "q": rq},
            "drone": {"base": db.tolist(), "q": dq},
            "scan_idx": -1,
            "cam": {"focus": [float(pad[0, 3]), float(pad[1, 3]),
                              float(pad[2, 3]) + 0.30 + 0.40 * climb],
                    "off": [4.9 - 1.0 * t, -6.2 + 1.2 * t, 2.9 - 0.4 * t],
                    "lens": 42, "dof": True},
        })

    p0 = np.array([frames[-1]["drone"]["base"][0][3],
                   frames[-1]["drone"]["base"][1][3],
                   frames[-1]["drone"]["base"][2][3]])
    p1 = D[0].copy()
    for i in range(N_DEPART):
        t = i / (N_DEPART - 1)
        e = ramp(t, 0.0, 1.0)
        p = p0 + (p1 - p0) * e
        p[2] += 3.0 * np.sin(np.pi * e) * 0.35
        yaw = float(np.arctan2(p1[1] - p0[1], p1[0] - p0[0]))
        spin += 46.0 / FPS
        dq = dict(D_HOVER)
        for r_ in ROTORS:
            dq[r_] = spin
        frames.append({
            "beat": "DEPART", "label": "OUTBOUND TO SURVEY AREA", "prov": "AUTHORED",
            "source": "transit leg, no log covers the ferry",
            "rover": {"base": rbase, "q": dict(ROVER_MAST, **ARM_STOWED,
                                               pad_panel_port=-3.14159,
                                               pad_panel_stbd=3.14159, pad_riser=0.44)},
            "drone": {"base": yawT(p[0], p[1], p[2], yaw), "q": dq},
            "scan_idx": -1,
            "cam": {"focus": [float(p[0]), float(p[1]), float(p[2])],
                    "off": [9.0 + 10.0 * e, -12.0 - 6.0 * e, 4.0 + 4.0 * e],
                    "lens": 38, "dof": False},
        })

    ks = np.linspace(0, len(D) - 1, N_SCOUT)
    speed_up = float(HT[-1] - HT[0]) / (N_SCOUT / FPS)
    for i, kf in enumerate(ks):
        j = int(np.floor(kf)); f = kf - j
        j2 = min(j + 1, len(D) - 1)
        p = D[j] * (1 - f) + D[j2] * f
        nxt = D[min(j2 + 1, len(D) - 1)]
        yaw = float(np.arctan2(nxt[1] - D[j][1], nxt[0] - D[j][0]))
        spin += 46.0 / FPS
        dq = dict(D_HOVER)
        for r_ in ROTORS:
            dq[r_] = spin
        out = np.array([p[0], p[1]]) - np.array([-8.0, 6.0])
        n_ = np.linalg.norm(out)
        out = out / n_ if n_ > 1e-6 else np.array([1.0, 0.0])
        sw = np.radians(45.0) * np.sin(2 * np.pi * (i / N_SCOUT) * 1.5)
        c_, s_ = np.cos(sw), np.sin(sw)
        d_ = np.array([out[0] * c_ - out[1] * s_, out[0] * s_ + out[1] * c_])
        frames.append({
            "beat": "SCOUT", "label": "SCANNING TERRAIN", "prov": "LOGGED",
            "source": f"recorded survey, {speed_up:.1f}x speed",
            "rover": None,
            "drone": {"base": yawT(p[0], p[1], p[2], yaw), "q": dq},
            "scan_idx": int(round(kf)),
            "cam": {"focus": [float(p[0]), float(p[1]), float(p[2]) - 1.5],
                    "off": [float(d_[0] * 10.0), float(d_[1] * 10.0), 5.0],
                    "lens": 34, "dof": False},
        })

    rq_open = dict(ROVER_MAST, **ARM_STOWED, pad_panel_port=-3.14159,
                   pad_panel_stbd=3.14159, pad_riser=0.44)
    pad_open = np.array(rbase) @ rover.fk(rq_open)["pad_mount"]
    HOVER_H = 2.4
    q0 = D[-1].copy()
    q1 = np.array([pad_open[0, 3], pad_open[1, 3], pad_open[2, 3] + HOVER_H])
    for i in range(N_RETURN):
        t = i / (N_RETURN - 1)
        e = ramp(t, 0.0, 1.0)
        p = q0 + (q1 - q0) * e
        yaw = float(np.arctan2(q1[1] - q0[1], q1[0] - q0[0]))
        spin += 46.0 / FPS
        dq = dict(D_HOVER)
        for r_ in ROTORS:
            dq[r_] = spin
        frames.append({
            "beat": "RETURN", "label": "RETURNING TO ROVER", "prov": "AUTHORED",
            "source": "transit leg, no log covers the return",
            "rover": {"base": rbase, "q": dict(ROVER_MAST, **ARM_STOWED,
                                               pad_panel_port=-3.14159,
                                               pad_panel_stbd=3.14159, pad_riser=0.44)},
            "drone": {"base": yawT(p[0], p[1], p[2], yaw), "q": dq},
            "scan_idx": len(D) - 1,
            "cam": {"focus": [float(p[0] * (1 - e) + ROVER_XY[0] * e),
                              float(p[1] * (1 - e) + ROVER_XY[1] * e),
                              float(p[2] * (1 - e) + (tz(h, *ROVER_XY) + 1.6) * e)],
                    "off": [7.0 + 4.0 * (1 - e), -9.0 - 4.0 * (1 - e), 4.0 + 3.0 * (1 - e)],
                    "lens": 40, "dof": False},
        })

    for i in range(N_LAND):
        t = i / (N_LAND - 1)
        touch = ramp(t, 0.00, 0.34)
        fold = ramp(t, 0.30, 0.62)
        sink = ramp(t, 0.56, 0.82)
        close = ramp(t, 0.76, 1.00)
        rq = dict(ROVER_MAST, **ARM_STOWED)
        rq.update({"pad_panel_port": -3.14159 * (1 - close),
                   "pad_panel_stbd": 3.14159 * (1 - close),
                   "pad_riser": 0.44 * (1 - sink),
                   "latch_fore": 1.39626 * (1 - ramp(t, 0.88, 1.0)),
                   "latch_aft": -1.39626 * (1 - ramp(t, 0.88, 1.0))})
        pad = np.array(rbase) @ rover.fk(rq)["pad_mount"]
        spin += (46.0 / FPS) * (1.0 - ramp(t, 0.10, 0.55))
        dq = lerp(D_HOVER, D_STOWED, fold)
        for r_ in ROTORS:
            dq[r_] = spin
        db = pad.copy()
        db[2, 3] += HOVER_H * (1 - touch) + 0.02
        frames.append({
            "beat": "LAND", "label": "DRONE STOWING INTO BAY", "prov": "AUTHORED",
            "source": "SRDF named poses, reverse of the deploy sequence",
            "rover": {"base": rbase, "q": rq},
            "drone": {"base": db.tolist(), "q": dq},
            "scan_idx": len(D) - 1,
            "cam": {"focus": [float(pad[0, 3]), float(pad[1, 3]),
                              float(pad[2, 3]) + 0.55 * (1 - close)],
                    "off": [4.4 + 1.6 * close, -5.6 - 1.8 * close, 2.5 + 0.9 * close],
                    "lens": 44, "dof": True},
        })

    a0, b0 = 0, len(route) - 1
    seg = route[a0:b0 + 1]
    idx = np.linspace(0, len(seg) - 1, N_DRIVE)
    travelled, prev = 0.0, seg[0]
    for i, ii in enumerate(idx):
        j = int(np.floor(ii)); f = ii - j
        j2 = min(j + 1, len(seg) - 1)
        xy = seg[j] * (1 - f) + seg[j2] * f
        travelled += float(np.linalg.norm(xy - prev)); prev = xy
        nx = seg[min(j2 + 1, len(seg) - 1)]
        yaw = float(np.arctan2(nx[1] - seg[j][1], nx[0] - seg[j][0]))
        rq = dict(ROVER_MAST, **ARM_STOWED)
        for w in WHEELS:
            rq[w] = travelled / WHEEL_R
        frames.append({
            "beat": "DRIVE", "label": "ROVER DRIVING SCOUTED ROUTE", "prov": "LOGGED",
            "source": f"A* route over the perceived map, waypoints {a0}-{b0}",
            "rover": {"base": yawT(xy[0], xy[1], tz(h, xy[0], xy[1]), yaw), "q": rq},
            "drone": None,
            "scan_idx": len(D) - 1,
            "cam": {"focus": [float(xy[0]), float(xy[1]), tz(h, xy[0], xy[1]) + 0.9],
                    "off": [5.4, -6.6, 2.7 + 0.9 * (i / N_DRIVE)],
                    "lens": 44, "dof": True},
        })

    out = {"fps": FPS, "n_frames": len(frames),
           "mesh_roots": {"drone": REPO + "skyscout_drone/",
                          "rover": REPO + "skyscout_rover/"},
           "terrain_obj": "b5c_mars_terrain.obj",
           "urdf": {"drone": REPO + "skyscout_drone/skyscout_drone.urdf",
                    "rover": REPO + "skyscout_rover/skyscout_rover.urdf"},
           "frames": frames}
    with open("b5c_anim2.json", "w") as f:
        json.dump(out, f)

    print(f"wrote b5c_anim2.json: {len(frames)} frames @ {FPS} fps = {len(frames)/FPS:.1f} s")
    for nm, n in (("ESTABLISH", N_ESTAB), ("DEPLOY", N_DEPLOY), ("DEPART", N_DEPART),
                  ("SCOUT", N_SCOUT), ("RETURN", N_RETURN), ("LAND", N_LAND),
                  ("DRIVE", N_DRIVE)):
        prov = "LOGGED" if nm in ("SCOUT", "DRIVE") else "AUTHORED"
        print(f"  {nm:7s} {n:3d} fr ({n/FPS:4.1f} s)  {prov}")
    print(f"  scout compresses {HT[-1]-HT[0]:.0f} s of survey into "
          f"{N_SCOUT/FPS:.0f} s  ({speed_up:.1f}x), coverage 0 -> 100%")


if __name__ == "__main__":
    main()
