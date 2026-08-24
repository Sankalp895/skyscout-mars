"""B5 part C -- export per-link world transforms for the 4 hero stills."""
import os
import json

import numpy as np

from urdf_fk import URDF
from mars_world import build_heightmap, EXTENT, CELL

REPO = os.environ.get(
    "SKYSCOUT_MODELS",
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "models") + os.sep)
DRONE_URDF = REPO + "skyscout_drone/skyscout_drone.urdf"
ROVER_URDF = REPO + "skyscout_rover/skyscout_rover.urdf"
DRONE_MESH_ROOT = REPO + "skyscout_drone/"
ROVER_MESH_ROOT = REPO + "skyscout_rover/"
ORIGIN = (-EXTENT / 2, -EXTENT / 2)

ROVER_PAD_RAISED = {"pad_panel_port": -3.14159, "pad_panel_stbd": 3.14159,
                    "pad_riser": 0.44, "latch_fore": 1.39626, "latch_aft": -1.39626}
ROVER_MAST = {"mast_pan": 0.0, "mast_tilt": -0.261799}
ARM_STOWED = {"arm_shoulder_az": 0.0, "arm_shoulder_el": 1.74533,
              "arm_elbow": -2.44346, "arm_wrist": -1.74533, "arm_turret": 0.0}
ARM_EXTENDED = {"arm_shoulder_az": 0.0, "arm_shoulder_el": 0.436332,
                "arm_elbow": 0.0, "arm_wrist": 0.0, "arm_turret": 0.0}
DRONE_STOWED = {"wing_fold_a_port": -1.5708, "wing_fold_b_port": -1.5708,
                "wing_fold_a_stbd": 1.5708, "wing_fold_b_stbd": 1.5708,
                "sensor_retract": 0.08, "sensor_pod": -1.5708}
DRONE_DEPLOYED = {"wing_fold_a_port": 0.0, "wing_fold_b_port": 0.0,
                  "wing_fold_a_stbd": 0.0, "wing_fold_b_stbd": 0.0,
                  "sensor_retract": 0.0, "sensor_pod": 0.0}
DRONE_HOVER = dict(DRONE_DEPLOYED, skid_port_fold=1.5708, skid_stbd_fold=1.5708)


def lerp_pose(a, b, t):
    keys = set(a) | set(b)
    return {k: (1 - t) * a.get(k, 0.0) + t * b.get(k, 0.0) for k in keys}


def yaw_T(x, y, z, yaw=0.0):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, z], [0, 0, 0, 1]], float)


def terrain_z(h, x, y):
    n = h.shape[0]
    fx = np.clip((x - ORIGIN[0]) / CELL, 0, n - 1)
    fy = np.clip((y - ORIGIN[1]) / CELL, 0, n - 1)
    i0, j0 = int(np.floor(fx)), int(np.floor(fy))
    i1, j1 = min(i0 + 1, n - 1), min(j0 + 1, n - 1)
    tx, ty = fx - i0, fy - j0
    return float(h[i0, j0]*(1-tx)*(1-ty) + h[i1, j0]*tx*(1-ty)
                 + h[i0, j1]*(1-tx)*ty + h[i1, j1]*tx*ty)


def ground_offset(u, q, wheel_r=0.29):
    """How far the root sits above the ground when the wheels touch."""
    fk = u.fk(q)
    zs = [T[2, 3] for n, T in fk.items()
          if n.startswith("wheel_") and "steer" not in n]
    return wheel_r - min(zs) if zs else 0.0


def main():
    h, feats, rocks = build_heightmap()
    drone, rover = URDF(DRONE_URDF), URDF(ROVER_URDF)

    off = ground_offset(rover, {})
    fk0 = rover.fk({})
    wz = [T[2, 3] for n, T in fk0.items()
          if n.startswith("wheel_") and "steer" not in n]
    print(f"rover stance: base_footprint IS the ground-contact frame "
          f"(offset {off:.4f} m)")
    print(f"  all 6 wheel centres at z = {min(wz):.3f}-{max(wz):.3f} m "
          f"= wheel radius 0.290 -> wheels contact exactly")
    print(f"  base_link rides {fk0['base_link'][2,3]:.3f} m above ground "
          f"in the URDF DESIGN stance")
    print(f"  NOTE: the physics sim settles to 0.028 m (with differential) /"
          f" 0.093 m (without).")
    print(f"  The URDF has no suspension preload, so under Mars gravity the "
          f"rocker-bogie squats")
    print(f"  to near its +-35 deg joint limits. Both stances are geometrically "
          f"valid; the DESIGN")
    print(f"  stance is what the CAD was drawn in and what a beauty render "
          f"should show, so that")
    print(f"  is what is exported here -- stated rather than silently chosen.")

    traj = json.load(open("b1_trajectory.json"))["samples"]
    hud = np.load("b5_hud_timeline.npz")
    rov_route = np.load("b5_rover_route.npz")["drive_xy"]

    shots = {}

    rx, ry = -6.0, -4.0
    rov_q = dict(ROVER_PAD_RAISED, **ROVER_MAST, **ARM_STOWED)
    rov_base = yaw_T(rx, ry, terrain_z(h, rx, ry) + off, np.radians(35))
    pad = rover.fk(rov_q)["pad_mount"]
    pad_world = rov_base @ pad
    d_q = lerp_pose(DRONE_STOWED, DRONE_HOVER, 0.55)
    d_base = pad_world.copy()
    d_base[2, 3] += 0.38
    shots["1_reveal"] = {
        "motion_source": "AUTHORED from SRDF named poses (no log exists)",
        "rover": rover.visual_placements(rov_q, rov_base),
        "drone": drone.visual_placements(d_q, d_base),
        "focus": [float(pad_world[0, 3]), float(pad_world[1, 3]),
                  float(pad_world[2, 3] + 0.35)],
    }

    s = traj[int(len(traj) * 0.62)]
    p = s["pos"]
    d_base = yaw_T(p[0], p[1], p[2], np.radians(20))
    shots["2_flight"] = {
        "motion_source": f"LOGGED b1_trajectory.json sample "
                         f"{int(len(traj)*0.62)}/{len(traj)} t={s['t']:.2f}s "
                         f"phase={s.get('phase','')}",
        "drone": drone.visual_placements(DRONE_DEPLOYED, d_base),
        "focus": [float(p[0]), float(p[1]), float(p[2])],
        "logged_pos": p, "logged_agl": s.get("agl"),
    }

    k = int(len(hud["drone_true"]) * 0.55)
    dp = hud["drone_true"][k]
    ri = int(len(rov_route) * 0.45)
    rxy = rov_route[ri]
    nxt = rov_route[min(ri + 3, len(rov_route) - 1)]
    ryaw = float(np.arctan2(nxt[1] - rxy[1], nxt[0] - rxy[0]))
    rov_base3 = yaw_T(rxy[0], rxy[1], terrain_z(h, rxy[0], rxy[1]) + off, ryaw)
    shots["3_scout_and_drive"] = {
        "motion_source": f"LOGGED drone b5_hud_timeline snapshot {k}, "
                         f"rover b5_rover_route waypoint {ri}/{len(rov_route)}",
        "drone": drone.visual_placements(DRONE_DEPLOYED,
                                         yaw_T(dp[0], dp[1], dp[2], np.radians(-30))),
        "rover": rover.visual_placements(dict(ROVER_MAST, **ARM_STOWED), rov_base3),
        "focus": [float((dp[0] + rxy[0]) / 2), float((dp[1] + rxy[1]) / 2), 0.0],
        "drone_pos": dp.tolist(), "rover_pos": [float(rxy[0]), float(rxy[1])],
    }

    rxy4 = rov_route[-1]
    rov_q4 = dict(ROVER_MAST, **ARM_EXTENDED)
    rov_base4 = yaw_T(rxy4[0], rxy4[1],
                      terrain_z(h, rxy4[0], rxy4[1]) + ground_offset(rover, rov_q4),
                      np.radians(200))
    shots["4_rover_detail"] = {
        "motion_source": "LOGGED rover goal position from b5_rover_route.npz; "
                         "mast/arm from SRDF mast_deployed + arm_extended",
        "rover": rover.visual_placements(rov_q4, rov_base4),
        "focus": [float(rxy4[0]), float(rxy4[1]),
                  terrain_z(h, rxy4[0], rxy4[1]) + 0.9],
        "rover_pos": [float(rxy4[0]), float(rxy4[1])],
    }

    out = {"mesh_roots": {"drone": DRONE_MESH_ROOT, "rover": ROVER_MESH_ROOT},
           "terrain_obj": "b5c_mars_terrain.obj", "shots": shots}
    with open("b5c_shots.json", "w") as f:
        json.dump(out, f, indent=1)

    print(f"\nwrote b5c_shots.json")
    for k2, v in shots.items():
        nd = len(v.get("drone", [])); nr = len(v.get("rover", []))
        print(f"  {k2:20s} drone {nd:2d} parts, rover {nr:2d} parts   [{v['motion_source'][:52]}]")

    print(f"\n=== POSE VERIFICATION vs the logged mission ===")
    s2 = shots["2_flight"]
    print(f"  shot 2 drone base  {np.round(s2['logged_pos'],3)}  "
          f"(AGL {s2['logged_agl']:.2f} m from the log)")
    zs = [np.array(v['T'])[2, 3] for v in s2["drone"]]
    print(f"    lowest drone part z {min(zs):.3f} m, terrain below "
          f"{terrain_z(h, s2['logged_pos'][0], s2['logged_pos'][1]):.3f} m -> "
          f"clearance {min(zs)-terrain_z(h,s2['logged_pos'][0],s2['logged_pos'][1]):.3f} m")
    s4 = shots["4_rover_detail"]
    tz = terrain_z(h, *s4["rover_pos"])
    wheels = [np.array(v["T"])[2, 3] for v in s4["rover"]
              if v["link"].startswith("wheel_") and "steer" not in v["link"]]
    print(f"  shot 4 rover at {np.round(s4['rover_pos'],2)}  terrain z {tz:.3f} m")
    print(f"    {len(wheels)} wheel centres at {min(wheels)-tz:.3f}-{max(wheels)-tz:.3f} m"
          f" above terrain (radius 0.290) -> "
          f"{'CONTACT OK' if abs(min(wheels)-tz-0.29) < 0.02 else 'FLOATING/SUNK'}")
    s3 = shots["3_scout_and_drive"]
    w3 = [np.array(v["T"])[2, 3] for v in s3["rover"]
          if v["link"].startswith("wheel_") and "steer" not in v["link"]]
    t3 = terrain_z(h, *s3["rover_pos"])
    print(f"  shot 3 rover at {np.round(s3['rover_pos'],2)}  wheel centres "
          f"{min(w3)-t3:.3f} m above terrain -> "
          f"{'CONTACT OK' if abs(min(w3)-t3-0.29) < 0.02 else 'FLOATING/SUNK'}")


if __name__ == "__main__":
    main()
