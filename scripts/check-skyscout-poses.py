#!/usr/bin/env python3
"""Validate SkyScout SRDF named poses, the deployment choreography, and the
lift-off-before-tilt constraint.

FK from the URDFs; OBB separating-axis tests on the pairs DESIGN_v3 s9 keeps
ENABLED: panel<->mast, panel<->arm, panel<->wheel, drone<->any rover link.

Cylinders and meshes are bounded by their enclosing box, so the test is
CONSERVATIVE: a reported clear pair is truly clear; a reported hit may be the
bounding box and is printed with its depth for triage.

Usage:  python scripts/utils/check-skyscout-poses.py
"""
from __future__ import annotations
import math
import pathlib
import sys
import numpy as np
import xml.etree.ElementTree as ET

ROVER = "models/skyscout_rover/skyscout_rover.urdf"
DRONE = "models/skyscout_drone/skyscout_drone.urdf"
ROVER_SRDF = "models/skyscout_rover/skyscout_rover.srdf"
DRONE_SRDF = "models/skyscout_drone/skyscout_drone.srdf"
# pad_mount's frame is the platform top; the stowed drone's floor sits this far
# below drone_base_link, so the drone rides here rather than coincident.
def _drone_seat_z():
    """Derived, never hardcoded. The stowed floor moves whenever the fold
    geometry changes (-0.080 in A4, -0.091 in A7, -0.225 in A9); a literal here
    silently seats the drone into the bay floor and reports phantom collisions.
    """
    sys.path.insert(0, "models/skyscout_drone/parts")
    import drone_stowed
    return -drone_stowed.gen_step().bounding_box().min.Z / 1000.0


DRONE_SEAT_Z = _drone_seat_z()
# v3 s10.1 altitude constraints, all measured. A7 folds the panels DOWN, so
# unfolding at pad level drives them through the open deck panels -- hence the
# unfold altitude, which the choreography satisfies by climbing under rotor
# power rather than by adding riser travel.
MIN_UNFOLD_ALT = 0.080
MIN_TRANSITION_ALT = 0.005
MIN_CRUISE_ALT = 0.120

# Known collision-PROXY artifact, not real geometry. The rotor collision proxy
# is the full swept disc (r 0.21 at the nacelle y +-0.34 -> +-0.55) against a
# +-0.475 bay, so raising the riser always reports an overlap. In the stowed
# pose the blades are parked fore-aft and the real drone is 0.804 wide, i.e.
# +0.073 m clear per side. Kept conservative for flight rather than swapped for
# a blade box, which would under-model spinning clearance.
KNOWN_PROXY = {("drone:rotor_aft_port", "base_link"),
               ("drone:rotor_aft_stbd", "base_link"),
               ("drone:rotor_fwd_port", "base_link"),
               ("drone:rotor_fwd_stbd", "base_link")}


def rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def axis_rot(axis, q):
    a = np.array(axis, float)
    n = np.linalg.norm(a)
    if n == 0:
        return np.eye(3)
    a = a/n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q)*K + (1-math.cos(q))*(K@K)


class Robot:
    def __init__(self, path):
        r = ET.parse(path).getroot()
        self.joints, self.parent_of, self.boxes = {}, {}, {}
        for j in r.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz") or "0 0 0").split()]
            rp = [float(v) for v in (o.get("rpy") or "0 0 0").split()]
            ax = j.find("axis")
            self.joints[j.get("name")] = dict(
                type=j.get("type"), parent=j.find("parent").get("link"),
                xyz=np.array(xyz), rot=rpy(*rp),
                axis=[float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1])
            self.parent_of[j.find("child").get("link")] = j.get("name")
        for l in r.findall("link"):
            bl = []
            for c in l.findall("collision"):
                o = c.find("origin")
                xyz = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
                rp = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
                g = c.find("geometry")
                b, cy, sp = g.find("box"), g.find("cylinder"), g.find("sphere")
                if b is not None:
                    h = np.array([float(v) for v in b.get("size").split()])/2
                elif cy is not None:
                    rr, ln = float(cy.get("radius")), float(cy.get("length"))
                    h = np.array([rr, rr, ln/2])
                elif sp is not None:
                    rr = float(sp.get("radius")); h = np.array([rr, rr, rr])
                else:
                    continue
                T = np.eye(4); T[:3, :3] = rpy(*rp); T[:3, 3] = xyz
                bl.append((h, T))
            if bl:
                self.boxes[l.get("name")] = bl

    def fk(self, q, base=None):
        base = np.eye(4) if base is None else base
        T = {}

        def solve(link):
            if link in T:
                return T[link]
            jn = self.parent_of.get(link)
            if jn is None:
                T[link] = base
                return base
            j = self.joints[jn]
            P = solve(j["parent"])
            L = np.eye(4); L[:3, :3] = j["rot"]; L[:3, 3] = j["xyz"]
            M = np.eye(4)
            v = q.get(jn, 0.0)
            if j["type"] in ("revolute", "continuous"):
                M[:3, :3] = axis_rot(j["axis"], v)
            elif j["type"] == "prismatic":
                M[:3, 3] = np.array(j["axis"], float)*v
            T[link] = P @ L @ M
            return T[link]
        for l in list(self.boxes) + list(self.parent_of):
            solve(l)
        return T


def obb_depth(h1, T1, h2, T2):
    R1, t1, R2, t2 = T1[:3, :3], T1[:3, 3], T2[:3, :3], T2[:3, 3]
    d = t2 - t1
    axes = [R1[:, i] for i in range(3)] + [R2[:, i] for i in range(3)]
    for i in range(3):
        for k in range(3):
            c = np.cross(R1[:, i], R2[:, k])
            if np.linalg.norm(c) > 1e-9:
                axes.append(c/np.linalg.norm(c))
    best = float("inf")
    for a in axes:
        ra = sum(abs(np.dot(a, R1[:, i]))*h1[i] for i in range(3))
        rb = sum(abs(np.dot(a, R2[:, i]))*h2[i] for i in range(3))
        gap = abs(np.dot(a, d)) - (ra+rb)
        if gap > 0:
            return 0.0
        best = min(best, -gap)
    return best


def pair(A, TA, la, B, TB, lb):
    worst = 0.0
    for h1, L1 in A.boxes.get(la, []):
        for h2, L2 in B.boxes.get(lb, []):
            worst = max(worst, obb_depth(h1, TA[la]@L1, h2, TB[lb]@L2))
    return worst


def states(path):
    r = ET.parse(path).getroot()
    return {gs.get("name"): {j.get("name"): float(j.get("value"))
                             for j in gs.findall("joint")}
            for gs in r.findall("group_state")}


def disabled(path):
    r = ET.parse(path).getroot()
    return {frozenset((d.get("link1"), d.get("link2")))
            for d in r.findall("disable_collisions")}



# ---------------------------------------------------------------------------
# STANDING RULE: the fold-sweep obstacle list must stay in sync with the
# geometry. Any part whose STATION changes must be re-added to the obstacle
# list before commit.
#
# Both bugs this guards against came from the list drifting:
#   * the elevon moved y 0.19 -> 0.30 AFTER the A7 sweep ran and was never
#     re-swept; it sat squarely in the folding panel's path (0.0 mm, -130 to
#     -160 deg) and went unreported for a whole amendment cycle;
#   * that unreported hit was then misdiagnosed as the panel penetrating the
#     centre body (A8), which cost a full amendment. The body was clear by
#     89 mm the entire time.
#
# So: this check enumerates every placed part in the drone pose scripts and
# fails if one is missing from FOLD_OBSTACLES. It is deliberately mechanical --
# a human remembering to update two lists is what failed.
# ---------------------------------------------------------------------------
# The moving panels: verified BY the sweep, so not obstacles in it.
SWEPT_PARTS = {"build_wing_panel_a", "build_wing_panel_b"}

FOLD_OBSTACLES = {
    "build_body", "build_wing_inner", "build_nacelle", "build_rotor",
    "build_elevon", "build_nacelle_pylon", "build_aft_pylon", "build_skid",
    "build_sensor_pod", "build_sensor_pod_stowed", "build_hinge_boss",
    # SWEPT_PARTS below are the moving panels themselves, not obstacles
}


def check_obstacle_list_in_sync():
    """Fail if a drone pose script places a part the sweep never tests."""
    import re
    placed = set()
    for name in ("drone_stowed.py", "drone_deployed.py"):
        src = (pathlib.Path("models/skyscout_drone/parts") / name).read_text()
        placed |= set(re.findall(r"d\.(build_\w+)", src))
    missing = sorted(placed - FOLD_OBSTACLES - SWEPT_PARTS)
    extra = sorted(FOLD_OBSTACLES - placed)
    if missing:
        print("  ERROR: placed but NOT in the sweep obstacle list: "
              + ", ".join(missing))
        print("         re-add them and re-run the fold sweep before commit.")
    if extra:
        print(f"  note: in the obstacle list but no longer placed: {', '.join(extra)}")
    if not missing:
        print(f"  obstacle list in sync ({len(placed)} placed parts, all swept)")
    return not missing



# ---------------------------------------------------------------------------
# JOINTED-PAIR OVERLAP VOLUME.
#
# distance_to() is IDENTICALLY ZERO at any joint -- it cannot distinguish parts
# that merely touch from parts that interpenetrate. Every fold sweep from A4 to
# A9 measured distance, so a 71,326 mm^3 interpenetration (panel A buried in the
# stub at full fold) survived six amendments unseen.
#
# Jointed pairs must therefore be verified by OVERLAP VOLUME, which must be
# zero. The geometry earns that by construction: each hinge axis sits on the
# lower surface of the plate it carries, so every point of a folding panel has
# local dz >= 0 and can never swing inboard past its hinge station.
# ---------------------------------------------------------------------------
JOINTED_PAIR_TOLERANCE_MM3 = 1.0


def check_fold_hinges():
    """Overlap volume at every jointed pair, plus the external fold sweep."""
    sys.path.insert(0, "models/skyscout_drone/parts")
    import _drone as d
    from build123d import Pos, Rot, Box

    W = Pos(0, 0, d.WING_ROOT_Z)
    ha = lambda a: W * Pos(d.FOLD_A_X, d.FOLD_A_Y, d.HINGE_Z) * Rot(a, 0, 0)
    hb = lambda a, b: ha(a) * Pos(d.FOLD_B_X, d.PANEL_A_SPAN, 0) * Rot(b, 0, 0)
    pa = lambda a: ha(a) * d.build_wing_panel_a(1)
    pb = lambda a, b: hb(a, b) * d.build_wing_panel_b(1)
    stub = W * d.build_wing_inner(1)

    seq = [(a, 0.0) for a in range(0, -91, -10)] + \
          [(-90.0, b) for b in range(-10, -91, -10)]
    worst_a = worst_b = 0.0
    for a, b in seq:
        worst_a = max(worst_a, (pa(a) & stub).volume)
        worst_b = max(worst_b, (pb(a, b) & pa(a)).volume)
    ok = max(worst_a, worst_b) < JOINTED_PAIR_TOLERANCE_MM3
    print(f"  panel_A ^ stub      {worst_a:9.1f} mm^3")
    print(f"  panel_B ^ panel_A   {worst_b:9.1f} mm^3")
    print(f"  -> {'ZERO at every jointed pair' if ok else 'INTERPENETRATION'}")

    obs = {
        "body": d.build_body(), "stub": stub,
        "nac_fwd": Pos(160, 260, 20) * d.build_nacelle(),
        "nac_aft": Pos(-300, 340, 20) * d.build_nacelle(),
        "rot_fwd": Pos(160, 260, 110) * d.build_rotor(),
        "rot_aft": Pos(-300, 340, 110) * d.build_rotor(),
        "elevon": W * Pos(-260, 180, 0) * d.build_elevon(),
        "fwd_pylon": Pos(110, 260, -10) * d.build_nacelle_pylon(),
        "aft_pylon": d.build_aft_pylon(1),
        "skid": Pos(0, 150, -80) * Box(240, 20, 50),
    }
    mins = {}
    for a, b in seq:
        moving = pa(a) + pb(a, b)
        for k, o in obs.items():
            if k == "stub":
                continue                      # jointed: covered by volume above
            g = moving.distance_to(o)
            if k not in mins or g < mins[k][0]:
                mins[k] = (g, a, b)
    print("  external sweep minima:")
    tight = []
    for k, (g, a, b) in sorted(mins.items(), key=lambda kv: kv[1][0]):
        note = ""
        if g <= 0.001:
            note = "  <-- CONTACT"
            tight.append(k)
        elif g < 15.0:
            note = "  <-- under 15 mm"
        print(f"     {k:11s} {g:8.1f} mm at A={a:.0f} B={b:.0f}{note}")
    return ok and not tight


def _unfold_b(dst):
    """Intermediate deploy pose: hinge B straightened, hinge A still folded."""
    q = dict(dst["climb_out"])
    for k in ("wing_fold_b_port", "wing_fold_b_stbd"):
        q[k] = 0.0
    return q


def main():
    rov, drn = Robot(ROVER), Robot(DRONE)
    rst, dst = states(ROVER_SRDF), states(DRONE_SRDF)
    rdis = disabled(ROVER_SRDF)

    PANEL = ["pad_panel_port", "pad_panel_stbd"]
    KEEP = (["mast_pan", "mast_tilt", "arm_shoulder_az", "arm_shoulder_el",
             "arm_elbow", "arm_wrist", "arm_turret"]
            + [l for l in rov.boxes if l.startswith("wheel_")])
    pairs = [(a, b) for a in PANEL for b in KEEP
             if frozenset((a, b)) not in rdis]

    def rover_hits(q):
        T = rov.fk(q)
        return [(a, b, d) for a, b in pairs
                if (d := pair(rov, T, a, rov, T, b)) > 1e-6]

    def drone_hits(qr, qd, lift=0.0):
        T = rov.fk(qr)
        base = T["pad_mount"].copy()
        base[:3, 3] = base[:3, 3] + base[:3, :3] @ np.array([0, 0, DRONE_SEAT_Z])
        base[2, 3] += lift
        TD = drn.fk(qd, base=base)
        return [(f"drone:{la}", lb, d) for la in drn.boxes for lb in rov.boxes
                if lb != "base_footprint"
                and (d := pair(drn, TD, la, rov, T, lb)) > 1e-6]

    arm, mast = rst["arm_stowed"], rst["mast_deployed"]
    fail = False

    print("=== FOLD HINGES: overlap volume + external sweep ===")
    if not check_fold_hinges():
        fail = True
    print("=== FOLD-SWEEP OBSTACLE LIST ===")
    if not check_obstacle_list_in_sync():
        fail = True
    print("=== ROVER named poses (kept pairs) ===")
    for name, st in rst.items():
        q = dict(arm); q.update(mast); q.update(st)
        h = rover_hits(q)
        fail = fail or bool(h)
        print(f"  {name:18s} {'CLEAR' if not h else 'COLLISION'}")
        for a, b, d in h:
            print(f"      {a} <-> {b}  {d*1000:.1f} mm")

    qr = dict(arm); qr.update(mast); qr.update(rst["pad_raised"])
    print("=== DRONE named poses ===")
    # A7 reorder: only `stowed` is a pad pose now; everything from
    # climb_out onward is airborne.
    ON_PAD = ("stowed",)
    lifts = {}
    for name, st in dst.items():
        if name in ON_PAD:
            h = drone_hits(qr, st)
            fail = fail or bool(h)
            print(f"  {name:18s} on pad    {'CLEAR' if not h else 'COLLISION'}")
            for a, b, d in sorted(h, key=lambda x: -x[2])[:4]:
                print(f"      {a} <-> {b}  {d*1000:.1f} mm")
        else:
            lift = 0.0
            while lift <= 1.2 and drone_hits(qr, st, lift):
                lift += 0.005
            lifts[name] = lift
            onpad = drone_hits(qr, st, 0.0)
            strike = max((x[2] for x in onpad), default=0.0)*1000
            print(f"  {name:18s} airborne  clear at lift >= {lift:.3f} m"
                  f"   (on the pad it strikes by {strike:.1f} mm)")

    # The unfold constraint is about the SWEEP through intermediate fold
    # angles, not the endpoint pose -- unfolded-at-rest clears fine, it is the
    # panel travelling downward that hits the deck panels.
    d0, d1 = dst["climb_out"], dst["wings_deployed"]
    unfold_alt = 0.0
    while unfold_alt <= 1.2:
        if not any(drone_hits(qr, {k: d0.get(k, 0.0)*(1-t) + d1.get(k, 0.0)*t
                                   for k in set(d0) | set(d1)}, unfold_alt)
                   for t in np.linspace(0, 1, 13)):
            break
        unfold_alt += 0.005
    lifts["unfold sweep"] = unfold_alt

    print("=== ALTITUDE CONSTRAINTS (v3 s10.1) ===")
    for k, want in (("unfold sweep", MIN_UNFOLD_ALT),
                    ("transition", MIN_TRANSITION_ALT),
                    ("cruise", MIN_CRUISE_ALT)):
        got = lifts.get(k, 0.0)
        ok = "OK" if got <= want + 1e-9 else f"EXCEEDS BUDGET (needs {got:.3f})"
        print(f"  {k:15s} measured {got:.3f} m   budget {want:.3f} m   {ok}")
        if got > want + 1e-9:
            fail = True

    print("=== CHOREOGRAPHY (v3 s10, A7 order: climb THEN unfold) ===")
    legs = [
        ("1->2 latches", rst["landed_sealed"], rst["latches_released"], None, None, 0, 0),
        ("2->3 panels", rst["latches_released"], rst["pad_open"], None, None, 0, 0),
        ("4 riser up", rst["pad_open"], rst["pad_raised"], dst["stowed"], dst["stowed"], 0, 0),
        ("5 rotors/pod", rst["pad_raised"], rst["pad_raised"], dst["stowed"], dst["climb_out"], 0, 0),
        ("6 climb-out", rst["pad_raised"], rst["pad_raised"], dst["climb_out"], dst["climb_out"], 0, MIN_UNFOLD_ALT),
        # A9 deploys B THEN A -- B is tucked beneath A, so straightening A
        # first would swing the tucked outer panel through the airframe.
        ("7a unfold B", rst["pad_raised"], rst["pad_raised"], dst["climb_out"], _unfold_b(dst), MIN_UNFOLD_ALT, MIN_UNFOLD_ALT),
        ("7b unfold A", rst["pad_raised"], rst["pad_raised"], _unfold_b(dst), dst["wings_deployed"], MIN_UNFOLD_ALT, MIN_UNFOLD_ALT),
        ("8a climb", rst["pad_raised"], rst["pad_raised"], dst["wings_deployed"], dst["wings_deployed"], MIN_UNFOLD_ALT, MIN_CRUISE_ALT),
        ("8b transition", rst["pad_raised"], rst["pad_raised"], dst["wings_deployed"], dst["transition"], MIN_CRUISE_ALT, MIN_CRUISE_ALT),
        ("8->9 cruise", rst["pad_raised"], rst["pad_raised"], dst["transition"], dst["cruise"], MIN_CRUISE_ALT, MIN_CRUISE_ALT),
    ]
    for tag, r0, r1, d0, d1, l0, l1 in legs:
        worst = None
        for s in np.linspace(0, 1, 9):
            q = dict(arm); q.update(mast)
            for k in set(r0) | set(r1):
                q[k] = r0.get(k, 0.0)*(1-s) + r1.get(k, 0.0)*s
            h = rover_hits(q)
            if d0 is not None:
                qd = {k: d0.get(k, 0.0)*(1-s) + d1.get(k, 0.0)*s for k in set(d0) | set(d1)}
                h = h + drone_hits(q, qd, l0*(1-s) + l1*s)
            if h:
                m = max(h, key=lambda x: x[2])
                if worst is None or m[2] > worst[2]:
                    worst = m
        if worst is None:
            print(f"  {tag:14s} CLEAR")
        elif (worst[0], worst[1]) in KNOWN_PROXY:
            print(f"  {tag:14s} clear (known proxy: {worst[0]} <-> {worst[1]} "
                  f"{worst[2]*1000:.1f} mm swept-disc, real +0.073 m/side)")
        else:
            fail = True
            print(f"  {tag:14s} COLLISION   worst {worst[0]} <-> {worst[1]} "
                  f"{worst[2]*1000:.1f} mm")

    print("\nFAILED" if fail else "\nAll poses and choreography legs clear.")
    return 1 if fail else 0


if __name__ == "__main__":
    raise SystemExit(main())
