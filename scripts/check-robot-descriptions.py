#!/usr/bin/env python3
"""Pre-export integrity check for URDF and SDF robot descriptions.

Run this BEFORE and AFTER every URDF/SDF export. It exists because three
separate faults shipped past both the bundled SDF validator and a "does it
parse?" eyeball in a single session:

  1. mesh URIs using a scheme the consumer could not resolve (model://);
  2. a mesh URI pointing at an .stl that was never modelled -- masked, because
     an unresolvable scheme is silently skipped by the existence check;
  3. link/joint name collisions, legal in URDF (separate namespaces) but
     invalid in SDF (frames, links and joints share one namespace).

Parsing cleanly proves almost nothing; each of those parsed fine.

Usage:
    python scripts/utils/check-robot-descriptions.py [paths ...]

With no paths it checks every .urdf/.sdf under models/. Exits non-zero if any
ERROR is found. Warnings do not fail the run.
"""
from __future__ import annotations

import collections
import os
import sys
import xml.etree.ElementTree as ET

# Element tags whose TEXT names a joint. Gazebo plugins use assorted spellings.
JOINT_REF_TAGS = {
    "joint_name", "left_joint", "right_joint", "front_joint", "back_joint",
    "parent_joint", "child_joint", "gearbox_reference_joint",
}
# URI schemes a typical consumer can resolve. Anything else is a hard error:
# CAD Viewer rejects unknown schemes outright, and model:// silently requires
# the model to be installed on the simulator resource path.
OK_SCHEMES = ("package://",)


class Report:
    def __init__(self, path):
        self.path, self.errors, self.warnings, self.info = path, [], [], []

    def error(self, msg):
        self.errors.append(msg)

    def warn(self, msg):
        self.warnings.append(msg)

    def note(self, msg):
        self.info.append(msg)


def _dups(names):
    return sorted(n for n, c in collections.Counter(names).items() if c > 1)


def _check_meshes(root, base, rep, kind):
    """Every mesh reference must use a resolvable scheme AND exist on disk."""
    refs = []
    if kind == "sdf":
        refs = [(u.text or "").strip() for u in root.iter("uri")]
    else:
        refs = [(m.get("filename") or "").strip() for m in root.iter("mesh")]
    for ref in refs:
        if not ref:
            rep.error("empty mesh reference")
            continue
        if "://" in ref:
            if not ref.startswith(OK_SCHEMES):
                rep.error(f"unresolvable mesh URI scheme: {ref}")
                continue
            rep.warn(f"mesh URI uses a scheme, not a relative path: {ref}")
            continue
        if not os.path.exists(os.path.join(base, ref)):
            rep.error(f"missing mesh file: {ref}")
    rep.note(f"{len(refs)} mesh refs, {len(set(refs))} unique")


def check_urdf(path, rep):
    root = ET.parse(path).getroot()
    links = [l.get("name") for l in root.findall("link")]
    joints = [j.get("name") for j in root.findall("joint")]
    for label, names in (("link", links), ("joint", joints)):
        d = _dups(names)
        if d:
            rep.error(f"duplicate {label} names: {', '.join(d)}")

    lset = set(links)
    parent_of = {}
    for j in root.findall("joint"):
        for tag in ("parent", "child"):
            el = j.find(tag)
            if el is None or el.get("link") not in lset:
                rep.error(f"joint {j.get('name')!r} has a dangling {tag}: "
                          f"{None if el is None else el.get('link')}")
        c = j.find("child")
        if c is not None and c.get("link") in parent_of:
            rep.error(f"link {c.get('link')!r} has more than one parent joint")
        if c is not None:
            parent_of[c.get("link")] = j.get("name")
        if j.get("type") in ("revolute", "prismatic") and j.find("limit") is None:
            rep.error(f"{j.get('type')} joint {j.get('name')!r} has no <limit>")

    roots = [l for l in links if l not in parent_of]
    if len(roots) != 1:
        rep.error(f"expected exactly 1 kinematic root, found {len(roots)}: {roots}")

    # Names that are legal here but WILL collide if this is exported to SDF.
    shared = sorted(set(joints) & lset)
    if shared:
        rep.warn(f"{len(shared)} joint names also name a link; legal in URDF but "
                 f"they must be renamed on SDF export: {', '.join(shared[:6])}"
                 + (" ..." if len(shared) > 6 else ""))

    _check_meshes(root, os.path.dirname(path), rep, "urdf")
    rep.note(f"links {len(links)}, joints {len(joints)}, root {roots[0] if len(roots)==1 else '?'}")


def check_sdf(path, rep):
    root = ET.parse(path).getroot()
    if not (root.tag == "sdf" and root.get("version")):
        rep.error("root must be <sdf> with a version attribute")
    for model in root.findall("model"):
        links = [l.get("name") for l in model.findall("link")]
        joints = [j.get("name") for j in model.findall("joint")]
        frames = [f.get("name") for f in model.findall("frame")]
        # THE big one: SDF shares one namespace across frames, links and joints.
        d = _dups(links + joints + frames)
        if d:
            rep.error(f"duplicate frame/link/joint names (SDF shares one "
                      f"namespace): {', '.join(d)}")

        lset, jset = set(links), set(joints)
        fset = lset | set(frames)
        for j in model.findall("joint"):
            for tag in ("parent", "child"):
                el = j.find(tag)
                name = (el.text or "").strip() if el is not None else None
                if name not in lset and name != "world":
                    rep.error(f"joint {j.get('name')!r} {tag} names "
                              f"{name!r}, which is not a link")
            for ax in list(j.findall("axis")) + list(j.findall("axis2")):
                xyz = ax.find("xyz")
                exp = xyz.get("expressed_in") if xyz is not None else None
                if exp and exp not in fset:
                    rep.error(f"joint {j.get('name')!r} axis expressed_in "
                              f"{exp!r}, which is not a link or frame")
            grb = j.find("gearbox_reference_body")
            if grb is not None and (grb.text or "").strip() not in lset:
                rep.error(f"joint {j.get('name')!r} gearbox_reference_body "
                          f"{grb.text!r} is not a link")

        for f in model.findall("frame"):
            att = f.get("attached_to")
            if att and att not in fset:
                rep.error(f"frame {f.get('name')!r} attached_to {att!r}, "
                          "which is not a link or frame")

        for el in model.iter():
            if el.tag in JOINT_REF_TAGS:
                name = (el.text or "").strip()
                if name and name not in jset:
                    rep.error(f"<{el.tag}> names {name!r}, which is not a joint "
                              "in this model (stale after a joint rename?)")

        for p in model.iter("pose"):
            rel = p.get("relative_to")
            if rel and rel not in fset:
                rep.error(f"pose relative_to {rel!r} is not a link or frame")

        plugins = len(list(model.iter("plugin")))
        sensors = len(list(model.iter("sensor")))
        with_inertial = sum(1 for l in model.findall("link")
                            if l.find("inertial") is not None)
        rep.note(f"model {model.get('name')!r}: links {len(links)}, joints "
                 f"{len(joints)}, frames {len(frames)}, inertials "
                 f"{with_inertial}, plugins {plugins}, sensors {sensors}")

    _check_meshes(root, os.path.dirname(path), rep, "sdf")


def main(argv):
    paths = argv[1:]
    if not paths:
        for dirpath, _, files in os.walk("models"):
            for fn in sorted(files):
                if fn.endswith((".urdf", ".sdf")):
                    paths.append(os.path.join(dirpath, fn))
    if not paths:
        print("no .urdf/.sdf files found")
        return 0

    failed = False
    for path in paths:
        rep = Report(path)
        try:
            (check_sdf if path.endswith(".sdf") else check_urdf)(path, rep)
        except ET.ParseError as exc:
            rep.error(f"XML parse failed: {exc}")
        status = "FAIL" if rep.errors else ("warn" if rep.warnings else "ok")
        print(f"[{status:4}] {path}")
        for m in rep.info:
            print(f"         {m}")
        for m in rep.warnings:
            print(f"  warn:  {m}")
        for m in rep.errors:
            print(f"  ERROR: {m}")
        failed = failed or bool(rep.errors)
    print("\nFAILED" if failed else "\nAll robot descriptions passed.")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
