"""Minimal URDF parser + forward kinematics, for the B5C beauty pass.

Deliberately not Genesis: it merges links joined by fixed joints (19 vs the
URDF's 21 for the drone, 27 vs 34 for the rover), which is right for physics
and drops visual meshes for rendering.
"""
import re
import numpy as np


def _rpy(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp,   cp*sr,            cp*cr]])


def _T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_angle(axis, q):
    a = np.asarray(axis, float)
    n = np.linalg.norm(a)
    if n < 1e-12:
        return np.eye(3)
    a = a / n
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q)*K + (1-np.cos(q))*(K @ K)


class URDF:
    def __init__(self, path):
        s = open(path).read()
        self.links = {}
        # self-closing <link/> tags matter: the rover's root (base_footprint) is one
        for m in re.finditer(r'<link name="([^"]+)"\s*/>', s):
            self.links[m.group(1)] = {"mesh": None, "prim": None, "origin": np.eye(4), "scale": 1.0}
        for m in re.finditer(r'<link name="([^"]+)"\s*>(.*?)</link>', s, re.S):
            name, body = m.group(1), m.group(2)
            vis = re.search(r'<visual>(.*?)</visual>', body, re.S)
            entry = {"mesh": None, "prim": None, "origin": np.eye(4), "scale": 1.0}
            if vis:
                v = vis.group(1)
                fn = re.search(r'filename="([^"]+)"', v)
                if fn:
                    entry["mesh"] = fn.group(1)
                else:
                    g = re.search(r'<box size="([^"]+)"', v)
                    if g:
                        entry["prim"] = ("box", [float(x) for x in g.group(1).split()])
                    g = re.search(r'<cylinder radius="([^"]+)" length="([^"]+)"', v)
                    if g:
                        entry["prim"] = ("cylinder", [float(g.group(1)), float(g.group(2))])
                    g = re.search(r'<sphere radius="([^"]+)"', v)
                    if g:
                        entry["prim"] = ("sphere", [float(g.group(1))])
                sc = re.search(r'scale="([^"]+)"', v)
                if sc:
                    entry["scale"] = float(sc.group(1).split()[0])
                o = re.search(r'<origin([^>]*)/>', v)
                if o:
                    xyz = re.search(r'xyz="([^"]+)"', o.group(1))
                    rpy = re.search(r'rpy="([^"]+)"', o.group(1))
                    entry["origin"] = _T(
                        [float(x) for x in xyz.group(1).split()] if xyz else [0, 0, 0],
                        [float(x) for x in rpy.group(1).split()] if rpy else [0, 0, 0])
            self.links[name] = entry

        self.joints = {}
        for m in re.finditer(r'<joint name="([^"]+)" type="([^"]+)"\s*>(.*?)</joint>',
                             s, re.S):
            name, jtype, body = m.group(1), m.group(2), m.group(3)
            parent = re.search(r'<parent link="([^"]+)"', body).group(1)
            child = re.search(r'<child link="([^"]+)"', body).group(1)
            o = re.search(r'<origin([^>]*)/>', body)
            xyz, rpy = [0, 0, 0], [0, 0, 0]
            if o:
                mx = re.search(r'xyz="([^"]+)"', o.group(1))
                mr = re.search(r'rpy="([^"]+)"', o.group(1))
                if mx: xyz = [float(v) for v in mx.group(1).split()]
                if mr: rpy = [float(v) for v in mr.group(1).split()]
            ax = re.search(r'<axis xyz="([^"]+)"', body)
            axis = [float(v) for v in ax.group(1).split()] if ax else [0, 0, 1]
            self.joints[name] = {"type": jtype, "parent": parent, "child": child,
                                 "origin": _T(xyz, rpy), "axis": axis}

        children = {j["child"] for j in self.joints.values()}
        roots = [l for l in self.links if l not in children]
        self.root = roots[0]

    def fk(self, q=None):
        """link name -> 4x4 world transform, base at the origin."""
        q = q or {}
        out = {self.root: np.eye(4)}
        kids = {}
        for jn, j in self.joints.items():
            kids.setdefault(j["parent"], []).append(jn)
        stack = [self.root]
        while stack:
            p = stack.pop()
            for jn in kids.get(p, []):
                j = self.joints[jn]
                T = out[p] @ j["origin"]
                v = float(q.get(jn, 0.0))
                if j["type"] in ("revolute", "continuous") and v:
                    R = np.eye(4); R[:3, :3] = _axis_angle(j["axis"], v)
                    T = T @ R
                elif j["type"] == "prismatic" and v:
                    D = np.eye(4); D[:3, 3] = np.asarray(j["axis"], float) * v
                    T = T @ D
                out[j["child"]] = T
                stack.append(j["child"])
        return out

    def visual_placements(self, q=None, base=np.eye(4)):
        """[(mesh, 4x4 world transform, scale)] for every link that has a visual."""
        fk = self.fk(q)
        out = []
        for name, L in self.links.items():
            if name not in fk or (L["mesh"] is None and L["prim"] is None):
                continue
            out.append({"link": name, "mesh": L["mesh"], "prim": L["prim"],
                        "T": (base @ fk[name] @ L["origin"]).tolist(),
                        "scale": L["scale"]})
        return out
