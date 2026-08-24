"""B5C -- render the mission clip from the recorded logs."""
import json, os, sys, time, math

import bpy
import numpy as np
from mathutils import Matrix, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from urdf_fk import URDF
import b5c_render as R

ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else ["test", "150"]
RES = (1280, 720)
SAMPLES = int(os.environ.get("B5C_SAMPLES", "64"))
OUT = os.path.join(HERE, "renders")

TL = json.load(open(os.path.join(HERE, "b5c_anim2.json")))
FRAMES = TL["frames"]
URDFS = {k: URDF(v) for k, v in TL["urdf"].items()}
PARTS = {}


def build_once():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    R._mesh_cache.clear()
    M = R.build_materials()
    R.build_world(TL, M)
    for kind in ("drone", "rover"):
        u = URDFS[kind]
        root = TL["mesh_roots"][kind]
        PARTS[kind] = {}
        for name, L in u.links.items():
            if L["mesh"] is None and L["prim"] is None:
                continue
            mat = R.pick(name, M, kind)
            T = Matrix.Identity(4)
            if L["mesh"]:
                o = R.add_mesh(os.path.join(root, L["mesh"]), T, L["scale"],
                               mat, f"{kind}_{name}")
            else:
                o = R.add_prim(L["prim"][0], L["prim"][1], T, mat, f"{kind}_{name}")
            if o is not None:
                PARTS[kind][name] = o
    bpy.ops.object.camera_add(location=(0, 0, 10))
    cam = bpy.context.object
    bpy.context.scene.camera = cam
    return cam


def pose_frame(idx, cam):
    fr = FRAMES[max(0, min(idx, len(FRAMES) - 1))]
    for kind in ("drone", "rover"):
        spec = fr.get(kind)
        objs = PARTS[kind]
        if spec is None:
            for o in objs.values():
                o.hide_render = True
            continue
        base = np.array(spec["base"], float)
        fk = URDFS[kind].fk(spec["q"])
        for name, o in objs.items():
            o.hide_render = False
            L = URDFS[kind].links[name]
            T = base @ fk[name] @ L["origin"]
            S = Matrix.Scale(L["scale"], 4) if L["mesh"] else _prim_scale(L)
            o.matrix_world = Matrix(T.tolist()) @ S
    c = fr["cam"]
    focus = Vector(c["focus"])
    cam.location = focus + Vector(c["off"])
    cam.data.lens = c["lens"]
    d = focus - cam.location
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    cam.data.dof.use_dof = bool(c["dof"])
    if c["dof"]:
        cam.data.dof.focus_distance = d.length
        cam.data.dof.aperture_fstop = 3.6


def _prim_scale(L):
    kind, p = L["prim"]
    if kind == "box":
        return Matrix.Diagonal((p[0], p[1], p[2], 1.0))
    return Matrix.Identity(4)


def setup_render(video, first, last):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = SAMPLES
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.adaptive_threshold = 0.03
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 4
    sc.render.resolution_x, sc.render.resolution_y = RES
    sc.render.fps = TL["fps"]
    sc.view_settings.look = "AgX - Medium High Contrast"
    sc.frame_start, sc.frame_end = first, last
    os.makedirs(OUT, exist_ok=True)
    sc.render.image_settings.file_format = "PNG"
    return sc


def main():
    mode = ARGS[0]
    cam = build_once()

    if mode == "test":
        idx = int(ARGS[1])
        sc = setup_render(False, idx, idx)
        pose_frame(idx, cam)
        sc.render.filepath = os.path.join(OUT, f"anim2_test_{idx:04d}.png")
        t0 = time.time()
        bpy.ops.render.render(write_still=True)
        dt = time.time() - t0
        fr = FRAMES[idx]
        print(f"TESTSTAT frame={idx} beat={fr['beat']} secs={dt:.1f} "
              f"res={RES[0]}x{RES[1]} samples={SAMPLES}", flush=True)
        print(f"  source: {fr['source']}", flush=True)
        return

    first, last = (int(ARGS[1]), int(ARGS[2])) if mode == "range" else (0, len(FRAMES) - 1)
    sc = setup_render(True, first, last)
    raw = os.path.join(OUT, "anim2_raw")
    os.makedirs(raw, exist_ok=True)

    # the differential constrains theta_port + theta_stbd = 0, leaving the difference free
    t0 = time.time()
    n = 0
    skipped = 0
    for i in range(first, last + 1):
        path = os.path.join(raw, f"f{i:04d}.png")
        if os.path.exists(path) and os.path.getsize(path) > 2000:
            skipped += 1
            continue
        pose_frame(i, cam)
        sc.render.filepath = path
        bpy.ops.render.render(write_still=True)
        n += 1
        if n % 10 == 0:
            el = time.time() - t0
            rate = el / n
            eta = (last - i) * rate
            print(f"PROGRESS frame={i}/{last} rendered={n} skipped={skipped} "
                  f"rate={rate:.1f}s/fr eta={eta/3600:.2f}h", flush=True)
    dt = time.time() - t0
    n = max(n, 1)
    print(f"ANIMSTAT rendered={n} skipped={skipped} secs={dt:.1f} per_frame={dt/n:.1f} "
          f"res={RES[0]}x{RES[1]} samples={SAMPLES}", flush=True)
    print(f"  output dir: {raw}", flush=True)


main()
