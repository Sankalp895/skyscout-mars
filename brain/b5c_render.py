"""B5 part C -- headless Blender scene build + hero stills."""
import json, os, sys, time, math

import bpy
from mathutils import Matrix, Vector

ARGS = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "renders")
RES = (1920, 1080)
MAX_SAMPLES = int(os.environ.get("B5C_SAMPLES", "128"))


def pbr(name, base, rough, metal=0.0, emit=None):
    m = bpy.data.materials.new(name)
    m.use_nodes = True
    b = m.node_tree.nodes["Principled BSDF"]
    b.inputs["Base Color"].default_value = (*base, 1.0)
    b.inputs["Roughness"].default_value = rough
    b.inputs["Metallic"].default_value = metal
    if emit:
        b.inputs["Emission Color"].default_value = (*emit, 1.0)
        b.inputs["Emission Strength"].default_value = 2.5
    return m


def build_materials():
    """The deferred ~16-group pass, finally done -- on the Blender side."""
    M = {}
    M["regolith"]   = pbr("regolith",   (0.42, 0.20, 0.12), 0.95, 0.0)
    M["d_body"]     = pbr("d_body",     (0.72, 0.74, 0.78), 0.35, 0.15)
    M["d_wing"]     = pbr("d_wing",     (0.80, 0.82, 0.86), 0.30, 0.10)
    M["d_rotor"]    = pbr("d_rotor",    (0.06, 0.06, 0.07), 0.55, 0.20)
    M["d_nacelle"]  = pbr("d_nacelle",  (0.55, 0.57, 0.62), 0.22, 0.90)
    M["d_skid"]     = pbr("d_skid",     (0.18, 0.18, 0.20), 0.60, 0.40)
    M["d_sensor"]   = pbr("d_sensor",   (0.10, 0.12, 0.16), 0.15, 0.30,
                          emit=(0.05, 0.35, 0.55))
    M["r_chassis"]  = pbr("r_chassis",  (0.78, 0.74, 0.66), 0.40, 0.25)
    M["r_wheel"]    = pbr("r_wheel",    (0.28, 0.27, 0.26), 0.70, 0.60)
    M["r_rocker"]   = pbr("r_rocker",   (0.62, 0.60, 0.56), 0.45, 0.55)
    M["r_rtg"]      = pbr("r_rtg",      (0.20, 0.19, 0.18), 0.35, 0.80)
    M["r_mast"]     = pbr("r_mast",     (0.68, 0.66, 0.62), 0.40, 0.50)
    M["r_cam"]      = pbr("r_cam",      (0.05, 0.06, 0.08), 0.12, 0.30,
                          emit=(0.10, 0.45, 0.60))
    M["r_arm"]      = pbr("r_arm",      (0.70, 0.68, 0.64), 0.38, 0.60)
    M["r_panel"]    = pbr("r_panel",    (0.17, 0.21, 0.38), 0.36, 0.22)
    M["r_antenna"]  = pbr("r_antenna",  (0.85, 0.85, 0.83), 0.50, 0.30)
    M["r_pad"]      = pbr("r_pad",      (0.55, 0.52, 0.48), 0.55, 0.45)
    return M


def pick(link, M, kind):
    n = link.lower()
    if kind == "drone":
        if "rotor" in n:                       return M["d_rotor"]
        if "tilt" in n or "nacelle" in n:      return M["d_nacelle"]
        if "wing" in n or "elevon" in n:       return M["d_wing"]
        if "skid" in n:                        return M["d_skid"]
        if "sensor" in n:                      return M["d_sensor"]
        return M["d_body"]
    if "wheel" in n:                           return M["r_wheel"]
    if "rocker" in n or "bogie" in n:          return M["r_rocker"]
    if "rtg" in n:                             return M["r_rtg"]
    if "mast" in n:                            return M["r_mast"]
    if "cam" in n:                             return M["r_cam"]
    if "arm" in n or "turret" in n:            return M["r_arm"]
    if "panel" in n:                           return M["r_panel"]
    if "antenna" in n:                         return M["r_antenna"]
    if "pad" in n or "helipad" in n or "latch" in n or "riser" in n:
        return M["r_pad"]
    return M["r_chassis"]


_mesh_cache = {}

def add_mesh(path, T, scale, mat, name):
    if path in _mesh_cache:
        obj = _mesh_cache[path].copy()
        obj.data = _mesh_cache[path].data
        bpy.context.collection.objects.link(obj)
    else:
        before = set(bpy.data.objects)
        try:
            bpy.ops.wm.stl_import(filepath=path)
        except AttributeError:
            bpy.ops.import_mesh.stl(filepath=path)
        new = list(set(bpy.data.objects) - before)
        if not new:
            return None
        obj = new[0]
        _mesh_cache[path] = obj
    obj.name = name
    obj.matrix_world = Matrix(T) @ Matrix.Scale(scale, 4)
    obj.data.materials.clear()
    obj.data.materials.append(mat)
    for p in obj.data.polygons:
        p.use_smooth = True
    return obj


def add_prim(kind, params, T, mat, name):
    # build the matrix explicitly: o.scale is not applied to matrix_world until the depsgraph runs
    if kind == "box":
        bpy.ops.mesh.primitive_cube_add(size=1.0)
        o = bpy.context.object
        S = Matrix.Diagonal((params[0], params[1], params[2], 1.0))
    elif kind == "cylinder":
        bpy.ops.mesh.primitive_cylinder_add(radius=params[0], depth=params[1],
                                            vertices=48)
        o = bpy.context.object
        S = Matrix.Identity(4)
    else:
        bpy.ops.mesh.primitive_uv_sphere_add(radius=params[0])
        o = bpy.context.object
        S = Matrix.Identity(4)
    o.matrix_world = Matrix(T) @ S
    o.name = name
    o.data.materials.clear()
    o.data.materials.append(mat)
    return o


def build_world(cfg, M):
    # up_axis MUST be Z: the importer defaults to Y and rotates the terrain into a wall
    bpy.ops.wm.obj_import(filepath=os.path.join(HERE, cfg["terrain_obj"]),
                          forward_axis="Y", up_axis="Z")
    for o in bpy.context.selected_objects:
        o.data.materials.clear()
        o.data.materials.append(M["regolith"])
        for p in o.data.polygons:
            p.use_smooth = True

    import numpy as _np
    _h = bpy.data.objects[0].data if False else None
    bpy.ops.mesh.primitive_plane_add(size=600.0, location=(0, 0, -0.35))
    ap = bpy.context.object
    ap.name = "apron"
    ap.data.materials.clear()
    ap.data.materials.append(M["regolith"])

    bpy.ops.object.light_add(type="SUN", location=(0, 0, 40))
    sun = bpy.context.object
    sun.data.energy = 4.0
    sun.data.angle = math.radians(0.35)
    sun.data.color = (1.0, 0.78, 0.55)
    sun.rotation_euler = (math.radians(62), 0, math.radians(35))

    bpy.ops.object.light_add(type="AREA", location=(-14, -18, 22))
    fill = bpy.context.object
    fill.data.energy = 900.0
    fill.data.size = 28.0
    fill.data.color = (0.85, 0.72, 0.68)
    fill.rotation_euler = (math.radians(38), 0, math.radians(-30))

    w = bpy.data.worlds.new("mars_sky")
    bpy.context.scene.world = w
    w.use_nodes = True
    bg = w.node_tree.nodes["Background"]
    bg.inputs[0].default_value = (0.55, 0.32, 0.22, 1.0)
    bg.inputs[1].default_value = 0.55


def build_shot(name, shot, cfg, M):
    for kind, root in (("drone", cfg["mesh_roots"]["drone"]),
                       ("rover", cfg["mesh_roots"]["rover"])):
        for v in shot.get(kind, []):
            mat = pick(v["link"], M, kind)
            nm = f"{kind}_{v['link']}"
            if v["mesh"]:
                add_mesh(os.path.join(root, v["mesh"]), v["T"], v["scale"], mat, nm)
            elif v["prim"]:
                add_prim(v["prim"][0], v["prim"][1], v["T"], mat, nm)


CAMS = {
    "1_reveal":          dict(off=(4.6, -5.9, 2.7), lens=42, dof=True),
    "2_flight":          dict(off=(-3.6, -4.6, 1.15), lens=54, dof=False),
    "3_scout_and_drive": dict(off=(12.0, -20.0, 10.5), lens=34, dof=False),
    "4_rover_detail":    dict(off=(3.5, -4.3, 1.95), lens=50, dof=True),
}


def add_camera(name, shot):
    c = CAMS[name]
    focus = Vector(shot["focus"])
    bpy.ops.object.camera_add(location=focus + Vector(c["off"]))
    cam = bpy.context.object
    cam.data.lens = c["lens"]
    d = focus - cam.location
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    if c["dof"]:
        cam.data.dof.use_dof = True
        cam.data.dof.focus_distance = d.length
        cam.data.dof.aperture_fstop = 3.2
    bpy.context.scene.camera = cam


def render(name, shot, cfg, M):
    bpy.ops.wm.read_factory_settings(use_empty=True)
    _mesh_cache.clear()
    M = build_materials()
    build_world(cfg, M)
    build_shot(name, shot, cfg, M)
    add_camera(name, shot)

    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = MAX_SAMPLES
    sc.cycles.use_adaptive_sampling = True
    sc.cycles.adaptive_threshold = 0.02
    sc.cycles.use_denoising = True
    sc.cycles.max_bounces = 6
    sc.render.resolution_x, sc.render.resolution_y = RES
    sc.render.film_transparent = False
    sc.view_settings.look = "AgX - Medium High Contrast"
    os.makedirs(OUT, exist_ok=True)
    sc.render.filepath = os.path.join(OUT, f"{name}.png")

    n_obj = len([o for o in bpy.data.objects if o.type == "MESH"])
    t0 = time.time()
    bpy.ops.render.render(write_still=True)
    dt = time.time() - t0
    print(f"RENDERSTAT {name} {dt:.1f} {n_obj} {MAX_SAMPLES}", flush=True)
    return dt


def main():
    cfg = json.load(open(os.path.join(HERE, "b5c_shots.json")))
    want = ARGS or list(cfg["shots"].keys())
    total = 0.0
    for name in want:
        shot = cfg["shots"][name]
        print(f"\n=== {name} :: {shot['motion_source']}", flush=True)
        total += render(name, shot, cfg, None)
    print(f"\nTOTAL {total:.1f} s for {len(want)} stills at {RES[0]}x{RES[1]}, "
          f"{MAX_SAMPLES} max samples, CPU Cycles", flush=True)


if __name__ == "__main__":
    main()
