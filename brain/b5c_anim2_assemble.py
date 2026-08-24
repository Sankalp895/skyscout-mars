"""B5C v2 -- assemble the composited PNG sequence into an MP4."""
import os, glob, time
import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "renders", "anim2_final")
OUT = os.path.join(HERE, "renders", "skyscout_scout_mission.mp4")
FPS = 24

files = sorted(os.path.basename(f) for f in glob.glob(os.path.join(SRC, "*.png")))
if not files:
    raise SystemExit(f"no composited frames in {SRC}")

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.sequence_editor_create()
strip = sc.sequence_editor.sequences.new_image(
    "clip", os.path.join(SRC, files[0]), 1, 1)
for f in files[1:]:
    strip.elements.append(f)

sc.frame_start = 1
sc.frame_end = len(files)
sc.render.fps = FPS
sc.render.resolution_x, sc.render.resolution_y = 1280, 720
sc.render.resolution_percentage = 100
sc.render.image_settings.file_format = "FFMPEG"
sc.render.ffmpeg.format = "MPEG4"
sc.render.ffmpeg.codec = "H264"
sc.render.ffmpeg.constant_rate_factor = "HIGH"
sc.render.ffmpeg.ffmpeg_preset = "GOOD"
sc.render.filepath = OUT

t0 = time.time()
bpy.ops.render.render(animation=True)
print(f"ASSEMBLED {len(files)} frames -> {OUT} in {time.time()-t0:.1f} s", flush=True)
