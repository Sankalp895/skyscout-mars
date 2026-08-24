"""Mux a music track onto the finished clip, using Blender's FFmpeg encoder."""
import os, sys, time
import bpy

HERE = os.path.dirname(os.path.abspath(__file__))
VIDEO = os.path.join(HERE, "renders", "skyscout_scout_mission.mp4")
OUT = os.path.join(HERE, "renders", "skyscout_scout_mission_music.mp4")
FPS, N_FRAMES = 24, 1200

args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
if not args:
    raise SystemExit("usage: blender -b -P b5c_add_music.py -- <audio file> [volume]")
audio = os.path.abspath(args[0])
vol = float(args[1]) if len(args) > 1 else 0.5
if not os.path.exists(audio):
    raise SystemExit(f"audio not found: {audio}")

bpy.ops.wm.read_factory_settings(use_empty=True)
sc = bpy.context.scene
sc.sequence_editor_create()
se = sc.sequence_editor

se.sequences.new_movie("vid", VIDEO, 1, 1)
snd = se.sequences.new_sound("music", audio, 2, 1)
snd.volume = vol
snd.frame_final_end = min(snd.frame_final_end, N_FRAMES)

sc.frame_start, sc.frame_end = 1, N_FRAMES
sc.render.fps = FPS
sc.render.resolution_x, sc.render.resolution_y = 1280, 720
sc.render.image_settings.file_format = "FFMPEG"
sc.render.ffmpeg.format = "MPEG4"
sc.render.ffmpeg.codec = "H264"
sc.render.ffmpeg.constant_rate_factor = "HIGH"
sc.render.ffmpeg.audio_codec = "AAC"
sc.render.ffmpeg.audio_bitrate = 192
sc.render.filepath = OUT

t0 = time.time()
bpy.ops.render.render(animation=True)
print(f"MUXED {audio} (vol {vol}) -> {OUT} in {time.time()-t0:.1f} s", flush=True)
