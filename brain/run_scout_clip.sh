#!/usr/bin/env bash
# SkyScout -- full scout-mission clip: render -> overlay -> MP4
# Usage:  ./run_scout_clip.sh            (all three stages)
#         ./run_scout_clip.sh overlay    (skip render, redo overlay + mp4)
set -e
cd "$(dirname "$0")"
BL=~/blender/blender
PY=.venv-genesis/bin/python

if [ "$1" != "overlay" ]; then
  echo "[1/3] rendering 1128 frames (~7 h, 720p/64 samples)..."
  $PY b5c_anim2_timeline.py
  $BL -b -P b5c_anim2_render.py -- all
fi
echo "[2/3] compositing scan minimap overlay..."
$PY b5c_anim2_overlay.py
echo "[3/3] assembling MP4..."
$BL -b -P b5c_anim2_assemble.py
echo
echo "DONE -> renders/skyscout_scout_mission.mp4"
