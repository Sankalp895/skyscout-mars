"""Interactive terrain viewer, tuned for llvmpipe (software rendering)."""
import argparse
import time

import pybullet as p
import pybullet_data

import mars_terrain as mt
from skyscout_sim import DRONE_URDF, MARS_G


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=64, help="display mesh resolution")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=420)
    ap.add_argument("--physics", action="store_true",
                    help="actually step physics (slower, usually not wanted here)")
    args = ap.parse_args()

    p.connect(p.GUI, options=f"--width={args.width} --height={args.height}")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    for flag in (p.COV_ENABLE_RGB_BUFFER_PREVIEW,
                 p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,
                 p.COV_ENABLE_SEGMENTATION_MARK_PREVIEW,
                 p.COV_ENABLE_SHADOWS,
                 p.COV_ENABLE_TINY_RENDERER,
                 p.COV_ENABLE_GUI):
        p.configureDebugVisualizer(flag, 0)

    p.setGravity(0, 0, MARS_G)
    terrain = mt.MarsTerrain(n=args.n)

    drone = p.loadURDF(DRONE_URDF, [0, 0, 9.3], useFixedBase=not args.physics)

    p.resetDebugVisualizerCamera(cameraDistance=38, cameraYaw=35,
                                 cameraPitch=-28, cameraTargetPosition=[0, 0, 0])

    s = terrain.summary()
    print(f"terrain {s['extent_m']:.0f} m across | display mesh {s['samples']} "
          f"({2 * (args.n - 1) ** 2} tris) | elev {terrain.min_h:.2f}..{terrain.max_h:.2f} m")
    print(f"window {args.width}x{args.height} | previews+shadows+tinyrenderer OFF"
          f" | physics {'ON' if args.physics else 'PARKED'}")
    print("left-drag orbit, ctrl+left-drag pan, scroll zoom.  Ctrl-C to quit.")

    try:
        while True:
            if args.physics:
                p.stepSimulation()
                time.sleep(1 / 240)
            else:
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()


if __name__ == "__main__":
    main()
