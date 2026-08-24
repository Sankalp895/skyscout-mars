"""B4 -- downward camera rig and ground projection."""
import numpy as np

# measured: depth is planar-z, fov is vertical (0.002 m RMS vs 0.84+ for the alternatives)
DEPTH_IS_EUCLIDEAN = False
FOV_IS_VERTICAL = True

MOUNT_OFFSET_BODY = np.array([0.0, 0.0, -0.08])


def intrinsics(res_wh, fov_deg, fov_is_vertical=None):
    """(fx, fy, cx, cy) in pixels, square pixels assumed."""
    if fov_is_vertical is None:
        fov_is_vertical = FOV_IS_VERTICAL
    w, h = res_wh
    half = np.tan(np.radians(fov_deg) / 2.0)
    f = (h / 2.0) / half if fov_is_vertical else (w / 2.0) / half
    return f, f, (w - 1) / 2.0, (h - 1) / 2.0


def pose_matrix(cam_pos, lookat, up=(1, 0, 0)):
    """Camera->world rotation for an OpenGL-style camera (looks down -Z_cam, +X_cam right, +Y_cam u."""
    cam_pos = np.asarray(cam_pos, float)
    fwd = np.asarray(lookat, float) - cam_pos
    fwd /= np.linalg.norm(fwd)
    up = np.asarray(up, float)
    right = np.cross(fwd, up)
    n = np.linalg.norm(right)
    if n < 1e-9:
        up = np.array([0.0, 1.0, 0.0])
        right = np.cross(fwd, up)
        n = np.linalg.norm(right)
    right /= n
    true_up = np.cross(right, fwd)
    R = np.column_stack([right, true_up, -fwd])
    return R, cam_pos


def ray_dirs(res_wh, fov_deg, fov_is_vertical=None):
    """Unit ray directions in the CAMERA frame, shape (h, w, 3)."""
    w, h = res_wh
    fx, fy, cx, cy = intrinsics(res_wh, fov_deg, fov_is_vertical)
    v, u = np.mgrid[0:h, 0:w]
    x = (u - cx) / fx
    y = -(v - cy) / fy
    z = -np.ones_like(x)
    d = np.stack([x, y, z], axis=-1)
    return d / np.linalg.norm(d, axis=-1, keepdims=True)


def backproject(depth, cam_pos, lookat, res_wh, fov_deg, up=(1, 0, 0),
                depth_is_euclidean=None, fov_is_vertical=None):
    """Depth frame -> world-frame points, shape (h, w, 3)."""
    if depth_is_euclidean is None:
        depth_is_euclidean = DEPTH_IS_EUCLIDEAN
    depth = np.asarray(depth, float)
    if depth.ndim == 3:
        depth = depth[..., 0]
    dirs = ray_dirs(res_wh, fov_deg, fov_is_vertical)
    if depth_is_euclidean:
        pts_cam = dirs * depth[..., None]
    else:
        pts_cam = dirs * (depth / np.abs(dirs[..., 2]))[..., None]
    R, t = pose_matrix(cam_pos, lookat, up)
    return pts_cam @ R.T + t


def footprint_corners(cam_pos, lookat, res_wh, fov_deg, ground_z=0.0,
                      up=(1, 0, 0), fov_is_vertical=None):
    """Where the four corner rays meet a FLAT plane at ground_z."""
    w, h = res_wh
    fx, fy, cx, cy = intrinsics(res_wh, fov_deg, fov_is_vertical)
    R, t = pose_matrix(cam_pos, lookat, up)
    out = []
    for (u, v) in ((0, 0), (w - 1, 0), (w - 1, h - 1), (0, h - 1)):
        d = np.array([(u - cx) / fx, -(v - cy) / fy, -1.0])
        d = R @ (d / np.linalg.norm(d))
        if abs(d[2]) < 1e-9:
            out.append([np.nan, np.nan]); continue
        s = (ground_z - t[2]) / d[2]
        p = t + s * d
        out.append([p[0], p[1]])
    return np.asarray(out)


class DownwardCamera:
    """Rigidly-mounted downward camera on the drone."""

    def __init__(self, sim, res_wh, fov_deg):
        if sim.camera is None:
            raise RuntimeError("build SkyScoutSim with camera_res=... "
                               "(cameras must be added before scene.build())")
        self.sim, self.cam = sim, sim.camera
        self.res_wh, self.fov = tuple(res_wh), float(fov_deg)
        self._warm = False

    def warm_up(self):
        self.cam.render(rgb=True)
        self._warm = True

    def pose_from(self, pos):
        """Camera pose for a drone body at `pos`."""
        p = np.asarray(pos, float) + MOUNT_OFFSET_BODY
        return p, np.array([p[0], p[1], p[2] - 1.0])

    # capture() = TRUE pose (physics). project() = ESTIMATE (interpretation). The gap is map smear.

    def capture(self, body_pos_true, want_rgb=False):
        """PHYSICAL capture."""
        if not self._warm:
            raise RuntimeError("call warm_up() before any timed capture")
        cam_pos, lookat = self.pose_from(body_pos_true)
        self.cam.set_pose(pos=tuple(cam_pos), lookat=tuple(lookat), up=(1, 0, 0))
        out = self.cam.render(rgb=want_rgb, depth=True)
        depth = np.asarray(out[1], float)
        rgb = np.asarray(out[0], np.uint8)[..., :3] if want_rgb else None
        return depth, rgb, cam_pos, lookat

    def self_view_pixels(self, depth, min_range=1.0):
        """Count pixels returning less than min_range."""
        d = np.asarray(depth, float)
        return int(((d > 0) & (d < min_range)).sum())

    def project(self, depth, body_pos_believed):
        """INTERPRETATION. body_pos_believed is the ESKF estimate in B4."""
        cam_pos, lookat = self.pose_from(body_pos_believed)
        return backproject(depth, cam_pos, lookat, self.res_wh, self.fov)
