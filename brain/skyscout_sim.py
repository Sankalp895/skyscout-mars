"""SkyScout Track B / B1 -- Genesis sim wrapper (HEADLESS)."""
import numpy as np
import genesis as gs

from nav_math import quat_to_R
from mars_world import MarsWorld

DRONE_URDF = "models/skyscout_drone/skyscout_drone.urdf"

MARS_G = -3.72

SIM_HZ = 240.0
CONTROL_HZ = 50.0
SIM_DT = 1.0 / SIM_HZ
STEPS_PER_CONTROL = int(round(SIM_HZ / CONTROL_HZ))


class SkyScoutSim:
    def __init__(self, start_pos=(0.0, 0.0, 10.0), seed=None, n=None,
                 logging_level="warning", camera_res=None, camera_fov=55.0,
                 camera_far=300.0):
        # Genesis allows one gs.init per process
        if not getattr(gs, "_initialized", False):
            gs.init(backend=gs.cpu, logging_level=logging_level)
        self.scene = gs.Scene(
            show_viewer=False,
            sim_options=gs.options.SimOptions(dt=SIM_DT, gravity=(0, 0, MARS_G)),
        )
        kw = {}
        if seed is not None:
            kw["seed"] = seed
        if n is not None:
            kw["n"] = n
        self.world = MarsWorld(self.scene, **kw)

        self.drone = self.scene.add_entity(
            gs.morphs.URDF(file=DRONE_URDF, pos=tuple(start_pos), fixed=False),
            surface=gs.surfaces.Default(color=(0.85, 0.90, 1.0), roughness=0.6))

        self.camera = None
        if camera_res is not None:
            # far MUST be set: Genesis defaults to 20 m and silently clips an 80 m map
            self.camera = self.scene.add_camera(
                res=tuple(camera_res), pos=(10, 10, 10), lookat=(0, 0, 0),
                fov=camera_fov, near=0.1, far=camera_far, GUI=False)

        self.scene.build()
        self.solver = self.scene.sim.rigid_solver
        self.base_idx = self.drone.base_link.idx

        self.mass = float(sum(l.inertial_mass for l in self.drone.links))
        self.n_links = self.drone.n_links
        self.n_dofs = self.drone.n_dofs

        self._joint_dofs = list(range(6, self.n_dofs))
        if self._joint_dofs:
            self.drone.set_dofs_kp([400.0] * len(self._joint_dofs), self._joint_dofs)
            self.drone.set_dofs_kv([40.0] * len(self._joint_dofs), self._joint_dofs)
            self.drone.control_dofs_position(
                [0.0] * len(self._joint_dofs), self._joint_dofs)

        self.sim_time = 0.0
        self.steps = 0
        self.log = []

    def step_sim(self, dt=None):
        """Advance one fixed timestep."""
        if dt is not None and abs(dt - SIM_DT) > 1e-9:
            raise ValueError(f"fixed timestep is {SIM_DT}; got {dt}")
        self.scene.step()
        self.sim_time += SIM_DT
        self.steps += 1

    def get_true_state(self):
        """Ground truth from Genesis. B2 compares its ESKF estimate to this."""
        return {
            "t": self.sim_time,
            "pos": np.asarray(self.drone.get_pos(), dtype=float).copy(),
            "quat": np.asarray(self.drone.get_quat(), dtype=float).copy(),  # w,x,y,z
            "lin_vel": np.asarray(self.drone.get_vel(), dtype=float).copy(),
            "ang_vel": np.asarray(self.drone.get_ang(), dtype=float).copy(),
        }

    def terrain_height(self, x, y):
        return self.world.terrain_height(x, y)

    def apply_hover(self):
        """PLACEHOLDER for rotor thrust."""
        self._force([0.0, 0.0, -self.mass * MARS_G])

    def apply_track(self, target, target_vel=(0, 0, 0), kp=6.0, kd=4.0,
                    meas_pos=None, meas_vel=None):
        """PLACEHOLDER position tracker: PD + velocity feedforward.

        meas_pos/meas_vel are what the loop CLOSES ON and DEFAULT TO TRUTH.
        Any estimate-guided test must pass kf.p/kf.v: leaving the default
        made a B3 step-4 run come back bit-identical to the truth run.
        """
        if meas_pos is None or meas_vel is None:
            s = self.get_true_state()
            meas_pos = s["pos"] if meas_pos is None else meas_pos
            meas_vel = s["lin_vel"] if meas_vel is None else meas_vel
        err = np.asarray(target, float) - np.asarray(meas_pos, float)
        verr = np.asarray(target_vel, float) - np.asarray(meas_vel, float)
        self._force((self.mass * (kp * err + kd * verr)).tolist())

    def apply_attitude_hold(self, kp=8.0, kd=3.0):
        """PLACEHOLDER attitude stabilizer: holds the airframe LEVEL.

        The base rotational DOFs are otherwise uncontrolled: tilt reached
        27 deg by t=35 s and 176 deg over a 166 s survey. Earlier phases
        only flew 14-43 s, which is why it was never noticed.
        """
        s = self.get_true_state()
        bz = quat_to_R(s["quat"])[:, 2]
        err = np.cross(bz, np.array([0.0, 0.0, 1.0]))
        w = np.asarray(s["ang_vel"], float)
        tau = self.mass * (kp * err - kd * w)
        self.solver.apply_links_external_torque(
            torque=np.asarray([tau], dtype=float),
            links_idx=np.asarray([self.base_idx]),
        )

    def _force(self, force_world):
        """World-frame force at the WHOLE-BODY COM, so it adds no torque.

        ref MUST be "root_com". This is a 19-link body; link_origin and
        link_com leave a lever arm to the system COM and it tumbles.
        Measured over 1 s of hover: link_origin 2.21, link_com 3.05,
        root_com 0.00 rad/s.
        """
        self.solver.apply_links_external_force(
            force=np.asarray([force_world], dtype=float),
            links_idx=np.asarray([self.base_idx]),
            ref="root_com",
        )

    def record(self, phase=""):
        """Append one sample to the trajectory log."""
        s = self.get_true_state()
        self.log.append({
            "t": s["t"],
            "pos": s["pos"].tolist(),
            "quat": s["quat"].tolist(),
            "lin_vel": s["lin_vel"].tolist(),
            "ang_vel": s["ang_vel"].tolist(),
            "agl": float(s["pos"][2] - self.terrain_height(s["pos"][0], s["pos"][1])),
            "phase": phase,
        })

    def save_log(self, path):
        import json
        with open(path, "w") as f:
            json.dump({"mars_g": MARS_G, "sim_hz": SIM_HZ,
                       "control_hz": CONTROL_HZ,
                       "world": self.world.summary(),
                       "samples": self.log}, f, indent=1)
        return path
