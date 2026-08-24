"""B5 -- Genesis scene holding the Mars terrain and the SkyScout rover."""
import numpy as np
import genesis as gs

from mars_world import MarsWorld

ROVER_URDF = "models/skyscout_rover/skyscout_rover.urdf"
MARS_G = -3.72
SIM_HZ = 240.0
SIM_DT = 1.0 / SIM_HZ

DRIVE = ["drive_fl", "drive_ml", "drive_rl", "drive_fr", "drive_mr", "drive_rr"]
STEER = ["steer_fl", "steer_rl", "steer_fr", "steer_rr"]
ARTIC = ["rocker_port", "bogie_port", "rocker_stbd", "bogie_stbd"]


class RoverSim:
    def __init__(self, start_xy=(0.0, 0.0), drop_h=1.2, logging_level="error",
                 camera_res=None):
        if not getattr(gs, "_initialized", False):
            gs.init(backend=gs.cpu, logging_level=logging_level)
        self.scene = gs.Scene(
            show_viewer=False,
            sim_options=gs.options.SimOptions(dt=SIM_DT, gravity=(0, 0, MARS_G)))
        self.world = MarsWorld(self.scene)
        z0 = self.world.terrain_height(*start_xy) + drop_h
        self.rover = self.scene.add_entity(
            gs.morphs.URDF(file=ROVER_URDF, pos=(start_xy[0], start_xy[1], z0),
                           fixed=False),
            surface=gs.surfaces.Default(color=(0.82, 0.78, 0.70), roughness=0.7))
        self.camera = None
        if camera_res is not None:
            self.camera = self.scene.add_camera(
                res=tuple(camera_res), pos=(10, 10, 10), lookat=(0, 0, 0),
                fov=55.0, near=0.1, far=300.0, GUI=False)
        self.scene.build()
        self.solver = self.scene.sim.rigid_solver
        self.base_idx = self.rover.base_link.idx
        self.mass = float(sum(l.inertial_mass for l in self.rover.links))
        self.sim_time = 0.0

        name = {j.name: j for j in self.rover.joints}
        self.drive_dofs = [int(name[n].dofs_idx_local[0]) for n in DRIVE if n in name]
        self.steer_dofs = [int(name[n].dofs_idx_local[0]) for n in STEER if n in name]
        self.artic_dofs = {n: int(name[n].dofs_idx_local[0])
                           for n in ARTIC if n in name}
        loco = set(self.drive_dofs) | set(self.steer_dofs) | set(self.artic_dofs.values())
        self.held = [d for d in range(6, self.rover.n_dofs) if d not in loco]
        if self.held:
            self.rover.set_dofs_kp([600.0] * len(self.held), self.held)
            self.rover.set_dofs_kv([60.0] * len(self.held), self.held)
            self.rover.control_dofs_position([0.0] * len(self.held), self.held)
        if self.steer_dofs:
            self.rover.set_dofs_kp([400.0] * len(self.steer_dofs), self.steer_dofs)
            self.rover.set_dofs_kv([40.0] * len(self.steer_dofs), self.steer_dofs)
            self.rover.control_dofs_position([0.0] * len(self.steer_dofs),
                                             self.steer_dofs)
        if self.drive_dofs:
            self.rover.set_dofs_kv([80.0] * len(self.drive_dofs), self.drive_dofs)

    def step(self):
        self.scene.step()
        self.sim_time += SIM_DT

    def state(self):
        return {
            "t": self.sim_time,
            "pos": np.asarray(self.rover.get_pos(), float).copy(),
            "quat": np.asarray(self.rover.get_quat(), float).copy(),
            "vel": np.asarray(self.rover.get_vel(), float).copy(),
        }

    def artic_angles(self):
        q = np.asarray(self.rover.get_dofs_position(), float)
        return {k: float(q[v]) for k, v in self.artic_dofs.items()}

    def drive(self, wheel_rad_s):
        self.rover.control_dofs_velocity([wheel_rad_s] * len(self.drive_dofs),
                                         self.drive_dofs)

    def settle(self, seconds=2.5):
        """Drop the rover onto the terrain and let it come to rest."""
        for _ in range(int(seconds * SIM_HZ)):
            self.drive(0.0)
            self.step()
        s = self.state()
        gnd = self.world.terrain_height(s["pos"][0], s["pos"][1])
        return float(s["pos"][2] - gnd), s

    def set_pose(self, xy, yaw, ride_h):
        """Kinematic placement: chassis level, following the terrain."""
        z = self.world.terrain_height(xy[0], xy[1]) + ride_h
        self.rover.set_pos(np.array([xy[0], xy[1], z], float))
        self.rover.set_quat(np.array([np.cos(yaw/2), 0.0, 0.0, np.sin(yaw/2)], float))
        return np.array([xy[0], xy[1], z])

    # URDF cannot express the differential; without it the chassis pitches with the rockers
    def apply_differential(self, k=4000.0, c=400.0):
        if "rocker_port" not in self.artic_dofs or "rocker_stbd" not in self.artic_dofs:
            return 0.0
        dp = self.artic_dofs["rocker_port"]
        ds = self.artic_dofs["rocker_stbd"]
        q = np.asarray(self.rover.get_dofs_position(), float)
        w = np.asarray(self.rover.get_dofs_velocity(), float)
        s_ang = q[dp] + q[ds]
        s_rate = w[dp] + w[ds]
        tau = -k * s_ang - c * s_rate
        self.rover.control_dofs_force([tau, tau], [dp, ds])
        return float(s_ang)
