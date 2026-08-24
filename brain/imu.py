"""B2 step 1 -- simulated IMU driven by Genesis ground truth."""
import numpy as np

from nav_math import quat_to_R
from nav_params import IMUParams

MARS_G_VEC = np.array([0.0, 0.0, -3.72])


class SimulatedIMU:
    def __init__(self, params: IMUParams = None):
        self.p = params or IMUParams()
        self.dt = 1.0 / self.p.rate_hz
        self.rng = np.random.default_rng(self.p.seed)
        self.gyro_bias = np.array(self.p.gyro_bias0, float).copy()
        self.accel_bias = np.array(self.p.accel_bias0, float).copy()

        self.sig_g = self.p.gyro_noise_density / np.sqrt(self.dt)
        self.sig_a = self.p.accel_noise_density / np.sqrt(self.dt)
        self.sig_gb = self.p.gyro_bias_rw * np.sqrt(self.dt)
        self.sig_ab = self.p.accel_bias_rw * np.sqrt(self.dt)

    def true_signals(self, quat, ang_vel_world, acc_world):
        """Noise-free body-frame (specific force, angular rate)."""
        R = quat_to_R(quat)                       # body -> world
        f_body = R.T @ (np.asarray(acc_world, float) - MARS_G_VEC)
        w_body = R.T @ np.asarray(ang_vel_world, float)
        return f_body, w_body

    def measure(self, quat, ang_vel_world, acc_world):
        """One IMU sample. Advances the bias random walks."""
        f_body, w_body = self.true_signals(quat, ang_vel_world, acc_world)
        accel = f_body + self.accel_bias + self.rng.normal(0, self.sig_a, 3)
        gyro = w_body + self.gyro_bias + self.rng.normal(0, self.sig_g, 3)
        self.accel_bias += self.rng.normal(0, self.sig_ab, 3)
        self.gyro_bias += self.rng.normal(0, self.sig_gb, 3)
        return accel, gyro, f_body, w_body
