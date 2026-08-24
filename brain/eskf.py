"""B2 steps 3-4 -- Error-State Kalman Filter, from scratch."""
import numpy as np

from dead_reckon import strapdown_step, G_WORLD
from nav_math import qmul, qnorm, quat_to_R, quat_from_small_angle, skew
from nav_params import IMUParams, ESKFParams

P_, V_, TH_, AB_, GB_ = (slice(0, 3), slice(3, 6), slice(6, 9),
                         slice(9, 12), slice(12, 15))
N = 15


class ESKF:
    def __init__(self, pos, vel, quat, imu: IMUParams = None,
                 params: ESKFParams = None):
        self.imu = imu or IMUParams()
        self.pr = params or ESKFParams()

        self.p = np.asarray(pos, float).copy()
        self.v = np.asarray(vel, float).copy()
        self.q = qnorm(np.asarray(quat, float).copy())
        self.ab = np.zeros(3)
        self.gb = np.zeros(3)

        pr = self.pr
        self.P = np.diag(np.r_[
            np.full(3, pr.sigma_p0 ** 2), np.full(3, pr.sigma_v0 ** 2),
            np.full(3, pr.sigma_th0 ** 2), np.full(3, pr.sigma_ab0 ** 2),
            np.full(3, pr.sigma_gb0 ** 2)])

        i = self.imu
        self._q_v = i.accel_noise_density ** 2
        self._q_th = i.gyro_noise_density ** 2
        self._q_ab = i.accel_bias_rw ** 2
        self._q_gb = i.gyro_bias_rw ** 2

    def predict(self, accel_m, gyro_m, dt):
        """Propagate nominal with the IMU, and the error covariance with the linearized model."""
        a_c = np.asarray(accel_m, float) - self.ab
        w_c = np.asarray(gyro_m, float) - self.gb

        R = quat_to_R(self.q)

        self.p, self.v, self.q, _ = strapdown_step(
            self.p, self.v, self.q, a_c, w_c, dt)

        F = np.eye(N)
        F[P_, V_] = np.eye(3) * dt
        F[V_, TH_] = -R @ skew(a_c) * dt      # attitude error tilts gravity in
        F[V_, AB_] = -R * dt
        F[TH_, TH_] = np.eye(3) - skew(w_c) * dt
        F[TH_, GB_] = -np.eye(3) * dt

        Q = np.zeros((N, N))
        Q[V_, V_] = np.eye(3) * self._q_v * dt
        Q[TH_, TH_] = np.eye(3) * self._q_th * dt
        Q[AB_, AB_] = np.eye(3) * self._q_ab * dt
        Q[GB_, GB_] = np.eye(3) * self._q_gb * dt

        self.P = F @ self.P @ F.T + Q
        self.P = 0.5 * (self.P + self.P.T)
        return self.P

    def update_position(self, z_pos, sigma):
        """Correct with a noisy position fix. Returns (innovation, NIS)."""
        H = np.zeros((3, N))
        H[:, P_] = np.eye(3)
        Rm = np.eye(3) * sigma ** 2

        y = np.asarray(z_pos, float) - self.p
        S = H @ self.P @ H.T + Rm
        K = self.P @ H.T @ np.linalg.inv(S)
        dx = K @ y

        I_KH = np.eye(N) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ Rm @ K.T
        self.P = 0.5 * (self.P + self.P.T)

        self.inject(dx)
        nis = float(y @ np.linalg.inv(S) @ y)
        return y, nis

    def inject(self, dx):
        """Fold the error state into the nominal, then RESET it to zero."""
        self.p = self.p + dx[P_]
        self.v = self.v + dx[V_]
        self.q = qnorm(qmul(self.q, quat_from_small_angle(dx[TH_])))
        self.ab = self.ab + dx[AB_]
        self.gb = self.gb + dx[GB_]

        # reset Jacobian G: skipping it is the classic ESKF bug
        G = np.eye(N)
        G[TH_, TH_] = np.eye(3) - skew(0.5 * dx[TH_])
        self.P = G @ self.P @ G.T
        self.P = 0.5 * (self.P + self.P.T)

    def get_estimated_state(self):
        """Mirror of sim.get_true_state(); B3 flies on THIS, not ground truth."""
        return {
            "pos": self.p.copy(), "vel": self.v.copy(), "quat": self.q.copy(),
            "accel_bias": self.ab.copy(), "gyro_bias": self.gb.copy(),
        }

    @property
    def sigma_pos(self):
        """1-sigma position uncertainty, as a 3D radius."""
        return float(np.sqrt(np.trace(self.P[P_, P_])))

    @property
    def sigma_pos_axes(self):
        return np.sqrt(np.diag(self.P)[P_])
