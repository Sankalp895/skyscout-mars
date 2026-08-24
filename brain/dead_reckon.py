"""B2 step 2 -- strapdown dead-reckoning."""
import numpy as np

from nav_math import qmul, qnorm, quat_to_R, quat_from_small_angle

G_WORLD = np.array([0.0, 0.0, -3.72])


def strapdown_step(p, v, q, accel_body, gyro_body, dt):
    """THE mechanization."""
    dq = quat_from_small_angle(np.asarray(gyro_body, float) * dt)
    q_new = qnorm(qmul(q, dq))
    R_mid = quat_to_R(qnorm(0.5 * (q + q_new)))
    a_w = R_mid @ np.asarray(accel_body, float) + G_WORLD
    p_new = p + v * dt + 0.5 * a_w * dt * dt
    v_new = v + a_w * dt
    return p_new, v_new, q_new, a_w


class DeadReckoner:
    def __init__(self, pos, vel, quat):
        self.p = np.asarray(pos, float).copy()
        self.v = np.asarray(vel, float).copy()
        self.q = qnorm(np.asarray(quat, float).copy())

    def propagate(self, accel_body, gyro_body, dt):
        """One IMU sample. gyro_body and accel_body are BODY frame."""
        self.p, self.v, self.q, a_w = strapdown_step(
            self.p, self.v, self.q, accel_body, gyro_body, dt)
        return a_w

    @property
    def state(self):
        return self.p.copy(), self.v.copy(), self.q.copy()
