"""Quaternion / rotation helpers for SkyScout navigation.

Conventions, all verified by measurement, not assumed:
  * quaternions are (w, x, y, z), matching Genesis get_quat()
  * R = quat_to_R(q) maps BODY -> WORLD
  * Genesis get_ang() is WORLD-frame angular velocity (body-frame residual
    2.5e-1 vs world 1.4e-3 when integrating the quaternion both ways)
"""
import numpy as np


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def qnorm(q):
    n = np.linalg.norm(q)
    q = q / n
    return -q if q[0] < 0 else q


def quat_to_R(q):
    """Body -> world rotation matrix."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def quat_from_small_angle(dtheta):
    """Small rotation vector -> quaternion. Used for ESKF error injection."""
    half = 0.5 * np.asarray(dtheta, float)
    n2 = half @ half
    if n2 < 1e-12:
        return qnorm(np.array([1.0, half[0], half[1], half[2]]))
    n = np.sqrt(n2)
    return np.array([np.cos(n), *(np.sin(n) / n * half)])


def quat_integrate_world(q, omega_world, dt):
    """Advance a body->world quaternion by a WORLD-frame angular rate."""
    return qnorm(q + 0.5 * qmul(np.r_[0.0, np.asarray(omega_world, float)], q) * dt)


def quat_integrate_body(q, omega_body, dt):
    """Advance a body->world quaternion by a BODY-frame angular rate."""
    return qnorm(q + 0.5 * qmul(q, np.r_[0.0, np.asarray(omega_body, float)]) * dt)


def quat_angle_between(qa, qb):
    """Smallest rotation angle (rad) between two attitudes."""
    d = abs(float(np.dot(qnorm(qa), qnorm(qb))))
    return 2.0 * np.arccos(np.clip(d, -1.0, 1.0))
