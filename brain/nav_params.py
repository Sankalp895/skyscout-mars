"""All B2 navigation parameters in one tunable place (B2.3)."""
from dataclasses import dataclass, field


@dataclass
class IMUParams:
    """Plausible mid-grade MEMS values."""
    rate_hz: float = 240.0

    gyro_noise_density: float = 3.0e-4
    gyro_bias_rw: float = 3.0e-5
    accel_noise_density: float = 3.0e-3
    accel_bias_rw: float = 3.0e-4

    gyro_bias0: tuple = (0.0040, -0.0030, 0.0020)
    accel_bias0: tuple = (0.050, -0.030, 0.040)

    seed: int = 7


@dataclass
class FixParams:
    """Low-rate noisy position fix -- a stand-in for a terrain-relative or visual localization meas."""
    rate_hz: float = 1.0
    sigma_m: float = 0.50
    seed: int = 11


@dataclass
class ESKFParams:
    """Initial uncertainty and process noise."""
    sigma_p0: float = 1e-3
    sigma_v0: float = 1e-3
    sigma_th0: float = 1e-3
    sigma_ab0: float = 0.10
    sigma_gb0: float = 0.01


@dataclass
class MapParams:
    """Sensor model for the B4 world-map fusion."""
    detect_threshold: float = 0.95
    p_detect_given_hazard: float = 0.9978
    p_detect_given_safe: float = 0.02297
    l_per_obs_clamp: float = 2.5
    l_total_clamp: float = 10.0
