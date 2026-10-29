"""Robot pose and heading normalization."""

from dataclasses import dataclass
from math import atan2, cos, hypot, isfinite, pi, sin


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    theta: float

    def __post_init__(self):
        if not all(isfinite(v) for v in (self.x, self.y, self.theta)):
            raise ValueError("Pose components must be finite")


def normalize_angle(theta: float) -> float:
    if not isfinite(theta):
        raise ValueError("Angle must be finite")
    return (theta + pi) % (2.0 * pi) - pi


def integrate(pose: Pose, linear: float, angular: float, dt: float) -> Pose:
    if not all(isfinite(v) for v in (linear, angular, dt)) or dt < 0:
        raise ValueError("Velocities and dt must be finite; dt must be nonnegative")
    return Pose(pose.x + linear * cos(pose.theta) * dt,
                pose.y + linear * sin(pose.theta) * dt,
                normalize_angle(pose.theta + angular * dt))



def quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    """Normalize an odometry quaternion and return its planar heading."""
    norm = hypot(x, y, z, w)
    if not isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid odometry quaternion")
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

