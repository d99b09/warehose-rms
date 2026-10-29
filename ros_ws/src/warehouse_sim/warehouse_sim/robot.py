"""Robot model needed for creation in the shared warehouse world."""

from dataclasses import dataclass
from enum import StrEnum
from math import hypot, isfinite

from .kinematics import Pose, normalize_angle


class Status(StrEnum):
    OK = 'OK'
    COLLISION = 'COLLISION'
    OUT_OF_BOUNDS = 'OUT_OF_BOUNDS'


@dataclass(frozen=True)
class RobotConfig:
    length: float = 0.6
    width: float = 0.4
    wheel_radius: float = 0.1
    wheel_base: float = 0.4
    max_linear_speed: float = 0.6
    max_angular_speed: float = 1.5
    cmd_timeout: float = 0.5

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if not isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')


@dataclass(frozen=True)
class CollisionInfo:
    status: Status
    attempted_pose: Pose
    time: float


@dataclass
class Robot:
    robot_id: str
    x: float
    y: float
    theta: float = 0.0
    config: RobotConfig = RobotConfig()
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    state: Status = Status.OK
    last_command_time: float | None = None
    last_collision: CollisionInfo | None = None

    def __post_init__(self):
        if not self.robot_id:
            raise ValueError('robot_id must be nonempty')
        self.set_pose(Pose(self.x, self.y, self.theta))

    @property
    def pose(self) -> Pose:
        return Pose(self.x, self.y, self.theta)

    @property
    def collision_radius(self) -> float:
        return hypot(self.config.length, self.config.width) / 2

    def set_pose(self, pose: Pose):
        self.x, self.y, self.theta = pose.x, pose.y, normalize_angle(pose.theta)

    def set_velocity(self, linear: float, angular: float, now: float):
        if not all(isfinite(value) for value in (linear, angular, now)):
            raise ValueError('Velocity command and timestamp must be finite')
        self.linear_velocity = max(-self.config.max_linear_speed,
                                   min(self.config.max_linear_speed, linear))
        self.angular_velocity = max(-self.config.max_angular_speed,
                                    min(self.config.max_angular_speed, angular))
        self.last_command_time = now

    def stop(self):
        self.linear_velocity = self.angular_velocity = 0.0

    def apply_watchdog(self, now: float):
        if self.last_command_time is None or now - self.last_command_time >= self.config.cmd_timeout:
            self.stop()
