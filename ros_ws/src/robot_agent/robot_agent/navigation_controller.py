"""Point P controller. All distances and speeds use SI."""
from math import atan2, cos, hypot

from warehouse_sim.kinematics import Pose, normalize_angle

from .core import AgentConfig, Command


class NavigationController:
    def __init__(self, config=AgentConfig()):
        self.config = config

    def compute(self, pose: Pose, target: tuple[float, float], rotating=False):
        c = self.config
        dx, dy = target[0] - pose.x, target[1] - pose.y
        distance = hypot(dx, dy)
        if distance <= c.position_tolerance:
            return Command(), "REACHED", distance
        error = normalize_angle(atan2(dy, dx) - pose.theta)
        angular = max(-c.max_angular_speed, min(c.max_angular_speed, c.angular_gain * error))
        if abs(error) > (c.angle_tolerance if rotating else c.rotate_threshold):
            return Command(0.0, angular), "ROTATING", distance
        linear = min(c.max_linear_speed, c.linear_gain * distance) * max(0.0, cos(error))
        return Command(linear, angular), "MOVING", distance
