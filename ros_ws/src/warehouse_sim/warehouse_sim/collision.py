"""Continuous swept-circle collision queries against static line segments."""

from math import hypot, isfinite

from .kinematics import Pose
from .robot import Status
from .warehouse_map import WarehouseMap


def point_segment_distance(p, a, b) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    denominator = dx * dx + dy * dy
    t = 0.0 if denominator == 0 else max(0.0, min(1.0,
        ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / denominator))
    return hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy)


def _cross(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def segment_distance(a, b, c, d) -> float:
    # Strict crossings; collinear/endpoint contact is covered by distances below.
    if _cross(a, b, c) * _cross(a, b, d) < 0 and _cross(c, d, a) * _cross(c, d, b) < 0:
        return 0.0
    return min(point_segment_distance(a, c, d), point_segment_distance(b, c, d),
               point_segment_distance(c, a, b), point_segment_distance(d, a, b))


class CollisionChecker:
    def __init__(self, warehouse_map: WarehouseMap):
        self.map = warehouse_map

    def check_motion(self, start: Pose, end: Pose, radius: float) -> Status:
        if not isfinite(radius) or radius <= 0:
            raise ValueError("Collision radius must be finite and positive")
        b = self.map.bounds
        # A rectangular admissible region is convex: checking both endpoints
        # guarantees the entire centre segment remains inside the inset bounds.
        for pose in (start, end):
            if not (b.min_x + radius <= pose.x <= b.max_x - radius and
                    b.min_y + radius <= pose.y <= b.max_y - radius):
                return Status.OUT_OF_BOUNDS
        a, z = (start.x, start.y), (end.x, end.y)
        for wall in self.map.walls:
            if segment_distance(a, z, wall.start, wall.end) <= radius:
                return Status.COLLISION
        return Status.OK
