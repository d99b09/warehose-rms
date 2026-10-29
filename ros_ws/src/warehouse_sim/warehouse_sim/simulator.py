"""Shared robot registry and creation in the warehouse world."""

from math import hypot, isfinite
import time

from .collision import CollisionChecker, segment_distance
from .kinematics import Pose, integrate
from .robot import CollisionInfo, Robot, RobotConfig, Status
from .warehouse_map import WarehouseMap


class KinematicSimulator:
    def __init__(self, warehouse_map: WarehouseMap, config: RobotConfig | None = None,
                 clock=time.monotonic):
        self.map = warehouse_map
        self.clock = clock
        self.config = config or RobotConfig()
        self.collision = CollisionChecker(warehouse_map)
        self.robots: dict[str, Robot] = {}

    def spawn_robot(self, robot_id: str, pose: Pose) -> Robot:
        if robot_id in self.robots:
            raise ValueError(f'Duplicate robot_id: {robot_id}')
        robot = Robot(robot_id, pose.x, pose.y, pose.theta, self.config)
        status = self.collision.check_motion(robot.pose, robot.pose, robot.collision_radius)
        if status != Status.OK:
            raise ValueError(f'Unsafe spawn pose: {status}')
        if any(hypot(robot.x - other.x, robot.y - other.y)
               <= robot.collision_radius + other.collision_radius
               for other in self.robots.values()):
            raise ValueError('Unsafe spawn pose: robot overlap')
        self.robots[robot_id] = robot
        return robot

    def set_velocity(self, robot_id: str, linear: float, angular: float):
        self.robots[robot_id].set_velocity(linear, angular, self.clock())

    def step(self, dt: float):
        if not isfinite(dt) or dt <= 0:
            raise ValueError("Simulation dt must be finite and positive")
        now = self.clock()
        candidates, states, attempted = {}, {}, {}
        for rid, robot in self.robots.items():
            robot.apply_watchdog(now)
            candidate = integrate(robot.pose, robot.linear_velocity, robot.angular_velocity, dt)
            attempted[rid] = candidate
            status = self.collision.check_motion(robot.pose, candidate, robot.collision_radius)
            candidates[rid] = candidate if status == Status.OK else Pose(robot.x,robot.y,candidate.theta)
            states[rid] = status
        # Treat both swept segments as occupied for the whole step. Reject both
        # translations on conflict; iterate after rejections because a stopped
        # robot can obstruct a third robot's candidate trajectory.
        changed = True
        while changed:
            changed = False
            items = list(self.robots.items())
            for i,(aid,a) in enumerate(items):
                for bid,b in items[i+1:]:
                    ap,bp = candidates[aid],candidates[bid]
                    if segment_distance((a.x,a.y),(ap.x,ap.y),(b.x,b.y),(bp.x,bp.y)) <= a.collision_radius+b.collision_radius:
                        for rid,robot in ((aid,a),(bid,b)):
                            c = candidates[rid]
                            if (c.x,c.y) != (robot.x,robot.y):
                                candidates[rid] = Pose(robot.x,robot.y,c.theta)
                                changed = True
                            states[rid] = Status.COLLISION
        for rid,robot in self.robots.items():
            candidate,status = candidates[rid],states[rid]
            moved = candidate != robot.pose
            robot.set_pose(candidate)
            if status != Status.OK:
                robot.linear_velocity = 0.0
                robot.state = status
                robot.last_collision = CollisionInfo(status,attempted[rid],now)
            elif moved:
                robot.state = Status.OK
