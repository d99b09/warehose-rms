"""Manual-mode control and local watchdogs, independent of ROS."""

from dataclasses import dataclass
from math import hypot, isfinite, pi

from warehouse_sim.kinematics import Pose, normalize_angle

from .route_validator import RouteError, validate_route
from .route_executor import RouteExecutor


@dataclass(frozen=True)
class AgentConfig:
    position_tolerance: float = 0.15
    angle_tolerance: float = 0.10
    linear_gain: float = 0.8
    angular_gain: float = 2.0
    rotate_threshold: float = 0.6
    max_linear_speed: float = 0.5
    max_angular_speed: float = 1.2
    control_frequency: float = 20.0
    odom_timeout: float = 1.0
    manual_timeout: float = 0.5
    stuck_timeout: float = 3.0
    stuck_min_distance: float = 0.05
    stuck_min_rotation: float = 0.05

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = getattr(self, name)
            if isinstance(value, bool) or not isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')
        if not self.angle_tolerance < self.rotate_threshold < pi / 2:
            raise ValueError('Require angle_tolerance < rotate_threshold < pi/2')
        if self.max_linear_speed > 0.6 or self.max_angular_speed > 1.5:
            raise ValueError('Agent speeds exceed simulator limits')


@dataclass(frozen=True)
class Command:
    linear: float = 0.0
    angular: float = 0.0


class AgentCore:
    def __init__(self, config=AgentConfig(), warehouse=None):
        self.config = config
        self.warehouse = warehouse
        self.state = 'IDLE'
        self.command = Command()
        self.pose = self.odom_time = self.manual_time = None
        self.error_code = self.message = ''
        self.collision = False
        self.route = None
        self.order_id = ''
        self.outcome = None
        from .navigation_controller import NavigationController
        self.controller = NavigationController(config)
        self._reset_progress()

    def validate_route(self, order_id, node_ids, now):
        if not order_id:
            raise RouteError('INVALID_ROUTE', 'Order ID must not be empty')
        if self.odom_time is None or now - self.odom_time >= self.config.odom_timeout:
            raise RouteError('ODOM_TIMEOUT', 'No fresh odometry received')
        return validate_route(self.warehouse, node_ids, self.pose, self.config.position_tolerance)

    def start_route(self, order_id, node_ids, now):
        nodes = self.validate_route(order_id, node_ids, now)
        self.route = RouteExecutor(self.warehouse, nodes)
        self.order_id = order_id
        self.outcome = None
        self.error_code = self.message = ''
        self.command = Command()
        self.state = 'MOVING'
        self._reset_progress()

    def _reset_progress(self):
        self.anchor = self.progress_time = self.progress_mode = None

    def odometry(self, pose: Pose, now: float):
        self.pose, self.odom_time = pose, now

    def stop(self, code='STOPPED', message='Stop service requested', *, error=False):
        if self.route is not None and self.outcome is None:
            self.outcome = (False, code, message)
        self.error_code, self.message = code, message
        self.state = 'ERROR' if error else 'STOPPED'
        self.command = Command()
        self.manual_time = None
        self._reset_progress()

    def manual_mode(self, enabled):
        if enabled:
            if self.state != 'MANUAL':
                self.stop('MANUAL_OVERRIDE', 'Manual mode requested')
                self.state = 'MANUAL'
        elif self.state in {'MANUAL', 'STOPPED'}:
            self.state = 'IDLE'
            self.command = Command()
            self.error_code = ''
            self.message = 'Manual mode disabled'
            self.manual_time = None
            self._reset_progress()

    def manual_command(self, linear, angular, now):
        if self.state != 'MANUAL':
            return
        if not all(isfinite(value) for value in (linear, angular)):
            self.command = Command()
            self.manual_time = None
            return
        c = self.config
        self.command = Command(max(-c.max_linear_speed, min(c.max_linear_speed, linear)),
                               max(-c.max_angular_speed, min(c.max_angular_speed, angular)))
        self.manual_time = now

    def _safety_check(self, now):
        if self.pose is None or self.odom_time is None or now - self.odom_time >= self.config.odom_timeout:
            return 'ODOM_TIMEOUT'
        if self.collision:
            return 'COLLISION'
        mode = ('translation' if abs(self.command.linear) > 1e-6 else
                'rotation' if abs(self.command.angular) > 1e-6 else None)
        if mode is None:
            self._reset_progress()
        else:
            progress = self.anchor is not None and (
                hypot(self.pose.x - self.anchor.x, self.pose.y - self.anchor.y)
                >= self.config.stuck_min_distance if mode == 'translation' else
                abs(normalize_angle(self.pose.theta - self.anchor.theta))
                >= self.config.stuck_min_rotation)
            if self.anchor is None or mode != self.progress_mode or progress:
                self.anchor, self.progress_time, self.progress_mode = self.pose, now, mode
            elif now - self.progress_time >= self.config.stuck_timeout:
                return 'STUCK'
        return ''

    def tick(self, now):
        if self.state in {'MANUAL', 'MOVING', 'ROTATING', 'WAITING'}:
            error = self._safety_check(now)
            if error:
                self.stop(error, f'Safety monitor: {error}', error=True)
            elif self.state == 'MANUAL':
                if self.manual_time is None or now - self.manual_time >= self.config.manual_timeout:
                    self.command = Command()
            elif self.state != 'WAITING' and self.route is not None and self.outcome is None:
                self.route.advance(self.pose, self.config.position_tolerance)
                if self.route.done:
                    self.command = Command()
                    self.state = 'PARKED' if self.route.current_node_id in self.warehouse.parking_nodes else 'IDLE'
                    self.message = 'Route completed'
                    self.outcome = (True, '', self.message)
                    self._reset_progress()
                else:
                    node = self.warehouse.nodes[self.route.target_node_id]
                    self.command, self.state, _ = self.controller.compute(
                        self.pose, (node.x, node.y), rotating=self.state == 'ROTATING')
        return self.command
