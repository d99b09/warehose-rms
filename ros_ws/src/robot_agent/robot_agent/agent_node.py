"""ROS adapter for one robot in manual mode."""

from dataclasses import fields
from pathlib import Path
import signal
import time
from uuid import uuid4

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.task import Future
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from warehouse_interfaces.action import FollowRoute
from warehouse_interfaces.msg import RobotState
from warehouse_sim.kinematics import Pose, quaternion_yaw
from warehouse_sim.warehouse_map import WarehouseMap

from .core import AgentConfig, AgentCore
from .route_validator import RouteError


class AgentNode(Node):
    def __init__(self):
        super().__init__('robot_agent')
        self.declare_parameter('robot_id', '')
        self.robot_id = self.get_parameter('robot_id').value or self.get_namespace().strip('/')
        if not self.robot_id:
            raise ValueError('Launch Agent inside a robot namespace, or provide robot_id')
        defaults = AgentConfig()
        for field in fields(AgentConfig):
            self.declare_parameter(field.name, getattr(defaults, field.name))
        config = AgentConfig(**{field.name: self.get_parameter(field.name).value
                                for field in fields(AgentConfig)})
        for name in ('map_path', 'building_path', 'level_name'):
            self.declare_parameter(name, '')
        path = self.get_parameter('map_path').value or str(
            Path(get_package_share_directory('warehouse_sim')) / 'maps/nav/warehouse_normal/0.yaml')
        warehouse = WarehouseMap.from_yaml(
            path, building_path=self.get_parameter('building_path').value or None,
            level_name=self.get_parameter('level_name').value or None)
        self.core = AgentCore(config, warehouse)
        self.agent_epoch = str(uuid4())
        self.state_sequence = 0
        self.odom_sequence = -1
        self.reserved = False
        self.goal_handle = self.done_future = None
        self.pending_stop = None
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 1)
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(RobotState, 'agent_state', retained)
        self.create_subscription(Odometry, 'odom', self._odom,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String, 'status', self._status, retained)
        self.create_subscription(Twist, 'manual_cmd_vel', self._manual, 1)
        self.create_service(Trigger, 'stop', self._stop)
        self.create_service(SetBool, 'set_traffic_wait', self._traffic_wait)
        self.create_service(SetBool, 'set_manual_mode', self._set_manual)
        self.server = ActionServer(self, FollowRoute, 'follow_route', self._execute,
            goal_callback=self._goal, cancel_callback=self._cancel,
            handle_accepted_callback=self._accepted)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1 / config.control_frequency, self._tick, clock=self.steady_clock)
        self._publish_command()
        self._publish_state()

    def _publish_command(self):
        message = Twist()
        message.linear.x = float(self.core.command.linear)
        message.angular.z = float(self.core.command.angular)
        self.velocity_pub.publish(message)

    def _publish_state(self):
        self.state_sequence += 1
        route = self.core.route
        self.state_pub.publish(RobotState(
            robot_id=self.robot_id, status=self.core.state,
            error_code=self.core.error_code, message=self.core.message,
            order_id=self.core.order_id,
            current_node_id=route.current_node_id if route else '',
            target_node_id=route.target_node_id if route and self.core.outcome is None else '',
            completed_nodes=route.completed if route else 0,
            total_nodes=len(route.nodes) if route else 0,
            timestamp=self.get_clock().now().to_msg(), agent_epoch=self.agent_epoch,
            sequence=self.state_sequence, odom_sequence=self.odom_sequence))

    def _odom(self, message):
        try:
            position, rotation = message.pose.pose.position, message.pose.pose.orientation
            pose = Pose(position.x, position.y,
                        quaternion_yaw(rotation.x, rotation.y, rotation.z, rotation.w))
        except ValueError:
            return
        sequence = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        if sequence <= self.odom_sequence:
            return
        self.odom_sequence = sequence
        self.core.odometry(pose, time.monotonic())

    def _status(self, message):
        self.core.collision = message.data in ('COLLISION', 'OUT_OF_BOUNDS')
        if self.core.collision:
            self.core.tick(time.monotonic())
            self._publish_command()

    def _goal(self, request):
        if self.reserved or self.core.state == 'MANUAL':
            return GoalResponse.REJECT
        try:
            self.core.validate_route(request.order_id, request.node_ids, time.monotonic())
        except RouteError as error:
            self.get_logger().warning(f'Rejected route: {error.code}: {error}')
            self.core.stop(error.code, str(error), error=True)
            self._publish_state()
            return GoalResponse.REJECT
        self.reserved = True
        self.pending_stop = None
        return GoalResponse.ACCEPT

    def _accepted(self, handle):
        self.goal_handle, self.done_future = handle, Future()
        try:
            self.core.start_route(handle.request.order_id, handle.request.node_ids, time.monotonic())
            if self.pending_stop:
                self.core.stop(*self.pending_stop)
        except RouteError as error:
            self.core.stop(error.code, str(error), error=True)
            self.core.outcome = (False, error.code, str(error))
        self.pending_stop = None
        self._publish_state()
        handle.execute()

    async def _execute(self, handle):
        success, code, message = await self.done_future
        if handle.is_cancel_requested:
            success, code, message = False, 'CANCELED', 'Action canceled by client'
            handle.canceled()
        elif success:
            handle.succeed()
        else:
            handle.abort()
        result = FollowRoute.Result(success=success, error_code=code, message=message,
            final_node_id=self.core.route.current_node_id if self.core.route else '')
        self.goal_handle = self.done_future = None
        self.reserved = False
        return result

    def _cancel(self, handle):
        if handle != self.goal_handle or self.done_future.done():
            return CancelResponse.REJECT
        self.core.stop('CANCELED', 'Action canceled by client')
        self._publish_command()
        self._publish_state()
        return CancelResponse.ACCEPT

    def _stop(self, request, response):
        if self.reserved and self.goal_handle is None:
            self.pending_stop = ('STOPPED', 'Stop service requested')
        self.core.stop()
        self._publish_command()
        self._publish_state()
        response.success, response.message = True, 'Robot stopped'
        return response

    def _set_manual(self, request, response):
        if request.data and self.reserved and self.goal_handle is None:
            self.pending_stop = ('MANUAL_OVERRIDE', 'Manual mode requested')
        self.core.manual_mode(request.data)
        self._publish_command()
        self._publish_state()
        response.success = True
        response.message = 'Manual mode enabled' if request.data else 'Manual mode disabled'
        return response

    def _traffic_wait(self, request, response):
        if self.reserved or self.core.state not in {'IDLE', 'PARKED', 'WAITING'}:
            response.success = False
            response.message = 'Agent is not idle'
            return response
        self.core.state = 'WAITING' if request.data else 'IDLE'
        self.core.command = type(self.core.command)()
        self.core._reset_progress()
        self._publish_command()
        self._publish_state()
        response.success = True
        response.message = 'Traffic wait updated'
        return response

    def _manual(self, message):
        if self.core.state != 'MANUAL':
            return
        now = time.monotonic()
        self.core.manual_command(message.linear.x, message.angular.z, now)
        self.core.tick(now)
        self._publish_command()

    def _tick(self):
        self.core.tick(time.monotonic())
        self._publish_command()
        self._publish_state()
        if self.goal_handle is not None:
            route = self.core.route
            if route is not None:
                distance = route.distance(self.core.pose) if self.core.pose is not None else 0.0
                self.goal_handle.publish_feedback(FollowRoute.Feedback(
                    current_node_id=route.current_node_id, completed_nodes=route.completed,
                    total_nodes=len(route.nodes), distance_to_next_node=float(distance),
                    state=self.core.state))
            if self.core.outcome is not None and not self.done_future.done():
                if self.core.outcome[1] != 'CANCELED' or self.goal_handle.is_cancel_requested:
                    self.done_future.set_result(self.core.outcome)

    def shutdown(self):
        self.core.stop('SHUTDOWN', 'Agent shutting down')
        if self.done_future is not None and not self.done_future.done():
            self.done_future.set_result(self.core.outcome)
        self._publish_command()
        self._publish_state()


def main(args=None):
    shutdown_requested = False

    def request_shutdown(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = AgentNode()
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.shutdown()
            node.destroy_node()
        rclpy.try_shutdown()
