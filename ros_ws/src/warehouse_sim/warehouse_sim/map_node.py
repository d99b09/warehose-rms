"""Publish the warehouse map as retained RViz markers."""

import json
import signal
import subprocess
import sys
from math import atan2, cos, sin
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import MarkerArray
from warehouse_interfaces.srv import SpawnRobot

from .kinematics import Pose
from .simulator import KinematicSimulator
from .visualization import map_markers, robot_markers
from .warehouse_map import WarehouseMap


def main(args=None):
    """Load and publish the map, keeping it available to late subscribers."""
    shutdown_requested = False

    def request_shutdown(signum, frame):
        nonlocal shutdown_requested
        shutdown_requested = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    agents = {}
    adapters = {}
    try:
        node = Node('warehouse_map')
        share = Path(get_package_share_directory('warehouse_sim'))
        default = share / 'maps/nav/warehouse_normal/0.yaml'
        path = node.declare_parameter('map_path', str(default)).value
        vda_config = node.declare_parameter('vda_config_path', str(Path(get_package_share_directory('vda_bridge')) / 'config/vda5050.yaml')).value
        warehouse = WarehouseMap.from_yaml(path)
        simulator = KinematicSimulator(warehouse)
        operations = {}
        odom_publishers = {}
        robot_publishers = {}
        status_publishers = {}
        tf_broadcaster = TransformBroadcaster(node)
        static_tf_broadcaster = StaticTransformBroadcaster(node)
        static_transforms = []
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        inventory_publisher = node.create_publisher(String, '/warehouse/robot_inventory', retained)

        def publish_inventory():
            inventory = [dict(robot_id=robot.robot_id, namespace=f'/{robot.robot_id}',
                              length=robot.config.length, width=robot.config.width)
                         for robot in simulator.robots.values()]
            inventory_publisher.publish(String(data=json.dumps(inventory)))

        def publish_odometry():
            stamp = node.get_clock().now().to_msg()
            for robot_id, robot in simulator.robots.items():
                message = Odometry()
                message.header.stamp = stamp
                message.header.frame_id = f'{robot_id}/odom'
                message.child_frame_id = f'{robot_id}/base_link'
                message.pose.pose.position.x = robot.x
                message.pose.pose.position.y = robot.y
                message.pose.pose.orientation.z = sin(robot.theta / 2)
                message.pose.pose.orientation.w = cos(robot.theta / 2)
                message.twist.twist.linear.x = robot.linear_velocity
                message.twist.twist.angular.z = robot.angular_velocity
                odom_publishers[robot_id].publish(message)
                status_publishers[robot_id].publish(String(data=robot.state.value))
                transform = TransformStamped()
                transform.header = message.header
                transform.child_frame_id = message.child_frame_id
                transform.transform.translation.x = robot.x
                transform.transform.translation.y = robot.y
                transform.transform.rotation = message.pose.pose.orientation
                tf_broadcaster.sendTransform(transform)
                robot_publishers[robot_id].publish(robot_markers(robot, message.child_frame_id, stamp))

        def receive_command(robot_id, message):
            try:
                simulator.set_velocity(robot_id, message.linear.x, message.angular.z)
            except ValueError as error:
                simulator.robots[robot_id].stop()
                node.get_logger().warning(f'Rejected cmd_vel for {robot_id}: {error}')

        def step():
            simulator.step(0.02)

        publish_inventory()
        node.create_timer(0.1, publish_odometry)
        steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        node.create_timer(0.02, step, clock=steady_clock)

        def spawn_robot(request, response):
            response.operation_id = request.operation_id
            response.http_status, response.status = 409, 'FAILED'
            if request.operation_id in operations:
                parking, previous = operations[request.operation_id]
                if not request.query and parking != request.parking_node_id:
                    response.message = 'operation_id already used with another parking'
                    return response
                return previous
            if request.query:
                response.http_status = 404
                response.message = 'Unknown operation_id'
                return response
            try:
                if not request.operation_id:
                    raise ValueError('operation_id is required')
                candidates = ([request.parking_node_id] if request.parking_node_id
                              else sorted(warehouse.parking_nodes))
                for parking in candidates:
                    if parking not in warehouse.parking_nodes:
                        raise ValueError(f'Unknown parking: {parking}')
                    origin = warehouse.nodes[parking]
                    neighbor = next((edge.target if edge.source == parking else edge.source
                                     for edge in warehouse.edges
                                     if parking in (edge.source, edge.target)), None)
                    target = warehouse.nodes[neighbor] if neighbor else origin
                    theta = atan2(target.y - origin.y, target.x - origin.x)
                    try:
                        robot = simulator.spawn_robot(
                            f'robot_{len(simulator.robots) + 1:02d}',
                            Pose(origin.x, origin.y, theta))
                    except ValueError:
                        if request.parking_node_id:
                            raise
                        continue
                    odom = marker = status = command = None
                    try:
                        odom = node.create_publisher(Odometry, f'/{robot.robot_id}/odom', 10)
                        marker = node.create_publisher(
                            MarkerArray, f'/{robot.robot_id}/robot_markers', 10)
                        status = node.create_publisher(
                            String, f'/{robot.robot_id}/status', retained)
                        command = node.create_subscription(
                            Twist, f'/{robot.robot_id}/cmd_vel',
                            lambda message, rid=robot.robot_id: receive_command(rid, message), 1)
                        executable = Path(get_package_prefix('robot_agent')) / 'lib/robot_agent/agent'
                        agents[robot.robot_id] = subprocess.Popen([
                            sys.executable, str(executable), '--ros-args', '-r', f'__ns:=/{robot.robot_id}',
                            '-p', f'map_path:={path}'])
                        adapter_executable = Path(get_package_prefix('vda_bridge')) / 'lib/vda_bridge/robot_adapter'
                        adapters[robot.robot_id] = subprocess.Popen([
                            sys.executable, str(adapter_executable), '--ros-args', '-r', f'__ns:=/{robot.robot_id}',
                            '-p', f'map_path:={path}', '-p', f'vda_config_path:={vda_config}'])
                        transform = TransformStamped()
                        transform.header.stamp = node.get_clock().now().to_msg()
                        transform.header.frame_id = 'map'
                        transform.child_frame_id = f'{robot.robot_id}/odom'
                        transform.transform.rotation.w = 1.0
                        static_tf_broadcaster.sendTransform([*static_transforms, transform])
                        static_transforms.append(transform)
                        odom_publishers[robot.robot_id] = odom
                        robot_publishers[robot.robot_id] = marker
                        status_publishers[robot.robot_id] = status
                    except Exception:
                        adapter = adapters.pop(robot.robot_id, None)
                        if adapter is not None:
                            adapter.terminate()
                            adapter.wait()
                        agent = agents.pop(robot.robot_id, None)
                        if agent is not None:
                            agent.terminate()
                            agent.wait()
                        if command is not None:
                            node.destroy_subscription(command)
                        if status is not None:
                            node.destroy_publisher(status)
                        if marker is not None:
                            node.destroy_publisher(marker)
                        if odom is not None:
                            node.destroy_publisher(odom)
                        simulator.robots.pop(robot.robot_id, None)
                        raise
                    publish_inventory()
                    response.accepted = True
                    response.http_status, response.status = 200, 'CREATED'
                    response.robot_id, response.parking_node_id = robot.robot_id, parking
                    response.message = f'Робот {robot.robot_id} создан на парковке {parking}'
                    node.get_logger().info(response.message)
                    break
                else:
                    raise ValueError('No free parking available')
            except (ValueError, RuntimeError, OSError) as error:
                response.message = str(error)
            operations[request.operation_id] = (request.parking_node_id, response)
            return response

        node.create_service(SpawnRobot, '/warehouse/spawn_robot', spawn_robot)
        publisher = node.create_publisher(
            MarkerArray, '/warehouse/map_markers',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        publisher.publish(map_markers(warehouse))
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        for agent in [*agents.values(), *adapters.values()]:
            if agent.poll() is None:
                agent.terminate()
        for agent in [*agents.values(), *adapters.values()]:
            try:
                agent.wait(timeout=3)
            except subprocess.TimeoutExpired:
                agent.kill()
                agent.wait()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
