"""Multi-robot ROS registry with VDA5050/MQTT dispatch."""
import json
import signal
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from warehouse_interfaces.msg import RobotState
from warehouse_interfaces.srv import SubmitOrder
from warehouse_sim.warehouse_map import WarehouseMap
from vda_bridge.config import load_config
from vda_bridge.master_adapter import MasterAdapter
from .core import FleetCore


class DispatcherNode(Node):
    def __init__(self):
        super().__init__('fleet_dispatcher')
        default = str(Path(get_package_share_directory('warehouse_sim')) / 'maps/nav/warehouse_normal/0.yaml')
        warehouse = WarehouseMap.from_yaml(self.declare_parameter('map_path', default).value)
        config_default = str(Path(get_package_share_directory('vda_bridge')) / 'config/vda5050.yaml')
        config = load_config(self.declare_parameter('vda_config_path', config_default).value)
        self.transport = MasterAdapter(warehouse, config)
        self.core = FleetCore(warehouse, self.transport)
        self.transport.core = self.core
        self.subscriptions_by_robot = {}
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/warehouse/robot_inventory', self._inventory, retained)
        self.publisher = self.create_publisher(String, '/fleet/state', retained)
        self.create_service(SubmitOrder, '/fleet/submit_order', self._submit)
        self.create_timer(.1, self._tick)

    def _inventory(self, message):
        try:
            robots = json.loads(message.data)
            for item in robots:
                rid, ns = item['robot_id'], item['namespace']
                self.core.register(rid, item['length'], item['width'])
                self.transport.register(rid)
                if rid in self.subscriptions_by_robot:
                    continue
                odom = self.create_subscription(Odometry, ns+'/odom',
                    lambda msg, rid=rid: self._odom(rid, msg),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                agent = self.create_subscription(RobotState, ns+'/agent_state',
                    lambda msg, rid=rid: self.core.agent(rid, msg.status, msg.agent_epoch, msg.sequence),
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                self.subscriptions_by_robot[rid] = (odom, agent)
        except (ValueError, TypeError, KeyError) as error:
            self.get_logger().warning(f'Invalid inventory: {error}')

    def _odom(self, rid, msg):
        p, v = msg.pose.pose.position, msg.twist.twist
        self.core.odometry(rid, p.x, p.y, v.linear.x, v.angular.z,
                           msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec)

    def _submit(self, request, response):
        accepted, code, message, order = self.core.submit(request.robot_id,
            request.target_node_id, request.order_type, request.order_id)
        response.accepted, response.http_status, response.message = accepted, code, message
        response.order_id = order.order_id if order else ''
        response.route = order.route if order else []
        return response

    def _tick(self):
        self.transport.poll()
        self.core.tick()
        self.publisher.publish(String(data=json.dumps(self.core.snapshot(), ensure_ascii=False)))

    def destroy_node(self):
        self.transport.close()
        return super().destroy_node()


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
        node = DispatcherNode()
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
