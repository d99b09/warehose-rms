"""Robot-local MQTT queue; only a released edge is handed to the ROS agent."""
import json
import signal
from math import atan2, hypot, isfinite
from pathlib import Path
import time

from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool
from warehouse_interfaces.action import FollowRoute
from warehouse_interfaces.msg import RobotState
from warehouse_sim.warehouse_map import WarehouseMap
from protocols.vda5050.message_factory import MessageFactory
from protocols.vda5050.mapper import state_from_buffer
from protocols.vda5050.validator import ProtocolError, validate_instant
from .config import load_config, identity
from .journal import Journal
from .mqtt_client import MqttClient
from .order_buffer import OrderBuffer


class RobotAdapter(Node):
    def __init__(self):
        super().__init__('vda_robot_adapter')
        self.rid = self.declare_parameter('robot_id', self.get_namespace().strip('/')).value
        default = str(Path(get_package_share_directory('warehouse_sim'))/'maps/nav/warehouse_normal/0.yaml')
        self.map = WarehouseMap.from_yaml(self.declare_parameter('map_path', default).value)
        default_config = str(Path(get_package_share_directory('vda_bridge'))/'config/vda5050.yaml')
        self.config = load_config(self.declare_parameter('vda_config_path', default_config).value)
        self.factory = MessageFactory(identity(self.config, self.rid))
        self.buffer = OrderBuffer(self.map, self.config['simulation']['map_id'])
        self.journal = Journal(Path(self.config['state_directory'])/(self.rid+'.sqlite3'))
        self.journal.db.execute('CREATE TABLE IF NOT EXISTS checkpoint(id INTEGER PRIMARY KEY, payload TEXT)')
        self.errors, self.actions = [], {}
        row = self.journal.db.execute('SELECT payload FROM checkpoint WHERE id=1').fetchone()
        if row:
            saved = json.loads(row[0])
            self.buffer.order, self.buffer.completed, self.buffer.cancelled = saved['order'], saved['completed'], saved['cancelled']
            if self.buffer.active:
                self.buffer.failed = True
                self._error('adapterRestart', 'Adapter restarted; cancel old execution before a new order', fatal=True)
        self.client = MqttClient(self.config['mqtt'] | {'command_max_age': self.config['simulation']['command_max_age']},
            {self.rid: self.factory.identity}, 'robot-'+self.rid,
            will=(self.factory.identity, self.factory.connection('CONNECTIONBROKEN')))
        for kind in ('order', 'instantActions'):
            self.client.subscribe(self.rid, kind)
        self.telemetry = {}
        self.odom_at = self.agent_at = float('-inf')
        self.odom_sequence = self.agent_sequence = -1
        self.agent_epoch = ''
        self.retired_epochs = set()
        self.goal_handle = None
        self.goal_generation = 0
        self.result_pending = None
        self.cancel_action = None
        self.wait_future = None
        self.wait_value = None
        self.last_state = float('-inf')
        self.action_client = ActionClient(self, FollowRoute, 'follow_route')
        self.wait_client = self.create_client(SetBool, 'set_traffic_wait')
        self.create_subscription(Odometry, 'odom', self._odom,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(RobotState, 'agent_state', self._agent,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(.05, self._tick)
        self.client.start()

    def _save(self):
        with self.journal.db:
            self.journal.db.execute('INSERT OR REPLACE INTO checkpoint VALUES(1, ?)',
                (json.dumps(dict(order=self.buffer.order, completed=self.buffer.completed,
                                 cancelled=self.buffer.cancelled)),))

    def _error(self, kind, description, oid=None, fatal=False):
        error = dict(errorType=kind, errorDescription=description,
                     errorLevel='FATAL' if fatal else 'WARNING', errorReferences=[])
        if oid:
            error['errorReferences'].append(dict(referenceKey='orderId', referenceValue=oid))
        self.errors = [e for e in self.errors if e['errorType'] != kind]+[error]

    def _odom(self, msg):
        seq = msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        if seq <= self.odom_sequence or not all(isfinite(x) for x in (p.x, p.y, q.x, q.y, q.z, q.w, v.linear.x, v.angular.z)):
            return
        norm = hypot(q.x, q.y, q.z, q.w)
        if norm < 1e-12:
            return
        x, y, z, w = q.x/norm, q.y/norm, q.z/norm, q.w/norm
        self.telemetry.update(pose=(p.x, p.y, atan2(2*(w*z+x*y), 1-2*(y*y+z*z))),
                              linear=v.linear.x, angular=v.angular.z)
        self.odom_sequence, self.odom_at = seq, time.monotonic()

    def _agent(self, msg):
        if msg.robot_id != self.rid or msg.agent_epoch in self.retired_epochs:
            return
        if msg.agent_epoch == self.agent_epoch and msg.sequence <= self.agent_sequence:
            return
        if msg.agent_epoch != self.agent_epoch and self.agent_epoch:
            self.retired_epochs.add(self.agent_epoch)
            if self.buffer.active:
                self.goal_generation += 1
                self.goal_handle = self.result_pending = None
                self.buffer.inflight = False
                self.buffer.failed = True
                self._error('agentRestart', 'Agent restarted during execution', fatal=True)
        self.agent_epoch, self.agent_sequence, self.agent_at = msg.agent_epoch, msg.sequence, time.monotonic()
        self.telemetry.update(status=msg.status, current_node_id=msg.current_node_id,
                              collision=msg.error_code in {'COLLISION', 'OUT_OF_BOUNDS'})
        if msg.status == 'MANUAL' and self.buffer.active:
            self.buffer.failed = True
            self._error('manualOverride', 'Manual control interrupted order', fatal=True)

    def _node(self):
        if 'pose' not in self.telemetry:
            return ''
        x, y, _ = self.telemetry['pose']
        node = min(self.map.nodes.values(), key=lambda n: hypot(n.x-x, n.y-y))
        return node.node_id if hypot(node.x-x, node.y-y) <= .15 else ''

    def _fresh(self):
        now = time.monotonic()
        return now-self.odom_at < 1. and now-self.agent_at < 1.

    def _stopped(self):
        return (self._fresh() and abs(self.telemetry.get('linear', 0)) < .001
                and abs(self.telemetry.get('angular', 0)) < .001 and self._node()
                and self.telemetry.get('status') in {'IDLE', 'PARKED', 'WAITING', 'STOPPED'})

    def _order(self, message):
        oid = message['orderId']
        try:
            if not self._fresh() or self.cancel_action:
                raise ProtocolError('No fresh telemetry or cancellation in progress')
            if not self.buffer.validate(message, self._node()):
                return
            if not self.buffer.order or oid != self.buffer.order['orderId']:
                if not self._stopped():
                    raise ProtocolError('New order requires stopped robot at a node')
            key = json.dumps([oid, message['orderUpdateId']])
            if not self.journal.admit('order', key, message):
                raise ProtocolError('Previously admitted command cannot be replayed after restart')
            new_order = not self.buffer.order or oid != self.buffer.order['orderId']
            self.buffer.accept(message, self._node())
            self.errors = []
            if new_order:
                self.actions = {k: a for k, a in self.actions.items() if a['actionStatus'] == 'RUNNING'}
            self._save()
            self.journal.finish('order', key, 'ACCEPTED')
        except (ValueError, KeyError) as error:
            self._error('orderError', str(error), oid)

    def _instant(self, message):
        for action in message['actions']:
            aid, kind = action['actionId'], action['actionType']
            try:
                validate_instant(action)
                if not self.journal.admit('action', aid, action):
                    if aid in self.actions:
                        continue
                    previous = self.journal.state('action', aid)
                    self.actions[aid] = dict(actionId=aid, actionType=kind,
                        actionStatus=previous if previous in {'FINISHED', 'FAILED'} else 'FAILED',
                        resultDescription='Previously admitted action; not executed again')
                    continue
                self.actions[aid] = dict(actionId=aid, actionType=kind, actionStatus='RUNNING')
                if kind == 'factsheetRequest':
                    self.client.publish(self.rid, 'factsheet', self.factory.factsheet(self.config['simulation']))
                    self._action_done(aid, True)
                elif kind == 'cancelOrder':
                    if self.cancel_action:
                        self._action_done(aid, False, 'Cancellation already in progress')
                    elif not self.buffer.active:
                        self._action_done(aid, False, 'No active order to cancel')
                    else:
                        self.cancel_action = aid
                        if self.goal_handle:
                            self.goal_handle.cancel_goal_async()
                else:
                    self._action_done(aid, False, 'Unsupported action')
            except ValueError as error:
                self.actions[aid] = dict(actionId=aid, actionType=kind, actionStatus='FAILED', resultDescription=str(error))
                self._error('actionError', str(error))

    def _action_done(self, aid, success, description=''):
        self.actions[aid].update(actionStatus='FINISHED' if success else 'FAILED', resultDescription=description)
        self.journal.finish('action', aid, self.actions[aid]['actionStatus'])

    def _send_step(self, step):
        self.buffer.inflight = True
        self._save()
        future = self.action_client.send_goal_async(FollowRoute.Goal(
            order_id=self.buffer.order['orderId'], node_ids=step))
        generation = self.goal_generation
        future.add_done_callback(lambda f: self._accepted(f, generation))

    def _accepted(self, future, generation):
        if generation != self.goal_generation:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('Agent rejected released edge')
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self._result(f, generation))
            if self.cancel_action:
                handle.cancel_goal_async()
        except Exception as error:
            self.buffer.finish_step(False)
            self._error('agentError', str(error), fatal=True)
            self._save()

    def _result(self, future, generation):
        if generation != self.goal_generation:
            return
        self.goal_handle = None
        try:
            result = future.result().result
            self.result_pending = (result.success, result.message, time.monotonic())
        except Exception as error:
            self.result_pending = (False, str(error), time.monotonic())

    def _wait_status(self, waiting):
        if self.wait_future is not None and not self.wait_future.done():
            return
        if self.wait_future is not None:
            try:
                if not self.wait_future.result().success:
                    self.wait_value = None
            except Exception:
                self.wait_value = None
            self.wait_future = None
        if self.wait_value != waiting and self.wait_client.service_is_ready():
            self.wait_value = waiting
            self.wait_future = self.wait_client.call_async(SetBool.Request(data=waiting))

    def _tick(self):
        for kind, rid, message in self.client.drain():
            if kind == 'connected':
                self.client.publish(self.rid, 'connection', self.factory.connection('ONLINE'))
                self.client.publish(self.rid, 'factsheet', self.factory.factsheet(self.config['simulation']))
            elif kind == 'order':
                self.get_logger().info(f'Robot {self.rid} received MQTT order: {json.dumps(message, ensure_ascii=False)}')
                self._order(message)
            elif kind == 'instantActions':
                self.get_logger().info(f'Robot {self.rid} received MQTT instantActions: {json.dumps(message, ensure_ascii=False)}')
                self._instant(message)
            elif kind == 'validation_error':
                self.get_logger().warning(f'Robot {self.rid} MQTT {message["topic"]}: {message["error"]}')
                self._error('validationError', message['error'])
        if self.result_pending:
            success, message, at = self.result_pending
            if not success:
                self.buffer.finish_step(False)
                if not self.cancel_action:
                    self._error('agentError', message, fatal=True)
                self.result_pending = None
                self._save()
            elif (self._stopped() and self.odom_at > at and
                  self._node() == self.buffer.order['nodes'][self.buffer.completed+1]['nodeId']):
                self.buffer.finish_step(True)
                self.result_pending = None
                self._save()
        if self.cancel_action and not self.buffer.inflight and self._fresh() and abs(self.telemetry.get('linear', 0)) < .001 and abs(self.telemetry.get('angular', 0)) < .001:
            self.buffer.cancel()
            self._save()
            self.errors = []
            self._action_done(self.cancel_action, True)
            self.cancel_action = None
        step = self.buffer.next_step()
        if step and not self.cancel_action and self.client.connected and self._stopped() and self.action_client.server_is_ready():
            if step[0] != self._node():
                self.buffer.failed = True
                self._error('positionError', 'Robot is not at released edge start', fatal=True)
            else:
                self._send_step(step)
        self._wait_status(self.buffer.waiting)
        now = time.monotonic()
        if self.client.connected and self._fresh() and now-self.last_state >= self.config['simulation']['state_interval']:
            self.last_state = now
            state = state_from_buffer(self.factory, self.buffer, self.telemetry, self.config['simulation'],
                                      self.actions.values(), self.errors)
            self.client.publish(self.rid, 'state', state)

    def destroy_node(self):
        self.client.close((self.rid, self.factory.connection('OFFLINE')))
        self.journal.close()
        self.action_client.destroy()
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
        node = RobotAdapter()
        while rclpy.ok() and not shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
