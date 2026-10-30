"""ROS telemetry and spawn service client running alongside the HTTP server."""

import json
import logging
from math import atan2, hypot, isfinite
import threading

logger = logging.getLogger(__name__)


def quaternion_yaw(x, y, z, w):
    norm = hypot(x, y, z, w)
    if not isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid odometry quaternion')
    x, y, z, w = (value / norm for value in (x, y, z, w))
    return atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


class RosBridge:
    def __init__(self, store):
        self.store = store
        self.lock = threading.Lock()
        self.context = self.node = self.executor = self.thread = None
        self.subscriptions = {}
        self.manual_robot = None
        self.manual_session = None
        self.manual_sequence = -1
        self.manual_publisher = None

    @property
    def healthy(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
        from warehouse_interfaces.srv import SpawnRobot, SubmitOrder

        self.context = Context()
        rclpy.init(args=[], context=self.context, signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('warehouse_web_bridge', context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.client = self.node.create_client(SpawnRobot, '/warehouse/spawn_robot')
        self.order_client = self.node.create_client(SubmitOrder, '/fleet/submit_order')
        self.node.create_subscription(
            String, '/warehouse/robot_inventory', self._inventory,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.node.create_subscription(String, '/fleet/state', self._fleet,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.thread = threading.Thread(target=self._spin, name='warehouse-ros', daemon=True)
        self.thread.start()

    def _fleet(self, message):
        try:
            self.store.update_fleet(json.loads(message.data))
        except (ValueError, TypeError):
            logger.warning('Ignoring invalid fleet state', exc_info=True)

    def _spin(self):
        from rclpy.executors import ExternalShutdownException
        try:
            self.executor.spin()
        except ExternalShutdownException:
            pass
        except Exception:
            logger.exception('ROS executor stopped unexpectedly')

    def _inventory(self, message):
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        from warehouse_interfaces.msg import RobotState
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        try:
            robots = json.loads(message.data)
            if not isinstance(robots, list):
                raise ValueError('Robot inventory must be a list')
            namespaces = {robot['robot_id']: robot['namespace'] for robot in robots}
            self.store.update_inventory(robots)
            for robot_id, (namespace, subscriptions) in list(self.subscriptions.items()):
                if namespaces.get(robot_id) != namespace:
                    for subscription in subscriptions:
                        self.node.destroy_subscription(subscription)
                    del self.subscriptions[robot_id]
            for robot_id, namespace in namespaces.items():
                if robot_id not in self.subscriptions:
                    subscription = self.node.create_subscription(
                        Odometry, namespace + '/odom',
                        lambda message, rid=robot_id: self._odometry(rid, message),
                        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
                    status = None
                    try:
                        status = self.node.create_subscription(
                            String, namespace + '/status',
                            lambda message, rid=robot_id: self.store.update_status(rid, message.data),
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                        agent = self.node.create_subscription(
                            RobotState, namespace + '/agent_state',
                            lambda message, rid=robot_id: self.store.update_agent(
                                rid, status=message.status, order_id=message.order_id,
                                current_node_id=message.current_node_id,
                                target_node_id=message.target_node_id, error_code=message.error_code,
                                sequence=message.sequence, agent_epoch=message.agent_epoch),
                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                    except Exception:
                        if status is not None:
                            self.node.destroy_subscription(status)
                        self.node.destroy_subscription(subscription)
                        raise
                    self.subscriptions[robot_id] = (namespace, (subscription, status, agent))
        except (KeyError, TypeError, ValueError):
            logger.warning('Ignoring invalid robot inventory', exc_info=True)

    def _odometry(self, robot_id, message):
        try:
            pose, velocity = message.pose.pose, message.twist.twist
            q = pose.orientation
            self.store.update_odometry(
                robot_id, x=pose.position.x, y=pose.position.y,
                theta=quaternion_yaw(q.x, q.y, q.z, q.w),
                linear_velocity=velocity.linear.x, angular_velocity=velocity.angular.z,
                sequence=message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec)
        except ValueError:
            logger.warning('Ignoring invalid odometry for %s', robot_id)

    def spawn(self, operation_id, parking_node_id):
        from warehouse_interfaces.srv import SpawnRobot

        with self.lock:
            if not self.healthy or not self.client.wait_for_service(timeout_sec=2.0):
                raise ConnectionError('Симулятор недоступен')
            future = self.client.call_async(SpawnRobot.Request(
                operation_id=operation_id, parking_node_id=parking_node_id, query=False))
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(5.0):
                self.client.remove_pending_request(future)
                future.cancel()
                raise TimeoutError('Результат создания неизвестен; повторите тот же operation_id')
            try:
                response = future.result()
            except Exception as error:
                raise ConnectionError('Ошибка ответа симулятора') from error
            return {key: getattr(response, key) for key in response.get_fields_and_field_types()}

    def submit_order(self, **payload):
        from warehouse_interfaces.srv import SubmitOrder

        with self.lock:
            if not self.healthy or not self.order_client.wait_for_service(timeout_sec=2.0):
                raise ConnectionError('Диспетчер недоступен')
            future = self.order_client.call_async(SubmitOrder.Request(**payload))
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(5.0):
                self.order_client.remove_pending_request(future)
                future.cancel()
                raise TimeoutError('Результат отправки маршрута неизвестен')
            try:
                reply = future.result()
            except Exception as error:
                raise ConnectionError('Ошибка ответа диспетчера') from error
            return {key: getattr(reply, key) for key in reply.get_fields_and_field_types()}

    def robot_connection(self, robot_id, enabled):
        from std_srvs.srv import SetBool

        with self.lock:
            if not self.healthy:
                raise ConnectionError('ROS недоступен')
            if robot_id not in self.subscriptions:
                raise ValueError('Неизвестный робот')
            if self.manual_robot == robot_id:
                raise ValueError('Сначала выйдите из ручного управления')
            namespace = self.subscriptions[robot_id][0]
            client = self.node.create_client(SetBool, namespace + '/set_connection_enabled')
            try:
                if not client.wait_for_service(timeout_sec=2.0):
                    raise ConnectionError('Адаптер робота недоступен')
                future = client.call_async(SetBool.Request(data=enabled))
                done = threading.Event()
                future.add_done_callback(lambda _: done.set())
                if not done.wait(8.0):
                    client.remove_pending_request(future)
                    future.cancel()
                    raise TimeoutError('Нет ответа от адаптера; проверьте состояние связи')
                try:
                    response = future.result()
                except Exception as error:
                    raise ConnectionError('Ошибка ответа адаптера робота') from error
                if not response.success:
                    raise ValueError(response.message)
                return {'accepted': True, 'enabled': enabled, 'message': response.message}
            finally:
                self.node.destroy_client(client)

    def manual_mode(self, robot_id, enabled):
        from geometry_msgs.msg import Twist
        from std_srvs.srv import SetBool

        with self.lock:
            if not self.healthy:
                raise ConnectionError('ROS недоступен')
            if robot_id not in self.subscriptions:
                raise ValueError('Неизвестный робот')
            if enabled and self.manual_robot is not None:
                raise ValueError('Ручное управление уже включено')
            if not enabled and self.manual_robot != robot_id:
                raise ValueError('Ручное управление для робота не включено')
            namespace = self.subscriptions[robot_id][0]
            client = self.node.create_client(SetBool, namespace + '/set_manual_mode')
            try:
                if not client.wait_for_service(timeout_sec=2.0):
                    raise ConnectionError('Агент недоступен')
                if not enabled:
                    self.manual_publisher.publish(Twist())
                future = client.call_async(SetBool.Request(data=enabled))
                done = threading.Event()
                future.add_done_callback(lambda _: done.set())
                if not done.wait(2.0):
                    client.remove_pending_request(future)
                    future.cancel()
                    raise TimeoutError('Нет ответа от агента')
                response = future.result()
                if not response.success:
                    raise ValueError(response.message)
                if enabled:
                    self.manual_publisher = self.node.create_publisher(
                        Twist, namespace + '/manual_cmd_vel', 1)
                    self.manual_robot = robot_id
                    self.manual_session, self.manual_sequence = None, -1
                else:
                    self.node.destroy_publisher(self.manual_publisher)
                    self.manual_publisher = self.manual_robot = self.manual_session = None
                return {'accepted': True, 'message': response.message}
            finally:
                self.node.destroy_client(client)

    def manual_command(self, robot_id, session_id, sequence, linear, angular):
        from geometry_msgs.msg import Twist

        with self.lock:
            if not self.healthy:
                raise ConnectionError('ROS недоступен')
            if self.manual_robot != robot_id:
                raise ValueError('Ручное управление для робота не включено')
            if self.manual_session is not None and self.manual_session != session_id:
                raise ValueError('Другая сессия ручного управления')
            if sequence <= self.manual_sequence:
                raise ValueError('Команда получена не по порядку')
            self.manual_session, self.manual_sequence = session_id, sequence
            message = Twist()
            message.linear.x, message.angular.z = linear, angular
            self.manual_publisher.publish(message)
            return {'accepted': True, 'message': 'Команда отправлена'}

    def stop(self):
        with self.lock:
            if self.executor:
                self.executor.shutdown(timeout_sec=2)
            if self.thread:
                self.thread.join(timeout=3)
                if self.thread.is_alive():
                    raise RuntimeError('ROS executor did not stop')
            if self.node:
                self.node.destroy_node()
            if self.context:
                self.context.try_shutdown()
