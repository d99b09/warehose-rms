"""Exercise simulated link loss with real ROS agents and an isolated MQTT broker."""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(os.environ.get('RUN_ROS_TESTS') != '1',
                                reason='Set RUN_ROS_TESTS=1 with sourced ROS workspace')


def test_offline_robot_blocks_node_and_recovers_without_losing_orders(tmp_path):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from warehouse_interfaces.srv import SpawnRobot, SubmitOrder

    points = [('A', -2., 0.), ('B', 0., 0.), ('C', 2., 0.), ('D', -2., 2.), ('E', 2., 2.)]
    lanes = [(0, 1), (1, 2), (0, 3), (3, 4), (4, 2)]
    map_path = tmp_path / 'graph.yaml'
    map_path.write_text(yaml.safe_dump(dict(building_name='connection_test', levels=dict(L1=dict(
        vertices=[[x, y, dict(name=n, is_parking_spot=n in {'A', 'B'})] for n, x, y in points],
        lanes=[[a, b, {}] for u, v in lanes for a, b in [(u, v), (v, u)]])))))
    (tmp_path / 'connection_test.building.yaml').write_text(yaml.safe_dump(dict(
        name='connection_test', coordinate_system='cartesian_meters', levels=dict(L1=dict(
            vertices=[[-6, -4, 0, ''], [6, 6, 0, '']], walls=[])))))
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    broker_config = tmp_path / 'broker.conf'
    broker_config.write_text(f'listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n')
    config = yaml.safe_load((ROOT / 'ros_ws/src/vda_bridge/config/vda5050.yaml').read_text())
    config['mqtt']['port'] = port
    config['state_directory'] = str(tmp_path / 'receipts')
    config['simulation']['state_interval'] = .1
    config_path = tmp_path / 'vda.yaml'
    config_path.write_text(yaml.safe_dump(config))
    domain = str(90 + os.getpid() % 50)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(tmp_path / 'logs'))
    processes, logs = [], []

    def launch(command, name):
        log = (tmp_path / f'{name}.log').open('w+')
        logs.append(log)
        processes.append(subprocess.Popen(command, env=env, stdout=log,
                                          stderr=subprocess.STDOUT, start_new_session=True))

    launch(['mosquitto', '-c', str(broker_config)], 'broker')
    args = ['--ros-args', '-p', f'map_path:={map_path}', '-p', f'vda_config_path:={config_path}']
    for package, executable in [('fleet_dispatcher', 'dispatcher'), ('warehouse_sim', 'map')]:
        launch([sys.executable, str(ROOT / f'ros_ws/install/{package}/lib/{package}/{executable}'), *args], package)
    old = {key: os.environ.get(key) for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY')}
    os.environ.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1')
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('connection_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    fleet, poses = {}, {}
    node.create_subscription(String, '/fleet/state', lambda m: fleet.update(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    for rid in ('robot_01', 'robot_02'):
        node.create_subscription(Odometry, f'/{rid}/odom', lambda m, rid=rid: poses.update(
            {rid: (m.pose.pose.position.x, m.pose.pose.position.y, m.twist.twist.linear.x)}),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    spawn = node.create_client(SpawnRobot, '/warehouse/spawn_robot')
    submit = node.create_client(SubmitOrder, '/fleet/submit_order')
    links = {rid: node.create_client(SetBool, f'/{rid}/set_connection_enabled')
             for rid in ('robot_01', 'robot_02')}

    def robot(rid):
        return next((r for r in fleet.get('robots', []) if r['robot_id'] == rid), {})

    def until(condition, timeout=35):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if condition():
                return
        for log in logs:
            log.flush(); log.seek(0)
            print(log.name, log.read()[-6000:])
        pytest.fail(f'timeout: fleet={fleet}, poses={poses}')

    def call(client, request):
        until(client.service_is_ready, 15)
        future = client.call_async(request)
        until(future.done, 12)
        return future.result()

    try:
        for i, parking in [(1, 'A'), (2, 'B')]:
            assert call(spawn, SpawnRobot.Request(operation_id=f'spawn{i}', parking_node_id=parking)).accepted
        until(lambda: all(robot(rid).get('status') == 'IDLE' for rid in links))
        assert call(links['robot_02'], SetBool.Request(data=False)).success
        assert call(links['robot_02'], SetBool.Request(data=False)).success  # idempotent
        until(lambda: not robot('robot_02').get('mqtt_connected', True) and 'B' in fleet.get('blocked_nodes', []))
        assert call(submit, SubmitOrder.Request(robot_id='robot_01', target_node_id='C',
                    order_type='MOVE', order_id='detour')).accepted
        until(lambda: robot('robot_01').get('status') == 'MOVING')
        assert robot('robot_01')['remaining_route'] == ['A', 'D', 'E', 'C']
        # MOVING in the dispatcher means a grant was sent. Wait for odometry
        # to confirm the agent actually began the edge before cutting the link.
        until(lambda: poses.get('robot_01', (0, 0, 0))[1] > .2)
        # Disconnect in motion: only A -> D may finish, and the queue survives.
        assert call(links['robot_01'], SetBool.Request(data=False)).success
        until(lambda: not robot('robot_01').get('mqtt_connected', True) and
              'robot_01' in poses and abs(poses['robot_01'][1] - 2) < .15 and
              abs(poses['robot_01'][2]) < .001)
        assert robot('robot_01')['queue'][0]['order_id'] == 'detour'
        assert {'A', 'D', 'B'} <= set(fleet['blocked_nodes'])
        assert call(links['robot_01'], SetBool.Request(data=True)).success
        until(lambda: robot('robot_01').get('status') == 'IDLE' and not robot('robot_01').get('queue'), 60)
        assert not robot('robot_02')['mqtt_connected']
        assert poses['robot_02'][:2] == (0., 0.)
        assert fleet['blocked_nodes'] == ['B']
        assert call(links['robot_02'], SetBool.Request(data=True)).success
        until(lambda: robot('robot_02').get('status') == 'IDLE' and robot('robot_02').get('mqtt_connected'))
        assert fleet['blocked_nodes'] == []
        assert call(submit, SubmitOrder.Request(robot_id='robot_02', target_node_id='A',
                    order_type='MOVE', order_id='after-reconnect')).accepted
        until(lambda: robot('robot_02').get('status') == 'MOVING')
        until(lambda: robot('robot_02').get('status') == 'IDLE' and not robot('robot_02').get('queue'))
    finally:
        executor.shutdown(); node.destroy_node(); context.shutdown()
        for key, value in old.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for process in reversed(processes):
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        for log in logs:
            log.close()
