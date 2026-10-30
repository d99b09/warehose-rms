"""End-to-end manual displacement and recovery with ROS and MQTT."""
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


@pytest.mark.parametrize('active_order', [False, True])
def test_manual_off_graph_returns_to_node(tmp_path, active_order):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    from warehouse_interfaces.srv import SpawnRobot, SubmitOrder

    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    conf = tmp_path / 'broker.conf'
    conf.write_text(f'listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n')
    config = yaml.safe_load((ROOT / 'ros_ws/src/vda_bridge/config/vda5050.yaml').read_text())
    config['mqtt']['port'] = port
    config['state_directory'] = str(tmp_path / 'receipts')
    config['simulation']['state_interval'] = .1
    config_path = tmp_path / 'vda.yaml'
    config_path.write_text(yaml.safe_dump(config))
    domain = str(30 + os.getpid() % 60)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(tmp_path / 'logs'))
    args = ['--ros-args', '-p', f'map_path:={ROOT / "maps/nav/warehouse_normal/0.yaml"}',
            '-p', f'vda_config_path:={config_path}']
    processes, logs = [], []

    def launch(command, name):
        log = (tmp_path / f'{name}.log').open('w+')
        logs.append(log)
        processes.append(subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True))

    launch(['mosquitto', '-c', str(conf)], 'broker')
    launch([sys.executable, str(ROOT / 'ros_ws/install/fleet_dispatcher/lib/fleet_dispatcher/dispatcher'), *args], 'dispatcher')
    launch([sys.executable, str(ROOT / 'ros_ws/install/warehouse_sim/lib/warehouse_sim/map'), *args], 'map')
    old = {key: os.environ.get(key) for key in ('ROS_DOMAIN_ID', 'ROS_LOCALHOST_ONLY')}
    os.environ.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1')
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('manual_recovery_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    fleet, pose = {}, {}
    node.create_subscription(String, '/fleet/state', lambda m: fleet.update(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Odometry, '/robot_01/odom', lambda m: pose.update(
        x=m.pose.pose.position.x, y=m.pose.pose.position.y),
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    spawn = node.create_client(SpawnRobot, '/warehouse/spawn_robot')
    manual = node.create_client(SetBool, '/robot_01/set_manual_mode')
    submit = node.create_client(SubmitOrder, '/fleet/submit_order')
    velocity = node.create_publisher(Twist, '/robot_01/manual_cmd_vel', 1)

    def until(condition, timeout=25):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if condition():
                return
        for log in logs:
            log.flush(); log.seek(0)
            print(log.name, log.read()[-5000:])
        pytest.fail(f'timeout: fleet={fleet}, pose={pose}')

    def call(client, request):
        until(client.service_is_ready)
        future = client.call_async(request)
        until(future.done)
        return future.result()

    try:
        assert call(spawn, SpawnRobot.Request(operation_id='manual-recovery', parking_node_id='P1')).accepted
        until(lambda: pose and fleet.get('robots') and fleet['robots'][0]['status'] == 'IDLE')
        start = (pose['x'], pose['y'])
        if active_order:
            reply = call(submit, SubmitOrder.Request(robot_id='robot_01', target_node_id='P2',
                order_type='MOVE', order_id='resume-after-manual'))
            assert reply.accepted
            until(lambda: fleet['robots'][0]['status'] == 'MOVING')
        assert call(manual, SetBool.Request(data=True)).success
        command = Twist(); command.linear.x = .25
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            velocity.publish(command)
            executor.spin_once(timeout_sec=.05)
        velocity.publish(Twist())
        assert ((pose['x']-start[0])**2 + (pose['y']-start[1])**2)**.5 > .2
        assert call(manual, SetBool.Request(data=False)).success
        until(lambda: fleet['robots'][0]['status'] == 'MOVING', 15)
        if active_order:
            until(lambda: fleet['robots'][0]['status'] == 'IDLE' and
                  not fleet['robots'][0]['queue'], 80)
        else:
            until(lambda: fleet['robots'][0]['status'] == 'IDLE' and
                  ((pose['x']-start[0])**2 + (pose['y']-start[1])**2)**.5 < .16, 30)
    finally:
        executor.shutdown(); node.destroy_node(); context.shutdown()
        for key, value in old.items():
            if value is None: os.environ.pop(key, None)
            else: os.environ[key] = value
        for process in reversed(processes):
            if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
        for process in reversed(processes):
            try: process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait()
        for log in logs: log.close()
