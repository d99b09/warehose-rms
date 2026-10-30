"""Real Mosquitto/ROS integration. Enable with RUN_ROS_TESTS=1 after colcon build."""
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
pytestmark = pytest.mark.skipif(os.environ.get('RUN_ROS_TESTS') != '1', reason='Set RUN_ROS_TESTS=1 with sourced ROS workspace')


@pytest.mark.parametrize('map_name', ['normal', 'stress'])
def test_multi_robot_fifo_yield_and_broker_restart(tmp_path, map_name):
    import rclpy
    from rclpy.context import Context
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from std_msgs.msg import String
    from warehouse_interfaces.srv import SpawnRobot, SubmitOrder
    from warehouse_sim.warehouse_map import WarehouseMap

    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    conf = tmp_path/'mosquitto.conf'
    conf.write_text(f'listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n')
    config = yaml.safe_load((ROOT/'ros_ws/src/vda_bridge/config/vda5050.yaml').read_text())
    config['mqtt']['port'] = port
    config['mqtt']['keepalive'] = 2
    config['state_directory'] = str(tmp_path/'receipts')
    config['simulation']['state_interval'] = .1
    vda_path = tmp_path/'vda.yaml'
    vda_path.write_text(yaml.safe_dump(config))
    map_path = ROOT/f'maps/nav/warehouse_{map_name}/0.yaml'
    warehouse = WarehouseMap.from_yaml(map_path)
    env = os.environ.copy()
    domain = str(80 + (os.getpid() % 90))
    env.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(tmp_path/'roslogs'))
    processes, logs = [], []

    def launch(args, name):
        log = (tmp_path/(name+'.log')).open('w+')
        p = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p); logs.append(log)
        return p

    broker = launch(['mosquitto','-c',str(conf)], 'broker')
    args = ['--ros-args', '-p', f'map_path:={map_path}', '-p', f'vda_config_path:={vda_path}']
    dispatcher_args = [sys.executable, str(ROOT/'ros_ws/install/fleet_dispatcher/lib/fleet_dispatcher/dispatcher'), *args]
    dispatcher = launch(dispatcher_args, 'dispatcher')
    simulator = launch([sys.executable, str(ROOT/'ros_ws/install/warehouse_sim/lib/warehouse_sim/map'), *args], 'map')
    old_domain, old_local = os.environ.get('ROS_DOMAIN_ID'), os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ['ROS_DOMAIN_ID'], os.environ['ROS_LOCALHOST_ONLY'] = domain, '1'
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('vda_integration_test', context=context)
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    state = {}
    node.create_subscription(String, '/fleet/state', lambda msg: state.update(json.loads(msg.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    spawn = node.create_client(SpawnRobot, '/warehouse/spawn_robot')
    submit = node.create_client(SubmitOrder, '/fleet/submit_order')

    def until(predicate, timeout=40):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if predicate():
                return
        for log in logs:
            log.flush(); log.seek(0)
            print(log.name, log.read()[-10000:])
        pytest.fail(f'Timed out; fleet={state}')

    def call(client, request):
        until(client.service_is_ready, 15)
        future = client.call_async(request)
        until(future.done, 15)
        return future.result()

    def robot(rid):
        return next((r for r in state.get('robots', []) if r['robot_id'] == rid), {})

    try:
        for i in (1,2):
            response = call(spawn, SpawnRobot.Request(operation_id=f'spawn{i}', parking_node_id=f'P{i}'))
            assert response.accepted, response.message
        until(lambda: len(state.get('robots', [])) == 2 and all(r['status'] == 'IDLE' for r in state['robots']))
        # First travel to the occupied parking: idle robot must vacate it.
        response = call(submit, SubmitOrder.Request(robot_id='robot_01', target_node_id='P2', order_type='MOVE', order_id='first'))
        assert response.accepted, response.message
        # A second user order is retained FIFO while the first one executes.
        response = call(submit, SubmitOrder.Request(robot_id='robot_01', target_node_id='P1', order_type='PARK', order_id='second'))
        assert response.accepted and response.route == []
        until(lambda: robot('robot_01').get('status') == 'MOVING')
        broker.terminate(); broker.wait(timeout=5)
        until(lambda: not state.get('mqtt_connected', True), 10)
        assert len(robot('robot_01')['queue']) == 2
        launch(['mosquitto','-c',str(conf)], 'broker-restarted')
        until(lambda: state.get('mqtt_connected') and all(r['mqtt_connected'] for r in state.get('robots', [])), 15)
        until(lambda: robot('robot_01').get('status') == 'IDLE' and not robot('robot_01').get('queue'), 100)
        assert robot('robot_02')['queue'] == []
        assert all(r['status'] != 'UNCERTAIN' for r in state['robots'])
        # Restart RMS while a route is in progress: no user queue is restored.
        response = call(submit, SubmitOrder.Request(robot_id='robot_01', target_node_id='P2', order_type='MOVE', order_id='discard-after-restart'))
        assert response.accepted
        until(lambda: robot('robot_01').get('status') == 'MOVING')
        os.killpg(dispatcher.pid, signal.SIGINT)
        dispatcher.wait(timeout=8)
        state.clear()
        dispatcher = launch(dispatcher_args, 'dispatcher-restarted')
        until(lambda: len(state.get('robots', [])) == 2 and all(r['status'] == 'IDLE' and not r['queue'] for r in state['robots']), 30)
        assert all(not r['vda_order_id'] for r in state['robots'])
        if map_name == 'normal':
            # Crash only the adapter spawned by this test, while its ROS agent lives.
            response = call(submit, SubmitOrder.Request(robot_id='robot_01', target_node_id='P2', order_type='MOVE', order_id='adapter-crash'))
            assert response.accepted
            until(lambda: robot('robot_01').get('status') == 'MOVING')
            children = Path(f'/proc/{simulator.pid}/task/{simulator.pid}/children').read_text().split()
            adapter_pid = next(int(pid) for pid in children
                if 'vda_bridge/robot_adapter' in Path(f'/proc/{pid}/cmdline').read_text()
                and '__ns:=/robot_01' in Path(f'/proc/{pid}/cmdline').read_text())
            os.kill(adapter_pid, signal.SIGKILL)
            launch([sys.executable, str(ROOT/'ros_ws/install/vda_bridge/lib/vda_bridge/robot_adapter'),
                    *args, '-r', '__ns:=/robot_01'], 'adapter-restarted')
            until(lambda: robot('robot_01').get('status') == 'UNCERTAIN', 15)
            assert robot('robot_01')['queue'][0]['order_id'] == 'adapter-crash'
            # A new dispatcher must cancel the checkpoint, not replay it.
            os.killpg(dispatcher.pid, signal.SIGINT); dispatcher.wait(timeout=8)
            state.clear()
            launch(dispatcher_args, 'dispatcher-after-adapter')
            until(lambda: len(state.get('robots', [])) == 2 and all(r['status'] == 'IDLE' and not r['queue'] for r in state['robots']), 30)
        # Real queues and state from both adapters survived broker reconnect.
        assert not any('Traceback' in Path(log.name).read_text() for log in logs)
    finally:
        executor.shutdown(); node.destroy_node(); context.shutdown()
        for key, old in [('ROS_DOMAIN_ID',old_domain),('ROS_LOCALHOST_ONLY',old_local)]:
            if old is None: os.environ.pop(key, None)
            else: os.environ[key] = old
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in reversed(processes):
            try: p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL); p.wait()
        for log in logs: log.close()
