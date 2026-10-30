"""Real ROS/MQTT check of the protected node during a yield."""
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


@pytest.mark.parametrize('scenario', ['single', 'recursive', 'backtrack', 'busy_recursive',
                                     'parallel', 'parallel_clearance'])
def test_yield_route_avoids_requester_node_with_real_robots(tmp_path, scenario):
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    from warehouse_interfaces.srv import SpawnRobot, SubmitOrder

    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    broker_conf = tmp_path / 'mosquitto.conf'
    broker_conf.write_text(f'listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n')
    config = yaml.safe_load((ROOT / 'ros_ws/src/vda_bridge/config/vda5050.yaml').read_text())
    config['mqtt']['port'] = port
    config['state_directory'] = str(tmp_path / 'receipts')
    config['simulation']['state_interval'] = .1
    config_path = tmp_path / 'vda.yaml'
    config_path.write_text(yaml.safe_dump(config))
    map_path = ROOT / 'maps/nav/warehouse_normal/0.yaml'
    parking, target = ['P1', 'P2'], 'P2'
    expected_yields = ['robot_02']
    if scenario != 'single':
        points = dict(A=(-2, 0), B=(0, 0), C=(2, 0), D=(0, 2))
        edges = [('A', 'B'), ('B', 'C'), ('B', 'D')]
        if scenario in {'recursive', 'busy_recursive'}:
            points.update(E=(0, 4), F=(0, 6))
            edges += [('D', 'E'), ('E', 'F')]
            parking = ['A', 'B', 'D', 'E']
            expected_yields = ['robot_04', 'robot_03', 'robot_02']
        elif scenario == 'backtrack':
            points['E'] = (0, -4)
            edges += [('B', 'E')]
            parking = ['A', 'B', 'D']
        else:
            points.update(X=(-2, 6), Y=(0, 6), Z=(2, 6), W=(0, 8))
            edges += [('X', 'Y'), ('Y', 'Z'), ('Y', 'W')]
            parking = ['A', 'B', 'X']
            if scenario == 'parallel_clearance':
                parking.append('Y')
                expected_yields.append('robot_04')
        names = list(points)
        lanes = [[names.index(a), names.index(b), {}]
                 for source, target in edges for a, b in ((source, target), (target, source))]
        map_path = tmp_path / 'graph.yaml'
        map_path.write_text(yaml.safe_dump(dict(
            building_name='recursive_test', levels=dict(L1=dict(
                vertices=[[x, y, dict(name=name, is_parking_spot=name in parking)]
                          for name, (x, y) in points.items()], lanes=lanes)))))
        (tmp_path / 'recursive_test.building.yaml').write_text(yaml.safe_dump(dict(
            name='recursive_test', coordinate_system='cartesian_meters',
            levels=dict(L1=dict(vertices=[[-10, -10, 0, ''], [10, 10, 0, '']], walls=[])))))
        target = 'C'
    domain = str(80 + os.getpid() % 90)
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(tmp_path/'roslogs'))
    processes, logs = [], []

    def launch(args, name):
        log = (tmp_path / (name + '.log')).open('w+')
        p = subprocess.Popen(args, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p); logs.append(log)

    launch(['mosquitto', '-c', str(broker_conf)], 'broker')
    args = ['--ros-args', '-p', f'map_path:={map_path}', '-p', f'vda_config_path:={config_path}']
    launch([sys.executable, str(ROOT / 'ros_ws/install/fleet_dispatcher/lib/fleet_dispatcher/dispatcher'), *args], 'dispatcher')
    launch([sys.executable, str(ROOT / 'ros_ws/install/warehouse_sim/lib/warehouse_sim/map'), *args], 'map')
    old_domain, old_local = os.environ.get('ROS_DOMAIN_ID'), os.environ.get('ROS_LOCALHOST_ONLY')
    os.environ.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1')
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('yield_integration_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    state, snapshots = {}, []

    def update(msg):
        state.clear()
        state.update(json.loads(msg.data))
        snapshots.append(json.loads(msg.data))

    node.create_subscription(String, '/fleet/state', update,
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    spawn = node.create_client(SpawnRobot, '/warehouse/spawn_robot')
    submit = node.create_client(SubmitOrder, '/fleet/submit_order')

    def until(predicate, timeout=40):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
            if predicate():
                return
        for log in logs:
            log.flush(); log.seek(0)
            print(log.name, log.read()[-6000:])
        pytest.fail(f'Timed out; fleet={state}')

    def call(client, request):
        until(client.service_is_ready, 15)
        future = client.call_async(request)
        until(future.done, 15)
        return future.result()

    def robot(rid):
        return next((r for r in state.get('robots', []) if r['robot_id'] == rid), {})

    try:
        for i, parking_node in enumerate(parking, 1):
            result = call(spawn, SpawnRobot.Request(operation_id=f'spawn{i}', parking_node_id=parking_node))
            assert result.accepted, result.message
        until(lambda: len(state.get('robots', [])) == len(parking) and all(r['status'] == 'IDLE' for r in state['robots']))
        result = call(submit, SubmitOrder.Request(robot_id='robot_01', target_node_id=target, order_type='MOVE', order_id='yield-test'))
        assert result.accepted
        if scenario.startswith('parallel'):
            assert call(submit, SubmitOrder.Request(robot_id='robot_03', target_node_id='Z',
                         order_type='MOVE', order_id='independent')).accepted
        if scenario == 'busy_recursive':
            for rid, goal, oid in [('robot_02', 'A', 'b1'), ('robot_02', 'C', 'b2'),
                                   ('robot_03', 'B', 'c1'), ('robot_04', 'D', 'd1')]:
                result = call(submit, SubmitOrder.Request(robot_id=rid, target_node_id=goal,
                                                         order_type='MOVE', order_id=oid))
                assert result.accepted, result.message
        until(lambda: robot(expected_yields[0]).get('yielding'))
        protected = robot('robot_01')['remaining_route'][0]
        yielding_route = robot(expected_yields[0])['remaining_route']
        assert protected not in yielding_route
        until(lambda: robot('robot_01').get('status') == 'IDLE' and not robot('robot_01').get('queue'), 90)
        observed_yields, seen_executions, root_executions = [], set(), set()
        for snapshot in snapshots:
            robots = {r['robot_id']: r for r in snapshot.get('robots', [])}
            for rid, r in robots.items():
                if r['yielding']:
                    assert protected not in r['remaining_route']
                    if r['vda_order_id'] not in seen_executions:
                        seen_executions.add(r['vda_order_id'])
                        observed_yields.append(rid)
                if rid == 'robot_01' and r['vda_order_id']:
                    root_executions.add(r['vda_order_id'])
        if scenario.startswith('parallel'):
            assert sorted(observed_yields) == sorted(expected_yields)
            def concurrent(snapshot):
                robots = {r['robot_id']: r for r in snapshot['robots']}
                other = robots.get('robot_04' if scenario == 'parallel_clearance' else 'robot_03', {})
                return robots.get('robot_02', {}).get('yielding') and (
                    other.get('yielding') if scenario == 'parallel_clearance' else other.get('status') == 'MOVING')
            assert any(concurrent(snapshot) for snapshot in snapshots)
            until(lambda: all(r['status'] == 'IDLE' and not r['queue'] for r in state['robots']), 90)
        else:
            assert observed_yields == expected_yields
        assert len(root_executions) == 1
        if scenario == 'busy_recursive':
            assert [o['order_id'] for o in robot('robot_02')['queue']] == ['b1', 'b2']
            assert [o['order_id'] for o in robot('robot_03')['queue']] == ['c1']
            assert [o['order_id'] for o in robot('robot_04')['queue']] == ['d1']
            until(lambda: all(r['status'] == 'IDLE' and not r['queue']
                              for r in state['robots']), 150)
        assert all(not r['queue'] for r in state['robots'])
        if scenario == 'backtrack':
            routes = [r['remaining_route'] for s in snapshots for r in s['robots']
                      if r['robot_id'] == 'robot_02' and r['yielding']]
            assert any(route == ['B', 'E'] for route in routes)
        assert not any('Traceback' in Path(log.name).read_text() for log in logs)
    finally:
        executor.shutdown(); node.destroy_node(); context.shutdown()
        for key, old in [('ROS_DOMAIN_ID', old_domain), ('ROS_LOCALHOST_ONLY', old_local)]:
            if old is None: os.environ.pop(key, None)
            else: os.environ[key] = old
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in reversed(processes):
            try:
                p.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL); p.wait()
        for log in logs:
            log.close()
