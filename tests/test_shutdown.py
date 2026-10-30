"""Exercise the installed ROS entry points with actual process signals."""
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


@pytest.mark.parametrize('component,stop_signal', [
    ('dispatcher', signal.SIGINT),
    ('dispatcher', signal.SIGTERM),
    ('robot_adapter', signal.SIGINT),
    ('robot_adapter', signal.SIGTERM),
])
def test_ros_process_signal_shutdown(tmp_path, component, stop_signal):
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    broker_config = tmp_path / 'mosquitto.conf'
    broker_config.write_text(f'listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n')
    config = yaml.safe_load((ROOT / 'ros_ws/src/vda_bridge/config/vda5050.yaml').read_text())
    config['mqtt']['port'] = port
    config['state_directory'] = str(tmp_path / 'receipts')
    config_path = tmp_path / 'vda.yaml'
    config_path.write_text(yaml.safe_dump(config))
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(20 + os.getpid() % 80), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(tmp_path / 'roslogs'))
    executable = (ROOT / ('ros_ws/install/fleet_dispatcher/lib/fleet_dispatcher/dispatcher'
                          if component == 'dispatcher' else
                          'ros_ws/install/vda_bridge/lib/vda_bridge/robot_adapter'))
    args = [sys.executable, str(executable), '--ros-args',
            '-p', f'vda_config_path:={config_path}']
    if component == 'robot_adapter':
        args += ['-r', '__ns:=/robot_shutdown_test']
    broker_log = tmp_path / 'broker.log'
    process_log = tmp_path / 'process.log'
    with broker_log.open('w') as broker_out, process_log.open('w') as process_out:
        broker = subprocess.Popen(['mosquitto', '-c', str(broker_config)],
                                  stdout=broker_out, stderr=subprocess.STDOUT, env=env)
        process = None
        try:
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                try:
                    with socket.create_connection(('127.0.0.1', port), timeout=.1):
                        break
                except OSError:
                    time.sleep(.05)
            else:
                pytest.fail('Mosquitto did not start')
            process = subprocess.Popen(args, stdout=process_out, stderr=subprocess.STDOUT, env=env)
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    pytest.fail(f'{component} exited early: {process_log.read_text()}')
                if 'New client connected' in broker_log.read_text():
                    break
                time.sleep(.05)
            else:
                pytest.fail(f'{component} did not connect: {process_log.read_text()}')
            # Allow timer callbacks to run before interrupting the executor.
            time.sleep(.2)
            process.send_signal(stop_signal)
            assert process.wait(timeout=10) == 0, process_log.read_text()
            assert 'Traceback' not in process_log.read_text()
            if component == 'robot_adapter':
                retained = subprocess.run(
                    ['mosquitto_sub', '-h', '127.0.0.1', '-p', str(port), '-C', '1', '-W', '3',
                     '-t', 'uagv/v2/WarehouseSim/robot_shutdown_test/connection'],
                    capture_output=True, text=True, check=True, timeout=5)
                assert json.loads(retained.stdout)['connectionState'] == 'OFFLINE'
        finally:
            if process is not None and process.poll() is None:
                process.kill()
                process.wait(timeout=5)
            broker.terminate()
            broker.wait(timeout=5)
