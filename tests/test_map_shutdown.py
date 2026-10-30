"""Exercise map node shutdown with real ROS process signals."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest


ROOT = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(os.environ.get('RUN_ROS_TESTS') != '1',
                                reason='Set RUN_ROS_TESTS=1 with sourced ROS workspace')


@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_map_process_signal_shutdown(tmp_path, stop_signal):
    env = os.environ.copy()
    env.update(ROS_DOMAIN_ID=str(20 + os.getpid() % 80), ROS_LOCALHOST_ONLY='1',
               ROS_LOG_DIR=str(tmp_path / 'roslogs'))
    executable = ROOT / 'ros_ws/install/warehouse_sim/lib/warehouse_sim/map'
    map_path = ROOT / 'maps/nav/warehouse_normal/0.yaml'
    process = subprocess.Popen(
        [sys.executable, str(executable), '--ros-args', '-p', f'map_path:={map_path}'],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        time.sleep(2)
        if process.poll() is not None:
            pytest.fail(f'map exited early: {process.communicate()[0]}')
        process.send_signal(stop_signal)
        output, _ = process.communicate(timeout=10)
        assert process.returncode == 0, output
        assert 'Traceback' not in output
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate(timeout=5)
