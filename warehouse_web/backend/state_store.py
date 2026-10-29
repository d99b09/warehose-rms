"""Thread-safe latest robot positions for WebSocket snapshots."""

from dataclasses import dataclass, field
import threading
import time

from .models import RobotState, StateSnapshot, FleetState


@dataclass
class Entry:
    state: RobotState
    odom_monotonic: float | None = None
    status_monotonic: float | None = None
    odom_sequence: int = -1
    agent_monotonic: float | None = None
    agent_sequence: int = -1
    agent_epoch: str = ''
    retired_agent_epochs: set = field(default_factory=set)


class StateStore:
    def __init__(self, stale_after=2.0):
        self.lock = threading.Lock()
        self.stale_after = stale_after
        self.entries: dict[str, Entry] = {}
        self.fleet = None
        self.fleet_at = float("-inf")

    def update_inventory(self, robots):
        templates = {item['robot_id']: RobotState(
            robot_id=item['robot_id'], length=item['length'], width=item['width'])
            for item in robots}
        with self.lock:
            entries = {}
            for robot_id, template in templates.items():
                entry = self.entries.get(robot_id) or Entry(template)
                entry.state = entry.state.model_copy(update={
                    'length': template.length, 'width': template.width})
                entries[robot_id] = entry
            self.entries = entries

    def update_odometry(self, robot_id, *, x, y, theta, linear_velocity, angular_velocity, sequence):
        with self.lock:
            entry = self.entries.get(robot_id)
            if entry is None or sequence <= entry.odom_sequence:
                return
            entry.state = RobotState.model_validate({**entry.state.model_dump(),
                'x': x, 'y': y, 'theta': theta, 'linear_velocity': linear_velocity,
                'angular_velocity': angular_velocity, 'received_at': time.time()})
            entry.odom_sequence = sequence
            entry.odom_monotonic = time.monotonic()

    def update_status(self, robot_id: str, status: str):
        with self.lock:
            entry = self.entries.get(robot_id)
            if entry is None:
                return
            entry.state = RobotState.model_validate({**entry.state.model_dump(),
                'status': status, 'status_received_at': time.time()})
            entry.status_monotonic = time.monotonic()

    def update_agent(self, robot_id, *, status, order_id, current_node_id,
                     target_node_id, error_code, sequence, agent_epoch):
        with self.lock:
            entry = self.entries.get(robot_id)
            if entry is None or agent_epoch in entry.retired_agent_epochs:
                return
            if agent_epoch == entry.agent_epoch and sequence <= entry.agent_sequence:
                return
            if agent_epoch != entry.agent_epoch:
                if entry.agent_epoch:
                    entry.retired_agent_epochs.add(entry.agent_epoch)
                entry.agent_epoch = agent_epoch
            entry.agent_sequence = sequence
            entry.state = RobotState.model_validate({**entry.state.model_dump(),
                'agent_status': status, 'order_id': order_id,
                'current_node_id': current_node_id, 'target_node_id': target_node_id,
                'error_code': error_code, 'agent_received_at': time.time()})
            entry.agent_monotonic = time.monotonic()

    def update_fleet(self, data):
        fleet = FleetState.model_validate(data)
        with self.lock:
            self.fleet, self.fleet_at = fleet, time.monotonic()

    def snapshot(self, bridge_ok=True):
        with self.lock:
            now = time.monotonic()
            robots = []
            for entry in self.entries.values():
                age = None if entry.odom_monotonic is None else max(0., now - entry.odom_monotonic)
                status_age = (None if entry.status_monotonic is None
                              else max(0., now - entry.status_monotonic))
                agent_age = (None if entry.agent_monotonic is None
                             else max(0., now - entry.agent_monotonic))
                robots.append(entry.state.model_copy(update={
                    'agent_age': agent_age,
                    'agent_stale': agent_age is None or agent_age >= self.stale_after,
                    'odom_age': age, 'stale': age is None or age >= self.stale_after,
                    'status_age': status_age,
                    'status_stale': status_age is None or status_age >= self.stale_after}))
            return StateSnapshot(server_time=time.time(), stale_after=self.stale_after,
                                 bridge_ok=bridge_ok, robots=tuple(robots),
                                 fleet=self.fleet.model_copy(update={"stale": not bridge_ok or now-self.fleet_at >= self.stale_after}) if self.fleet else None)
