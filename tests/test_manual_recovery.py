"""Recovery from manual displacement and continuation of the order queue."""
from robot_agent.core import AgentCore
from warehouse_sim.kinematics import Pose
from test_fleet_queue import Harness
from test_manual_routing import make_map
from protocols.vda5050.mapper import state_from_buffer
from protocols.vda5050.message_factory import MessageFactory
from protocols.vda5050.models import Identity


class RecoveryHarness(Harness):
    def __init__(self, warehouse):
        super().__init__(warehouse)
        self.recoveries = []

    def recover(self, rid, target):
        self.recoveries.append((rid, target))


def set_pose(h, x, y, status):
    h.now += .01
    seq = int(h.now * 10000)
    h.core.odometry('r', x, y, 0, 0, seq)
    h.core.agent('r', status, 'epoch', seq)


def publish_state(h):
    r = h.core.robots['r']
    state = state_from_buffer(MessageFactory(Identity(serial_number='r')), h.buffers['r'],
        dict(pose=(*r.pose, 0)), dict(battery_charge=100., map_id='map'), h.actions['r'])
    h.core.state('r', state)


def test_recovery_from_off_graph_with_active_order_keeps_queue():
    h = RecoveryHarness(make_map())
    h.add('r', 'A')
    h.core.submit('r', 'C', order_id='first')
    h.core.submit('r', 'E', order_id='second')
    h.pump(move=False)
    set_pose(h, -3, 1, 'MANUAL')
    h.buffers['r'].failed = True
    h.core.tick()
    set_pose(h, -3, 1, 'IDLE')
    h.core.tick()
    assert h.core.robots['r'].cancellation
    publish_state(h)
    h.core.tick()
    h.core.tick()
    assert h.recoveries == [('r', 'B')]
    assert list(h.core.robots['r'].queue) == ['first', 'second']
    h.nodes['r'] = 'B'
    set_pose(h, 0, 0, 'IDLE')
    h.core.recovery_result('r', True)
    h.run()
    assert h.core.orders['first'].route == ['B', 'C']
    assert h.core.orders['first'].status == h.core.orders['second'].status == 'COMPLETED'


def test_recovery_without_active_order_uses_nearest_node():
    h = RecoveryHarness(make_map())
    h.add('r', 'A')
    h.pump()
    set_pose(h, -3.5, 3.5, 'MANUAL')
    set_pose(h, -3.5, 3.5, 'IDLE')
    h.core.tick()
    assert h.recoveries == [('r', 'D')]


def test_recovery_goal_can_start_off_graph_but_normal_goal_cannot():
    from robot_agent.route_validator import RouteError
    import pytest
    agent = AgentCore(warehouse=make_map())
    agent.odometry(Pose(-3, 1, 0), 1.)
    with pytest.raises(RouteError):
        agent.validate_route('ordinary', ['B'], 1.)
    agent.start_route('recovery:r', ['B'], 1.)
    agent.tick(1.)
    assert agent.route.target_node_id == 'B'
    assert agent.state in {'MOVING', 'ROTATING'}


def test_recovery_reserves_path_and_waits_for_blocking_robot():
    h = RecoveryHarness(make_map())
    h.add('r', 'A'); h.add('other', 'B')
    h.pump(move=False)
    set_pose(h, -2, 0, 'MANUAL')
    set_pose(h, -2, 0, 'IDLE')
    r = h.core.robots['r']
    r.recovery_target = 'B'
    h.core.tick()
    assert h.recoveries == []
    assert r.status == 'WAITING'
    h.nodes['other'] = 'C'
    h.pump(move=False)
    # The harness places r back at A; restore its off-graph position.
    set_pose(h, -2, 0, 'IDLE')
    r.manual_seen = True
    r.recovery_target = 'B'
    h.core.tick()
    assert h.recoveries == [('r', 'B')]
    assert r.recovery_segment == ((-2, 0), (0, 0))
    assert 'r' in h.core.blockers('other', 'B', 'C')
