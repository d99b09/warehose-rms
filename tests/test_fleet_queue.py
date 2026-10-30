from copy import deepcopy
import pytest
from fleet_dispatcher.core import FleetCore
from protocols.vda5050.models import Identity
from protocols.vda5050.message_factory import MessageFactory
from protocols.vda5050.mapper import order_from_route, state_from_buffer
from vda_bridge.order_buffer import OrderBuffer
from test_vda_queue import warehouse


class Harness:
    def __init__(self, warehouse):
        self.now = 1.
        self.core = FleetCore(warehouse, self, clock=lambda: self.now)
        self.core.connected = True
        self.buffers, self.nodes, self.actions, self.commands = {}, {}, {}, []

    def add(self, rid, node):
        r = self.core.register(rid)
        r.online = True
        self.nodes[rid] = node
        self.buffers[rid] = OrderBuffer(self.core.map, 'map')
        self.actions[rid] = []

    def order(self, rid, eid, version, route, offset, released):
        m = order_from_route(MessageFactory(Identity(serial_number=rid)), self.core.map,
            eid, route, 'map', version=version, offset=offset, released=released)
        self.commands.append((rid, deepcopy(m)))
        self.buffers[rid].accept(m, self.nodes[rid])

    def cancel(self, rid, aid):
        self.buffers[rid].cancel()
        self.actions[rid] = [dict(actionId=aid, actionStatus='FINISHED', actionType='cancelOrder')]

    def uncertain(self, rid):
        return False

    def pump(self, move=True):
        self.now += .1
        for rid, b in self.buffers.items():
            step = b.next_step()
            if step and move:
                self.nodes[rid] = step[-1]
                b.finish_step(True)
            node = self.core.map.nodes[self.nodes[rid]]
            self.core.odometry(rid, node.x, node.y, 0., 0., int(self.now*10000))
            self.core.agent(rid, 'WAITING' if b.waiting else 'IDLE', 'epoch', int(self.now*10000))
            state = state_from_buffer(MessageFactory(Identity(serial_number=rid)), b,
                dict(pose=(node.x, node.y, 0.)), dict(battery_charge=100., map_id='map'), self.actions[rid])
            self.core.state(rid, state)
        self.core.tick()

    def run(self, count=40):
        for _ in range(count):
            self.pump()


def test_fifo_and_idempotency(warehouse):
    h = Harness(warehouse)
    h.add('r', 'A')
    assert h.core.submit('r','C',order_id='one')[0]
    assert h.core.submit('r','D',order_id='two')[0]
    assert h.core.submit('r','C',order_id='one')[0]
    assert not h.core.submit('r','E',order_id='one')[0]
    assert len(h.core.robots['r'].queue) == 2
    h.run()
    assert h.core.orders['one'].status == 'COMPLETED'
    assert h.core.orders['two'].status == 'COMPLETED'
    assert h.nodes['r'] == 'D'


def test_idle_robot_yields_and_stays(warehouse):
    h = Harness(warehouse)
    h.add('a', 'A'); h.add('b', 'B')
    h.core.submit('a','C',order_id='one')
    h.run()
    assert h.core.orders['one'].status == 'COMPLETED'
    assert h.nodes == {'a': 'C', 'b': 'D'}
    assert any(rid == 'b' for rid, _ in h.commands)


@pytest.fixture
def held_stress_fleet():
    from pathlib import Path
    from warehouse_sim.warehouse_map import WarehouseMap

    warehouse = WarehouseMap.from_yaml(str(
        Path(__file__).resolve().parents[1] / 'maps/nav/warehouse_stress/0.yaml'))
    h = Harness(warehouse)
    for rid, node in [('robot_01', 'R2C3'), ('robot_02', 'R3C4'),
                      ('robot_03', 'R3C3'), ('robot_04', 'R2C4'),
                      ('robot_05', 'R3C5')]:
        h.add(rid, node)
    h.pump()
    for rid, target in [('robot_01', 'R2C4'), ('robot_02', 'R2C2'),
                        ('robot_03', 'R2C2'), ('robot_05', 'R3C2')]:
        assert h.core.submit(rid, target, order_id=rid)[0]
        robot = h.core.robots[rid]
        h.core._start(robot, h.core.planner.find_path(h.nodes[rid], target))
    # robot_04 already vacated R2C3 for robot_03, but now blocks robot_01.
    h.core.robots['robot_04'].clearance_for = (
        'robot_03', h.core.robots['robot_03'].execution.execution_id)
    return h


def test_held_idle_robot_yields_again_and_unblocks_five_robot_chain(held_stress_fleet):
    h = held_stress_fleet
    idle = h.core.robots['robot_04']
    h.pump(move=False)
    assert idle.execution and idle.execution.service
    assert idle.execution.route == ['R2C4', 'R2C5']
    assert not idle.queue
    h.run(100)
    assert all(order.status == 'COMPLETED' for order in h.core.orders.values())
    assert h.nodes['robot_01'] == 'R2C4'
    # A completed robot may subsequently vacate its destination for a later job.
    assert h.core.orders['robot_02'].route[-1] == 'R2C2'
    assert h.nodes['robot_05'] == 'R3C2'
    assert h.nodes['robot_04'] == 'R2C5'


def test_idle_robot_does_not_move_when_no_safe_escape(held_stress_fleet):
    from fleet_dispatcher.planner import PathPlanner

    h = held_stress_fleet
    h.core.map.edges = [edge for edge in h.core.map.edges
                        if {edge.source, edge.target} != {'R2C4', 'R2C5'}]
    h.core.planner = PathPlanner(h.core.map)
    idle = h.core.robots['robot_04']
    h.run(10)
    assert idle.execution is None and idle.reserved is None
    assert idle.status == 'IDLE'
    assert h.nodes['robot_04'] == 'R2C4'
    assert not any(rid == 'robot_04' for rid, _ in h.commands)


def test_held_robot_with_queue_yields_to_earlier_job_without_resuming_fifo(held_stress_fleet):
    h = held_stress_fleet
    idle = h.core.robots['robot_04']
    assert h.core.submit('robot_04', 'R2C1', order_id='first')[0]
    assert h.core.submit('robot_04', 'P1', order_id='second')[0]
    for _ in range(10):
        h.pump(move=False)
    assert idle.execution.service
    assert idle.clearance_for[0] == 'robot_01'
    assert list(idle.queue) == ['first', 'second']
    assert h.core.orders['first'].status == 'SUSPENDED'
    assert h.core.orders['second'].status == 'QUEUED'
    h.run(200)
    assert all(order.status == 'COMPLETED' for order in h.core.orders.values())


def test_deadlock_yields_preserves_goal_and_replans(warehouse):
    h = Harness(warehouse)
    h.add('a', 'A'); h.add('b', 'B')
    h.core.submit('a','C',order_id='one')
    h.core.submit('b','A',order_id='two')
    h.run()
    assert h.core.orders['one'].status == h.core.orders['two'].status == 'COMPLETED'
    assert h.nodes == {'a': 'C', 'b': 'A'}
    b_executions = {m['orderId'] for rid, m in h.commands if rid == 'b'}
    assert len(b_executions) == 3  # original, yield, resume
    assert h.core.orders['two'].route == ['D','B','A']


def test_no_pocket_preserves_queue(warehouse):
    warehouse.edges = [e for e in warehouse.edges if not {'D', 'E'} & {e.source, e.target}]
    h = Harness(warehouse)
    h.add('a', 'A'); h.add('b', 'B')
    h.core.submit('a','C',order_id='one')
    h.core.submit('b','A',order_id='two')
    h.run(10)
    assert h.nodes == {'a':'A', 'b':'B'}
    assert h.core.robots['a'].status == 'WAITING'
    assert h.core.robots['b'].queue


def test_single_winner_for_same_node_and_departing_claim_held(warehouse):
    h = Harness(warehouse)
    h.add('a','A'); h.add('b','C')
    h.core.submit('a','B'); h.core.submit('b','B')
    for _ in range(5): h.pump(move=False)
    a, b = h.core.robots['a'], h.core.robots['b']
    assert a.reserved == ('A','B')
    assert b.reserved is None
    assert 'a' in b.blockers
    h.core.connection(False)
    h.core.tick()
    assert a.reserved == ('A','B')


def test_queue_limits(warehouse):
    h = Harness(warehouse); h.add('r','A')
    for _ in range(16): assert h.core.submit('r','C')[0]
    assert h.core.submit('r','C')[1] == 409


def test_stale_result_never_releases_claim(warehouse):
    h = Harness(warehouse); h.add('r','A'); h.core.submit('r','C')
    for _ in range(4): h.pump(move=False)
    r = h.core.robots['r']
    assert r.reserved
    r.state['lastNodeId'] = 'B'; r.state['lastNodeSequenceId'] = 2
    h.now += 10
    h.core.tick()
    assert r.reserved == ('A','B')


def test_geometric_crossing_blocks_distinct_edges(warehouse):
    from warehouse_sim.warehouse_map import WarehouseMap
    m = WarehouseMap(dict(bounds=dict(min_x=-10,max_x=10,min_y=-10,max_y=10),
        nodes=[dict(id=n,position=p) for n,p in dict(A=(-4,0),B=(4,0),C=(0,-4),D=(0,4)).items()],
        edges=[dict(source='A',target='B'),dict(source='C',target='D')],walls=[],parking_nodes=['A','C']))
    h = Harness(m); h.add('a','A'); h.add('b','C')
    h.core.submit('a','B'); h.core.submit('b','D')
    for _ in range(5): h.pump(move=False)
    assert h.core.robots['a'].reserved
    assert not h.core.robots['b'].reserved


def test_three_robot_cycle_yields_to_oldest(warehouse):
    from warehouse_sim.warehouse_map import WarehouseMap
    m = WarehouseMap(dict(bounds=dict(min_x=-12,max_x=12,min_y=-12,max_y=12),
        nodes=[dict(id=n,position=p) for n,p in dict(A=(-4,0),B=(4,0),C=(0,6),D=(0,10),E=(8,6)).items()],
        edges=[dict(source=a,target=b) for a,b in [('A','B'),('B','C'),('C','A'),('C','D'),('C','E')]],
        walls=[],parking_nodes=['A','B','C']))
    h = Harness(m)
    for rid, node in [('a','A'),('b','B'),('c','C')]: h.add(rid,node)
    for rid, target in [('a','B'),('b','C'),('c','A')]: h.core.submit(rid,target,order_id=rid)
    h.run(80)
    assert all(o.status == 'COMPLETED' for o in h.core.orders.values())
    assert h.nodes['a'] == 'B' and h.nodes['c'] == 'A'
    assert h.nodes['b'] == 'E'  # completed robot subsequently vacates C for c


def test_unknown_or_manual_position_prevents_new_grants(warehouse):
    h = Harness(warehouse); h.add('a','A'); h.add('b','E')
    h.core.submit('a','C')
    h.pump(move=False)
    r = h.core.robots['b']
    r.agent_status = 'MANUAL'
    h.core.tick()
    assert h.core.robots['a'].reserved is None


def test_independent_edges_can_move_simultaneously(warehouse):
    from warehouse_sim.warehouse_map import WarehouseMap
    m = WarehouseMap(dict(bounds=dict(min_x=-10,max_x=10,min_y=-10,max_y=10),
        nodes=[dict(id=n,position=p) for n,p in dict(A=(-4,-4),B=(4,-4),C=(-4,4),D=(4,4)).items()],
        edges=[dict(source='A',target='B'),dict(source='C',target='D')],walls=[],parking_nodes=['A','C']))
    h = Harness(m); h.add('a','A'); h.add('b','C')
    h.core.submit('a','B'); h.core.submit('b','D')
    for _ in range(4): h.pump(move=False)
    assert h.core.robots['a'].reserved and h.core.robots['b'].reserved


def test_startup_cancels_old_execution_without_restoring_user_queue(warehouse):
    h = Harness(warehouse); h.add('a','A')
    h.core.submit('a','C')
    for _ in range(4): h.pump(move=False)
    h.core = FleetCore(warehouse, h, clock=lambda: h.now)
    h.core.connected = True
    h.core.register('a').online = True
    h.run(6)
    assert h.buffers['a'].cancelled
    assert h.core.robots['a'].synchronized
    assert not h.core.orders and not h.core.robots['a'].queue
    assert h.nodes['a'] == 'B'  # at most the already released edge may finish


def test_waiting_does_not_trigger_stuck_monitor(warehouse):
    from robot_agent.core import AgentCore
    from warehouse_sim.kinematics import Pose
    agent = AgentCore(warehouse=warehouse)
    agent.state = 'WAITING'
    for i in range(100):
        agent.odometry(Pose(-4,0,0), i*.1)
        command = agent.tick(i*.1)
        assert command.linear == command.angular == 0
    assert agent.state == 'WAITING'


def test_manual_relocation_replans_without_losing_fifo(warehouse):
    h = Harness(warehouse); h.add('a','A')
    h.core.submit('a','C',order_id='first')
    h.core.submit('a','E',order_id='second')
    h.pump(move=False)  # complete Horizon queued, no edge released yet
    h.core.agent('a','MANUAL','epoch',int(h.now*10000)+1)
    h.core.tick()
    h.buffers['a'].failed = True
    h.nodes['a'] = 'D'
    h.run()
    assert h.core.orders['first'].route == ['D','B','C']
    assert h.core.orders['first'].status == h.core.orders['second'].status == 'COMPLETED'
    assert h.nodes['a'] == 'E'


@pytest.fixture
def yield_node_map():
    from warehouse_sim.warehouse_map import WarehouseMap
    return WarehouseMap(dict(bounds=dict(min_x=-10, max_x=10, min_y=-10, max_y=15),
        nodes=[dict(id=n, position=p) for n, p in
               dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(-4, 4), E=(0, 10)).items()],
        edges=[dict(source=a, target=b) for a, b in
               [('A', 'B'), ('B', 'C'), ('A', 'D'), ('B', 'E')]],
        walls=[], parking_nodes=['A', 'B', 'C']))


def test_yield_path_avoids_requesters_current_node_even_if_otherwise_shorter(yield_node_map, monkeypatch):
    from fleet_dispatcher.core import Execution
    h = Harness(yield_node_map)
    h.add('a', 'A'); h.add('b', 'B')
    h.pump()
    requester = h.core.robots['a']
    requester.execution = Execution(['A', 'B', 'C'])
    # Isolate the graph invariant from the separate geometric collision checks.
    monkeypatch.setattr(h.core, 'blockers', lambda *_: set())
    assert h.core.yield_planner.plan(requester, h.core.robots['b'])[0].route == ['B', 'E']


def test_yield_waits_if_only_exit_crosses_requester_node(yield_node_map, monkeypatch):
    from fleet_dispatcher.core import Execution
    yield_node_map.edges = [edge for edge in yield_node_map.edges if edge.target != 'E']
    h = Harness(yield_node_map)
    h.add('a', 'A'); h.add('b', 'B')
    h.pump()
    requester = h.core.robots['a']
    requester.execution = Execution(['A', 'B', 'C'])
    monkeypatch.setattr(h.core, 'blockers', lambda *_: set())
    assert h.core.yield_planner.plan(requester, h.core.robots['b']) is None


def test_yield_step_waits_for_requesters_new_current_node(yield_node_map, monkeypatch):
    h = Harness(yield_node_map)
    h.add('a', 'A'); h.add('b', 'B')
    h.pump()
    requester, yielding = h.core.robots['a'], h.core.robots['b']
    h.core._start(requester, ['A', 'B', 'C'])
    h.core._start(yielding, ['B', 'E'], service=True, beneficiary=requester,
                  forbidden_nodes={'A'})
    # Telemetry can reach a new node before the earlier progress is acknowledged.
    h.nodes['a'] = 'E'
    monkeypatch.setattr(h.core, 'blockers', lambda *_: set())
    h.pump(move=False)
    assert yielding.status == 'WAITING'
    assert yielding.reserved is None
    assert 'Узел робота' in yielding.message
    h.nodes['a'] = 'A'
    h.pump(move=False)
    assert yielding.reserved == ('B', 'E')


def test_yield_path_does_not_cross_requester_node_inside_an_edge(monkeypatch):
    from warehouse_sim.warehouse_map import WarehouseMap
    from fleet_dispatcher.core import Execution
    warehouse = WarehouseMap(dict(bounds=dict(min_x=-15, max_x=15, min_y=-15, max_y=15),
        nodes=[dict(id=n, position=p) for n, p in
               dict(A=(0, 0), B=(-4, 0), C=(0, 4), D=(-4, -10), E=(4, 0)).items()],
        edges=[dict(source=a, target=b) for a, b in [('A', 'C'), ('B', 'E'), ('B', 'D')]],
        walls=[], parking_nodes=['A', 'B']))
    h = Harness(warehouse)
    h.add('a', 'A'); h.add('b', 'B')
    h.pump()
    requester = h.core.robots['a']
    requester.execution = Execution(['A', 'C'])
    monkeypatch.setattr(h.core, 'blockers', lambda *_: set())
    assert h.core.yield_planner.plan(requester, h.core.robots['b'])[0].route == ['B', 'D']


def test_yield_replans_after_requester_moves_during_cancellation(warehouse):
    h = Harness(warehouse)
    h.add('a', 'A'); h.add('b', 'B')
    h.core.submit('a', 'C', order_id='one')
    h.core.submit('b', 'A', order_id='two')
    h.pump(move=False)
    h.pump(move=False)
    yielding = h.core.robots['b']
    assert yielding.cancellation and yielding.yield_plan[0] == ['B', 'D']
    # The original escape target became the requester's current node before the
    # pending cancellation completed; keep the original protected node as well.
    h.nodes['a'] = 'D'
    h.pump(move=False)
    assert yielding.execution.service
    assert yielding.execution.route == ['B', 'C', 'E']
    assert yielding.execution.yield_forbidden_nodes == frozenset({'A', 'D'})
    assert list(yielding.queue) == ['two']


def test_yield_does_not_start_if_requester_blocks_only_exit_after_cancellation(warehouse):
    warehouse.edges = [edge for edge in warehouse.edges if edge.target != 'E']
    h = Harness(warehouse)
    h.add('a', 'A'); h.add('b', 'B')
    h.core.submit('a', 'C', order_id='one')
    h.core.submit('b', 'A', order_id='two')
    h.pump(move=False)
    h.pump(move=False)
    yielding = h.core.robots['b']
    assert yielding.cancellation and yielding.yield_plan[0] == ['B', 'D']
    command_count = len(h.commands)
    h.nodes['a'] = 'D'
    h.pump(move=False)
    assert yielding.execution is None
    assert yielding.status == 'WAITING'
    assert list(yielding.queue) == ['two']
    assert len(h.commands) == command_count


def test_restart_does_not_cancel_a_robot_between_nodes(warehouse):
    h = Harness(warehouse)
    h.add('r', 'A')
    h.core.submit('r', 'C')
    for _ in range(4):
        h.pump(move=False)
    buffer = h.buffers['r']
    assert buffer.next_step() == ['A', 'B']
    h.core = FleetCore(warehouse, h, clock=lambda: h.now)
    h.core.connected = True
    robot = h.core.register('r')
    robot.online = True
    h.now += .1
    h.core.odometry('r', -2., 0., .5, 0., int(h.now * 10000))
    h.core.agent('r', 'MOVING', 'epoch', int(h.now * 10000))
    state = state_from_buffer(MessageFactory(Identity(serial_number='r')), buffer,
        dict(pose=(-2., 0., 0.), linear=.5), dict(battery_charge=100., map_id='map'))
    h.core.state('r', state)
    h.core.tick()
    assert robot.cancellation is None
    assert not buffer.cancelled
    assert robot.status == 'WAITING'
    h.nodes['r'] = 'B'
    buffer.finish_step(True)
    h.pump(move=False)
    assert robot.cancellation is not None
    h.pump(move=False)
    assert buffer.cancelled and robot.synchronized


def test_only_if_idle_admission_is_atomic_and_preserves_fifo_default(warehouse):
    h = Harness(warehouse)
    h.add('r', 'A')
    assert not h.core.submit('r', 'C', only_if_idle=True)[0]  # not synchronized yet
    h.pump()
    assert h.core.submit('r', 'C', order_id='manual')[0]
    assert h.core.submit('r', 'D', order_id='generated', only_if_idle=True)[1] == 409
    assert list(h.core.robots['r'].queue) == ['manual']
    assert h.core.submit('r', 'D', order_id='manual-next')[0]
    h.run()
    assert h.core.submit('r', 'C', order_id='generated', only_if_idle=True)[0]
    assert h.core.submit('r', 'C', order_id='generated', only_if_idle=True)[0]
    assert list(h.core.robots['r'].queue) == ['generated']
