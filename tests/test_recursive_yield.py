"""Priority inheritance, recursive retreats and side-effect-free backtracking."""
import pytest

from test_fleet_queue import Harness
from warehouse_sim.warehouse_map import WarehouseMap


def make_map(points, edges, *, directed=False):
    return WarehouseMap(dict(
        bounds=dict(min_x=-20, max_x=20, min_y=-20, max_y=20),
        nodes=[dict(id=n, position=p) for n, p in points.items()],
        edges=[dict(source=a, target=b, bidirectional=not directed) for a, b in edges],
        walls=[], parking_nodes=list(points)))


@pytest.fixture
def recursive_fleet():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(0, 8), F=(0, 12)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('D', 'E'), ('E', 'F')]))
    for rid, node in [('a', 'A'), ('b', 'B'), ('c', 'D'), ('d', 'E')]:
        h.add(rid, node)
    return h


def run_until(h, predicate, count=200):
    for _ in range(count):
        h.pump()
        if predicate():
            return
    pytest.fail(f'No progress: {h.core.snapshot()}')


def test_recursive_chain_moves_deepest_first_and_keeps_first_route(recursive_fleet):
    h = recursive_fleet
    assert h.core.submit('a', 'C', order_id='first')[0]
    run_until(h, lambda: h.core.orders['first'].status == 'COMPLETED')
    starts = [(rid, [n['nodeId'] for n in m['nodes']])
              for rid, m in h.commands if m['orderUpdateId'] == 0]
    assert starts == [('a', ['A', 'B', 'C']), ('d', ['E', 'F']),
                      ('c', ['D', 'E']), ('b', ['B', 'D'])]
    assert h.nodes == dict(a='C', b='D', c='E', d='F')
    assert h.core.orders['first'].route == ['A', 'B', 'C']


def test_busy_chain_inherits_oldest_priority_and_preserves_queues(recursive_fleet):
    h = recursive_fleet
    for rid, target, oid in [('a', 'C', 'first'), ('b', 'A', 'b1'),
                             ('b', 'C', 'b2'), ('c', 'B', 'c1'), ('d', 'D', 'd1')]:
        assert h.core.submit(rid, target, order_id=oid)[0]
    original_execution = None
    for _ in range(150):
        h.pump()
        a = h.core.robots['a']
        if a.execution:
            original_execution = original_execution or a.execution.execution_id
            assert a.execution.execution_id == original_execution
            assert a.execution.route == ['A', 'B', 'C']
        if h.core.orders['first'].status == 'COMPLETED':
            break
        assert list(h.core.robots['b'].queue) == ['b1', 'b2']
        assert list(h.core.robots['c'].queue) == ['c1']
        assert list(h.core.robots['d'].queue) == ['d1']
    else:
        pytest.fail(str(h.core.snapshot()))
    run_until(h, lambda: all(o.status == 'COMPLETED' for o in h.core.orders.values()))
    assert h.core.orders['b2'].route[0] == 'A'


def test_parent_tries_another_node_when_nearest_occupant_cannot_retreat():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(0, -8)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('B', 'E')]))
    for rid, node in [('a', 'A'), ('b', 'B'), ('c', 'D')]:
        h.add(rid, node)
    h.core.submit('a', 'C', order_id='first')
    run_until(h, lambda: h.core.orders['first'].status == 'COMPLETED')
    assert h.nodes == dict(a='C', b='E', c='D')
    assert not any(rid == 'c' for rid, _ in h.commands)


def test_later_clearance_does_not_pause_an_independent_older_route():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(-4, 8), E=(0, 8), F=(4, 8), G=(0, 12)),
        [('A', 'B'), ('B', 'C'), ('D', 'E'), ('E', 'F'), ('E', 'G')]))
    for rid, node in [('a', 'A'), ('b', 'D'), ('c', 'E')]:
        h.add(rid, node)
    h.core.submit('a', 'C', order_id='first')
    h.core.submit('b', 'F', order_id='second')
    h.pump(move=False)
    h.pump(move=False)
    assert h.core.robots['a'].reserved == ('A', 'B')
    assert h.core.robots['c'].reserved == ('E', 'G')
    run_until(h, lambda: all(o.status == 'COMPLETED' for o in h.core.orders.values()))
    assert h.nodes == dict(a='C', b='F', c='G')


def test_priority_uses_order_age_rather_than_robot_id():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(-4, 4), E=(0, 4)),
        [('A', 'B'), ('B', 'C'), ('A', 'D'), ('B', 'E')]))
    h.add('z', 'B')
    h.add('a', 'A')
    h.core.submit('z', 'A', order_id='first')
    h.core.submit('a', 'C', order_id='second')
    run_until(h, lambda: h.core.orders['first'].status == 'COMPLETED')
    assert h.core.orders['first'].route == ['B', 'A']
    assert h.nodes['z'] == 'A'
    assert list(h.core.robots['a'].queue) == ['second']
    starts = [(rid, [n['nodeId'] for n in m['nodes']]) for rid, m in h.commands
              if m['orderUpdateId'] == 0]
    assert starts[:3] == [('z', ['B', 'A']), ('a', ['A', 'B', 'C']), ('a', ['A', 'D'])]
    run_until(h, lambda: h.core.orders['second'].status == 'COMPLETED')


def test_child_backtracks_without_moving_failed_descendants():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(4, 4), F=(-4, 8)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('D', 'E'), ('D', 'F')]))
    for rid, node in [('a', 'A'), ('b', 'B'), ('c', 'D'), ('d', 'E')]:
        h.add(rid, node)
    h.core.submit('a', 'C', order_id='first')
    run_until(h, lambda: h.core.orders['first'].status == 'COMPLETED')
    assert h.nodes == dict(a='C', b='D', c='F', d='E')
    assert not any(rid == 'd' for rid, _ in h.commands)


def test_impossible_chain_has_no_partial_movements_or_cancellations():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4)),
        [('A', 'B'), ('B', 'C'), ('B', 'D')]))
    for rid, node in [('a', 'A'), ('b', 'B'), ('c', 'D')]:
        h.add(rid, node)
    h.core.submit('a', 'C', order_id='first')
    h.run(20)
    assert h.nodes == dict(a='A', b='B', c='D')
    assert len(h.commands) == 1 and h.commands[0][1]['orderUpdateId'] == 0
    assert not any(h.actions.values())
    assert all(not r.reserved for r in h.core.robots.values())
    assert 'нет доступной цепочки' in h.core.robots['a'].message


def test_recursive_search_respects_directed_edges():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(0, 8)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('D', 'E')], directed=True))
    for rid, node in [('a', 'A'), ('b', 'B'), ('c', 'D')]:
        h.add(rid, node)
    h.core.submit('a', 'C', order_id='first')
    run_until(h, lambda: h.core.orders['first'].status == 'COMPLETED')
    assert h.nodes == dict(a='C', b='D', c='E')
    for _, message in h.commands:
        path = [n['nodeId'] for n in message['nodes']]
        assert all(h.core.planner.graph.has_edge(a, b) for a, b in zip(path, path[1:]))


def test_exhausted_search_waits_without_publishing_partial_retreats(recursive_fleet):
    h = recursive_fleet
    h.core.yield_planner.search_limit = 1
    h.core.submit('a', 'C', order_id='first')
    h.run(10)
    assert h.core.yield_planner.exhausted
    assert len(h.commands) == 1
    assert 'лимита' in h.core.robots['a'].message


def test_stale_leaf_is_not_moved(recursive_fleet):
    h = recursive_fleet
    h.pump()
    h.core.robots['d'].state_at = float('-inf')
    h.core.submit('a', 'C', order_id='first')
    h.core.tick()
    assert not any(rid != 'a' for rid, _ in h.commands)
    assert h.core.robots['a'].reserved is None


def test_cyclic_retreat_dependencies_do_not_move_any_robot():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(4, 8), F=(-4, 8)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('D', 'E'), ('E', 'F'), ('F', 'D')]))
    for rid, node in [('a', 'A'), ('b', 'B'), ('c', 'D'), ('d', 'E'), ('e', 'F')]:
        h.add(rid, node)
    h.core.submit('a', 'C', order_id='first')
    h.run(10)
    assert not h.core.yield_planner.exhausted
    assert len(h.commands) == 1 and not any(h.actions.values())
    assert all(r.execution is None for rid, r in h.core.robots.items() if rid != 'a')


def test_recursive_retreat_is_revalidated_after_cancel_confirmation():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(0, 8),
             F=(0, 12), G=(4, 12)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('D', 'E'), ('E', 'F'), ('F', 'G')]))
    for rid, node in [('a', 'A'), ('b', 'B'), ('c', 'D'), ('d', 'E'), ('x', 'G')]:
        h.add(rid, node)
    h.core.submit('a', 'C', order_id='first')
    h.core.submit('d', 'B', order_id='later')
    h.pump(move=False)
    h.pump(move=False)
    leaf = h.core.robots['d']
    assert leaf.cancellation and leaf.yield_plan[0] == ['E', 'F']
    assert leaf.clearance_paths[-1][0] == 'b'  # Keep the parent escape protected too.
    count = len(h.commands)
    h.nodes['x'] = 'F'
    h.pump(move=False)
    assert leaf.execution is None and leaf.reserved is None
    assert not any(rid == 'd' for rid, _ in h.commands[count:])
    assert list(leaf.queue) == ['later']


def test_replaced_priority_execution_does_not_start_old_retreat(recursive_fleet):
    h = recursive_fleet
    h.core.submit('a', 'C', order_id='first')
    h.core.submit('d', 'B', order_id='later')
    h.pump(move=False)
    h.pump(move=False)
    leaf = h.core.robots['d']
    assert leaf.cancellation
    count = len(h.commands)
    h.core.robots['a'].execution.execution_id = 'replaced'
    h.pump(move=False)
    assert leaf.execution is None and leaf.reserved is None
    assert not any(rid == 'd' for rid, _ in h.commands[count:])
    assert list(leaf.queue) == ['later']
