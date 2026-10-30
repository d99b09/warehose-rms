"""Available transitions keep moving while independent clearances run."""
from protocols.vda5050.mapper import state_from_buffer
from protocols.vda5050.message_factory import MessageFactory
from protocols.vda5050.models import Identity

from test_fleet_queue import Harness
from test_recursive_yield import make_map, run_until


def two_corridors():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4),
             X=(-4, 8), Y=(0, 8), Z=(4, 8), W=(0, 12)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('X', 'Y'), ('Y', 'Z'), ('Y', 'W')]))
    h.add('a', 'A'); h.add('b', 'B'); h.add('x', 'X')
    return h


def test_independent_robot_moves_during_yield():
    h = two_corridors()
    h.core.submit('a', 'C', order_id='first')
    h.core.submit('x', 'Z', order_id='second')
    h.pump(move=False); h.pump(move=False)
    assert h.core.robots['b'].reserved == ('B', 'D')
    assert h.core.robots['x'].reserved == ('X', 'Y')


def test_independent_robot_moves_while_another_order_is_cancelled():
    h = two_corridors()
    h.core.submit('a', 'C', order_id='first')
    h.core.submit('b', 'A', order_id='blocked')
    h.core.submit('x', 'Z', order_id='independent')
    h.pump(move=False); h.pump(move=False)
    assert h.core.robots['b'].cancellation
    assert h.core.robots['x'].reserved == ('X', 'Y')


def test_two_independent_clearance_chains_move_in_parallel():
    h = two_corridors()
    h.add('y', 'Y')
    h.core.submit('a', 'C', order_id='first')
    h.core.submit('x', 'Z', order_id='second')
    h.pump(move=False); h.pump(move=False)
    assert h.core.robots['b'].reserved == ('B', 'D')
    assert h.core.robots['y'].reserved == ('Y', 'W')
    run_until(h, lambda: all(o.status == 'COMPLETED' for o in h.core.orders.values()))


def test_free_prefix_moves_while_a_distant_blocker_yields():
    h = Harness(make_map(
        dict(A=(-12, 0), X=(-8, 0), Y=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4)),
        [('A', 'X'), ('X', 'Y'), ('Y', 'B'), ('B', 'C'), ('B', 'D')]))
    h.add('a', 'A'); h.add('b', 'B')
    h.core.submit('a', 'C', order_id='first')
    h.pump(move=False); h.pump(move=False)
    assert h.core.robots['a'].reserved == ('A', 'X')
    assert h.core.robots['b'].reserved == ('B', 'D')


def test_later_robot_can_cross_a_distant_unreserved_part_of_older_route():
    h = Harness(make_map(
        dict(A=(-8, 0), B=(-4, 0), C=(0, 0), D=(4, 0), X=(0, -4), Y=(0, 4)),
        [('A', 'B'), ('B', 'C'), ('C', 'D'), ('X', 'Y')]))
    h.add('a', 'A'); h.add('x', 'X')
    h.core.submit('a', 'D', order_id='first')
    h.core.submit('x', 'Y', order_id='second')
    h.pump(move=False); h.pump(move=False)
    assert h.core.robots['a'].reserved == ('A', 'B')
    assert h.core.robots['x'].reserved == ('X', 'Y')


def test_blocker_keeps_its_execution_when_own_route_vacates_the_path():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(4, 4)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('D', 'E')]))
    h.add('a', 'A'); h.add('b', 'B')
    h.core.submit('a', 'C', order_id='first')
    h.core.submit('b', 'E', order_id='second')
    h.pump(move=False)
    eid = h.core.robots['b'].execution.execution_id
    h.pump(move=False)
    assert h.core.robots['b'].reserved == ('B', 'D')
    assert h.core.robots['b'].execution.execution_id == eid
    assert not h.core.robots['b'].execution.service
    assert not any(h.actions.values())
    run_until(h, lambda: all(o.status == 'COMPLETED' for o in h.core.orders.values()))
    assert len({m['orderId'] for rid, m in h.commands if rid == 'b'}) == 1


def test_completed_retreat_can_resume_an_independent_queue_before_beneficiary_finishes():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(0, 8)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('D', 'E')]))
    h.add('a', 'A'); h.add('b', 'D')
    h.core.submit('a', 'C', order_id='first')
    h.core.submit('b', 'E', order_id='second')
    h.pump(move=False)
    h.core.robots['b'].clearance_for = ('a', h.core.robots['a'].execution.execution_id)
    h.pump(move=False)
    assert h.core.orders['first'].status != 'COMPLETED'
    assert h.core.robots['b'].reserved == ('D', 'E')


def test_return_to_a_passed_section_does_not_wait_for_the_whole_order():
    h = Harness(make_map(
        dict(A=(0, 0), B=(4, 0), C=(8, 0), D=(0, 4)),
        [('A', 'B'), ('B', 'C'), ('A', 'D')]))
    h.add('a', 'B'); h.add('b', 'D')
    h.core.submit('a', 'C', order_id='first')
    h.core.submit('b', 'A', order_id='second')
    h.pump(move=False)
    h.core.robots['b'].clearance_for = ('a', h.core.robots['a'].execution.execution_id)
    h.pump(move=False)
    assert h.core.robots['a'].reserved == ('B', 'C')
    assert h.core.robots['b'].reserved == ('D', 'A')


def test_localized_unsynchronized_robot_does_not_stop_an_independent_route():
    h = two_corridors()
    # The idle blocker is elsewhere; keep the first corridor empty.
    h.nodes['b'] = 'D'
    h.core.submit('a', 'C', order_id='first')
    h.pump(move=False)
    a = h.core.robots['a']
    node = h.core.map.nodes[h.nodes['a']]
    h.core.state('a', state_from_buffer(MessageFactory(Identity(serial_number='a')),
        h.buffers['a'], dict(pose=(node.x, node.y, 0.)), dict(battery_charge=100., map_id='map')))
    x = h.core.robots['x']
    x.synchronized = False
    x.state['driving'] = True
    h.core.tick()
    assert not x.synchronized
    assert a.reserved == ('A', 'B')


def test_priority_robot_does_not_enter_an_ancestors_retreat_path():
    h = Harness(make_map(
        dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(0, 8)),
        [('A', 'B'), ('B', 'C'), ('B', 'D'), ('D', 'E')]))
    h.add('a', 'A'); h.add('b', 'C'); h.add('c', 'D')
    h.core.submit('a', 'C', order_id='first')
    h.pump(move=False); h.pump(move=False)
    assert h.core.robots['c'].reserved == ('D', 'E')
    # b needs C -> B -> D after c leaves; entering B would cut off b.
    assert h.core.robots['a'].reserved is None
    run_until(h, lambda: h.core.orders['first'].status == 'COMPLETED')


def test_approaching_a_cul_de_sac_waits_until_its_occupant_can_escape():
    h = Harness(make_map(
        dict(A=(-8, 0), B=(-4, 0), C=(0, 0), D=(4, 0), E=(0, 4), F=(-4, 4)),
        [('A', 'B'), ('B', 'C'), ('C', 'D'), ('C', 'E'), ('E', 'F')]))
    h.add('a', 'A'); h.add('b', 'D'); h.add('c', 'E')
    h.core.submit('a', 'D', order_id='first')
    h.core.submit('c', 'F', order_id='departing')
    for _ in range(60):
        h.pump()
        if h.nodes['b'] == 'D':
            assert h.core.robots['a'].reserved != ('B', 'C')
        if all(o.status == 'COMPLETED' for o in h.core.orders.values()):
            break
    else:
        raise AssertionError(h.core.snapshot())


def test_busy_grid_does_not_enter_the_only_exit_of_a_blocked_destination():
    h = Harness(make_map(
        {f'{x}{y}': (x*4, y*4) for x in range(3) for y in range(3)},
        [('00','10'),('00','01'),('01','11'),('01','02'),('02','12'),
         ('10','20'),('10','11'),('11','21'),('11','12'),('12','22'),('20','21')]))
    for rid, node in enumerate(['20', '12', '22', '01']):
        h.add(str(rid), node)
    for rid, goal in enumerate(['22', '20', '00', '12']):
        h.core.submit(str(rid), goal, order_id=str(rid))
    run_until(h, lambda: all(o.status == 'COMPLETED' for o in h.core.orders.values()))
