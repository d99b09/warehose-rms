"""Manual robots are dynamic obstacles for fleet route planning."""
from protocols.vda5050.mapper import state_from_buffer
from protocols.vda5050.message_factory import MessageFactory
from protocols.vda5050.models import Identity
from warehouse_sim.warehouse_map import WarehouseMap
from test_fleet_queue import Harness


def make_map():
    return WarehouseMap(dict(bounds=dict(min_x=-10, max_x=10, min_y=-10, max_y=10),
        nodes=[dict(id=n, position=p) for n, p in
               dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(-4, 4), E=(4, 4)).items()],
        edges=[dict(source=a, target=b) for a, b in
               [('A', 'B'), ('B', 'C'), ('A', 'D'), ('D', 'E'), ('E', 'C')]],
        walls=[], parking_nodes=['A', 'B', 'C']))


def manual(h, x, y):
    h.now += .01
    seq = int(h.now * 10000)
    h.core.odometry('manual', x, y, 0, 0, seq)
    h.core.agent('manual', 'MANUAL', 'epoch', seq)


def test_manual_node_and_edge_are_avoided_and_position_updates():
    h = Harness(make_map())
    h.add('auto', 'A'); h.add('manual', 'B')
    h.pump(move=False)
    manual(h, 0, 0)
    assert h.core.submit('auto', 'C', order_id='one')[0]
    h.core.tick()
    assert h.core.orders['one'].route == ['A', 'D', 'E', 'C']
    # Moving onto the detour triggers cancellation before another edge is granted.
    manual(h, 0, 4)
    state = state_from_buffer(MessageFactory(Identity(serial_number='auto')), h.buffers['auto'],
        dict(pose=(-4, 0, 0)), dict(battery_charge=100., map_id='map'))
    h.core.state('auto', state)
    h.core.tick()
    assert h.core.robots['auto'].cancellation
    assert h.core.robots['auto'].reserved is None


def test_no_detour_waits_and_stale_manual_position_stops_grants():
    m = make_map()
    m.edges = [e for e in m.edges if e.source in {'A', 'B'} and e.target in {'B', 'C'}]
    h = Harness(m)
    h.add('auto', 'A'); h.add('manual', 'B')
    h.pump(move=False)
    manual(h, 0, 0)
    h.core.submit('auto', 'C', order_id='one')
    h.core.tick()
    assert h.core.orders['one'].status == 'QUEUED'
    assert h.core.robots['auto'].execution is None
    h.now += h.core.timeout + .1
    h.core.tick()
    assert h.core.robots['auto'].reserved is None


def test_queued_orders_resume_from_manual_final_node_in_fifo_order():
    h = Harness(make_map())
    h.add('manual', 'A')
    h.pump(move=False)
    manual(h, -4, 0)
    assert h.core.submit('manual', 'C', order_id='first')[0]
    assert h.core.submit('manual', 'E', order_id='second')[0]
    h.core.tick()
    assert not h.commands
    h.nodes['manual'] = 'D'
    h.run()
    assert h.core.orders['first'].route == ['D', 'E', 'C']
    assert h.core.orders['first'].status == 'COMPLETED'
    assert h.core.orders['second'].status == 'COMPLETED'
    assert h.nodes['manual'] == 'E'
