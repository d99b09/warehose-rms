from copy import deepcopy
import pytest
from warehouse_sim.warehouse_map import WarehouseMap
from protocols.vda5050.models import Identity
from protocols.vda5050.message_factory import MessageFactory
from protocols.vda5050.mapper import order_from_route, state_from_buffer
from protocols.vda5050.validator import ProtocolError, validate
from vda_bridge.order_buffer import OrderBuffer
from vda_bridge.journal import Journal


@pytest.fixture
def warehouse():
    return WarehouseMap(dict(bounds=dict(min_x=-10, max_x=15, min_y=-10, max_y=10),
        nodes=[dict(id=n, position=p) for n, p in dict(A=(-4, 0), B=(0, 0), C=(4, 0), D=(0, 4), E=(8, 0)).items()],
        edges=[dict(source=a, target=b) for a, b in [('A','B'), ('B','C'), ('B','D'), ('C','E')]],
        walls=[], parking_nodes=['A','C','D']))


def command(warehouse, version=0, route=None, offset=0, released=0, oid='order'):
    return order_from_route(MessageFactory(Identity()), warehouse, oid, route or ['A', 'B', 'C'],
                            'map', version=version, offset=offset, released=released)


def test_horizon_never_moves_and_updates_do_not_replay(warehouse):
    buffer = OrderBuffer(warehouse, 'map')
    first = command(warehouse)
    assert buffer.accept(first, 'A')
    assert buffer.waiting and not buffer.next_step()
    update = command(warehouse, version=1, released=1)
    assert buffer.accept(update, 'A')
    assert buffer.next_step() == ['A', 'B']
    buffer.inflight = True
    buffer.finish_step(True)
    assert buffer.waiting and not buffer.next_step()
    assert not buffer.accept(update, 'B')
    last = command(warehouse, version=2, route=['B','C'], offset=1, released=1)
    buffer.accept(last, 'B')
    assert buffer.next_step() == ['B','C']
    buffer.finish_step(True)
    assert not buffer.active


def test_stitch_changed_payload_and_old_update_rejected(warehouse):
    buffer = OrderBuffer(warehouse, 'map')
    buffer.accept(command(warehouse), 'A')
    buffer.accept(command(warehouse, version=1, released=1), 'A')
    wrong = command(warehouse, version=2, released=1)
    with pytest.raises(ProtocolError, match='Stitching'):
        buffer.accept(wrong, 'A')
    changed = command(warehouse, version=1, released=2)
    with pytest.raises(ProtocolError, match='Changed payload'):
        buffer.accept(changed, 'A')
    buffer.cancel()
    with pytest.raises(ProtocolError):
        buffer.accept(command(warehouse, version=2, route=['B','C'], offset=1, released=1), 'B')


def test_validator_rejects_released_island(warehouse):
    m = command(warehouse)
    m['nodes'][-1]['released'] = True
    with pytest.raises(ProtocolError, match='Horizon'):
        validate('order', m, Identity())


def test_directed_edge_validation(warehouse):
    warehouse.edges = [type(e)(e.source, e.target, False) for e in warehouse.edges]
    with pytest.raises(ProtocolError, match='reversed'):
        command(warehouse, route=['B','A'])


def test_state_reports_horizon_not_completion(warehouse):
    b = OrderBuffer(warehouse, 'map')
    b.accept(command(warehouse), 'A')
    state = state_from_buffer(MessageFactory(Identity()), b,
        dict(pose=(-4., 0., 0.)), dict(battery_charge=100., map_id='map'))
    validate('state', state, Identity())
    assert state['nodeStates'] and state['newBaseRequest']
    assert not state['driving']


def test_journal_persists_admission_not_replay(warehouse, tmp_path):
    path = tmp_path/'journal.sqlite'
    m = command(warehouse)
    j = Journal(path)
    assert j.admit('order', '["order",0]', m)
    j.close()
    j = Journal(path)
    assert not j.admit('order', '["order",0]', m)
    changed = deepcopy(m)
    changed['nodes'][-1]['nodeId'] = 'D'
    with pytest.raises(ValueError):
        j.admit('order', '["order",0]', changed)
    j.close()


def test_action_receipt_retains_terminal_result(tmp_path):
    j = Journal(tmp_path/'actions.sqlite')
    action = dict(actionId='cancel', actionType='cancelOrder', blockingType='HARD')
    assert j.admit('action','cancel',action)
    j.finish('action','cancel','FINISHED')
    j.close()
    j = Journal(tmp_path/'actions.sqlite')
    assert not j.admit('action','cancel',action)
    assert j.state('action','cancel') == 'FINISHED'
    j.close()
