"""Lost robots exclude their nodes from routing while preserving fleet queues."""
import pytest

from fleet_dispatcher.planner import PathPlanner
from warehouse_web.backend.state_store import StateStore
from test_fleet_queue import Harness
from test_manual_routing import make_map
from test_vda_queue import warehouse


def fleet():
    h = Harness(make_map())
    h.add('auto', 'A')
    h.add('offline', 'B')
    h.pump(move=False)
    return h


@pytest.mark.parametrize('broker_connected,robot_online', [(True, False), (False, True), (False, False)])
def test_disconnected_robot_rejects_new_order_before_planning(monkeypatch, broker_connected, robot_online):
    h = fleet()
    h.core.submit('auto', 'C', order_id='existing')
    r = h.core.robots['auto']
    serial = h.core.serial
    h.core.connected, r.online = broker_connected, robot_online

    def unexpected_path(*args, **kwargs):
        pytest.fail('A disconnected robot must not reach route planning')

    monkeypatch.setattr(h.core.planner, 'find_path', unexpected_path)
    accepted, status, message, order = h.core.submit('auto', 'E', order_id='new')
    assert not accepted and status == 409
    assert 'Нет связи с роботом' in message
    assert order is None
    assert list(r.queue) == ['existing']
    assert 'new' not in h.core.orders and h.core.serial == serial
    assert not h.commands


@pytest.mark.parametrize('disconnected_id', ['auto', 'offline'])
def test_unavailable_robot_or_route_error_reaches_http_client(monkeypatch, warehouse, disconnected_id):
    from fastapi.testclient import TestClient
    from warehouse_web.backend.app import create_app
    from warehouse_web.backend.ros_bridge import RosBridge

    h = Harness(warehouse)
    h.add('auto', 'A')
    h.add('offline', 'B')
    h.pump(move=False)
    h.core.robots[disconnected_id].online = False
    monkeypatch.setattr(RosBridge, 'start', lambda self: None)
    monkeypatch.setattr(RosBridge, 'stop', lambda self: None)

    def submit(self, robot_id, target_node_id, order_type, order_id):
        ok, status, message, order = h.core.submit(robot_id, target_node_id, order_type, order_id)
        return dict(accepted=ok, http_status=status, message=message,
                    order_id=order.order_id if order else '', route=order.route if order else [])

    monkeypatch.setattr(RosBridge, 'submit_order', submit)
    with TestClient(create_app()) as client:
        response = client.post('/api/orders', json=dict(robot_id='auto', target_node_id='C',
                                                       order_type='MOVE', order_id='offline-order'))
    assert response.status_code == 409
    assert response.json()['accepted'] is False
    assert response.json()['route'] == []
    assert ('Нет связи с роботом' if disconnected_id == 'auto' else
            'Маршрут до целевого узла не найден') in response.json()['message']
    assert not h.core.robots['auto'].queue


@pytest.mark.parametrize('queued_order', [False, True])
def test_only_route_through_offline_node_rejects_order_without_changing_queue(warehouse, queued_order):
    h = Harness(warehouse)
    h.add('auto', 'A')
    h.add('offline', 'B')
    h.pump(move=False)
    if queued_order:
        assert h.core.submit('auto', 'C', order_id='existing')[0]
    expected_queue = list(h.core.robots['auto'].queue)
    serial = h.core.serial
    h.core.robots['offline'].online = False
    target = 'A' if queued_order else 'C'
    accepted, status, message, order = h.core.submit('auto', target, order_id='blocked')
    assert not accepted and status == 409
    assert message == 'Маршрут до целевого узла не найден'
    assert order is None and 'blocked' not in h.core.orders
    assert list(h.core.robots['auto'].queue) == expected_queue
    assert h.core.serial == serial and not h.commands
    h.core.robots['offline'].online = True
    assert h.core.submit('auto', target, order_id='blocked')[0]


def test_order_to_node_occupied_by_offline_robot_is_rejected():
    h = fleet()
    h.core.robots['offline'].online = False
    accepted, status, message, order = h.core.submit('auto', 'B', order_id='blocked-target')
    assert not accepted and status == 409
    assert message == 'Узел заблокирован роботом без связи'
    assert order is None and not h.core.robots['auto'].queue


def test_new_route_avoids_offline_robot_and_publishes_blocked_node():
    h = fleet()
    h.core.robots['offline'].online = False
    assert h.core.submit('auto', 'C', order_id='detour')[0]
    h.run()
    assert h.core.orders['detour'].route == ['A', 'D', 'E', 'C']
    assert h.core.orders['detour'].status == 'COMPLETED'
    assert not any(rid == 'offline' for rid, _ in h.commands)
    store = StateStore()
    store.update_fleet(h.core.snapshot())
    assert store.snapshot().fleet.blocked_nodes == ('B',)


def test_existing_horizon_is_cancelled_and_replanned_around_offline_robot():
    h = fleet()
    h.core.submit('auto', 'C', order_id='first')
    h.core.submit('auto', 'E', order_id='second')
    # Deliver the initial Horizon without advancing into clearance planning.
    h.core._start(h.core.robots['auto'], ['A', 'B', 'C'])
    h.core.robots['offline'].online = False
    h.pump(move=False)
    assert h.core.robots['auto'].cancellation
    assert h.core.robots['auto'].reserved is None
    assert list(h.core.robots['auto'].queue) == ['first', 'second']
    h.run()
    assert h.core.orders['first'].route == ['A', 'D', 'E', 'C']
    assert all(o.status == 'COMPLETED' for o in h.core.orders.values())


def test_no_detour_preserves_fifo_until_connection_returns(warehouse):
    h = Harness(warehouse)
    h.add('auto', 'A')
    h.add('offline', 'B')
    h.pump(move=False)
    h.core.submit('auto', 'C', order_id='first')
    h.core.submit('auto', 'A', order_id='second')
    h.core.robots['offline'].online = False
    h.run(8)
    assert list(h.core.robots['auto'].queue) == ['first', 'second']
    assert h.core.robots['auto'].status == 'WAITING'
    assert h.core.robots['auto'].execution is None
    h.core.robots['offline'].online = True
    h.run(80)
    assert all(o.status == 'COMPLETED' for o in h.core.orders.values())
    assert h.core.snapshot()['blocked_nodes'] == []


def test_offline_reserved_transition_is_kept_until_resynchronized():
    h = fleet()
    r = h.core.robots['offline']
    h.core._start(r, ['B', 'C'])
    h.pump(move=False)
    assert r.reserved == ('B', 'C')
    r.online = False
    h.run(5)
    assert r.reserved == ('B', 'C')
    assert h.core.unavailable_nodes('auto') == {'B', 'C'}
    r.online = True
    h.run()
    assert r.reserved is None
    assert h.core.unavailable_nodes() == set()


def test_stale_state_also_blocks_last_known_node():
    h = fleet()
    h.core.robots['offline'].state_at = float('-inf')
    assert h.core.unavailable_nodes('auto') == {'B'}


def test_reconnection_keeps_block_until_state_is_synchronized():
    h = fleet()
    r = h.core.robots['offline']
    r.synchronized = False
    assert h.core.fresh(r)
    assert h.core.unavailable_nodes('auto') == {'B'}
    h.pump(move=False)
    assert h.core.unavailable_nodes('auto') == set()


def test_planner_excludes_blocked_endpoints_without_mutating_graph():
    planner = PathPlanner(make_map())
    assert planner.find_path('A', 'C', {'B'}) == ['A', 'D', 'E', 'C']
    with pytest.raises(ValueError, match='заблокирован'):
        planner.find_path('A', 'B', {'B'})
    with pytest.raises(ValueError, match='заблокирован'):
        planner.find_path('B', 'B', {'B'})
    assert planner.find_path('A', 'C') == ['A', 'B', 'C']


def test_http_connection_toggle_and_failures(monkeypatch):
    from fastapi.testclient import TestClient
    from warehouse_web.backend.app import create_app
    from warehouse_web.backend.ros_bridge import RosBridge

    monkeypatch.setattr(RosBridge, 'start', lambda self: None)
    monkeypatch.setattr(RosBridge, 'stop', lambda self: None)
    calls = []

    def toggle(self, robot_id, enabled):
        if robot_id == 'missing':
            raise ValueError('Неизвестный робот')
        if robot_id == 'unavailable':
            raise ConnectionError('Адаптер робота недоступен')
        calls.append((robot_id, enabled))
        return dict(accepted=True, enabled=enabled, message='OK')

    monkeypatch.setattr(RosBridge, 'robot_connection', toggle)
    with TestClient(create_app()) as client:
        for enabled in (False, True):
            reply = client.post('/api/robots/r/connection', json=dict(enabled=enabled))
            assert reply.status_code == 200
            assert reply.json()['enabled'] is enabled
        assert client.post('/api/robots/r/connection', json=dict(enabled='false')).status_code == 422
        assert client.post('/api/robots/missing/connection', json=dict(enabled=False)).status_code == 409
        assert client.post('/api/robots/unavailable/connection', json=dict(enabled=False)).status_code == 503
    assert calls == [('r', False), ('r', True)]
