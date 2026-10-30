from warehouse_web.backend.state_store import StateStore
from warehouse_web.backend.models import StateSnapshot
from test_fleet_queue import Harness
from test_vda_queue import warehouse


def test_snapshot_carries_validated_fifo_and_staleness(warehouse):
    h = Harness(warehouse)
    h.add('r','A')
    h.core.submit('r','C',order_id='job')
    h.pump()
    store = StateStore()
    store.update_fleet(h.core.snapshot())
    snapshot = store.snapshot()
    wire = snapshot.model_dump(mode='json')
    StateSnapshot.model_validate(wire)
    assert wire['fleet']['robots'][0]['queue'][0]['order_id'] == 'job'
    assert not wire['fleet']['stale']
    store.fleet_at = float('-inf')
    assert store.snapshot().fleet.stale


def test_http_orders_fifo_duplicate_conflict_and_websocket(warehouse, monkeypatch):
    from fastapi.testclient import TestClient
    from warehouse_web.backend.app import create_app
    from warehouse_web.backend.ros_bridge import RosBridge

    h = Harness(warehouse); h.add('r', 'A')
    monkeypatch.setattr(RosBridge, 'start', lambda self: None)
    monkeypatch.setattr(RosBridge, 'stop', lambda self: None)
    monkeypatch.setattr(RosBridge, 'healthy', property(lambda self: True))

    def submit(self, robot_id, target_node_id, order_type, order_id, only_if_idle=False):
        ok, status, message, order = h.core.submit(robot_id, target_node_id, order_type, order_id,
                                                   only_if_idle=only_if_idle)
        return dict(accepted=ok, http_status=status, message=message,
                    order_id=order.order_id if order else '', route=order.route if order else [])

    monkeypatch.setattr(RosBridge, 'submit_order', submit)
    app = create_app()
    with TestClient(app) as client:
        payload = dict(robot_id='r', target_node_id='C', order_type='MOVE', order_id='first')
        response = client.post('/api/orders', json=payload)
        assert response.status_code == 202 and response.json()['route'] == []
        assert client.post('/api/orders', json=payload).status_code == 202
        assert len(h.core.robots['r'].queue) == 1
        assert client.post('/api/orders', json=payload | {'target_node_id':'D'}).status_code == 409
        assert client.post('/api/orders', json=payload | {'order_id':'second','target_node_id':'D'}).status_code == 202
        app.state.store.update_fleet(h.core.snapshot())
        with client.websocket_connect('/ws/state') as ws:
            data = ws.receive_json()
            assert [o['order_id'] for o in data['fleet']['robots'][0]['queue']] == ['first','second']
            ws.close()
            assert ws.receive()['type'] == 'websocket.close'


def test_http_optional_idle_guard_does_not_queue_behind_an_order(warehouse, monkeypatch):
    from fastapi.testclient import TestClient
    from warehouse_web.backend.app import create_app
    from warehouse_web.backend.ros_bridge import RosBridge

    h = Harness(warehouse)
    h.add('r', 'A')
    h.pump()
    monkeypatch.setattr(RosBridge, 'start', lambda self: None)
    monkeypatch.setattr(RosBridge, 'stop', lambda self: None)
    monkeypatch.setattr(RosBridge, 'healthy', property(lambda self: True))

    def submit(self, robot_id, target_node_id, order_type, order_id, only_if_idle=False):
        ok, status, message, order = h.core.submit(robot_id, target_node_id, order_type, order_id,
                                                   only_if_idle=only_if_idle)
        return dict(accepted=ok, http_status=status, message=message,
                    order_id=order.order_id if order else '', route=order.route if order else [])

    monkeypatch.setattr(RosBridge, 'submit_order', submit)
    with TestClient(create_app()) as client:
        payload = dict(robot_id='r', target_node_id='C', order_type='MOVE', order_id='first')
        assert client.post('/api/orders', json=payload).status_code == 202
        assert client.post('/api/orders', json=payload | {'order_id':'guarded', 'only_if_idle':True}).status_code == 409
        assert client.post('/api/orders', json=payload | {'order_id':'normal'}).status_code == 202
        assert list(h.core.robots['r'].queue) == ['first', 'normal']
