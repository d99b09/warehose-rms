from warehouse_web.backend.state_store import StateStore
from test_fleet_queue import Harness
from test_vda_queue import warehouse


def test_assignment_numbers_survive_web_snapshot_and_duplicate_submission(warehouse):
    h = Harness(warehouse)
    h.add('r2', 'E')
    h.add('r1', 'A')
    assert h.core.submit('r1', 'C', order_id='first')[0]
    assert h.core.submit('r2', 'D', order_id='second')[0]
    assert h.core.submit('r1', 'C', order_id='first')[0]
    assert h.core.submit('r1', 'E', order_id='third')[0]
    store = StateStore()
    store.update_fleet(h.core.snapshot())
    wire = store.snapshot().model_dump(mode='json')
    tasks = sorted((order for robot in wire['fleet']['robots']
                    for order in robot['queue']), key=lambda order: order['number'])
    assert [(order['order_id'], order['number']) for order in tasks] == [
        ('first', 1), ('second', 2), ('third', 3)]


def test_completed_tasks_disappear_without_renumbering_remaining_tasks(warehouse):
    h = Harness(warehouse)
    h.add('r', 'A')
    h.core.submit('r', 'B', order_id='first')
    h.core.submit('r', 'D', order_id='second')
    store = StateStore()
    for _ in range(20):
        h.pump()
        if h.core.orders['first'].status == 'COMPLETED':
            break
    assert h.core.orders['first'].status == 'COMPLETED'
    store.update_fleet(h.core.snapshot())
    queue = store.snapshot().fleet.robots[0].queue
    assert [(order.order_id, order.number) for order in queue] == [('second', 2)]
    h.run()
    store.update_fleet(h.core.snapshot())
    assert store.snapshot().fleet.robots[0].queue == ()


def test_http_assignment_and_websocket_queue_updates(warehouse, monkeypatch):
    from fastapi.testclient import TestClient
    from warehouse_web.backend.app import create_app
    from warehouse_web.backend.ros_bridge import RosBridge

    h = Harness(warehouse)
    h.add('r', 'A')
    monkeypatch.setattr(RosBridge, 'start', lambda self: None)
    monkeypatch.setattr(RosBridge, 'stop', lambda self: None)
    monkeypatch.setattr(RosBridge, 'healthy', property(lambda self: True))

    def submit(self, robot_id, target_node_id, order_type, order_id):
        ok, status, message, order = h.core.submit(
            robot_id, target_node_id, order_type, order_id)
        return dict(accepted=ok, http_status=status, message=message,
                    order_id=order.order_id if order else '', route=order.route if order else [])

    monkeypatch.setattr(RosBridge, 'submit_order', submit)
    app = create_app()
    with TestClient(app) as client:
        for order_id, target in [('first', 'B'), ('second', 'D')]:
            response = client.post('/api/orders', json=dict(
                robot_id='r', target_node_id=target, order_type='MOVE', order_id=order_id))
            assert response.status_code == 202
        app.state.store.update_fleet(h.core.snapshot())
        with client.websocket_connect('/ws/state') as ws:
            queue = ws.receive_json()['fleet']['robots'][0]['queue']
            assert [(task['order_id'], task['number']) for task in queue] == [
                ('first', 1), ('second', 2)]
            h.run()
            assert all(order.status == 'COMPLETED' for order in h.core.orders.values())
            app.state.store.update_fleet(h.core.snapshot())
            for _ in range(20):
                queue = ws.receive_json()['fleet']['robots'][0]['queue']
                if not queue:
                    break
            assert queue == []
