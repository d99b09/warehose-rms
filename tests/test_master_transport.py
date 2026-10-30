from pathlib import Path
from copy import deepcopy
import pytest
from vda_bridge.master_adapter import MasterAdapter
from vda_bridge.config import load_config
from protocols.vda5050.message_factory import MessageFactory
from protocols.vda5050.models import Identity
from protocols.vda5050.mapper import state_from_buffer
from vda_bridge.order_buffer import OrderBuffer
from fleet_dispatcher.core import FleetCore
from test_vda_queue import warehouse


class Client:
    def __init__(self, *args, **kwargs):
        self.connected = True
        self.events, self.sent = [], []
    def start(self): pass
    def subscribe(self, *args): pass
    def publish(self, rid, kind, message): self.sent.append((rid, kind, deepcopy(message)))
    def drain(self):
        events, self.events = self.events, []
        return events
    def close(self): pass


@pytest.fixture
def master(warehouse, monkeypatch):
    monkeypatch.setattr('vda_bridge.master_adapter.MqttClient', Client)
    config = load_config(Path(__file__).resolve().parents[1]/'ros_ws/src/vda_bridge/config/vda5050.yaml')
    now = [1.]
    m = MasterAdapter(warehouse, config, clock=lambda: now[0])
    m.core = FleetCore(warehouse, m, clock=lambda: now[0])
    m.core.register('r'); m.register('r')
    return m, now


def test_retries_same_command_and_timeout_holds_pending(master):
    m, now = master
    m.order('r','job',0,['A','B','C'],0,0)
    first = deepcopy(m.client.sent[-1][2])
    now[0] += 1.
    m.poll()
    assert len(m.client.sent) == 2
    retry = m.client.sent[-1][2]
    assert retry['orderId'] == first['orderId'] and retry['orderUpdateId'] == first['orderUpdateId']
    assert retry['nodes'] == first['nodes']
    assert retry['headerId'] > first['headerId']
    now[0] += 4.
    m.poll()
    assert m.uncertain('r') and 'r' in m.pending
    assert len(m.client.sent) == 2


def test_publish_is_not_ack_and_reconnect_does_not_blindly_replay(master, warehouse):
    m, now = master
    m.order('r','job',0,['A','B'],0,0)
    assert 'r' in m.pending
    m.client.events.append(('connected', None, None)); m.poll()
    assert m.pending['r']['started'] is None
    assert len(m.client.sent) == 1
    b = OrderBuffer(warehouse, m.config['simulation']['map_id'])
    b.accept(m.client.sent[0][2], 'A')
    state = state_from_buffer(MessageFactory(Identity(serial_number='r')), b,
        dict(pose=(-4.,0.,0.)), m.config['simulation'])
    m.client.events.append(('state','r',state)); m.poll()
    assert 'r' not in m.pending


def test_old_timestamp_cannot_acknowledge_command(master, warehouse):
    m, now = master
    m.order('r','job',0,['A','B'],0,0)
    b = OrderBuffer(warehouse, m.config['simulation']['map_id'])
    b.accept(m.client.sent[0][2], 'A')
    state = state_from_buffer(MessageFactory(Identity(serial_number='r')), b,
        dict(pose=(-4.,0.,0.)), m.config['simulation'])
    state['timestamp'] = '2020-01-01T00:00:00Z'
    m.client.events.append(('state','r',state)); m.poll()
    assert 'r' in m.pending


def test_robot_disconnect_suspends_retries_until_online_state_reconciliation(master, warehouse):
    m, now = master
    m.order('r', 'job', 0, ['A', 'B'], 0, 0)
    factory = MessageFactory(Identity(serial_number='r'))
    m.client.events.append(('connection', 'r', factory.connection('CONNECTIONBROKEN')))
    m.poll()
    assert m.pending['r']['started'] is None
    now[0] += 10
    m.poll()
    assert len(m.client.sent) == 1
    m.client.events.append(('connection', 'r', factory.connection('ONLINE')))
    m.poll()
    assert len(m.client.sent) == 1
    state = state_from_buffer(factory, OrderBuffer(warehouse, m.config['simulation']['map_id']),
                             dict(pose=(-4., 0., 0.)), m.config['simulation'])
    m.client.events.append(('state', 'r', state))
    m.poll()
    assert len(m.client.sent) == 2
    assert m.pending['r']['started'] == now[0]
