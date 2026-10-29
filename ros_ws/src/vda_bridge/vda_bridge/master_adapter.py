"""MQTT transport owned by the dispatcher event loop."""
from datetime import datetime, timezone
import time
from queue import Full

from protocols.vda5050.message_factory import MessageFactory
from protocols.vda5050.mapper import order_from_route
from .config import identity
from .mqtt_client import MqttClient


class MasterAdapter:
    def __init__(self, warehouse, config, *, clock=time.monotonic):
        self.map, self.config, self.clock = warehouse, config, clock
        self.identities, self.factories, self.pending, self.stamps = {}, {}, {}, {}
        self.core = None
        self.client = MqttClient(config['mqtt'] | {'command_max_age': config['simulation']['command_max_age']},
                                 self.identities, 'master')
        self.client.start()

    def register(self, rid):
        if rid in self.factories:
            return
        self.identities[rid] = identity(self.config, rid)
        self.factories[rid] = MessageFactory(self.identities[rid])
        for kind in ('state', 'connection', 'factsheet'):
            self.client.subscribe(rid, kind)

    def _send(self, rid, kind, message):
        self.pending[rid] = dict(kind=kind, message=message, started=self.clock(), last=float('-inf'))
        self._publish(rid)

    def _publish(self, rid):
        p = self.pending[rid]
        if not self.client.connected:
            return
        # Repetition preserves command identity/body; transport headers are new.
        p['message'].update(self.factories[rid].header(p['kind']))
        try:
            self.client.publish(rid, p['kind'], p['message'])
            p['last'] = self.clock()
        except (ConnectionError, Full):
            pass

    def order(self, rid, eid, version, route, offset, released):
        message = order_from_route(self.factories[rid], self.map, eid, route,
            self.config['simulation']['map_id'], version=version, offset=offset, released=released)
        self._send(rid, 'order', message)

    def cancel(self, rid, aid):
        self._send(rid, 'instantActions', self.factories[rid].instant('cancelOrder', aid))

    def uncertain(self, rid):
        p = self.pending.get(rid)
        return bool(p and (p['started'] is None or self.clock()-p['started'] >= 5.))

    def poll(self):
        for kind, rid, message in self.client.drain():
            if kind in {'connected', 'disconnected'}:
                self.core.connection(kind == 'connected')
                self.stamps.clear()
                for p in self.pending.values():
                    p['started'] = None
                continue
            if rid not in self.core.robots:
                continue
            r = self.core.robots[rid]
            if kind == 'connection':
                r.online = message['connectionState'] == 'ONLINE'
                if not r.online:
                    r.synchronized = False
                    r.state_at = float('-inf')
            elif kind == 'state':
                stamp = datetime.fromisoformat(message['timestamp'].replace('Z', '+00:00')).timestamp()
                age = datetime.now(timezone.utc).timestamp()-stamp
                if not -2 <= age < self.config['simulation']['state_timeout'] or stamp <= self.stamps.get(rid, float('-inf')):
                    continue
                self.stamps[rid] = stamp
                self.core.state(rid, message)
                p = self.pending.get(rid)
                if p:
                    command = p['message']
                    if p['kind'] == 'order':
                        ack = (message['orderId'], message['orderUpdateId']) == (command['orderId'], command['orderUpdateId'])
                        reconcile = (message['orderId'] == command['orderId'] and message['orderUpdateId'] < command['orderUpdateId']) or (not message['driving'] and not message['nodeStates'] and command['orderUpdateId'] == 0)
                    else:
                        aid = command['actions'][0]['actionId']
                        result = next((a for a in message['actionStates'] if a['actionId'] == aid), None)
                        ack = bool(result and result['actionStatus'] in {'FINISHED', 'FAILED'})
                        reconcile = True
                    if ack:
                        del self.pending[rid]
                    elif p['started'] is None and reconcile:
                        p['started'], p['last'] = self.clock(), float('-inf')
            elif kind == 'validation_error':
                r.message = 'Ошибка MQTT: '+message['error']
        for rid, p in list(self.pending.items()):
            if p['started'] is not None and self.clock()-p['started'] < 5. and self.clock()-p['last'] >= 1.:
                self._publish(rid)

    def close(self):
        self.client.close()
