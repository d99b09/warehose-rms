"""Robot-side Base/Horizon queue. ROS and network independent."""
from copy import deepcopy
from .journal import fingerprint
from protocols.vda5050.mapper import route_from_order
from protocols.vda5050.validator import ProtocolError


class OrderBuffer:
    def __init__(self, warehouse, map_id):
        self.warehouse, self.map_id = warehouse, map_id
        self.order = None
        self.completed = 0
        self.cancelled = False
        self.failed = False
        self.inflight = False
        self.receipts = {}

    @property
    def active(self):
        return bool(self.order and not self.cancelled and
                    (self.inflight or self.failed or self.completed < len(self.order['nodes'])-1))

    @property
    def waiting(self):
        return bool(self.active and not self.failed and not self.inflight and
                    not self.order['edges'][self.completed]['released'])

    def validate(self, message, current_node):
        route_from_order(message, self.warehouse, self.map_id)
        key = (message['orderId'], message['orderUpdateId'])
        payload = fingerprint(message)
        if key in self.receipts:
            if self.receipts[key] != payload:
                raise ProtocolError('Changed payload for existing order update')
            return False
        old = self.order
        if old and message['orderId'] == old['orderId']:
            if self.cancelled or self.failed:
                raise ProtocolError('Cannot update cancelled/failed order')
            if message['orderUpdateId'] <= old['orderUpdateId']:
                raise ProtocolError('Stale order update')
            last = max(i for i, n in enumerate(old['nodes']) if n['released'])
            if message['nodes'][0] != old['nodes'][last]:
                raise ProtocolError('Stitching node must match unchanged end of Base')
        else:
            if self.active or message['orderUpdateId'] != 0:
                raise ProtocolError('Busy robot or nonzero first update')
            if message['nodes'][0]['nodeId'] != current_node:
                raise ProtocolError('Order does not start at confirmed robot position')
        return True

    def accept(self, message, current_node):
        if not self.validate(message, current_node):
            return False
        old = self.order
        if old and old['orderId'] == message['orderId']:
            start = message['nodes'][0]['sequenceId']//2
            merged = deepcopy(message)
            merged['nodes'] = deepcopy(old['nodes'][:start]) + merged['nodes']
            merged['edges'] = deepcopy(old['edges'][:start]) + merged['edges']
            self.order = merged
        else:
            self.order = deepcopy(message)
            self.completed = 0
            self.cancelled = self.failed = False
        self.receipts[(message['orderId'], message['orderUpdateId'])] = fingerprint(message)
        return True

    def next_step(self):
        if not self.active or self.failed or self.inflight:
            return None
        edge = self.order['edges'][self.completed]
        if not edge['released']:
            return None
        return [self.order['nodes'][self.completed]['nodeId'],
                self.order['nodes'][self.completed+1]['nodeId']]

    def finish_step(self, success):
        self.inflight = False
        if success:
            self.completed += 1
        else:
            self.failed = True

    def cancel(self):
        if self.inflight:
            raise RuntimeError('Cancellation needs confirmed local action completion')
        self.cancelled = True
        self.failed = False
