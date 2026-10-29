from collections import defaultdict
from datetime import datetime, timezone
from threading import Lock
from uuid import uuid4
from . import VERSION


class MessageFactory:
    def __init__(self, identity):
        self.identity = identity
        self.counters = defaultdict(int)
        self.lock = Lock()

    def header(self, kind):
        with self.lock:
            count = self.counters[kind]
            self.counters[kind] = (count+1) % (2**32)
        return dict(headerId=count, timestamp=datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z'),
                    version=VERSION, manufacturer=self.identity.manufacturer, serialNumber=self.identity.serial_number)

    def connection(self, state):
        return self.header('connection') | dict(connectionState=state)

    def instant(self, action_type, action_id=None):
        return self.header('instantActions') | dict(actions=[dict(actionType=action_type,
            actionId=action_id or str(uuid4()), blockingType='HARD' if action_type == 'cancelOrder' else 'NONE')])

    def factsheet(self, simulation):
        # Simulation capability model: no physical battery, load handling or scanner.
        return self.header('factsheet') | dict(
            typeSpecification=dict(seriesName='WarehouseSim', seriesDescription='2D kinematic simulator; incremental Base/Horizon; no payload hardware',
                agvKinematic='DIFF', agvClass='CARRIER', maxLoadMass=0,
                localizationTypes=['NATURAL'], navigationTypes=['VIRTUAL_LINE_GUIDED']),
            physicalParameters=dict(speedMin=0, speedMax=simulation['speed_max'],
                accelerationMax=simulation['acceleration_max'], decelerationMax=simulation['deceleration_max'],
                heightMax=simulation['height'], width=simulation['width'], length=simulation['length']),
            protocolLimits=dict(maxStringLens=dict(msgLen=1048576), maxArrayLens={},
                timing=dict(minOrderInterval=0, minStateInterval=0, defaultStateInterval=simulation['state_interval'])),
            protocolFeatures=dict(optionalParameters=[dict(parameter='order.nodes.nodePosition', support='REQUIRED')],
                agvActions=[dict(actionType=a, actionScopes=['INSTANT']) for a in
                    ['cancelOrder', 'factsheetRequest']]),
            agvGeometry=dict(envelopes2d=[dict(set='body', polygonPoints=[dict(x=x, y=y)
                for x,y in [(-simulation['length']/2,-simulation['width']/2), (simulation['length']/2,-simulation['width']/2),
                            (simulation['length']/2,simulation['width']/2), (-simulation['length']/2,simulation['width']/2)]])]),
            loadSpecification={})
