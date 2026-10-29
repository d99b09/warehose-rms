"""Incremental orders over the RMS2 directed map."""
from math import hypot
from .validator import ProtocolError, validate


def order_from_route(factory, warehouse, order_id, route, map_id, *, version=0,
                     offset=0, released=0):
    nodes = []
    for i, name in enumerate(route):
        node = warehouse.nodes[name]
        nodes.append(dict(nodeId=name, sequenceId=2*(offset+i),
                          released=i <= released, actions=[],
                          nodePosition=dict(x=node.x, y=node.y, mapId=map_id)))
    edges = [dict(edgeId=f'{a}->{b}', sequenceId=2*(offset+i)+1,
                  released=i < released, startNodeId=a, endNodeId=b, actions=[])
             for i, (a, b) in enumerate(zip(route, route[1:]))]
    message = factory.header('order') | dict(orderId=order_id, orderUpdateId=version,
                                             nodes=nodes, edges=edges)
    validate('order', message, factory.identity)
    route_from_order(message, warehouse, map_id)
    return message


def route_from_order(message, warehouse, map_id):
    route = []
    for node in message['nodes']:
        name, p = node['nodeId'], node['nodePosition']
        if name not in warehouse.nodes or p['mapId'] != map_id:
            raise ProtocolError('Unknown mapId/nodeId')
        actual = warehouse.nodes[name]
        if hypot(actual.x-p['x'], actual.y-p['y']) > 1e-6:
            raise ProtocolError('Node coordinates disagree with map')
        route.append(name)
    allowed = {(e.source, e.target) for e in warehouse.edges}
    allowed |= {(e.target, e.source) for e in warehouse.edges if e.bidirectional}
    if any((a, b) not in allowed for a, b in zip(route, route[1:])):
        raise ProtocolError('Missing or reversed graph edge')
    return route


def state_from_buffer(factory, buffer, telemetry, simulation, actions=(), errors=()):
    order = buffer.order
    current = buffer.completed
    nodes, edges = (order['nodes'], order['edges']) if order else ([], [])
    message = factory.header('state') | dict(
        orderId=order['orderId'] if order else '',
        orderUpdateId=order['orderUpdateId'] if order else 0,
        lastNodeId=nodes[current]['nodeId'] if nodes else telemetry.get('current_node_id', ''),
        lastNodeSequenceId=nodes[current]['sequenceId'] if nodes else 0,
        nodeStates=[{k: n[k] for k in ('nodeId', 'sequenceId', 'released')}
                    for n in nodes[current+1:]] if not buffer.cancelled else [],
        edgeStates=[{k: e[k] for k in ('edgeId', 'sequenceId', 'released')}
                    for e in edges[current:]] if not buffer.cancelled else [],
        driving=abs(telemetry.get('linear', 0)) > .001 or abs(telemetry.get('angular', 0)) > .001,
        paused=False, newBaseRequest=buffer.waiting,
        operatingMode='MANUAL' if telemetry.get('status') == 'MANUAL' else 'AUTOMATIC',
        actionStates=list(actions), errors=list(errors),
        batteryState=dict(batteryCharge=simulation['battery_charge'], charging=False),
        safetyState=dict(eStop='NONE', fieldViolation=telemetry.get('collision', False)))
    if telemetry.get('pose') is not None:
        x, y, theta = telemetry['pose']
        message['agvPosition'] = dict(x=x, y=y, theta=theta, mapId=simulation['map_id'], positionInitialized=True)
        message['velocity'] = dict(vx=telemetry.get('linear', 0), omega=telemetry.get('angular', 0))
    return message
