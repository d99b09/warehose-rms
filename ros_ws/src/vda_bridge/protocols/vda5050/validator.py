"""Unmodified official schemas plus the semantic checks they cannot express."""
from functools import lru_cache
from importlib.resources import files
import json
from datetime import datetime, timezone
from jsonschema import Draft202012Validator, FormatChecker, ValidationError
from . import VERSION
from .models import dumps


class ProtocolError(ValueError):
    pass


@lru_cache(maxsize=None)
def schema_validator(kind):
    schema = json.loads(files('protocols.vda5050').joinpath('schemas', kind+'.schema').read_text())
    Draft202012Validator.check_schema(schema)
    return Draft202012Validator(schema, format_checker=FormatChecker())


def validate(kind, message, identity):
    try:
        dumps(message)  # IEEE NaN/Infinity forbidden, even where schema says number.
        schema_validator(kind).validate(message)
        if message['version'] != VERSION:
            raise ProtocolError('Only VDA 5050 2.1.0 is supported')
        if (message['manufacturer'], message['serialNumber']) != (identity.manufacturer, identity.serial_number):
            raise ProtocolError('Manufacturer/serialNumber mismatch')
        # factsheet tag omits some header requirements; profile requires all headers.
        for key in ('headerId', 'timestamp', 'version', 'manufacturer', 'serialNumber'):
            if key not in message:
                raise ProtocolError('Missing header: '+key)
        if type(message['headerId']) is not int or not 0 <= message['headerId'] < 2**32:
            raise ProtocolError('headerId must be uint32')
        stamp = datetime.fromisoformat(message['timestamp'].replace('Z', '+00:00'))
        if stamp.utcoffset() != timezone.utc.utcoffset(stamp):
            raise ProtocolError('timestamp must include UTC timezone')
        if kind == 'order':
            validate_order(message)
        if kind == 'instantActions':
            ids = [a['actionId'] for a in message['actions']]
            if len(set(ids)) != len(ids) or any(not x for x in ids):
                raise ProtocolError('actionId must be unique and nonempty')
    except (ValueError, KeyError, TypeError) as exc:
        raise ProtocolError(str(exc)) from exc
    except ValidationError as exc:
        raise ProtocolError(exc.message) from exc
    return message


def validate_order(message):
    nodes, edges = message['nodes'], message['edges']
    if type(message['orderUpdateId']) is not int:
        raise ProtocolError('orderUpdateId must be uint32')
    if not message['orderId'] or not nodes or len(edges) != len(nodes)-1:
        raise ProtocolError('Order requires a nonempty ID and an alternating connected path')
    if set(message) - {'headerId','timestamp','version','manufacturer','serialNumber','orderId','orderUpdateId','nodes','edges'}:
        raise ProtocolError('Unsupported order option')
    offset = nodes[0]['sequenceId']
    if offset % 2 or not nodes[0]['released'] or (message['orderUpdateId'] == 0 and offset != 0):
        raise ProtocolError('Invalid start sequence or unreleased first node')
    horizon = False
    for item in [x for pair in zip(nodes, edges) for x in pair] + [nodes[-1]]:
        if horizon and item['released']:
            raise ProtocolError('Released element after Horizon')
        horizon = horizon or not item['released']
    for i, node in enumerate(nodes):
        if not node['nodeId'] or type(node['sequenceId']) is not int or node['sequenceId'] != offset+2*i:
            raise ProtocolError('Invalid node sequence')
        if node['actions']:
            raise ProtocolError('Node actions are unsupported')
        if set(node) - {'nodeId','sequenceId','released','actions','nodePosition','nodeDescription'}:
            raise ProtocolError('Unsupported node option')
        position = node.get('nodePosition', {})
        if set(position) != {'x','y','mapId'}:
            raise ProtocolError('nodePosition x/y/mapId required; orientation/deviations unsupported')
    for i, edge in enumerate(edges):
        if (not edge['edgeId'] or type(edge['sequenceId']) is not int or edge['sequenceId'] != offset+2*i+1 or edge['startNodeId'] != nodes[i]['nodeId']
                or edge['endNodeId'] != nodes[i+1]['nodeId']):
            raise ProtocolError('Invalid edge sequence/connectivity or unsupported Horizon')
        if edge['released'] != nodes[i+1]['released']:
            raise ProtocolError('Edge and destination release disagree')
        if edge['actions']:
            raise ProtocolError('Edge actions are unsupported')
        if set(edge) - {'edgeId','sequenceId','released','startNodeId','endNodeId','actions','edgeDescription'}:
            raise ProtocolError('Unsupported edge option (trajectory/speed/orientation/corridor)')


def validate_instant(action):
    if action['actionType'] not in {'cancelOrder','startPause','stopPause','factsheetRequest'}:
        raise ProtocolError('Unsupported instant action: '+action['actionType'])
    if action.get('actionParameters') or set(action)-{'actionId','actionType','blockingType','actionParameters','actionDescription'}:
        raise ProtocolError('Action parameters/extensions are unsupported')
    expected = 'HARD' if action['actionType'] == 'cancelOrder' else 'NONE'
    if action['blockingType'] != expected:
        raise ProtocolError('Unsupported blockingType for '+action['actionType'])
