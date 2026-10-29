"""Transport-neutral identities and JSON wire values; no ROS imports."""
from dataclasses import dataclass
import json
import re


@dataclass(frozen=True)
class Identity:
    manufacturer: str = 'WarehouseSim'
    serial_number: str = 'robot_01'
    interface_name: str = 'uagv'

    def __post_init__(self):
        for part in (self.manufacturer, self.serial_number, self.interface_name):
            if not re.fullmatch(r'[A-Za-z0-9_.:-]+', part):
                raise ValueError('MQTT identity must be a single non-wildcard topic level')


def dumps(message):
    return json.dumps(message, allow_nan=False, separators=(',', ':'))


def loads(payload):
    def invalid(value):
        raise ValueError('Non-finite JSON number: '+value)
    message = json.loads(payload, parse_constant=invalid)
    if not isinstance(message, dict):
        raise ValueError('VDA message must be an object')
    return message
