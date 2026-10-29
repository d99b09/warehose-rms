from pathlib import Path
import math
import yaml
from protocols.vda5050.models import Identity


def load_config(path):
    data = yaml.safe_load(Path(path).read_text())
    vda, mqtt = data['vda5050'], data['mqtt']
    if vda['version'] != '2.1.0':
        raise ValueError('Only VDA 5050 2.1.0 supported')
    for key in ('port','keepalive'):
        if type(mqtt[key]) is not int or mqtt[key] <= 0:
            raise ValueError('Invalid MQTT '+key)
    if bool(mqtt.get('username_env')) != bool(mqtt.get('password_env')):
        raise ValueError('Both MQTT credential environment variable names required')
    if mqtt['port'] > 65535 or mqtt['keepalive'] > 65535:
        raise ValueError('Invalid MQTT port')
    if not mqtt.get('username_env') and not (mqtt.get('local_test') and mqtt['host'] in {'127.0.0.1','localhost','::1'}):
        raise ValueError('MQTT authentication required outside explicit loopback test mode')
    serials, ids = set(), set()
    for robot in data['robots']:
        Identity(vda['manufacturer'],robot['serial_number'],vda['interface_name'])
        if robot['serial_number'] in serials or robot['robot_id'] in ids:
            raise ValueError('Duplicate VDA identity')
        serials.add(robot['serial_number']); ids.add(robot['robot_id'])
    sim = data['simulation']
    for key in ('state_interval','state_timeout','command_max_age','speed_max','width','length','height','acceleration_max','deceleration_max'):
        if isinstance(sim[key],bool) or not math.isfinite(sim[key]) or sim[key] <= 0:
            raise ValueError('Invalid simulation '+key)
    if (isinstance(sim['battery_charge'], bool) or not math.isfinite(sim['battery_charge'])
            or not 0 <= sim['battery_charge'] <= 100 or not sim['map_id']):
        raise ValueError('Invalid simulation battery/map')
    data['state_directory'] = str(Path(data['state_directory']).expanduser().resolve())
    return data


def identity(config, robot_id):
    r = next((r for r in config['robots'] if r['robot_id'] == robot_id), dict(serial_number=robot_id))
    v = config['vda5050']
    return Identity(v['manufacturer'],r['serial_number'],v['interface_name'])
