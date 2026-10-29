"""Section 6.2/6.3 of the tagged 2.1.0 specification."""
TOPICS = {'order', 'instantActions', 'state', 'connection', 'factsheet', 'visualization'}
RETAINED = {'connection', 'factsheet'}


def topic(identity, kind):
    if kind not in TOPICS:
        raise ValueError('Unsupported VDA topic: '+kind)
    return f'{identity.interface_name}/v2/{identity.manufacturer}/{identity.serial_number}/{kind}'


def delivery(kind):
    if kind not in TOPICS:
        raise ValueError('Unknown topic')
    return (1 if kind == 'connection' else 0), kind in RETAINED
