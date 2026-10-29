"""Paho owns network I/O; a bounded worker validates all incoming/outgoing JSON.

No Paho thread touches ROS or domain state. ROS drains validated events.
"""
from datetime import datetime, timezone
from copy import deepcopy
import os
import queue
import threading
import time
from uuid import uuid4
import paho.mqtt.client as mqtt
from protocols.vda5050.models import dumps, loads
from protocols.vda5050.topics import topic, delivery
from protocols.vda5050.validator import validate


class MqttClient:
    def __init__(self, config, identities, role, will=None):
        self.config, self.identities = config, identities
        self.events = queue.Queue(maxsize=512)
        self.work = queue.Queue(maxsize=512)
        self.connected = False
        self.generation = 0
        self.error = ''
        self.subscriptions = []
        self.stopping = threading.Event()
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
            client_id=f'warehouse-{role}-{uuid4().hex[:16]}', clean_session=True,
            protocol=mqtt.MQTTv311)
        self.client.max_queued_messages_set(512)
        self.client.reconnect_delay_set(1, 5)
        if config.get('username_env'):
            username, password = os.environ[config['username_env']], os.environ[config['password_env']]
            self.client.username_pw_set(username,password)
        if config.get('tls_ca'):
            self.client.tls_set(ca_certs=config['tls_ca'])
        if will:
            ident, message = will
            validate('connection',message,ident)
            self.client.will_set(topic(ident,'connection'),dumps(message),qos=1,retain=True)
        self.client.on_connect = self._connect
        self.client.on_disconnect = self._disconnect
        self.client.on_message = self._message
        self.worker = threading.Thread(target=self._worker, name='vda-validation', daemon=False)

    def subscribe(self, rid, kind):
        if (rid, kind) not in self.subscriptions:
            self.subscriptions.append((rid,kind))
            if self.connected:
                self.client.subscribe(topic(self.identities[rid], kind), qos=delivery(kind)[0])

    def start(self):
        self.client.connect_async(self.config['host'], self.config['port'], self.config['keepalive'])
        self.worker.start()
        self.client.loop_start()

    def _event(self, event):
        try:
            self.events.put_nowait(event)
        except queue.Full:
            self.error = 'MQTT event queue overflow; control must recover'
            self.connected = False
            self.client.disconnect()

    def _connect(self, client, userdata, flags, reason, properties):
        self.generation += 1
        self.connected = not reason.is_failure
        if not self.connected:
            self.error = str(reason)
            return
        for rid,kind in self.subscriptions:
            client.subscribe(topic(self.identities[rid],kind),qos=delivery(kind)[0])
        self._event(('connected',None,None))

    def _disconnect(self, client, userdata, flags, reason, properties):
        self.generation += 1
        self.connected = False
        self._event(('disconnected',None,None))

    def _message(self, client, userdata, message):
        if len(message.payload) > 1048576:
            self.error = 'MQTT payload exceeds profile limit'
            return
        for rid,kind in self.subscriptions:
            if message.topic == topic(self.identities[rid],kind):
                try:
                    self.work.put_nowait(('in',rid,kind,message.payload,message.retain,self.generation))
                except queue.Full:
                    self.error = 'MQTT validation queue full'
                break

    def publish(self, rid, kind, message):
        if not self.connected:
            raise ConnectionError('MQTT disconnected')
        self.work.put_nowait(('out',rid,kind,deepcopy(message),False,self.generation))

    def _worker(self):
        while not self.stopping.is_set() or not self.work.empty():
            try:
                direction,rid,kind,value,retained,generation = self.work.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                if generation != self.generation or not self.connected:
                    raise ConnectionError('Discarded message from a previous MQTT connection')
                message = loads(value) if direction == 'in' else value
                validate(kind,message,self.identities[rid])
                if kind in {'order', 'instantActions'}:
                    if direction == 'in' and retained:
                        raise ValueError('Retained motion commands are forbidden')
                    age = (datetime.now(timezone.utc)-datetime.fromisoformat(message['timestamp'].replace('Z','+00:00'))).total_seconds()
                    if not -2 <= age <= self.config.get('command_max_age',5.):
                        raise ValueError('Expired/future command timestamp' if direction == 'in'
                                         else 'Expired outgoing command timestamp')
                if direction == 'in':
                    self._event((kind,rid,message))
                else:
                    if generation != self.generation or not self.connected:
                        raise ConnectionError('Discarded publication after MQTT connection changed')
                    qos,retain = delivery(kind)
                    result = self.client.publish(topic(self.identities[rid],kind),dumps(message),qos=qos,retain=retain)
                    if result.rc != mqtt.MQTT_ERR_SUCCESS:
                        raise ConnectionError('MQTT publication failed')
            except Exception as exc:
                self.error = str(exc)
                self._event(('validation_error',rid,dict(topic=kind,error=str(exc))))
            finally:
                self.work.task_done()

    def drain(self):
        for _ in range(512):
            try:
                yield self.events.get_nowait()
            except queue.Empty:
                break

    def close(self, offline=None):
        # Queue OFFLINE after pending state; keep the network loop alive until PUBACK.
        deadline = time.monotonic()+2
        while self.work.unfinished_tasks and time.monotonic() < deadline:
            time.sleep(.01)
        if offline and self.connected:
            rid,message = offline
            validate('connection',message,self.identities[rid])
            result = self.client.publish(topic(self.identities[rid],'connection'),dumps(message),qos=1,retain=True)
            try:
                result.wait_for_publish(timeout=1)
            except RuntimeError:
                pass
        self.stopping.set()
        if self.worker.is_alive():
            self.worker.join(timeout=3)
        self.client.disconnect()
        self.client.loop_stop()
