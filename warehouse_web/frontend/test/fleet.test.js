import test from 'node:test';
import assert from 'node:assert/strict';
import { fleetDetails, fleetRobot } from '../fleet.js';

const snapshot = { fleet: { stale: false, robots: [{
  robot_id: 'r1', status: 'WAITING', message: 'Путь занят: r2', mqtt_connected: true,
  order_id: 'job', remaining_route: ['A', 'B'], confirmed_update_id: 2, order_update_id: 3,
  queue: [{ target_node_id: 'B', order_type: 'MOVE', status: 'EXECUTING' },
          { target_node_id: 'C', order_type: 'PARK', status: 'QUEUED' }],
}] } };

test('waiting, FIFO, destination and acknowledged version are visible', () => {
  const rows = new Map(fleetDetails(snapshot, 'r1'));
  assert.equal(rows.get('Диспетчер'), 'Ожидание');
  assert.equal(rows.get('Причина'), 'Путь занят: r2');
  assert.equal(rows.get('Цель задания'), 'B');
  assert.equal(rows.get('Оставшийся маршрут'), 'A → B');
  assert.match(rows.get('Очередь'), /1\. MOVE B.*2\. PARK C/);
  assert.equal(rows.get('Версия VDA'), '2 / 3');
});

test('disconnected dashboard does not present stale MQTT status as connected', () => {
  const rows = new Map(fleetDetails(snapshot, 'r1', false));
  assert.equal(rows.get('MQTT'), 'Нет актуальных данных');
  assert.match(rows.get('Диспетчер'), /устарело/);
  assert.equal(fleetRobot(snapshot, 'missing'), undefined);
  assert.deepEqual(fleetDetails(null, 'r1'), [['Диспетчер', 'Нет данных']]);
});
