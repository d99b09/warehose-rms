import test from 'node:test';
import assert from 'node:assert/strict';
import { setupDarkFactory } from '../dashboard.js';

test('stop prevents further generation while accepted tasks finish, including a pending response', async () => {
  const elements = Object.fromEntries(['dark-form', 'dark-state', 'dark-stop', 'dark-interval', 'dark-distance', 'dark-seed', 'dark-limit'].map(id => [id, { value: '' }]));
  for (const [id, value] of Object.entries({ 'dark-interval': '2', 'dark-distance': '1', 'dark-seed': '42', 'dark-limit': '1' })) elements[id].value = value;
  const previous = { document: globalThis.document, fetch: globalThis.fetch,
    setInterval: globalThis.setInterval, clearInterval: globalThis.clearInterval };
  const fleetRobot = { robot_id: 'r1', status: 'IDLE', order_id: '', queue: [], yielding: false };
  const current = { connected: true, age: 0, snapshot: { bridge_ok: true, stale_after: 2,
    fleet: { stale: false, robots: [fleetRobot] }, robots: [{ robot_id: 'r1', x: 0, y: 0,
      stale: false, agent_stale: false, error_code: '', agent_status: 'IDLE' }] } };
  const map = { vertices: [{ id: 'A', x: 0, y: 0 }, { id: 'B', x: 5, y: 0 }],
    parking: [], edges: [{ source: 'A', target: 'B', bidirectional: true }] };
  const requests = [];
  let finishRequest, timerCallback, cleared = false;
  globalThis.document = { getElementById: id => elements[id] };
  globalThis.fetch = async (url, options) => {
    requests.push({ url, body: JSON.parse(options.body) });
    await new Promise(resolve => { finishRequest = resolve; });
    return { ok: true, json: async () => ({ accepted: true, order_id: 'generated' }) };
  };
  globalThis.setInterval = callback => { timerCallback = callback; return 123; };
  globalThis.clearInterval = timer => { if (timer === 123) cleared = true; };
  try {
    const factory = setupDarkFactory(map, () => current);
    elements['dark-form'].onsubmit({ preventDefault() {} });
    assert.equal(requests.length, 1);
    elements['dark-stop'].onclick();
    assert.equal(cleared, true);
    assert.match(elements['dark-state'].textContent, /ожидание ответа/);

    // A request already accepted by the server remains valid after stopping generation.
    fleetRobot.order_id = 'generated';
    fleetRobot.queue = [{ order_id: 'generated' }];
    fleetRobot.status = 'MOVING';
    finishRequest();
    await new Promise(resolve => setImmediate(resolve));
    assert.match(elements['dark-state'].textContent, /Выключен · принято 1 · завершаются задания в парке: 1/);
    await timerCallback();
    assert.equal(requests.length, 1);
    assert.equal(fleetRobot.queue.length, 1);

    current.connected = false;
    factory.update();
    assert.match(elements['dark-state'].textContent, /ожидание актуальных данных/);
    current.connected = true;
    fleetRobot.queue = [];
    fleetRobot.order_id = '';
    fleetRobot.yielding = true;
    fleetRobot.status = 'YIELDING';
    factory.update();
    assert.match(elements['dark-state'].textContent, /завершение служебных перемещений/);

    fleetRobot.yielding = false;
    fleetRobot.status = 'IDLE';
    factory.update();
    assert.match(elements['dark-state'].textContent, /назначенных заданий нет/);
    await timerCallback();
    assert.equal(requests.length, 1);

    elements['dark-form'].onsubmit({ preventDefault() {} });
    assert.equal(requests.length, 2);
    finishRequest();
    await new Promise(resolve => setImmediate(resolve));
    elements['dark-stop'].onclick();
  } finally {
    Object.assign(globalThis, previous);
  }
});
