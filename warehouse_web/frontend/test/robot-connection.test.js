import test from 'node:test';
import assert from 'node:assert/strict';
import { setupRobotConnection } from '../robot-connection.js';

test('connection button toggles selected robot, handles delayed state and stale data', async () => {
  const previous = { document: globalThis.document, fetch: globalThis.fetch };
  const button = {}, result = {};
  globalThis.document = { getElementById: id => id === 'robot-connection' ? button : result };
  let finish;
  const calls = [];
  globalThis.fetch = (url, request) => {
    calls.push([url, JSON.parse(request.body)]);
    return new Promise(resolve => { finish = resolve; });
  };
  try {
    const control = setupRobotConnection();
    const snapshot = { bridge_ok: true, stale_after: 2, robots: [{ robot_id: 'r/1', agent_status: 'IDLE' }],
      fleet: { stale: false, robots: [{ robot_id: 'r/1', mqtt_connected: true }] } };
    control.update(null, null, false);
    assert.equal(button.disabled, true);
    control.update(snapshot, 'r/1', true);
    assert.equal(button.textContent, 'Выключить робота');
    const disabling = button.onclick();
    assert.equal(button.disabled, true);
    await button.onclick(); // A second click cannot start another request.
    assert.equal(calls.length, 1);
    finish({ ok: true, json: async () => ({ message: 'Связь выключена' }) });
    await disabling;
    assert.deepEqual(calls[0], ['/api/robots/r%2F1/connection', { enabled: false }]);
    assert.equal(button.textContent, 'Включить робота');
    snapshot.fleet.robots[0].mqtt_connected = false;
    control.update(snapshot, 'r/1', true);
    const enabling = button.onclick();
    finish({ ok: true, json: async () => ({ message: 'Восстановление связи' }) });
    await enabling;
    assert.deepEqual(calls[1][1], { enabled: true });
    snapshot.fleet.robots[0].mqtt_connected = true;
    control.update(snapshot, 'r/1', true);
    assert.equal(button.textContent, 'Выключить робота');
    control.update(snapshot, 'r/1', false);
    assert.equal(button.disabled, true);
    control.update(snapshot, 'r/1', true, 2);
    assert.equal(button.disabled, true);
    snapshot.robots[0].agent_status = 'MANUAL';
    control.update(snapshot, 'r/1', true);
    assert.equal(button.disabled, true);
    snapshot.robots[0].agent_status = 'IDLE';
    control.update(snapshot, 'r/1', true);
    const failed = button.onclick();
    finish({ ok: false, json: async () => ({ detail: 'Адаптер недоступен' }) });
    await failed;
    assert.match(result.textContent, /Адаптер недоступен/);
    assert.equal(button.textContent, 'Выключить робота');
  } finally {
    Object.assign(globalThis, previous);
  }
});
