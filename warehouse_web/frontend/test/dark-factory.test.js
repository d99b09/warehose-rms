import test from 'node:test';
import assert from 'node:assert/strict';
import { setupDarkFactory } from '../dashboard.js';

test('dispatches to free robots up to the limit even with an unavailable robot', async () => {
  const elements = Object.fromEntries(['dark-form', 'dark-state', 'dark-stop', 'dark-interval', 'dark-distance', 'dark-seed', 'dark-limit'].map(id => [id, { value: '' }]));
  elements['dark-interval'].value = '3600';
  elements['dark-distance'].value = '1';
  elements['dark-seed'].value = '42';
  elements['dark-limit'].value = '2';
  const previousDocument = globalThis.document, previousFetch = globalThis.fetch;
  globalThis.document = { getElementById: id => elements[id] };
  const requests = [];
  globalThis.fetch = async (_url, options) => {
    requests.push(JSON.parse(options.body));
    return { ok: true, json: async () => ({ accepted: true }) };
  };
  const fleet = ['a', 'b', 'offline'].map(robot_id => ({ robot_id, status: robot_id === 'offline' ? 'UNCERTAIN' : 'IDLE', order_id: '', queue: [] }));
  const robots = ['a', 'b'].map((robot_id, i) => ({ robot_id, x: i, y: 0, stale: false, agent_stale: false, odom_age: 0, agent_age: 0, error_code: '', agent_status: i ? 'WAITING' : 'PARKED' }));
  const map = { vertices: [{ id: 'A', x: 0, y: 0 }, { id: 'B', x: 5, y: 0 }], parking: [], edges: [{ source: 'A', target: 'B', bidirectional: true }] };
  try {
    setupDarkFactory(map, () => ({ connected: true, age: 0, snapshot: { bridge_ok: true, stale_after: 2, fleet: { stale: false, robots: fleet }, robots } }));
    elements['dark-form'].onsubmit({ preventDefault() {} });
    await new Promise(resolve => setImmediate(resolve));
    assert.deepEqual(new Set(requests.map(request => request.robot_id)), new Set(['a', 'b']));
    assert.equal(requests.length, 2);
  } finally {
    elements['dark-stop'].onclick();
    globalThis.document = previousDocument;
    globalThis.fetch = previousFetch;
  }
});
