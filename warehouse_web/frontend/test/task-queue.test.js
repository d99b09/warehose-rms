import test from 'node:test';
import assert from 'node:assert/strict';
import { fleetTasks } from '../fleet.js';
import { setupTaskQueue } from '../task-queue.js';

function snapshot() {
  return { bridge_ok: true, stale_after: 2, fleet: { stale: false, robots: [
    { robot_id: 'r2', queue: [
      { number: 2, order_id: 'second', order_type: 'PARK', target_node_id: 'C', status: 'SUSPENDED' },
    ] },
    { robot_id: 'r1', queue: [
      { number: 1, order_id: 'first', order_type: 'MOVE', target_node_id: 'B', status: 'EXECUTING' },
      { number: 3, order_id: 'third', order_type: 'MOVE', target_node_id: 'D', status: 'QUEUED' },
    ] },
  ] } };
}

test('tasks follow assignment order across robots, including on initial load', () => {
  const state = snapshot();
  const before = structuredClone(state);
  const tasks = fleetTasks(state);
  assert.deepEqual(tasks.map(task => [task.number, task.robot_id, task.order_id]), [
    [1, 'r1', 'first'], [2, 'r2', 'second'], [3, 'r1', 'third'],
  ]);
  assert.deepEqual(tasks.map(task => task.status_label), ['Выполнение', 'Приостановлено', 'В очереди']);
  assert.deepEqual(state, before);
  assert.deepEqual(fleetTasks(null), []);
  assert.deepEqual(fleetTasks({ fleet: null }), []);
});

class Element {
  constructor() { this.dataset = {}; this.children = []; this.textContent = ''; this.hidden = false; }
  append(...children) { this.children.push(...children); }
  replaceChildren(...children) { this.children = children; }
}

test('panel removes completed tasks and distinguishes stale, missing and empty data', () => {
  const elements = Object.fromEntries(['task-list', 'task-count', 'task-state', 'task-empty'].map(id => [id, new Element()]));
  const card = new Element();
  elements['task-list'].closest = () => card;
  const previousDocument = globalThis.document;
  globalThis.document = {
    getElementById: id => elements[id], createElement: () => new Element(),
  };
  try {
    const panel = setupTaskQueue();
    const state = snapshot();
    const list = elements['task-list'];
    panel.update(null, false);
    assert.equal(elements['task-empty'].hidden, true);
    assert.match(elements['task-state'].textContent, /Ожидание/);

    panel.update(state, true);
    assert.equal(elements['task-count'].textContent, '3');
    assert.equal(elements['task-state'].hidden, true);
    assert.equal(list.children[0].children[0].children[0].textContent, '№ 1 · r1');
    assert.equal(list.children[1].children[1].textContent, 'PARK → C');
    const firstRow = list.children[0];
    panel.update(state, true);
    assert.equal(list.children[0], firstRow);

    // Completion removes the first task from the authoritative dispatcher queue.
    state.fleet.robots[1].queue.shift();
    panel.update(state, true);
    assert.equal(elements['task-count'].textContent, '2');
    assert.deepEqual(list.children.map(row => row.children[2].textContent), ['second', 'third']);
    state.fleet.robots[0].queue[0].status = 'EXECUTING';
    panel.update(state, true);
    assert.equal(list.children[0].children[0].children[1].textContent, 'Выполнение');

    for (const [connected, bridgeOk, fleetStale, age] of [
      [false, true, false, 0], [true, false, false, 0],
      [true, true, true, 0], [true, true, false, 2],
    ]) {
      state.bridge_ok = bridgeOk;
      state.fleet.stale = fleetStale;
      panel.update(state, connected, age);
      assert.equal(card.dataset.stale, 'true');
      assert.equal(elements['task-state'].hidden, false);
      assert.equal(list.children.length, 2);
    }

    state.fleet.robots.forEach(robot => { robot.queue = []; });
    panel.update(state, true);
    assert.equal(card.dataset.stale, 'false');
    assert.equal(elements['task-count'].textContent, '0');
    assert.equal(elements['task-empty'].hidden, false);
    assert.equal(list.children.length, 0);
  } finally {
    globalThis.document = previousDocument;
  }
});
