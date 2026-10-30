export function setupSpawn(map) {
  const dialog = document.getElementById('spawn-dialog');
  const parking = document.getElementById('spawn-parking');
  const result = document.getElementById('spawn-result');
  const confirm = document.getElementById('spawn-confirm');
  for (const node of map.parking) parking.add(new Option(node.id, node.id));
  const add = document.getElementById('add-robot');
  add.disabled = !map.parking.length;
  add.onclick = () => { result.textContent = ''; dialog.showModal(); };
  document.getElementById('spawn-close').onclick = () => dialog.close();
  document.getElementById('spawn-form').onsubmit = async event => {
    event.preventDefault();
    confirm.disabled = true;
    result.textContent = '';
    try {
      const response = await fetch('/api/robots', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          operation_id: `${Date.now()}-${Array.from(crypto.getRandomValues(new Uint32Array(4))).join('-')}`,
          parking_node_id: parking.value,
        }),
      });
      const data = await response.json();
      if (!response.ok) throw new Error(JSON.stringify(data.message ?? data.detail ?? `HTTP ${response.status}`));
      result.textContent = data.message;
    } catch (error) {
      result.textContent = String(error);
    } finally {
      confirm.disabled = false;
    }
  };
}


export function setupManual(selected) {
  const state = document.getElementById('manual-state');
  const start = document.getElementById('manual-start');
  const stop = document.getElementById('manual-stop');
  const zero = document.getElementById('manual-zero');
  const buttons = document.querySelectorAll('[data-motion]');
  const session = `${Date.now()}-${Array.from(crypto.getRandomValues(new Uint32Array(4))).join('-')}`;
  let robotId = '', held = null, timer, sequence = 0, sending = false, pending = false;

  async function post(id, action, body = {}) {
    const response = await fetch(`/api/robots/${encodeURIComponent(id)}/manual/${action}`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body),
    });
    const data = await response.json();
    if (!response.ok) throw new Error(data.detail ?? `HTTP ${response.status}`);
    state.textContent = data.message;
    return data;
  }

  function sendMotion(command) {
    if (!robotId) return Promise.resolve();
    return post(robotId, 'cmd', {
      session_id: session, sequence: ++sequence, linear: command[0], angular: command[1],
    });
  }

  function stopMotion(force = false) {
    const wasHeld = held !== null;
    held = null;
    clearInterval(timer);
    if ((wasHeld || force) && robotId) {
      void sendMotion([0, 0]).catch(error => { state.textContent = String(error); });
    }
  }

  function update() {
    if (robotId && selected() !== robotId) {
      stopMotion();
      robotId = '';
      state.textContent = 'Выберите робота и включите MANUAL';
    }
    start.disabled = !selected() || !!robotId || pending;
    stop.disabled = !robotId || pending;
    zero.disabled = !robotId;
    for (const button of buttons) button.disabled = !robotId;
  }

  function startMotion(command) {
    stopMotion();
    if (!robotId || selected() !== robotId) return;
    held = command;
    const pulse = async () => {
      if (!held || sending) return;
      sending = true;
      try { await sendMotion(held); }
      catch (error) { stopMotion(false); state.textContent = String(error); }
      finally { sending = false; }
    };
    void pulse();
    timer = setInterval(() => void pulse(), 100);
  }

  start.onclick = async () => {
    const id = selected();
    if (!id) return;
    pending = true;
    update();
    try {
      await post(id, 'start');
      if (selected() === id) robotId = id;
    } catch (error) { state.textContent = String(error); }
    finally { pending = false; update(); }
  };
  stop.onclick = async () => {
    if (!robotId) return;
    const id = robotId;
    stopMotion();
    robotId = '';
    update();
    try { await post(id, 'stop'); }
    catch (error) { state.textContent = String(error); }
  };
  zero.onclick = () => stopMotion(true);
  for (const button of buttons) {
    const command = button.dataset.motion.split(',').map(Number);
    button.onpointerdown = event => {
      if (event.button !== 0) return;
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      startMotion(command);
    };
    button.onpointerup = () => stopMotion();
    button.onpointercancel = () => stopMotion();
    button.onlostpointercapture = () => stopMotion();
  }
  const keys = {
    ArrowUp: [0.2, 0], KeyW: [0.2, 0], ArrowDown: [-0.2, 0], KeyS: [-0.2, 0],
    ArrowLeft: [0, 0.6], KeyA: [0, 0.6], ArrowRight: [0, -0.6], KeyD: [0, -0.6],
  };
  window.addEventListener('keydown', event => {
    if (event.target?.closest('input,select,textarea') || event.ctrlKey || event.metaKey || event.altKey) return;
    if (event.code === 'Space' || event.code === 'Escape') { event.preventDefault(); stopMotion(true); }
    if (keys[event.code] && !event.repeat && robotId) {
      event.preventDefault();
      startMotion(keys[event.code]);
    }
  });
  window.addEventListener('keyup', event => { if (keys[event.code]) stopMotion(); });
  window.addEventListener('blur', () => stopMotion());
  window.addEventListener('pagehide', () => stopMotion());
  document.addEventListener('visibilitychange', () => { if (document.hidden) stopMotion(); });
  update();
  return { update };
}

export function setupOrder(map, selectRobot) {
  const form = document.getElementById('order-form');
  const robots = document.getElementById('order-robot');
  const target = document.getElementById('order-target');
  const type = document.getElementById('order-type');
  const submit = document.getElementById('order-submit');
  const result = document.getElementById('order-result');
  const parking = new Set(map.parking.map(node => node.id));
  for (const node of map.vertices) {
    target.add(new Option(`${node.id}${parking.has(node.id) ? ' · парковка' : ''}`, node.id));
  }
  robots.onchange = () => selectRobot(robots.value);
  form.onsubmit = async event => {
    event.preventDefault();
    submit.disabled = true;
    try {
      const response = await fetch('/api/orders', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ robot_id: robots.value, target_node_id: target.value,
          order_type: type.value }),
      });
      const data = await response.json();
      if (!response.ok) throw new Error(data.message ?? JSON.stringify(data.detail ?? `HTTP ${response.status}`));
      result.textContent = `${data.message}${data.route.length ? ': ' + data.route.join(' → ') : ''} · ${data.order_id}`;
    } catch (error) {
      result.textContent = String(error);
    } finally {
      submit.disabled = !robots.value || !target.value;
    }
  };
  return {
    update(items, selected) {
      const ids = items.map(robot => robot.robot_id);
      if (Array.from(robots.options, option => option.value).join('|') !== ids.join('|')) {
        robots.replaceChildren(...ids.map(id => new Option(id, id)));
      }
      robots.value = selected ?? '';
      submit.disabled = !robots.value || !target.value;
    },
  };
}

export function setupDarkFactory(map, state) {
  const form = document.getElementById('dark-form');
  const status = document.getElementById('dark-state');
  const stop = document.getElementById('dark-stop');
  const interval = document.getElementById('dark-interval');
  const minimum = document.getElementById('dark-distance');
  const seed = document.getElementById('dark-seed');
  const limit = document.getElementById('dark-limit');
  const nodes = new Map(map.vertices.map(node => [node.id, node]));
  const parking = new Set(map.parking.map(node => node.id));
  const edges = new Map(map.vertices.map(node => [node.id, []]));
  for (const edge of map.edges) {
    const a = nodes.get(edge.source), b = nodes.get(edge.target);
    const length = Math.hypot(a.x - b.x, a.y - b.y);
    edges.get(edge.source).push([edge.target, length]);
    if (edge.bidirectional) edges.get(edge.target).push([edge.source, length]);
  }
  let timer = null, generation = 0, busy = false, running = false, random = 0, accepted = 0;
  const sent = new Map();

  function refreshSent(fleet) {
    for (const [id, sentAt] of sent) {
      const item = fleet.find(robot => robot.robot_id === id);
      if (!item || item.queue.length || item.order_id || item.status !== 'IDLE' || Date.now() - sentAt > 10000) sent.delete(id);
    }
  }

  function update() {
    if (running) return;
    const prefix = `Выключен · принято ${accepted}`;
    if (busy) {
      status.textContent = `${prefix} · ожидание ответа на отправленный приказ`;
      return;
    }
    const { snapshot, connected, age } = state();
    if (!connected || !snapshot?.bridge_ok || !snapshot.fleet || snapshot.fleet.stale || age >= snapshot.stale_after) {
      status.textContent = `${prefix} · ожидание актуальных данных диспетчера`;
      return;
    }
    const fleet = snapshot.fleet.robots;
    refreshSent(fleet);
    const remaining = fleet.reduce((total, robot) => total + robot.queue.length, 0);
    status.textContent = remaining ? `${prefix} · завершаются задания в парке: ${remaining}` :
      fleet.some(robot => robot.order_id || robot.yielding || ['MOVING', 'YIELDING', 'EXECUTING'].includes(robot.status)) ?
        `${prefix} · завершение служебных перемещений` : `${prefix} · назначенных заданий нет`;
  }

  function candidates(start, distance) {
    const distances = new Map([[start, 0]]), visited = new Set();
    while (true) {
      let current = null, shortest = Infinity;
      for (const [id, value] of distances) {
        if (!visited.has(id) && value < shortest) { current = id; shortest = value; }
      }
      if (current === null) break;
      visited.add(current);
      for (const [next, length] of edges.get(current) ?? []) {
        if (shortest + length < (distances.get(next) ?? Infinity)) distances.set(next, shortest + length);
      }
    }
    return [...distances].filter(([id, value]) => id !== start && !parking.has(id) && value >= distance).map(([id]) => id).sort();
  }

  async function tick(run) {
    if (busy || run !== generation) return;
    const { snapshot, connected, age } = state();
    if (!connected || !snapshot?.bridge_ok || !snapshot.fleet || snapshot.fleet.stale || age >= snapshot.stale_after) {
      status.textContent = 'Ожидание актуальных данных диспетчера';
      return;
    }
    const fleet = snapshot.fleet.robots;
    refreshSent(fleet);
    const active = fleet.filter(robot => robot.queue.length || robot.order_id || robot.status !== 'IDLE').length + sent.size;
    if (active >= Number(limit.value)) { status.textContent = `Включён · принято ${accepted} · ожидание свободного робота`; return; }
    const available = snapshot.robots.filter(item => {
      const info = fleet.find(entry => entry.robot_id === item.robot_id);
      return info && info.status === 'IDLE' && !info.order_id && !info.queue.length &&
        !sent.has(item.robot_id) && !item.stale && !item.agent_stale && !item.error_code &&
        ['IDLE', 'PARKED'].includes(item.agent_status) && Number.isFinite(item.x) && Number.isFinite(item.y);
    });
    if (!available.length) { status.textContent = `Включён · принято ${accepted} · ожидание свободного робота`; return; }
    random = (Math.imul(random, 1664525) + 1013904223) >>> 0;
    const robot = available[Math.floor(random / 4294967296 * available.length)];
    const start = [...nodes.values()].reduce((best, node) =>
      Math.hypot(node.x - robot.x, node.y - robot.y) < Math.hypot(best.x - robot.x, best.y - robot.y) ? node : best);
    const options = candidates(start.id, Number(minimum.value));
    if (!options.length) { status.textContent = 'Нет доступной цели с заданной длиной пути'; return; }
    random = (Math.imul(random, 1664525) + 1013904223) >>> 0;
    const target = options[Math.floor(random / 4294967296 * options.length)];
    busy = true;
    try {
      const response = await fetch('/api/orders', {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ robot_id: robot.robot_id, target_node_id: target, order_type: 'MOVE' }),
      });
      const data = await response.json();
      if (!response.ok || !data.accepted) throw new Error(data.message ?? `HTTP ${response.status}`);
      sent.set(robot.robot_id, Date.now());
      accepted++;
      if (run === generation) status.textContent = `Включён · принято ${accepted} · ${robot.robot_id} → ${target}`;
    } catch (error) {
      if (run === generation) status.textContent = `Ошибка: ${error.message}`;
    } finally {
      busy = false;
      if (!running) update();
    }
  }

  form.onsubmit = event => {
    event.preventDefault();
    clearInterval(timer);
    generation++;
    running = true;
    random = Number(seed.value) >>> 0;
    accepted = 0;
    status.textContent = 'Включён · ожидание робота';
    const run = generation;
    void tick(run);
    timer = setInterval(() => void tick(run), Number(interval.value) * 1000);
  };
  stop.onclick = () => {
    generation++;
    running = false;
    clearInterval(timer);
    timer = null;
    update();
  };
  return { update };
}
