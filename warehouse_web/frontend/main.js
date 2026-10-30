import { fleetDetails, fleetRobot } from './fleet.js';
import { setupDarkFactory, setupManual, setupOrder, setupSpawn } from './dashboard.js';
import { StateConnection } from './socket.js';
import { RobotMotion } from './motion.js';
import { setupTaskQueue } from './task-queue.js';
import { setupRobotConnection } from './robot-connection.js';

const canvas = document.getElementById('map');
const ctx = canvas.getContext('2d');
if (!ctx) throw new Error('Canvas 2D недоступен');
const response = await fetch('/api/map', { cache: 'no-store' });
if (!response.ok) throw new Error(`HTTP ${response.status}`);
const map = await response.json();
setupSpawn(map);
document.getElementById('map-title').textContent = `${map.building_name} / ${map.level_name}`;
const nodes = new Map(map.vertices.map(node => [node.id, node]));
const parking = new Set(map.parking.map(node => node.id));
let snapshot = null, connected = false, selected = null, receivedAt = 0;
const robotButtons = new Map();
const motion = new RobotMotion();
let animationFrame = null;
const manual = setupManual(() => selected);
const order = setupOrder(map, id => { selected = id; updatePanel(); render(); });
const taskQueue = setupTaskQueue();
const robotConnection = setupRobotConnection();
const darkFactory = setupDarkFactory(map, () => ({ snapshot, connected,
  age: (performance.now() - receivedAt) / 1000 }));

function stale(robot) {
  return !connected || !snapshot?.bridge_ok || robot.stale ||
    robot.odom_age === null || robot.odom_age + (performance.now() - receivedAt) / 1000 >= snapshot.stale_after;
}

function agentStale(robot) {
  return !connected || !snapshot?.bridge_ok || robot.agent_stale ||
    robot.agent_age === null || robot.agent_age + (performance.now() - receivedAt) / 1000 >= snapshot.stale_after;
}

function movement(robot) {
  if (stale(robot)) return 'Нет актуальных данных';
  if (Math.abs(robot.linear_velocity ?? 0) > 0.01) {
    return robot.linear_velocity > 0 ? 'Движется вперёд' : 'Движется назад';
  }
  if (Math.abs(robot.angular_velocity ?? 0) > 0.01) return 'Поворачивает';
  return 'Стоит';
}

function animate() {
  if (animationFrame !== null) return;
  animationFrame = requestAnimationFrame(() => {
    animationFrame = null;
    render();
    if (motion.isAnimating(performance.now())) animate();
  });
}

function render() {
  const now = performance.now();
  const { width, height } = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = Math.round(width * dpr);
  canvas.height = Math.round(height * dpr);
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  const b = map.bounds;
  const scale = Math.min(Math.max(1, width - 88) / (b.max_x - b.min_x),
    Math.max(1, height - 88) / (b.max_y - b.min_y));
  function toScreen(point) {
    return { x: width / 2 + (point.x - (b.min_x + b.max_x) / 2) * scale,
      y: height / 2 - (point.y - (b.min_y + b.max_y) / 2) * scale };
  }
  function line(a, b, color, thickness) {
    ctx.strokeStyle = color;
    ctx.lineWidth = thickness;
    ctx.beginPath();
    ctx.moveTo(a.x, a.y);
    ctx.lineTo(b.x, b.y);
    ctx.stroke();
  }
  for (const edge of map.edges) {
    const a = toScreen(nodes.get(edge.source)), b = toScreen(nodes.get(edge.target));
    line(a, b, '#baccc0', 1);
    if (!edge.bidirectional) {
      const x = a.x + (b.x - a.x) * 0.62, y = a.y + (b.y - a.y) * 0.62;
      const angle = Math.atan2(b.y - a.y, b.x - a.x);
      for (const offset of [-0.5, 0.5]) {
        line({ x, y }, { x: x - 5 * Math.cos(angle + offset),
          y: y - 5 * Math.sin(angle + offset) }, '#91a99a', 1);
      }
    }
  }
  for (const wall of map.walls) {
    line(toScreen(wall.start), toScreen(wall.end), '#34493d', 3);
  }
  ctx.font = '11px system-ui';
  const blockedNodes = new Set(connected && !snapshot?.fleet?.stale ? snapshot?.fleet?.blocked_nodes ?? [] : []);
  for (const node of map.vertices) {
    const p = toScreen(node);
    if (blockedNodes.has(node.id)) {
      ctx.beginPath();
      ctx.arc(p.x, p.y, 11, 0, Math.PI * 2);
      ctx.strokeStyle = '#c64c4c';
      ctx.lineWidth = 3;
      ctx.stroke();
    }
    ctx.fillStyle = parking.has(node.id) ? '#378158' : '#91a99a';
    if (parking.has(node.id)) {
      ctx.fillRect(p.x - 5, p.y - 5, 10, 10);
    } else {
      ctx.beginPath();
      ctx.arc(p.x, p.y, 2.5, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.fillStyle = '#4e6456';
    ctx.fillText(node.id, p.x + 8, p.y - 8);
  }
  const fleet = fleetRobot(snapshot, selected);
  if (fleet && !snapshot.fleet.stale && connected) {
    const route = fleet.remaining_route.map(id => nodes.get(id));
    ctx.save();
    ctx.setLineDash([5, 4]);
    for (let i = 1; i < route.length; i++) {
      if (route[i - 1] && route[i]) line(toScreen(route[i - 1]), toScreen(route[i]), fleet.yielding ? '#a75dbe' : '#416cbd', 3);
    }
    ctx.restore();
  }
  const active = snapshot?.robots.find(robot => robot.robot_id === selected);
  if (active && !agentStale(active)) {
    const current = nodes.get(active.current_node_id);
    const next = nodes.get(fleet?.remaining_route[1] ?? active.target_node_id);
    if (next && !stale(active) && [active.x, active.y, active.theta].every(Number.isFinite)) {
      const a = toScreen(motion.pose(active, now)), b = toScreen(next);
      ctx.save();
      ctx.setLineDash([7, 5]);
      line(a, b, '#b46c13', 2.5);
      ctx.restore();
      if (Math.hypot(b.x - a.x, b.y - a.y) > 18) {
        const angle = Math.atan2(b.y - a.y, b.x - a.x);
        const tip = { x: b.x - 13 * Math.cos(angle), y: b.y - 13 * Math.sin(angle) };
        for (const offset of [-0.5, 0.5]) {
          line(tip, { x: tip.x - 9 * Math.cos(angle + offset),
            y: tip.y - 9 * Math.sin(angle + offset) }, '#b46c13', 2.5);
        }
      }
    }
    for (const [node, color, radius] of [[current, '#416cbd', 9], [next, '#b46c13', 12]]) {
      if (!node) continue;
      const p = toScreen(node);
      ctx.beginPath();
      ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
      ctx.strokeStyle = color;
      ctx.lineWidth = 3;
      ctx.stroke();
    }
  }
  for (const robot of snapshot?.robots ?? []) {
    if (robot.x === null || robot.y === null || robot.theta === null) continue;
    const pose = motion.pose(robot, now);
    const p = toScreen(pose);
    const length = robot.length * scale, bodyWidth = robot.width * scale;
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-pose.theta);
    ctx.fillStyle = stale(robot) || fleetRobot(snapshot, robot.robot_id)?.mqtt_connected === false ? '#969da3' :
      !robot.status_stale && ['COLLISION', 'OUT_OF_BOUNDS'].includes(robot.status) ? '#c64c4c' : '#399a69';
    ctx.fillRect(-length / 2, -bodyWidth / 2, length, bodyWidth);
    if (selected === robot.robot_id) {
      ctx.strokeStyle = '#4f80ba';
      ctx.lineWidth = 2;
      ctx.strokeRect(-length / 2 - 3, -bodyWidth / 2 - 3, length + 6, bodyWidth + 6);
    }
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.moveTo(length * 0.35, 0);
    ctx.lineTo(-length * 0.08, -bodyWidth * 0.25);
    ctx.lineTo(-length * 0.08, bodyWidth * 0.25);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
    ctx.fillStyle = stale(robot) ? '#7d858b' : '#416c9e';
    ctx.fillText(robot.robot_id, p.x + 10, p.y - 14);
  }
}

function updatePanel() {
  const robots = snapshot?.robots ?? [];
  document.getElementById('robot-count').textContent = String(robots.length);
  document.getElementById('empty-state').hidden = robots.length > 0;
  const ids = new Set(robots.map(robot => robot.robot_id));
  for (const [id, button] of robotButtons) {
    if (!ids.has(id)) { button.remove(); robotButtons.delete(id); }
  }
  if (!ids.has(selected)) selected = robots[0]?.robot_id ?? null;
  manual.update();
  robotConnection.update(snapshot, selected, connected, (performance.now() - receivedAt) / 1000);
  order.update(robots, selected);
  taskQueue.update(snapshot, connected, (performance.now() - receivedAt) / 1000);
  darkFactory.update();
  const list = document.getElementById('robot-list');
  for (const robot of robots) {
    let button = robotButtons.get(robot.robot_id);
    if (!button) {
      button = document.createElement('button');
      button.type = 'button';
      button.className = 'robot-row';
      button.onclick = () => { selected = robot.robot_id; updatePanel(); render(); };
      robotButtons.set(robot.robot_id, button);
      list.append(button);
    }
    button.textContent = `${robot.robot_id}${stale(robot) ? ' · устарело' : ''}`;
    button.setAttribute('aria-pressed', String(selected === robot.robot_id));
  }
  const robot = robots.find(item => item.robot_id === selected);
  const fleet = fleetRobot(snapshot, selected);
  const navigation = document.getElementById('navigation-state');
  navigation.dataset.stale = String(!robot || stale(robot) || agentStale(robot));
  document.getElementById('motion-state').textContent = robot ? movement(robot) : 'Выберите робота';
  const nodeValue = id => `${id || '—'}${robot && agentStale(robot) ? ' · устарело' : ''}`;
  document.getElementById('current-node').textContent = nodeValue(robot?.current_node_id);
  document.getElementById('next-node').textContent = nodeValue(fleet?.remaining_route[1] ?? robot?.target_node_id);
  document.getElementById('selected-title').textContent = robot ? robot.robot_id : 'Выберите робота';
  const details = document.getElementById('details');
  details.replaceChildren();
  if (!robot) return;
  const value = (number, unit) => number === null ? '—' : `${number.toFixed(2)} ${unit}`;
  const age = robot.odom_age === null ? null :
    robot.odom_age + Math.max(0, (performance.now() - receivedAt) / 1000);
  const statusAge = robot.status_age === null ? null :
    robot.status_age + Math.max(0, (performance.now() - receivedAt) / 1000);
  const statusStale = !connected || !snapshot?.bridge_ok || robot.status_stale ||
    statusAge === null || statusAge >= snapshot.stale_after;
  const agentAge = robot.agent_age === null ? null :
    robot.agent_age + Math.max(0, (performance.now() - receivedAt) / 1000);
  const agentIsStale = agentStale(robot);
  for (const [name, text] of [
    ['X', value(robot.x, 'м')], ['Y', value(robot.y, 'м')],
    ['θ', value(robot.theta, 'рад')],
    ['Линейная скорость', value(robot.linear_velocity, 'м/с')],
    ['Угловая скорость', value(robot.angular_velocity, 'рад/с')],
    ['Статус', `${robot.status}${statusStale ? ' · устарел' : ''}`],
    ['Движение', movement(robot)],
    ['Состояние агента', `${robot.agent_status ?? 'Нет данных'}${agentIsStale ? ' · устарело' : ''}`],
    ['Ошибка агента', robot.error_code || '—'],
    ...fleetDetails(snapshot, selected, connected),
    ['Текущий узел', nodeValue(robot.current_node_id)],
    ['Следующий узел', nodeValue(fleet?.remaining_route[1] ?? robot.target_node_id)],
    ['Возраст состояния агента', value(agentAge, 'с')],
    ['Одометрия', stale(robot) ? 'Устарела / нет данных' : 'Актуальна'],
    ['Возраст одометрии', value(age, 'с')],
  ]) {
    const row = document.createElement('div');
    const label = document.createElement('dt');
    const data = document.createElement('dd');
    label.textContent = name;
    data.textContent = text;
    row.append(label, data);
    details.append(row);
  }
}

new ResizeObserver(render).observe(canvas);
const connection = new StateConnection(state => {
  snapshot = state;
  receivedAt = performance.now();
  motion.update(state.robots, receivedAt, state.stale_after);
  updatePanel();
  render();
  if (motion.isAnimating(receivedAt)) animate();
}, state => {
  connected = state === 'online';
  const indicator = document.getElementById('connection');
  indicator.dataset.state = state;
  indicator.textContent = state === 'online' ? 'WebSocket подключён' :
    state === 'connecting' ? 'Подключение…' : 'Связь потеряна · переподключение';
  updatePanel();
  render();
});
connection.start();
setInterval(() => {
  if (snapshot) {
    updatePanel();
    render();
  }
}, 1000);
window.addEventListener('pagehide', () => {
  connection.stop();
  cancelAnimationFrame(animationFrame);
  animationFrame = null;
});
window.addEventListener('pageshow', event => { if (event.persisted) location.reload(); });
