import { fleetRobot } from './fleet.js';

export function setupRobotConnection() {
  const button = document.getElementById('robot-connection');
  const result = document.getElementById('robot-connection-result');
  const expected = new Map();
  let selected = null, enabled = true, pending = false, lastId = null;
  let latest = [null, null, false, 0];

  function update(snapshot, id, connected, age = 0) {
    latest = [snapshot, id, connected, age];
    selected = id;
    if (lastId !== id) result.textContent = '';
    lastId = id;
    const robot = fleetRobot(snapshot, id);
    const fresh = connected && snapshot?.bridge_ok && robot && !snapshot.fleet.stale &&
      age < snapshot.stale_after;
    const waiting = expected.get(id);
    if (fresh && waiting && (robot.mqtt_connected === waiting.enabled || performance.now() > waiting.until)) {
      expected.delete(id);
    }
    enabled = expected.get(id)?.enabled ?? robot?.mqtt_connected ?? true;
    button.textContent = pending ? 'Переключение…' : enabled ? 'Выключить робота' : 'Включить робота';
    button.disabled = pending || !fresh || snapshot.robots.find(r => r.robot_id === id)?.agent_status === 'MANUAL';
  }

  button.onclick = async () => {
    if (button.disabled || pending || !selected) return;
    const id = selected, target = !enabled;
    pending = true;
    result.textContent = '';
    update(...latest);
    try {
      const response = await fetch(`/api/robots/${encodeURIComponent(id)}/connection`, {
        method: 'POST', headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ enabled: target }),
      });
      const data = await response.json();
      if (!response.ok) throw new Error(data.detail ?? `HTTP ${response.status}`);
      expected.set(id, { enabled: target, until: performance.now() + 5000 });
      if (selected === id) result.textContent = data.message;
    } catch (error) {
      if (selected === id) result.textContent = String(error);
    } finally {
      pending = false;
      update(...latest);
    }
  };
  return { update };
}
