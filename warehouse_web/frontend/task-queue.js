import { fleetTasks } from './fleet.js';

export function setupTaskQueue() {
  const list = document.getElementById('task-list');
  const count = document.getElementById('task-count');
  const state = document.getElementById('task-state');
  const empty = document.getElementById('task-empty');
  const card = list.closest('.task-card');
  let previous = '';

  return {
    update(snapshot, connected, age = 0) {
      const tasks = fleetTasks(snapshot);
      const available = !!snapshot?.fleet;
      const stale = !connected || !snapshot?.bridge_ok || !available ||
        snapshot.fleet.stale || age >= snapshot.stale_after;
      card.dataset.stale = String(stale);
      count.textContent = String(tasks.length);
      state.textContent = !available ? 'Ожидание данных диспетчера' :
        stale ? 'Данные устарели · ожидание обновления' : '';
      state.hidden = available && !stale;
      empty.hidden = !available || stale || tasks.length > 0;

      const signature = JSON.stringify(tasks);
      if (signature === previous) return;
      previous = signature;
      list.replaceChildren(...tasks.map(task => {
        const row = document.createElement('li');
        row.className = 'task-row';
        row.dataset.status = task.status;
        const heading = document.createElement('div');
        heading.className = 'task-row-heading';
        const robot = document.createElement('strong');
        robot.textContent = `№ ${task.number} · ${task.robot_id}`;
        const status = document.createElement('span');
        status.className = 'task-status';
        status.textContent = task.status_label;
        heading.append(robot, status);
        const target = document.createElement('div');
        target.className = 'task-target';
        target.textContent = `${task.order_type} → ${task.target_node_id}`;
        const id = document.createElement('div');
        id.className = 'task-order';
        id.textContent = task.order_id;
        row.append(heading, target, id);
        return row;
      }));
    },
  };
}
