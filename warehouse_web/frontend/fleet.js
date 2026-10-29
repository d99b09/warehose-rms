const labels = {
  WAITING: 'Ожидание', YIELDING: 'Освобождение пути', MOVING: 'Движение',
  EXECUTING: 'Выполнение', QUEUED: 'В очереди', SUSPENDED: 'Приостановлено',
  UNCERTAIN: 'Требуется сверка состояния', SYNCHRONIZING: 'Синхронизация', IDLE: 'Свободен',
};

export function fleetRobot(snapshot, id) {
  return snapshot?.fleet?.robots.find(robot => robot.robot_id === id);
}

export function fleetDetails(snapshot, id, connected = true) {
  const robot = fleetRobot(snapshot, id);
  if (!robot) return [['Диспетчер', 'Нет данных']];
  const stale = !connected || snapshot.fleet.stale;
  return [
    ['Диспетчер', `${labels[robot.status] ?? robot.status}${stale ? ' · устарело' : ''}`],
    ['Причина', robot.message || '—'],
    ['MQTT', stale ? 'Нет актуальных данных' : robot.mqtt_connected ? 'Подключен' : 'Нет связи'],
    ['Задание', robot.order_id || '—'],
    ['Цель задания', robot.queue[0]?.target_node_id ?? '—'],
    ['Оставшийся маршрут', robot.remaining_route.join(' → ') || '—'],
    ['Очередь', robot.queue.map((order, i) => `${i + 1}. ${order.order_type} ${order.target_node_id} (${labels[order.status] ?? order.status})`).join('; ') || 'Пустая'],
    ['Версия VDA', `${robot.confirmed_update_id < 0 ? 'не подтверждена' : robot.confirmed_update_id} / ${robot.order_update_id}`],
  ];
}
