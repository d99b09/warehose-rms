"""Single-owner fleet scheduler; transport and time are injected for testing."""
from collections import deque
from dataclasses import dataclass, field
from functools import cache
from math import hypot, isfinite
import time
from uuid import uuid4

from warehouse_sim.collision import segment_distance, point_segment_distance, CollisionChecker
from warehouse_sim.kinematics import Pose
from warehouse_sim.robot import Status
from .planner import PathPlanner
from .yield_planner import YieldPlanner


@dataclass
class Order:
    order_id: str
    robot_id: str
    target: str
    kind: str
    number: int
    status: str = 'QUEUED'
    route: list = field(default_factory=list)
    message: str = ''


@dataclass
class Execution:
    route: list
    execution_id: str = field(default_factory=lambda: str(uuid4()))
    version: int = 0
    confirmed_version: int = -1
    completed: int = 0
    released: int = 0
    grant_at: float = 0.
    service: bool = False
    yield_forbidden_nodes: frozenset = field(default_factory=frozenset)


@dataclass
class Robot:
    robot_id: str
    radius: float
    queue: deque = field(default_factory=deque)
    execution: Execution | None = None
    pose: tuple | None = None
    velocity: tuple = (0., 0.)
    odom_at: float = float('-inf')
    odom_sequence: int = -1
    agent_at: float = float('-inf')
    agent_status: str = 'UNKNOWN'
    agent_epoch: str = ''
    agent_sequence: int = -1
    retired_epochs: set = field(default_factory=set)
    state: dict | None = None
    state_at: float = float('-inf')
    online: bool = False
    synchronized: bool = False
    status: str = 'SYNCHRONIZING'
    message: str = 'Ожидание телеметрии и MQTT'
    reserved: tuple | None = None
    blockers: set = field(default_factory=set)
    cancellation: tuple | None = None
    yield_plan: tuple | None = None
    clearance_for: tuple | None = None
    clearance_paths: list = field(default_factory=list)
    fault: str = ''
    manual_seen: bool = False


class FleetCore:
    def __init__(self, warehouse, transport, *, clock=time.monotonic, tolerance=.15, timeout=2.):
        self.map, self.transport, self.clock = warehouse, transport, clock
        self.planner = PathPlanner(warehouse)
        self.tolerance, self.timeout = tolerance, timeout
        self.robots, self.orders = {}, {}
        self.serial = 0
        self.connected = False
        self.checker = CollisionChecker(warehouse)
        self.traversable = cache(self.traversable)
        self.yield_planner = YieldPlanner(self)

    def register(self, rid, length=.6, width=.4):
        if rid not in self.robots:
            self.robots[rid] = Robot(rid, hypot(length, width)/2)
        return self.robots[rid]

    def connection(self, connected):
        self.connected = connected
        for r in self.robots.values():
            r.synchronized = False
            r.online = False
            r.state_at = float('-inf')

    def odometry(self, rid, x, y, linear, angular, sequence):
        r = self.robots[rid]
        if sequence <= r.odom_sequence or not all(isfinite(v) for v in (x, y, linear, angular)):
            return
        r.pose, r.velocity = (x, y), (linear, angular)
        r.odom_at, r.odom_sequence = self.clock(), sequence

    def agent(self, rid, status, epoch, sequence):
        r = self.robots[rid]
        if epoch in r.retired_epochs or (epoch == r.agent_epoch and sequence <= r.agent_sequence):
            return
        if epoch != r.agent_epoch:
            if r.agent_epoch:
                r.retired_epochs.add(r.agent_epoch)
                r.synchronized = False
                if r.execution:
                    r.fault = 'Агент перезапущен; требуется восстановление'
            r.agent_epoch = epoch
        r.agent_status, r.agent_sequence, r.agent_at = status, sequence, self.clock()
        if status == 'MANUAL':
            r.manual_seen = True

    def position_node(self, r):
        if r.pose is None:
            return ''
        node = min(self.map.nodes.values(), key=lambda n: hypot(n.x-r.pose[0], n.y-r.pose[1]))
        return node.node_id if hypot(node.x-r.pose[0], node.y-r.pose[1]) <= self.tolerance else ''

    def stopped(self, r):
        return (self.clock()-r.odom_at < self.timeout and self.clock()-r.agent_at < self.timeout
                and max(map(abs, r.velocity)) < .001 and bool(self.position_node(r))
                and r.agent_status in {'IDLE', 'PARKED', 'WAITING', 'STOPPED'})

    def fresh(self, r):
        return (self.connected and r.online and self.clock()-r.state_at < self.timeout
                and self.clock()-r.odom_at < self.timeout and self.clock()-r.agent_at < self.timeout)

    def unavailable_nodes(self, rid=None):
        """Keep the last position and any unconfirmed transition out of routing."""
        nodes = set()
        for other in self.robots.values():
            if other.robot_id != rid and (not self.fresh(other) or not other.synchronized):
                nodes.add(self.position_node(other))
                nodes.update(other.reserved or ())
        nodes.discard('')
        return frozenset(nodes)

    def submit(self, rid, target, kind='MOVE', order_id=''):
        if order_id in self.orders:
            old = self.orders[order_id]
            if (old.robot_id, old.target, old.kind) != (rid, target, kind):
                return False, 409, 'order_id уже используется с другими параметрами', old
            return True, 202, 'Задание уже принято', old
        if rid not in self.robots:
            return False, 400, 'Неизвестный робот', None
        if target not in self.map.nodes or kind not in {'MOVE', 'PARK'} or (kind == 'PARK' and target not in self.map.parking_nodes):
            return False, 400, 'Недопустимый узел или тип задания', None
        r = self.robots[rid]
        if not self.connected or not r.online:
            return False, 409, 'Нет связи с роботом: построение маршрута невозможно', None
        if len(r.queue) >= 16 or sum(len(x.queue) for x in self.robots.values()) >= 512:
            return False, 409, 'Очередь заданий заполнена', None
        start = self.orders[r.queue[-1]].target if r.queue else self.position_node(r)
        if start:
            try:
                self.planner.find_path(start, target, self.unavailable_nodes(rid))
            except ValueError as error:
                return False, 409, str(error), None
        self.serial += 1
        order = Order(order_id or str(uuid4()), rid, target, kind, self.serial)
        self.orders[order.order_id] = order
        r.queue.append(order.order_id)
        return True, 202, 'Задание принято в очередь', order

    def _point(self, node):
        n = self.map.nodes[node]
        return (n.x, n.y)

    def _crosses_nodes(self, a, b, nodes):
        p, q = self._point(a), self._point(b)
        return any(point_segment_distance(self._point(node), p, q) <= 1e-9 for node in nodes)

    def _yield_claims(self, r):
        e = r.execution
        route = r.yield_plan[0] if r.yield_plan else []
        if not route and e:
            route = e.route[e.completed:]
            if not e.service:
                goal = r.clearance_paths[0][1][-1] if r.clearance_paths else None
                route = route[:route.index(goal)+1] if goal in route else []
        return [(r.robot_id, route)] + r.clearance_paths[1:] if route else []

    def _segments(self, r, poses=None):
        segments = []
        pose = r.pose if poses is None else poses[r.robot_id]
        if pose is not None:
            segments.append((pose, pose))
        if r.reserved:
            segments.append(tuple(self._point(n) for n in r.reserved))
        return segments

    def blockers(self, rid, a, b, poses=None, intentions=False):
        r = self.robots[rid]
        p, q = self._point(a), self._point(b)
        return {other.robot_id for other in self.robots.values() if other.robot_id != rid and
                (any(segment_distance(p, q, u, v) <= r.radius+other.radius+2*self.tolerance
                     for u, v in self._segments(other, poses)) or
                 intentions and any(self.conflicts(rid, a, b, owner, path)
                                    for owner, path in self._yield_claims(other)))}

    def conflicts(self, rid, a, b, other_id, route):
        return any(segment_distance(self._point(a), self._point(b),
                   self._point(u), self._point(v)) <=
                   self.robots[rid].radius + self.robots[other_id].radius + 2*self.tolerance
                   for u, v in zip(route, route[1:] or route))

    def traversable(self, rid, a, b):
        return self.checker.check_motion(Pose(*self._point(a), 0), Pose(*self._point(b), 0),
                                         self.robots[rid].radius+self.tolerance) == Status.OK

    def state(self, rid, message):
        r = self.robots[rid]
        e = r.execution
        if e and e.confirmed_version >= 0 and message['orderId'] != e.execution_id:
            r.synchronized = False
        if e and message['orderId'] == e.execution_id:
            version, seq = message['orderUpdateId'], message['lastNodeSequenceId']
            if version > e.version or seq % 2 or seq//2 > e.released:
                r.fault = 'Недопустимый прогресс VDA'
                return
            if version != e.version or seq//2 < e.completed:
                return
            if message['lastNodeId'] != e.route[seq//2]:
                r.fault = 'VDA сообщает неверный узел'
                return
            expected_nodes = [(e.route[i], i*2, i <= e.released) for i in range(seq//2+1, len(e.route))]
            actual_nodes = [(n['nodeId'], n['sequenceId'], n['released']) for n in message['nodeStates']]
            if not r.cancellation and expected_nodes != actual_nodes:
                r.fault = 'Остаток маршрута VDA не совпадает с выданным'
                return
            e.confirmed_version = version
            if any(x['errorLevel'] == 'FATAL' for x in message['errors']):
                r.fault = 'Ошибка исполнения VDA; требуется восстановление'
        r.state, r.state_at = message, self.clock()

    def _send(self, r, offset):
        e = r.execution
        self.transport.order(r.robot_id, e.execution_id, e.version, e.route[offset:], offset,
                             e.released-offset)

    def _start(self, r, route, *, service=False, beneficiary=None, forbidden_nodes=()):
        if beneficiary:
            r.clearance_for = (beneficiary.robot_id, beneficiary.execution.execution_id)
        r.execution = Execution(list(route), service=service,
                                yield_forbidden_nodes=frozenset(forbidden_nodes))
        self._send(r, 0)
        r.status = 'YIELDING' if service else 'EXECUTING'
        r.message = 'Освобождение пути' if service else 'Маршрут отправлен'
        if r.queue and not service:
            order = self.orders[r.queue[0]]
            order.route, order.status = list(route), 'EXECUTING'

    def _cancel(self, r, reason):
        if not r.cancellation:
            aid = str(uuid4())
            r.cancellation = (aid, reason)
            self.transport.cancel(r.robot_id, aid)

    def _consume_cancel(self, r):
        if not r.cancellation or not r.state:
            return False
        aid, reason = r.cancellation
        done = next((a for a in r.state['actionStates'] if a['actionId'] == aid), None)
        no_order = bool(done and done['actionStatus'] == 'FAILED'
                        and done.get('resultDescription') == 'No active order to cancel'
                        and not r.state['nodeStates'])
        if not done or (done['actionStatus'] != 'FINISHED' and not no_order) or r.state['driving'] or not self.stopped(r):
            r.status = 'UNCERTAIN' if done and done['actionStatus'] == 'FAILED' else 'WAITING'
            r.message = 'Ожидание подтвержденной отмены и остановки в узле'
            return True
        r.cancellation = None
        r.reserved = None
        r.execution = None
        r.fault = ''
        if reason == 'yield':
            planned_route, beneficiary_id, forbidden = r.yield_plan
            r.yield_plan = None
            beneficiary = self.robots.get(beneficiary_id)
            route = None
            if (beneficiary and beneficiary.execution and
                    r.clearance_for == (beneficiary_id, beneficiary.execution.execution_id)):
                forbidden |= self._beneficiary_nodes(beneficiary)
                if self._safe_retreat(r, planned_route, forbidden):
                    route = planned_route
            if route:
                self._start(r, route, service=True, beneficiary=beneficiary,
                            forbidden_nodes=forbidden)
            else:
                r.clearance_paths.clear()
                r.status, r.message = 'WAITING', 'Нет доступного пути отъезда'
                if r.queue:
                    self.orders[r.queue[0]].status = 'QUEUED'
        else:
            r.clearance_paths.clear()
            r.synchronized = True
        return True

    def _synchronize(self, r):
        if not self.fresh(r):
            r.status, r.message = 'UNCERTAIN', 'Нет актуального MQTT state или телеметрии; узел заблокирован'
            return False
        if r.manual_seen and r.agent_status != 'MANUAL' and self.stopped(r):
            r.manual_seen = False
            if r.execution:
                self._cancel(r, 'replan')
            else:
                r.fault = ''
        if r.cancellation:
            self._consume_cancel(r)
            return False
        if not r.synchronized:
            if r.execution:
                if r.state['orderId'] != r.execution.execution_id:
                    r.status, r.message = 'UNCERTAIN', 'Заказ робота отличается от ожидаемого'
                    return False
                if r.state['orderUpdateId'] != r.execution.version:
                    r.status, r.message = 'UNCERTAIN', 'Ожидание сверки версии заказа'
                    return False
                r.synchronized = True
            elif r.state['nodeStates'] or r.state['driving'] or any(a['actionStatus'] in {'WAITING', 'RUNNING', 'INITIALIZING'} for a in r.state['actionStates']):
                # The old Base may still be in progress after dispatcher restart.
                # Cancel only at a confirmed node so we do not strand the robot
                # between nodes while reconciling the previous order.
                if r.state['driving'] or not self.stopped(r):
                    r.status = 'WAITING'
                    r.message = 'Ожидание остановки в узле перед отменой старого задания'
                    return False
                self._cancel(r, 'startup')
                return False
            elif self.stopped(r):
                r.synchronized = True
            else:
                r.status, r.message = 'UNCERTAIN', 'Требуется восстановление положения в узле'
                return False
        return True

    def _progress(self, r):
        e, state = r.execution, r.state
        if not e or state['orderId'] != e.execution_id or e.confirmed_version != e.version:
            return
        index = state['lastNodeSequenceId']//2
        if (index == e.released and index > e.completed and not state['driving']
                and self.stopped(r) and self.position_node(r) == e.route[index]
                and r.odom_at > e.grant_at):
            e.completed = index
            r.reserved = None
        if (r.clearance_paths and not r.reserved and self.stopped(r) and
                self.position_node(r) == r.clearance_paths[0][1][-1]):
            r.clearance_paths.clear()
        if e.completed == len(e.route)-1 and not state['nodeStates'] and self.stopped(r):
            if not e.service and r.queue:
                order = self.orders[r.queue.popleft()]
                order.status, order.message = 'COMPLETED', 'Задание выполнено'
            r.execution = None
            r.status, r.message = 'IDLE', ''

    def _beneficiary(self, r):
        if r.clearance_for:
            rid, eid = r.clearance_for
            other = self.robots.get(rid)
            if other and other.execution and other.execution.execution_id == eid:
                return other
            r.clearance_for = None

    def _step(self, r):
        e = r.execution
        if (e and not r.fault and not r.cancellation and self.fresh(r) and self.stopped(r)
                and e.confirmed_version == e.version and e.completed == e.released
                and e.completed < len(e.route)-1 and not self.transport.uncertain(r.robot_id)):
            return e.route[e.completed:e.completed+2]

    def tick(self):
        if not self.connected:
            for r in self.robots.values():
                r.status, r.message = 'UNCERTAIN', 'Нет соединения с Mosquitto'
            return
        ready = []
        for r in self.robots.values():
            r.blockers.clear()
            if self._synchronize(r):
                if not r.fault:
                    self._progress(r)
                ready.append(r)
        # Unlocalized/manual robots can enter any corridor. Known stale claims remain held.
        if any(r.pose is None or r.agent_status == 'MANUAL' or
               (not r.reserved and not self.position_node(r)) for r in self.robots.values()):
            for r in ready:
                r.status, r.message = 'WAITING', 'Ручное управление или неизвестное положение робота'
            return
        for r in sorted(ready, key=self._priority):
            blocked = self.unavailable_nodes(r.robot_id)
            e = r.execution
            if (e and not r.cancellation and self._step(r) and
                    blocked.intersection(e.route[e.completed:])):
                # Cancel only at a confirmed node. Already released motion keeps
                # its reservation until it has finished and been acknowledged.
                self._cancel(r, 'replan')
                r.status, r.message = 'WAITING', 'Перестроение маршрута: узел заблокирован роботом без связи'
                continue
            if not r.execution and r.queue and not r.fault and self.stopped(r):
                order = self.orders[r.queue[0]]
                try:
                    self._start(r, self.planner.find_path(self.position_node(r), order.target, blocked))
                except ValueError as error:
                    if blocked:
                        order.status, order.message = 'WAITING', str(error)
                        r.status, r.message = 'WAITING', 'Нет маршрута в обход узлов роботов без связи'
                    else:
                        order.status, order.message = 'FAILED', str(error)
                        r.queue.popleft()
        failures = self._resolve(ready)
        for r in sorted(ready, key=self._priority):
            if r.fault or r.agent_status in {'ERROR', 'MANUAL'}:
                r.status, r.message = 'UNCERTAIN', r.fault or 'Ошибка или ручное управление агента'
                continue
            if r.cancellation:
                continue
            e = r.execution
            if not e:
                if not r.queue:
                    r.status, r.message = 'IDLE', ''
                continue
            if self.transport.uncertain(r.robot_id):
                r.status, r.message = 'UNCERTAIN', 'Нет подтверждения команды VDA'
                continue
            if e.completed < e.released:
                r.status = 'YIELDING' if e.service else 'MOVING'
                continue
            step = self._step(r)
            if not step:
                continue
            a, b = step
            reason = self._blocked(r, a, b)
            if reason:
                r.status, r.message = 'WAITING', reason + failures.get(r.robot_id, '')
                continue
            r.reserved = (a, b)
            previous = e.released
            e.released += 1
            e.version += 1
            e.grant_at = self.clock()
            self._send(r, previous)
            r.status, r.message = ('YIELDING' if e.service else 'MOVING'), ''

    def _priority(self, r):
        service = bool(self._yield_claims(r))
        owner = self._beneficiary(r) if service else None
        queue = owner.queue if owner else r.queue
        return (self.orders[queue[0]].number if queue else float('inf'), not service, r.robot_id)

    def _beneficiary_nodes(self, beneficiary):
        nodes = {self.position_node(beneficiary)}
        if beneficiary.execution:
            nodes.add(beneficiary.execution.route[beneficiary.execution.completed])
        nodes.discard('')
        return frozenset(nodes)

    def _safe_retreat(self, r, route, forbidden):
        return (self.stopped(r) and self.position_node(r) == route[0] and
                all(not self._crosses_nodes(a, b, forbidden) and
                    not self.blockers(r.robot_id, a, b) and self.traversable(r.robot_id, a, b)
                    for a, b in zip(route, route[1:])))

    def _yield(self, r, beneficiary, route, *, forbidden_nodes=(), paths=()):
        forbidden = self._beneficiary_nodes(beneficiary) | frozenset(forbidden_nodes)
        r.clearance_for = (beneficiary.robot_id, beneficiary.execution.execution_id)
        r.clearance_paths = list(paths)
        if r.queue:
            self.orders[r.queue[0]].status = 'SUSPENDED'
        if r.execution:
            r.yield_plan = (route, beneficiary.robot_id, forbidden)
            self._cancel(r, 'yield')
            r.message = 'Приостановка задания для освобождения пути'
        else:
            self._start(r, route, service=True, beneficiary=beneficiary,
                        forbidden_nodes=forbidden)

    def _can_displace(self, r, beneficiary):
        e = r.execution
        if (r is beneficiary or self._priority(r) <= self._priority(beneficiary) or
                not self.fresh(r) or not r.synchronized or not self.stopped(r) or
                r.fault or r.reserved or r.cancellation or self.transport.uncertain(r.robot_id)):
            return False
        if e and (self._yield_claims(r) or e.confirmed_version != e.version or e.completed != e.released):
            return False
        previous = self._beneficiary(r)
        return (not previous or previous is beneficiary or
                self._priority(previous) >= self._priority(beneficiary))

    def _priority_conflict(self, r, a, b):
        beneficiary = self._beneficiary(r)
        if beneficiary and not self._yield_claims(r):
            e = beneficiary.execution
            if self.conflicts(r.robot_id, a, b, beneficiary.robot_id, e.route[e.completed:]):
                return True
        for other in self.robots.values():
            if (other is not r and self._priority(other) < self._priority(r) and
                    any(self.conflicts(r.robot_id, a, b, owner, path)
                        for owner, path in self._yield_claims(other))):
                return True
        return False

    def _blocked(self, r, a, b):
        r.blockers = self.blockers(r.robot_id, a, b)
        if r.execution.service:
            other = self._beneficiary(r)
            forbidden = r.execution.yield_forbidden_nodes | (
                self._beneficiary_nodes(other) if other else frozenset())
            if self._crosses_nodes(a, b, forbidden):
                return 'Узел робота, которому уступают, недоступен'
        if self._priority_conflict(r, a, b):
            return 'Переход мешает освобождению приоритетного пути'
        if r.blockers:
            return 'Путь занят: '+', '.join(sorted(r.blockers))
        if not self.traversable(r.robot_id, a, b):
            return 'Переход не обеспечивает допустимый просвет'
        if not r.execution.service:
            ahead = r.execution.route[r.execution.completed+1:]
            for other in self.robots.values():
                if (other is not r and not other.reserved and self.position_node(other) in ahead and
                        not self.yield_planner.plan(r, other, forbidden_nodes={b}, static=True)):
                    return 'Переход перекрывает выезд роботу '+other.robot_id

    def _departing(self, r, waiting):
        step = self._step(r)
        route = waiting.execution.route[waiting.execution.completed:]
        return bool(step and not self._blocked(r, *step) and
                    (r.robot_id in self.blockers(waiting.robot_id, *route[:2]) or
                     not self.conflicts(waiting.robot_id, *route[:2], r.robot_id, step)))

    def _resolve(self, ready):
        failures = {}
        for waiting in sorted(ready, key=self._priority):
            e = waiting.execution
            if (not e or self._yield_claims(waiting) or waiting.cancellation or waiting.fault or
                    waiting.reserved or not self.stopped(waiting)):
                continue
            if any(self._beneficiary(r) is waiting and self._yield_claims(r)
                   for r in self.robots.values()):
                continue
            route = e.route[e.completed:]
            occupants = set().union(*(self.blockers(waiting.robot_id, a, b)
                                     for a, b in zip(route, route[1:])))
            for rid in sorted(occupants, key=lambda rid: self._priority(self.robots[rid])):
                displaced = self.robots[rid]
                if self._departing(displaced, waiting):
                    continue
                plan = self.yield_planner.plan(waiting, displaced)
                if plan:
                    move = plan[0]
                    leaf = self.robots[move.robot_id]
                    paths = [(m.robot_id, m.route) for m in plan]
                    own_route = leaf.execution.route[leaf.execution.completed:] if leaf.execution else []
                    if self._departing(leaf, waiting) and own_route[:len(move.route)] == move.route:
                        leaf.clearance_for = (waiting.robot_id, e.execution_id)
                        leaf.clearance_paths = paths
                    else:
                        self._yield(leaf, waiting, move.route,
                                    forbidden_nodes=move.forbidden_nodes, paths=paths)
                    break
                failures[waiting.robot_id] = ('; поиск цепочки достиг лимита' if self.yield_planner.exhausted
                                              else '; нет доступной цепочки отъезда')
        return failures

    def snapshot(self):
        robots = []
        for r in self.robots.values():
            e = r.execution
            robots.append(dict(robot_id=r.robot_id, status=r.status, message=r.message,
                mqtt_connected=self.connected and r.online,
                order_id=r.queue[0] if r.queue else '',
                vda_order_id=e.execution_id if e else '',
                order_update_id=e.version if e else 0,
                confirmed_update_id=e.confirmed_version if e else -1,
                remaining_route=e.route[e.completed:] if e else [],
                yielding=bool(e and e.service), blocking_robots=sorted(r.blockers),
                queue=[dict(order_id=oid, target_node_id=self.orders[oid].target,
                            order_type=self.orders[oid].kind, status=self.orders[oid].status,
                            number=self.orders[oid].number)
                       for oid in r.queue]))
        return dict(mqtt_connected=self.connected, robots=robots,
                    blocked_nodes=sorted(self.unavailable_nodes()))
