"""Backtracking clearance search over virtual, stationary robot positions."""
from dataclasses import dataclass

import networkx as nx


@dataclass
class YieldMove:
    robot_id: str
    route: list
    forbidden_nodes: frozenset


class YieldPlanner:
    def __init__(self, fleet, search_limit=2000):
        self.fleet = fleet
        self.search_limit = search_limit
        self.exhausted = False

    def plan(self, beneficiary, displaced, *, forbidden_nodes=(), static=False):
        """Return deepest-first moves, without publishing or changing fleet state.

        A failed child explores its other retreats before its parent tries a
        different destination. Ancestors and already relocated robots are fixed
        within a branch, so a cycle cannot masquerade as an available retreat.
        """
        f = self.fleet
        self.exhausted = False
        if (not beneficiary.execution or displaced.reserved or not f.stopped(displaced)):
            return None
        root = beneficiary.robot_id
        route = beneficiary.execution.route[beneficiary.execution.completed:]
        positions = {rid: r.pose for rid, r in f.robots.items()}
        movable = {rid for rid, r in f.robots.items()
                   if f._can_displace(r, beneficiary)}
        if not static and displaced.robot_id not in movable:
            return None
        higher = [(rid, r.execution.route[r.execution.completed:])
                  for rid, r in f.robots.items() if r is not beneficiary and r.execution
                  and not r.execution.service and f._priority(r) < f._priority(beneficiary)]
        remaining = self.search_limit

        def spend():
            nonlocal remaining
            remaining -= 1
            if remaining < 0:
                self.exhausted = True
                return False
            return True

        def blockers(rid, path, poses):
            return next((other for a, b in zip(path, path[1:])
                         for other in sorted(f.blockers(rid, a, b, poses, True))), None)

        def clear_path(rid, path, protected, ancestors, poses, moves):
            if not spend():
                return
            blocked_by = None if static else blockers(rid, path, poses)
            if blocked_by is None:
                yield poses, moves
                return
            if (blocked_by in ancestors or blocked_by not in movable or
                    any(move.robot_id == blocked_by for move in moves)):
                return
            for new_poses, new_moves in retreat(
                    blocked_by, protected, ancestors, poses, moves):
                yield from clear_path(rid, path, protected, ancestors,
                                      new_poses, new_moves)

        def retreat(rid, protected, ancestors, poses, moves):
            robot = f.robots[rid]
            start = f.position_node(robot)
            forbidden = (frozenset(forbidden_nodes) | f.unavailable_nodes(rid) | f._beneficiary_nodes(beneficiary) |
                         frozenset(f.position_node(f.robots[a]) for a in ancestors))
            if not start or start in forbidden:
                return

            def edge_ok(a, b):
                return (not f._crosses_nodes(a, b, forbidden) and f.traversable(rid, a, b) and
                        (not static or not any(f.conflicts(rid, a, b, root, [n]) for n in forbidden)) and
                        not any(f.conflicts(rid, a, b, owner, path) for owner, path in higher))

            view = nx.subgraph_view(f.planner.graph,
                                   filter_node=lambda n: n not in forbidden,
                                   filter_edge=edge_ok)
            distances, paths = nx.single_source_dijkstra(view, start, weight='length')
            for target in sorted(distances, key=lambda n: (distances[n], n)):
                if not spend():
                    return
                if target == start:
                    continue
                point = f._point(target)
                if any(f.conflicts(rid, target, target, owner, path) for owner, path in protected):
                    continue
                path = paths[target]
                for new_poses, new_moves in clear_path(
                        rid, path, protected + [(rid, path)], ancestors | {rid},
                        poses, moves):
                    yield ({**new_poses, rid: point},
                           new_moves + [YieldMove(rid, path, forbidden)])

        result = next(retreat(displaced.robot_id, [(root, route)], {root}, positions, []), None)
        return result[1] if result else None
