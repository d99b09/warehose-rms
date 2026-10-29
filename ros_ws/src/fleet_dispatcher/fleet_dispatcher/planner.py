"""Directed A* adapted from fleet_dispatcher for one robot."""
from math import hypot
import networkx as nx


class PathPlanner:
    def __init__(self, warehouse):
        self.map = warehouse
        graph = nx.DiGraph()
        graph.add_nodes_from(warehouse.nodes)
        for edge in warehouse.edges:
            a, b = warehouse.nodes[edge.source], warehouse.nodes[edge.target]
            length = hypot(a.x - b.x, a.y - b.y)
            graph.add_edge(edge.source, edge.target, length=length)
            if edge.bidirectional:
                graph.add_edge(edge.target, edge.source, length=length)
        self.graph = nx.freeze(graph)

    def find_path(self, start, target):
        if start not in self.graph or target not in self.graph:
            raise ValueError('Неизвестный узел карты')
        def heuristic(a, b):
            p, q = self.map.nodes[a], self.map.nodes[b]
            return hypot(q.x - p.x, q.y - p.y)
        try:
            return nx.astar_path(self.graph, start, target, heuristic=heuristic, weight='length')
        except nx.NetworkXNoPath as error:
            raise ValueError('Маршрут до целевого узла не найден') from error
