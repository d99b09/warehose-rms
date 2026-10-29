"""Progress through the exact client-supplied route; no planning."""
from math import hypot


class RouteExecutor:
    def __init__(self, warehouse, nodes):
        self.warehouse, self.nodes = warehouse, tuple(nodes)
        self.completed = 0

    @property
    def current_node_id(self):
        return self.nodes[self.completed - 1] if self.completed else ""

    @property
    def target_node_id(self):
        return self.nodes[self.completed] if self.completed < len(self.nodes) else ""

    @property
    def done(self):
        return self.completed == len(self.nodes)

    def distance(self, pose):
        if self.done:
            return 0.0
        node = self.warehouse.nodes[self.target_node_id]
        x, y = node.x, node.y
        return hypot(x - pose.x, y - pose.y)

    def advance(self, pose, tolerance):
        while not self.done and self.distance(pose) <= tolerance:
            self.completed += 1
