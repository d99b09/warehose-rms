"""Validation against the existing directed WarehouseMap."""
from math import hypot


class RouteError(ValueError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def validate_route(warehouse, node_ids, pose, tolerance):
    if not node_ids or any(not isinstance(n, str) or not n for n in node_ids):
        raise RouteError("INVALID_ROUTE", "Route must contain nonempty node IDs")
    for node in node_ids:
        if node not in warehouse.nodes:
            raise RouteError("UNKNOWN_NODE", f"Unknown navigation node: {node}")
    for source, target in zip(node_ids, node_ids[1:]):
        if not any((edge.source == source and edge.target == target) or
                   (edge.bidirectional and edge.target == source and edge.source == target)
                   for edge in warehouse.edges):
            raise RouteError("INVALID_ROUTE", f"No directed edge {source} -> {target}")
    if pose is None:
        raise RouteError("ODOM_TIMEOUT", "No valid odometry received")
    node = warehouse.nodes[node_ids[0]]
    x, y = node.x, node.y
    if hypot(pose.x - x, pose.y - y) > tolerance:
        raise RouteError("INVALID_ROUTE", "Robot must be within position_tolerance of the first route node")
    return tuple(node_ids)
