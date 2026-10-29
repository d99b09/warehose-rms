"""RViz markers for the loaded warehouse map."""

from math import hypot, sqrt

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from .robot import Status


def _marker(frame, namespace, index, kind, color):
    marker = Marker()
    marker.header.frame_id = frame
    marker.ns, marker.id, marker.type = namespace, index, kind
    marker.pose.orientation.w = 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
    return marker


def map_markers(warehouse):
    result = MarkerArray()
    clear = Marker()
    clear.header.frame_id = 'map'
    clear.pose.orientation.w = 1.0
    clear.action = Marker.DELETEALL
    result.markers.append(clear)
    bounds = warehouse.bounds
    floor = _marker('map', 'floor', 0, Marker.CUBE, (0.18, 0.22, 0.28, 0.7))
    floor.pose.position.x = (bounds.min_x + bounds.max_x) / 2
    floor.pose.position.y = (bounds.min_y + bounds.max_y) / 2
    floor.pose.position.z = -0.02
    floor.scale.x, floor.scale.y = bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y
    floor.scale.z = 0.02
    result.markers.append(floor)
    walls = _marker('map', 'walls', 0, Marker.LINE_LIST, (0.9, 0.9, 0.9, 1.0))
    walls.scale.x = 0.08
    for wall in warehouse.walls:
        walls.points.extend(
            Point(x=float(x), y=float(y), z=0.04) for x, y in (wall.start, wall.end))
    result.markers.append(walls)
    for index, edge in enumerate(warehouse.edges):
        a, b = warehouse.nodes[edge.source], warehouse.nodes[edge.target]
        length = hypot(b.x - a.x, b.y - a.y)
        if length == 0:
            continue
        ux, uy = (b.x - a.x) / length, (b.y - a.y) / length
        inset = min(0.2, length / 4)
        arrow = _marker('map', 'lanes', index, Marker.ARROW, (0.25, 0.65, 1.0, 0.85))
        arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.025, 0.12, min(0.18, length / 4)
        arrow.points = [Point(x=a.x + ux * inset - uy * 0.04,
                              y=a.y + uy * inset + ux * 0.04, z=0.06),
                        Point(x=b.x - ux * inset - uy * 0.04,
                              y=b.y - uy * inset + ux * 0.04, z=0.06)]
        result.markers.append(arrow)
    parking = set(warehouse.parking_nodes)
    for index, node in enumerate(warehouse.nodes.values()):
        color = (1.0, 0.65, 0.1, 1.0) if node.node_id in parking else (0.25, 0.8, 1.0, 1.0)
        dot = _marker('map', 'nodes', index, Marker.SPHERE, color)
        dot.pose.position = Point(x=node.x, y=node.y, z=0.1)
        dot.scale.x = dot.scale.y = dot.scale.z = 0.18
        result.markers.append(dot)
        label = _marker('map', 'node_labels', index, Marker.TEXT_VIEW_FACING, color)
        label.pose.position = Point(x=node.x, y=node.y + 0.32, z=0.15)
        label.scale.z = 0.28
        label.text = node.node_id
        result.markers.append(label)
    return result


def robot_markers(robot, frame, stamp):
    color = (0.2, 0.9, 0.35, 1.0) if robot.state == Status.OK else (1.0, 0.2, 0.15, 1.0)
    body = _marker(frame, 'body', 0, Marker.CUBE, color)
    body.scale.x, body.scale.y, body.scale.z = robot.config.length, robot.config.width, 0.16
    body.pose.position.z = robot.config.wheel_radius + 0.08
    markers = [body]
    for index, side in enumerate((-1, 1)):
        wheel = _marker(frame, 'wheels', index, Marker.CYLINDER, (0.1, 0.1, 0.1, 1.0))
        wheel.pose.position.y = side * robot.config.wheel_base / 2
        wheel.pose.position.z = robot.config.wheel_radius
        wheel.pose.orientation.x = wheel.pose.orientation.w = sqrt(0.5)
        wheel.scale.x = wheel.scale.y = robot.config.wheel_radius * 2
        wheel.scale.z = 0.05
        markers.append(wheel)
    heading = _marker(frame, 'heading', 0, Marker.ARROW, (1.0, 0.9, 0.2, 1.0))
    heading.pose.position.z = 0.4
    heading.scale.x, heading.scale.y, heading.scale.z = 0.8, 0.05, 0.12
    markers.append(heading)
    label = _marker(frame, 'robot_status', 0, Marker.TEXT_VIEW_FACING, color)
    label.pose.position.z = 0.8
    label.scale.z = 0.28
    label.text = (f'{robot.robot_id}: {robot.state.value}\n'
                  f'v={robot.linear_velocity:.2f} m/s, w={robot.angular_velocity:.2f} rad/s')
    markers.append(label)
    for marker in markers:
        marker.header.stamp = stamp
        marker.frame_locked = True
    return MarkerArray(markers=markers)
