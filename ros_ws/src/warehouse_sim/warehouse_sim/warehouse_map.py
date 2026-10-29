"""Load warehouse geometry and navigation graphs from YAML."""

from dataclasses import dataclass
from math import hypot, isfinite
from pathlib import Path

import yaml


def finite_number(value, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} must be a finite number')
    result = float(value)
    if not isfinite(result):
        raise ValueError(f'{name} must be a finite number')
    return result


def _point(value, name: str) -> tuple[float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ValueError(f'{name} must contain [x, y]')
    return tuple(finite_number(v, name) for v in value)


def _read_yaml(path: str | Path) -> dict:
    try:
        with Path(path).open(encoding='utf-8') as stream:
            data = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        raise ValueError(f'Invalid YAML in {path}: {exc}') from exc
    if not isinstance(data, dict):
        raise ValueError(f'Map {path} must be a YAML mapping')
    return data


def _level(data: dict, level_name: str | None) -> tuple[str, dict]:
    levels = data.get('levels')
    if not isinstance(levels, dict) or not levels:
        raise ValueError('Map must contain levels')
    if not level_name:
        if len(levels) != 1:
            raise ValueError('Specify level_name for a map with multiple levels')
        level_name = next(iter(levels))
    if level_name not in levels or not isinstance(levels[level_name], dict):
        raise ValueError(f'Unknown or invalid level: {level_name!r}')
    return level_name, levels[level_name]


def _sequence(value, name: str) -> list:
    if not isinstance(value, list):
        raise ValueError(f'{name} must be a sequence')
    return value


def _indexed_pair(item, count: int, name: str) -> tuple[int, int]:
    if not isinstance(item, list) or len(item) != 3 or not isinstance(item[2], dict):
        raise ValueError(f'{name} must contain [start_index, end_index, properties]')
    start, end = item[:2]
    if any(type(index) is not int or not 0 <= index < count for index in (start, end)):
        raise ValueError(f'{name} references an unknown vertex index')
    return start, end


def _building_geometry(data: dict, level_name: str) -> dict:
    """Convert editor geometry to the same metric frame as an RMF nav export."""
    _, level = _level(data, level_name)
    points = []
    for vertex in _sequence(level.get('vertices'), 'building vertices'):
        if not isinstance(vertex, list) or len(vertex) not in (4, 5):
            raise ValueError('Building vertex must contain [x, y, z, name, optional properties]')
        points.append(_point(vertex[:2], 'building vertex'))
    if not points:
        raise ValueError('Building geometry must contain vertices')
    coordinate_system = data.get('coordinate_system', 'reference_image')
    if coordinate_system == 'reference_image':
        scales = []
        for item in _sequence(level.get('measurements'), 'building measurements'):
            start, end = _indexed_pair(item, len(points), 'Measurement')
            distance = item[2].get('distance')
            if not isinstance(distance, list) or len(distance) != 2 or distance[0] != 3:
                raise ValueError('Measurement distance must contain [3, metres]')
            metres = finite_number(distance[1], 'measurement distance')
            pixels = hypot(points[end][0] - points[start][0], points[end][1] - points[start][1])
            if metres <= 0 or pixels == 0:
                raise ValueError('Measurement length and distance must be positive')
            scales.append(metres / pixels)
        if not scales:
            raise ValueError('reference_image geometry requires measurements in metres')
        scale = sum(scales) / len(scales)
        points = [(x * scale, -y * scale) for x, y in points]
    elif coordinate_system != 'cartesian_meters':
        raise ValueError(f'Unsupported building coordinate_system: {coordinate_system!r}')
    xs, ys = zip(*points)
    walls = []
    for item in _sequence(level.get('walls', []), 'building walls'):
        start, end = _indexed_pair(item, len(points), 'Wall')
        walls.append({'start': points[start], 'end': points[end]})
    return {
        'bounds': {'min_x': min(xs), 'max_x': max(xs), 'min_y': min(ys), 'max_y': max(ys)},
        'walls': walls,
    }


@dataclass(frozen=True)
class Bounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True)
class NavigationNode:
    node_id: str
    x: float
    y: float


@dataclass(frozen=True)
class Edge:
    source: str
    target: str
    bidirectional: bool = True


@dataclass(frozen=True)
class Wall:
    """An impermeable, zero-thickness line segment (metres)."""

    start: tuple[float, float]
    end: tuple[float, float]


class WarehouseMap:
    """Validated bounds, walls, navigation nodes, edges and parking IDs."""

    def __init__(self, data: dict):
        if not isinstance(data, dict):
            raise ValueError('Map must be a YAML mapping')
        try:
            self.bounds = Bounds(**{
                key: finite_number(data['bounds'][key], f'bounds.{key}')
                for key in ('min_x', 'max_x', 'min_y', 'max_y')
            })
            b = self.bounds
            if b.min_x >= b.max_x or b.min_y >= b.max_y:
                raise ValueError('World bounds must have positive dimensions')
            self.nodes: dict[str, NavigationNode] = {}
            for item in data['nodes']:
                node_id = item['id']
                if not isinstance(node_id, str) or not node_id:
                    raise ValueError('Node ID must be a nonempty string')
                if node_id in self.nodes:
                    raise ValueError(f'Duplicate node ID: {node_id}')
                x, y = _point(item['position'], f'node {node_id}')
                if not (b.min_x <= x <= b.max_x and b.min_y <= y <= b.max_y):
                    raise ValueError(f'Node {node_id} is outside world bounds')
                self.nodes[node_id] = NavigationNode(node_id, x, y)
            if not self.nodes:
                raise ValueError('Map must contain navigation nodes')
            self.edges: list[Edge] = []
            arcs = set()
            for item in data['edges']:
                edge = Edge(item['source'], item['target'], item.get('bidirectional', True))
                if not isinstance(edge.bidirectional, bool):
                    raise ValueError('Edge bidirectional must be boolean')
                for node_id in (edge.source, edge.target):
                    if node_id not in self.nodes:
                        raise ValueError(f'Edge references an unknown node: {node_id!r}')
                if edge.source == edge.target:
                    raise ValueError('Self edges are not supported')
                directions = [(edge.source, edge.target)]
                if edge.bidirectional:
                    directions.append((edge.target, edge.source))
                for source, target in directions:
                    if (source, target) in arcs:
                        raise ValueError(f'Duplicate edge: {source} -> {target}')
                    arcs.add((source, target))
                self.edges.append(edge)
            self.parking_nodes = list(data['parking_nodes'])
            if not self.parking_nodes or len(set(self.parking_nodes)) != len(self.parking_nodes):
                raise ValueError('Parking nodes must be nonempty and unique')
            for node_id in self.parking_nodes:
                if node_id not in self.nodes:
                    raise ValueError(f'Parking references an unknown node: {node_id!r}')
            self.walls = tuple(
                Wall(_point(w['start'], 'wall.start'), _point(w['end'], 'wall.end'))
                for w in data['walls']
            )
        except (KeyError, TypeError) as exc:
            raise ValueError(f'Invalid map structure or unknown node: {exc}') from exc

    @classmethod
    def from_yaml(cls, path: str | Path, *, building_path: str | Path | None = None,
                  level_name: str | None = None) -> 'WarehouseMap':
        """Load a directed maps/nav graph; building YAML supplies geometry only."""
        path = Path(path)
        data = _read_yaml(path)
        building_name = data.get('building_name')
        if (not isinstance(building_name, str) or not building_name
                or Path(building_name).name != building_name or building_name in ('.', '..')):
            raise ValueError(
                'Expected an RMF nav graph with building_name; '
                'use maps/nav/<building>/0.yaml')
        level_name, level = _level(data, level_name)
        nodes, parking = [], []
        for index, item in enumerate(_sequence(level.get('vertices'), 'nav vertices')):
            if not isinstance(item, list) or len(item) != 3 or not isinstance(item[2], dict):
                raise ValueError('Nav vertex must contain [x, y, properties]')
            position = _point(item[:2], f'nav vertex {index}')
            name = item[2].get('name', '')
            if not isinstance(name, str):
                raise ValueError('Nav vertex name must be a string')
            name = name or str(index)
            is_parking = item[2].get('is_parking_spot', False)
            if not isinstance(is_parking, bool):
                raise ValueError('is_parking_spot must be boolean')
            nodes.append({'id': name, 'position': position})
            if is_parking:
                parking.append(name)
        if not parking:
            raise ValueError('Nav graph must contain parking nodes (is_parking_spot: true)')
        edges = []
        for item in _sequence(level.get('lanes'), 'nav lanes'):
            start, end = _indexed_pair(item, len(nodes), 'Lane')
            edges.append({'source': nodes[start]['id'], 'target': nodes[end]['id'],
                          'bidirectional': False})
        if not building_path:
            maps_dir = (
                path.parent.parent.parent
                if path.parent.parent.name == 'nav' else path.parent)
            building_path = maps_dir / f'{building_name}.building.yaml'
        building = _read_yaml(building_path)
        if building.get('name') != building_name:
            raise ValueError(f'Building name does not match nav graph: {building_name!r}')
        geometry = _building_geometry(building, level_name)
        warehouse = cls({**geometry, 'nodes': nodes, 'edges': edges,
                         'parking_nodes': parking})
        warehouse.building_name = building_name
        warehouse.level_name = level_name
        return warehouse
