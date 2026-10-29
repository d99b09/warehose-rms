"""Validated wire contracts. All world geometry uses simulation metres/radians."""

from dataclasses import asdict
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, FiniteFloat

from warehouse_sim.warehouse_map import WarehouseMap


class Model(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


class Point(Model):
    x: FiniteFloat
    y: FiniteFloat


class Bounds(Model):
    min_x: FiniteFloat
    max_x: FiniteFloat
    min_y: FiniteFloat
    max_y: FiniteFloat


class Vertex(Point):
    id: str


class Wall(Model):
    start: Point
    end: Point


class Edge(Model):
    source: str
    target: str
    bidirectional: bool


class MapState(Model):
    schema_version: int = 1
    frame_id: str = "map"
    units: str = "m"
    building_name: str
    level_name: str
    bounds: Bounds
    walls: tuple[Wall, ...]
    vertices: tuple[Vertex, ...]
    edges: tuple[Edge, ...]
    parking: tuple[Vertex, ...]

    @classmethod
    def from_warehouse(cls, warehouse: WarehouseMap) -> "MapState":
        def vertex(node):
            return Vertex(id=node.node_id, x=node.x, y=node.y)

        return cls(
            building_name=warehouse.building_name, level_name=warehouse.level_name,
            bounds=Bounds(**asdict(warehouse.bounds)),
            walls=tuple(Wall(start=Point(x=w.start[0], y=w.start[1]),
                             end=Point(x=w.end[0], y=w.end[1]))
                        for w in warehouse.walls),
            vertices=tuple(vertex(n) for n in warehouse.nodes.values()),
            edges=tuple(Edge(**asdict(e)) for e in warehouse.edges),
            parking=tuple(vertex(warehouse.nodes[key]) for key in warehouse.parking_nodes),
        )


class SpawnRequest(Model):
    operation_id: str = Field(pattern=r'^[A-Za-z0-9_-]{1,128}$')
    parking_node_id: str = Field(default='', max_length=128)


class OrderRequest(Model):
    robot_id: str = Field(min_length=1, max_length=128)
    target_node_id: str = Field(min_length=1, max_length=128)
    order_type: Literal['MOVE', 'PARK'] = 'MOVE'
    order_id: str = Field(default='', max_length=128, pattern=r'^(?:\S(?:.*\S)?)?$')


class ManualCommand(Model):
    session_id: str = Field(min_length=1, max_length=64)
    sequence: int = Field(ge=0, le=9007199254740991, strict=True)
    linear: FiniteFloat = Field(ge=-0.2, le=0.2)
    angular: FiniteFloat = Field(ge=-0.6, le=0.6)


class RobotState(Model):
    robot_id: str
    length: FiniteFloat = Field(gt=0)
    width: FiniteFloat = Field(gt=0)
    x: FiniteFloat | None = None
    y: FiniteFloat | None = None
    theta: FiniteFloat | None = None
    linear_velocity: FiniteFloat | None = None
    angular_velocity: FiniteFloat | None = None
    status: str = 'UNKNOWN'
    agent_status: str | None = None
    order_id: str = ''
    current_node_id: str = ''
    target_node_id: str = ''
    error_code: str = ''
    agent_received_at: FiniteFloat | None = None
    agent_age: FiniteFloat | None = None
    agent_stale: bool = True
    received_at: FiniteFloat | None = None
    status_received_at: FiniteFloat | None = None
    odom_age: FiniteFloat | None = None
    status_age: FiniteFloat | None = None
    stale: bool = True
    status_stale: bool = True


class QueuedOrder(Model):
    order_id: str
    target_node_id: str
    order_type: str
    status: str


class FleetRobot(Model):
    robot_id: str
    status: str
    message: str
    mqtt_connected: bool
    order_id: str
    vda_order_id: str
    order_update_id: int
    confirmed_update_id: int
    remaining_route: tuple[str, ...]
    yielding: bool
    blocking_robots: tuple[str, ...]
    queue: tuple[QueuedOrder, ...]


class FleetState(Model):
    mqtt_connected: bool
    robots: tuple[FleetRobot, ...]
    stale: bool = False


class StateSnapshot(Model):
    schema_version: int = 1
    server_time: FiniteFloat
    stale_after: FiniteFloat
    bridge_ok: bool
    robots: tuple[RobotState, ...]
    fleet: FleetState | None = None
