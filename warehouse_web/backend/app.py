"""Map API with startup loading and a validated response model."""

import asyncio
import logging
from contextlib import asynccontextmanager, suppress
from pathlib import Path

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from warehouse_sim.warehouse_map import WarehouseMap

from .config import ROOT, WebConfig
from .models import ConnectionRequest, ManualCommand, MapState, OrderRequest, SpawnRequest
from .ros_bridge import RosBridge
from .state_store import StateStore


def create_app(config: WebConfig | None = None, *,
               static_dir: Path | None = None) -> FastAPI:
    config = config or WebConfig.load()
    static_dir = static_dir or ROOT / 'warehouse_web/frontend'
    store = StateStore(config.stale_after)
    bridge = RosBridge(store)

    @asynccontextmanager
    async def lifespan(app):
        warehouse = await asyncio.to_thread(
            WarehouseMap.from_yaml, config.map_path,
            building_path=config.building_path, level_name=config.level_name)
        app.state.map = MapState.from_warehouse(warehouse)
        try:
            await asyncio.to_thread(bridge.start)
            yield
        finally:
            await asyncio.to_thread(bridge.stop)

    app = FastAPI(title='Warehouse RMS2', lifespan=lifespan)
    app.state.store, app.state.bridge = store, bridge

    @app.get('/api/map', response_model=MapState)
    async def get_map():
        return app.state.map

    @app.post('/api/robots')
    async def spawn_robot(body: SpawnRequest):
        logging.getLogger('uvicorn.error').info(
            'Spawn requested: operation_id=%s parking_node_id=%r',
            body.operation_id, body.parking_node_id)
        try:
            reply = await asyncio.to_thread(bridge.spawn, **body.model_dump())
        except (ConnectionError, TimeoutError) as error:
            raise HTTPException(503, str(error)) from error
        return JSONResponse(status_code=reply.pop('http_status'), content=reply)

    @app.post('/api/orders', status_code=202)
    async def submit_order(body: OrderRequest):
        try:
            reply = await asyncio.to_thread(bridge.submit_order, **body.model_dump())
        except (ConnectionError, TimeoutError) as error:
            raise HTTPException(503, str(error)) from error
        return JSONResponse(status_code=reply.pop('http_status'), content=reply)

    async def manual_call(method, *args, **kwargs):
        try:
            return await asyncio.to_thread(method, *args, **kwargs)
        except ValueError as error:
            raise HTTPException(409, str(error)) from error
        except (ConnectionError, TimeoutError) as error:
            raise HTTPException(503, str(error)) from error

    @app.post('/api/robots/{robot_id}/connection')
    async def robot_connection(robot_id: str, body: ConnectionRequest):
        return await manual_call(bridge.robot_connection, robot_id, body.enabled)

    @app.post('/api/robots/{robot_id}/manual/start', status_code=202)
    async def manual_start(robot_id: str):
        return await manual_call(bridge.manual_mode, robot_id, True)

    @app.post('/api/robots/{robot_id}/manual/cmd')
    async def manual_cmd(robot_id: str, body: ManualCommand):
        return await manual_call(bridge.manual_command, robot_id, **body.model_dump())

    @app.post('/api/robots/{robot_id}/manual/stop', status_code=202)
    async def manual_stop(robot_id: str):
        return await manual_call(bridge.manual_mode, robot_id, False)

    @app.websocket('/ws/state')
    async def websocket_state(websocket: WebSocket):
        await websocket.accept()

        async def send():
            while True:
                snapshot = store.snapshot(bridge_ok=bridge.healthy)
                await asyncio.wait_for(websocket.send_json(snapshot.model_dump(mode='json')),
                                       timeout=config.send_timeout)
                await asyncio.sleep(1 / config.ws_hz)

        async def receive():
            while True:
                if (await websocket.receive())['type'] == 'websocket.disconnect':
                    return

        sender = asyncio.create_task(send())
        receiver = asyncio.create_task(receive())
        try:
            done, _ = await asyncio.wait((sender, receiver), return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                task.result()
        except (WebSocketDisconnect, OSError, asyncio.TimeoutError):
            pass
        finally:
            sender.cancel()
            receiver.cancel()
            await asyncio.gather(sender, receiver, return_exceptions=True)
            with suppress(WebSocketDisconnect, RuntimeError, OSError):
                await websocket.close(code=1001)

    @app.get('/', include_in_schema=False)
    async def index():
        return FileResponse(static_dir / 'index.html', headers={'Cache-Control': 'no-cache'})

    app.mount('/assets', StaticFiles(directory=static_dir), name='assets')
    return app
