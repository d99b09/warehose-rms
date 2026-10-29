"""Command-line entry point for the map API."""

import argparse
from pathlib import Path

import uvicorn

from .app import create_app
from .config import ROOT, WebConfig


def main():
    parser = argparse.ArgumentParser(description='Warehouse RMS2 map API')
    parser.add_argument('--config', type=Path, default=ROOT / 'config/web.yaml')
    parser.add_argument('--map', choices=['normal', 'stress'])
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8000)
    args = parser.parse_args()
    config = WebConfig.load(args.config)
    if args.map:
        config = config.model_copy(update={
            'map_path': ROOT / f'maps/nav/warehouse_{args.map}/0.yaml',
            'building_path': None, 'level_name': None})
    uvicorn.run(create_app(config), host=args.host, port=args.port,
                workers=1)


if __name__ == '__main__':
    main()
