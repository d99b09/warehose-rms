"""Startup configuration for the map API."""

from pathlib import Path

import yaml
from pydantic import Field, FiniteFloat

from .models import Model

ROOT = Path(__file__).resolve().parents[2]


class WebConfig(Model):
    map_path: Path
    building_path: Path | None = None
    level_name: str | None = None
    ws_hz: FiniteFloat = Field(default=10, ge=1, le=15)
    stale_after: FiniteFloat = Field(default=2, gt=0)
    send_timeout: FiniteFloat = Field(default=2, gt=0, le=30)

    @classmethod
    def load(cls, path: Path | str = ROOT / 'config/web.yaml') -> 'WebConfig':
        path = Path(path).resolve()
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        if not isinstance(data, dict):
            raise ValueError('Web configuration must be a YAML mapping')
        for key in ('map_path', 'building_path'):
            if data.get(key):
                data[key] = (path.parent / data[key]).resolve()
        return cls.model_validate(data)
