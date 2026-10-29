#!/usr/bin/env zsh
# Запускает FastAPI с картой normal (по умолчанию) или stress.
set -e
cd "${0:A:h}/.."
map_name=normal
if [[ $# -gt 0 && $1 != -* ]]; then
  map_name=$1
  shift
fi
if [[ $map_name != normal && $map_name != stress ]]; then
  print -u2 "Usage: $0 [normal|stress]"
  exit 2
fi
source ros_ws/install/setup.zsh
exec .venv/bin/python -m warehouse_web.backend --map "$map_name" "$@"
