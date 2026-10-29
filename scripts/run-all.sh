#!/usr/bin/env zsh
# Запускает ROS и FastAPI с одной картой: normal (по умолчанию) или stress.
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
setsid ./scripts/run-simulation.sh "$map_name" &
map_pid=$!
trap 'kill -TERM -- -$map_pid 2>/dev/null || true; wait $map_pid 2>/dev/null || true' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
./scripts/run-fastapi.sh "$map_name" "$@"
