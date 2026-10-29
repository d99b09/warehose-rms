#!/usr/bin/env zsh
# Публикует карту normal (по умолчанию) или stress в /warehouse/map_markers.
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
source /opt/ros/jazzy/setup.zsh
source ros_ws/install/setup.zsh
map_path="$PWD/maps/nav/warehouse_${map_name}/0.yaml"
vda_config=${WAREHOUSE_VDA_CONFIG:-$PWD/ros_ws/src/vda_bridge/config/vda5050.yaml}
broker_pid=""
# A custom broker configuration is managed by its operator.
if [[ -z ${WAREHOUSE_VDA_CONFIG:-} ]]; then
  if ! .venv/bin/python -c 'import socket; socket.create_connection(("127.0.0.1", 1883), timeout=0.3).close()' 2>/dev/null; then
    mosquitto -c "$PWD/config/mosquitto/mosquitto.conf" &
    broker_pid=$!
  fi
fi
.venv/bin/python ros_ws/install/fleet_dispatcher/lib/fleet_dispatcher/dispatcher --ros-args -p map_path:="$map_path" -p vda_config_path:="$vda_config" &
dispatcher_pid=$!
cleanup() {
  kill -TERM $dispatcher_pid 2>/dev/null || true
  wait $dispatcher_pid 2>/dev/null || true
  if [[ -n $broker_pid ]]; then
    kill -TERM $broker_pid 2>/dev/null || true
    wait $broker_pid 2>/dev/null || true
  fi
}
trap cleanup EXIT
.venv/bin/python ros_ws/install/warehouse_sim/lib/warehouse_sim/map --ros-args -p map_path:="$map_path" -p vda_config_path:="$vda_config" "$@"
