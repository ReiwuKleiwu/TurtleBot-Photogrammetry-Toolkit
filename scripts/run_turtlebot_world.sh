#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/common.sh
source "${SCRIPT_DIR}/common.sh"
load_project_config

WORLD_NAME="${TURTLEBOT_WORLD}"
RVIZ="true"
LOCALIZATION="true"
NAV2="true"
ROBOT_X="-3.5"
ROBOT_Y="-4.5"
ROBOT_Z="0.35"
ROBOT_YAW="1.57"
GUI_CONFIG_OVERRIDE=""

usage() {
  cat <<'EOF_USAGE'
Usage: run_turtlebot_world.sh [options]

Options:
  --world NAME          Gazebo world name passed to turtlebot4_gz.launch.py.
                        Example: small_house, warehouse, depot
  --map PATH            Nav2/localization map yaml. Relative paths resolve from repo root.
  --gui-config PATH     Gazebo GUI config. Relative paths resolve from repo root.
  --rviz true|false
  --localization true|false
  --nav2 true|false
  --x VALUE
  --y VALUE
  --z VALUE
  --yaw VALUE
  -h, --help
EOF_USAGE
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world) WORLD_NAME="$2"; shift 2 ;;
    --map) MAP_YAML="$(resolve_project_path "$2")"; shift 2 ;;
    --gui-config) GUI_CONFIG_OVERRIDE="$(resolve_project_path "$2")"; shift 2 ;;
    --rviz) RVIZ="$2"; shift 2 ;;
    --localization) LOCALIZATION="$2"; shift 2 ;;
    --nav2) NAV2="$2"; shift 2 ;;
    --x) ROBOT_X="$2"; shift 2 ;;
    --y) ROBOT_Y="$2"; shift 2 ;;
    --z) ROBOT_Z="$2"; shift 2 ;;
    --yaw) ROBOT_YAW="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${WORLD_NAME}" ]]; then
  echo "Missing TurtleBot world name. Set TURTLEBOT_WORLD in config or pass --world." >&2
  exit 2
fi
require_file "${ROS_SETUP}" "ROS setup file"
require_file "${OVERLAY_SETUP}" "overlay setup file; run scripts/build_overlay.sh first"
if [[ "${LOCALIZATION}" == "true" || "${NAV2}" == "true" ]]; then
  require_file "${MAP_YAML}" "map yaml"
fi
if [[ -n "${GUI_CONFIG_OVERRIDE}" ]]; then
  require_file "${GUI_CONFIG_OVERRIDE}" "Gazebo GUI config"
fi

# Clean up the full launch stack, not just Gazebo itself. A crash can leave
# bridge or spawn helper processes behind, which then block the next launch.
CLEANUP_DONE=0
CLEANUP_PATTERNS=(
  "ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py"
  "gz sim"
  "ros_gz_bridge"
  "ros_gz_sim create"
  "turtlebot4_spawn.launch.py"
  "turtlebot4_nodes.launch.py"
)

cleanup_processes() {
  if [[ "${CLEANUP_DONE}" -eq 1 ]]; then
    return
  fi

  local signal pattern
  for signal in INT TERM KILL; do
    for pattern in "${CLEANUP_PATTERNS[@]}"; do
      pkill "-${signal}" -f "${pattern}" 2>/dev/null || true
    done
    sleep 1
  done

  CLEANUP_DONE=1
}

trap cleanup_processes EXIT INT TERM

set +u
source "${ROS_SETUP}"
source "${OVERLAY_SETUP}"
set -u

export GZ_SIM_RESOURCE_PATH="${MODEL_PATH}"

cleanup_processes
CLEANUP_DONE=0

launch_args=(
  world:="${WORLD_NAME}"
  rviz:="${RVIZ}"
  localization:="${LOCALIZATION}"
  nav2:="${NAV2}"
  map:="${MAP_YAML}"
  x:="${ROBOT_X}"
  y:="${ROBOT_Y}"
  z:="${ROBOT_Z}"
  yaw:="${ROBOT_YAW}"
)
if [[ -n "${GUI_CONFIG_OVERRIDE}" ]]; then
  launch_args+=(gui_config:="${GUI_CONFIG_OVERRIDE}")
fi

ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py "${launch_args[@]}" &
LAUNCH_PID=$!

set +e
wait "${LAUNCH_PID}"
STATUS=$?
set -e

cleanup_processes
exit "${STATUS}"
