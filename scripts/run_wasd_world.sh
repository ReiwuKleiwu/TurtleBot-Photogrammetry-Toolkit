#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/common.sh
source "${SCRIPT_DIR}/common.sh"
load_project_config

usage() {
  cat <<'EOF'
Usage: run_wasd_world.sh [options]

Options:
  --world NAME_OR_PATH      World name or SDF path. Relative paths resolve from repo root.
                            Names resolve under tb4_overlay_ws/src/turtlebot4_gz_bringup/worlds.
  --gui-config PATH         Gazebo GUI config. Relative paths resolve from repo root.
  --model-path PATH         Gazebo model resource path. Relative paths resolve from repo root.
  --plugin-path PATH        Gazebo GUI plugin path. Relative paths resolve from repo root.
  -h, --help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --world) WORLD="$(resolve_world_arg "$2")"; shift 2 ;;
    --gui-config) GUI_CONFIG="$(resolve_project_path "$2")"; shift 2 ;;
    --model-path) MODEL_PATH="$(resolve_project_path "$2")"; shift 2 ;;
    --plugin-path) PLUGIN_PATH="$(resolve_project_path "$2")"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ "${WORLD}" != /* && "${WORLD}" != *.sdf ]]; then
  WORLD="${REPO_ROOT}/tb4_overlay_ws/src/turtlebot4_gz_bringup/worlds/${WORLD}.sdf"
fi

require_file "${ROS_SETUP}" "ROS setup file"
require_file "${OVERLAY_SETUP}" "overlay setup file; run scripts/build_overlay.sh first"
require_file "${WORLD}" "world file"
require_file "${GUI_CONFIG}" "Gazebo GUI config"
require_dir "${MODEL_PATH}" "model path"
require_dir "${PLUGIN_PATH}" "GUI plugin path"

# Gazebo can leave helper processes behind after an abnormal exit, so always
# drain the previous stack before starting a fresh session.
CLEANUP_DONE=0
CLEANUP_PATTERNS=(
  "gz sim"
  "ros_gz_bridge"
  "ros_gz_sim create"
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

export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE-}"
export COLCON_CURRENT_PREFIX="${COLCON_CURRENT_PREFIX-}"
export COLCON_TRACE="${COLCON_TRACE-}"

set +u
source "${ROS_SETUP}"
source "${OVERLAY_SETUP}"
set -u

export GZ_SIM_RESOURCE_PATH="$MODEL_PATH"
export GZ_GUI_PLUGIN_PATH="$PLUGIN_PATH"

cleanup_processes
CLEANUP_DONE=0

gz sim "${WORLD}" -r --gui-config "${GUI_CONFIG}" &
SIM_PID=$!

set +e
wait "${SIM_PID}"
STATUS=$?
set -e

cleanup_processes
exit "${STATUS}"
