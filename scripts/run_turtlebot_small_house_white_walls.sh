#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INSTALLED_SHARE="${REPO_ROOT}/tb4_overlay_ws/install/turtlebot4_gz_bringup/share/turtlebot4_gz_bringup"

if [[ ! -f "${INSTALLED_SHARE}/worlds/small_house_white_walls.sdf" ||
      ! -d "${INSTALLED_SHARE}/models/aws_robomaker_residential_RoomWall_01_white" ||
      ! -d "${INSTALLED_SHARE}/models/aws_robomaker_residential_HouseWallB_01_white" ]]; then
  echo "White-wall small_house assets are not installed yet." >&2
  echo "Run: bash scripts/build_overlay.sh" >&2
  exit 1
fi

exec "${SCRIPT_DIR}/run_turtlebot_world.sh" --world small_house_white_walls "$@"
