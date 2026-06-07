#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
INSTALLED_SHARE="${REPO_ROOT}/tb4_overlay_ws/install/turtlebot4_gz_bringup/share/turtlebot4_gz_bringup"
GUI_CONFIG="tb4_overlay_ws/src/turtlebot4_gz_bringup/gui/dark/gui.config"

if [[ ! -f "${INSTALLED_SHARE}/worlds/small_house_dark.sdf" ]]; then
  echo "Dark small_house world is not installed yet." >&2
  echo "Run: bash scripts/build_overlay.sh" >&2
  exit 1
fi

exec "${SCRIPT_DIR}/run_turtlebot_world.sh" \
  --world small_house_dark \
  --gui-config "${GUI_CONFIG}" \
  "$@"
