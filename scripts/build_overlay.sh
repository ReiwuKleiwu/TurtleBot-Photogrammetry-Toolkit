#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=scripts/common.sh
source "${SCRIPT_DIR}/common.sh"
load_project_config

require_file "${ROS_SETUP}" "ROS setup file"

set +u
source "${ROS_SETUP}"
set -u

cd "${REPO_ROOT}/tb4_overlay_ws"
colcon build --symlink-install
