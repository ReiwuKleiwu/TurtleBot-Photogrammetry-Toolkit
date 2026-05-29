#!/usr/bin/env bash

repo_root() {
  cd "$(dirname "${BASH_SOURCE[1]}")/.." && pwd
}

load_project_config() {
  REPO_ROOT="$(repo_root)"
  CONFIG_FILE="${CONFIG_FILE:-${REPO_ROOT}/config/project.env}"

  if [[ -f "${CONFIG_FILE}" ]]; then
    # shellcheck source=/dev/null
    source "${CONFIG_FILE}"
  fi

  ROS_SETUP="${ROS_SETUP:-}"
  OVERLAY_SETUP="${OVERLAY_SETUP:-}"
  MAP_YAML="${MAP_YAML:-}"
  WORLD="${WORLD:-}"
  TURTLEBOT_WORLD="${TURTLEBOT_WORLD:-}"
  GUI_CONFIG="${GUI_CONFIG:-}"
  MODEL_PATH="${MODEL_PATH:-}"
  PLUGIN_PATH="${PLUGIN_PATH:-}"

  ROS_SETUP="$(resolve_project_path "${ROS_SETUP}")"
  OVERLAY_SETUP="$(resolve_project_path "${OVERLAY_SETUP}")"
  MAP_YAML="$(resolve_project_path "${MAP_YAML}")"
  WORLD="$(resolve_project_path "${WORLD}")"
  GUI_CONFIG="$(resolve_project_path "${GUI_CONFIG}")"
  MODEL_PATH="$(resolve_project_path "${MODEL_PATH}")"
  PLUGIN_PATH="$(resolve_project_path "${PLUGIN_PATH}")"
}

resolve_project_path() {
  local value="$1"
  if [[ -z "${value}" ]]; then
    printf '%s\n' ""
  elif [[ "${value}" = /* ]]; then
    printf '%s\n' "${value}"
  else
    printf '%s\n' "${REPO_ROOT}/${value}"
  fi
}

require_file() {
  local path="$1"
  local label="$2"
  if [[ ! -f "${path}" ]]; then
    echo "Missing ${label}: ${path}" >&2
    return 1
  fi
}

require_dir() {
  local path="$1"
  local label="$2"
  if [[ ! -d "${path}" ]]; then
    echo "Missing ${label}: ${path}" >&2
    return 1
  fi
}

resolve_world_arg() {
  local value="$1"
  if [[ "${value}" == */* || "${value}" == *.sdf ]]; then
    resolve_project_path "${value}"
  else
    printf '%s\n' "${value}"
  fi
}
