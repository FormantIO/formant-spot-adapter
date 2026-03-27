#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY_SCRIPT="${ROOT_DIR}/scripts/camera_calibration.py"
ENV_FILE="${ENV_FILE:-${ROOT_DIR}/config/formant-spot-adapter.env}"
ENV_EXAMPLE="${ROOT_DIR}/config/formant-spot-adapter.env.example"
DEFAULT_CONFIG="${ROOT_DIR}/config/formant-spot-adapter.json"
CONFIG_EXAMPLE="${ROOT_DIR}/config/formant-spot-adapter.json.example"
PYTHON_BIN="${PYTHON_BIN:-}"

ensure_local_file() {
  local example_path="$1"
  local target_path="$2"
  local label="$3"

  if [[ -f "${target_path}" ]]; then
    return 0
  fi
  if [[ ! -f "${example_path}" ]]; then
    echo "Missing ${label}: ${target_path}" >&2
    echo "Also missing example file: ${example_path}" >&2
    exit 1
  fi

  mkdir -p "$(dirname "${target_path}")"
  cp "${example_path}" "${target_path}"
  echo "[camera_calibration] created ${label} from example: ${target_path}"
}

has_placeholder_credentials() {
  [[ -z "${SPOT_USERNAME:-}" || -z "${SPOT_PASSWORD:-}" || "${SPOT_USERNAME}" == "user" || "${SPOT_PASSWORD}" == "changeme" ]]
}

if [[ -z "${PYTHON_BIN}" ]]; then
  if [[ -x "${ROOT_DIR}/.venv/bin/python3" ]]; then
    PYTHON_BIN="${ROOT_DIR}/.venv/bin/python3"
  else
    PYTHON_BIN="python3"
  fi
fi

if ! "${PYTHON_BIN}" - <<'PY' >/dev/null 2>&1
import bosdyn.client  # noqa: F401
PY
then
  PYTHON_BIN="python3"
fi

if [[ ! -f "${PY_SCRIPT}" ]]; then
  echo "Missing script: ${PY_SCRIPT}" >&2
  exit 1
fi

export CONFIG_PATH="${CONFIG_PATH:-${DEFAULT_CONFIG}}"

ensure_local_file "${ENV_EXAMPLE}" "${ENV_FILE}" "env file"
ensure_local_file "${CONFIG_EXAMPLE}" "${CONFIG_PATH}" "config JSON"

shell_spot_username="${SPOT_USERNAME:-}"
shell_spot_password="${SPOT_PASSWORD:-}"

if [[ -f "${ENV_FILE}" && -r "${ENV_FILE}" ]]; then
  set -a
  # shellcheck source=/dev/null
  source "${ENV_FILE}"
  set +a
elif [[ -f "${ENV_FILE}" ]]; then
  echo "Warning: env file exists but is not readable: ${ENV_FILE}" >&2
fi

if [[ -n "${shell_spot_username}" ]]; then
  export SPOT_USERNAME="${shell_spot_username}"
fi
if [[ -n "${shell_spot_password}" ]]; then
  export SPOT_PASSWORD="${shell_spot_password}"
fi

if [[ -z "${SPOT_HOST:-}" && -f "${CONFIG_PATH}" ]]; then
  SPOT_HOST_FROM_JSON="$("${PYTHON_BIN}" - <<'PY'
import json
import os
path = os.environ.get("CONFIG_PATH", "")
try:
    with open(path, "r", encoding="utf-8") as f:
        j = json.load(f)
    print(j.get("spotHost", ""))
except Exception:
    print("")
PY
)"
  if [[ -n "${SPOT_HOST_FROM_JSON}" ]]; then
    export SPOT_HOST="${SPOT_HOST_FROM_JSON}"
  fi
fi

if [[ -z "${SPOT_HOST:-}" ]]; then
  echo "Missing SPOT_HOST (set env var or config json spotHost)." >&2
  exit 1
fi
if has_placeholder_credentials; then
  echo "Edit ${ENV_FILE} and set SPOT_USERNAME/SPOT_PASSWORD before running camera_calibration." >&2
  exit 1
fi

exec "${PYTHON_BIN}" "${PY_SCRIPT}" "$@"
