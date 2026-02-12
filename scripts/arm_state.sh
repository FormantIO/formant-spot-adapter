#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY_SCRIPT="${ROOT_DIR}/scripts/print_arm_state.py"
ENV_FILE="${ENV_FILE:-${ROOT_DIR}/config/formant-spot-adapter.env}"
DEFAULT_CONFIG="${ROOT_DIR}/config/formant-spot-adapter.json"
PYTHON_BIN="${PYTHON_BIN:-}"

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

# Load secrets/env overrides.
if [[ -f "${ENV_FILE}" && -r "${ENV_FILE}" ]]; then
  set -a
  # shellcheck source=/dev/null
  source "${ENV_FILE}"
  set +a
elif [[ -f "${ENV_FILE}" ]]; then
  echo "Warning: env file exists but is not readable: ${ENV_FILE}" >&2
fi

export CONFIG_PATH="${CONFIG_PATH:-${DEFAULT_CONFIG}}"

# Fill SPOT_HOST from config json if not provided.
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
if [[ -z "${SPOT_USERNAME:-}" || -z "${SPOT_PASSWORD:-}" ]]; then
  echo "Missing SPOT_USERNAME/SPOT_PASSWORD (set in env file or shell env)." >&2
  exit 1
fi

exec "${PYTHON_BIN}" "${PY_SCRIPT}" "$@"
