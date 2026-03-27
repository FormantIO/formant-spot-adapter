#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT_DIR}/build/formant-spot-adapter"
ENV_FILE="${ENV_FILE:-${ROOT_DIR}/config/formant-spot-adapter.env}"
ENV_EXAMPLE="${ROOT_DIR}/config/formant-spot-adapter.env.example"
DEFAULT_CONFIG="${ROOT_DIR}/config/formant-spot-adapter.json"
CONFIG_EXAMPLE="${ROOT_DIR}/config/formant-spot-adapter.json.example"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/logs}"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/adapter-$(date +%Y%m%d-%H%M%S).log}"

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
  echo "[run] created ${label} from example: ${target_path}"
}

has_placeholder_credentials() {
  [[ -z "${SPOT_USERNAME:-}" || -z "${SPOT_PASSWORD:-}" || "${SPOT_USERNAME}" == "user" || "${SPOT_PASSWORD}" == "changeme" ]]
}

if [[ ! -x "${BIN}" ]]; then
  echo "Missing binary: ${BIN}" >&2
  echo "Build first with: ./scripts/build.sh" >&2
  exit 1
fi

export CONFIG_PATH="${CONFIG_PATH:-${DEFAULT_CONFIG}}"

ensure_local_file "${ENV_EXAMPLE}" "${ENV_FILE}" "env file"
ensure_local_file "${CONFIG_EXAMPLE}" "${CONFIG_PATH}" "config JSON"

shell_spot_username="${SPOT_USERNAME:-}"
shell_spot_password="${SPOT_PASSWORD:-}"

if [[ -f "${ENV_FILE}" && -r "${ENV_FILE}" ]]; then
  # shellcheck source=/dev/null
  set -a
  source "${ENV_FILE}"
  set +a
elif [[ -f "${ENV_FILE}" ]]; then
  echo "Warning: env file exists but is not readable: ${ENV_FILE}" >&2
  echo "Set credentials in shell env or fix file permissions." >&2
fi

if [[ -n "${shell_spot_username}" ]]; then
  export SPOT_USERNAME="${shell_spot_username}"
fi
if [[ -n "${shell_spot_password}" ]]; then
  export SPOT_PASSWORD="${shell_spot_password}"
fi

if has_placeholder_credentials; then
  echo "Edit ${ENV_FILE} and set SPOT_USERNAME/SPOT_PASSWORD before running." >&2
  echo "The file was created from ${ENV_EXAMPLE} if it did not already exist." >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"
echo "[run] adapter binary: ${BIN}"
echo "[run] env file: ${ENV_FILE}"
echo "[run] config path: ${CONFIG_PATH}"
echo "[run] log file: ${LOG_FILE}"
echo "[run] starting..."

exec "${BIN}" "$@" 2>&1 | tee -a "${LOG_FILE}"
