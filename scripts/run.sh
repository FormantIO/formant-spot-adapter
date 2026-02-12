#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${ROOT_DIR}/build/formant-spot-adapter"
ENV_FILE="${ENV_FILE:-${ROOT_DIR}/config/formant-spot-adapter.env}"
DEFAULT_CONFIG="${ROOT_DIR}/config/formant-spot-adapter.json"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/logs}"
LOG_FILE="${LOG_FILE:-${LOG_DIR}/adapter-$(date +%Y%m%d-%H%M%S).log}"

if [[ ! -x "${BIN}" ]]; then
  echo "Missing binary: ${BIN}" >&2
  echo "Build first with: ./scripts/build.sh" >&2
  exit 1
fi

# Load environment variables from env file if present and readable.
if [[ -f "${ENV_FILE}" && -r "${ENV_FILE}" ]]; then
  # shellcheck source=/dev/null
  set -a
  source "${ENV_FILE}"
  set +a
elif [[ -f "${ENV_FILE}" ]]; then
  echo "Warning: env file exists but is not readable: ${ENV_FILE}" >&2
  echo "Set credentials in shell env or fix file permissions." >&2
fi

# Default config path if not supplied by env.
export CONFIG_PATH="${CONFIG_PATH:-${DEFAULT_CONFIG}}"

if [[ ! -f "${CONFIG_PATH}" ]]; then
  echo "Missing config JSON: ${CONFIG_PATH}" >&2
  echo "Expected ${ROOT_DIR}/config/formant-spot-adapter.json or set CONFIG_PATH." >&2
  exit 1
fi

if [[ -z "${SPOT_USERNAME:-}" || -z "${SPOT_PASSWORD:-}" ]]; then
  echo "Missing SPOT_USERNAME/SPOT_PASSWORD." >&2
  echo "Set them in ${ENV_FILE} or current shell env." >&2
  exit 1
fi

mkdir -p "${LOG_DIR}"
echo "[run] adapter binary: ${BIN}"
echo "[run] env file: ${ENV_FILE}"
echo "[run] config path: ${CONFIG_PATH}"
echo "[run] log file: ${LOG_FILE}"
echo "[run] starting..."

exec "${BIN}" "$@" 2>&1 | tee -a "${LOG_FILE}"
