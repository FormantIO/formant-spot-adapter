#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="formant-spot-adapter.service"
ENV_FILE="${ENV_FILE:-${ROOT_DIR}/config/formant-spot-adapter.env}"
ENV_EXAMPLE="${ROOT_DIR}/config/formant-spot-adapter.env.example"
CONFIG_PATH="${CONFIG_PATH:-${ROOT_DIR}/config/formant-spot-adapter.json}"
CONFIG_EXAMPLE="${ROOT_DIR}/config/formant-spot-adapter.json.example"
PULL_FROM_GIT="${PULL_FROM_GIT:-0}"
ALLOW_DIRTY_TREE="${ALLOW_DIRTY_TREE:-0}"

ensure_local_file() {
  local example_path="$1"
  local target_path="$2"
  local label="$3"

  if [[ -f "${target_path}" ]]; then
    return 0
  fi
  if [[ ! -f "${example_path}" ]]; then
    echo "[deploy] missing ${label}: ${target_path}" >&2
    echo "[deploy] also missing example file: ${example_path}" >&2
    exit 1
  fi

  mkdir -p "$(dirname "${target_path}")"
  cp "${example_path}" "${target_path}"
  echo "[deploy] created ${label} from example: ${target_path}"
}

has_placeholder_credentials() {
  [[ -z "${SPOT_USERNAME:-}" || -z "${SPOT_PASSWORD:-}" || "${SPOT_USERNAME}" == "user" || "${SPOT_PASSWORD}" == "changeme" ]]
}

cd "${ROOT_DIR}"

if [[ "${PULL_FROM_GIT}" == "1" ]]; then
  if [[ "${ALLOW_DIRTY_TREE}" != "1" ]] && [[ -n "$(git status --porcelain)" ]]; then
    echo "[deploy] working tree is dirty; commit/stash first or set ALLOW_DIRTY_TREE=1" >&2
    exit 1
  fi

  echo "[deploy] fetching latest changes"
  git fetch --all --prune
  echo "[deploy] pulling latest commit (ff-only)"
  git pull --ff-only
fi

echo "[deploy] building adapter"
./scripts/build.sh

ensure_local_file "${ENV_EXAMPLE}" "${ENV_FILE}" "env file"
ensure_local_file "${CONFIG_EXAMPLE}" "${CONFIG_PATH}" "config JSON"

shell_spot_username="${SPOT_USERNAME:-}"
shell_spot_password="${SPOT_PASSWORD:-}"

if [[ -f "${ENV_FILE}" && -r "${ENV_FILE}" ]]; then
  # shellcheck source=/dev/null
  set -a
  source "${ENV_FILE}"
  set +a
else
  echo "[deploy] env file is not readable: ${ENV_FILE}" >&2
  exit 1
fi

if [[ -n "${shell_spot_username}" ]]; then
  export SPOT_USERNAME="${shell_spot_username}"
fi
if [[ -n "${shell_spot_password}" ]]; then
  export SPOT_PASSWORD="${shell_spot_password}"
fi

if has_placeholder_credentials; then
  echo "[deploy] edit ${ENV_FILE} and set SPOT_USERNAME/SPOT_PASSWORD before deploying." >&2
  echo "[deploy] rerun ./scripts/deploy.sh after updating credentials." >&2
  exit 1
fi

echo "[deploy] installing/updating systemd unit"
ENV_FILE="${ENV_FILE}" CONFIG_PATH="${CONFIG_PATH}" ./scripts/setup_service.sh

echo "[deploy] installing/updating logrotate policy"
./scripts/setup_logrotate.sh

echo "[deploy] enabling service"
sudo systemctl enable "${SERVICE_NAME}"

if sudo systemctl is-active --quiet "${SERVICE_NAME}"; then
  echo "[deploy] restarting service"
  sudo systemctl restart "${SERVICE_NAME}"
else
  echo "[deploy] starting service"
  sudo systemctl start "${SERVICE_NAME}"
fi

echo "[deploy] service status"
sudo systemctl --no-pager --full status "${SERVICE_NAME}" || true
