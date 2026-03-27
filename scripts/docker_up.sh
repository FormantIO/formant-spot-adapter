#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# shellcheck source=/dev/null
source "${ROOT_DIR}/scripts/docker_common.sh"

ENV_FILE="${ENV_FILE:-${ROOT_DIR}/config/formant-spot-adapter.env}"
ENV_EXAMPLE="${ROOT_DIR}/config/formant-spot-adapter.env.example"
CONFIG_PATH="${CONFIG_PATH:-${ROOT_DIR}/config/formant-spot-adapter.json}"
CONFIG_EXAMPLE="${ROOT_DIR}/config/formant-spot-adapter.json.example"
IMAGE_NAME="$(docker_image_name)"
CONTAINER_NAME="$(docker_container_name)"

ensure_local_file() {
  local example_path="$1"
  local target_path="$2"
  local label="$3"

  if [[ -f "${target_path}" ]]; then
    return 0
  fi
  if [[ ! -f "${example_path}" ]]; then
    echo "[docker-up] missing ${label}: ${target_path}" >&2
    echo "[docker-up] also missing example file: ${example_path}" >&2
    exit 1
  fi

  mkdir -p "$(dirname "${target_path}")"
  cp "${example_path}" "${target_path}"
  echo "[docker-up] created ${label} from example: ${target_path}"
}

has_placeholder_credentials() {
  [[ -z "${SPOT_USERNAME:-}" || -z "${SPOT_PASSWORD:-}" || "${SPOT_USERNAME}" == "user" || "${SPOT_PASSWORD}" == "changeme" ]]
}

ensure_local_file "${ENV_EXAMPLE}" "${ENV_FILE}" "env file"
ensure_local_file "${CONFIG_EXAMPLE}" "${CONFIG_PATH}" "config JSON"
mkdir -p "${ROOT_DIR}/data"

shell_spot_username="${SPOT_USERNAME:-}"
shell_spot_password="${SPOT_PASSWORD:-}"

if [[ -f "${ENV_FILE}" && -r "${ENV_FILE}" ]]; then
  # shellcheck source=/dev/null
  set -a
  source "${ENV_FILE}"
  set +a
else
  echo "[docker-up] env file is not readable: ${ENV_FILE}" >&2
  exit 1
fi

if [[ -n "${shell_spot_username}" ]]; then
  export SPOT_USERNAME="${shell_spot_username}"
fi
if [[ -n "${shell_spot_password}" ]]; then
  export SPOT_PASSWORD="${shell_spot_password}"
fi

if has_placeholder_credentials; then
  echo "[docker-up] edit ${ENV_FILE} and set SPOT_USERNAME/SPOT_PASSWORD before starting the container." >&2
  exit 1
fi

if [[ "${1:-}" == "--build" ]]; then
  shift
  "${ROOT_DIR}/scripts/docker_build.sh"
fi

if ! docker_cmd image inspect "${IMAGE_NAME}" >/dev/null 2>&1; then
  echo "[docker-up] image not found: ${IMAGE_NAME}" >&2
  echo "[docker-up] build it first with ./scripts/docker_build.sh or rerun with --build." >&2
  exit 1
fi

docker_cmd stop "${CONTAINER_NAME}" >/dev/null 2>&1 || true
docker_cmd rm "${CONTAINER_NAME}" >/dev/null 2>&1 || true

docker_cmd run -d \
  --init \
  --network host \
  --restart unless-stopped \
  --name "${CONTAINER_NAME}" \
  --env-file "${ENV_FILE}" \
  -e CONFIG_PATH=/opt/formant-spot-adapter/config/formant-spot-adapter.json \
  -v "${ROOT_DIR}/config:/opt/formant-spot-adapter/config" \
  -v "${ROOT_DIR}/data:/opt/formant-spot-adapter/data" \
  "${IMAGE_NAME}" \
  "$@"
