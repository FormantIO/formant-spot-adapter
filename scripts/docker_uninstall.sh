#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# shellcheck source=/dev/null
source "${ROOT_DIR}/scripts/docker_common.sh"

IMAGE_NAME="$(docker_image_name)"
CONTAINER_NAME="$(docker_container_name)"
REMOVE_IMAGE="${REMOVE_IMAGE:-0}"

echo "[docker-uninstall] stopping and removing container ${CONTAINER_NAME}"
"${ROOT_DIR}/scripts/docker_down.sh"

if [[ "${REMOVE_IMAGE}" == "1" ]]; then
  echo "[docker-uninstall] removing image ${IMAGE_NAME}"
  docker_cmd rmi "${IMAGE_NAME}" >/dev/null 2>&1 || true
fi

echo "[docker-uninstall] done"
