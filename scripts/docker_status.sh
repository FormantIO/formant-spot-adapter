#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# shellcheck source=/dev/null
source "${ROOT_DIR}/scripts/docker_common.sh"

CONTAINER_NAME="$(docker_container_name)"

if ! docker_cmd container inspect "${CONTAINER_NAME}" >/dev/null 2>&1; then
  echo "[docker-status] container not found: ${CONTAINER_NAME}" >&2
  exit 1
fi

docker_cmd ps -a \
  --filter "name=^/${CONTAINER_NAME}$" \
  --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"

docker_cmd inspect -f \
  'NetworkMode={{.HostConfig.NetworkMode}} RestartPolicy={{.HostConfig.RestartPolicy.Name}}' \
  "${CONTAINER_NAME}"
