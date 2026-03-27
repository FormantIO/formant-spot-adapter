#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# shellcheck source=/dev/null
source "${ROOT_DIR}/scripts/docker_common.sh"

CONTAINER_NAME="$(docker_container_name)"

docker_cmd stop "${CONTAINER_NAME}" >/dev/null 2>&1 || true
docker_cmd rm "${CONTAINER_NAME}" "$@" >/dev/null 2>&1 || true
