#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# shellcheck source=/dev/null
source "${ROOT_DIR}/scripts/docker_common.sh"

LINES="${LINES:-200}"
CONTAINER_NAME="$(docker_container_name)"

docker_cmd logs -f --tail "${LINES}" "$@" "${CONTAINER_NAME}"
