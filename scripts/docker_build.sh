#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# shellcheck source=/dev/null
source "${ROOT_DIR}/scripts/docker_common.sh"

IMAGE_NAME="$(docker_image_name)"

docker_cmd build -t "${IMAGE_NAME}" -f "${ROOT_DIR}/Dockerfile" "$@" "${ROOT_DIR}"
