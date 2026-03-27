#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

# shellcheck source=/dev/null
source "${ROOT_DIR}/scripts/docker_common.sh"

PULL_FROM_GIT="${PULL_FROM_GIT:-0}"
ALLOW_DIRTY_TREE="${ALLOW_DIRTY_TREE:-0}"
SERVICE_NAME="$(host_service_name)"

if [[ "${PULL_FROM_GIT}" == "1" ]]; then
  if [[ "${ALLOW_DIRTY_TREE}" != "1" ]] && [[ -n "$(git status --porcelain)" ]]; then
    echo "[docker-deploy] working tree is dirty; commit/stash first or set ALLOW_DIRTY_TREE=1" >&2
    exit 1
  fi

  echo "[docker-deploy] fetching latest changes"
  git fetch --all --prune
  echo "[docker-deploy] pulling latest commit (ff-only)"
  git pull --ff-only
fi

echo "[docker-deploy] disabling host service ${SERVICE_NAME} to avoid dual adapter instances"
sudo systemctl disable --now "${SERVICE_NAME}" >/dev/null 2>&1 || true

echo "[docker-deploy] building adapter image"
"${ROOT_DIR}/scripts/docker_build.sh"

echo "[docker-deploy] starting adapter container"
"${ROOT_DIR}/scripts/docker_up.sh" "$@"

echo "[docker-deploy] container status"
"${ROOT_DIR}/scripts/docker_status.sh" || true
