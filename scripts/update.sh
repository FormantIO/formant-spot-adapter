#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="formant-spot-adapter.service"
PULL_FROM_GIT="${PULL_FROM_GIT:-1}"
ALLOW_DIRTY_TREE="${ALLOW_DIRTY_TREE:-0}"

cd "${ROOT_DIR}"

if [[ "${PULL_FROM_GIT}" == "1" ]]; then
  if [[ "${ALLOW_DIRTY_TREE}" != "1" ]] && [[ -n "$(git status --porcelain)" ]]; then
    echo "[update] working tree is dirty; commit/stash first or set ALLOW_DIRTY_TREE=1" >&2
    exit 1
  fi

  echo "[update] fetching latest changes"
  git fetch --all --prune
  echo "[update] pulling latest commit (ff-only)"
  git pull --ff-only
fi

echo "[update] rebuilding adapter"
./scripts/build.sh

echo "[update] ensuring systemd unit is up to date"
./scripts/setup_service.sh

echo "[update] ensuring logrotate policy is up to date"
./scripts/setup_logrotate.sh

echo "[update] restarting service"
sudo systemctl restart "${SERVICE_NAME}"

echo "[update] service status"
sudo systemctl --no-pager --full status "${SERVICE_NAME}" || true
