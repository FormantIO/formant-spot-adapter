#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

export PULL_FROM_GIT="${PULL_FROM_GIT:-1}"
export ALLOW_DIRTY_TREE="${ALLOW_DIRTY_TREE:-0}"

exec ./scripts/docker_deploy.sh "$@"
