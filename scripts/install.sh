#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

if [[ ! -d build ]]; then
  echo "[install] missing build directory: ${ROOT_DIR}/build" >&2
  echo "[install] build first with: ./scripts/build.sh" >&2
  exit 1
fi

cmake --install build

echo "[install] installed the binary and example config artifacts."
echo "[install] use ./scripts/deploy.sh for the supported systemd deployment path for this checkout."
