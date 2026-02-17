#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_NAME="formant-spot-adapter.service"

cd "${ROOT_DIR}"

echo "[deploy] building adapter"
./scripts/build.sh

if [[ ! -f config/formant-spot-adapter.env ]]; then
  echo "[deploy] creating config/formant-spot-adapter.env from example"
  cp config/formant-spot-adapter.env.example config/formant-spot-adapter.env
fi

if [[ ! -f config/formant-spot-adapter.json ]]; then
  echo "[deploy] creating config/formant-spot-adapter.json from example"
  cp config/formant-spot-adapter.json.example config/formant-spot-adapter.json
fi

echo "[deploy] installing/updating systemd unit"
./scripts/setup_service.sh

echo "[deploy] installing/updating logrotate policy"
./scripts/setup_logrotate.sh

echo "[deploy] enabling and starting service"
sudo systemctl enable --now "${SERVICE_NAME}"

echo "[deploy] restarting service to ensure latest binary is active"
sudo systemctl restart "${SERVICE_NAME}"

echo "[deploy] service status"
sudo systemctl --no-pager --full status "${SERVICE_NAME}" || true
