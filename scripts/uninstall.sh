#!/usr/bin/env bash
set -euo pipefail

SERVICE_NAME="formant-spot-adapter.service"
SERVICE_PATH="/etc/systemd/system/${SERVICE_NAME}"
LOGROTATE_PATH="/etc/logrotate.d/formant-spot-adapter"

echo "[uninstall] disabling and stopping ${SERVICE_NAME}"
sudo systemctl disable --now "${SERVICE_NAME}" || true

if [[ -f "${SERVICE_PATH}" ]]; then
  echo "[uninstall] removing ${SERVICE_PATH}"
  sudo rm -f "${SERVICE_PATH}"
fi

if [[ -f "${LOGROTATE_PATH}" ]]; then
  echo "[uninstall] removing ${LOGROTATE_PATH}"
  sudo rm -f "${LOGROTATE_PATH}"
fi

echo "[uninstall] reloading systemd"
sudo systemctl daemon-reload

echo "[uninstall] done"
