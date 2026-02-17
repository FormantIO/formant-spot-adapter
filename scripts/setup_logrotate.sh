#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGROTATE_PATH="/etc/logrotate.d/formant-spot-adapter"
RUN_AS_USER="${RUN_AS_USER:-${SUDO_USER:-$USER}}"
RUN_AS_GROUP="${RUN_AS_GROUP:-${RUN_AS_USER}}"

if ! command -v logrotate >/dev/null 2>&1; then
  echo "logrotate is not installed. Install it first (e.g. sudo apt-get install -y logrotate)." >&2
  exit 1
fi

sudo tee "${LOGROTATE_PATH}" >/dev/null <<EOF2
${ROOT_DIR}/logs/*.log {
  daily
  rotate 14
  missingok
  notifempty
  compress
  delaycompress
  copytruncate
  su ${RUN_AS_USER} ${RUN_AS_GROUP}
  create 0640 ${RUN_AS_USER} ${RUN_AS_GROUP}
  dateext
  maxsize 50M
}
EOF2

echo "[logrotate] installed ${LOGROTATE_PATH}"
