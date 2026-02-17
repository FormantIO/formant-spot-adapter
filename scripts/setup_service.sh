#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SERVICE_PATH="/etc/systemd/system/formant-spot-adapter.service"
RUN_AS_USER="${RUN_AS_USER:-${SUDO_USER:-$USER}}"
RUN_AS_GROUP="${RUN_AS_GROUP:-${RUN_AS_USER}}"
ENV_FILE="${ENV_FILE:-${ROOT_DIR}/config/formant-spot-adapter.env}"
CONFIG_PATH="${CONFIG_PATH:-${ROOT_DIR}/config/formant-spot-adapter.json}"

sudo tee "${SERVICE_PATH}" >/dev/null <<EOF
[Unit]
Description=Formant Spot Adapter
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
WorkingDirectory=${ROOT_DIR}
ExecStart=${ROOT_DIR}/scripts/run.sh
Restart=always
RestartSec=3
User=${RUN_AS_USER}
Group=${RUN_AS_GROUP}
Environment=ENV_FILE=${ENV_FILE}
Environment=CONFIG_PATH=${CONFIG_PATH}
NoNewPrivileges=true
PrivateTmp=true
ProtectSystem=full
ProtectHome=true
ReadWritePaths=${ROOT_DIR}/logs ${ROOT_DIR}/data
UMask=0077
TimeoutStopSec=20

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
