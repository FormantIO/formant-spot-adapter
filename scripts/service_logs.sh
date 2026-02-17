#!/usr/bin/env bash
set -euo pipefail

LINES="${LINES:-200}"
sudo journalctl -u formant-spot-adapter.service -n "${LINES}" -f
