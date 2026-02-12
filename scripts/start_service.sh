#!/usr/bin/env bash
set -euo pipefail

sudo systemctl enable --now formant-spot-adapter.service
sudo systemctl status formant-spot-adapter.service --no-pager
