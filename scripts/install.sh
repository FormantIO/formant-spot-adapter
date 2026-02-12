#!/usr/bin/env bash
set -euo pipefail

cmake --install build

if [[ ! -f config/formant-spot-adapter.env ]]; then
  cp config/formant-spot-adapter.env.example config/formant-spot-adapter.env
fi

if [[ ! -f config/formant-spot-adapter.json ]]; then
  cp config/formant-spot-adapter.json.example config/formant-spot-adapter.json
fi
