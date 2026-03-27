#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  ninja-build \
  ccache \
  pkg-config \
  git \
  curl \
  unzip \
  libopencv-dev \
  libeigen3-dev \
  libprotobuf-dev \
  protobuf-compiler \
  protobuf-compiler-grpc \
  libgrpc++-dev

# Optional on some distros (missing on Ubuntu 20.04 arm64 repos).
sudo apt-get install -y libcli11-dev || true

./scripts/ensure_spot_sdk.sh
