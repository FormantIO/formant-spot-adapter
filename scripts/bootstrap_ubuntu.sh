#!/usr/bin/env bash
set -euo pipefail

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

mkdir -p third_party

if [[ ! -d third_party/spot-cpp-sdk ]]; then
  if [[ -f bosdyn/spot-cpp-sdk-5.1.0.zip ]]; then
    echo "Using local bosdyn/spot-cpp-sdk-5.1.0.zip"
    unzip -q bosdyn/spot-cpp-sdk-5.1.0.zip -d third_party
    mv third_party/spot-cpp-sdk-5.1.0 third_party/spot-cpp-sdk
  else
    echo "Cloning spot-cpp-sdk..."
    git clone --depth 1 https://github.com/boston-dynamics/spot-cpp-sdk.git third_party/spot-cpp-sdk
  fi
fi
