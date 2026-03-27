#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

SPOT_CPP_SDK_DIR="${SPOT_CPP_SDK_DIR:-${ROOT_DIR}/third_party/spot-cpp-sdk}"
SPOT_SDK_REF="${SPOT_SDK_REF:-v5.1.0}"
SPOT_SDK_VERSION="${SPOT_SDK_VERSION:-${SPOT_SDK_REF#v}}"
GRPC_SHIM_DIR="${GRPC_SHIM_DIR:-${ROOT_DIR}/cmake/grpc-shim}"
EIGEN3_SHIM_DIR="${EIGEN3_SHIM_DIR:-${ROOT_DIR}/cmake/eigen3-shim}"
SPOT_SDK_CMAKE_FILE="${SPOT_CPP_SDK_DIR}/cpp/CMakeLists.txt"
ENABLE_CCACHE="${ENABLE_CCACHE:-1}"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build}"
BUILD_JOBS="${BUILD_JOBS:-}"

./scripts/ensure_spot_sdk.sh

SPOT_CPP_SDK_DIR="${SPOT_CPP_SDK_DIR}" SPOT_SDK_VERSION="${SPOT_SDK_VERSION}" python3 "${ROOT_DIR}/scripts/prepare_spot_sdk.py"

declare -a cmake_args
cmake_args=(
  -S "${ROOT_DIR}" -B "${BUILD_DIR}" -G Ninja
  -DSPOT_CPP_SDK_DIR="${SPOT_CPP_SDK_DIR}"
  -DgRPC_DIR="${GRPC_SHIM_DIR}"
  -DEigen3_DIR="${EIGEN3_SHIM_DIR}"
)

if [[ "${ENABLE_CCACHE}" == "1" ]] && command -v ccache >/dev/null 2>&1; then
  cmake_args+=(
    -DCMAKE_C_COMPILER_LAUNCHER=ccache
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache
  )
fi

if [[ -z "${BUILD_JOBS}" ]]; then
  cpu_count="$(getconf _NPROCESSORS_ONLN 2>/dev/null || nproc 2>/dev/null || echo 2)"
  if [[ "${cpu_count}" -le 4 ]]; then
    BUILD_JOBS="${cpu_count}"
  else
    BUILD_JOBS="$((cpu_count - 2))"
  fi
fi

cmake "${cmake_args[@]}"

PATH="${ROOT_DIR}/scripts:${PATH}" cmake --build "${BUILD_DIR}" --target formant-spot-adapter -j "${BUILD_JOBS}"
