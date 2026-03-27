#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SPOT_CPP_SDK_DIR="${SPOT_CPP_SDK_DIR:-${ROOT_DIR}/third_party/spot-cpp-sdk}"
SPOT_SDK_REPO="${SPOT_SDK_REPO:-https://github.com/boston-dynamics/spot-cpp-sdk.git}"
SPOT_SDK_REF="${SPOT_SDK_REF:-v5.1.0}"
SPOT_SDK_VERSION="${SPOT_SDK_VERSION:-${SPOT_SDK_REF#v}}"
SPOT_SDK_ARCHIVE="${SPOT_SDK_ARCHIVE:-${ROOT_DIR}/bosdyn/spot-cpp-sdk-${SPOT_SDK_VERSION}.zip}"
FORCE_REINSTALL_SPOT_SDK="${FORCE_REINSTALL_SPOT_SDK:-0}"

detect_sdk_version() {
  local sdk_dir="$1"
  local readme_path="${sdk_dir}/README.md"

  if [[ ! -f "${readme_path}" ]]; then
    return 1
  fi

  sed -n 's/^This is version \([0-9.][0-9.]*\) of the C++ SDK\..*$/\1/p' "${readme_path}" | head -n 1
}

remove_existing_sdk() {
  local sdk_dir="$1"

  if [[ "${FORCE_REINSTALL_SPOT_SDK}" != "1" ]]; then
    return 1
  fi

  echo "[spot-sdk] removing existing SDK checkout at ${sdk_dir}"
  rm -rf "${sdk_dir}"
}

validate_existing_sdk() {
  local sdk_dir="$1"
  local detected_version=""

  if [[ ! -d "${sdk_dir}" ]]; then
    return 1
  fi

  detected_version="$(detect_sdk_version "${sdk_dir}" || true)"
  if [[ -n "${detected_version}" ]]; then
    if [[ "${detected_version}" == "${SPOT_SDK_VERSION}" ]]; then
      echo "[spot-sdk] using existing SDK ${detected_version} at ${sdk_dir}"
      return 0
    fi

    echo "[spot-sdk] found SDK version ${detected_version} at ${sdk_dir}, expected ${SPOT_SDK_VERSION}" >&2
    if remove_existing_sdk "${sdk_dir}"; then
      return 1
    fi
    echo "[spot-sdk] remove the directory, set FORCE_REINSTALL_SPOT_SDK=1, or override SPOT_CPP_SDK_DIR/SPOT_SDK_REF." >&2
    exit 1
  fi

  echo "[spot-sdk] found existing directory at ${sdk_dir} but could not determine the SDK version." >&2
  if remove_existing_sdk "${sdk_dir}"; then
    return 1
  fi
  echo "[spot-sdk] remove the directory, set FORCE_REINSTALL_SPOT_SDK=1, or point SPOT_CPP_SDK_DIR at a clean SDK checkout." >&2
  exit 1
}

install_from_archive() {
  local tmp_dir="$1"
  local extract_root="${tmp_dir}/extract"
  local extracted_dir=""

  mkdir -p "${extract_root}"
  echo "[spot-sdk] installing Spot SDK ${SPOT_SDK_VERSION} from local archive ${SPOT_SDK_ARCHIVE}"
  unzip -q "${SPOT_SDK_ARCHIVE}" -d "${extract_root}"
  extracted_dir="$(find "${extract_root}" -mindepth 1 -maxdepth 1 -type d | head -n 1)"

  if [[ -z "${extracted_dir}" ]]; then
    echo "[spot-sdk] archive did not contain an SDK directory: ${SPOT_SDK_ARCHIVE}" >&2
    exit 1
  fi

  mv "${extracted_dir}" "${SPOT_CPP_SDK_DIR}"
}

clone_from_git() {
  local tmp_dir="$1"
  local clone_dir="${tmp_dir}/spot-cpp-sdk"

  echo "[spot-sdk] cloning ${SPOT_SDK_REPO} at ${SPOT_SDK_REF}"
  git -c advice.detachedHead=false clone --depth 1 --branch "${SPOT_SDK_REF}" "${SPOT_SDK_REPO}" "${clone_dir}"
  mv "${clone_dir}" "${SPOT_CPP_SDK_DIR}"
}

main() {
  local detected_version=""
  local tmp_dir=""

  validate_existing_sdk "${SPOT_CPP_SDK_DIR}" && return 0

  mkdir -p "$(dirname "${SPOT_CPP_SDK_DIR}")"
  tmp_dir="$(mktemp -d "${TMPDIR:-/tmp}/spot-sdk.XXXXXX")"
  trap "rm -rf '${tmp_dir}'" EXIT

  if [[ -f "${SPOT_SDK_ARCHIVE}" ]]; then
    install_from_archive "${tmp_dir}"
  else
    clone_from_git "${tmp_dir}"
  fi

  detected_version="$(detect_sdk_version "${SPOT_CPP_SDK_DIR}" || true)"
  if [[ -n "${detected_version}" && "${detected_version}" != "${SPOT_SDK_VERSION}" ]]; then
    echo "[spot-sdk] installed SDK version ${detected_version}, expected ${SPOT_SDK_VERSION}" >&2
    exit 1
  fi

  if [[ ! -f "${SPOT_CPP_SDK_DIR}/cpp/CMakeLists.txt" ]]; then
    echo "[spot-sdk] SDK checkout is missing cpp/CMakeLists.txt: ${SPOT_CPP_SDK_DIR}" >&2
    exit 1
  fi

  echo "[spot-sdk] ready at ${SPOT_CPP_SDK_DIR}"
}

main "$@"
