#!/usr/bin/env bash
set -euo pipefail

SPOT_CPP_SDK_DIR="${SPOT_CPP_SDK_DIR:-$(pwd)/third_party/spot-cpp-sdk}"
GRPC_SHIM_DIR="${GRPC_SHIM_DIR:-$(pwd)/cmake/grpc-shim}"
EIGEN3_SHIM_DIR="${EIGEN3_SHIM_DIR:-$(pwd)/cmake/eigen3-shim}"
SPOT_SDK_CMAKE_FILE="${SPOT_CPP_SDK_DIR}/cpp/CMakeLists.txt"
ENABLE_CCACHE="${ENABLE_CCACHE:-1}"
BUILD_DIR="${BUILD_DIR:-build}"
BUILD_JOBS="${BUILD_JOBS:-}"

if [[ ! -f "${SPOT_SDK_CMAKE_FILE}" ]]; then
  echo "Missing Spot SDK CMake file: ${SPOT_SDK_CMAKE_FILE}" >&2
  exit 1
fi

# Patch Spot SDK CMake for Ubuntu 20.04 compatibility:
# - protobuf package name normalization
# - make CLI11 optional
# - skip heavy example targets by default
SPOT_SDK_CMAKE_FILE="${SPOT_SDK_CMAKE_FILE}" SPOT_CPP_SDK_DIR="${SPOT_CPP_SDK_DIR}" python3 - << 'PY'
import os
from pathlib import Path
p = Path(os.environ["SPOT_SDK_CMAKE_FILE"])
s = p.read_text()
original = s

# protobuf package normalization
s = s.replace("find_package(protobuf REQUIRED)", "find_package(Protobuf REQUIRED)")
if "set(PROTOBUF_LIBRARIES ${Protobuf_LIBRARIES})" not in s:
    s = s.replace("find_package(Protobuf REQUIRED)",
                  "find_package(Protobuf REQUIRED)\nset(PROTOBUF_LIBRARIES ${Protobuf_LIBRARIES})", 1)
# Repair any previous literal "\\n" insertion.
s = s.replace(
    "find_package(Protobuf REQUIRED)\\nset(PROTOBUF_LIBRARIES ${Protobuf_LIBRARIES})",
    "find_package(Protobuf REQUIRED)\nset(PROTOBUF_LIBRARIES ${Protobuf_LIBRARIES})",
)

# CLI11 optional fallback
s = s.replace(
    "find_package(CLI11 REQUIRED)",
    "find_package(CLI11 QUIET)\nif(NOT TARGET CLI11::CLI11)\n  add_library(CLI11::CLI11 INTERFACE IMPORTED)\nendif()"
)
s = s.replace(
    "get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)",
    "set(grpc_cpp_plugin_location /usr/bin/grpc_cpp_plugin)"
)

# Generated protobufs are emitted under ${protos_OUTPUT_DIR}/build/protos on Ubuntu cmake/protobuf tooling.
if "$<BUILD_INTERFACE:${protos_OUTPUT_DIR}/build/protos>" not in s:
    s = s.replace(
        "$<BUILD_INTERFACE:${protos_OUTPUT_DIR}>",
        "$<BUILD_INTERFACE:${protos_OUTPUT_DIR}>\n    $<BUILD_INTERFACE:${protos_OUTPUT_DIR}/build/protos>",
    )
if "$<BUILD_INTERFACE:${choreography_protos_OUTPUT_DIR}/build/choreography_protos>" not in s:
    s = s.replace(
        "$<BUILD_INTERFACE:${choreography_protos_OUTPUT_DIR}>",
        "$<BUILD_INTERFACE:${choreography_protos_OUTPUT_DIR}>\n    $<BUILD_INTERFACE:${choreography_protos_OUTPUT_DIR}/build/choreography_protos>",
    )

# Use absolute proto source paths to keep generated symbol namespaces consistent.
s = s.replace(
    "file(GLOB_RECURSE bosdyn_protos_files CONFIGURE_DEPENDS\n    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} \"${API_protos_PATH}/*.proto\")",
    "file(GLOB_RECURSE bosdyn_protos_files CONFIGURE_DEPENDS \"${API_protos_PATH}/*.proto\")",
)
s = s.replace(
    "file(GLOB_RECURSE bosdyn_choreography_protos_files CONFIGURE_DEPENDS\n    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} \"${API_choreography_protos_PATH}/*.proto\")",
    "file(GLOB_RECURSE bosdyn_choreography_protos_files CONFIGURE_DEPENDS \"${API_choreography_protos_PATH}/*.proto\")",
)

# Generate from canonical proto roots (not copied build/protos paths) so imported
# symbols use a consistent basename like bosdyn/api/*.proto.
s = s.replace(
    "set(API_protos_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/protos/)",
    "set(API_protos_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../protos)",
)
s = s.replace(
    "file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../protos/ DESTINATION ${API_protos_PATH})",
    "",
)
s = s.replace(
    "set(API_choreography_protos_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/choreography_protos/)",
    "set(API_choreography_protos_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../choreography_protos)",
)
s = s.replace(
    "file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../choreography_protos/ DESTINATION ${API_choreography_protos_PATH})",
    "",
)

# protobuf 3.6 compatibility in SDK templates.
svc = Path(os.environ["SPOT_CPP_SDK_DIR"]) / "cpp/bosdyn/client/service_client/service_client.h"
if svc.exists():
    hs = svc.read_text()
    hs2 = hs.replace("Request::GetDescriptor()->full_name().c_str()", "request.GetDescriptor()->full_name().c_str()")
    if hs2 != hs:
        svc.write_text(hs2)
s = s.replace(
    "find_package(CLI11 QUIET)\\nif(NOT TARGET CLI11::CLI11)\\n  add_library(CLI11::CLI11 INTERFACE IMPORTED)\\nendif()",
    "find_package(CLI11 QUIET)\nif(NOT TARGET CLI11::CLI11)\n  add_library(CLI11::CLI11 INTERFACE IMPORTED)\nendif()"
)

# Gate examples
if "option(BUILD_SPOT_CPP_EXAMPLES" not in s:
    s = s.replace(
        "option(BUILD_CHOREOGRAPHY_LIBS \"Boolean to control whether choreography proto libraries are built\" ON)",
        "option(BUILD_CHOREOGRAPHY_LIBS \"Boolean to control whether choreography proto libraries are built\" ON)\noption(BUILD_SPOT_CPP_EXAMPLES \"Build Spot C++ SDK examples\" OFF)"
    )
# Repair any previous literal "\\n" insertion.
s = s.replace(
    "option(BUILD_CHOREOGRAPHY_LIBS \"Boolean to control whether choreography proto libraries are built\" ON)\\noption(BUILD_SPOT_CPP_EXAMPLES \"Build Spot C++ SDK examples\" OFF)",
    "option(BUILD_CHOREOGRAPHY_LIBS \"Boolean to control whether choreography proto libraries are built\" ON)\noption(BUILD_SPOT_CPP_EXAMPLES \"Build Spot C++ SDK examples\" OFF)",
)

if "### EXAMPLE EXECUTABLES ###\nif (BUILD_SPOT_CPP_EXAMPLES)" not in s:
    s = s.replace("### EXAMPLE EXECUTABLES ###", "### EXAMPLE EXECUTABLES ###\nif (BUILD_SPOT_CPP_EXAMPLES)")
# Repair any previous literal "\\n" insertion or duplicated insertion.
s = s.replace(
    "### EXAMPLE EXECUTABLES ###\nif (BUILD_SPOT_CPP_EXAMPLES)\\nif (BUILD_SPOT_CPP_EXAMPLES)",
    "### EXAMPLE EXECUTABLES ###\nif (BUILD_SPOT_CPP_EXAMPLES)",
)
s = s.replace(
    "if (BUILD_SPOT_CPP_EXAMPLES)\\nif (BUILD_SPOT_CPP_EXAMPLES)",
    "if (BUILD_SPOT_CPP_EXAMPLES)",
)

if "if (BUILD_SPOT_CPP_EXAMPLES)" in s and "install(TARGETS spot_cam DESTINATION ${CMAKE_INSTALL_BINDIR})\nendif()" not in s:
    s = s.replace(
        "install(TARGETS spot_cam DESTINATION ${CMAKE_INSTALL_BINDIR})",
        "install(TARGETS spot_cam DESTINATION ${CMAKE_INSTALL_BINDIR})\nendif()",
    )

# Exclude CLI11-dependent source when CLI11 headers are unavailable (Ubuntu 20.04 often lacks libcli11-dev).
if "list(FILTER bosdyn_client_SRC EXCLUDE REGEX \".*/bosdyn/client/util/cli_util\\\\.cpp$\")" not in s:
    s = s.replace(
        "file(GLOB_RECURSE bosdyn_client_SRC CONFIGURE_DEPENDS \"${CMAKE_CURRENT_SOURCE_DIR}/bosdyn/*\")",
        "file(GLOB_RECURSE bosdyn_client_SRC CONFIGURE_DEPENDS \"${CMAKE_CURRENT_SOURCE_DIR}/bosdyn/*\")\n"
        "get_target_property(_cli11_include_dirs CLI11::CLI11 INTERFACE_INCLUDE_DIRECTORIES)\n"
        "if(NOT _cli11_include_dirs)\n"
        "  list(FILTER bosdyn_client_SRC EXCLUDE REGEX \".*/bosdyn/client/util/cli_util\\\\.cpp$\")\n"
        "endif()",
    )

if s != original:
    p.write_text(s)
    print("Patched", p)
else:
    print("Spot SDK CMake already patched:", p)
PY

declare -a cmake_args
cmake_args=(
  -S . -B "${BUILD_DIR}" -G Ninja
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

PATH="$(pwd)/scripts:${PATH}" cmake --build "${BUILD_DIR}" --target formant-spot-adapter -j "${BUILD_JOBS}"
