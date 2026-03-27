#!/usr/bin/env python3
import os
import sys
from pathlib import Path


def replace_once_or_fail(source: str, old: str, new: str, context: str) -> str:
    if old not in source:
        raise RuntimeError(f"missing expected SDK content for {context}")
    return source.replace(old, new, 1)


def main() -> int:
    spot_sdk_dir = Path(os.environ["SPOT_CPP_SDK_DIR"]).resolve()
    spot_sdk_cmake_file = spot_sdk_dir / "cpp" / "CMakeLists.txt"
    if not spot_sdk_cmake_file.exists():
        print(f"Missing Spot SDK CMake file: {spot_sdk_cmake_file}", file=sys.stderr)
        return 1

    content = spot_sdk_cmake_file.read_text(encoding="utf-8")
    original = content

    # Normalize protobuf package naming for Ubuntu's packaged CMake modules.
    content = content.replace("find_package(protobuf REQUIRED)", "find_package(Protobuf REQUIRED)")
    if "set(PROTOBUF_LIBRARIES ${Protobuf_LIBRARIES})" not in content:
        content = replace_once_or_fail(
            content,
            "find_package(Protobuf REQUIRED)",
            "find_package(Protobuf REQUIRED)\nset(PROTOBUF_LIBRARIES ${Protobuf_LIBRARIES})",
            "protobuf package normalization",
        )

    # Make CLI11 optional so the client library can still build on lean systems.
    cli11_required = "find_package(CLI11 REQUIRED)"
    cli11_optional = (
        "find_package(CLI11 QUIET)\n"
        "if(NOT TARGET CLI11::CLI11)\n"
        "  add_library(CLI11::CLI11 INTERFACE IMPORTED)\n"
        "endif()"
    )
    if cli11_required in content:
        content = replace_once_or_fail(content, cli11_required, cli11_optional, "CLI11 fallback")

    grpc_plugin_lookup = "get_target_property(grpc_cpp_plugin_location gRPC::grpc_cpp_plugin LOCATION)"
    if grpc_plugin_lookup in content:
        content = replace_once_or_fail(
            content,
            grpc_plugin_lookup,
            "set(grpc_cpp_plugin_location /usr/bin/grpc_cpp_plugin)",
            "grpc_cpp_plugin lookup",
        )

    generated_proto_include = "$<BUILD_INTERFACE:${protos_OUTPUT_DIR}/build/protos>"
    if generated_proto_include not in content:
        content = replace_once_or_fail(
            content,
            "$<BUILD_INTERFACE:${protos_OUTPUT_DIR}>",
            "$<BUILD_INTERFACE:${protos_OUTPUT_DIR}>\n    $<BUILD_INTERFACE:${protos_OUTPUT_DIR}/build/protos>",
            "generated protobuf include dir",
        )

    generated_choreo_include = "$<BUILD_INTERFACE:${choreography_protos_OUTPUT_DIR}/build/choreography_protos>"
    if generated_choreo_include not in content:
        content = replace_once_or_fail(
            content,
            "$<BUILD_INTERFACE:${choreography_protos_OUTPUT_DIR}>",
            "$<BUILD_INTERFACE:${choreography_protos_OUTPUT_DIR}>\n    $<BUILD_INTERFACE:${choreography_protos_OUTPUT_DIR}/build/choreography_protos>",
            "generated choreography include dir",
        )

    proto_glob = (
        "file(GLOB_RECURSE bosdyn_protos_files CONFIGURE_DEPENDS\n"
        "    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} \"${API_protos_PATH}/*.proto\")"
    )
    if proto_glob in content:
        content = replace_once_or_fail(
            content,
            proto_glob,
            "file(GLOB_RECURSE bosdyn_protos_files CONFIGURE_DEPENDS \"${API_protos_PATH}/*.proto\")",
            "proto glob root",
        )

    choreography_glob = (
        "file(GLOB_RECURSE bosdyn_choreography_protos_files CONFIGURE_DEPENDS\n"
        "    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} \"${API_choreography_protos_PATH}/*.proto\")"
    )
    if choreography_glob in content:
        content = replace_once_or_fail(
            content,
            choreography_glob,
            "file(GLOB_RECURSE bosdyn_choreography_protos_files CONFIGURE_DEPENDS \"${API_choreography_protos_PATH}/*.proto\")",
            "choreography proto glob root",
        )

    api_proto_copy = "file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../protos/ DESTINATION ${API_protos_PATH})"
    if api_proto_copy in content:
        content = replace_once_or_fail(content, api_proto_copy, "", "redundant proto copy")

    choreography_proto_copy = (
        "file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../choreography_protos/ DESTINATION ${API_choreography_protos_PATH})"
    )
    if choreography_proto_copy in content:
        content = replace_once_or_fail(content, choreography_proto_copy, "", "redundant choreography proto copy")

    api_proto_root = "set(API_protos_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/protos/)"
    if api_proto_root in content:
        content = replace_once_or_fail(
            content,
            api_proto_root,
            "set(API_protos_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../protos)",
            "API proto source root",
        )

    choreography_proto_root = "set(API_choreography_protos_PATH ${CMAKE_CURRENT_SOURCE_DIR}/build/choreography_protos/)"
    if choreography_proto_root in content:
        content = replace_once_or_fail(
            content,
            choreography_proto_root,
            "set(API_choreography_protos_PATH ${CMAKE_CURRENT_SOURCE_DIR}/../choreography_protos)",
            "choreography proto source root",
        )

    examples_option = 'option(BUILD_SPOT_CPP_EXAMPLES "Build Spot C++ SDK examples" OFF)'
    if examples_option not in content:
        content = replace_once_or_fail(
            content,
            'option(BUILD_CHOREOGRAPHY_LIBS "Boolean to control whether choreography proto libraries are built" ON)',
            'option(BUILD_CHOREOGRAPHY_LIBS "Boolean to control whether choreography proto libraries are built" ON)\n'
            'option(BUILD_SPOT_CPP_EXAMPLES "Build Spot C++ SDK examples" OFF)',
            "example build option",
        )

    example_gate = "### EXAMPLE EXECUTABLES ###\nif (BUILD_SPOT_CPP_EXAMPLES)"
    if example_gate not in content:
        content = replace_once_or_fail(
            content,
            "### EXAMPLE EXECUTABLES ###",
            "### EXAMPLE EXECUTABLES ###\nif (BUILD_SPOT_CPP_EXAMPLES)",
            "example section gate",
        )

    example_gate_end = "install(TARGETS spot_cam DESTINATION ${CMAKE_INSTALL_BINDIR})\nendif()"
    if example_gate_end not in content:
        content = replace_once_or_fail(
            content,
            "install(TARGETS spot_cam DESTINATION ${CMAKE_INSTALL_BINDIR})",
            "install(TARGETS spot_cam DESTINATION ${CMAKE_INSTALL_BINDIR})\nendif()",
            "example section terminator",
        )

    cli11_filter = (
        'list(FILTER bosdyn_client_SRC EXCLUDE REGEX ".*/bosdyn/client/util/cli_util\\\\.cpp$")'
    )
    if cli11_filter not in content:
        content = replace_once_or_fail(
            content,
            'file(GLOB_RECURSE bosdyn_client_SRC CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/bosdyn/*")',
            'file(GLOB_RECURSE bosdyn_client_SRC CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/bosdyn/*")\n'
            "get_target_property(_cli11_include_dirs CLI11::CLI11 INTERFACE_INCLUDE_DIRECTORIES)\n"
            "if(NOT _cli11_include_dirs)\n"
            '  list(FILTER bosdyn_client_SRC EXCLUDE REGEX ".*/bosdyn/client/util/cli_util\\\\.cpp$")\n'
            "endif()",
            "CLI11-dependent source filtering",
        )

    service_client_header = spot_sdk_dir / "cpp" / "bosdyn" / "client" / "service_client" / "service_client.h"
    if service_client_header.exists():
        header_content = service_client_header.read_text(encoding="utf-8")
        header_updated = header_content.replace(
            "Request::GetDescriptor()->full_name().c_str()",
            "request.GetDescriptor()->full_name().c_str()",
        )
        if header_updated != header_content:
            service_client_header.write_text(header_updated, encoding="utf-8")
            print(f"[spot-sdk] patched {service_client_header}")

    if content != original:
        spot_sdk_cmake_file.write_text(content, encoding="utf-8")
        print(f"[spot-sdk] patched {spot_sdk_cmake_file}")
    else:
        print(f"[spot-sdk] SDK already prepared: {spot_sdk_cmake_file}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
