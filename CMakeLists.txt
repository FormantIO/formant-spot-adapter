cmake_minimum_required(VERSION 3.16)
project(formant_spot_adapter VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)

# Ubuntu 20.04's gRPC packages generally do not provide gRPCConfig.cmake.
# Use binaries/libs directly for compatibility.
find_program(GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN_EXECUTABLE)
  message(FATAL_ERROR "grpc_cpp_plugin not found. Install protobuf-compiler-grpc.")
endif()

find_library(GRPCPP_LIBRARY grpc++)
find_library(GRPC_LIBRARY grpc)
if(NOT GRPCPP_LIBRARY OR NOT GRPC_LIBRARY)
  message(FATAL_ERROR "gRPC libraries not found. Install libgrpc++-dev.")
endif()

# Spot SDK location must point to the root of spot-cpp-sdk source checkout.
set(SPOT_CPP_SDK_DIR "" CACHE PATH "Path to spot-cpp-sdk root")
if(NOT SPOT_CPP_SDK_DIR)
  message(FATAL_ERROR "SPOT_CPP_SDK_DIR is required. Set -DSPOT_CPP_SDK_DIR=/path/to/spot-cpp-sdk")
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/proto
  ${SPOT_CPP_SDK_DIR}/cpp
  ${SPOT_CPP_SDK_DIR}/protos
  ${CMAKE_BINARY_DIR}/spot_cpp_sdk_build/protos/build/protos
  ${Protobuf_INCLUDE_DIRS}
)

# Build Spot SDK client library from source checkout.
add_subdirectory(${SPOT_CPP_SDK_DIR}/cpp ${CMAKE_BINARY_DIR}/spot_cpp_sdk_build EXCLUDE_FROM_ALL)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/generated)
# Clean up obsolete generated files from the pre-vendored Formant proto layout so
# manual inspection of build/generated does not pick up stale message shapes.
file(REMOVE
  ${CMAKE_CURRENT_BINARY_DIR}/generated/formant_agent.pb.cc
  ${CMAKE_CURRENT_BINARY_DIR}/generated/formant_agent.pb.h
  ${CMAKE_CURRENT_BINARY_DIR}/generated/formant_agent.grpc.pb.cc
  ${CMAKE_CURRENT_BINARY_DIR}/generated/formant_agent.grpc.pb.h
  ${CMAKE_CURRENT_BINARY_DIR}/generated/formant_model.pb.cc
  ${CMAKE_CURRENT_BINARY_DIR}/generated/formant_model.pb.h
  ${CMAKE_CURRENT_BINARY_DIR}/generated/formant_model.grpc.pb.cc
  ${CMAKE_CURRENT_BINARY_DIR}/generated/formant_model.grpc.pb.h
)

set(FORMANT_PROTO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/proto)
set(PROTO_FILES
  ${FORMANT_PROTO_ROOT}/protos/agent/v1/agent.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/commands.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/config.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/datapoint.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/event.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/file.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/health.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/intervention.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/math.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/media.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/navigation.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/ros.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/text.proto
  ${FORMANT_PROTO_ROOT}/protos/model/v1/views.proto
  ${FORMANT_PROTO_ROOT}/adapter_config.proto
)

set(GENERATED_SRCS)
set(GENERATED_HDRS)

foreach(PROTO ${PROTO_FILES})
  file(RELATIVE_PATH PROTO_REL ${FORMANT_PROTO_ROOT} ${PROTO})
  get_filename_component(PROTO_DIR ${PROTO_REL} DIRECTORY)
  get_filename_component(PROTO_NAME ${PROTO_REL} NAME_WE)

  if(PROTO_DIR)
    set(GENERATED_PROTO_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/${PROTO_DIR})
  else()
    set(GENERATED_PROTO_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
  endif()

  set(PB_CC ${GENERATED_PROTO_DIR}/${PROTO_NAME}.pb.cc)
  set(PB_H  ${GENERATED_PROTO_DIR}/${PROTO_NAME}.pb.h)
  set(GRPC_CC ${GENERATED_PROTO_DIR}/${PROTO_NAME}.grpc.pb.cc)
  set(GRPC_H  ${GENERATED_PROTO_DIR}/${PROTO_NAME}.grpc.pb.h)

  add_custom_command(
    OUTPUT ${PB_CC} ${PB_H}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED_PROTO_DIR}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --proto_path=${FORMANT_PROTO_ROOT}
         --cpp_out=${CMAKE_CURRENT_BINARY_DIR}/generated
         ${PROTO}
    DEPENDS ${PROTO}
  )

  add_custom_command(
    OUTPUT ${GRPC_CC} ${GRPC_H}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${GENERATED_PROTO_DIR}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --proto_path=${FORMANT_PROTO_ROOT}
         --grpc_out=${CMAKE_CURRENT_BINARY_DIR}/generated
         --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN_EXECUTABLE}
         ${PROTO}
    DEPENDS ${PROTO}
  )

  list(APPEND GENERATED_SRCS ${PB_CC} ${GRPC_CC})
  list(APPEND GENERATED_HDRS ${PB_H} ${GRPC_H})
endforeach()

add_library(formant_proto ${GENERATED_SRCS})
target_include_directories(formant_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_link_libraries(formant_proto PUBLIC ${Protobuf_LIBRARIES} ${GRPCPP_LIBRARY} ${GRPC_LIBRARY})

add_executable(formant-spot-adapter
  src/main.cpp
  src/config.cpp
  src/formant_agent_client.cpp
  src/spot_client.cpp
  src/spot_check_client.cpp
  src/adapter.cpp
)

target_include_directories(formant-spot-adapter PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_link_libraries(formant-spot-adapter PRIVATE
  formant_proto
  bosdyn_client_static
  Threads::Threads
  ${OpenCV_LIBS}
)

add_executable(image-pull-benchmark
  src/image_pull_benchmark.cpp
  src/config.cpp
  src/spot_client.cpp
  src/spot_check_client.cpp
)

target_include_directories(image-pull-benchmark PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_BINARY_DIR}/generated
)

target_link_libraries(image-pull-benchmark PRIVATE
  formant_proto
  bosdyn_client_static
  Threads::Threads
)

# The supported service deployment path for a source checkout is scripts/deploy.sh,
# which generates a checkout-specific unit file. Keep cmake --install limited to
# the binary and example configuration artifacts.
install(TARGETS formant-spot-adapter RUNTIME DESTINATION /usr/local/bin)
install(FILES config/formant-spot-adapter.env.example DESTINATION /usr/local/share/formant-spot-adapter/examples)
install(FILES config/formant-spot-adapter.json.example DESTINATION /usr/local/share/formant-spot-adapter/examples)
