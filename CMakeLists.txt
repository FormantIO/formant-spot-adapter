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

set(PROTO_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/formant_model.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/formant_agent.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/proto/adapter_config.proto
)

set(GENERATED_SRCS)
set(GENERATED_HDRS)

foreach(PROTO ${PROTO_FILES})
  get_filename_component(PROTO_NAME ${PROTO} NAME_WE)

  set(PB_CC ${CMAKE_CURRENT_BINARY_DIR}/generated/${PROTO_NAME}.pb.cc)
  set(PB_H  ${CMAKE_CURRENT_BINARY_DIR}/generated/${PROTO_NAME}.pb.h)
  set(GRPC_CC ${CMAKE_CURRENT_BINARY_DIR}/generated/${PROTO_NAME}.grpc.pb.cc)
  set(GRPC_H  ${CMAKE_CURRENT_BINARY_DIR}/generated/${PROTO_NAME}.grpc.pb.h)

  add_custom_command(
    OUTPUT ${PB_CC} ${PB_H}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
         --cpp_out=${CMAKE_CURRENT_BINARY_DIR}/generated
         ${PROTO}
    DEPENDS ${PROTO}
  )

  add_custom_command(
    OUTPUT ${GRPC_CC} ${GRPC_H}
    COMMAND ${Protobuf_PROTOC_EXECUTABLE}
    ARGS --proto_path=${CMAKE_CURRENT_SOURCE_DIR}/proto
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

# The supported service deployment path for a source checkout is scripts/deploy.sh,
# which generates a checkout-specific unit file. Keep cmake --install limited to
# the binary and example configuration artifacts.
install(TARGETS formant-spot-adapter RUNTIME DESTINATION /usr/local/bin)
install(FILES config/formant-spot-adapter.env.example DESTINATION /usr/local/share/formant-spot-adapter/examples)
install(FILES config/formant-spot-adapter.json.example DESTINATION /usr/local/share/formant-spot-adapter/examples)
