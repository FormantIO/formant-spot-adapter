# gRPC CMake shim for distro packages that do not ship gRPCConfig.cmake
# (Ubuntu 20.04/22.04 libgrpc++-dev, which is what scripts/bootstrap_ubuntu.sh
# and the Dockerfile install).
#
# The library paths are FOUND, not hardcoded. They used to be written out as
# /usr/lib/aarch64-linux-gnu/..., which works on a Jetson and on an arm64
# container and fails everywhere else: on an x86_64 machine ninja stops with
# "libgrpc++.so, needed by formant-spot-adapter, missing and no known rule to
# make it". CI runs on x86_64, which is how that surfaced.

set(gRPC_FOUND TRUE)

find_library(FSA_GRPC_LIBRARY NAMES grpc)
find_library(FSA_GRPCPP_LIBRARY NAMES grpc++)
find_program(FSA_GRPC_CPP_PLUGIN NAMES grpc_cpp_plugin)

foreach(_fsa_grpc_var FSA_GRPC_LIBRARY FSA_GRPCPP_LIBRARY FSA_GRPC_CPP_PLUGIN)
  if(NOT ${_fsa_grpc_var})
    message(
      FATAL_ERROR
      "gRPC shim: ${_fsa_grpc_var} not found. Install the distro gRPC packages "
      "(libgrpc++-dev protobuf-compiler-grpc), or point CMAKE_PREFIX_PATH at a "
      "gRPC that ships its own gRPCConfig.cmake and drop -DgRPC_DIR."
    )
  endif()
endforeach()

if(NOT TARGET gRPC::grpc)
  add_library(gRPC::grpc UNKNOWN IMPORTED)
  set_target_properties(gRPC::grpc PROPERTIES
    IMPORTED_LOCATION "${FSA_GRPC_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "/usr/include"
  )
endif()

if(NOT TARGET gRPC::grpc++)
  add_library(gRPC::grpc++ UNKNOWN IMPORTED)
  set_target_properties(gRPC::grpc++ PROPERTIES
    IMPORTED_LOCATION "${FSA_GRPCPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "/usr/include"
    INTERFACE_LINK_LIBRARIES "gRPC::grpc"
  )
endif()

if(NOT TARGET gRPC::grpc_cpp_plugin)
  add_executable(gRPC::grpc_cpp_plugin IMPORTED)
  set_target_properties(gRPC::grpc_cpp_plugin PROPERTIES
    IMPORTED_LOCATION "${FSA_GRPC_CPP_PLUGIN}"
  )
endif()
