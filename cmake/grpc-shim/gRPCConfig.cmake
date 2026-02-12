# gRPC CMake shim for distro packages that do not ship gRPCConfig.cmake.

set(gRPC_FOUND TRUE)

if(NOT TARGET gRPC::grpc)
  add_library(gRPC::grpc UNKNOWN IMPORTED)
  set_target_properties(gRPC::grpc PROPERTIES
    IMPORTED_LOCATION "/usr/lib/aarch64-linux-gnu/libgrpc.so"
    INTERFACE_INCLUDE_DIRECTORIES "/usr/include"
  )
endif()

if(NOT TARGET gRPC::grpc++)
  add_library(gRPC::grpc++ UNKNOWN IMPORTED)
  set_target_properties(gRPC::grpc++ PROPERTIES
    IMPORTED_LOCATION "/usr/lib/aarch64-linux-gnu/libgrpc++.so"
    INTERFACE_INCLUDE_DIRECTORIES "/usr/include"
    INTERFACE_LINK_LIBRARIES "gRPC::grpc"
  )
endif()

if(NOT TARGET gRPC::grpc_cpp_plugin)
  add_executable(gRPC::grpc_cpp_plugin IMPORTED)
  set_target_properties(gRPC::grpc_cpp_plugin PROPERTIES
    IMPORTED_LOCATION "/usr/bin/grpc_cpp_plugin"
  )
endif()
