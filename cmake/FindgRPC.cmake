# FindgRPC.cmake
# -------------------------------
# Find the gRPC libraries and create imported targets
#
# Targets created:
#   gRPC::grpc++         - gRPC C++ library
#   gRPC::grpc           - gRPC C library
#   gRPC::grpc_cpp_plugin - gRPC C++ plugin for protoc
#   gRPC::grpc++_reflection - gRPC reflection library
#
# Variables:
#   gRPC_FOUND          - True if all components found
#   gRPC_INCLUDE_DIRS   - Include directories for gRPC
#   gRPC_LIBRARIES      - Libraries needed to link gRPC

include(FindPackageHandleStandardArgs)

# Find gRPC include directory
find_path(gRPC_INCLUDE_DIR
    NAMES grpcpp/grpcpp.h grpc++/grpc++.h
    HINTS ENV gRPC_ROOT
    PATHS
        ${CMAKE_PREFIX_PATH}/include
        /usr/include
        /usr/local/include
        ${gRPC_ROOT}/include
)

# Find gRPC C++ library
find_library(gRPC_GRPC++_LIBRARY
    NAMES grpc++ grpc++_unsecure
    HINTS ENV gRPC_ROOT
    PATHS
        ${CMAKE_PREFIX_PATH}/lib
        ${CMAKE_PREFIX_PATH}/lib/x86_64-linux-gnu
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        ${gRPC_ROOT}/lib
)

# Find gRPC C library
find_library(gRPC_GRPC_LIBRARY
    NAMES grpc grpc_unsecure
    HINTS ENV gRPC_ROOT
    PATHS
        ${CMAKE_PREFIX_PATH}/lib
        ${CMAKE_PREFIX_PATH}/lib/x86_64-linux-gnu
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        ${gRPC_ROOT}/lib
)

# Find gRPC++ reflection library
find_library(gRPC_GRPC++_REFLECTION_LIBRARY
    NAMES grpc++_reflection grpc++_reflection_unsecure
    HINTS ENV gRPC_ROOT
    PATHS
        ${CMAKE_PREFIX_PATH}/lib
        ${CMAKE_PREFIX_PATH}/lib/x86_64-linux-gnu
        /usr/lib
        /usr/lib/x86_64-linux-gnu
        /usr/local/lib
        ${gRPC_ROOT}/lib
)

# Find grpc_cpp_plugin for proto generation
find_program(gRPC_CPP_PLUGIN
    NAMES grpc_cpp_plugin
    HINTS ENV gRPC_ROOT
    PATHS
        ${CMAKE_PREFIX_PATH}/bin
        /usr/bin
        /usr/local/bin
        ${gRPC_ROOT}/bin
)

# Handle standard arguments
find_package_handle_standard_args(gRPC
    REQUIRED_VARS
        gRPC_INCLUDE_DIR
        gRPC_GRPC++_LIBRARY
        gRPC_GRPC_LIBRARY
        gRPC_CPP_PLUGIN
)

# Create imported targets
if(gRPC_FOUND AND NOT TARGET gRPC::grpc++)
    add_library(gRPC::grpc++ UNKNOWN IMPORTED)
    set_target_properties(gRPC::grpc++ PROPERTIES
        IMPORTED_LOCATION "${gRPC_GRPC++_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${gRPC_INCLUDE_DIR}"
        INTERFACE_LINK_LIBRARIES "protobuf::libprotobuf"
    )
endif()

if(gRPC_FOUND AND NOT TARGET gRPC::grpc)
    add_library(gRPC::grpc UNKNOWN IMPORTED)
    set_target_properties(gRPC::grpc PROPERTIES
        IMPORTED_LOCATION "${gRPC_GRPC_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${gRPC_INCLUDE_DIR}"
    )
endif()

if(gRPC_GRPC++_REFLECTION_LIBRARY AND NOT TARGET gRPC::grpc++_reflection)
    add_library(gRPC::grpc++_reflection UNKNOWN IMPORTED)
    set_target_properties(gRPC::grpc++_reflection PROPERTIES
        IMPORTED_LOCATION "${gRPC_GRPC++_REFLECTION_LIBRARY}"
    )
endif()

if(gRPC_CPP_PLUGIN AND NOT TARGET gRPC::grpc_cpp_plugin)
    add_executable(gRPC::grpc_cpp_plugin IMPORTED)
    set_target_properties(gRPC::grpc_cpp_plugin PROPERTIES
        IMPORTED_LOCATION "${gRPC_CPP_PLUGIN}"
    )
endif()

mark_as_advanced(
    gRPC_INCLUDE_DIR
    gRPC_GRPC++_LIBRARY
    gRPC_GRPC_LIBRARY
    gRPC_GRPC++_REFLECTION_LIBRARY
    gRPC_CPP_PLUGIN
)