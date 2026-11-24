# SPDX-License-Identifier: MIT OR Apache-2.0
# The adapter, linked against the OpenCascade of conda-forge's occt package. build.sh makes the
# environment and sets the cache variables.
cmake_minimum_required(VERSION 3.24)
project(occt_adapter LANGUAGES CXX)

set(OCCT_RELEASE "" CACHE STRING "The OpenCascade release of the occt package, which hello reports")
set(OCCT_PACKAGE "" CACHE STRING "The occt package's version and build, which configuration names")

# The suite's C++ binding of the protocol, as the binding's own release carries it, pinned by its
# hash. FETCHCONTENT_SOURCE_DIR_BREP_PROTOCOL names a binding to build against instead, such as a
# checkout's protocol/cpp.
include(FetchContent)
FetchContent_Declare(
    brep_protocol
    URL https://github.com/springcraft-ai/brep-conformance/releases/download/brep-protocol-cpp-v2.0.1/brep_protocol-2.0.1.tar.xz
    URL_HASH SHA256=ba4a25e29ab779363f70f66c0a68837c8a87430a9c5ac29a11eb0067ba91ec27
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(brep_protocol)

find_package(OpenCASCADE CONFIG REQUIRED)

add_executable(occt-adapter adapter.cpp arithmetic.cpp)
# arithmetic.h says why.
if(NOT MSVC)
    set_source_files_properties(arithmetic.cpp PROPERTIES
        COMPILE_OPTIONS "-ffp-contract=off;-fno-builtin-sin;-fno-builtin-cos")
endif()
target_compile_features(occt-adapter PRIVATE cxx_std_17)
target_include_directories(occt-adapter SYSTEM PRIVATE "${OpenCASCADE_INCLUDE_DIR}")
target_link_libraries(occt-adapter PRIVATE brep_protocol::brep_protocol
    TKRWMesh TKXCAF TKDESTEP TKXSBase TKHLR TKMesh TKOffset TKFillet TKBool TKBO TKPrim
    TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel)
if(OCCT_RELEASE)
    target_compile_definitions(occt-adapter PRIVATE OCCT_RELEASE="${OCCT_RELEASE}")
endif()
if(OCCT_PACKAGE)
    target_compile_definitions(occt-adapter PRIVATE OCCT_PACKAGE="${OCCT_PACKAGE}")
endif()
if(NOT MSVC)
    target_compile_options(occt-adapter PRIVATE -Wall -Wextra)
endif()
# The adapter loads OpenCascade, and on Linux the libstdc++ it needs, from the environment.
set_target_properties(occt-adapter PROPERTIES BUILD_RPATH "${OpenCASCADE_LIBRARY_DIR}")
