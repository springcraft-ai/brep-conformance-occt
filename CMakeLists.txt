# SPDX-License-Identifier: MIT
# The adapter, linked against the OpenCascade libraries of the cadquery-ocp-novtk package that
# pyproject.toml pins, built with the headers of the same release. build.sh sets the cache
# variables.
cmake_minimum_required(VERSION 3.24)
project(occt_adapter LANGUAGES CXX)

set(OCCT_INCLUDE_DIR "" CACHE PATH "The headers of the OpenCascade release the package bundles")
set(OCCT_LIBRARY_DIR "" CACHE PATH "The folder of the package's OpenCascade libraries")
set(OCCT_PACKAGE_VERSION "" CACHE STRING "The package's version, which hello reports")
if(NOT IS_DIRECTORY "${OCCT_INCLUDE_DIR}" OR NOT IS_DIRECTORY "${OCCT_LIBRARY_DIR}")
    message(FATAL_ERROR "Set OCCT_INCLUDE_DIR and OCCT_LIBRARY_DIR, or run build.sh")
endif()

# The suite's C++ binding of the protocol, as the binding's own release carries it, pinned by its
# hash. FETCHCONTENT_SOURCE_DIR_BREP_PROTOCOL names a binding to build against instead, such as a
# checkout's protocol/cpp.
include(FetchContent)
FetchContent_Declare(
    brep_protocol
    URL https://github.com/springcraft-ai/brep-conformance/releases/download/brep-protocol-cpp-v2.0.0/brep_protocol-2.0.0.tar.xz
    URL_HASH SHA256=03897e5a780523b39697b3eb71d6b3772a4e9237f336e59ecc6f085699412a04
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(brep_protocol)

set(TOOLKITS
    TKRWMesh TKXCAF TKDESTEP TKXSBase TKHLR TKMesh TKOffset TKFillet TKBool TKBO TKPrim
    TKShHealing TKTopAlgo TKGeomAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel
)
set(LIBRARIES)
foreach(toolkit IN LISTS TOOLKITS)
    # The package names each library with its version, and on Linux with a hash of its contents.
    file(GLOB found "${OCCT_LIBRARY_DIR}/lib${toolkit}.*dylib"
         "${OCCT_LIBRARY_DIR}/lib${toolkit}-*.so*" "${OCCT_LIBRARY_DIR}/lib${toolkit}.so*")
    list(LENGTH found count)
    if(NOT count EQUAL 1)
        message(FATAL_ERROR "Expected one ${toolkit} in ${OCCT_LIBRARY_DIR}, found: ${found}")
    endif()
    list(APPEND LIBRARIES ${found})
endforeach()

add_executable(occt-adapter adapter.cpp arithmetic.cpp)
# arithmetic.h says why.
if(NOT MSVC)
    set_source_files_properties(arithmetic.cpp PROPERTIES
        COMPILE_OPTIONS "-ffp-contract=off;-fno-builtin-sin;-fno-builtin-cos")
endif()
target_compile_features(occt-adapter PRIVATE cxx_std_17)
target_include_directories(occt-adapter SYSTEM PRIVATE "${OCCT_INCLUDE_DIR}")
target_link_libraries(occt-adapter PRIVATE brep_protocol::brep_protocol ${LIBRARIES})
if(OCCT_PACKAGE_VERSION)
    target_compile_definitions(occt-adapter PRIVATE
                               OCCT_PACKAGE_VERSION="${OCCT_PACKAGE_VERSION}")
endif()
if(NOT MSVC)
    target_compile_options(occt-adapter PRIVATE -Wall -Wextra)
endif()

if(APPLE)
    # The package's libraries carry the install names of the machine that built them. The adapter
    # loads them, and the copy of libc++ they share, from the package's folder.
    target_link_options(occt-adapter PRIVATE -Wl,-headerpad_max_install_names)
    add_custom_command(TARGET occt-adapter POST_BUILD
        COMMAND ${CMAKE_COMMAND} -DBINARY=$<TARGET_FILE:occt-adapter>
                -DLIBRARY_DIR=${OCCT_LIBRARY_DIR} -P ${CMAKE_CURRENT_SOURCE_DIR}/relink.cmake
        VERBATIM)
else()
    set_target_properties(occt-adapter PROPERTIES BUILD_RPATH "${OCCT_LIBRARY_DIR}")
endif()
