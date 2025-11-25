# Cross-compilation toolchain file for x64 to arm64
# This file enables cross-compilation from x64 host to arm64 target
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Specify the cross compiler
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

# Set the root directory for the target system
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Additional compiler flags for ARM64
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=armv8-a")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")

set(MV_CAM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/hardware/MVcam" CACHE PATH "Root of MVcam SDK")
set(MV_CAM_LIB_DIR "${MV_CAM_ROOT}/lib/arm64")
add_library(MV_CAM::MVSDK SHARED IMPORTED)
set_target_properties(MV_CAM::MVSDK PROPERTIES
  IMPORTED_LOCATION "${MV_CAM_LIB_DIR}/libMVSDK.so"
  INTERFACE_INCLUDE_DIRECTORIES "${MV_CAM_ROOT}/inc"
)
set(VCPKG_TARGET_TRIPLET arm64-linux-dynamic)
set(VCPKG_HOST_TRIPLET x64-linux)

# Set the build type to Release for cross-compilation
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build" FORCE)
endif()

# Disable some tests that may not work in cross-compilation
set(CMAKE_CROSSCOMPILING ON)