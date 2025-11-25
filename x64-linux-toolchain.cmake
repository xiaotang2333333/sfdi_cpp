set(VCPKG_TARGET_TRIPLET x64-linux-dynamic)
set(VCPKG_HOST_TRIPLET=x64-linux-dynamic)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
set(MV_CAM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/hardware/MVcam" CACHE PATH "Root of MVcam SDK")
set(MV_CAM_LIB_DIR "${MV_CAM_ROOT}/lib/x64")
add_library(MV_CAM::MVSDK SHARED IMPORTED)
set_target_properties(MV_CAM::MVSDK PROPERTIES
  IMPORTED_LOCATION "${MV_CAM_LIB_DIR}/libMVSDK.so"
  INTERFACE_INCLUDE_DIRECTORIES "${MV_CAM_ROOT}/inc"
)