add_library(MVSDK SHARED IMPORTED GLOBAL)
if(VCPKG_TARGET_TRIPLET STREQUAL "arm64-linux-dynamic")
    set_target_properties(MVSDK PROPERTIES
      IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/arm64/libMVSDK.so"
    )
elseif(VCPKG_TARGET_TRIPLET STREQUAL "x64-linux-dynamic")
    set_target_properties(MVSDK PROPERTIES
      IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/x64/libMVSDK.so"
    )
elseif(VCPKG_TARGET_TRIPLET STREQUAL "x64-mingw-dynamic")
    set_target_properties(MVSDK PROPERTIES
      IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/lib/win64/MVCAMSDK_X64.dll"
      IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/win64/MVCAMSDK_X64.lib"
    )
else()
    message(FATAL_ERROR "Unsupported VCPKG_TARGET_TRIPLET: ${VCPKG_TARGET_TRIPLET}")
endif()
target_include_directories(MVSDK INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/inc")
