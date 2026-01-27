add_library(HTSDK SHARED IMPORTED GLOBAL)
if(VCPKG_TARGET_TRIPLET STREQUAL "arm64-linux-dynamic")
    
elseif(VCPKG_TARGET_TRIPLET STREQUAL "x64-mingw-dynamic")
    set_target_properties(HTSDK PROPERTIES
        IMPORTED_LOCATION "$ENV{GENICAM_GENTL64_PATH}/MVSDKmd.dll"
        IMPORTED_IMPLIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/win64/MVSDKmd.lib"
    )
endif()
target_include_directories(HTSDK INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/inc")