add_library(HTSDK INTERFACE)

if(VCPKG_TARGET_TRIPLET STREQUAL "arm64-linux-dynamic")
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/arm64/GenICam)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/lib/arm64/dep)
    find_package(PkgConfig REQUIRED)
    target_link_directories(HTSDK INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/arm64"
    )
    target_link_libraries(HTSDK INTERFACE
        "-l:libMVSDK.so.2.4.2.342978"
        GCBASE
        GENAPI
        LOG
        MATHPARSER
        NODEMAPDATA
        XMLPARSER
        LOG4CPP
        IMAGECONVERT
        IMAGESAVE
        TINYXML
        LOG4CPP_DEP
    )

    # Ensure runtime can locate these vendor .so files.
    target_link_options(HTSDK INTERFACE
        "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}/lib/arm64"
        "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}/lib/arm64/dep"
        "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}/lib/arm64/GenICam"
    )
elseif(VCPKG_TARGET_TRIPLET STREQUAL "x64-mingw-dynamic")
    target_link_libraries(HTSDK INTERFACE
        "${CMAKE_CURRENT_SOURCE_DIR}/lib/win64/MVSDKmd.lib"
    )
endif()

target_include_directories(HTSDK INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/inc")