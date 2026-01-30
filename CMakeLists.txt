cmake_minimum_required(VERSION 3.10.0)
project(sfdi VERSION 0.1.0 LANGUAGES C CXX)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)
qt_standard_project_setup()
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/hardware)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sfdi-algorithm)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sfdi-ui)
qt_add_executable(sfdi ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

target_link_libraries(sfdi PRIVATE sfdiAlgorithm)
target_link_libraries(sfdi PRIVATE Hardware)
target_link_libraries(sfdi PRIVATE sfdiUI)

target_compile_options(sfdi PRIVATE -ffast-math -funroll-loops -flto)
#target_compile_definitions(sfdi PRIVATE GENERATE_LOOKUP)
target_link_libraries(sfdi PRIVATE -fopenmp -flto)
add_dependencies(sfdi sfdiAlgorithm Hardware sfdiUI)
# 生成qt.conf文件以指定插件路径
set(QT_CONF_DIR "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/Qt6/plugins")

file(GENERATE
    OUTPUT "$<TARGET_FILE_DIR:sfdi>/qt.conf"
    CONTENT "[Paths]\nPlugins=${QT_CONF_DIR}"
)
set_target_properties(sfdi PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../lib"
)
#以下是cpack打包依赖
include(GNUInstallDirs)
include(CPack)
install(TARGETS sfdi
    RUNTIME_DEPENDENCY_SET sfdi_deps
    DESTINATION bin
)
if(VCPKG_TARGET_TRIPLET STREQUAL "x64-mingw-dynamic")
    qt_generate_deploy_app_script(
        TARGET     sfdi
        OUTPUT_SCRIPT deploy_script
    )
    get_filename_component(COMPILER_BIN_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)
    install(SCRIPT ${deploy_script})
    install(RUNTIME_DEPENDENCY_SET sfdi_deps
    DESTINATION bin
    DIRECTORIES 
        ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}
        ${COMPILER_BIN_DIR}
    PRE_EXCLUDE_REGEXES
        "api-ms-win-.*"
        "ext-ms-win-.*"
    POST_INCLUDE_REGEXES    
        "${CMAKE_CURRENT_BINARY_DIR}/.*"
    POST_EXCLUDE_REGEXES
        "[Ww][Ii][Nn][Dd][Oo][Ww][Ss].*"
    )
    set(CPACK_GENERATOR "NSIS")
elseif(VCPKG_TARGET_TRIPLET STREQUAL "arm64-linux-dynamic")
    install(DIRECTORY
        "${QT_CONF_DIR}"
        DESTINATION .
    )
    install(
        RUNTIME_DEPENDENCY_SET sfdi_deps
        DESTINATION lib
        DIRECTORIES
            ${CMAKE_CURRENT_SOURCE_DIR}/hardware/HTcam/lib/arm64
            ${CMAKE_CURRENT_SOURCE_DIR}/hardware/HTcam/lib/arm64/dep
            ${CMAKE_CURRENT_SOURCE_DIR}/hardware/HTcam/lib/arm64/GenICam
            ${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/lib
        PRE_EXCLUDE_REGEXES 
        "libgcc." 
        "libgomp."
        POST_EXCLUDE_REGEXES
        "^/lib" 
        "^/usr/lib"
        "^/usr/local/lib"
    )
    install( 
        FILES
            "${CMAKE_CURRENT_SOURCE_DIR}/qt.conf"
        DESTINATION bin
    )
    file(GLOB XCB_DEP "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/lib/libQt6XcbQpa.so*")
    install( 
        FILES
            ${XCB_DEP}
        DESTINATION lib
    )
    set(CPACK_GENERATOR "DEB")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "libudev1")
endif()