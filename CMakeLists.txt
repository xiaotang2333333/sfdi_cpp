cmake_minimum_required(VERSION 3.10.0)
project(sfdi VERSION 0.1.0 LANGUAGES C CXX)
find_package(Qt6 REQUIRED COMPONENTS Core Widgets)
qt_standard_project_setup()
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/hardware)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sfdi-algorithm)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sfdi-ui)

qt_add_executable(sfdi ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)
target_precompile_headers(
    sfdi PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/pch.h
)
target_link_libraries(sfdi PRIVATE sfdiAlgorithm)
target_link_libraries(sfdi PRIVATE Hardware)
target_link_libraries(sfdi PRIVATE sfdiUI)

target_compile_options(sfdi PRIVATE -ffast-math -funroll-loops -flto)
target_link_libraries(sfdi PRIVATE -fopenmp -flto)
add_dependencies(sfdi sfdiAlgorithm Hardware sfdiUI)
# 生成qt.conf文件以指定插件路径
get_target_property(QT_CORE_LIB Qt6::Core LOCATION)
message(STATUS "Qt6::Core location: ${QT_CORE_LIB}")
get_filename_component(QT_BIN_DIR "${QT_CORE_LIB}" DIRECTORY)
get_filename_component(VCPKG_LIB_ROOT_DIR "${QT_BIN_DIR}" DIRECTORY)
set(VCPKG_QT_PLUGIN_DIR "${VCPKG_LIB_ROOT_DIR}/Qt6/plugins")
file(TO_CMAKE_PATH "${VCPKG_QT_PLUGIN_DIR}" VCPKG_QT_PLUGIN_DIR_POSIX)

set(QT_CONF_CONTENT "[Paths]\nPlugins=${VCPKG_QT_PLUGIN_DIR_POSIX}")

file(GENERATE 
    OUTPUT "$<TARGET_FILE_DIR:sfdi>/qt.conf"
    CONTENT "${QT_CONF_CONTENT}"
)
#以下是安装部分
include(GNUInstallDirs)
install(
    TARGETS sfdi 
    DESTINATION ${CMAKE_INSTALL_BINDIR}
)
qt_generate_deploy_app_script(
    TARGET     sfdi
    OUTPUT_SCRIPT deploy_script
)
install(SCRIPT ${deploy_script})

install(TARGETS sfdi
    RUNTIME_DEPENDENCY_SET sfdi_deps
    DESTINATION bin
)

get_filename_component(COMPILER_BIN_DIR ${CMAKE_CXX_COMPILER} DIRECTORY)

install(RUNTIME_DEPENDENCY_SET sfdi_deps
    DESTINATION bin
    DIRECTORIES 
        ${CMAKE_CURRENT_BINARY_DIR}/
        ${COMPILER_BIN_DIR}/
    PRE_EXCLUDE_REGEXES
        "api-ms-win-.*"
        "ext-ms-win-.*"
    POST_INCLUDE_REGEXES 
        "${CMAKE_CURRENT_BINARY_DIR}/.*"
    POST_EXCLUDE_REGEXES
        "[Ww][Ii][Nn][Dd][Oo][Ww][Ss].*"
)

# CPack configuration for Windows installer
set(CPACK_GENERATOR "NSIS")
set(CPACK_PACKAGE_NAME "sfdi")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_VENDOR "Your Company")
set(CPACK_PACKAGE_DESCRIPTION "SFDI Application")
set(CPACK_WIX_UPGRADE_GUID "12345678-1234-1234-1234-123456789012")
set(CPACK_WIX_PROGRAM_MENU_FOLDER "sfdi")
set(CPACK_PACKAGE_INSTALL_DIRECTORY "sfdi")

include(CPack)