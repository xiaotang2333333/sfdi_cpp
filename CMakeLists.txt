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

install(TARGETS sfdi
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)