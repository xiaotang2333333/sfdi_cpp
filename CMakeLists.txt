cmake_minimum_required(VERSION 3.10.0)
project(sfdi VERSION 0.1.0 LANGUAGES C CXX)

# 把src目录下所有.cpp文件收集起来
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

# 添加可执行文件，包含main.cpp和src目录下的所有源文件
add_executable(sfdi ${SRC_FILES})

find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenBLAS CONFIG REQUIRED)
find_package(OpenMP REQUIRED)
find_package(NLopt REQUIRED)
find_package(TIFF REQUIRED)
target_include_directories(sfdi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)


target_link_libraries(sfdi PRIVATE 
    Eigen3::Eigen
    OpenBLAS::OpenBLAS 
    OpenMP::OpenMP_CXX
    NLopt::nlopt
    TIFF::TIFF
)
target_compile_options(sfdi PRIVATE -O3 -ffast-math -funroll-loops -flto )
target_link_libraries(sfdi PRIVATE -fopenmp -flto)

include(CPack)
