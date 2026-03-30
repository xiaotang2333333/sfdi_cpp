find_package(Qt6 REQUIRED COMPONENTS Core Widgets)
find_package(Eigen3 REQUIRED)
qt_add_library(Hardware OBJECT)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/dlp)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/HTcam)
target_sources(Hardware
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/CamControl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/CamControl.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/DlpControl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/DlpControl.hpp
)
add_dependencies(Hardware HTSDK DLP_Driver)
target_include_directories(Hardware
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)


target_link_libraries(Hardware
    PUBLIC
    Qt6::Core
    Qt6::Widgets
    Eigen3::Eigen
    HTSDK
    DLP_Driver
)
