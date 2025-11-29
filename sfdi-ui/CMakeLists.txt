find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

qt_add_library(sfdiUI OBJECT)

set_target_properties(sfdiUI PROPERTIES
    AUTOUIC_SEARCH_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/ui"
)
target_sources(sfdiUI
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/ui/MainWindow.ui
    ${CMAKE_CURRENT_SOURCE_DIR}/src/MainWindow.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/inc/MainWindow.h
)


target_include_directories(sfdiUI
    PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

target_link_libraries(sfdiUI
    PUBLIC
        Qt6::Core
        Qt6::Widgets
        Hardware
)