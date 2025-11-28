add_library(sfdiAlgorithm OBJECT)
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenBLAS CONFIG REQUIRED)
find_package(OpenMP REQUIRED)
find_package(NLopt REQUIRED)
find_package(TIFF REQUIRED)
target_sources(sfdiAlgorithm 
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src/grid_inverse.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/grid_forward.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/model_SFDI.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/lookup.cpp
    PUBLIC 
        FILE_SET HEADERS
        BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/inc
        FILES 
            ${CMAKE_CURRENT_SOURCE_DIR}/inc/grid_inverse.hpp 
            ${CMAKE_CURRENT_SOURCE_DIR}/inc/grid_forward.hpp
            ${CMAKE_CURRENT_SOURCE_DIR}/inc/model_SFDI.hpp 
            ${CMAKE_CURRENT_SOURCE_DIR}/inc/lookup.hpp
)
target_link_libraries(sfdiAlgorithm PUBLIC 
    Eigen3::Eigen
    OpenBLAS::OpenBLAS 
    OpenMP::OpenMP_CXX
    NLopt::nlopt
    TIFF::TIFF
)
