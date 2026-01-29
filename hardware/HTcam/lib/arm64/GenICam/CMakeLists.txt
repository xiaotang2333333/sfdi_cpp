add_library(GCBASE INTERFACE)
target_link_directories(GCBASE INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(GCBASE INTERFACE "-l:libGCBase_gcc485_v3_0.so")
target_link_options(GCBASE INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(GENAPI INTERFACE)
target_link_directories(GENAPI INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(GENAPI INTERFACE "-l:libGenApi_gcc485_v3_0.so")
target_link_options(GENAPI INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(LOG INTERFACE)
target_link_directories(LOG INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(LOG INTERFACE "-l:libLog_gcc485_v3_0.so")
target_link_options(LOG INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(MATHPARSER INTERFACE)
target_link_directories(MATHPARSER INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(MATHPARSER INTERFACE "-l:libMathParser_gcc485_v3_0.so")
target_link_options(MATHPARSER INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(NODEMAPDATA INTERFACE)
target_link_directories(NODEMAPDATA INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(NODEMAPDATA INTERFACE "-l:libNodeMapData_gcc485_v3_0.so")
target_link_options(NODEMAPDATA INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(XMLPARSER INTERFACE)
target_link_directories(XMLPARSER INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(XMLPARSER INTERFACE "-l:libXmlParser_gcc485_v3_0.so")
target_link_options(XMLPARSER INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(LOG4CPP INTERFACE)
target_link_directories(LOG4CPP INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(LOG4CPP INTERFACE "-l:liblog4cpp_gcc485_v3_0.so"
)
target_link_options(LOG4CPP INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")