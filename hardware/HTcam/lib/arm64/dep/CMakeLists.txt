add_library(IMAGECONVERT INTERFACE)
target_link_directories(IMAGECONVERT INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(IMAGECONVERT INTERFACE "-l:libImageConvert.so")
target_link_options(IMAGECONVERT INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(IMAGESAVE INTERFACE)
target_link_directories(IMAGESAVE INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(IMAGESAVE INTERFACE "-l:libImageSave.so")
target_link_options(IMAGESAVE INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(TINYXML INTERFACE)
target_link_directories(TINYXML INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(TINYXML INTERFACE "-l:libTinyXml.so")
target_link_options(TINYXML INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(IIMAGEPROCESSING64 INTERFACE)
target_link_directories(IIMAGEPROCESSING64 INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(IIMAGEPROCESSING64 INTERFACE "-l:libiImageProcessing64.so")
target_link_options(IIMAGEPROCESSING64 INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

add_library(LOG4CPP_DEP INTERFACE)
target_link_directories(LOG4CPP_DEP INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(LOG4CPP_DEP INTERFACE "-l:liblog4cpp.so")
target_link_options(LOG4CPP_DEP INTERFACE "LINKER:-rpath,${CMAKE_CURRENT_SOURCE_DIR}")

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/liblog4cpp.so" AND NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/liblog4cpp.so.5")
	file(CREATE_LINK
		"${CMAKE_CURRENT_SOURCE_DIR}/liblog4cpp.so"
		"${CMAKE_CURRENT_SOURCE_DIR}/liblog4cpp.so.5"
		SYMBOLIC
	)
endif()

target_link_libraries(IMAGECONVERT INTERFACE IIMAGEPROCESSING64)
