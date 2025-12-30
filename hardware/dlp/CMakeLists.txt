find_package(hidapi REQUIRED)
qt_add_library(DLP_Driver STATIC)
target_sources(DLP_Driver
    PRIVATE
    src/dlpc350_common.cpp
    src/dlpc350_usb.cpp
    src/dlpc350_api.cpp
    src/dlpc350_BMPParser.cpp
    src/dlpc350_firmware.cpp
    PUBLIC
    FILE_SET HEADERS
    BASE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/inc
    FILES
        inc/dlpc350_api.h
        inc/dlpc350_BMPParser.h
        inc/dlpc350_common.h
        inc/dlpc350_error.h
        inc/dlpc350_firmware.h
        inc/dlpc350_flashDevice.h
        inc/dlpc350_usb.h
        inc/dlpc350_version.h
)
target_include_directories(DLP_Driver PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_link_libraries(DLP_Driver PUBLIC hidapi::hidapi)