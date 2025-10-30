set(VCPKG_TARGET_TRIPLET x64-linux-dynamic)
set(VCPKG_HOST_TRIPLET=x64-linux-dynamic)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")