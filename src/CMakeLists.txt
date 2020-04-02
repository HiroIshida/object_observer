cmake_minimum_required(VERSION 3.1)
project(test_cmake CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set (CMAKE_CXX_STANDARD 11)
add_library(mylib SHARED process.cpp)
add_executable(a.out tmp.cpp)
target_link_libraries(a.out mylib)
