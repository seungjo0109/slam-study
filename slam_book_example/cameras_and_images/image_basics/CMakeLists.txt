cmake_minimum_required(VERSION 3.5)
project(images_basics)

# Default to C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(images_basics
    images_basics.cc
)

target_link_libraries(images_basics
    ${OpenCV_LIBS}
)
