cmake_minimum_required(VERSION 3.5)
project(undistort_image)

# Default to C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(undistort_image
    undistort_image.cc
)

target_link_libraries(undistort_image
    ${OpenCV_LIBS}
)
