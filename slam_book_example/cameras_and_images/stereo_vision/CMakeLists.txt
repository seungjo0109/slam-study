cmake_minimum_required(VERSION 3.5)
project(stereo_vision)

# Default to C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(stereo_vision
stereo_vision.cc
)

target_link_libraries(stereo_vision
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)
