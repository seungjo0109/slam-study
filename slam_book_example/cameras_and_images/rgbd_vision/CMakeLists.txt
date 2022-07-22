cmake_minimum_required(VERSION 3.5)
project(rgbd_vision)

# Default to C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin)
set(sophus ../../../third_party/Sophus/)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
    ${sophus}
)

add_executable(rgbd_vision
rgbd_vision.cc
)

target_link_libraries(rgbd_vision
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)
