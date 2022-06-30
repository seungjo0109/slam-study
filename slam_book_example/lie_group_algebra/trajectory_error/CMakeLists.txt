cmake_minimum_required(VERSION 3.5)
project(trajectory_error)

# Default to C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -w")
set(sophus ../../../third_party/Sophus/)

find_package(Eigen3 REQUIRED)
find_package(Pangolin)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${sophus}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(trajectory_error
    ${CMAKE_CURRENT_SOURCE_DIR}/trajectory_error.cc
)
target_link_libraries(trajectory_error
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)
