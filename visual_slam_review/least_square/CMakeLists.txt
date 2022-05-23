cmake_minimum_required(VERSION 2.8)
project(least_square)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14")

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(gauss_newton
    ${CMAKE_CURRENT_SOURCE_DIR}/gauss_newton.cc
)
target_link_libraries(gauss_newton
    ${EIGEN3_LIBS}
)