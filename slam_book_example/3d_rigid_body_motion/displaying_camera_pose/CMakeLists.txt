cmake_minimum_required(VERSION 3.5)
project(displaying_camera_pose)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(Pangolin)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${Pangolin_INCLUDE_DIRS}
)

add_executable(displaying_camera_pose
    ${CMAKE_CURRENT_SOURCE_DIR}/displaying_camera_pose.cc
)
target_link_libraries(displaying_camera_pose
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
)
