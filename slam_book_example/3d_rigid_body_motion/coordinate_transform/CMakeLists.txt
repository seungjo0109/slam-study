cmake_minimum_required(VERSION 3.5)
project(coordinate_transform)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
)

add_executable(coordinate_transform
    ${CMAKE_CURRENT_SOURCE_DIR}/coordinate_transform.cc
)
target_link_libraries(coordinate_transform
    ${EIGEN3_LIBS}
)
