cmake_minimum_required(VERSION 3.5)
project(eigen_matrix)

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

add_executable(eigen_matrix
    ${CMAKE_CURRENT_SOURCE_DIR}/eigen_matrix.cc
)
target_link_libraries(eigen_matrix
    ${EIGEN3_LIBS}
)
