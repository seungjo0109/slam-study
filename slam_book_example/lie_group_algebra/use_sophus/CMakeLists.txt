cmake_minimum_required(VERSION 3.5)
project(use_sophus)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
find_package(sophus REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)

add_executable(use_sophus
    ${CMAKE_CURRENT_SOURCE_DIR}/use_sophus.cc
)
target_link_libraries(use_sophus
    ${EIGEN3_LIBS}    
)
