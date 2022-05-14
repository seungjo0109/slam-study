cmake_minimum_required(VERSION 3.5)
project(localization)

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

find_package(GTSAM REQUIRED)
set(GTSAM_LIBRARIES gtsam)

find_package(Boost 1.50 REQUIRED)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

add_executable(localization
    ${CMAKE_CURRENT_SOURCE_DIR}/src/localization.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/localization_test.cc
)
target_link_libraries(localization
    ${EIGEN3_LIBS}
    gtsam
)