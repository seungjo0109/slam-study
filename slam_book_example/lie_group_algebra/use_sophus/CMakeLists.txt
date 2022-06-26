cmake_minimum_required(VERSION 3.5)
project(use_sophus)

# Default to C++14
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -w")
set(sophus ../../../third_party/Sophus/)

find_package(Eigen3 REQUIRED)
# find_package(Sophus REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    # ${Sophus_INCLUDE_DIRS}
    ${sophus}
)

add_executable(use_sophus
    ${CMAKE_CURRENT_SOURCE_DIR}/use_sophus.cc
)
target_link_libraries(use_sophus
    ${EIGEN3_LIBS}
    # ${Sophus_LIBS}    
)
