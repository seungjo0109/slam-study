cmake_minimum_required(VERSION 3.5)
project(g2o_data_fitting)

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

find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog 0.6.0 REQUIRED)
find_package(Ceres REQUIRED)

set(OpenGL_GL_PREFERENCE LEGACY)
set(G2O_LIBS
  g2o_core 
  g2o_types_sba 
  g2o_types_slam3d
  g2o_solver_csparse
  g2o_solver_dense 
  g2o_stuff 
  g2o_csparse_extension
)

include_directories(
    include
    ${EIGEN3_INCLUDE_DIRS}
    /usr/include/suitesparse/
)

add_executable(g2o_data_fitting
    ${CMAKE_CURRENT_SOURCE_DIR}/src/g2o_data_fitting.cc
)
target_link_libraries(g2o_data_fitting
    ${G2O_LIBS}
    ${EIGEN3_LIBS}
    glog::glog
    ceres
)