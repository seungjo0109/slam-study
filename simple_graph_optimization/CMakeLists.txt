cmake_minimum_required(VERSION 3.5)
project(simple_graph_optimization)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(g2o REQUIRED)

set(OpenGL_GL_PREFERENCE LEGACY)
set(G2O_LIBS
  g2o_core 
  g2o_types_sba 
  g2o_types_slam3d
  g2o_solver_csparse 
  g2o_stuff 
  g2o_csparse_extension
)

set(dependencies
  rclcpp
  std_msgs
  visualization_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/../third_party
)

add_executable(simple_graph_optimization 
  src/simple_graph_optimization.cpp
  src/pose_graph_optimizer.cpp
  src/sim_model.cpp
)
ament_target_dependencies(simple_graph_optimization
  ${dependencies}
)
target_link_libraries(simple_graph_optimization
  ${EIGEN3_LIBS}
  ${G2O_LIBS}
  ${visualization_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

target_include_directories(simple_graph_optimization PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS simple_graph_optimization
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
