#include <memory>

#include "simple_graph_optimization/simple_graph_optimization.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleGraphOptimization>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}