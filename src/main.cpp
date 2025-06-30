#include <rclcpp/rclcpp.hpp>
#include <hybrid_astar/hybrid_astar_node.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HybridAStarNode>();
  RCLCPP_INFO(node->get_logger(), "Hybrid A* node has been started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}