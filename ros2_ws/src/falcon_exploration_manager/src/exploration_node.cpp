#include <rclcpp/rclcpp.hpp>
#include "exploration_manager/exploration_fsm.h"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("exploration_node");
  try {
    fast_planner::ExplorationFSM fsm;
    fsm.init(*node);
    rclcpp::spin(node);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(node->get_logger(), "Exploration failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
