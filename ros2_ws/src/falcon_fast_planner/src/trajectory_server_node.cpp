#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "fast_planner/trajectory_server.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fast_planner::TrajectoryServer>());
  rclcpp::shutdown();
  return 0;
}
