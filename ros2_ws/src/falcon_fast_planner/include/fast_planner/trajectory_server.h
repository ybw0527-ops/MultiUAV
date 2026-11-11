#ifndef FALCON_FAST_PLANNER_TRAJECTORY_SERVER_H_
#define FALCON_FAST_PLANNER_TRAJECTORY_SERVER_H_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "falcon_interfaces/msg/bspline.hpp"
#include "falcon_interfaces/msg/position_command.hpp"
#include "bspline/non_uniform_bspline.h"
#include "perception_utils/perception_utils.h"

namespace fast_planner {

class TrajectoryServer : public rclcpp::Node {
public:
  explicit TrajectoryServer(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  bool setTrajectory(const falcon_interfaces::msg::Bspline &message,
                     std::string *error = nullptr);
  bool sampleCommand(const rclcpp::Time &sample_time,
                     falcon_interfaces::msg::PositionCommand &command) const;

  bool hasTrajectory() const;
  double trajectoryDuration() const;
  int64_t trajectoryId() const;

private:
  void bsplineCallback(const falcon_interfaces::msg::Bspline::SharedPtr message);
  void replanCallback(const std_msgs::msg::Int32::SharedPtr message);
  void commandTimerCallback();
  void visualizationTimerCallback();
  void runInitializationStep();
  void publishFieldOfView(const Eigen::Vector3d &position, double yaw);
  void publishExecutedTrajectory();
  void recordCommand(const falcon_interfaces::msg::PositionCommand &command,
                     const rclcpp::Time &time);
  static double pathLength(const std::vector<Eigen::Vector3d> &path);
  Eigen::Vector2d sampleYaw(double relative_time) const;

  mutable std::mutex trajectory_mutex_;
  std::vector<NonUniformBspline> trajectory_;
  double trajectory_duration_{0.0};
  double yaw_duration_{0.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  int64_t trajectory_id_{-1};
  bool received_trajectory_{false};
  bool task_finished_{false};

  falcon_interfaces::msg::PositionCommand command_template_;
  falcon_interfaces::msg::PositionCommand last_command_;
  bool has_last_command_{false};
  double max_velocity_{2.0}, max_acceleration_{2.0};
  double max_yaw_velocity_{1.57079632679}, max_yaw_acceleration_{1.57079632679};
  bool yaw_stop_active_{false};
  double yaw_stop_time_{0.0}, yaw_stop_yaw_{0.0}, yaw_stop_rate_{0.0};
  PerceptionUtils::Ptr perception_utils_;

  Eigen::Vector3d initial_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d initial_offset_{Eigen::Vector3d::Zero()};
  double initial_yaw_{0.0};
  bool perform_initial_motion_{true};
  int initialization_step_{0};
  bool initialization_complete_{false};

  std::vector<Eigen::Vector3d> commanded_positions_;
  std::vector<Eigen::Vector3d> commanded_velocities_;
  rclcpp::Time flight_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time flight_end_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  double jerk_energy_{0.0};

  std::string world_frame_{"world"};
  rclcpp::Subscription<falcon_interfaces::msg::Bspline>::SharedPtr bspline_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr replan_sub_;
  rclcpp::Publisher<falcon_interfaces::msg::PositionCommand>::SharedPtr position_command_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_pub_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr visualization_timer_;
};

} // namespace fast_planner

#endif // FALCON_FAST_PLANNER_TRAJECTORY_SERVER_H_
