#ifndef _EXPLORATION_FSM_H_
#define _EXPLORATION_FSM_H_

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <glog/logging.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <falcon_interfaces/msg/bspline.hpp>
#include <falcon_interfaces/msg/position_command.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class ExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, RTB};

class ExplorationFSM {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  ExplorationFSM(/* args */) {}
  ~ExplorationFSM() {}

  void init(rclcpp::Node &node);
  EXPL_STATE state() const { return state_; }

private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<ExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_{INIT};

  /* ROS utils */
  rclcpp::Node *node_{nullptr};
  rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_, frontier_timer_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr trigger_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<falcon_interfaces::msg::PositionCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr initialization_sub_;
  bool initialization_ready_{false};
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr replan_pub_;
  rclcpp::Publisher<falcon_interfaces::msg::Bspline>::SharedPtr bspline_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr uncertainty_pub_;

  bool frontier_ready_;

  rclcpp::Time trajectory_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time stop_requested_time_{0, 0, RCL_ROS_TIME};
  bool waiting_for_stop_{false};
  bool finish_stop_requested_{false};
  rclcpp::Time stopped_command_stamp_{0,0,RCL_ROS_TIME};
  double publication_margin_{0.2};

  /* helper functions */
  int callExplorationPlanner();
  void stopAndWaitForOdometry(int reason=1);
  bool stopAcknowledged() const;
  void transitState(EXPL_STATE new_state, string pos_call);

  /* ROS functions */
  void FSMCallback();
  void safetyCallback();
  void frontierCallback();
  void triggerCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
  void visualize();
  void clearVisMarker();
};

} // namespace fast_planner

#endif
