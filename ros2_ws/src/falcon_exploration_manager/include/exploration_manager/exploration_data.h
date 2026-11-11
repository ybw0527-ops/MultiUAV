#ifndef _EXPLORATION_DATA_H_
#define _EXPLORATION_DATA_H_

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "exploration_types.h"
using std::string;

#include <Eigen/Eigen>

#include <falcon_interfaces/msg/bspline.hpp>

using Eigen::Vector3d;
using std::pair;
using std::vector;

namespace fast_planner {
struct FSMData {
  bool triggered_, have_odom_, static_state_;
  vector<string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_; // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;
  double odom_angular_speed_{0.0};

  Eigen::Vector3d start_pos_, start_vel_, start_acc_, start_yaw_; // start state
  vector<Eigen::Vector3d> start_poss;
  falcon_interfaces::msg::Bspline newest_traj_;

  rclcpp::Time fsm_init_time_{0, 0, RCL_ROS_TIME};
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_duration_; // second

  double replan_duration_fast_;
  double replan_duration_default_;
  double replan_duration_slow_;
};

struct ExplorationData {
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME}, finish_time_{0, 0, RCL_ROS_TIME};

  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dormant_frontiers_;
  vector<vector<Vector3d>> tiny_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<Vector3d> views1_, views2_;
  vector<double> yaws_;
  vector<Vector3d> global_tour_;
  vector<Vector3d> frontier_tour_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<vector<double>> n_yaws_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_; // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> n_views1_, n_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_;
  Vector3d next_pos_;
  double next_yaw_;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;

  // Coverage planning
  vector<Vector3d> grid_tour_, last_grid_tour_, grid_tour2_, grid_tour3_;
  vector<pair<int, int>> grid_tour2_cell_centers_id_;
  vector<int> grid_ids_;
  double grid_tour_cost_, last_grid_tour_cost_;

  double mock_uncertainty_, last_loop_time_, last_loop_uncertainty_;

  Position update_bbox_min_, update_bbox_max_;

  vector<Position> trajecotry_;
};

struct ExplorationParam {
  // params
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  string tsp_dir_; // resource dir of tsp solver

  double unknown_penalty_factor_;
  double hybrid_search_radius_;
  bool auto_start_;

  double replan_duration_;
};

struct ExplorationExpData {
  // Time analysis
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME}, finish_time_{0, 0, RCL_ROS_TIME};
  std::vector<double> frontier_times_;
  std::vector<double> total_times_;
  std::vector<double> space_decomp_times_;
  std::vector<double> connectivity_graph_times_;
  std::vector<std::pair<double, double>> cp_times_;
  std::vector<std::pair<double, double>> sop_times_;
};

} // namespace fast_planner

#endif
