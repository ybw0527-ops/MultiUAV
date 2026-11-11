#ifndef _PLANNER_MANAGER_H_
#define _PLANNER_MANAGER_H_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "bspline/bspline_optimizer.h"
#include "bspline/non_uniform_bspline.h"
#include "pathfinding/astar.h"
#include "polynomial/polynomial_traj.h"
#include "voxel_mapping/map_server.h"

namespace fast_planner {
struct PlanParameters {
  double max_vel_{0.0}, max_acc_{0.0}; // physical limits
  double ctrl_pt_dist_{0.0};           // distance between adjacent B-spline control points
  double boundary_clearance_{0.3};
};

struct LocalTrajData {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int traj_id_{0};
  double duration_{0.0};
  rclcpp::Time start_time_{0, 0, RCL_ROS_TIME};
  Eigen::Vector3d start_pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d end_pos_{Eigen::Vector3d::Zero()};
  double start_yaw_{0.0}, end_yaw_{0.0};

  PolynomialTraj init_traj_poly_;
  NonUniformBspline init_traj_bspline_;
  NonUniformBspline position_traj_, velocity_traj_, acceleration_traj_;

  PolynomialTraj yaw_init_traj_poly_;
  NonUniformBspline yaw_init_traj_bspline_;
  NonUniformBspline yaw_traj_, yawdot_traj_, yawdotdot_traj_;

  // Bspline optimization input
  vector<Eigen::Vector3d> tour_;
  Eigen::Vector3d cur_vel_, cur_acc_;
  double time_lb_{0.0};
  double cur_yaw_{0.0}, cur_yaw_vel_{0.0}, goal_yaw_{0.0};
  std::vector<Eigen::Vector3d> yaw_waypts_;
  std::vector<int> yaw_waypts_idx_;
};

class FastPlannerManager {
public:
  using Ptr = std::shared_ptr<FastPlannerManager>;
  using ConstPtr = std::shared_ptr<const FastPlannerManager>;

  FastPlannerManager() = default;
  ~FastPlannerManager() = default;

  void init(rclcpp::Node &node);
  void planExplorationPositionTraj(const std::vector<Eigen::Vector3d> &tour,
                                   const Eigen::Vector3d &cur_vel, const Eigen::Vector3d &cur_acc,
                                   const double &time_lb, const bool verbose = false);
  void planExplorationYawWaypointsTraj(const double &cur_yaw, const double &cur_yaw_vel,
                                       const double &goal_yaw,
                                       const std::vector<Eigen::Vector3d> &yaw_waypts,
                                       const std::vector<int> &yaw_waypts_idx);
  bool checkTrajCollision();

  // Save traj_id, duration, evaluated position, velocity, acceleration
  void saveBsplineTraj(const std::string &file_name);

  PlanParameters pp_;
  LocalTrajData local_data_;
  Astar::Ptr path_finder_;
  RayCaster::Ptr caster_;
  voxel_mapping::MapServer::Ptr map_server_;

private:
  void calcNextYaw(const double &last_yaw, double &yaw);

  std::vector<BsplineOptimizer::Ptr> bspline_optimizers_;
  rclcpp::Node *node_{nullptr};
};
} // namespace fast_planner

#endif
