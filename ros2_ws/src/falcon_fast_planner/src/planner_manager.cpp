#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>

#include "fast_planner/planner_manager.h"

namespace fast_planner {
namespace {

template <typename T>
void readParameter(rclcpp::Node &node, const std::string &name, T &value,
                   const T &default_value) {
  if (node.has_parameter(name)) {
    node.get_parameter(name, value);
  } else {
    value = node.declare_parameter<T>(name, default_value);
  }
}

using PlannerClock = std::chrono::steady_clock;

double elapsedMilliseconds(const PlannerClock::time_point &start) {
  return std::chrono::duration<double, std::milli>(PlannerClock::now() - start).count();
}

template <typename Derived> std::string eigenToString(const Eigen::MatrixBase<Derived> &value) {
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

} // namespace

void FastPlannerManager::init(rclcpp::Node &node) {
  node_ = &node;
  // Load parameters
  readParameter(node, "uav_model.dynamics_parameters.max_linear_velocity", pp_.max_vel_, -1.0);
  readParameter(node, "uav_model.dynamics_parameters.max_linear_acceleration", pp_.max_acc_,
                -1.0);
  readParameter(node, "fast_planner.control_points_distance", pp_.ctrl_pt_dist_, -1.0);

  // Check parameters
  if (pp_.max_vel_ <= 0.0 || pp_.max_acc_ <= 0.0 || pp_.ctrl_pt_dist_ <= 0.0)
    throw std::invalid_argument("Fast planner velocity, acceleration, and control-point distance "
                                "parameters must be positive");

  local_data_.traj_id_ = 0;
  map_server_ = std::make_shared<voxel_mapping::MapServer>(node);

  double resolution = map_server_->getResolution();
  Eigen::Vector3d origin, size;
  map_server_->getRegion(origin, size);
  caster_ = std::make_shared<RayCaster>();
  caster_->setParams(resolution, origin);

  path_finder_ = std::make_shared<Astar>();
  path_finder_->init(node, map_server_);

  // 2 Bspline optimizers for position and yaw
  bspline_optimizers_.resize(2);
  for (size_t i = 0; i < bspline_optimizers_.size(); ++i) {
    bspline_optimizers_[i] = std::make_shared<BsplineOptimizer>();
    bspline_optimizers_[i]->setParam(node);
    bspline_optimizers_[i]->setMap(map_server_);
    if (i == 0)
      // Position optimizer physical limits
      bspline_optimizers_[i]->setPhysicalLimits(pp_.max_vel_, pp_.max_acc_);
  }
}

void FastPlannerManager::planExplorationPositionTraj(const std::vector<Eigen::Vector3d> &tour,
                                                     const Eigen::Vector3d &cur_vel,
                                                     const Eigen::Vector3d &cur_acc,
                                                     const double &time_lb, const bool verbose) {
  if (node_ == nullptr || !map_server_ || bspline_optimizers_.size() < 2)
    throw std::logic_error("FastPlannerManager must be initialized before planning");
  if (tour.size() < 2)
    throw std::invalid_argument("Trajectory planning requires at least two tour points");
  for (size_t i = 1; i < tour.size(); ++i) {
    if (!tour[i].allFinite() || (tour[i] - tour[i - 1]).norm() <= 1e-6)
      throw std::invalid_argument("Tour points must be finite and consecutive points distinct");
  }
  if (!tour.front().allFinite() || !cur_vel.allFinite() || !cur_acc.allFinite() ||
      !std::isfinite(time_lb) || time_lb < 0.0)
    throw std::invalid_argument("Trajectory planning inputs must be finite and time bound nonnegative");

  const auto t1 = PlannerClock::now();

  Eigen::Vector3d cur_vel_bound = cur_vel;
  for (int i = 0; i < 3; ++i) {
    cur_vel_bound(i) = std::min(pp_.max_vel_, std::max(-pp_.max_vel_, cur_vel(i)));
  }

  Eigen::Vector3d cur_acc_bound = cur_acc;
  for (int i = 0; i < 3; ++i) {
    cur_acc_bound(i) = std::min(pp_.max_acc_, std::max(-pp_.max_acc_, cur_acc(i)));
  }

  if (verbose) {
    // Print current states
    std::cout << "cur_vel: " << cur_vel.transpose() << std::endl;
    std::cout << "cur_vel_bound: " << cur_vel_bound.transpose() << std::endl;
    std::cout << "cur_acc: " << cur_acc.transpose() << std::endl;
    std::cout << "cur_acc_bound: " << cur_acc_bound.transpose() << std::endl;

    // Print tour
    std::cout << "tour size: " << tour.size() << std::endl;
    for (size_t i = 0; i < tour.size(); ++i) {
      std::cout << "tour[" << i << "]: " << tour[i].transpose() << std::endl;
    }
  }

  // Generate traj through waypoints-based method
  const int pt_num = static_cast<int>(tour.size());
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i)
    pos.row(i) = tour[i];

  Eigen::VectorXd times(pt_num - 1); // time for each segment on tour
  Eigen::Vector3d first_seg_vel = Eigen::Vector3d::Zero();
  for (int i = 0; i < pt_num - 1; ++i) {
    Eigen::Vector3d time_xyz = Eigen::Vector3d::Zero();
    Eigen::Vector3d dir = (pos.row(i + 1) - pos.row(i)).normalized();
    for (int j = 0; j < 3; ++j) {
      const double delta = pos(i + 1, j) - pos(i, j);
      const double len = std::fabs(delta);
      const double vc_bounded = cur_vel_bound(j);
      const double len_threshold =
          (std::pow(pp_.max_vel_, 2) - std::pow(vc_bounded, 2)) / (2 * pp_.max_acc_);
      time_xyz(j) = len / pp_.max_vel_;
      if (i == 0) {
        // consider cur_vel for first segment
        if (len < len_threshold) {
          const double vc = vc_bounded;
          double t = (std::sqrt(std::pow(vc, 2) + 2 * pp_.max_acc_ * 0.5 * len) -
                      std::fabs(vc)) /
                     (pp_.max_acc_ * 0.5);
          if (vc * dir(j) < 0) {
            t += 2 * std::fabs(vc) / (pp_.max_acc_ * 0.5);
          }
          time_xyz(j) = t;
        } else {
          const double vc = vc_bounded;
          double t = std::pow(pp_.max_vel_ - std::fabs(vc), 2) /
                     (2 * pp_.max_vel_ * pp_.max_acc_);
          if (vc * dir(j) < 0) {
            t += 2 * std::fabs(vc) / pp_.max_acc_;
          }
          time_xyz(j) = len / pp_.max_vel_ + t;
        }
        if (delta < 0) {
          first_seg_vel(j) = vc_bounded - pp_.max_acc_ * time_xyz(j);
        } else {
          first_seg_vel(j) = vc_bounded + pp_.max_acc_ * time_xyz(j);
        }
        first_seg_vel(j) =
            std::max(-pp_.max_vel_, std::min(pp_.max_vel_, first_seg_vel(j)));
      }
      if (i == pt_num - 2) {
        // consider vel change for last segment to zero
        const double vc = (i == 0) ? first_seg_vel(j)
                                   : (delta < 0 ? -pp_.max_vel_ : pp_.max_vel_);
        double t = std::pow(std::fabs(vc), 2) / (2 * pp_.max_vel_ * pp_.max_acc_);
        if (vc * dir(j) < 0) {
          t += 2 * std::fabs(vc) / pp_.max_acc_;
        }
        time_xyz(j) += t;
      }
    }
    times(i) = time_xyz.maxCoeff();
  }

  // Print times
  if (verbose) {
    for (int i = 0; i < pt_num - 1; ++i) {
      std::cout << "times[" << i << "]: " << times(i) << std::endl;
    }
  }

  double times_sum = times.sum();
  if (!std::isfinite(times_sum) || times_sum <= 0.0)
    throw std::runtime_error("Failed to compute positive trajectory segment times");
  if (times_sum < time_lb) {
    double scale = time_lb / times_sum;
    times *= scale;
  }

  bool need_optimize = true;

  bool is_tour_straight_line = false;
  if (tour.size() == 3) {
    Eigen::Vector3d dir1 = (tour[1] - tour[0]).normalized();
    Eigen::Vector3d dir2 = (tour[2] - tour[1]).normalized();
    if (dir1.dot(dir2) > 1 - 1e-3) {
      is_tour_straight_line = true;
    }
  }
  if (is_tour_straight_line) {
    PolynomialTraj::oneSegmentTraj(tour[0], tour[2], cur_vel_bound, Eigen::Vector3d::Zero(),
                                   cur_acc_bound, Eigen::Vector3d::Zero(), times.sum(), local_data_.init_traj_poly_);
  } else {
    PolynomialTraj::waypointsTraj(pos, cur_vel_bound, Eigen::Vector3d::Zero(), cur_acc_bound,
                                  Eigen::Vector3d::Zero(), times, local_data_.init_traj_poly_);
  }
  PolynomialTraj &init_traj = local_data_.init_traj_poly_;

  if (tour.size() == 3 && (tour[1] - tour[0]).norm() + (tour[2] - tour[1]).norm() < 1.0) {
    RCLCPP_WARN(node_->get_logger(), "Short path, no need to optimize");
    need_optimize = false;

    PolynomialTraj::oneSegmentTraj(tour[0], tour[2], cur_vel_bound, Eigen::Vector3d::Zero(),
                                   cur_acc_bound, Eigen::Vector3d::Zero(), times.sum(), init_traj);

    // Hack for short tour glitch
    double tour_len = (tour[1] - tour[0]).norm() + (tour[2] - tour[1]).norm();
    if (init_traj.getLength() > 5.0 * tour_len) {
      RCLCPP_WARN(node_->get_logger(), "Short tour glitch detected; recalculating trajectory");
      // Static state
      for (int i = 0; i < pt_num - 1; ++i) {
        times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);
      }
      double times_sum = times.sum();
      if (times_sum < time_lb) {
        double scale = time_lb / times_sum;
        times *= scale;
      }
      PolynomialTraj::waypointsTraj(pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                    Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), times,
                                    local_data_.init_traj_poly_);
    }
  }

  // B-spline-based optimization
  vector<Vector3d> points, boundary_deri;
  double duration = init_traj.getTotalTime();
  int seg_num = init_traj.getLength() / pp_.ctrl_pt_dist_;
  seg_num = std::max(8, seg_num);
  double dt = duration / double(seg_num);

  // std::cout << "duration: " << duration << ", seg_num: " << seg_num << ", dt: " << dt <<
  // std::endl;

  for (double ts = 0.0; ts <= duration + 1e-4; ts += dt)
    points.push_back(init_traj.evaluate(ts, 0));
  // Evaluate velocity at start and end
  boundary_deri.push_back(init_traj.evaluate(0.0, 1));
  boundary_deri.push_back(init_traj.evaluate(duration, 1));
  // Evaluate acceleration at start and end
  boundary_deri.push_back(init_traj.evaluate(0.0, 2));
  boundary_deri.push_back(init_traj.evaluate(duration, 2));

  Eigen::MatrixXd ctrl_pts;
  int bspline_degree = bspline_optimizers_[0]->getBsplineDegree();
  NonUniformBspline::parameterizeToBspline(dt, points, boundary_deri, bspline_degree, ctrl_pts);
  local_data_.init_traj_bspline_ = NonUniformBspline(ctrl_pts, bspline_degree, dt);

  int cost_func = BsplineOptimizer::EXPLORATION_POSITION_PHASE;

  vector<Vector3d> start, end;
  local_data_.init_traj_bspline_.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  if (need_optimize) {
    // bspline optimizer generate extreme slow trajectory for short tours
    bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func);
  }

  local_data_.position_traj_.setUniformBspline(ctrl_pts, bspline_degree, dt);
  local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
  local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

  local_data_.duration_ = local_data_.position_traj_.getTimeSum();
  local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
  local_data_.end_pos_ = local_data_.position_traj_.evaluateDeBoorT(local_data_.duration_);

  local_data_.traj_id_ += 1;

  local_data_.tour_ = tour;
  local_data_.cur_vel_ = cur_vel;
  local_data_.cur_acc_ = cur_acc;
  local_data_.time_lb_ = time_lb;

  if (verbose) {
    // log trajectory data for debugging
    RCLCPP_INFO(node_->get_logger(),
                "Position trajectory planned: id=%d duration=%.3f start=(%.3f, %.3f, %.3f) "
                "end=(%.3f, %.3f, %.3f) optimized=%s",
                local_data_.traj_id_, local_data_.duration_, local_data_.start_pos_.x(),
                local_data_.start_pos_.y(), local_data_.start_pos_.z(), local_data_.end_pos_.x(),
                local_data_.end_pos_.y(), local_data_.end_pos_.z(), need_optimize ? "true" : "false");
    // log sampled velocity
    std::vector<double> vel_x, vel_y, vel_z;
    for (double t = 0.0; t <= local_data_.duration_; t += 0.1) {
      Eigen::Vector3d vel = local_data_.velocity_traj_.evaluateDeBoorT(t);
      vel_x.push_back(vel(0));
      vel_y.push_back(vel(1));
      vel_z.push_back(vel(2));
    }
    std::string vel_x_str =
        std::accumulate(vel_x.begin(), vel_x.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    std::string vel_y_str =
        std::accumulate(vel_y.begin(), vel_y.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    std::string vel_z_str =
        std::accumulate(vel_z.begin(), vel_z.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    RCLCPP_INFO(node_->get_logger(), "Sampled velocity x: %s", vel_x_str.c_str());
    RCLCPP_INFO(node_->get_logger(), "Sampled velocity y: %s", vel_y_str.c_str());
    RCLCPP_INFO(node_->get_logger(), "Sampled velocity z: %s", vel_z_str.c_str());

    // log sampled acceleration
    std::vector<double> acc_x, acc_y, acc_z;
    for (double t = 0.0; t <= local_data_.duration_; t += 0.1) {
      Eigen::Vector3d acc = local_data_.acceleration_traj_.evaluateDeBoorT(t);
      acc_x.push_back(acc(0));
      acc_y.push_back(acc(1));
      acc_z.push_back(acc(2));
    }
    std::string acc_x_str =
        std::accumulate(acc_x.begin(), acc_x.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    std::string acc_y_str =
        std::accumulate(acc_y.begin(), acc_y.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    std::string acc_z_str =
        std::accumulate(acc_z.begin(), acc_z.end(), std::string(),
                        [](std::string &ss, double d) { return ss + std::to_string(d) + ","; });
    RCLCPP_INFO(node_->get_logger(), "Sampled acceleration x: %s", acc_x_str.c_str());
    RCLCPP_INFO(node_->get_logger(), "Sampled acceleration y: %s", acc_y_str.c_str());
    RCLCPP_INFO(node_->get_logger(), "Sampled acceleration z: %s", acc_z_str.c_str());

    // log start position difference
    Eigen::Vector3d start_pos_diff = local_data_.position_traj_.evaluateDeBoorT(0.0) - tour[0];
    RCLCPP_INFO(node_->get_logger(), "Start position difference: %s",
                eigenToString(start_pos_diff.transpose()).c_str());

    // log start velocity difference
    Eigen::Vector3d start_vel_diff = local_data_.velocity_traj_.evaluateDeBoorT(0.0) - cur_vel;
    RCLCPP_INFO(node_->get_logger(), "Start velocity difference: %s",
                eigenToString(start_vel_diff.transpose()).c_str());

    // log start acceleration difference
    Eigen::Vector3d start_acc_diff = local_data_.acceleration_traj_.evaluateDeBoorT(0.0) - cur_acc;
    RCLCPP_INFO(node_->get_logger(), "Start acceleration difference: %s",
                eigenToString(start_acc_diff.transpose()).c_str());
  }

  RCLCPP_INFO(node_->get_logger(), "Position trajectory optimization time: %.3f ms",
              elapsedMilliseconds(t1));
}

void FastPlannerManager::planExplorationYawWaypointsTraj(
    const double &cur_yaw, const double &cur_yaw_vel, const double &goal_yaw,
    const std::vector<Eigen::Vector3d> &yaw_waypts, const std::vector<int> &yaw_waypts_idx) {
  if (node_ == nullptr || bspline_optimizers_.size() < 2)
    throw std::logic_error("FastPlannerManager must be initialized before yaw planning");
  if (local_data_.duration_ <= 0.0 || !std::isfinite(local_data_.duration_))
    throw std::logic_error("A valid position trajectory is required before yaw planning");
  if (!std::isfinite(cur_yaw) || !std::isfinite(cur_yaw_vel) || !std::isfinite(goal_yaw) ||
      yaw_waypts.size() != yaw_waypts_idx.size())
    throw std::invalid_argument("Yaw inputs must be finite and waypoint arrays must match");

  const auto t1 = PlannerClock::now();

  const int seg_num = 12;
  double dt_yaw = local_data_.duration_ / seg_num; // time of B-spline segment
  Eigen::Vector3d start_yaw3d(cur_yaw, 0, 0);
  Eigen::Vector3d end_yaw3d(goal_yaw, 0, 0);

  // Yaw traj control points
  Eigen::MatrixXd yaw(seg_num + 3, 1);
  yaw.setZero();

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw, 1.0, 0.0, -(1 / 6.0) * dt_yaw * dt_yaw,
      1.0, dt_yaw, (1 / 3.0) * dt_yaw * dt_yaw;

  // Initial state
  yaw.block<3, 1>(0, 0) = states2pts * start_yaw3d;
  // Final state
  yaw.block<3, 1>(seg_num, 0) = states2pts * end_yaw3d;

  // Call B-spline optimization solver
  int cost_func = BsplineOptimizer::EXPLORATION_YAW_PHASE;

  vector<Eigen::Vector3d> start = {Eigen::Vector3d(start_yaw3d[0], start_yaw3d[1], start_yaw3d[2]),
                                   Eigen::Vector3d(cur_yaw_vel, 0, 0), Eigen::Vector3d(0, 0, 0)};
  vector<Eigen::Vector3d> end = {Eigen::Vector3d(end_yaw3d[0], 0, 0), Eigen::Vector3d(0, 0, 0)};
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->setWaypoints(yaw_waypts, yaw_waypts_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func);

  // Update traj info
  local_data_.yaw_traj_.setUniformBspline(yaw, 3, dt_yaw);
  local_data_.yawdot_traj_ = local_data_.yaw_traj_.getDerivative();
  local_data_.yawdotdot_traj_ = local_data_.yawdot_traj_.getDerivative();

  local_data_.start_yaw_ = local_data_.yaw_traj_.evaluateDeBoorT(0.0)[0];
  local_data_.end_yaw_ = local_data_.yaw_traj_.evaluateDeBoorT(local_data_.duration_)[0];

  local_data_.cur_yaw_ = cur_yaw;
  local_data_.cur_yaw_vel_ = cur_yaw_vel;
  local_data_.goal_yaw_ = goal_yaw;
  local_data_.yaw_waypts_ = yaw_waypts;
  local_data_.yaw_waypts_idx_ = yaw_waypts_idx;

  RCLCPP_INFO(node_->get_logger(), "Yaw trajectory optimization time: %.3f ms",
              elapsedMilliseconds(t1));
}

void FastPlannerManager::calcNextYaw(const double &last_yaw, double &yaw) {
  // round yaw to [-PI, PI]
  double round_last = last_yaw;
  while (round_last < -M_PI) {
    round_last += 2 * M_PI;
  }
  while (round_last > M_PI) {
    round_last -= 2 * M_PI;
  }

  double diff = yaw - round_last;
  if (fabs(diff) <= M_PI) {
    yaw = last_yaw + diff;
  } else if (diff > M_PI) {
    yaw = last_yaw + diff - 2 * M_PI;
  } else if (diff < -M_PI) {
    yaw = last_yaw + diff + 2 * M_PI;
  }
}

bool FastPlannerManager::checkTrajCollision() {
  if (node_ == nullptr || !map_server_ || local_data_.duration_ <= 0.0)
    throw std::logic_error("A valid initialized trajectory is required for collision checking");
  double t_now = (node_->now() - local_data_.start_time_).seconds();
  t_now = std::clamp(t_now, 0.0, local_data_.duration_);

  Eigen::Vector3d cur_pt = local_data_.position_traj_.evaluateDeBoorT(t_now);
  double radius = 0.0;
  Eigen::Vector3d fut_pt;
  double fut_t = 0.02;

  while (radius < 6.0 && t_now + fut_t < local_data_.duration_) {
    fut_pt = local_data_.position_traj_.evaluateDeBoorT(t_now + fut_t);
    if (map_server_->getOccupancy(fut_pt) == voxel_mapping::OccupancyType::OCCUPIED) {
      RCLCPP_ERROR(node_->get_logger(), "Collision detected at (%.2f, %.2f, %.2f)", fut_pt.x(),
                   fut_pt.y(), fut_pt.z());
      return false;
    }
    radius = (fut_pt - cur_pt).norm();
    fut_t += 0.02;
  }

  return true;
}

void FastPlannerManager::saveBsplineTraj(const std::string &file_name) {
  std::ofstream file(file_name);
  if (!file.is_open()) {
    if (node_ != nullptr)
      RCLCPP_ERROR(node_->get_logger(), "Failed to open B-spline file %s", file_name.c_str());
    return;
  }

  // Save bspline parameters to file
  file << "TRAJ_ID: " << local_data_.traj_id_ << std::endl;
  file << "DURATION: " << local_data_.duration_ << std::endl;
  file << "START_POS: " << local_data_.start_pos_.transpose() << std::endl;
  file << "END_POS: " << local_data_.end_pos_.transpose() << std::endl;
  file << "START_YAW: " << local_data_.start_yaw_ << std::endl;
  file << "END_YAW: " << local_data_.end_yaw_ << std::endl;
  file << "TOUR: " << std::endl;
  for (size_t i = 0; i < local_data_.tour_.size(); ++i) {
    file << local_data_.tour_[i].transpose() << std::endl;
  }
  file << "CUR_VEL: " << local_data_.cur_vel_.transpose() << std::endl;
  file << "CUR_ACC: " << local_data_.cur_acc_.transpose() << std::endl;
  file << "TIME_LB: " << local_data_.time_lb_ << std::endl;
  file << "CUR_YAW: " << local_data_.cur_yaw_ << std::endl;
  file << "CUR_YAW_VEL: " << local_data_.cur_yaw_vel_ << std::endl;
  file << "GOAL_YAW: " << local_data_.goal_yaw_ << std::endl;
  file << "YAW_WAYPTS: " << std::endl;
  for (size_t i = 0; i < local_data_.yaw_waypts_.size(); ++i) {
    file << local_data_.yaw_waypts_[i].transpose() << std::endl;
  }
  file << "YAW_WAYPTS_IDX: " << std::endl;
  for (size_t i = 0; i < local_data_.yaw_waypts_idx_.size(); ++i) {
    file << local_data_.yaw_waypts_idx_[i] << std::endl;
  }
  const double dt = 0.02;
  for (double t = 0.0; t < local_data_.duration_; t += dt) {
    Eigen::Vector3d pos = local_data_.position_traj_.evaluateDeBoorT(t);
    Eigen::Vector3d vel = local_data_.velocity_traj_.evaluateDeBoorT(t);
    Eigen::Vector3d acc = local_data_.acceleration_traj_.evaluateDeBoorT(t);
    double yaw = local_data_.yaw_traj_.evaluateDeBoorT(t)[0];

    file << t << " " << pos.x() << " " << pos.y() << " " << pos.z() << " " << vel.x() << " "
         << vel.y() << " " << vel.z() << " " << acc.x() << " " << acc.y() << " " << acc.z() << " "
         << yaw << std::endl;
  }

  file.close();
}

} // namespace fast_planner
