#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
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

constexpr int yaw_segments = 12;

void anchorYaw(Eigen::MatrixXd &points, double dt, double yaw, double rate,
               double acceleration, double goal) {
  points(0,0) = yaw-dt*rate+dt*dt/3.0*acceleration;
  points(1,0) = yaw-dt*dt/6.0*acceleration;
  points(2,0) = yaw+dt*rate+dt*dt/3.0*acceleration;
  points.bottomRows(3).setConstant(goal);
}

// Sample a quintic handoff-to-rest shape, then impose the exact cubic endpoint
// states. Acceptance uses derivative control hulls, not these shape samples.
NonUniformBspline yawFallback(double duration, double yaw, double rate,
                             double acceleration, double goal) {
  const double dt = duration/yaw_segments;
  const double delta = goal-yaw, v = rate*duration, a = acceleration*duration*duration;
  const double c3 = 10*delta-6*v-1.5*a;
  const double c4 = -15*delta+8*v+1.5*a;
  const double c5 = 6*delta-3*v-0.5*a;
  Eigen::MatrixXd points(yaw_segments+3,1);
  for (int i=0;i<points.rows();++i) {
    const double u = std::clamp((i-1.0)/yaw_segments,0.0,1.0);
    points(i,0) = yaw+v*u+0.5*a*u*u+c3*std::pow(u,3)+c4*std::pow(u,4)+c5*std::pow(u,5);
  }
  anchorYaw(points,dt,yaw,rate,acceleration,goal);
  return NonUniformBspline(points,3,dt);
}

bool yawWithinLimits(const NonUniformBspline &curve, double rate, double acceleration) {
  const auto bounds = curve.derivativeNormBounds();
  return bounds[0]<=rate+1e-8 && bounds[1]<=acceleration+1e-8;
}

bool feasibleCubic(const Eigen::MatrixXd &shape, double dt,
                   const Eigen::Vector3d &position, const Eigen::Vector3d &velocity,
                   const Eigen::Vector3d &acceleration, const Eigen::Vector3d &goal,
                   double max_velocity, double max_acceleration, double minimum_duration,
                   NonUniformBspline &result) {
  if (shape.rows()<6 || shape.cols()!=3 || !shape.allFinite() || !std::isfinite(dt) || dt<=0)
    return false;
  dt=std::max(dt,minimum_duration/(shape.rows()-3));
  for (int attempt=0;attempt<64;++attempt,dt*=1.06) {
    if (dt*(shape.rows()-3)>120.0) break;
    auto points=shape;
    points.row(0)=(position-dt*velocity+dt*dt/3.0*acceleration).transpose();
    points.row(1)=(position-dt*dt/6.0*acceleration).transpose();
    points.row(2)=(position+dt*velocity+dt*dt/3.0*acceleration).transpose();
    // Reach the endpoint at rest, including zero terminal acceleration.
    points.bottomRows(3)=goal.transpose().replicate(3,1);
    NonUniformBspline trial(points,3,dt);
    const auto bounds=trial.derivativeNormBounds();
    if (bounds[0]<=max_velocity+1e-8 && bounds[1]<=max_acceleration+1e-8) {
      result=std::move(trial);
      return true;
    }
  }
  return false;
}

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
  readParameter(node, "uav_model.dynamics_parameters.max_yaw_velocity", pp_.max_yaw_vel_, M_PI/2.0);
  readParameter(node, "uav_model.dynamics_parameters.max_yaw_acceleration", pp_.max_yaw_acc_, M_PI/2.0);
  readParameter(node, "fast_planner.control_points_distance", pp_.ctrl_pt_dist_, -1.0);
  readParameter(node, "bspline_opt.boundary_clearance", pp_.boundary_clearance_, 0.3);
  readParameter(node, "fast_planner.collision_clearance", pp_.collision_clearance_, 0.2);

  // Check parameters
  if (!std::isfinite(pp_.max_vel_) || !std::isfinite(pp_.max_acc_) ||
      !std::isfinite(pp_.ctrl_pt_dist_) || pp_.max_vel_ <= 0.0 || pp_.max_acc_ <= 0.0 || pp_.ctrl_pt_dist_ <= 0.0 ||
      !std::isfinite(pp_.boundary_clearance_) || pp_.boundary_clearance_ < 0.0 ||
      !std::isfinite(pp_.collision_clearance_) || pp_.collision_clearance_ < 0.0)
    throw std::invalid_argument("Fast planner velocity, acceleration, and control-point distance "
                                "parameters must be positive");
  if (!std::isfinite(pp_.max_yaw_vel_) || !std::isfinite(pp_.max_yaw_acc_) ||
      pp_.max_yaw_vel_<=0 || pp_.max_yaw_acc_<=0)
    throw std::invalid_argument("Yaw rate and acceleration limits must be finite and positive");

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

bool FastPlannerManager::planExplorationPositionTraj(const std::vector<Eigen::Vector3d> &tour,
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
  if (cur_vel.norm()>pp_.max_vel_+1e-6 || cur_acc.norm()>pp_.max_acc_+1e-6) {
    RCLCPP_WARN(node_->get_logger(), "Start state exceeds hard derivative limits; stop before replanning");
    return false;
  }

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
  NonUniformBspline feasible_initial;
  bool initial_ok = bspline_degree==3 && feasibleCubic(ctrl_pts,dt,tour.front(),cur_vel,
      cur_acc,tour.back(),pp_.max_vel_,pp_.max_acc_,time_lb,feasible_initial);

  int cost_func = BsplineOptimizer::EXPLORATION_POSITION_PHASE;

  vector<Vector3d> start, end;
  local_data_.init_traj_bspline_.getBoundaryStates(2, 0, start, end);
  bspline_optimizers_[0]->setBoundaryStates(start, end);
  bspline_optimizers_[0]->setTimeLowerBound(time_lb);

  if (need_optimize) {
    // bspline optimizer generate extreme slow trajectory for short tours
    bspline_optimizers_[0]->optimize(ctrl_pts, dt, cost_func);
  }

  NonUniformBspline feasible_position;
  bool optimized_ok = bspline_degree==3 && feasibleCubic(ctrl_pts,dt,tour.front(),cur_vel,
      cur_acc,tour.back(),pp_.max_vel_,pp_.max_acc_,time_lb,feasible_position);
  if (initial_ok) initial_ok = isCurveSafe(feasible_initial,0.0,true);
  if (optimized_ok) optimized_ok = isCurveSafe(feasible_position,0.0,true);
  if (!optimized_ok && !initial_ok) {
    RCLCPP_WARN(node_->get_logger(), "No safe position spline satisfies hard speed/acceleration bounds");
    return false;
  }
  // Time repair can make the optimized candidate much slower than the fitted
  // path. Try the faster feasible candidate first and retain the other for the
  // FSM's mandatory collision-checked fallback.
  const bool use_initial = initial_ok &&
      (!optimized_ok || feasible_initial.getTimeSum() < feasible_position.getTimeSum());
  local_data_.position_traj_ = use_initial ? feasible_initial : feasible_position;
  local_data_.init_traj_bspline_ = use_initial && optimized_ok ? feasible_position :
      (initial_ok ? feasible_initial : feasible_position);
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
  return true;
}

double FastPlannerManager::yawDurationLowerBound(double yaw, double rate,
                                                double acceleration, double goal) const {
  if (!std::isfinite(yaw) || !std::isfinite(rate) || !std::isfinite(acceleration) ||
      !std::isfinite(goal) || std::abs(rate)>pp_.max_yaw_vel_+1e-8 ||
      std::abs(acceleration)>pp_.max_yaw_acc_+1e-8)
    return std::numeric_limits<double>::infinity();
  goal = yaw+std::remainder(goal-yaw,2*M_PI);
  for (double duration=0.1;duration<=120;duration*=1.08) {
    if (yawWithinLimits(yawFallback(duration,yaw,rate,acceleration,goal),
                        pp_.max_yaw_vel_,pp_.max_yaw_acc_))
      return duration;
  }
  return std::numeric_limits<double>::infinity();
}

bool FastPlannerManager::planExplorationYawWaypointsTraj(
    const double &cur_yaw, const double &cur_yaw_vel, const double &goal_yaw,
    const std::vector<Eigen::Vector3d> &yaw_waypts, const std::vector<int> &yaw_waypts_idx,
    double cur_yaw_acc) {
  if (node_ == nullptr || bspline_optimizers_.size() < 2)
    throw std::logic_error("FastPlannerManager must be initialized before yaw planning");
  if (local_data_.duration_ <= 0.0 || !std::isfinite(local_data_.duration_))
    throw std::logic_error("A valid position trajectory is required before yaw planning");
  if (!std::isfinite(cur_yaw) || !std::isfinite(cur_yaw_vel) || !std::isfinite(cur_yaw_acc) || !std::isfinite(goal_yaw) ||
      yaw_waypts.size() != yaw_waypts_idx.size())
    throw std::invalid_argument("Yaw inputs must be finite and waypoint arrays must match");
  for (size_t i=0;i<yaw_waypts.size();++i)
    if (!yaw_waypts[i].allFinite() || yaw_waypts_idx[i]<0 || yaw_waypts_idx[i]>yaw_segments)
      throw std::invalid_argument("Yaw waypoints must be finite and inside the segment range");
  if (std::abs(cur_yaw_vel)>pp_.max_yaw_vel_+1e-8 ||
      std::abs(cur_yaw_acc)>pp_.max_yaw_acc_+1e-8) return false;

  const auto t1 = PlannerClock::now();

  const int seg_num = yaw_segments;
  double dt_yaw = local_data_.duration_ / seg_num; // time of B-spline segment
  const double unwrapped_goal = cur_yaw+std::remainder(goal_yaw-cur_yaw,2*M_PI);
  Eigen::Vector3d start_yaw3d(cur_yaw, 0, 0);
  Eigen::Vector3d end_yaw3d(unwrapped_goal, 0, 0);

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
                                   Eigen::Vector3d(cur_yaw_vel, 0, 0), Eigen::Vector3d(cur_yaw_acc, 0, 0)};
  vector<Eigen::Vector3d> end = {Eigen::Vector3d(end_yaw3d[0], 0, 0), Eigen::Vector3d(0, 0, 0)};
  bspline_optimizers_[1]->setBoundaryStates(start, end);
  bspline_optimizers_[1]->setWaypoints(yaw_waypts, yaw_waypts_idx);
  bspline_optimizers_[1]->optimize(yaw, dt_yaw, cost_func);

  // Optimizer boundary costs are soft. Restore the actual handoff and terminal
  // states before testing the entire curve. Never independently stretch yaw:
  // the position spline must keep the same clock and its exact handoff state.
  anchorYaw(yaw,dt_yaw,cur_yaw,cur_yaw_vel,cur_yaw_acc,unwrapped_goal);
  NonUniformBspline accepted(yaw,3,dt_yaw);
  if (!yawWithinLimits(accepted,pp_.max_yaw_vel_,pp_.max_yaw_acc_)) {
    accepted = yawFallback(local_data_.duration_,cur_yaw,cur_yaw_vel,cur_yaw_acc,unwrapped_goal);
    if (!yawWithinLimits(accepted,pp_.max_yaw_vel_,pp_.max_yaw_acc_)) return false;
  }

  // Update traj info
  local_data_.yaw_traj_ = std::move(accepted);
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
  return true;
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

bool FastPlannerManager::isCurveSafe(NonUniformBspline curve, double start_time, bool log) {
  constexpr double sample_period = 0.02;
  const auto bounds=curve.derivativeNormBounds();
  if (bounds[0]>pp_.max_vel_+1e-6 || bounds[1]>pp_.max_acc_+1e-6) return false;
  // A speed bound limits deviation between checks. Occupied voxel AABBs include
  // discretization uncertainty instead of treating an obstacle as its center.
  const double clearance = pp_.collision_clearance_ + 0.5*pp_.max_vel_*sample_period;
  const double resolution = map_server_->getResolution();
  const auto grid = map_server_->getOccupancyGrid();
  Eigen::Vector3d bmin,bmax;
  map_server_->getBox(bmin,bmax);
  if (clearance>=0.5*(bmax-bmin).minCoeff()) return false;
  const int extent = static_cast<int>(std::ceil(clearance/resolution))+1;
  const double margin=std::max(pp_.boundary_clearance_,clearance);
  const double duration = curve.getTimeSum();
  double time = std::clamp(start_time,0.0,duration);
  const Eigen::Vector3d trusted_start=curve.evaluateDeBoorT(time);
  while (true) {
    const Eigen::Vector3d point = curve.evaluateDeBoorT(time);
    bool safe = point.allFinite() &&
        (point.array() >= bmin.array()+margin-1e-6).all() &&
        (point.array() <= bmax.array()-margin+1e-6).all();
    const char *reason="boundary";
    Eigen::Vector3d obstacle=Eigen::Vector3d::Zero();
    if (safe) {
      const auto state=map_server_->getOccupancy(point);
      // The measured handoff body is already known to be free even when the
      // depth sensor's near-range blind spot leaves its center unobserved.
      safe=state==voxel_mapping::OccupancyType::FREE ||
          (state==voxel_mapping::OccupancyType::UNKNOWN &&
           (point-trusted_start).norm()<=pp_.collision_clearance_);
      reason=state==voxel_mapping::OccupancyType::UNKNOWN ? "unobserved center" : "occupied center";
    }
    if (safe) {
      const auto index=grid->positionToIndex(point);
      for (int x=-extent;x<=extent && safe;++x)
        for (int y=-extent;y<=extent && safe;++y)
          for (int z=-extent;z<=extent && safe;++z) {
            const Eigen::Vector3i neighbor=index+Eigen::Vector3i(x,y,z);
            if (!grid->isInMap(neighbor)) continue;
            const Eigen::Vector3d gap=((grid->indexToPosition(neighbor)-point).cwiseAbs()-
                Eigen::Vector3d::Constant(0.5*resolution)).cwiseMax(0.0);
            if (gap.squaredNorm()>clearance*clearance) continue;
            if (grid->getVoxel(neighbor).value==voxel_mapping::OccupancyType::OCCUPIED) {
              safe=false; reason="occupied neighbor"; obstacle=grid->indexToPosition(neighbor);
            }
          }
    }
    if (!safe) {
      if (log) RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
          "Unsafe curve: %s at (%.3f, %.3f, %.3f), obstacle (%.3f, %.3f, %.3f)",
          reason,point.x(),point.y(),point.z(),obstacle.x(),obstacle.y(),obstacle.z());
      return false;
    }
    if (time>=duration) return true;
    time=std::min(time+sample_period,duration);
  }
}

bool FastPlannerManager::checkTrajCollision() {
  if (node_ == nullptr || !map_server_ || local_data_.duration_ <= 0.0)
    throw std::logic_error("A valid initialized trajectory is required for collision checking");
  return isCurveSafe(local_data_.position_traj_,(node_->now()-local_data_.start_time_).seconds(),true);
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
