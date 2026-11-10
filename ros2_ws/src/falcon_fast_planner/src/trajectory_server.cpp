#include "fast_planner/trajectory_server.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <geometry_msgs/msg/point.hpp>

namespace fast_planner {
namespace {

bool finitePoint(const geometry_msgs::msg::Point &point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double normalizeYaw(double yaw) {
  return std::remainder(yaw, 2.0 * M_PI);
}

} // namespace

TrajectoryServer::TrajectoryServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("trajectory_server", options) {
  world_frame_ = declare_parameter<std::string>("traj_server.world_frame", world_frame_);
  perform_initial_motion_ =
      declare_parameter<bool>("traj_server.perform_initial_motion", perform_initial_motion_);

  initial_position_.x() = declare_parameter<double>("map_config.init_x", 0.0);
  initial_position_.y() = declare_parameter<double>("map_config.init_y", 0.0);
  initial_position_.z() = declare_parameter<double>("map_config.init_z", 0.0);
  initial_yaw_ = declare_parameter<double>("map_config.init_yaw", 0.0);

  Eigen::Vector3d body_offset;
  body_offset.x() = declare_parameter<double>("traj_server.init_dx", 1.0);
  body_offset.y() = declare_parameter<double>("traj_server.init_dy", 0.0);
  body_offset.z() = declare_parameter<double>("traj_server.init_dz", 1.0);
  Eigen::Matrix3d initial_rotation;
  initial_rotation << std::cos(initial_yaw_), -std::sin(initial_yaw_), 0.0,
      std::sin(initial_yaw_), std::cos(initial_yaw_), 0.0, 0.0, 0.0, 1.0;
  initial_offset_ = initial_rotation * body_offset;

  const auto kx = declare_parameter<std::vector<double>>("traj_server.kx", {5.7, 5.7, 6.2});
  const auto kv = declare_parameter<std::vector<double>>("traj_server.kv", {3.4, 3.4, 4.0});
  if (kx.size() != 3 || kv.size() != 3)
    throw std::invalid_argument("traj_server.kx and traj_server.kv must contain three values");
  for (size_t index = 0; index < 3; ++index) {
    if (!std::isfinite(kx[index]) || !std::isfinite(kv[index]) || kx[index] < 0.0 ||
        kv[index] < 0.0)
      throw std::invalid_argument("Trajectory controller gains must be finite and nonnegative");
    command_template_.kx[index] = kx[index];
    command_template_.kv[index] = kv[index];
  }

  command_template_.header.frame_id = world_frame_;
  command_template_.trajectory_flag =
      falcon_interfaces::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  command_template_.trajectory_id = 0;
  command_template_.position.x = initial_position_.x();
  command_template_.position.y = initial_position_.y();
  command_template_.position.z = initial_position_.z();
  command_template_.yaw = initial_yaw_;

  perception_utils_ = std::make_shared<PerceptionUtils>(*this);

  position_command_pub_ = create_publisher<falcon_interfaces::msg::PositionCommand>(
      "planning/pos_cmd", rclcpp::QoS(50));
  fov_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "planning/position_cmd_vis", rclcpp::QoS(10));
  trajectory_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "planning/travel_traj", rclcpp::QoS(10));
  bspline_sub_ = create_subscription<falcon_interfaces::msg::Bspline>(
      "planning/bspline", rclcpp::QoS(10),
      std::bind(&TrajectoryServer::bsplineCallback, this, std::placeholders::_1));
  replan_sub_ = create_subscription<std_msgs::msg::Int32>(
      "planning/replan", rclcpp::QoS(10),
      std::bind(&TrajectoryServer::replanCallback, this, std::placeholders::_1));

  if (!perform_initial_motion_) {
    command_template_.position.x += initial_offset_.x();
    command_template_.position.y += initial_offset_.y();
    command_template_.position.z += initial_offset_.z();
    initialization_complete_ = true;
  }

  using namespace std::chrono_literals;
  command_timer_ = create_wall_timer(10ms, std::bind(&TrajectoryServer::commandTimerCallback, this));
  visualization_timer_ =
      create_wall_timer(250ms, std::bind(&TrajectoryServer::visualizationTimerCallback, this));
}

bool TrajectoryServer::setTrajectory(const falcon_interfaces::msg::Bspline &message,
                                     std::string *error) {
  try {
    if (message.traj_id < 0 ||
        static_cast<uint64_t>(message.traj_id) > std::numeric_limits<uint32_t>::max())
      throw std::invalid_argument("Trajectory ID must fit the PositionCommand uint32 field");
    if (message.order < 1 || message.pos_pts.size() <= static_cast<size_t>(message.order))
      throw std::invalid_argument("Position B-spline has too few control points");
    const size_t expected_knots =
        message.pos_pts.size() + static_cast<size_t>(message.order) + 1U;
    if (message.knots.size() != expected_knots)
      throw std::invalid_argument("Position B-spline knot count does not match its order");
    if (message.yaw_pts.size() < 4 || !std::isfinite(message.yaw_dt) || message.yaw_dt <= 0.0)
      throw std::invalid_argument("Yaw B-spline requires four points and a positive interval");
    for (const auto &point : message.pos_pts) {
      if (!finitePoint(point))
        throw std::invalid_argument("Position control points must be finite");
    }
    for (size_t index = 0; index < message.knots.size(); ++index) {
      if (!std::isfinite(message.knots[index]) ||
          (index > 0 && message.knots[index] < message.knots[index - 1]))
        throw std::invalid_argument("B-spline knots must be finite and nondecreasing");
    }
    for (const double yaw : message.yaw_pts) {
      if (!std::isfinite(yaw))
        throw std::invalid_argument("Yaw control points must be finite");
    }

    Eigen::MatrixXd position_points(message.pos_pts.size(), 3);
    for (size_t index = 0; index < message.pos_pts.size(); ++index) {
      position_points.row(static_cast<Eigen::Index>(index)) << message.pos_pts[index].x,
          message.pos_pts[index].y, message.pos_pts[index].z;
    }
    Eigen::VectorXd knots(message.knots.size());
    for (size_t index = 0; index < message.knots.size(); ++index)
      knots(static_cast<Eigen::Index>(index)) = message.knots[index];

    NonUniformBspline position_trajectory(position_points, message.order, 0.1);
    position_trajectory.setKnot(knots);

    Eigen::MatrixXd yaw_points(message.yaw_pts.size(), 1);
    for (size_t index = 0; index < message.yaw_pts.size(); ++index)
      yaw_points(static_cast<Eigen::Index>(index), 0) = message.yaw_pts[index];
    NonUniformBspline yaw_trajectory(yaw_points, 3, message.yaw_dt);

    const double duration = position_trajectory.getTimeSum();
    const double yaw_duration = yaw_trajectory.getTimeSum();
    if (!std::isfinite(duration) || duration <= 0.0 ||
        std::fabs(duration - yaw_duration) > std::max(1e-3, duration * 1e-3))
      throw std::invalid_argument("Position and yaw trajectory durations must match");

    std::vector<NonUniformBspline> parsed;
    parsed.reserve(6);
    parsed.push_back(position_trajectory);
    parsed.push_back(parsed[0].getDerivative());
    parsed.push_back(parsed[1].getDerivative());
    parsed.push_back(yaw_trajectory);
    parsed.push_back(parsed[3].getDerivative());
    parsed.push_back(parsed[2].getDerivative());

    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    if (message.traj_id <= trajectory_id_)
      throw std::invalid_argument("Trajectory IDs must be strictly increasing");
    trajectory_ = std::move(parsed);
    trajectory_duration_ = duration;
    start_time_ = rclcpp::Time(message.start_time, RCL_ROS_TIME);
    trajectory_id_ = message.traj_id;
    received_trajectory_ = true;
    task_finished_ = false;
    if (flight_start_time_.nanoseconds() == 0)
      flight_start_time_ = now();
    return true;
  } catch (const std::exception &exception) {
    if (error != nullptr)
      *error = exception.what();
    return false;
  }
}

bool TrajectoryServer::sampleCommand(
    const rclcpp::Time &sample_time, falcon_interfaces::msg::PositionCommand &command) const {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (!received_trajectory_ || trajectory_.size() != 6)
    return false;
  const double relative_time = (sample_time - start_time_).seconds();
  if (!std::isfinite(relative_time) || relative_time < 0.0)
    return false;

  const double evaluation_time = std::min(relative_time, trajectory_duration_);
  const bool finished = relative_time >= trajectory_duration_;
  const Eigen::Vector3d position = trajectory_[0].evaluateDeBoorT(evaluation_time);
  const Eigen::Vector3d velocity =
      finished ? Eigen::Vector3d::Zero() : trajectory_[1].evaluateDeBoorT(evaluation_time);
  const Eigen::Vector3d acceleration =
      finished ? Eigen::Vector3d::Zero() : trajectory_[2].evaluateDeBoorT(evaluation_time);
  const Eigen::Vector3d jerk =
      finished ? Eigen::Vector3d::Zero() : trajectory_[5].evaluateDeBoorT(evaluation_time);
  const double yaw = normalizeYaw(trajectory_[3].evaluateDeBoorT(evaluation_time)[0]);
  const double yaw_rate = finished ? 0.0 : trajectory_[4].evaluateDeBoorT(evaluation_time)[0];
  if (!position.allFinite() || !velocity.allFinite() || !acceleration.allFinite() ||
      !jerk.allFinite() || !std::isfinite(yaw) || !std::isfinite(yaw_rate))
    return false;

  command = command_template_;
  command.header.stamp = sample_time;
  command.trajectory_id = static_cast<uint32_t>(trajectory_id_);
  command.position.x = position.x();
  command.position.y = position.y();
  command.position.z = position.z();
  command.velocity.x = velocity.x();
  command.velocity.y = velocity.y();
  command.velocity.z = velocity.z();
  command.acceleration.x = acceleration.x();
  command.acceleration.y = acceleration.y();
  command.acceleration.z = acceleration.z();
  command.jerk.x = jerk.x();
  command.jerk.y = jerk.y();
  command.jerk.z = jerk.z();
  command.yaw = yaw;
  command.yaw_dot = yaw_rate;
  return true;
}

bool TrajectoryServer::hasTrajectory() const {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  return received_trajectory_;
}

double TrajectoryServer::trajectoryDuration() const {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  return trajectory_duration_;
}

int64_t TrajectoryServer::trajectoryId() const {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  return trajectory_id_;
}

void TrajectoryServer::bsplineCallback(
    const falcon_interfaces::msg::Bspline::SharedPtr message) {
  std::string error;
  if (!setTrajectory(*message, &error)) {
    RCLCPP_ERROR(get_logger(), "Rejected B-spline trajectory: %s", error.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Accepted B-spline trajectory %ld (%.3f s)",
              static_cast<long>(message->traj_id), trajectoryDuration());
}

void TrajectoryServer::replanCallback(const std_msgs::msg::Int32::SharedPtr message) {
  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (!received_trajectory_)
    return;
  const double elapsed = std::max(0.0, (now() - start_time_).seconds());
  if (message->data == 0) {
    trajectory_duration_ = std::min(trajectory_duration_, elapsed + 0.5);
  } else if (message->data == 1) {
    trajectory_duration_ = std::min(trajectory_duration_, elapsed);
    RCLCPP_WARN(get_logger(), "Collision replan requested; stopping the current trajectory");
  } else if (message->data == 2) {
    trajectory_duration_ = std::min(trajectory_duration_, elapsed);
    task_finished_ = true;
    RCLCPP_WARN(get_logger(), "Exploration-finished replan requested");
  } else {
    RCLCPP_ERROR(get_logger(), "Invalid replan type: %d", message->data);
  }
}

void TrajectoryServer::commandTimerCallback() {
  if (!initialization_complete_) {
    runInitializationStep();
    return;
  }

  falcon_interfaces::msg::PositionCommand command;
  const rclcpp::Time sample_time = now();
  if (!sampleCommand(sample_time, command)) {
    command = command_template_;
    command.header.stamp = sample_time;
    position_command_pub_->publish(command);
    return;
  }

  position_command_pub_->publish(command);
  publishFieldOfView(Eigen::Vector3d(command.position.x, command.position.y, command.position.z),
                     command.yaw);
  recordCommand(command, sample_time);
}

void TrajectoryServer::visualizationTimerCallback() { publishExecutedTrajectory(); }

void TrajectoryServer::runInitializationStep() {
  constexpr int kVerticalSteps = 100;
  constexpr int kHorizontalSteps = 100;
  if (initialization_step_ < kVerticalSteps) {
    command_template_.position.z += initial_offset_.z() / kVerticalSteps;
  } else if (initialization_step_ < kVerticalSteps + kHorizontalSteps) {
    command_template_.position.x += initial_offset_.x() / kHorizontalSteps;
    command_template_.position.y += initial_offset_.y() / kHorizontalSteps;
  } else {
    initialization_complete_ = true;
    RCLCPP_INFO(get_logger(), "Initial positioning completed");
    return;
  }
  ++initialization_step_;
  command_template_.header.stamp = now();
  position_command_pub_->publish(command_template_);
  publishFieldOfView(
      Eigen::Vector3d(command_template_.position.x, command_template_.position.y,
                      command_template_.position.z),
      command_template_.yaw);
}

void TrajectoryServer::publishFieldOfView(const Eigen::Vector3d &position, const double yaw) {
  if (fov_pub_->get_subscription_count() == 0)
    return;
  perception_utils_->setPose(position, yaw);
  std::vector<Eigen::Vector3d> starts;
  std::vector<Eigen::Vector3d> ends;
  perception_utils_->getFOV(starts, ends);
  const size_t line_count = std::min(starts.size(), ends.size());

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = now();
  marker.ns = "current_fov";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.color.a = 1.0;
  for (size_t index = 0; index < line_count; ++index) {
    geometry_msgs::msg::Point point;
    point.x = starts[index].x();
    point.y = starts[index].y();
    point.z = starts[index].z();
    marker.points.push_back(point);
    point.x = ends[index].x();
    point.y = ends[index].y();
    point.z = ends[index].z();
    marker.points.push_back(point);
  }
  fov_pub_->publish(marker);
}

void TrajectoryServer::publishExecutedTrajectory() {
  if (trajectory_pub_->get_subscription_count() == 0)
    return;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = world_frame_;
  marker.header.stamp = now();
  marker.ns = "commanded_trajectory";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  marker.points.reserve(commanded_positions_.size());
  for (const Eigen::Vector3d &position : commanded_positions_) {
    geometry_msgs::msg::Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    marker.points.push_back(point);
  }
  trajectory_pub_->publish(marker);
}

void TrajectoryServer::recordCommand(
    const falcon_interfaces::msg::PositionCommand &command, const rclcpp::Time &time) {
  const Eigen::Vector3d position(command.position.x, command.position.y, command.position.z);
  const Eigen::Vector3d velocity(command.velocity.x, command.velocity.y, command.velocity.z);
  const Eigen::Vector3d jerk(command.jerk.x, command.jerk.y, command.jerk.z);

  std::lock_guard<std::mutex> lock(trajectory_mutex_);
  if (commanded_positions_.empty() || (position - commanded_positions_.back()).norm() > 1e-6) {
    if (!commanded_positions_.empty() && (position - commanded_positions_.back()).norm() > 0.05)
      RCLCPP_WARN(get_logger(), "Commanded trajectory discontinuity: %.3f m",
                  (position - commanded_positions_.back()).norm());
    commanded_positions_.push_back(position);
    commanded_velocities_.push_back(velocity);
    if (last_command_time_.nanoseconds() != 0) {
      const double dt = std::max(0.0, (time - last_command_time_).seconds());
      jerk_energy_ += jerk.squaredNorm() * dt;
    }
    flight_end_time_ = time;
  }
  last_command_time_ = time;
}

double TrajectoryServer::pathLength(const std::vector<Eigen::Vector3d> &path) {
  double length = 0.0;
  for (size_t index = 1; index < path.size(); ++index)
    length += (path[index] - path[index - 1]).norm();
  return length;
}

} // namespace fast_planner
