#include "perception_utils/perception_utils.h"

#include <cmath>
#include <stdexcept>

namespace fast_planner {

PerceptionUtils::PerceptionUtils(rclcpp::Node &node) {
  Config config;
  config.horizontal_fov = node.declare_parameter<double>(
      "uav_model.sensing_parameters.fov.horizontal", config.horizontal_fov);
  config.vertical_fov = node.declare_parameter<double>(
      "uav_model.sensing_parameters.fov.vertical", config.vertical_fov);
  config.fov_type = node.declare_parameter<std::string>(
      "perception_utils.fov_type", config.fov_type);
  config.max_dist = node.declare_parameter<double>(
      "perception_utils.max_dist", config.max_dist);
  config.vis_dist = node.declare_parameter<double>(
      "perception_utils.vis_dist", config.vis_dist);
  configure(config);
}

PerceptionUtils::PerceptionUtils(const Config &config) { configure(config); }

void PerceptionUtils::configure(const Config &config) {
  if (config.fov_type == "pinhole") {
    fov_type_ = PINHOLE;
  } else if (config.fov_type == "omni_lidar") {
    fov_type_ = OMNI_LIDAR;
  } else {
    throw std::invalid_argument("Unknown FOV type: " + config.fov_type);
  }

  const double horizontal_limit = fov_type_ == PINHOLE ? M_PI : 2.0 * M_PI;
  if (config.horizontal_fov <= 0.0 || config.horizontal_fov > horizontal_limit ||
      config.vertical_fov <= 0.0 || config.vertical_fov >= M_PI) {
    throw std::invalid_argument("FOV angles are outside the sensor model's valid range");
  }
  if (config.max_dist <= 0.0 || config.vis_dist <= 0.0) {
    throw std::invalid_argument("FOV distances must be positive");
  }

  left_angle_ = config.horizontal_fov / 2.0;
  right_angle_ = config.horizontal_fov / 2.0;
  top_angle_ = config.vertical_fov / 2.0;
  bottom_angle_ = config.vertical_fov / 2.0;
  max_dist_ = config.max_dist;
  vis_dist_ = config.vis_dist;

  if (fov_type_ != PINHOLE) {
    return;
  }

  n_top_ << 0.0, std::sin(M_PI_2 - top_angle_), std::cos(M_PI_2 - top_angle_);
  n_bottom_ << 0.0, -std::sin(M_PI_2 - bottom_angle_), std::cos(M_PI_2 - bottom_angle_);
  n_left_ << std::sin(M_PI_2 - left_angle_), 0.0, std::cos(M_PI_2 - left_angle_);
  n_right_ << -std::sin(M_PI_2 - right_angle_), 0.0, std::cos(M_PI_2 - right_angle_);

  T_cb_ << 0, -1, 0, 0,
           0, 0, 1, 0,
           1, 0, 0, 0,
           0, 0, 0, 1;
  T_bc_ = T_cb_.inverse();

  const double horizontal = vis_dist_ * std::tan(left_angle_);
  const double vertical = vis_dist_ * std::tan(top_angle_);
  const Eigen::Vector3d origin(0, 0, 0);
  const Eigen::Vector3d left_up(vis_dist_, horizontal, vertical);
  const Eigen::Vector3d left_down(vis_dist_, horizontal, -vertical);
  const Eigen::Vector3d right_up(vis_dist_, -horizontal, vertical);
  const Eigen::Vector3d right_down(vis_dist_, -horizontal, -vertical);

  cam_vertices1_ = {origin, origin, origin, origin,
                    left_up, right_up, right_down, left_down};
  cam_vertices2_ = {left_up, left_down, right_up, right_down,
                    right_up, right_down, left_down, left_up};

  cam_fov_voxels_.clear();
  const double y_max = max_dist_ * std::tan(left_angle_);
  const double z_max = max_dist_ * std::tan(top_angle_);
  for (double y = -y_max; y <= y_max + 1e-4; y += y_max / 3.0) {
    for (double z = -z_max; z <= z_max + 1e-4; z += z_max / 3.0) {
      cam_fov_voxels_.emplace_back(max_dist_, y, z);
    }
  }
}

void PerceptionUtils::setPose(const Eigen::Vector3d &pos, const double yaw) {
  pos_ = pos;
  yaw_ = yaw;

  Eigen::Matrix3d rotation_world_body;
  rotation_world_body << std::cos(yaw_), -std::sin(yaw_), 0.0,
                         std::sin(yaw_), std::cos(yaw_), 0.0,
                         0.0, 0.0, 1.0;
  Eigen::Matrix4d transform_world_body = Eigen::Matrix4d::Identity();
  transform_world_body.block<3, 3>(0, 0) = rotation_world_body;
  transform_world_body.block<3, 1>(0, 3) = pos_;
  const Eigen::Matrix3d rotation_world_camera =
      (transform_world_body * T_bc_).block<3, 3>(0, 0);

  normals_ = {n_top_, n_bottom_, n_left_, n_right_};
  for (auto &normal : normals_) {
    normal = rotation_world_camera * normal;
  }
}

void PerceptionUtils::getFOV(std::vector<Eigen::Vector3d> &list1,
                             std::vector<Eigen::Vector3d> &list2) const {
  list1.clear();
  list2.clear();
  if (fov_type_ == OMNI_LIDAR) {
    return;
  }

  Eigen::Matrix3d rotation_world_body;
  rotation_world_body << std::cos(yaw_), -std::sin(yaw_), 0.0,
                         std::sin(yaw_), std::cos(yaw_), 0.0,
                         0.0, 0.0, 1.0;
  for (std::size_t index = 0; index < cam_vertices1_.size(); ++index) {
    list1.push_back(rotation_world_body * cam_vertices1_[index] + pos_);
    list2.push_back(rotation_world_body * cam_vertices2_[index] + pos_);
  }
}

bool PerceptionUtils::insideFOV(const Eigen::Vector3d &point) const {
  return fov_type_ == PINHOLE ? insidePinholeFOV(point) : insideOmniLidarFOV(point);
}

bool PerceptionUtils::insidePinholeFOV(const Eigen::Vector3d &point) const {
  Eigen::Vector3d direction = point - pos_;
  if (direction.norm() > max_dist_ || direction.isZero()) {
    return false;
  }
  direction.normalize();
  for (const auto &normal : normals_) {
    if (direction.dot(normal) < 0.0) {
      return false;
    }
  }
  return true;
}

bool PerceptionUtils::insideOmniLidarFOV(const Eigen::Vector3d &point) const {
  Eigen::Vector3d direction = point - pos_;
  if (direction.norm() > max_dist_ || direction.isZero()) {
    return false;
  }
  direction.normalize();
  const double elevation = std::atan2(
      direction.z(), std::hypot(direction.x(), direction.y()));
  return elevation > -bottom_angle_ && elevation < top_angle_;
}

}  // namespace fast_planner
