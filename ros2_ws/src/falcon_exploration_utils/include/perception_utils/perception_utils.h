#ifndef FALCON_EXPLORATION_UTILS_PERCEPTION_UTILS_H_
#define FALCON_EXPLORATION_UTILS_PERCEPTION_UTILS_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>

namespace fast_planner {

class PerceptionUtils {
public:
  using Ptr = std::shared_ptr<PerceptionUtils>;
  using ConstPtr = std::shared_ptr<const PerceptionUtils>;

  enum FOVType { PINHOLE, OMNI_LIDAR };

  struct Config {
    std::string fov_type{"pinhole"};
    double horizontal_fov{1.500983156};
    double vertical_fov{0.994837674};
    double max_dist{5.0};
    double vis_dist{1.0};
  };

  explicit PerceptionUtils(rclcpp::Node &node);
  explicit PerceptionUtils(const Config &config);

  void setPose(const Eigen::Vector3d &pos, double yaw);
  void getFOV(std::vector<Eigen::Vector3d> &list1,
              std::vector<Eigen::Vector3d> &list2) const;
  bool insideFOV(const Eigen::Vector3d &point) const;
  bool insidePinholeFOV(const Eigen::Vector3d &point) const;
  bool insideOmniLidarFOV(const Eigen::Vector3d &point) const;

  std::vector<Eigen::Vector3d> cam_fov_voxels_;

private:
  void configure(const Config &config);

  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  double yaw_{0.0};
  std::vector<Eigen::Vector3d> normals_;

  FOVType fov_type_{PINHOLE};
  double left_angle_{0.0};
  double right_angle_{0.0};
  double top_angle_{0.0};
  double bottom_angle_{0.0};
  double max_dist_{5.0};
  double vis_dist_{1.0};

  Eigen::Vector3d n_top_;
  Eigen::Vector3d n_bottom_;
  Eigen::Vector3d n_left_;
  Eigen::Vector3d n_right_;
  Eigen::Matrix4d T_cb_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d T_bc_{Eigen::Matrix4d::Identity()};
  std::vector<Eigen::Vector3d> cam_vertices1_;
  std::vector<Eigen::Vector3d> cam_vertices2_;
};

}  // namespace fast_planner

#endif  // FALCON_EXPLORATION_UTILS_PERCEPTION_UTILS_H_
