#ifndef FALCON_EXPLORATION_UTILS_TRANSFORMER_H_
#define FALCON_EXPLORATION_UTILS_TRANSFORMER_H_

#include <cstddef>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "exploration_types.h"

class Transformer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<Transformer>;
  using ConstPtr = std::shared_ptr<const Transformer>;

  struct Config {
    enum class PoseType { POSE, ODOMETRY, TRANSFORM, UNKNOWN };
    PoseType pose_topic_type{PoseType::UNKNOWN};
    std::string world_frame{"world"};
    std::string sensor_frame{"camera"};
    std::string sensor_pose_topic{"/sensor_pose"};
    double timestamp_tolerance{1e-3};
    bool verbose{false};
    std::size_t queue_limit{1000};
  };

  explicit Transformer(rclcpp::Node &node);
  Transformer() = default;

  bool lookupTransform(const rclcpp::Time &timestamp, Transformation &transform) const;

  void transformCallback(const TransformStampedMsgType &msg);
  void poseCallback(const PoseStampedMsgType &msg);
  void odometryCallback(const OdometryMsgType &msg);

  const std::string &getWorldFrame() const { return world_frame_; }
  void getWorldFrame(std::string &world_frame) const { world_frame = world_frame_; }
  const std::string &getSensorFrame() const { return sensor_frame_; }
  void getSensorFrame(std::string &sensor_frame) const { sensor_frame = sensor_frame_; }

  void setWorldFrame(const std::string &world_frame) { world_frame_ = world_frame; }
  void setSensorFrame(const std::string &sensor_frame) { sensor_frame_ = sensor_frame; }

private:
  static Transformation messageToTransformation(const TransformMsgType &msg);
  void pushTransform(const TransformStampedMsgType &msg);

  Config config_;
  rclcpp::Node *node_{nullptr};
  std::string world_frame_{"world"};
  std::string sensor_frame_{"camera"};
  rclcpp::Duration timestamp_tolerance_{0, 1000000};

  rclcpp::Subscription<TransformStampedMsgType>::SharedPtr transform_sub_;
  rclcpp::Subscription<PoseStampedMsgType>::SharedPtr pose_sub_;
  rclcpp::Subscription<OdometryMsgType>::SharedPtr odometry_sub_;
  AlignedDeque<TransformStampedMsgType> transform_queue_;

  bool verbose_{false};
  std::size_t queue_limit_{1000};
};

#endif  // FALCON_EXPLORATION_UTILS_TRANSFORMER_H_
