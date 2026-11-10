#include "transformer/transformer.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {

Transformer::Config::PoseType parsePoseType(const std::string &value) {
  if (value == "pose") {
    return Transformer::Config::PoseType::POSE;
  }
  if (value == "odometry") {
    return Transformer::Config::PoseType::ODOMETRY;
  }
  if (value == "transform") {
    return Transformer::Config::PoseType::TRANSFORM;
  }
  return Transformer::Config::PoseType::UNKNOWN;
}

rclcpp::Time stampOf(const TransformStampedMsgType &message) {
  return rclcpp::Time(message.header.stamp);
}

}  // namespace

Transformer::Transformer(rclcpp::Node &node) : node_(&node) {
  config_.world_frame = node.declare_parameter<std::string>(
      "transformer.world_frame", config_.world_frame);
  config_.sensor_frame = node.declare_parameter<std::string>(
      "transformer.sensor_frame", config_.sensor_frame);
  config_.sensor_pose_topic = node.declare_parameter<std::string>(
      "transformer.sensor_pose_topic", config_.sensor_pose_topic);
  const auto pose_topic_type = node.declare_parameter<std::string>(
      "transformer.pose_topic_type", "odometry");
  config_.timestamp_tolerance = node.declare_parameter<double>(
      "transformer.timestamp_tolerance", config_.timestamp_tolerance);
  config_.verbose = node.declare_parameter<bool>(
      "transformer.verbose", config_.verbose);
  const auto queue_limit = node.declare_parameter<int>(
      "transformer.queue_limit", static_cast<int>(config_.queue_limit));

  config_.pose_topic_type = parsePoseType(pose_topic_type);
  if (config_.pose_topic_type == Config::PoseType::UNKNOWN) {
    throw std::invalid_argument("Unknown transformer.pose_topic_type: " + pose_topic_type);
  }
  if (config_.timestamp_tolerance < 0.0 || queue_limit <= 0) {
    throw std::invalid_argument("Transformer tolerance must be non-negative and queue limit positive");
  }

  world_frame_ = config_.world_frame;
  sensor_frame_ = config_.sensor_frame;
  timestamp_tolerance_ = rclcpp::Duration::from_seconds(config_.timestamp_tolerance);
  verbose_ = config_.verbose;
  queue_limit_ = static_cast<std::size_t>(queue_limit);

  const auto qos = rclcpp::SensorDataQoS().keep_last(100);
  switch (config_.pose_topic_type) {
    case Config::PoseType::POSE:
      pose_sub_ = node.create_subscription<PoseStampedMsgType>(
          config_.sensor_pose_topic, qos,
          [this](PoseStampedMsgType::ConstSharedPtr msg) { poseCallback(*msg); });
      break;
    case Config::PoseType::ODOMETRY:
      odometry_sub_ = node.create_subscription<OdometryMsgType>(
          config_.sensor_pose_topic, qos,
          [this](OdometryMsgType::ConstSharedPtr msg) { odometryCallback(*msg); });
      break;
    case Config::PoseType::TRANSFORM:
      transform_sub_ = node.create_subscription<TransformStampedMsgType>(
          config_.sensor_pose_topic, qos,
          [this](TransformStampedMsgType::ConstSharedPtr msg) { transformCallback(*msg); });
      break;
    case Config::PoseType::UNKNOWN:
      break;
  }

  if (verbose_) {
    RCLCPP_INFO(node.get_logger(),
                "Transformer initialized: %s -> %s, topic %s",
                world_frame_.c_str(), sensor_frame_.c_str(),
                config_.sensor_pose_topic.c_str());
  }
}

Transformation Transformer::messageToTransformation(const TransformMsgType &msg) {
  const Eigen::Quaterniond rotation(
      msg.rotation.w, msg.rotation.x, msg.rotation.y, msg.rotation.z);
  const Eigen::Vector3d translation(
      msg.translation.x, msg.translation.y, msg.translation.z);
  return Transformation(rotation.normalized(), translation);
}

bool Transformer::lookupTransform(
    const rclcpp::Time &timestamp, Transformation &transform) const {
  if (transform_queue_.empty()) {
    if (verbose_ && node_ != nullptr) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "No transforms have been received yet");
    }
    return false;
  }

  const auto lower = std::lower_bound(
      transform_queue_.begin(), transform_queue_.end(), timestamp,
      [](const TransformStampedMsgType &message, const rclcpp::Time &time) {
        return stampOf(message) < time;
      });

  const auto within_tolerance = [this, &timestamp](const TransformStampedMsgType &message) {
    return std::llabs((stampOf(message) - timestamp).nanoseconds()) <=
           timestamp_tolerance_.nanoseconds();
  };

  if (lower != transform_queue_.end() && within_tolerance(*lower)) {
    transform = messageToTransformation(lower->transform);
    return true;
  }
  if (lower != transform_queue_.begin()) {
    const auto previous = std::prev(lower);
    if (within_tolerance(*previous)) {
      transform = messageToTransformation(previous->transform);
      return true;
    }
  }

  if (lower == transform_queue_.begin() || lower == transform_queue_.end()) {
    return false;
  }

  const auto older = std::prev(lower);
  const rclcpp::Time older_time = stampOf(*older);
  const rclcpp::Time newer_time = stampOf(*lower);
  const auto interval_ns = (newer_time - older_time).nanoseconds();
  if (interval_ns <= 0) {
    return false;
  }

  const double ratio = static_cast<double>((timestamp - older_time).nanoseconds()) /
                       static_cast<double>(interval_ns);
  const Transformation older_transform = messageToTransformation(older->transform);
  const Transformation newer_transform = messageToTransformation(lower->transform);
  const Transformation::Vector6 difference =
      (older_transform.inverse() * newer_transform).log();
  transform = older_transform * Transformation::exp(ratio * difference);
  return true;
}

void Transformer::pushTransform(const TransformStampedMsgType &msg) {
  if (!transform_queue_.empty() && stampOf(msg) < stampOf(transform_queue_.back())) {
    const auto insertion = std::lower_bound(
        transform_queue_.begin(), transform_queue_.end(), stampOf(msg),
        [](const TransformStampedMsgType &message, const rclcpp::Time &time) {
          return stampOf(message) < time;
        });
    transform_queue_.insert(insertion, msg);
  } else {
    transform_queue_.push_back(msg);
  }
  while (transform_queue_.size() > queue_limit_) {
    transform_queue_.pop_front();
  }
}

void Transformer::transformCallback(const TransformStampedMsgType &msg) {
  pushTransform(msg);
}

void Transformer::poseCallback(const PoseStampedMsgType &msg) {
  TransformStampedMsgType transform_msg;
  transform_msg.header = msg.header;
  transform_msg.child_frame_id = sensor_frame_;
  transform_msg.transform.translation.x = msg.pose.position.x;
  transform_msg.transform.translation.y = msg.pose.position.y;
  transform_msg.transform.translation.z = msg.pose.position.z;
  transform_msg.transform.rotation = msg.pose.orientation;
  pushTransform(transform_msg);
}

void Transformer::odometryCallback(const OdometryMsgType &msg) {
  TransformStampedMsgType transform_msg;
  transform_msg.header = msg.header;
  transform_msg.child_frame_id = msg.child_frame_id.empty() ? sensor_frame_ : msg.child_frame_id;
  transform_msg.transform.translation.x = msg.pose.pose.position.x;
  transform_msg.transform.translation.y = msg.pose.pose.position.y;
  transform_msg.transform.translation.z = msg.pose.pose.position.z;
  transform_msg.transform.rotation = msg.pose.pose.orientation;
  pushTransform(transform_msg);
}
