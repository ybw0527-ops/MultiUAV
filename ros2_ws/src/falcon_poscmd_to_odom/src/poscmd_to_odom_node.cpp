#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "falcon_interfaces/msg/position_command.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace falcon_poscmd_to_odom
{

class PoscmdToOdomNode final : public rclcpp::Node
{
public:
  PoscmdToOdomNode()
  : Node("poscmd_to_odom"), command_received_(false)
  {
    init_x_ = declare_parameter<double>("init_x", 0.0);
    init_y_ = declare_parameter<double>("init_y", 0.0);
    init_z_ = declare_parameter<double>("init_z", 0.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "world");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 100.0);

    if (publish_rate_hz <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be greater than zero");
    }

    odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
    command_subscription_ = create_subscription<falcon_interfaces::msg::PositionCommand>(
      "command", rclcpp::QoS(1).reliable(),
      [this](falcon_interfaces::msg::PositionCommand::ConstSharedPtr message) {
        command_ = *message;
        command_received_ = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publish_odometry();});
  }

private:
  void publish_odometry()
  {
    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = frame_id_;

    if (!command_received_) {
      odometry.header.stamp = now();
      odometry.pose.pose.position.x = init_x_;
      odometry.pose.pose.position.y = init_y_;
      odometry.pose.pose.position.z = init_z_;
      odometry.pose.pose.orientation.w = 1.0;
      odometry_publisher_->publish(odometry);
      return;
    }

    odometry.header.stamp = command_.header.stamp;
    odometry.pose.pose.position = command_.position;

    const Eigen::Vector3d acceleration(
      command_.acceleration.x, command_.acceleration.y, command_.acceleration.z);
    const Eigen::Vector3d alpha = acceleration + 9.8 * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d x_heading(std::cos(command_.yaw), std::sin(command_.yaw), 0.0);
    const Eigen::Vector3d y_heading(-x_heading.y(), x_heading.x(), 0.0);
    const Eigen::Vector3d x_body = y_heading.cross(alpha).normalized();
    const Eigen::Vector3d y_body = alpha.cross(x_body).normalized();
    const Eigen::Vector3d z_body = x_body.cross(y_body);

    Eigen::Matrix3d rotation;
    rotation.col(0) = x_body;
    rotation.col(1) = y_body;
    rotation.col(2) = z_body;
    const Eigen::Quaterniond orientation(rotation);

    odometry.pose.pose.orientation.w = orientation.w();
    odometry.pose.pose.orientation.x = orientation.x();
    odometry.pose.pose.orientation.y = orientation.y();
    odometry.pose.pose.orientation.z = orientation.z();
    odometry.twist.twist.linear = command_.velocity;

    // Preserve the upstream simulator contract. These fields carry commanded
    // acceleration even though nav_msgs names them angular velocity.
    odometry.twist.twist.angular = command_.acceleration;
    odometry_publisher_->publish(odometry);
  }

  bool command_received_;
  double init_x_;
  double init_y_;
  double init_z_;
  std::string frame_id_;
  falcon_interfaces::msg::PositionCommand command_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Subscription<falcon_interfaces::msg::PositionCommand>::SharedPtr
    command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace falcon_poscmd_to_odom

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<falcon_poscmd_to_odom::PoscmdToOdomNode>());
  rclcpp::shutdown();
  return 0;
}
