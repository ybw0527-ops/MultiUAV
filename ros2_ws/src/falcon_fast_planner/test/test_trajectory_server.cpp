#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "bspline/non_uniform_bspline.h"
#include "fast_planner/trajectory_server.h"

namespace fast_planner {
namespace {

void ensureRosInitialized() {
  if (!rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }
}

rclcpp::NodeOptions serverOptions() {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      rclcpp::Parameter("traj_server.perform_initial_motion", false),
      rclcpp::Parameter("traj_server.init_dx", 0.0),
      rclcpp::Parameter("traj_server.init_dy", 0.0),
      rclcpp::Parameter("traj_server.init_dz", 0.0),
      rclcpp::Parameter("perception_utils.max_dist", 2.0),
      rclcpp::Parameter("perception_utils.vis_dist", 1.0),
  });
  return options;
}

falcon_interfaces::msg::Bspline makeTrajectoryMessage(const rclcpp::Time &start_time) {
  Eigen::MatrixXd position_points(6, 3);
  position_points << 0.0, 0.0, 1.0,
      0.0, 0.0, 1.0,
      0.5, 0.0, 1.0,
      1.0, 0.2, 1.1,
      1.5, 0.2, 1.1,
      1.5, 0.2, 1.1;
  NonUniformBspline spline(position_points, 3, 1.0);

  falcon_interfaces::msg::Bspline message;
  message.order = 3;
  message.traj_id = 7;
  message.start_time = static_cast<builtin_interfaces::msg::Time>(start_time);
  const Eigen::VectorXd knots = spline.getKnot();
  message.knots.resize(static_cast<size_t>(knots.size()));
  for (Eigen::Index index = 0; index < knots.size(); ++index)
    message.knots[static_cast<size_t>(index)] = knots(index);
  message.pos_pts.resize(static_cast<size_t>(position_points.rows()));
  for (Eigen::Index index = 0; index < position_points.rows(); ++index) {
    auto &point = message.pos_pts[static_cast<size_t>(index)];
    point.x = position_points(index, 0);
    point.y = position_points(index, 1);
    point.z = position_points(index, 2);
  }
  message.yaw_pts = {0.0, 0.0, 0.2, 0.5, 0.5, 0.5};
  message.yaw_dt = 1.0;
  return message;
}

TEST(TrajectoryServerTest, ParsesAndSamplesPositionCommands) {
  ensureRosInitialized();
  auto server = std::make_shared<TrajectoryServer>(serverOptions());
  const rclcpp::Time start = server->now();
  const auto message = makeTrajectoryMessage(start);

  std::string error;
  ASSERT_TRUE(server->setTrajectory(message, &error)) << error;
  EXPECT_TRUE(server->hasTrajectory());
  EXPECT_EQ(server->trajectoryId(), 7);
  EXPECT_NEAR(server->trajectoryDuration(), 3.0, 1e-9);

  falcon_interfaces::msg::PositionCommand command;
  EXPECT_FALSE(server->sampleCommand(start - rclcpp::Duration::from_seconds(0.1), command));
  ASSERT_TRUE(server->sampleCommand(start + rclcpp::Duration::from_seconds(1.0), command));
  EXPECT_EQ(command.trajectory_id, 7U);
  EXPECT_EQ(command.header.frame_id, "world");
  EXPECT_TRUE(std::isfinite(command.position.x));
  EXPECT_TRUE(std::isfinite(command.velocity.x));
  EXPECT_TRUE(std::isfinite(command.acceleration.x));
  EXPECT_TRUE(std::isfinite(command.jerk.x));
  EXPECT_GE(command.yaw, -M_PI);
  EXPECT_LE(command.yaw, M_PI);

  ASSERT_TRUE(server->sampleCommand(start + rclcpp::Duration::from_seconds(4.0), command));
  EXPECT_DOUBLE_EQ(command.velocity.x, 0.0);
  EXPECT_DOUBLE_EQ(command.velocity.y, 0.0);
  EXPECT_DOUBLE_EQ(command.velocity.z, 0.0);
  EXPECT_DOUBLE_EQ(command.yaw_dot, 0.0);
}

TEST(TrajectoryServerTest, RejectsMalformedAndMisorderedTrajectories) {
  ensureRosInitialized();
  auto server = std::make_shared<TrajectoryServer>(serverOptions());
  auto message = makeTrajectoryMessage(server->now());

  std::string error;
  ASSERT_TRUE(server->setTrajectory(message, &error)) << error;
  EXPECT_FALSE(server->setTrajectory(message, &error));
  EXPECT_NE(error.find("strictly increasing"), std::string::npos);

  message.traj_id = 8;
  message.knots.pop_back();
  EXPECT_FALSE(server->setTrajectory(message, &error));
  EXPECT_NE(error.find("knot count"), std::string::npos);
}

TEST(TrajectoryServerTest, RejectsWholeCurveDerivativeViolationsAndKeepsPreviousTrajectory) {
  ensureRosInitialized();
  auto server=std::make_shared<TrajectoryServer>(serverOptions());
  auto message=makeTrajectoryMessage(server->now());
  std::string error;
  ASSERT_TRUE(server->setTrajectory(message,&error)) << error;
  message.traj_id++;
  for(auto &knot:message.knots) knot*=0.05;
  message.yaw_dt*=0.05;
  EXPECT_FALSE(server->setTrajectory(message,&error));
  EXPECT_NE(error.find("hard speed/acceleration"),std::string::npos);
  EXPECT_EQ(server->trajectoryId(),7);
  message=makeTrajectoryMessage(server->now()); message.traj_id=8;
  for(size_t i=0;i<message.pos_pts.size();++i) {
    message.pos_pts[i].x=1.6*i; message.pos_pts[i].y=1.6*i;
  }
  EXPECT_FALSE(server->setTrajectory(message,&error)); // Each axis is <2; norm is >2.
  EXPECT_EQ(server->trajectoryId(),7);
}

TEST(TrajectoryServerTest, HoldsLastPositionWhileReplacementStartsInFuture) {
  ensureRosInitialized();
  auto server = std::make_shared<TrajectoryServer>(serverOptions());
  auto observer = std::make_shared<rclcpp::Node>("hold_command_test");
  falcon_interfaces::msg::PositionCommand last;
  int count=0;
  auto subscription=observer->create_subscription<falcon_interfaces::msg::PositionCommand>(
      "/planning/pos_cmd", 10, [&](falcon_interfaces::msg::PositionCommand::ConstSharedPtr msg) {
        last=*msg; ++count;
      });
  auto message=makeTrajectoryMessage(server->now()-rclcpp::Duration::from_seconds(4.0));
  ASSERT_TRUE(server->setTrajectory(message));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server); executor.add_node(observer);
  auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(count<3 && std::chrono::steady_clock::now()<until) {
    executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GE(count,3);
  const auto held=last.position;
  ASSERT_GT(held.x,1.0);
  message.traj_id++;
  message.start_time=server->now()+rclcpp::Duration::from_seconds(5.0);
  ASSERT_TRUE(server->setTrajectory(message));
  const int initial_count=count;
  until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(count<initial_count+5 && std::chrono::steady_clock::now()<until) {
    executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GE(count,initial_count+5);
  EXPECT_DOUBLE_EQ(last.position.x,held.x);
  EXPECT_DOUBLE_EQ(last.position.y,held.y);
  EXPECT_DOUBLE_EQ(last.position.z,held.z);
  EXPECT_DOUBLE_EQ(last.velocity.x,0.0);
  EXPECT_DOUBLE_EQ(last.acceleration.x,0.0);
}

TEST(TrajectoryServerTest, RejectsYawRateAndAccelerationViolations) {
  ensureRosInitialized();
  auto server=std::make_shared<TrajectoryServer>(serverOptions());
  auto message=makeTrajectoryMessage(server->now());
  std::string error;
  ASSERT_TRUE(server->setTrajectory(message,&error)) << error;
  message.traj_id++;
  for(size_t i=0;i<message.yaw_pts.size();++i) message.yaw_pts[i]=2.0*i;
  EXPECT_FALSE(server->setTrajectory(message,&error));
  EXPECT_NE(error.find("Yaw spline exceeds"),std::string::npos);
  EXPECT_EQ(server->trajectoryId(),7);
  for(size_t i=0;i<message.yaw_pts.size();++i) message.yaw_pts[i]=(i%2 ? 0.75 : -0.75);
  // Rate control hull is only 1.5 rad/s; acceleration is 3 rad/s^2.
  EXPECT_FALSE(server->setTrajectory(message,&error));
  EXPECT_NE(error.find("Yaw spline exceeds"),std::string::npos);
  EXPECT_EQ(server->trajectoryId(),7);
  message=makeTrajectoryMessage(server->now()); message.traj_id=8;
  message.yaw_pts.back()+=0.01;
  EXPECT_FALSE(server->setTrajectory(message,&error));
  EXPECT_NE(error.find("Yaw spline must end at rest"),std::string::npos);
  EXPECT_EQ(server->trajectoryId(),7);
}

TEST(TrajectoryServerTest, ExplicitStopsBrakeYawWithoutRateDiscontinuity) {
  ensureRosInitialized();
  for(int type : {0,1,2}) {
    auto server=std::make_shared<TrajectoryServer>(serverOptions());
    auto sender=std::make_shared<rclcpp::Node>("yaw_stop_test");
    auto publisher=sender->create_publisher<std_msgs::msg::Int32>("/planning/replan",10);
    const auto start=server->now()-rclcpp::Duration::from_seconds(0.5);
    auto message=makeTrajectoryMessage(start);
    ASSERT_TRUE(server->setTrajectory(message));
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(server); executor.add_node(sender);
    auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(server->trajectoryDuration()>=3.0 && std::chrono::steady_clock::now()<until) {
      std_msgs::msg::Int32 stop; stop.data=type; publisher->publish(stop);
      executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const double cut=server->trajectoryDuration();
    ASSERT_LT(cut,3.0);
    falcon_interfaces::msg::PositionCommand previous, command;
    ASSERT_TRUE(server->sampleCommand(start+rclcpp::Duration::from_seconds(cut),previous));
    ASSERT_GT(previous.yaw_dot,0.01);
    const auto stopped_position=previous.position;
    for(int i=1;i<=100;++i) {
      ASSERT_TRUE(server->sampleCommand(start+rclcpp::Duration::from_seconds(cut+0.01*i),command));
      EXPECT_LE(std::abs(command.yaw_dot-previous.yaw_dot),M_PI/2*0.01+1e-7);
      EXPECT_LE(std::abs(std::remainder(command.yaw-previous.yaw,2*M_PI)),0.01*M_PI/2+1e-7);
      EXPECT_NEAR(command.position.x,stopped_position.x,1e-9);
      EXPECT_DOUBLE_EQ(command.velocity.x,0.0);
      previous=command;
    }
    EXPECT_DOUBLE_EQ(command.yaw_dot,0.0);
    // Repeated FINISH/STOP messages must not move the stopping point or restart braking.
    std_msgs::msg::Int32 stop; stop.data=type; publisher->publish(stop);
    executor.spin_some();
    EXPECT_DOUBLE_EQ(server->trajectoryDuration(),cut);
  }
}

} // namespace
} // namespace fast_planner
