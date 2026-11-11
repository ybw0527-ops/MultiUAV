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
      1.5, 0.2, 1.1,
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
    const Eigen::Vector3d initial_position(previous.position.x,previous.position.y,previous.position.z);
    const Eigen::Vector3d initial_velocity(previous.velocity.x,previous.velocity.y,previous.velocity.z);
    ASSERT_GT(initial_velocity.norm(),0.1);
    for(int i=1;i<=100;++i) {
      ASSERT_TRUE(server->sampleCommand(start+rclcpp::Duration::from_seconds(cut+0.01*i),command));
      EXPECT_LE(std::abs(command.yaw_dot-previous.yaw_dot),M_PI/2*0.01+1e-7);
      EXPECT_LE(std::abs(std::remainder(command.yaw-previous.yaw,2*M_PI)),0.01*M_PI/2+1e-7);
      const Eigen::Vector3d velocity(command.velocity.x,command.velocity.y,command.velocity.z);
      const Eigen::Vector3d previous_velocity(previous.velocity.x,previous.velocity.y,previous.velocity.z);
      EXPECT_LE((velocity-previous_velocity).norm()/0.01,2.0+1e-6);
      EXPECT_LE(Eigen::Vector3d(command.acceleration.x,command.acceleration.y,command.acceleration.z).norm(),2.0+1e-8);
      previous=command;
    }
    EXPECT_DOUBLE_EQ(command.yaw_dot,0.0);
    EXPECT_DOUBLE_EQ(Eigen::Vector3d(command.velocity.x,command.velocity.y,command.velocity.z).norm(),0.0);
    const Eigen::Vector3d stopped_position(command.position.x,command.position.y,command.position.z);
    EXPECT_NEAR((stopped_position-initial_position).norm(),initial_velocity.squaredNorm()/4.0,1e-6);
    EXPECT_EQ(command.trajectory_flag,falcon_interfaces::msg::PositionCommand::TRAJECTORY_STATUS_COMPLETED);
    // Repeated FINISH/STOP messages must not move the stopping point or restart braking.
    std_msgs::msg::Int32 stop; stop.data=type; publisher->publish(stop);
    executor.spin_some();
    EXPECT_DOUBLE_EQ(server->trajectoryDuration(),cut);
  }
}

TEST(TrajectoryServerTest, RejectsNonstationaryEndpointAndMovingFutureReplacement) {
  ensureRosInitialized();
  auto server=std::make_shared<TrajectoryServer>(serverOptions());
  auto message=makeTrajectoryMessage(server->now()-rclcpp::Duration::from_seconds(0.5));
  std::string error;
  ASSERT_TRUE(server->setTrajectory(message,&error)) << error;
  message.traj_id++;
  message.pos_pts.back().x+=0.01;
  EXPECT_FALSE(server->setTrajectory(message,&error));
  EXPECT_NE(error.find("Position spline must end at rest"),std::string::npos);
  message=makeTrajectoryMessage(server->now()+rclcpp::Duration::from_seconds(5)); message.traj_id=8;
  EXPECT_FALSE(server->setTrajectory(message,&error));
  EXPECT_NE(error.find("Future replacement"),std::string::npos);
  EXPECT_EQ(server->trajectoryId(),7);
}

TEST(TrajectoryServerTest, FullSpeedDiagonalStopTravelsOneMeterAndCannotBeInterrupted) {
  ensureRosInitialized();
  auto server=std::make_shared<TrajectoryServer>(serverOptions());
  auto sender=std::make_shared<rclcpp::Node>("full_speed_stop_test");
  auto publisher=sender->create_publisher<std_msgs::msg::Int32>("/planning/replan",10);
  const auto start=server->now()-rclcpp::Duration::from_seconds(4.0);
  auto message=makeTrajectoryMessage(start);
  const std::vector<double> distances{0,0,0.5,2,4,6,8,10,12,13.5,14,14,14};
  const Eigen::Vector3d direction=Eigen::Vector3d(1,1,0).normalized();
  Eigen::MatrixXd points(distances.size(),3);
  message.pos_pts.resize(distances.size());
  for(size_t i=0;i<distances.size();++i) {
    const Eigen::Vector3d p=Eigen::Vector3d(0,0,2)+distances[i]*direction;
    points.row(i)=p.transpose();
    message.pos_pts[i].x=p.x(); message.pos_pts[i].y=p.y(); message.pos_pts[i].z=p.z();
  }
  NonUniformBspline spline(points,3,1.0);
  const auto knots=spline.getKnot(); message.knots.assign(knots.data(),knots.data()+knots.size());
  message.yaw_pts.assign(distances.size(),0); message.yaw_dt=1;
  std::string error;
  ASSERT_TRUE(server->setTrajectory(message,&error)) << error;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server); executor.add_node(sender);
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(server->trajectoryDuration()>=10 && std::chrono::steady_clock::now()<until) {
    std_msgs::msg::Int32 stop; stop.data=1; publisher->publish(stop);
    executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  const double cut=server->trajectoryDuration();
  ASSERT_LT(cut,5.0);
  falcon_interfaces::msg::PositionCommand initial, stopped;
  // Reconstruct the exact nanosecond stop stamp; from_seconds truncation can
  // otherwise sample one nanosecond before braking (a 2 nm position offset).
  const auto cut_time=start+rclcpp::Duration(std::chrono::nanoseconds(std::llround(cut*1e9)));
  ASSERT_TRUE(server->sampleCommand(cut_time,initial));
  EXPECT_NEAR(Eigen::Vector3d(initial.velocity.x,initial.velocity.y,initial.velocity.z).norm(),2,1e-9);
  auto replacement=makeTrajectoryMessage(server->now()); replacement.traj_id=8;
  EXPECT_FALSE(server->setTrajectory(replacement,&error));
  EXPECT_NE(error.find("active emergency brake"),std::string::npos);
  ASSERT_TRUE(server->sampleCommand(cut_time+rclcpp::Duration::from_seconds(1.01),stopped));
  const Eigen::Vector3d displacement(stopped.position.x-initial.position.x,
                                    stopped.position.y-initial.position.y,stopped.position.z-initial.position.z);
  EXPECT_TRUE(displacement.isApprox(direction,1e-9)) << "displacement error: " << (displacement-direction).transpose();
  EXPECT_DOUBLE_EQ(Eigen::Vector3d(stopped.velocity.x,stopped.velocity.y,stopped.velocity.z).norm(),0);
  EXPECT_EQ(stopped.trajectory_flag,falcon_interfaces::msg::PositionCommand::TRAJECTORY_STATUS_COMPLETED);
}

TEST(TrajectoryServerTest, AcknowledgesStopBeforeFirstTrajectory) {
  ensureRosInitialized();
  auto server=std::make_shared<TrajectoryServer>(serverOptions());
  auto observer=std::make_shared<rclcpp::Node>("initial_hover_stop_test");
  auto publisher=observer->create_publisher<std_msgs::msg::Int32>("/planning/replan",10);
  bool acknowledged=false;
  const auto requested=server->now();
  auto subscription=observer->create_subscription<falcon_interfaces::msg::PositionCommand>(
      "/planning/pos_cmd",10,[&](falcon_interfaces::msg::PositionCommand::ConstSharedPtr msg) {
        if(msg->trajectory_flag!=falcon_interfaces::msg::PositionCommand::TRAJECTORY_STATUS_COMPLETED) return;
        EXPECT_GE(rclcpp::Time(msg->header.stamp,RCL_ROS_TIME),requested);
        EXPECT_DOUBLE_EQ(Eigen::Vector3d(msg->velocity.x,msg->velocity.y,msg->velocity.z).norm(),0);
        acknowledged=true;
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(server); executor.add_node(observer);
  const auto until=std::chrono::steady_clock::now()+std::chrono::seconds(2);
  while(!acknowledged && std::chrono::steady_clock::now()<until) {
    std_msgs::msg::Int32 stop; stop.data=1; publisher->publish(stop);
    executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(acknowledged);
}

} // namespace
} // namespace fast_planner
