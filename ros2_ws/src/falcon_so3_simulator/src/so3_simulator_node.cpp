#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <falcon_interfaces/msg/position_command.hpp>
#include <falcon_interfaces/msg/so3_command.hpp>
#include "quadrotor_simulator/control.h"
#include "quadrotor_simulator/execution_timeline.h"

namespace falcon_so3_simulator {
class Simulator : public rclcpp::Node {
public:
  Simulator() : Node("so3_simulator") {
    auto positive=[this](const char *name,double fallback) {
      const double value=declare_parameter<double>(name,fallback);
      if(!std::isfinite(value) || value<=0) throw std::invalid_argument(name);
      return value;
    };
    quad_.setMass(positive("so3_simulator.mass",0.98));
    quad_.setGravity(positive("so3_simulator.gravity",9.81));
    quad_.setMotorTimeConstant(positive("so3_simulator.motor_time_constant",1.0/30));
    quad_.setMaxRPM(positive("so3_simulator.max_rpm",35000)); quad_.setMinRPM(0);
    step_=positive("so3_simulator.integration_step",0.001);
    maximum_catchup_=positive("so3_simulator.maximum_clock_catchup",2.0);
    control_acceleration_limit_=positive("so3_simulator.control_acceleration_limit",1.5);
    reference_.maximum_control_acceleration=control_acceleration_limit_;
    const double rate=positive("so3_simulator.odometry_rate",100);
    if(step_>0.01 || rate>1/step_) throw std::invalid_argument("Invalid integration/publication rate");
    period_=1/rate;
    auto gains=[this](const char *name,const std::vector<double> &fallback) {
      const auto values=declare_parameter<std::vector<double>>(name,fallback);
      if(values.size()!=3) throw std::invalid_argument(name);
      Eigen::Vector3d result;
      for(int i=0;i<3;++i) { if(!std::isfinite(values[i]) || values[i]<=0) throw std::invalid_argument(name); result[i]=values[i]; }
      return result;
    };
    kr_=gains("so3_simulator.rotation_gains",{1.5,1.5,1.0});
    kom_=gains("so3_simulator.angular_gains",{0.13,0.13,0.1});
    auto state=quad_.getState();
    state.x << declare_parameter<double>("map_config.init_x",0),declare_parameter<double>("map_config.init_y",0),declare_parameter<double>("map_config.init_z",0);
    reference_.yaw=declare_parameter<double>("map_config.init_yaw",0);
    if(!state.x.allFinite() || !std::isfinite(reference_.yaw)) throw std::invalid_argument("Nonfinite initial pose");
    state.R=Eigen::AngleAxisd(reference_.yaw,Eigen::Vector3d::UnitZ()).toRotationMatrix();
    const double hover=std::sqrt(quad_.getMass()*quad_.getGravity()/(4*quad_.getPropellerThrustCoefficient()));
    if(hover>quad_.getMaxRPM()) throw std::invalid_argument("RPM limit cannot support hover");
    state.motor_rpm.setConstant(hover); quad_.setState(state); reference_.position=state.x;
    timeline_=ReferenceTimeline(reference_,positive("so3_simulator.reference_speed_limit",1.7),
                               positive("so3_simulator.watchdog_deceleration",1.2));
    odom_pub_=create_publisher<nav_msgs::msg::Odometry>("odometry",rclcpp::SensorDataQoS());
    imu_pub_=create_publisher<sensor_msgs::msg::Imu>("imu",rclcpp::SensorDataQoS());
    rpm_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("motor_rpm",10);
    control_pub_=create_publisher<falcon_interfaces::msg::SO3Command>("so3_cmd",10);
    integration_pub_=create_publisher<std_msgs::msg::Float64MultiArray>("integration_status",10);
    trace_pub_=create_publisher<nav_msgs::msg::Path>("state_trace",20);
    command_sub_=create_subscription<falcon_interfaces::msg::PositionCommand>("command",10,
        [this](falcon_interfaces::msg::PositionCommand::ConstSharedPtr msg) {
          Reference next;
          next.maximum_control_acceleration=control_acceleration_limit_;
          next.position={msg->position.x,msg->position.y,msg->position.z};
          next.velocity={msg->velocity.x,msg->velocity.y,msg->velocity.z};
          next.acceleration={msg->acceleration.x,msg->acceleration.y,msg->acceleration.z};
          next.kx={msg->kx[0],msg->kx[1],msg->kx[2]}; next.kv={msg->kv[0],msg->kv[1],msg->kv[2]}; next.yaw=msg->yaw;
          if(!next.position.allFinite() || !next.velocity.allFinite() || !next.acceleration.allFinite() ||
              !next.kx.allFinite() || !next.kv.allFinite() || !std::isfinite(next.yaw) ||
              (next.kx.array()<0).any() || (next.kv.array()<0).any()) {
            RCLCPP_ERROR(get_logger(),"Rejected nonfinite/invalid position command"); return;
          }
          const double source=rclcpp::Time(msg->header.stamp,RCL_ROS_TIME).seconds();
          if(source<last_source_stamp_) { RCLCPP_WARN(get_logger(),"Ignoring out-of-order position command"); return; }
          last_source_stamp_=source;
          // A late command is applied no earlier than receipt/integrated time.
          // If this node's clock updates lag the sender, its future event waits.
          const double activation=std::max({source,now().seconds(),last_.seconds()});
          max_input_lateness_=std::max(max_input_lateness_,activation-source);
          if(!timeline_.push(activation,next)) RCLCPP_WARN(get_logger(),"Ignoring out-of-order command activation");
        });
    force_sub_=create_subscription<geometry_msgs::msg::Vector3>("force_disturbance",10,
      [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) { Eigen::Vector3d f(msg->x,msg->y,msg->z); if(f.allFinite()) quad_.setExternalForce(f); });
    moment_sub_=create_subscription<geometry_msgs::msg::Vector3>("moment_disturbance",10,
      [this](geometry_msgs::msg::Vector3::ConstSharedPtr msg) { Eigen::Vector3d m(msg->x,msg->y,msg->z); if(m.allFinite()) quad_.setExternalMoment(m); });
    timer_=create_wall_timer(std::chrono::milliseconds(1),[this]{ tick(); });
  }
private:
  void tick() {
    const auto stamp=now();
    if(get_parameter("use_sim_time").as_bool() && stamp.nanoseconds()==0) return;
    if(!started_) { last_=stamp; last_publish_=stamp; started_=true; publish(stamp); return; }
    double dt=(stamp-last_).seconds();
    if(dt<0 || dt>maximum_catchup_) throw std::runtime_error("SO3 invalid clock interval: dt="+std::to_string(dt)+
        " from="+std::to_string(last_.seconds())+" to="+std::to_string(stamp.seconds()));
    maximum_clock_interval_=std::max(maximum_clock_interval_,dt);
    if(dt>0.1) { ++catchup_events_; RCLCPP_WARN(get_logger(),"Replaying %.3f s of SO3 command history",dt); }
    nav_msgs::msg::Path trace; trace.header.frame_id="world"; trace.header.stamp=stamp;
    advanceExecution(quad_,timeline_,last_.seconds(),stamp.seconds(),step_,maximum_catchup_,kr_,kom_,
      [&](double time) {
        const auto &state=quad_.getState();
        maximum_integrated_speed_=std::max(maximum_integrated_speed_,state.v.norm());
        maximum_integrated_acceleration_=std::max(maximum_integrated_acceleration_,quad_.getAcc().norm());
        if(time-last_trace_time_<0.01-1e-10) return;
        geometry_msgs::msg::PoseStamped pose; pose.header.frame_id="world";
        pose.header.stamp=rclcpp::Time(static_cast<int64_t>(std::llround(time*1e9)),RCL_ROS_TIME);
        pose.pose.position.x=state.x.x(); pose.pose.position.y=state.x.y(); pose.pose.position.z=state.x.z();
        const Eigen::Quaterniond q(state.R);
        pose.pose.orientation.x=q.x(); pose.pose.orientation.y=q.y(); pose.pose.orientation.z=q.z(); pose.pose.orientation.w=q.w();
        trace.poses.push_back(pose); last_trace_time_=time;
      });
    last_=stamp;
    if(!trace.poses.empty()) trace_pub_->publish(trace);
    if((stamp-last_publish_).seconds()>=period_) { publish(stamp); last_publish_=stamp; }
  }
  void publish(const rclcpp::Time &stamp) {
    const auto &state=quad_.getState();
    const Eigen::Quaterniond q(state.R);
    nav_msgs::msg::Odometry odom; odom.header.stamp=stamp; odom.header.frame_id="world";
    odom.pose.pose.position.x=state.x.x(); odom.pose.pose.position.y=state.x.y(); odom.pose.pose.position.z=state.x.z();
    odom.pose.pose.orientation.x=q.x(); odom.pose.pose.orientation.y=q.y(); odom.pose.pose.orientation.z=q.z(); odom.pose.pose.orientation.w=q.w();
    // Preserve FALCON's world-linear/body-angular odometry convention.
    odom.twist.twist.linear.x=state.v.x(); odom.twist.twist.linear.y=state.v.y(); odom.twist.twist.linear.z=state.v.z();
    odom.twist.twist.angular.x=state.omega.x(); odom.twist.twist.angular.y=state.omega.y(); odom.twist.twist.angular.z=state.omega.z();
    odom_pub_->publish(odom);
    sensor_msgs::msg::Imu imu; imu.header.stamp=stamp; imu.header.frame_id="base_link";
    imu.orientation=odom.pose.pose.orientation; imu.angular_velocity=odom.twist.twist.angular;
    const Eigen::Vector3d specific=state.R.transpose()*(quad_.getAcc()+quad_.getGravity()*Eigen::Vector3d::UnitZ());
    imu.linear_acceleration.x=specific.x(); imu.linear_acceleration.y=specific.y(); imu.linear_acceleration.z=specific.z(); imu_pub_->publish(imu);
    std_msgs::msg::Float64MultiArray rpm; rpm.data.assign(state.motor_rpm.data(),state.motor_rpm.data()+4); rpm_pub_->publish(rpm);
    const auto control=calculate(quad_,timeline_.at(stamp.seconds()),kr_,kom_);
    falcon_interfaces::msg::SO3Command cmd; cmd.header=odom.header;
    cmd.force.x=control.force.x(); cmd.force.y=control.force.y(); cmd.force.z=control.force.z();
    cmd.orientation.x=control.orientation.x(); cmd.orientation.y=control.orientation.y(); cmd.orientation.z=control.orientation.z(); cmd.orientation.w=control.orientation.w();
    for(int i=0;i<3;++i) { cmd.kr[i]=kr_[i]; cmd.kom[i]=kom_[i]; }
    cmd.aux.enable_motors=true; control_pub_->publish(cmd);
    std_msgs::msg::Float64MultiArray integration;
    integration.data={maximum_clock_interval_,static_cast<double>(catchup_events_),max_input_lateness_,
                      maximum_integrated_speed_,maximum_integrated_acceleration_};
    integration_pub_->publish(integration);
  }
  QuadrotorSimulator::Quadrotor quad_; Reference reference_;
  ReferenceTimeline timeline_;
  Eigen::Vector3d kr_,kom_; double step_,period_,maximum_catchup_; bool started_{false};
  double control_acceleration_limit_{1.5};
  double maximum_clock_interval_{0},max_input_lateness_{0},last_source_stamp_{-1};
  double last_trace_time_{0},maximum_integrated_speed_{0},maximum_integrated_acceleration_{0};
  uint64_t catchup_events_{0};
  rclcpp::Time last_{0,0,RCL_ROS_TIME},last_publish_{0,0,RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rpm_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr integration_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trace_pub_;
  rclcpp::Publisher<falcon_interfaces::msg::SO3Command>::SharedPtr control_pub_;
  rclcpp::Subscription<falcon_interfaces::msg::PositionCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr force_sub_,moment_sub_;
};
} // namespace falcon_so3_simulator
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<falcon_so3_simulator::Simulator>());
  rclcpp::shutdown();
}
