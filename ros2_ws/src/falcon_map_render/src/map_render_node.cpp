#include "falcon_map_render/render_core.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <memory>

namespace falcon_map_render {
class MapRenderNode : public rclcpp::Node {
public:
  MapRenderNode() : Node("map_render_node") {
    camera_.width=declare_parameter("uav_model.sensing_parameters.image_width",160);
    camera_.height=declare_parameter("uav_model.sensing_parameters.image_height",120);
    camera_.fx=declare_parameter("uav_model.sensing_parameters.camera_intrinsics.fx",80.0);
    camera_.fy=declare_parameter("uav_model.sensing_parameters.camera_intrinsics.fy",80.0);
    camera_.cx=declare_parameter("uav_model.sensing_parameters.camera_intrinsics.cx",80.0);
    camera_.cy=declare_parameter("uav_model.sensing_parameters.camera_intrinsics.cy",60.0);
    camera_.near=declare_parameter("uav_model.sensing_parameters.min_depth",0.1);
    camera_.far=declare_parameter("uav_model.sensing_parameters.max_depth",5.0);
    camera_.point_radius=declare_parameter("pointcloud_render.inflate_radius",0.1);
    camera_.validate();
    const double rate=declare_parameter("map_render.render_rate",10.0);
    if (!std::isfinite(rate) || rate<=0 || rate>1000) throw std::invalid_argument("Invalid render rate");
    world_frame_=declare_parameter<std::string>("map_render.world_frame","world");
    camera_frame_=declare_parameter<std::string>("map_render.camera_frame","camera");
    // Optical +z is body +x, optical +x is body -y, optical +y is body -z.
    const std::vector<double> default_body{0,0,1,0, -1,0,0,0, 0,-1,0,0, 0,0,0,1};
    const std::vector<double> identity{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1};
    body_from_camera_=readTransform("map_config.T_b_c",default_body);
    const auto map_from_world=readTransform("map_config.T_m_w",identity);
    auto file=std::filesystem::path(declare_parameter<std::string>("map_config.map_file",""));
    if (file.empty()) throw std::invalid_argument("map_config.map_file is required");
    if (file.is_relative()) file=std::filesystem::path(ament_index_cpp::get_package_share_directory("falcon_map_render"))/"resource"/file;
    scene_.load(file.string(), declare_parameter("map_config.scale",1.0), map_from_world);
    RCLCPP_INFO(get_logger(), "Loaded %s: %zu %s; CPU renderer",file.c_str(),scene_.primitiveCount(),scene_.mesh()?"triangles":"points");
    pose_pub_=create_publisher<geometry_msgs::msg::TransformStamped>("/uav_simulator/sensor_pose",10);
    depth_pub_=create_publisher<sensor_msgs::msg::Image>("/uav_simulator/depth_image",rclcpp::SensorDataQoS());
    info_pub_=create_publisher<sensor_msgs::msg::CameraInfo>("/uav_simulator/camera_info",10);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("/uav_simulator/odometry",rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const auto &p=msg->pose.pose.position; const auto &q=msg->pose.pose.orientation;
        const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
        if (!Eigen::Vector3d(p.x,p.y,p.z).allFinite() || !rotation.coeffs().allFinite() || rotation.norm()<1e-6) {
          RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"Ignoring invalid odometry"); return;
        }
        Eigen::Isometry3d body=Eigen::Isometry3d::Identity();
        body.linear()=rotation.normalized().toRotationMatrix();
        body.translation()=Eigen::Vector3d(p.x,p.y,p.z);
        world_from_camera_=body*body_from_camera_;
        stamp_=msg->header.stamp;
        have_pose_=true; dirty_=true;
      });
    timer_=create_wall_timer(std::chrono::nanoseconds(static_cast<int64_t>(1e9/rate)),[this]{render();});
  }
private:
  Eigen::Isometry3d readTransform(const std::string &name,const std::vector<double> &fallback) {
    const auto values=declare_parameter<std::vector<double>>(name,fallback);
    if (values.size()!=16) throw std::invalid_argument(name+" must contain 16 row-major values");
    Eigen::Isometry3d result=Eigen::Isometry3d::Identity();
    for (int row=0;row<4;++row) for (int col=0;col<4;++col) result.matrix()(row,col)=values[4*row+col];
    if (!validTransform(result)) throw std::invalid_argument(name+" must be a rigid transform");
    return result;
  }
  void render() {
    if (!have_pose_ || !dirty_) return;
    const auto start=std::chrono::steady_clock::now();
    auto depth=scene_.render(camera_,world_from_camera_);
    geometry_msgs::msg::TransformStamped pose;
    pose.header.stamp=stamp_; pose.header.frame_id=world_frame_; pose.child_frame_id=camera_frame_;
    const auto &p=world_from_camera_.translation();
    const Eigen::Quaterniond q(world_from_camera_.linear());
    pose.transform.translation.x=p.x(); pose.transform.translation.y=p.y(); pose.transform.translation.z=p.z();
    pose.transform.rotation.w=q.w(); pose.transform.rotation.x=q.x(); pose.transform.rotation.y=q.y(); pose.transform.rotation.z=q.z();
    pose_pub_->publish(pose);
    sensor_msgs::msg::Image image;
    image.header.stamp=stamp_; image.header.frame_id=camera_frame_;
    image.width=camera_.width; image.height=camera_.height; image.encoding="32FC1";
    const uint16_t endian_test=1;
    image.is_bigendian=(*reinterpret_cast<const uint8_t*>(&endian_test)==0);
    image.step=image.width*sizeof(float); image.data.resize(depth.size()*sizeof(float));
    std::memcpy(image.data.data(),depth.data(),image.data.size());
    depth_pub_->publish(image);
    sensor_msgs::msg::CameraInfo info;
    info.header=image.header; info.width=image.width; info.height=image.height;
    info.distortion_model="plumb_bob"; info.d.assign(5,0.0);
    info.k={camera_.fx,0,camera_.cx,0,camera_.fy,camera_.cy,0,0,1};
    info.r={1,0,0,0,1,0,0,0,1};
    info.p={camera_.fx,0,camera_.cx,0,0,camera_.fy,camera_.cy,0,0,0,1,0};
    info_pub_->publish(info);
    dirty_=false;
    const double elapsed=std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    RCLCPP_DEBUG(get_logger(),"Rendered frame in %.2f ms",elapsed*1000);
  }
  Camera camera_;
  Scene scene_;
  std::string world_frame_,camera_frame_;
  Eigen::Isometry3d body_from_camera_{Eigen::Isometry3d::Identity()},world_from_camera_{Eigen::Isometry3d::Identity()};
  builtin_interfaces::msg::Time stamp_;
  bool have_pose_{false},dirty_{false};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try { rclcpp::spin(std::make_shared<falcon_map_render::MapRenderNode>()); }
  catch(const std::exception &error) { RCLCPP_FATAL(rclcpp::get_logger("map_render"),"%s",error.what()); result=1; }
  rclcpp::shutdown(); return result;
}
