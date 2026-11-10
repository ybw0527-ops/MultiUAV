#ifndef VOXEL_MAPPING_MAP_SERVER_H
#define VOXEL_MAPPING_MAP_SERVER_H

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "tic_toc.h"
#include "transformer/transformer.h"
#include "voxel_mapping/esdf.h"
#include "voxel_mapping/occupancy_grid.h"
#include "voxel_mapping/tsdf.h"

namespace voxel_mapping {

class MapServer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<MapServer>;
  using ConstPtr = std::shared_ptr<const MapServer>;

  struct Config {
    enum Mode { AIRSIM, UAV_SIMULATOR, REAL };

    Mode mode_{UAV_SIMULATOR};
    double fx_{0.0}, fy_{0.0}, cx_{0.0}, cy_{0.0};
    int image_width_{0}, image_height_{0};
    int depth_filter_margin_{0};
    double depth_scaling_factor_{1000.0}, depth_scaling_factor_inv_{0.001};
    bool depth_decimation_filter_{false};
    int skip_pixel_{1};
    int concurrent_depth_input_max_{1}, concurrent_pointcloud_input_max_{1};

    bool publish_tsdf_{false}, publish_esdf_{false}, publish_occupancy_grid_{false};
    bool publish_tsdf_slice_{false}, publish_esdf_slice_{false};
    bool publish_occupancy_grid_slice_{false};
    double publish_tsdf_period_{1.0}, publish_esdf_period_{1.0};
    double publish_occupancy_grid_period_{1.0};
    double tsdf_slice_height_{1.0}, esdf_slice_height_{1.0};
    double occupancy_grid_slice_height_{1.0};
    double tsdf_slice_visualization_height_{1.0};
    double esdf_slice_visualization_height_{1.0};
    double occupancy_grid_slice_visualization_height_{1.0};

    std::string world_frame_{"world"}, sensor_frame_{"camera"};

    bool verbose_{false}, verbose_time_{false};

    void print() const;
  };

  explicit MapServer(rclcpp::Node &node);

  TSDF::Ptr getTSDF() { return tsdf_; }
  ESDF::Ptr getESDF() { return esdf_; }
  OccupancyGrid::Ptr getOccupancyGrid() { return occupancy_grid_; }
  Transformer::Ptr getTransformer() { return transformer_; }
  std::mutex &getMapMutex() { return map_mutex_; }

  void depthCallback(const sensor_msgs::msg::Image::ConstSharedPtr image_msg);
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg);
  void globalMapCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud_msg);

  void saveMapPCDCallback(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                          std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void testESDFCallback(const geometry_msgs::msg::PoseStamped &msg);

  void publishMapTimerCallback();
  void publishTSDF();
  void publishTSDFSlice();
  void publishESDF();
  void publishESDFSlice();
  void publishOccupancyGrid();
  void publishOccupancyGridHighResolution();
  void publishOccupancyGridSlice();
  void publishDepthPointcloud(const PointCloudType &pointcloud, const rclcpp::Time &img_stamp);
  void publishInterpolatedPose(const Transformation &sensor_pose,
                               const rclcpp::Time &img_stamp);
  void publishDebugVisualizationBBox(const Position &bbox_min, const Position &bbox_max);
  void publishMapCoverage();

  void depthToPointcloud(const cv::Mat &depth_image, const Transformation &T_w_c,
                         PointCloudType &pointcloud, const rclcpp::Time &img_stamp);
  void depthToPointcloudDecimation(const cv::Mat &depth_image, const Transformation &T_w_c,
                                   PointCloudType &pointcloud, const rclcpp::Time &img_stamp);
  bool getNextImageFromQueue(std::queue<sensor_msgs::msg::Image::ConstSharedPtr> &queue,
                             cv::Mat &depth_image, Transformation &sensor_pose,
                             rclcpp::Time &img_stamp);
  bool getNextPointcloudFromQueue(
      std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> &queue,
      PointCloudType &pointcloud, Transformation &sensor_pose, rclcpp::Time &pcl_stamp);

  void pointcloudTransform(const PointCloudType &pointcloud_sensor, const Transformation &T_w_c,
                           PointCloudType &pointcloud_world);

  void getRegion(Position &ori, Eigen::Vector3d &size);
  Position getOrigin();
  double getResolution();
  int getVoxelNum();
  void getBox(Position &bmin, Position &bmax);
  void getBoxIndex(VoxelIndex &bmin_idx, VoxelIndex &bmax_idx);
  bool isInBox(const Position &pos);
  bool isInBox(const VoxelIndex &idx);
  OccupancyType getOccupancy(const Position &pos);
  OccupancyType getOccupancy(const VoxelIndex &idx);
  double getMapCoverage();

  void saveMap(const std::string &filename);
  void saveMapToPCD(const std::string &filename);
  void loadMap(const std::string &filename);
  void loadMap(const std::string &filename_occupied, const std::string &filename_free);
  void loadMapFromPCL(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void scaleColor(double value, double max_value, double min_value, double &color_value);

private:
  Config config_;
  rclcpp::Node *node_{nullptr};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_pub_, tsdf_slice_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr esdf_pub_, esdf_slice_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_grid_occupied_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_grid_free_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_grid_unknown_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupancy_grid_slice_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr interpolated_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr depth_pointcloud_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr map_coverage_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_visualization_pub_;

  rclcpp::TimerBase::SharedPtr publish_map_timer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;

  TSDF::Ptr tsdf_;
  ESDF::Ptr esdf_;
  OccupancyGrid::Ptr occupancy_grid_;
  Transformer::Ptr transformer_;

  std::mutex map_mutex_;
  std::atomic<int> map_update_counter_{0};

  std::queue<sensor_msgs::msg::Image::ConstSharedPtr> image_queue_;
  std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pointcloud_queue_;

  Position pos_min_, pos_max_;
  bool load_map_from_pcd_{false};
  int counter_{0};
  double map_coverage_{0.0};
  std::string save_map_path_{"map.pcd"};
};

} // namespace voxel_mapping

#endif // VOXEL_MAPPING_MAP_SERVER_H
