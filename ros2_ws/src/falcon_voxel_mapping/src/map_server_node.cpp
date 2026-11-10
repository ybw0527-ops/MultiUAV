#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "voxel_mapping/map_server.h"

namespace voxel_mapping {

class MapServerNode : public rclcpp::Node {
public:
  MapServerNode() : rclcpp::Node("falcon_map_server") {
    map_server_ = std::make_shared<MapServer>(*this);
  }

private:
  MapServer::Ptr map_server_;
};

} // namespace voxel_mapping

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<voxel_mapping::MapServerNode>());
  rclcpp::shutdown();
  return 0;
}
