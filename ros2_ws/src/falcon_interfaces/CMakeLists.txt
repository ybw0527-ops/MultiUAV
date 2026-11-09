cmake_minimum_required(VERSION 3.8)
project(falcon_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(message_files
  "msg/AuxCommand.msg"
  "msg/Bspline.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/LQRTrajectory.msg"
  "msg/MultiOccupancyGrid.msg"
  "msg/MultiSparseMap3D.msg"
  "msg/Odometry.msg"
  "msg/OutputData.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/SparseMap3D.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/VerticalOccupancyGridList.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  DEPENDENCIES
    builtin_interfaces
    geometry_msgs
    nav_msgs
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
