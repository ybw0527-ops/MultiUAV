cmake_minimum_required(VERSION 3.8)
project(falcon_voxel_mapping LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters io kdtree search)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(falcon_exploration_utils REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/esdf.cpp
  src/map_server.cpp
  src/occupancy_grid.cpp
  src/tsdf.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  cv_bridge
  falcon_exploration_utils
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

add_executable(map_server_node src/map_server_node.cpp)
target_link_libraries(map_server_node ${PROJECT_NAME})
ament_target_dependencies(map_server_node rclcpp)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(TARGETS map_server_node DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  cv_bridge
  Eigen3
  falcon_exploration_utils
  geometry_msgs
  nav_msgs
  PCL
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voxel_mapping_core test/test_voxel_mapping_core.cpp)
  target_link_libraries(test_voxel_mapping_core ${PROJECT_NAME})

  ament_add_gtest(test_map_server test/test_map_server.cpp)
  target_link_libraries(test_map_server ${PROJECT_NAME})
  ament_target_dependencies(test_map_server rclcpp)
endif()

ament_package()
