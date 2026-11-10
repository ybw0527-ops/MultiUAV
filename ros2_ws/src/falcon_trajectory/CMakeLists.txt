cmake_minimum_required(VERSION 3.8)
project(falcon_trajectory LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(falcon_voxel_mapping REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(NLopt REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(TARGET NLopt::nlopt_cxx)
  set(FALCON_NLOPT_TARGET NLopt::nlopt_cxx)
elseif(TARGET NLopt::nlopt)
  set(FALCON_NLOPT_TARGET NLopt::nlopt)
else()
  message(FATAL_ERROR "NLopt was found but did not export a supported CMake target")
endif()

add_library(${PROJECT_NAME} SHARED
  src/bspline/bspline_optimizer.cpp
  src/bspline/non_uniform_bspline.cpp
  src/polynomial/polynomial_traj.cpp
  src/traj_utils/planning_visualization.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${FALCON_NLOPT_TARGET})
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  falcon_voxel_mapping
  geometry_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  visualization_msgs
)

install(DIRECTORY include/bspline include/polynomial include/traj_utils DESTINATION include)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  falcon_voxel_mapping
  geometry_msgs
  NLopt
  pcl_conversions
  rclcpp
  sensor_msgs
  visualization_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_trajectory_core test/test_trajectory_core.cpp)
  target_link_libraries(test_trajectory_core ${PROJECT_NAME})
  ament_target_dependencies(test_trajectory_core rclcpp)
endif()

ament_package()
