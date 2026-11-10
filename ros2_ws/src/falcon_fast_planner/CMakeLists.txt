cmake_minimum_required(VERSION 3.8)
project(falcon_fast_planner LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(falcon_exploration_utils REQUIRED)
find_package(falcon_pathfinding REQUIRED)
find_package(falcon_trajectory REQUIRED)
find_package(falcon_voxel_mapping REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/planner_manager.cpp)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  falcon_exploration_utils
  falcon_pathfinding
  falcon_trajectory
  falcon_voxel_mapping
  rclcpp
)

install(DIRECTORY include/ DESTINATION include)
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
  falcon_exploration_utils
  falcon_pathfinding
  falcon_trajectory
  falcon_voxel_mapping
  rclcpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planner_manager test/test_planner_manager.cpp)
  target_link_libraries(test_planner_manager ${PROJECT_NAME})
  ament_target_dependencies(test_planner_manager rclcpp)
endif()

ament_package()
