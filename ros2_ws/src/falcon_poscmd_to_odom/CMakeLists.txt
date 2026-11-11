cmake_minimum_required(VERSION 3.8)
project(falcon_poscmd_to_odom)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(falcon_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(poscmd_to_odom_node src/poscmd_to_odom_node.cpp)
target_include_directories(poscmd_to_odom_node PRIVATE include)
target_link_libraries(poscmd_to_odom_node Eigen3::Eigen)
ament_target_dependencies(poscmd_to_odom_node
  falcon_interfaces
  nav_msgs
  rclcpp
)

install(TARGETS poscmd_to_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_attitude test/test_attitude.cpp)
  target_include_directories(test_attitude PRIVATE include)
  target_link_libraries(test_attitude Eigen3::Eigen)
endif()

ament_package()
