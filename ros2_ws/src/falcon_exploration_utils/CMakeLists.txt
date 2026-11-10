cmake_minimum_required(VERSION 3.8)
project(falcon_exploration_utils LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

file(GLOB LKH_SRCS CONFIGURE_DEPENDS
  src/lkh_tsp_solver/*.c
  src/lkh_tsp_solver/*.cpp
)
file(GLOB SOP_SRCS CONFIGURE_DEPENDS src/sop_solver/*.cpp)
set_source_files_properties(${LKH_SRCS} ${SOP_SRCS} PROPERTIES COMPILE_OPTIONS "-w")

add_library(${PROJECT_NAME} SHARED
  src/raycast/raycast.cpp
  src/transformer/transformer.cpp
  src/perception_utils/perception_utils.cpp
  src/system_info.cpp
  ${LKH_SRCS}
  ${SOP_SRCS}
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${PCL_INCLUDE_DIRS}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include/lkh_tsp_solver
    ${CMAKE_CURRENT_SOURCE_DIR}/include/sop_solver
)
target_compile_options(${PROJECT_NAME} PRIVATE $<$<COMPILE_LANGUAGE:C>:-fcommon>)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} PUBLIC
  geometry_msgs
  nav_msgs
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
  REGEX ".*/kindr/minkindr_conversions/.*" EXCLUDE
)
install(
  DIRECTORY config resource
  DESTINATION share/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 PCL geometry_msgs nav_msgs rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_exploration_utils test/test_exploration_utils.cpp)
  target_link_libraries(test_exploration_utils ${PROJECT_NAME})
endif()

ament_package()
