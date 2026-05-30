cmake_minimum_required(VERSION 3.14)
project(forest_planner_ros2)

find_package(ament_cmake REQUIRED)
find_package(forest_hybrid_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mission_manager_node src/mission_manager_node.cpp)
ament_target_dependencies(
  mission_manager_node
  forest_hybrid_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs)
target_compile_features(mission_manager_node PUBLIC cxx_std_17)

install(TARGETS mission_manager_node DESTINATION lib/${PROJECT_NAME})
ament_package()
