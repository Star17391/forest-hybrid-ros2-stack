cmake_minimum_required(VERSION 3.14)
project(forest_nav2_bringup)

find_package(ament_cmake REQUIRED)
find_package(forest_hybrid_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(mission_nav2_bridge src/mission_nav2_bridge.cpp)
ament_target_dependencies(mission_nav2_bridge
  forest_hybrid_msgs geometry_msgs nav2_msgs nav_msgs rclcpp rclcpp_action std_msgs)
target_compile_features(mission_nav2_bridge PUBLIC cxx_std_17)

add_executable(cmd_vel_contract_relay src/cmd_vel_contract_relay.cpp)
ament_target_dependencies(cmd_vel_contract_relay geometry_msgs rclcpp)
target_compile_features(cmd_vel_contract_relay PUBLIC cxx_std_17)

install(TARGETS mission_nav2_bridge cmd_vel_contract_relay
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
