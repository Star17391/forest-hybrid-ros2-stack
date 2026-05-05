cmake_minimum_required(VERSION 3.14)
project(forest_robot_supervisor)

find_package(ament_cmake REQUIRED)
find_package(forest_hybrid_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(operation_mode_node src/operation_mode_node.cpp)
ament_target_dependencies(operation_mode_node forest_hybrid_msgs rclcpp)
target_compile_features(operation_mode_node PUBLIC cxx_std_17)

install(TARGETS operation_mode_node DESTINATION lib/${PROJECT_NAME})

ament_package()
