cmake_minimum_required(VERSION 3.14)
project(forest_semantic_segmentation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(semantic_segmentation_node src/semantic_segmentation_node.cpp)
ament_target_dependencies(semantic_segmentation_node rclcpp sensor_msgs std_msgs)
target_compile_features(semantic_segmentation_node PUBLIC cxx_std_17)

install(TARGETS semantic_segmentation_node DESTINATION lib/${PROJECT_NAME})

ament_package()
