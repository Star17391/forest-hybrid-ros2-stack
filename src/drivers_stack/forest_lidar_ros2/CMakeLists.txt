cmake_minimum_required(VERSION 3.14)
project(forest_lidar_ros2)

find_package(ament_cmake REQUIRED)

install(FILES DEPENDENCIES.md DESTINATION share/${PROJECT_NAME})

ament_package()
