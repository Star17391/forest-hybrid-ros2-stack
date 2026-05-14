cmake_minimum_required(VERSION 3.14)
project(forest_navigation_ros2)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(forest_navigation_core SHARED
  src/global_planner/global_planner.cpp
  src/local_planner/local_planner.cpp
  src/trajectory_sampler/trajectory_sampler.cpp
  src/tracking_controller/pure_pursuit.cpp
  src/navigation_metrics.cpp
  src/navigation_viz.cpp)
target_include_directories(forest_navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(forest_navigation_core
  geometry_msgs nav_msgs std_msgs visualization_msgs)
target_compile_features(forest_navigation_core PUBLIC cxx_std_17)

add_executable(navigation_node src/navigation_node.cpp)
target_link_libraries(navigation_node forest_navigation_core)
ament_target_dependencies(navigation_node geometry_msgs nav_msgs rclcpp std_msgs visualization_msgs)
target_compile_features(navigation_node PUBLIC cxx_std_17)

install(TARGETS forest_navigation_core navigation_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plot_navigation_metrics.py DESTINATION lib/${PROJECT_NAME})

ament_package()
