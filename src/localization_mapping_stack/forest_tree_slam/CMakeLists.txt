cmake_minimum_required(VERSION 3.8)
project(forest_tree_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(forest_hybrid_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(forest_3d_perception REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)

include_directories(include)

# --- Núcleo agnóstico de ROS: tracker, backend GTSAM, relocalizador, FSM ---
add_library(tree_slam_core
  src/tracker.cpp
  src/multiview_dbh.cpp
  src/backend.cpp
  src/relocalizer.cpp
  src/mode_manager.cpp
)
target_include_directories(tree_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${forest_3d_perception_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(tree_slam_core PUBLIC
  Eigen3::Eigen
  gtsam
  ${forest_3d_perception_LIBRARIES}
  ${PCL_LIBRARIES}
)

# --- Nó ROS2 ----------------------------------------------------------
add_executable(tree_slam_node src/tree_slam_node.cpp)
target_link_libraries(tree_slam_node tree_slam_core)
ament_target_dependencies(tree_slam_node
  rclcpp geometry_msgs nav_msgs visualization_msgs sensor_msgs tf2 tf2_ros
  tf2_geometry_msgs forest_hybrid_msgs
)

install(TARGETS tree_slam_node tree_slam_core
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_tracker test/test_tracker.cpp)
  target_link_libraries(test_tracker tree_slam_core)

  ament_add_gtest(test_backend test/test_backend.cpp)
  target_link_libraries(test_backend tree_slam_core)

  ament_add_gtest(test_relocalizer test/test_relocalizer.cpp)
  target_link_libraries(test_relocalizer tree_slam_core)

  ament_add_gtest(test_mode_manager test/test_mode_manager.cpp)
  target_link_libraries(test_mode_manager tree_slam_core)

  # Testes de diagnóstico da cola do nó (isolam Causa nº1/nº2/tracker).
  ament_add_gtest(test_multiview_dbh test/test_multiview_dbh.cpp)
  target_link_libraries(test_multiview_dbh tree_slam_core)

  ament_add_gtest(test_multiview_class_score test/test_multiview_class_score.cpp)
  target_link_libraries(test_multiview_class_score tree_slam_core)

  ament_add_gtest(test_node_glue test/test_node_glue.cpp)
  target_link_libraries(test_node_glue tree_slam_core)
endif()

ament_package()
