cmake_minimum_required(VERSION 3.8)
project(forest_sensors_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(static_sensor_tf_node src/static_sensor_tf_node.cpp)
ament_target_dependencies(static_sensor_tf_node rclcpp geometry_msgs tf2_ros)

add_executable(laserscan_to_pointcloud2_node src/laserscan_to_pointcloud2_node.cpp)
ament_target_dependencies(laserscan_to_pointcloud2_node rclcpp sensor_msgs)

add_executable(laserscan_preprocess_node src/laserscan_preprocess_node.cpp)
ament_target_dependencies(laserscan_preprocess_node rclcpp sensor_msgs geometry_msgs)
target_include_directories(laserscan_preprocess_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(laserscan_preprocess_node tf2::tf2)

add_executable(tracked_wheel_odometry_node src/tracked_wheel_odometry_node.cpp)
ament_target_dependencies(tracked_wheel_odometry_node rclcpp nav_msgs geometry_msgs std_msgs)

add_executable(imu_sanitize_node src/imu_sanitize_node.cpp)
ament_target_dependencies(imu_sanitize_node rclcpp sensor_msgs)
target_include_directories(imu_sanitize_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(laserscan_to_pointcloud2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(
  TARGETS
    static_sensor_tf_node
    laserscan_to_pointcloud2_node
    laserscan_preprocess_node
    tracked_wheel_odometry_node
    imu_sanitize_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
