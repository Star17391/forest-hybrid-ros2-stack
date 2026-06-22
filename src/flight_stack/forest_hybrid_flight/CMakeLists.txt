cmake_minimum_required(VERSION 3.14)
project(forest_hybrid_flight)

find_package(ament_cmake REQUIRED)
find_package(forest_hybrid_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# MAVLink vendorizado (header-only). Inclui-se ardupilotmega/mavlink.h no código.
set(MAVLINK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/mavlink/v2.0")

add_executable(hybrid_hop_executor_node
  src/hybrid_hop_executor_node.cpp
  src/mavlink_client.cpp)
target_include_directories(hybrid_hop_executor_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${MAVLINK_INCLUDE_DIR})
# Os headers MAVLink gerados usam membros de structs "packed"; silenciar avisos só deles.
target_compile_options(hybrid_hop_executor_node PRIVATE
  -Wno-address-of-packed-member -Wno-cast-align)
ament_target_dependencies(hybrid_hop_executor_node
  forest_hybrid_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  std_msgs)
target_compile_features(hybrid_hop_executor_node PUBLIC cxx_std_17)

install(TARGETS hybrid_hop_executor_node DESTINATION lib/${PROJECT_NAME})
ament_package()
