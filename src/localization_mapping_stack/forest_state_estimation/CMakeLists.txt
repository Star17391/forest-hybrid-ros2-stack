cmake_minimum_required(VERSION 3.8)
project(forest_state_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(forest_hybrid_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── state_contract_node (existente) ─────────────────────────────────────────
add_executable(state_contract_node src/state_contract_node.cpp)
ament_target_dependencies(
  state_contract_node
  rclcpp geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs
)
target_link_libraries(state_contract_node tf2::tf2)

# ── gnss_cov_adapter_node ────────────────────────────────────────────────────
# Infla a covariância do NavSatFix para valores realistas sob dossel florestal.
add_executable(gnss_cov_adapter_node src/gnss_cov_adapter_node.cpp)
ament_target_dependencies(
  gnss_cov_adapter_node
  rclcpp sensor_msgs
)

# ── map_odom_authority_node ──────────────────────────────────────────────────
# Publica TF map→odom apenas em modo AERIAL (EKF global como fonte).
# No solo, o Tree-SLAM é autoridade — este nó fica silencioso.
add_executable(map_odom_authority_node src/map_odom_authority_node.cpp)
ament_target_dependencies(
  map_odom_authority_node
  rclcpp forest_hybrid_msgs geometry_msgs nav_msgs tf2_ros tf2_geometry_msgs
)
target_link_libraries(map_odom_authority_node tf2::tf2)

install(TARGETS
  state_contract_node
  gnss_cov_adapter_node
  map_odom_authority_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
