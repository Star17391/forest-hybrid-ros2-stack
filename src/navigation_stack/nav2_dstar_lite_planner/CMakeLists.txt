cmake_minimum_required(VERSION 3.14)
project(nav2_dstar_lite_planner)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/dstar_lite.cpp
  src/dstar_lite_planner.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  geometry_msgs nav2_core nav2_costmap_2d nav2_util rclcpp_lifecycle
  nav_msgs pluginlib rclcpp tf2_ros)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

pluginlib_export_plugin_description_file(nav2_core dstar_lite_plugin.xml)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES dstar_lite_plugin.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  geometry_msgs nav2_core nav2_costmap_2d nav2_util rclcpp_lifecycle
  nav_msgs pluginlib rclcpp tf2_ros)

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  # Teste do core (sem ROS): valida a correção do allow_unknown (255→livre) e a
  # navegação básica (aberto / parede). Linka só o core dstar_lite.cpp.
  add_executable(dstar_lite_core_test test/dstar_lite_core_test.cpp src/dstar_lite.cpp)
  target_include_directories(dstar_lite_core_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  target_compile_features(dstar_lite_core_test PRIVATE cxx_std_17)
  ament_add_test(dstar_lite_core_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:dstar_lite_core_test>"
    TIMEOUT 30)
endif()

ament_package()
