cmake_minimum_required(VERSION 3.14)
project(forest_navigation_ros2)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(FOREST_NAV_CORE_SOURCES
  src/global_planner/global_planner.cpp
  src/local_planner/local_planner.cpp
  src/trajectory_sampler/trajectory_sampler.cpp
  src/tracking_controller/pure_pursuit.cpp
  src/tracking_controller/nmpc_skid_steer.cpp
  src/tracking_controller/tracking_controller_interface.cpp
  src/navigation_metrics.cpp
  src/navigation_viz.cpp)

set(ACADOS_GEN_DIR "${CMAKE_CURRENT_SOURCE_DIR}/acados_generated/skid_steer_kin")
set(ACADOS_SOLVER_HEADER "${ACADOS_GEN_DIR}/acados_solver_skid_steer_kin.h")

if(DEFINED ENV{ACADOS_SOURCE_DIR} AND EXISTS "${ACADOS_SOLVER_HEADER}")
  message(STATUS "forest_navigation_ros2: building with acados NMPC")

  set(ACADOS_INSTALL_DIR "$ENV{ACADOS_SOURCE_DIR}")

  file(GLOB ACADOS_OCP_C_SOURCES
    "${ACADOS_GEN_DIR}/acados_solver_skid_steer_kin.c"
    "${ACADOS_GEN_DIR}/skid_steer_kin_model/*.c"
    "${ACADOS_GEN_DIR}/skid_steer_kin_cost/*.c")

  add_library(forest_nmpc_skid_steer_kin SHARED ${ACADOS_OCP_C_SOURCES})
  target_include_directories(forest_nmpc_skid_steer_kin PUBLIC
    "${ACADOS_GEN_DIR}"
    "${ACADOS_GEN_DIR}/skid_steer_kin_model"
    "${ACADOS_GEN_DIR}/skid_steer_kin_cost"
    "${ACADOS_INSTALL_DIR}/include"
    "${ACADOS_INSTALL_DIR}/include/acados"
    "${ACADOS_INSTALL_DIR}/include/blasfeo/include"
    "${ACADOS_INSTALL_DIR}/include/hpipm/include")
  target_link_directories(forest_nmpc_skid_steer_kin PUBLIC "${ACADOS_INSTALL_DIR}/lib")
  target_link_libraries(forest_nmpc_skid_steer_kin PUBLIC acados hpipm blasfeo m)
  set_target_properties(forest_nmpc_skid_steer_kin PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    OUTPUT_NAME acados_ocp_solver_skid_steer_kin
    BUILD_RPATH "${ACADOS_INSTALL_DIR}/lib"
    INSTALL_RPATH "${ACADOS_INSTALL_DIR}/lib")

  set(FOREST_NAV_ENABLE_NMPC 1)
else()
  message(WARNING
    "forest_navigation_ros2: NMPC disabled. Set ACADOS_SOURCE_DIR and run "
    "scripts/generate_nmpc_solver.sh to enable controller_type:=nmpc")
  set(FOREST_NAV_ENABLE_NMPC 0)
endif()

add_library(forest_navigation_core SHARED ${FOREST_NAV_CORE_SOURCES})
target_include_directories(forest_navigation_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

if(FOREST_NAV_ENABLE_NMPC)
  target_compile_definitions(forest_navigation_core PUBLIC FOREST_NAV_ENABLE_NMPC=1)
  target_include_directories(forest_navigation_core PRIVATE
    "${ACADOS_GEN_DIR}"
    "$ENV{ACADOS_SOURCE_DIR}/include"
    "$ENV{ACADOS_SOURCE_DIR}/include/acados")
  target_link_libraries(forest_navigation_core forest_nmpc_skid_steer_kin)
endif()

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

if(FOREST_NAV_ENABLE_NMPC)
  install(TARGETS forest_nmpc_skid_steer_kin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/plot_navigation_metrics.py DESTINATION lib/${PROJECT_NAME})

ament_package()
