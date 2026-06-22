cmake_minimum_required(VERSION 3.8)
project(forest_3d_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(forest_hybrid_msgs REQUIRED)
find_package(PCL 1.10 REQUIRED COMPONENTS common filters segmentation search kdtree)

# --- CSF library for experimental pipeline ---
set(CSF_VENDOR_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/csf")
if(EXISTS "${CSF_VENDOR_DIR}/CSF.cpp")
  message(STATUS "forest_3d_perception: using vendored CSF in third_party/csf")
  set(CSF_SOURCE_DIR "${CSF_VENDOR_DIR}")
else()
  include(FetchContent)
  FetchContent_Declare(
    csf_vendor
    GIT_REPOSITORY https://github.com/jianboqi/CSF.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(csf_vendor)
  set(CSF_SOURCE_DIR "${csf_vendor_SOURCE_DIR}/src")
  message(STATUS "forest_3d_perception: CSF fetched to ${CSF_SOURCE_DIR}")
endif()

file(GLOB CSF_SOURCES "${CSF_SOURCE_DIR}/*.cpp")
if(NOT CSF_SOURCES)
  message(FATAL_ERROR
    "CSF sources not found. Run: bash scripts/fetch_csf_vendor.sh "
    "or ensure network access during cmake (FetchContent).")
endif()

add_library(forest_csf STATIC ${CSF_SOURCES})
target_include_directories(forest_csf PUBLIC "${CSF_SOURCE_DIR}")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(forest_csf PRIVATE -Wno-pedantic)
endif()

# --- Scorer suave reutilizável (P-C / S-F) ---
add_library(forest_landmark_class_scorer src/landmark_class_scorer.cpp)
target_include_directories(forest_landmark_class_scorer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(forest_landmark_class_scorer PUBLIC ${PCL_LIBRARIES})

# --- Experimental node (parallel pipeline) ---
add_executable(lidar3d_experimental_node src/lidar3d_experimental_node.cpp)
target_include_directories(lidar3d_experimental_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${CSF_SOURCE_DIR}
)
target_link_libraries(lidar3d_experimental_node forest_csf ${PCL_LIBRARIES})
ament_target_dependencies(lidar3d_experimental_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_conversions
  tf2_sensor_msgs
  forest_hybrid_msgs
)

# --- Offline CSF regression (movableFilter / slope_smooth) ---
add_executable(csf_ground_segmentation_smoke_test test/csf_ground_segmentation_smoke_test.cpp)
target_include_directories(csf_ground_segmentation_smoke_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
  ${CSF_SOURCE_DIR}
)
target_link_libraries(csf_ground_segmentation_smoke_test forest_csf ${PCL_LIBRARIES})

# --- Robust DBH: Kåsa circle fit + gravity-align (perception Agente 2) ---
add_executable(cylinder_fit_dbh_smoke_test test/cylinder_fit_dbh_smoke_test.cpp)
target_include_directories(cylinder_fit_dbh_smoke_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(cylinder_fit_dbh_smoke_test ${PCL_LIBRARIES})

add_executable(trunk_band_canopy_rejection_test test/trunk_band_canopy_rejection_test.cpp)
target_include_directories(trunk_band_canopy_rejection_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(trunk_band_canopy_rejection_test ${PCL_LIBRARIES})

add_executable(soft_cluster_scorer_test test/soft_cluster_scorer_test.cpp)
target_include_directories(soft_cluster_scorer_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(soft_cluster_scorer_test ${PCL_LIBRARIES})

add_executable(accumulated_cloud_scorer_test test/accumulated_cloud_scorer_test.cpp)
target_include_directories(accumulated_cloud_scorer_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(accumulated_cloud_scorer_test ${PCL_LIBRARIES})

add_executable(soft_scorer_continuity_test test/soft_scorer_continuity_test.cpp)
target_include_directories(soft_scorer_continuity_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(soft_scorer_continuity_test ${PCL_LIBRARIES})

add_executable(path_b_no_ground_test test/path_b_no_ground_test.cpp)
target_include_directories(path_b_no_ground_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(path_b_no_ground_test ${PCL_LIBRARIES})

if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  ament_add_test(
    csf_ground_segmentation_smoke_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:csf_ground_segmentation_smoke_test>"
    OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/csf_smoke_test.log"
    TIMEOUT 180
  )
  ament_add_test(
    cylinder_fit_dbh_smoke_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:cylinder_fit_dbh_smoke_test>"
    OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/cylinder_fit_dbh_smoke_test.log"
    TIMEOUT 30
  )
  ament_add_test(
    trunk_band_canopy_rejection_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:trunk_band_canopy_rejection_test>"
    OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/trunk_band_canopy_rejection_test.log"
    TIMEOUT 30
  )
  ament_add_test(
    soft_cluster_scorer_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:soft_cluster_scorer_test>"
    TIMEOUT 30
  )
  ament_add_test(
    accumulated_cloud_scorer_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:accumulated_cloud_scorer_test>"
    TIMEOUT 30
  )
  ament_add_test(
    soft_scorer_continuity_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:soft_scorer_continuity_test>"
    TIMEOUT 30
  )
  ament_add_test(
    path_b_no_ground_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:path_b_no_ground_test>"
    OUTPUT_FILE "${CMAKE_CURRENT_BINARY_DIR}/path_b_no_ground_test.log"
    TIMEOUT 30
  )
endif()

install(TARGETS
  lidar3d_experimental_node
  forest_landmark_class_scorer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/forest_3d_perception
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(forest_landmark_class_scorer)

ament_package()
