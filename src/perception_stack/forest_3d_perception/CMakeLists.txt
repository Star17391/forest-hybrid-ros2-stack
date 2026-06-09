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

# --- Legacy node (unchanged) ---
add_executable(lidar3d_segmentation_node src/lidar3d_segmentation_node.cpp)
target_include_directories(lidar3d_segmentation_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(lidar3d_segmentation_node ${PCL_LIBRARIES})
ament_target_dependencies(lidar3d_segmentation_node
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
)

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
add_executable(tree_candidate_extractor_smoke_test test/tree_candidate_extractor_smoke_test.cpp)
target_include_directories(tree_candidate_extractor_smoke_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(tree_candidate_extractor_smoke_test ${PCL_LIBRARIES})

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
    tree_candidate_extractor_smoke_test
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:tree_candidate_extractor_smoke_test>"
    TIMEOUT 30
  )
endif()

install(TARGETS
  lidar3d_segmentation_node
  lidar3d_experimental_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/forest_3d_perception/experimental
  DESTINATION include
)

ament_package()
