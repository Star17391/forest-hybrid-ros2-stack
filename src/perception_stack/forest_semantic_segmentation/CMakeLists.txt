cmake_minimum_required(VERSION 3.14)
project(forest_semantic_segmentation)

find_package(ament_cmake REQUIRED)
find_package(forest_hybrid_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core dnn imgproc)

add_executable(semantic_segmentation_node src/semantic_segmentation_node.cpp)
ament_target_dependencies(semantic_segmentation_node forest_hybrid_msgs rclcpp sensor_msgs)
target_include_directories(semantic_segmentation_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(semantic_segmentation_node ${OpenCV_LIBS})
target_compile_features(semantic_segmentation_node PUBLIC cxx_std_17)

install(TARGETS semantic_segmentation_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES models/forest_semantic.onnx DESTINATION share/${PROJECT_NAME}/models)

ament_package()
