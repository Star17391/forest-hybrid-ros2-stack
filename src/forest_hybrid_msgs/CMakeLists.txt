cmake_minimum_required(VERSION 3.14)
project(forest_hybrid_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/MissionAck.msg"
  "msg/MissionCommand.msg"
  "msg/MissionStatus.msg"
  "msg/OperationMode.msg"
  "msg/HybridTransitionStatus.msg"
  "msg/HybridHopRequest.msg"
  "msg/HybridHopStatus.msg"
  "msg/SemanticClass.msg"
  "msg/TreeLandmark.msg"
  "msg/TreeLandmarkArray.msg"
  "msg/TrackedTreeLandmark.msg"
  "msg/TrackedTreeLandmarkArray.msg"
  "msg/SlamStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
