cmake_minimum_required(VERSION 3.14)
project(forest_hybrid_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/MissionAck.msg"
  "msg/MissionCommand.msg"
  "msg/MissionStatus.msg"
  "msg/OperationMode.msg"
  "msg/HybridTransitionStatus.msg"
  "msg/HybridHopRequest.msg"
  "msg/HybridHopStatus.msg"
  "msg/SemanticClass.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
