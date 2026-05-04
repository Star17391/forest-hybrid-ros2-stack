cmake_minimum_required(VERSION 3.14)
project(forest_hybrid_conf)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
