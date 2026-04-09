cmake_minimum_required(VERSION 3.14)
project(csb_launch)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
