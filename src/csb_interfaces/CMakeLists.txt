cmake_minimum_required(VERSION 3.14)
project(csb_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Weather.msg"
  "srv/SetWeather.srv"
  "srv/GetWeather.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
