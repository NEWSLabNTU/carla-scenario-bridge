cmake_minimum_required(VERSION 3.14)
project(csb_examples)

find_package(ament_cmake REQUIRED)

# Starter scenarios, installed to share/csb_examples/scenarios/{basic,multi_av,awf}.
# Every file names its map as $(env CARLA_MAPS)/<Town>: export CARLA_MAPS (the directory
# holding Town01/, Town02/, ...) before running one.
install(
  DIRECTORY scenarios
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
