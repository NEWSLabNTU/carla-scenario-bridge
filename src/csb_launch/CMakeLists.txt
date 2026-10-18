cmake_minimum_required(VERSION 3.14)
project(csb_launch)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch data rviz
  DESTINATION share/${PROJECT_NAME}
)

# `ros2 run csb_launch run_suite`: many scenarios, one verdict each, one suite.junit.xml.
# `ros2 run csb_launch physical_verdict`: CARLA's contacts as a JUnit testcase (021).
# `ros2 run csb_launch carla_poses`: poses on CARLA's map as OpenSCENARIO WorldPositions.
# `ros2 run csb_launch chase_view`: a chase camera window on the ego, for demos (027).
# `ros2 run csb_launch gap_oracle`: SSv2-vs-CARLA disagreements in a gap trace (024).
install(
  PROGRAMS scripts/run_suite scripts/physical_verdict scripts/carla_poses scripts/chase_view
    scripts/gap_oracle scripts/incident_judge
  DESTINATION lib/${PROJECT_NAME}
)

# `isaac_world_server`: the Isaac world server as a launch node (isaac_world.launch.xml,
# roadmap 043), pointed at the csb_isaac sources beside this package (CSB_ISAAC_DIR overrides).
get_filename_component(CSB_ISAAC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../csb_isaac" ABSOLUTE)
configure_file(scripts/isaac_world_server.in ${CMAKE_CURRENT_BINARY_DIR}/isaac_world_server @ONLY)
install(
  PROGRAMS ${CMAKE_CURRENT_BINARY_DIR}/isaac_world_server
  DESTINATION lib/${PROJECT_NAME}
)

# Library modules the gap oracle (024) imports from its own directory: oriented-box
# geometry and the gap trace reader. Not executables.
install(
  FILES scripts/gap_geometry.py scripts/gap_trace.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_run_suite test/test_run_suite.py)
  ament_add_pytest_test(test_physical_verdict test/test_physical_verdict.py)
  ament_add_pytest_test(test_gap_geometry test/test_gap_geometry.py)
  ament_add_pytest_test(test_gap_trace test/test_gap_trace.py)
  ament_add_pytest_test(test_gap_oracle test/test_gap_oracle.py)
  ament_add_pytest_test(test_incident_judge test/test_incident_judge.py)
endif()

ament_package()
