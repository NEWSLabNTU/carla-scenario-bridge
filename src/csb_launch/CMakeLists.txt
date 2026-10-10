cmake_minimum_required(VERSION 3.14)
project(csb_launch)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch data
  DESTINATION share/${PROJECT_NAME}
)

# `ros2 run csb_launch run_suite`: many scenarios, one verdict each, one suite.junit.xml.
install(
  PROGRAMS scripts/run_suite
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_run_suite test/test_run_suite.py)
endif()

ament_package()
