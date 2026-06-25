cmake_minimum_required(VERSION 3.8)
project(mm_navigation)

# find dependencies
find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY launch maps config
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/waypoint_patrol.py
  scripts/lane_navigator.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
