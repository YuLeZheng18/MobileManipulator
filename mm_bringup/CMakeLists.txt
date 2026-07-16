cmake_minimum_required(VERSION 3.8)
project(mm_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# 仿真专用 mock 感知节点 (sim-only, 不进队友 mm_perception)
install(PROGRAMS
  scripts/mock_object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
