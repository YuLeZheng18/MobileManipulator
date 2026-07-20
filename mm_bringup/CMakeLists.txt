cmake_minimum_required(VERSION 3.8)
project(mm_bringup)

find_package(ament_cmake REQUIRED)

# Only install directories that actually exist so the skeleton package
# builds before launch/config content is added.
foreach(dir launch config)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${dir}")
    install(DIRECTORY ${dir}
      DESTINATION share/${PROJECT_NAME}
    )
  endif()
endforeach()

# 仿真专用 mock 感知节点 (sim-only, 不进队友 mm_perception)
install(PROGRAMS
  scripts/mock_object_detector.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
