cmake_minimum_required(VERSION 3.8)
project(mm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# 安装launch文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 安装urdf文件
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

# 安装config文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# 安装world文件
install(DIRECTORY
  world
  DESTINATION share/${PROJECT_NAME}/
)

# 安装Python脚本
install(PROGRAMS
  scripts/custom_keyboard_control.py
  scripts/cmd_vel_smoother.py
  DESTINATION lib/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
