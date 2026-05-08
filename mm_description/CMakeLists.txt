cmake_minimum_required(VERSION 3.8)
project(mm_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(omni_kinematics src/omni_kinematics.cpp)
ament_target_dependencies(omni_kinematics geometry_msgs rclcpp std_msgs)

install(TARGETS
  omni_kinematics
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf config world
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
