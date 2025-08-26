cmake_minimum_required(VERSION 3.8)
project(robot_stepper_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(stepper_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Build the hardware interface library
add_library(${PROJECT_NAME} SHARED
  src/stepper_hardware_interface.cpp
)

# Link libraries
ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  stepper_msgs
)

# Export plugin description
pluginlib_export_plugin_description_file(
  hardware_interface
  config/stepper_hardware_interface.xml
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/
)
install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)


ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  controller_manager
  joint_trajectory_controller
  urdf
)

ament_package()
