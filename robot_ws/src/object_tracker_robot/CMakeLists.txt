################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(object_tracker_robot)

################################################################################
# Find ament_cmake packages and libraries for ament_cmake and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Build
################################################################################
include_directories(
  ${rclpy_INCLUDE_DIRS}
)

################################################################################
# Install
################################################################################
install(PROGRAMS
  scripts/download_model.py
  config/model_config.yaml
  scripts/run_turtlebot_controller.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Declare ament specific configuration to be passed to dependent projects
################################################################################
ament_package()

################################################################################
# Test
################################################################################
