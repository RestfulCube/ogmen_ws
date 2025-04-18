cmake_minimum_required(VERSION 3.8)
project(bot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Core build tool
find_package(ament_cmake REQUIRED)
# Xacro for URDF macros
find_package(xacro REQUIRED)
# Other dependencies
# Gazebo integration (Ignition/Gazebo) via ros_gz_sim
find_package(ros_gz_sim REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(controller_manager REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Install the URDF (Xacro) so launch files can process it at runtime
install(DIRECTORY
  launch
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()