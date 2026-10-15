cmake_minimum_required(VERSION 3.8)
project(kinova_moveit)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(joint_teleop src/joint_teleop.cpp)
ament_target_dependencies(joint_teleop
  rclcpp rclcpp_action control_msgs trajectory_msgs)

add_executable(keyboard_joy_teleop src/keyboard_joy_teleop.cpp)
ament_target_dependencies(keyboard_joy_teleop
  rclcpp rclcpp_action control_msgs trajectory_msgs sensor_msgs)

add_executable(keyboard_cartesian_plan src/keyboard_cartesian_plan.cpp)
ament_target_dependencies(keyboard_cartesian_plan
  rclcpp rclcpp_action control_msgs trajectory_msgs moveit_msgs
  geometry_msgs tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS
  joint_teleop
  keyboard_joy_teleop
  keyboard_cartesian_plan
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY urdf meshes config rviz launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
