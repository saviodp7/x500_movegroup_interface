cmake_minimum_required(VERSION 3.8)
project(x500_movegroup_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 standard required for MoveIt2
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(
  include
)

# Create the executable
add_executable(x500_movegroup_interface_node 
  src/x500_movegroup_interface_node.cpp
)

# Link dependencies
ament_target_dependencies(
  x500_movegroup_interface_node
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  shape_msgs
)

# Install the executable
install(
  TARGETS x500_movegroup_interface_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install launch files if any
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
  OPTIONAL
)

# Install config files if any
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()