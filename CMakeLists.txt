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

# Include directories
include_directories(
  include
)

# === WRAPPER LIBRARY ===
# Create a library for the X500MoveGroupInterface class
add_library(${PROJECT_NAME}_lib
  src/x500_movegroup_interface.cpp
)

# Link dependencies for the library
ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  shape_msgs
)

# Make library headers available
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# === TEST EXECUTABLE ===
# Create the test executable
add_executable(x500_movegroup_test_node
  src/x500_movegroup_test_node.cpp
)

# Link the library and dependencies to the test executable
target_link_libraries(x500_movegroup_test_node
  ${PROJECT_NAME}_lib
)

ament_target_dependencies(x500_movegroup_test_node
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  shape_msgs
)

# === INSTALLATION ===
# Install library
install(
  TARGETS ${PROJECT_NAME}_lib
  EXPORT ${PROJECT_NAME}_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(
  TARGETS x500_movegroup_test_node
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

# Export targets for downstream packages
ament_export_targets(${PROJECT_NAME}_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  shape_msgs
)

# === TESTING ===
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()