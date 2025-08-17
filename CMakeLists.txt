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
find_package(shape_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(
  include
)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/X500PlanningService.srv"
  DEPENDENCIES geometry_msgs
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
  trajectory_msgs
)

# Make library headers available
target_include_directories(${PROJECT_NAME}_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# === SERVICE SERVER LIBRARY ===
add_library(${PROJECT_NAME}_service_lib
  src/x500_planning_service_server.cpp
)


# Link dependencies including generated interfaces
ament_target_dependencies(${PROJECT_NAME}_service_lib
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_service_lib
  ${PROJECT_NAME}_lib
)

# Link with generated interfaces
rosidl_target_interfaces(${PROJECT_NAME}_service_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

# === EXECUTABLES ===

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

# # Service Server Node
# add_executable(x500_planning_service_node
#   src/x500_planning_service_node.cpp
# )

# target_link_libraries(x500_planning_service_node
#   ${PROJECT_NAME}_service_lib
# )

# ament_target_dependencies(x500_planning_service_node
#   rclcpp
# )

# rosidl_target_interfaces(x500_planning_service_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# # Service Client Example
# add_executable(x500_service_client_example
#   src/x500_service_client_example.cpp
# )

# ament_target_dependencies(x500_service_client_example
#   rclcpp
#   geometry_msgs
#   tf2
#   tf2_geometry_msgs
# )

# === INSTALLATION ===
# Install libraries
install(
  TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_service_lib
  EXPORT ${PROJECT_NAME}_libTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install executables
install(
  TARGETS 
    x500_movegroup_test_node
    # x500_planning_service_node
    # x500_service_client_example
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Install config files (if any)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
  OPTIONAL
)

# Export information
ament_export_targets(${PROJECT_NAME}_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  trajectory_msgs
  tf2
  tf2_geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()