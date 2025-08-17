# 🚁 X500 Trajectory Planner

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![MoveIt2](https://img.shields.io/badge/MoveIt2-Enabled-green.svg)](https://moveit.ros.org/)

Trajectory planning service for X500 drones using MoveIt2 and ROS2.

## 📋 Description

The **X500 Trajectory Planner** provides trajectory planning services for X500 drones. This package offers a ROS2 service interface for requesting trajectory planning from a current position to a specified target position.

### ✨ Key Features

- 🎯 **3D trajectory planning** with full orientation control
- 🚀 **ROS2 service interface** for asynchronous requests
- ⚙️ **Configurable planning parameters** (time, attempts)
- 📊 **Detailed trajectory statistics** (linear distance, waypoints)
- 🛡️ **Request validation**
- 🔄 **MoveIt2 integration**

## 🔧 Dependencies

### ROS2 Dependencies
- `rclcpp` - Core ROS2 C++ API
- `geometry_msgs` - Standard geometric messages
- `trajectory_msgs` - Trajectory messages

### MoveIt2 Dependencies
- `moveit_core` - MoveIt2 core functionality
- `moveit_ros_planning` - ROS planning
- `moveit_ros_planning_interface` - Planning interface
- `moveit_msgs` - MoveIt2 messages

### TF2 Dependencies
- `tf2` - Transformation library
- `tf2_ros` - ROS bindings for TF2
- `tf2_geometry_msgs` - TF2 conversions for geometry_msgs

## 🚀 Installation

1. **Clone the repository** into your ROS2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/saviodp7/x500_trajectory_planner.git
   ```

2. **Install dependencies**:
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select x500_trajectory_planner
   source install/setup.bash
   ```

## 🎮 Usage

### Starting the Service

**Simple launch:**
```bash
ros2 launch x500_trajectory_planner x500_planning_service.launch.py
```

**Launch with custom parameters:**
```bash
ros2 launch x500_trajectory_planner x500_planning_service.launch.py \
    service_name:=my_planner \
    planning_group:=x500_group \
    log_level:=DEBUG
```

### Using the Client

**Example request via command line:**
```bash
ros2 service call /x500_planner x500_trajectory_planner/srv/X500PlanningService \
"{
  target_pose: {
    position: {x: 2.0, y: 1.0, z: 3.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  planning_time: 15.0,
  planning_attempts: 5
}"
```

**Using the example client:**
```bash
ros2 run x500_trajectory_planner x500_service_client_example
```

## 📡 Service Interface

### Request (`X500PlanningService::Request`)

| Field | Type | Description |
|-------|------|-------------|
| `target_pose` | `geometry_msgs/Pose` | Target position and orientation |
| `planning_time` | `float64` | Maximum planning time (default: 10.0s) |
| `planning_attempts` | `int32` | Number of planning attempts (default: 3) |

### Response (`X500PlanningService::Response`)

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Planning success status |
| `message` | `string` | Status/error message |
| `planning_duration` | `float64` | Planning duration (seconds) |
| `linear_distance` | `float64` | Total linear distance (meters) |
| `num_waypoints` | `int32` | Number of waypoints in trajectory |
| `trajectory_poses` | `geometry_msgs/PoseStamped[]` | Complete trajectory |

## ⚙️ Configuration

### Configurable Parameters

- **service_name**: Service name (default: `x500_planner`)
- **planning_group**: MoveIt planning group (default: `x500_group`)
- **planning_time**: Maximum planning time (default: 10.0s)
- **planning_attempts**: Number of attempts (default: 3)

### MoveIt2 Configuration Files
This package requires proper MoveIt2 configuration for the X500 drone. For a complete configuration example, see:

- [**x500_moveit_config**](https://github.com/saviodp7/x500_moveit_config) - MoveIt2 configuration package
- [**x500_description**](https://github.com/saviodp7/x500_description) - Robot description files

## 🧪 Testing

### Service Testing
```bash
# Verify service is running
ros2 service list | grep x500_planner

# Test a simple request
ros2 service call /x500_planner x500_trajectory_planner/srv/X500PlanningService \
"{target_pose: {position: {x: 1.0, y: 0.0, z: 2.0}, orientation: {w: 1.0}}}"
```

### Using the Example Client
```bash
ros2 run x500_trajectory_planner x500_service_client_example
```

## 📚 API Reference

### X500PlanningServiceServer

Main class that handles the planning service.

### X500MoveGroupInterface

Simplified wrapper for MoveIt2's MoveGroupInterface.

## 👤 Author

**Salvatore Del Peschio**
- Email: salvatoredelpeschio@gmail.com
- GitHub: [@saviodp7](https://github.com/saviodp7)
