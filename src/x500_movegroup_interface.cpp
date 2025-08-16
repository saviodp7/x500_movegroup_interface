#include "x500_movegroup_interface.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/workspace_parameters.hpp>

// Constructor
X500MoveGroupInterface::X500MoveGroupInterface(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& planning_group)
    : node_(node)
    , planning_group_(planning_group)
    , last_planning_successful_(false)
    , last_target_valid_(false)  // ← Nuovo
    , logger_(node->get_logger())
{
    RCLCPP_INFO(logger_, "🚁 Initializing X500MoveGroupInterface...");
    
    try {
        // Initialize MoveGroupInterface
        move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            node_, planning_group_);
        
        if (!move_group_) {
            throw std::runtime_error("Failed to create MoveGroupInterface");
        }
        
        // Set default planning parameters
        move_group_->setPlanningTime(10.0);
        move_group_->setNumPlanningAttempts(3);
        
        // Setup ScalingFactors for indoor environments
        move_group_->setMaxVelocityScalingFactor(0.5);
        move_group_->setMaxAccelerationScalingFactor(0.3);
        
        const double workspace_min_x = -0.5;
        const double workspace_max_x = 20.0;
        const double workspace_min_y = -0.5;
        const double workspace_max_y = 10.0;
        const double workspace_min_z = -0.1;
        const double workspace_max_z = 3.5;
        
        // Set default workspace constraints
        setWorkspaceConstraints(workspace_min_x, workspace_min_y, workspace_min_z,
                               workspace_max_x, workspace_max_y, workspace_max_z);
        
        RCLCPP_INFO(logger_, "✅ X500MoveGroupInterface initialized successfully");
        RCLCPP_INFO(logger_, "   Planning group: %s", planning_group_.c_str());
        RCLCPP_INFO(logger_, "   Planning frame: %s", move_group_->getPlanningFrame().c_str());
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(logger_, "❌ Failed to initialize X500MoveGroupInterface: %s", e.what());
        throw;
    }
}

// Set workspace constraints
void X500MoveGroupInterface::setWorkspaceConstraints(
    double min_x, double min_y, double min_z,
    double max_x, double max_y, double max_z)
{
    // Validate bounds
    if (min_x >= max_x || min_y >= max_y || min_z >= max_z) {
        RCLCPP_ERROR(logger_, "❌ Invalid workspace bounds: min values must be < max values");
        return;
    }
    
    move_group_->setWorkspace(min_x, min_y, min_z, max_x, max_y, max_z);
    
    RCLCPP_INFO(logger_, "🔧 Workspace constraints set:");
    RCLCPP_INFO(logger_, "   X: [%.2f, %.2f]", min_x, max_x);
    RCLCPP_INFO(logger_, "   Y: [%.2f, %.2f]", min_y, max_y);
    RCLCPP_INFO(logger_, "   Z: [%.2f, %.2f]", min_z, max_z);
}

// Set planning parameters
void X500MoveGroupInterface::setPlanningParameters(double planning_time, int attempts)
{
    if (planning_time <= 0.0 || attempts <= 0) {
        RCLCPP_ERROR(logger_, "❌ Invalid planning parameters: time=%.2f, attempts=%d", 
                     planning_time, attempts);
        return;
    }
    
    move_group_->setPlanningTime(planning_time);
    move_group_->setNumPlanningAttempts(attempts);
    
    RCLCPP_INFO(logger_, "🔧 Planning parameters set: time=%.2fs, attempts=%d", 
                planning_time, attempts);
}

// Set target pose (RPY)
void X500MoveGroupInterface::setTargetPose(
    double x, double y, double z,
    double roll, double pitch, double yaw)
{
    // Convert RPY to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    
    std::map<std::string, double> target_joints;
    target_joints["virtual_joint/trans_x"] = x;
    target_joints["virtual_joint/trans_y"] = y;
    target_joints["virtual_joint/trans_z"] = z;
    target_joints["virtual_joint/rot_x"] = q.x();
    target_joints["virtual_joint/rot_y"] = q.y();
    target_joints["virtual_joint/rot_z"] = q.z();
    target_joints["virtual_joint/rot_w"] = q.w();
    
    // Call joint_values map version
    setTargetPose(target_joints);
}

// Set target pose (geometry_msgs version)
void X500MoveGroupInterface::setTargetPose(const geometry_msgs::msg::Pose& pose)
{
    // Convert geometry_msgs::Pose to joint values map
    std::map<std::string, double> target_joints = poseToJointValues(pose);
    
    // Call joint_values map version
    setTargetPose(target_joints);
}

// Set target pose (joint values map version)
void X500MoveGroupInterface::setTargetPose(const std::map<std::string, double>& joint_values)
{
    move_group_->setJointValueTarget(joint_values);
    
    // Store last target for rememberCurrentTarget()
    last_target_joint_values_ = joint_values;
    last_target_valid_ = true;
    
    // Extract position and orientation for RPY logging
    double x = joint_values.at("virtual_joint/trans_x");
    double y = joint_values.at("virtual_joint/trans_y");
    double z = joint_values.at("virtual_joint/trans_z");
    
    // Convert quaternion back to RPY for logging
    tf2::Quaternion q(
        joint_values.at("virtual_joint/rot_x"),
        joint_values.at("virtual_joint/rot_y"),
        joint_values.at("virtual_joint/rot_z"),
        joint_values.at("virtual_joint/rot_w")
    );
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(logger_, "🎯 Target set: pos=[%.2f,%.2f,%.2f], rpy=[%.3f,%.3f,%.3f]",
                x, y, z, roll, pitch, yaw);
}

// Plan trajectory to target
moveit::planning_interface::MoveGroupInterface::Plan X500MoveGroupInterface::planToTarget()
{
    RCLCPP_INFO(logger_, "🔄 Planning trajectory...");
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Create plan object
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    // Execute planning
    moveit::core::MoveItErrorCode result = move_group_->plan(plan);
    last_planning_successful_ = (result == moveit::core::MoveItErrorCode::SUCCESS);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (last_planning_successful_) {
        // Store the successful plan
        plan_ = plan;
        
        RCLCPP_INFO(logger_, "✅ Planning successful!");
        RCLCPP_INFO(logger_, "   Planning time: %ld ms", duration.count());
    } else {
        RCLCPP_ERROR(logger_, "❌ Planning failed! Error code: %d", result.val);
        // Return empty plan on failure
        moveit::planning_interface::MoveGroupInterface::Plan empty_plan;
        plan = empty_plan;
    }
    
    return plan;
}

// Remember position (geometry_msgs::Pose version)
void X500MoveGroupInterface::rememberPosition(const geometry_msgs::msg::Pose& pose, const std::string& name)
{
    if (name.empty()) {
        RCLCPP_ERROR(logger_, "❌ Position name cannot be empty");
        return;
    }
    
    // Convert pose to joint values and store
    auto joint_values = poseToJointValues(pose);
    remembered_positions_[name] = joint_values;
    
    // Convert quaternion to RPY for logging
    tf2::Quaternion q(
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(logger_, "💾 Position '%s' remembered: [%.2f,%.2f,%.2f] RPY: [%.2f,%.2f,%.2f]",
                name.c_str(), 
                pose.position.x, pose.position.y, pose.position.z,
                roll, pitch, yaw);
}

// Remember position (RPY version)
void X500MoveGroupInterface::rememberPosition(
    double x, double y, double z,
    double roll, double pitch, double yaw,
    const std::string& name)
{
    if (name.empty()) {
        RCLCPP_ERROR(logger_, "❌ Position name cannot be empty");
        return;
    }
    
    // Convert RPY to quaternion
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    
    // Create joint values map
    std::map<std::string, double> joint_values;
    joint_values["virtual_joint/trans_x"] = x;
    joint_values["virtual_joint/trans_y"] = y;
    joint_values["virtual_joint/trans_z"] = z;
    joint_values["virtual_joint/rot_x"] = q.x();
    joint_values["virtual_joint/rot_y"] = q.y();
    joint_values["virtual_joint/rot_z"] = q.z();
    joint_values["virtual_joint/rot_w"] = q.w();
    
    remembered_positions_[name] = joint_values;
    
    RCLCPP_INFO(logger_, "💾 Position '%s' remembered: [%.2f,%.2f,%.2f] RPY=[%.3f,%.3f,%.3f]", 
                name.c_str(), x, y, z, roll, pitch, yaw);
}

// Remember current target position
void X500MoveGroupInterface::rememberCurrentTarget(const std::string& name)
{
    if (name.empty()) {
        RCLCPP_ERROR(logger_, "❌ Position name cannot be empty");
        return;
    }
    
    if (!last_target_valid_) {
        RCLCPP_ERROR(logger_, "❌ No target has been set yet. Use setTargetPose() first.");
        return;
    }
    
    // Salva TUTTI i 7 DOF (posizione XYZ + quaternione XYZW)
    remembered_positions_[name] = last_target_joint_values_;
    
    // Extract position and orientation for logging
    double x = last_target_joint_values_.at("virtual_joint/trans_x");
    double y = last_target_joint_values_.at("virtual_joint/trans_y");
    double z = last_target_joint_values_.at("virtual_joint/trans_z");
    
    tf2::Quaternion q(
        last_target_joint_values_.at("virtual_joint/rot_x"),
        last_target_joint_values_.at("virtual_joint/rot_y"),
        last_target_joint_values_.at("virtual_joint/rot_z"),
        last_target_joint_values_.at("virtual_joint/rot_w")
    );
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    RCLCPP_INFO(logger_, "💾 Current target remembered as '%s':", name.c_str());
    RCLCPP_INFO(logger_, "   Position: [%.2f, %.2f, %.2f] m", x, y, z);
    RCLCPP_INFO(logger_, "   Orientation: [%.3f, %.3f, %.3f] rad (RPY)", roll, pitch, yaw);
}

// Set target to remembered position
bool X500MoveGroupInterface::setTargetToRememberedPosition(const std::string& name)
{
    if (remembered_positions_.find(name) == remembered_positions_.end()) {
        RCLCPP_ERROR(logger_, "❌ Position '%s' not found in memory", name.c_str());
        return false;
    }
    
    setTargetPose(remembered_positions_[name]);
    RCLCPP_INFO(logger_, "📍 Target set to remembered position '%s'", name.c_str());
    return true;
}

// Get remembered position names
std::vector<std::string> X500MoveGroupInterface::getRememberedPositionNames() const
{
    std::vector<std::string> names;
    names.reserve(remembered_positions_.size());
    
    for (const auto& pair : remembered_positions_) {
        names.push_back(pair.first);
    }
    
    return names;
}

// Clear specific remembered position
bool X500MoveGroupInterface::clearRememberedPosition(const std::string& name)
{
    auto it = remembered_positions_.find(name);
    if (it == remembered_positions_.end()) {
        RCLCPP_WARN(logger_, "⚠️  Position '%s' not found in memory", name.c_str());
        return false;
    }
    
    remembered_positions_.erase(it);
    RCLCPP_INFO(logger_, "🗑️  Position '%s' cleared from memory", name.c_str());
    return true;
}

// Clear all remembered positions
void X500MoveGroupInterface::clearAllRememberedPositions()
{
    size_t count = remembered_positions_.size();
    remembered_positions_.clear();
    RCLCPP_INFO(logger_, "🗑️  All %zu remembered positions cleared", count);
}

// Get last plan
const moveit::planning_interface::MoveGroupInterface::Plan& X500MoveGroupInterface::getLastPlan() const
{
    return plan_;
}

// Check if last planning was successful
bool X500MoveGroupInterface::wasLastPlanningSuccessful() const
{
    return last_planning_successful_;
}

// Get planning group name
const std::string& X500MoveGroupInterface::getPlanningGroupName() const
{
    return planning_group_;
}

// Get active planner ID
std::string X500MoveGroupInterface::getActivePlannerId() const
{
    return move_group_->getPlannerId();
}

// Print status
void X500MoveGroupInterface::printStatus() const
{
    RCLCPP_INFO(logger_, "📊 X500MoveGroupInterface Status:");
    RCLCPP_INFO(logger_, "   Planning Group: %s", planning_group_.c_str());
    RCLCPP_INFO(logger_, "   Planning Frame: %s", move_group_->getPlanningFrame().c_str());
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    RCLCPP_INFO(logger_, "   Active Planner: %s", move_group_->getPlannerId().c_str());
    RCLCPP_INFO(logger_, "   Last Planning: %s", 
                last_planning_successful_ ? "✅ SUCCESS" : "❌ FAILED");
    RCLCPP_INFO(logger_, "   Last Target Valid: %s", 
                last_target_valid_ ? "✅ YES" : "❌ NO");
    RCLCPP_INFO(logger_, "   Remembered Positions: %zu", remembered_positions_.size());
    
    if (!remembered_positions_.empty()) {
        RCLCPP_INFO(logger_, "   Remembered position names:");
        for (const auto& pair : remembered_positions_) {
            RCLCPP_INFO(logger_, "     - %s", pair.first.c_str());
        }
    }
}

// === PRIVATE HELPER METHODS ===

// Convert geometry_msgs::Pose to joint values
std::map<std::string, double> X500MoveGroupInterface::poseToJointValues(const geometry_msgs::msg::Pose& pose) const
{
    std::map<std::string, double> joint_values;
    
    // Position components
    joint_values["virtual_joint/trans_x"] = pose.position.x;
    joint_values["virtual_joint/trans_y"] = pose.position.y;
    joint_values["virtual_joint/trans_z"] = pose.position.z;
    
    // Orientation components (quaternion)
    joint_values["virtual_joint/rot_x"] = pose.orientation.x;
    joint_values["virtual_joint/rot_y"] = pose.orientation.y;
    joint_values["virtual_joint/rot_z"] = pose.orientation.z;
    joint_values["virtual_joint/rot_w"] = pose.orientation.w;
    
    return joint_values;
}

// Convert joint values to geometry_msgs::Pose
geometry_msgs::msg::Pose X500MoveGroupInterface::jointValuesToPose(const std::map<std::string, double>& joint_values) const
{
    geometry_msgs::msg::Pose pose;
    
    // Position components
    auto it = joint_values.find("virtual_joint/trans_x");
    if (it != joint_values.end()) pose.position.x = it->second;
    
    it = joint_values.find("virtual_joint/trans_y");
    if (it != joint_values.end()) pose.position.y = it->second;
    
    it = joint_values.find("virtual_joint/trans_z");
    if (it != joint_values.end()) pose.position.z = it->second;
    
    // Orientation components
    it = joint_values.find("virtual_joint/rot_x");
    if (it != joint_values.end()) pose.orientation.x = it->second;
    
    it = joint_values.find("virtual_joint/rot_y");
    if (it != joint_values.end()) pose.orientation.y = it->second;
    
    it = joint_values.find("virtual_joint/rot_z");
    if (it != joint_values.end()) pose.orientation.z = it->second;
    
    it = joint_values.find("virtual_joint/rot_w");
    if (it != joint_values.end()) pose.orientation.w = it->second;
    else pose.orientation.w = 1.0; // Default to identity quaternion
    
    return pose;
}