#include "x500_planning_service_server.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Constructor
X500PlanningServiceServer::X500PlanningServiceServer(
    std::shared_ptr<rclcpp::Node> node,
    const std::string& service_name,
    const std::string& planning_group)
    : node_(node)                          
    , service_(nullptr)                    
    , logger_(node->get_logger())          
    , move_group_interface_(nullptr)  
    , service_name_(service_name)        
    , planning_group_(planning_group)   
    , service_active_(false)            
{
    RCLCPP_INFO(logger_, "🛠️  Initializing X500PlanningServiceServer...");
    
    try {
        // Initialize MoveGroupInterface
        move_group_interface_ = std::make_unique<X500MoveGroupInterface>(node_, planning_group_);
        
        RCLCPP_INFO(logger_, "✅ X500PlanningServiceServer initialized successfully");
        RCLCPP_INFO(logger_, "   Service name: %s", service_name_.c_str());
        RCLCPP_INFO(logger_, "   Planning group: %s", planning_group_.c_str());
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(logger_, "❌ Failed to initialize X500PlanningServiceServer: %s", e.what());
        throw;
    }
}

// Start service
void X500PlanningServiceServer::start()
{
    if (service_active_) {
        RCLCPP_WARN(logger_, "⚠️  Service is already active!");
        return;
    }
    
    // Create service
    service_ = node_->create_service<x500_trajectory_planner::srv::X500PlanningService>(
        service_name_,
        std::bind(&X500PlanningServiceServer::planTrajectoryCallback, 
                  this, std::placeholders::_1, std::placeholders::_2)
    );
    
    service_active_ = true;
    RCLCPP_INFO(logger_, "🚀 X500 Planning Service started on: %s", service_name_.c_str());
}

// Stop service
void X500PlanningServiceServer::stop()
{
    if (!service_active_) {
        RCLCPP_WARN(logger_, "⚠️  Service is not active!");
        return;
    }
    
    service_.reset();
    service_active_ = false;
    RCLCPP_INFO(logger_, "🛑 X500 Planning Service stopped");
}

// Service callback
void X500PlanningServiceServer::planTrajectoryCallback(
    const std::shared_ptr<x500_trajectory_planner::srv::X500PlanningService::Request> request,
    std::shared_ptr<x500_trajectory_planner::srv::X500PlanningService::Response> response)
{
    RCLCPP_INFO(logger_, "🎯 Received planning request:");
    RCLCPP_INFO(logger_, "   Target: [%.2f, %.2f, %.2f]", 
                request->target_pose.position.x,
                request->target_pose.position.y,
                request->target_pose.position.z);
    RCLCPP_INFO(logger_, "   Orient: [%.3f, %.3f, %.3f, %.3f]", 
            request->target_pose.orientation.x,
            request->target_pose.orientation.y,
            request->target_pose.orientation.z,
            request->target_pose.orientation.w);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Validate request
    if (!validateRequest(request)) {
        response->success = false;
        response->message = "Invalid request parameters";
        RCLCPP_ERROR(logger_, "❌ Invalid request parameters");
        return;
    }
    
    try {
        // Set planning parameters if specified
        double planning_time = (request->planning_time > 0.0) ? request->planning_time : DEFAULT_PLANNING_TIME;
        int planning_attempts = (request->planning_attempts > 0) ? request->planning_attempts : DEFAULT_PLANNING_ATTEMPTS;
        
        move_group_interface_->setPlanningParameters(planning_time, planning_attempts);
        
        // Set target pose
        move_group_interface_->setTargetPose(request->target_pose);
        
        // Plan trajectory
        auto plan = move_group_interface_->planToTarget();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        response->planning_duration = duration.count() / 1000.0;
        
        // Check if planning was successful
        if (move_group_interface_->wasLastPlanningSuccessful() && plan.planning_time_ > 0) {
            response->success = true;
            response->message = "Planning successful";
            
            // Extract trajectory data
            extractTrajectoryData(plan, response);
            
            RCLCPP_INFO(logger_, "✅ Planning successful!");
            RCLCPP_INFO(logger_, "   Planning duration: %.3f s", response->planning_duration);
            RCLCPP_INFO(logger_, "   Linear distance: %.3f m", response->linear_distance);
            RCLCPP_INFO(logger_, "   Waypoints: %d", response->num_waypoints);
            
        } else {
            response->success = false;
            response->message = "Planning failed - no valid trajectory found";
            response->linear_distance = 0.0;
            response->num_waypoints = 0;
            
            RCLCPP_ERROR(logger_, "❌ Planning failed!");
        }
        
    } catch (const std::exception& e) {
        response->success = false;
        response->message = std::string("Planning error: ") + e.what();
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        response->planning_duration = duration.count() / 1000.0;
        
        RCLCPP_ERROR(logger_, "❌ Planning exception: %s", e.what());
    }
}

// Extract trajectory data from MoveIt plan
void X500PlanningServiceServer::extractTrajectoryData(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    std::shared_ptr<x500_trajectory_planner::srv::X500PlanningService::Response> response)
{
    const auto& joint_trajectory = plan.trajectory_.multi_dof_joint_trajectory;
    
    if (joint_trajectory.points.empty()) {
        RCLCPP_WARN(logger_, "⚠️  Empty trajectory received from planner");
        return;
    }
    
    response->num_waypoints = static_cast<int32_t>(joint_trajectory.points.size());
    
    // Reserve space for trajectory data
    response->trajectory_poses.reserve(joint_trajectory.points.size());
    
    std::string frame_id = move_group_interface_->getPlanningFrame();
    rclcpp::Time base_time = node_->get_clock()->now();
    
    // Extract each trajectory point
    for (size_t i = 0; i < joint_trajectory.points.size(); ++i) {
        const auto& point = joint_trajectory.points[i];
        
        // Convert to PoseStamped
        auto pose_stamped = jointPointToPoseStamped(point, frame_id, base_time);
        response->trajectory_poses.push_back(pose_stamped);
    }
    
    // Calculate linear distance
    response->linear_distance = calculateLinearDistance(response->trajectory_poses);
}

// Calculate linear distance of trajectory
double X500PlanningServiceServer::calculateLinearDistance(
    const std::vector<geometry_msgs::msg::PoseStamped>& trajectory_poses) const
{
    if (trajectory_poses.size() < 2) {
        return 0.0;
    }
    
    double total_distance = 0.0;
    
    for (size_t i = 1; i < trajectory_poses.size(); ++i) {
        const auto& p1 = trajectory_poses[i-1].pose.position;
        const auto& p2 = trajectory_poses[i].pose.position;
        
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        
        double segment_distance = std::sqrt(dx*dx + dy*dy + dz*dz);
        total_distance += segment_distance;
    }
    
    return total_distance;
}

// Convert joint trajectory point to PoseStamped
geometry_msgs::msg::PoseStamped X500PlanningServiceServer::jointPointToPoseStamped(
    const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& joint_point,
    const std::string& frame_id,
    const rclcpp::Time& timestamp) const
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = frame_id;
    pose_stamped.header.stamp = timestamp;
    
    // For multi-DOF joint trajectory, data is in transforms array
    if (!joint_point.transforms.empty()) {
        const auto& transform = joint_point.transforms[0];  // First transform
        pose_stamped.pose.position.x = transform.translation.x;
        pose_stamped.pose.position.y = transform.translation.y;
        pose_stamped.pose.position.z = transform.translation.z;
        
        pose_stamped.pose.orientation = transform.rotation;
    } else {
        RCLCPP_WARN(logger_, "⚠️  Empty transforms in multi-DOF trajectory point");
        pose_stamped.pose.orientation.w = 1.0;  // Identity quaternion
    }
    
    return pose_stamped;
}

// Validate service request
bool X500PlanningServiceServer::validateRequest(
    const std::shared_ptr<x500_trajectory_planner::srv::X500PlanningService::Request> request) const
{
    // Validate target pose position
    const auto& pos = request->target_pose.position;
    if (std::isnan(pos.x) || std::isnan(pos.y) || std::isnan(pos.z) ||
        std::isinf(pos.x) || std::isinf(pos.y) || std::isinf(pos.z)) {
        return false;
    }
    
    // Validate target pose orientation (quaternion should be normalized)
    const auto& ori = request->target_pose.orientation;
    if (std::isnan(ori.x) || std::isnan(ori.y) || std::isnan(ori.z) || std::isnan(ori.w) ||
        std::isinf(ori.x) || std::isinf(ori.y) || std::isinf(ori.z) || std::isinf(ori.w)) {
        return false;
    }
    
    // Check quaternion magnitude (should be close to 1.0)
    double quat_magnitude = std::sqrt(ori.x*ori.x + ori.y*ori.y + ori.z*ori.z + ori.w*ori.w);
    if (std::abs(quat_magnitude - 1.0) > 0.1) {  // Allow some tolerance
        RCLCPP_WARN(logger_, "⚠️  Quaternion not normalized: magnitude = %.3f", quat_magnitude);
        return false;
    }
    
    // Validate planning parameters if specified
    if (request->planning_time < 0.0 || request->planning_attempts < 0) {
        return false;
    }
    
    return true;
}