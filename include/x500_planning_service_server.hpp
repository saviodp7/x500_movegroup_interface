#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <cmath>

// ROS2 msgs
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

// Service definition
#include "x500_trajectory_planner/srv/x500_planning_service.hpp"

// MoveGroupInterface
#include "x500_movegroup_interface.hpp"

/**
 * @brief Service server for X500 drone trajectory planning
 * @author Salvatore Del Peschio <salvatoredelpeschio@gmail.com>
 */
class X500PlanningServiceServer
{
public:
    /**
     * @brief Constructor
     * @param node ROS2 node pointer
     * @param service_name Service name (default: "x500_planner")
     * @param planning_group Planning group (default: "x500_group")
     */
    explicit X500PlanningServiceServer(
        std::shared_ptr<rclcpp::Node> node,
        const std::string& service_name = "x500_planner",
        const std::string& planning_group = "x500_group"
    );

    /**
     * @brief Destructor
     */
    ~X500PlanningServiceServer() = default;

    /**
     * @brief Start the service server
     */
    void start();

    /**
     * @brief Stop the service server
     */
    void stop();

private:
    // ROS2 components
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Service<x500_trajectory_planner::srv::X500PlanningService>::SharedPtr service_;
    rclcpp::Logger logger_;

    // MoveGroupInterface
    std::unique_ptr<X500MoveGroupInterface> move_group_interface_;
    
    // Default planning parameters
    static constexpr double DEFAULT_PLANNING_TIME = 10.0;
    static constexpr int DEFAULT_PLANNING_ATTEMPTS = 3;

    // Configuration
    std::string service_name_;
    std::string planning_group_;
    bool service_active_;

    /**
     * @brief Service callback for trajectory planning
     * @param request Request with target pose and parameters
     * @param response Response with planned trajectory
     */
    void planTrajectoryCallback(
        const std::shared_ptr<x500_trajectory_planner::srv::X500PlanningService::Request> request,
        std::shared_ptr<x500_trajectory_planner::srv::X500PlanningService::Response> response
    );

    /**
     * @brief Extract trajectory from MoveIt plan
     * @param plan MoveIt motion plan
     * @param response Response to fill with trajectory data
     */
    void extractTrajectoryData(
        const moveit::planning_interface::MoveGroupInterface::Plan& plan,
        std::shared_ptr<x500_trajectory_planner::srv::X500PlanningService::Response> response
    );

    /**
     * @brief Calculate total linear distance of trajectory
     * @param trajectory_poses Array of trajectory poses
     * @return Total distance in meters
     */
    double calculateLinearDistance(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory_poses) const;

    /**
     * @brief Convert joint trajectory point to PoseStamped
     * @param joint_point Joint trajectory point
     * @param frame_id Reference frame
     * @param timestamp Timestamp for the pose
     * @return Corresponding PoseStamped
     */
    geometry_msgs::msg::PoseStamped jointPointToPoseStamped(
        const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint& joint_point,
        const std::string& frame_id,
        const rclcpp::Time& timestamp
    ) const;

    /**
     * @brief Validate service request
     * @param request Request to validate
     * @return True if valid, false otherwise
     */
    bool validateRequest(const std::shared_ptr<x500_trajectory_planner::srv::X500PlanningService::Request> request) const;
};