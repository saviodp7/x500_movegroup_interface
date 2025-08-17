#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <map>
#include <string>
#include <vector>

// TODO: Aggiunta profili di volo modificando ScalingFactor con void X500MoveGroupInterface::setMotionProfile(const std::string& profile)

/**
 * @brief Wrapper class for MoveIt2 MoveGroupInterface designed for X500 drone
 * @author Salvatore Del Peschio <salvatoredelpeschio@gmail.com>
 */
class X500MoveGroupInterface
{
public:
    /**
     * @brief Constructor
     * @param node ROS2 node pointer
     * @param planning_group Planning group name (default: "x500_group")
     */
    explicit X500MoveGroupInterface(
        std::shared_ptr<rclcpp::Node> node,
        const std::string& planning_group = "x500_group"
    );

    /**
     * @brief Destructor
     */
    ~X500MoveGroupInterface() = default;

    /**
     * @brief Set workspace constraints for the drone
     * @param min_x, min_y, min_z Minimum workspace bounds
     * @param max_x, max_y, max_z Maximum workspace bounds
     */
    void setWorkspaceConstraints(
        double min_x = -5.0, double min_y = -5.0, double min_z = -1.0,
        double max_x = 5.0, double max_y = 5.0, double max_z = 5.0
    );

    /**
     * @brief Set planning time and attempts
     * @param planning_time Maximum planning time in seconds
     * @param attempts Number of planning attempts
     */
    void setPlanningParameters(double planning_time = 10.0, int attempts = 3);

    /**
     * @brief Set target pose for the drone
     * @param x, y, z Position in 3D space
     * @param roll, pitch, yaw Orientation in RPY
     */
    void setTargetPose(
        double x, double y, double z,
        double roll = 0.0, double pitch = 0.0, double yaw = 0.0
    );

    /**
     * @brief Set target pose using a geometry_msgs Pose
     * @param pose Target pose for the drone
     */
    void setTargetPose(const geometry_msgs::msg::Pose& pose);

    /**
     * @brief Set target pose using joint values map (internal MoveIt representation)
     * @param joint_values Map of floating joint components
     */
    void setTargetPose(const std::map<std::string, double>& joint_values);

    /**
     * @brief Plan a trajectory to the current target
     * @return Plan object containing the trajectory (check plan.planning_time_ > 0 for success)
     */
    moveit::planning_interface::MoveGroupInterface::Plan planToTarget();

    /**
     * @brief Remember a position with given pose and name
     * @param pose The pose to remember
     * @param name Name to associate with this position
     */
    void rememberPosition(const geometry_msgs::msg::Pose& pose, const std::string& name);

    /**
     * @brief Remember a position with given coordinates and name
     * @param x, y, z Position in 3D space
     * @param roll, pitch, yaw Orientation in RPY
     * @param name Name to associate with this position
     */
    void rememberPosition(
        double x, double y, double z,
        double roll, double pitch, double yaw,
        const std::string& name
    );

    /**
     * @brief Remember the currently set target position
     * @param name Name to associate with the current target position
     * @note This remembers the last target set via setTargetPose(), not the actual robot position
     */
    void rememberCurrentTarget(const std::string& name);

    /**
     * @brief Set target to a previously remembered position
     * @param name Name of the remembered position
     * @return True if position was found and set, false otherwise
     */
    bool setTargetToRememberedPosition(const std::string& name);

    /**
     * @brief Get list of all remembered position names
     * @return Vector of position names
     */
    std::vector<std::string> getRememberedPositionNames() const;

    /**
     * @brief Clear a specific remembered position
     * @param name Name of the position to clear
     * @return True if position was found and cleared
     */
    bool clearRememberedPosition(const std::string& name);

    /**
     * @brief Clear all remembered positions
     */
    void clearAllRememberedPositions();

    /**
     * @brief Get the last planning result
     * @return The last computed plan
     */
    const moveit::planning_interface::MoveGroupInterface::Plan& getLastPlan() const;

    /**
     * @brief Check if the last planning was successful
     * @return True if last planning succeeded
     */
    bool wasLastPlanningSuccessful() const;

    /**
     * @brief Get planning group name
     * @return Planning group name
     */
    const std::string& getPlanningGroupName() const;

    /**
     * @brief Get planning frame
     * @return Planning frame name
     */
    const std::string& getPlanningFrame() const;

    /**
     * @brief Get active planner ID
     * @return Current planner identifier
     */
    std::string getActivePlannerId() const;

    /**
     * @brief Print current status and configuration
     */
    void printStatus() const;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    
    std::string planning_group_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    bool last_planning_successful_;
    
    // Map to store remembered positions as joint values
    std::map<std::string, std::map<std::string, double>> remembered_positions_;
    
    // Track last target for rememberCurrentTarget()
    std::map<std::string, double> last_target_joint_values_;
    bool last_target_valid_;
    
    // Logger
    rclcpp::Logger logger_;

    /**
     * @brief Convert geometry_msgs::Pose to joint values map
     * @param pose Input pose
     * @return Joint values map for MoveIt
     */
    std::map<std::string, double> poseToJointValues(const geometry_msgs::msg::Pose& pose) const;

    /**
     * @brief Convert joint values map to geometry_msgs::Pose
     * @param joint_values Input joint values
     * @return Pose representation
     */
    geometry_msgs::msg::Pose jointValuesToPose(const std::map<std::string, double>& joint_values) const;
};