#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include <thread>

#include "x500_trajectory_planner/srv/x500_planning_service.hpp"

/**
 * @brief Example client to test the X500 Planning Service
 */
class X500ServiceClient
{
public:
    explicit X500ServiceClient(std::shared_ptr<rclcpp::Node> node)
        : node_(node), logger_(node->get_logger())
    {
        // Create service client
        client_ = node_->create_client<x500_trajectory_planner::srv::X500PlanningService>(
            "x500_planner");
    }

    /**
     * @brief Request planning for a target position
     */
    void requestPlanning(double x, double y, double z, 
                        double roll = 0.0, double pitch = 0.0, double yaw = 0.0,
                        double planning_time = 10.0, int attempts = 3)
    {
        RCLCPP_INFO(logger_, "🎯 Requesting planning to: [%.2f, %.2f, %.2f] RPY: [%.3f, %.3f, %.3f]", 
            x, y, z, roll, pitch, yaw);
        
        // Wait for service to be available
        if (!client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(logger_, "❌ Service not available after 2 seconds");
            return;
        }
        
        // Create request
        auto request = std::make_shared<x500_trajectory_planner::srv::X500PlanningService::Request>();
        
        // Set target pose
        request->target_pose.position.x = x;
        request->target_pose.position.y = y;
        request->target_pose.position.z = z;
        
        // Convert RPY to quaternion
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        q.normalize();
        request->target_pose.orientation.x = q.x();
        request->target_pose.orientation.y = q.y();
        request->target_pose.orientation.z = q.z();
        request->target_pose.orientation.w = q.w();
        
        // Set planning parameters
        request->planning_time = planning_time;
        request->planning_attempts = attempts;
        
        // Send async request
        auto future = client_->async_send_request(request);
        
        // Wait for response
        if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(12)) == rclcpp::FutureReturnCode::SUCCESS) {
            auto response = future.get();
            printResponse(response);
        } else {
            RCLCPP_ERROR(logger_, "❌ Failed to receive response from service");
        }
    }

private:
    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Client<x500_trajectory_planner::srv::X500PlanningService>::SharedPtr client_;
    rclcpp::Logger logger_;

    /**
     * @brief Print service response
     */
    void printResponse(const std::shared_ptr<x500_trajectory_planner::srv::X500PlanningService::Response> response)
    {
        RCLCPP_INFO(logger_, "📊 Planning Response:");
        RCLCPP_INFO(logger_, "=====================================");
        
        if (response->success) {
            RCLCPP_INFO(logger_, "✅ Status: SUCCESS");
            RCLCPP_INFO(logger_, "📝 Message: %s", response->message.c_str());
            RCLCPP_INFO(logger_, "⏱️  Planning duration: %.3f s", response->planning_duration);
            RCLCPP_INFO(logger_, "📏 Linear distance: %.3f m", response->linear_distance);
            RCLCPP_INFO(logger_, "📍 Number of waypoints: %d", response->num_waypoints);            
        } else {
            RCLCPP_ERROR(logger_, "❌ Status: FAILED");
            RCLCPP_ERROR(logger_, "📝 Message: %s", response->message.c_str());
            RCLCPP_ERROR(logger_, "⏱️  Planning duration: %.3f s", response->planning_duration);
        }
        
        RCLCPP_INFO(logger_, "=====================================\n");
    }
};

/**
 * @brief Main function for example client
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = rclcpp::Node::make_shared("x500_service_client_example");
    auto logger = node->get_logger();
    
    RCLCPP_INFO(logger, "🚁 X500 Planning Service Client Example");
    RCLCPP_INFO(logger, "=======================================");
    
    X500ServiceClient client(node);
    
    // Test planning at different positions
    RCLCPP_INFO(logger, "🧪 Test 1: Basic planning");
    client.requestPlanning(2.0, 2.0, 1.5);
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    RCLCPP_INFO(logger, "🧪 Test 2: Planning with rotation");
    client.requestPlanning(3.0, 1.0, 2.0, 0.0, 0.0, M_PI/4);
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    RCLCPP_INFO(logger, "🧪 Test 3: Long distance planning");
    client.requestPlanning(5.0, 4.0, 2.5, 0.0, 0.0, M_PI);
    
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    RCLCPP_INFO(logger, "🧪 Test 4: Planning with custom parameters");
    client.requestPlanning(1.5, 3.5, 1.0, 0.0, 0.0, -M_PI/2, 15.0, 5);
    
    RCLCPP_INFO(logger, "✅ All tests completed!");
    
    rclcpp::shutdown();
    return 0;
}