#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <signal.h>

#include "x500_planning_service_server.hpp"

// Global pointer for signal handler
std::shared_ptr<X500PlanningServiceServer> service_server_ptr = nullptr;

/**
 * @brief Signal handler for graceful shutdown
 */
void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("x500_planning_service"), 
                "🛑 Received signal %d, shutting down gracefully...", signum);
    
    if (service_server_ptr) {
        service_server_ptr->stop();
    }
    
    rclcpp::shutdown();
    exit(signum);
}

/**
 * @brief Main node for X500 Planning Service
 */
int main(int argc, char** argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    auto node = rclcpp::Node::make_shared("x500_planning_service_node");
    auto logger = node->get_logger();
    
    RCLCPP_INFO(logger, "🚁 Starting X500 Planning Service Node...");
    
    try {
        // Declare parameters
        node->declare_parameter("service_name", "x500_planner");
        node->declare_parameter("planning_group", "x500_group");
        
        // Get parameters
        std::string service_name = node->get_parameter("service_name").as_string();
        std::string planning_group = node->get_parameter("planning_group").as_string();
        
        RCLCPP_INFO(logger, "📋 Configuration:");
        RCLCPP_INFO(logger, "   Service name: %s", service_name.c_str());
        RCLCPP_INFO(logger, "   Planning group: %s", planning_group.c_str());
        
        // Create service server
        service_server_ptr = std::make_shared<X500PlanningServiceServer>(
            node, service_name, planning_group
        );
        
        // Start service
        service_server_ptr->start();
        
        RCLCPP_INFO(logger, "✅ X500 Planning Service Node ready!");
        RCLCPP_INFO(logger, "📞 Waiting for planning requests on service: %s", service_name.c_str());
        RCLCPP_INFO(logger, "💡 Usage example:");
        RCLCPP_INFO(logger, "   ros2 service call /%s x500_trajectory_planner/srv/X500PlanningService "
                            "\"{target_pose: {position: {x: 7.0, y: 3.0, z: 1.5}, orientation: {w: 1.0}}}\"", 
                    service_name.c_str());
        
        // Spin node
        rclcpp::spin(node);
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(logger, "❌ Failed to start X500 Planning Service: %s", e.what());
        return 1;
    }
    
    RCLCPP_INFO(logger, "🏁 X500 Planning Service Node terminated");
    return 0;
}