#include "x500_movegroup_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

/**
 * @brief Test node for X500MoveGroupInterface
 * @author Salvatore Del Peschio <salvatoredelpeschio@gmail.com>
 * 
 * Uncomment/comment test blocks to perform specific tests
 */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Create node with namespace
    auto node = rclcpp::Node::make_shared("x500_movegroup_test_node");
    auto logger = node->get_logger();
    
    RCLCPP_INFO(logger, "🧪 Starting X500MoveGroupInterface Test Suite");
    RCLCPP_INFO(logger, "========================================");
    
    try {
        // Initialize the interface
        RCLCPP_INFO(logger, "📦 Creating X500MoveGroupInterface...");
        X500MoveGroupInterface mg_interface(node, "x500_group");
        
        // Small delay to ensure everything is ready
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        // ================================================
        // TEST 1: BASIC STATUS AND INITIALIZATION
        // ================================================
        {
            RCLCPP_INFO(logger, "🔍 TEST 1: Basic Status and Initialization");
            RCLCPP_INFO(logger, "==========================================");
            
            mg_interface.printStatus();
        }
        
        // ================================================
        // TEST 2: WORKSPACE CONSTRAINTS TESTING
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "\n🔧 TEST 2: Workspace Constraints Testing");
        //     RCLCPP_INFO(logger, "========================================");
            
        //     // Test valid workspace
        //     mg_interface.setWorkspaceConstraints(-2.0, -2.0, 0.0, 5.0, 5.0, 3.0);
            
        //     // Test invalid workspace (should log error)
        //     RCLCPP_INFO(logger, "Testing invalid workspace (min >= max)...");
        //     mg_interface.setWorkspaceConstraints(5.0, -2.0, 0.0, -2.0, 5.0, 3.0);
            
        //     // Restore valid workspace
        //     mg_interface.setWorkspaceConstraints(-1.0, -1.0, 0.0, 10.0, 8.0, 3.5);
        // }
        
        // ================================================
        // TEST 3: PLANNING PARAMETERS TESTING
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "\n⏱️ TEST 3: Planning Parameters Testing");
        //     RCLCPP_INFO(logger, "====================================");
            
        //     // Test valid parameters
        //     mg_interface.setPlanningParameters(5.0, 2);
            
        //     // Test invalid parameters (should log errors)
        //     RCLCPP_INFO(logger, "Testing invalid planning time...");
        //     mg_interface.setPlanningParameters(-1.0, 2);
            
        //     RCLCPP_INFO(logger, "Testing invalid attempts...");
        //     mg_interface.setPlanningParameters(5.0, 0);
            
        //     // Restore good parameters
        //     mg_interface.setPlanningParameters(8.0, 3);
        // }
        
        // ================================================
        // TEST 4: TARGET POSE SETTING (RPY VERSION)
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "\n🎯 TEST 4: Target Pose Setting (RPY Version)");
        //     RCLCPP_INFO(logger, "============================================");
            
        //     // Test basic position
        //     mg_interface.setTargetPose(1.0, 2.0, 1.5);
            
        //     // Test with orientation
        //     mg_interface.setTargetPose(2.0, 1.0, 2.0, 0.0, 0.0, M_PI/4);  // 45° yaw
            
        //     // Test with full RPY
        //     mg_interface.setTargetPose(1.5, 1.5, 1.0, M_PI/6, M_PI/12, M_PI/3);  // 30°, 15°, 60°
            
        //     // Test edge of workspace (should be valid)
        //     mg_interface.setTargetPose(9.5, 7.5, 3.0);
            
        //     // Test outside workspace (should be rejected)
        //     RCLCPP_INFO(logger, "Testing position outside workspace...");
        //     mg_interface.setTargetPose(15.0, 15.0, 5.0);  // Should fail validation
        // }
        
        // ================================================
        // TEST 5: TARGET POSE SETTING (GEOMETRY_MSGS VERSION)
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "\n📐 TEST 5: Target Pose Setting (geometry_msgs Version)");
        //     RCLCPP_INFO(logger, "====================================================");
            
        //     geometry_msgs::msg::Pose pose;
        //     pose.position.x = 3.0;
        //     pose.position.y = 2.5;
        //     pose.position.z = 2.0;
            
        //     // Identity quaternion
        //     pose.orientation.x = 0.0;
        //     pose.orientation.y = 0.0;
        //     pose.orientation.z = 0.0;
        //     pose.orientation.w = 1.0;
            
        //     mg_interface.setTargetPose(pose);
            
        //     // Test with 90° rotation around Z
        //     pose.position.x = 2.0;
        //     pose.position.y = 3.0;
        //     pose.position.z = 1.8;
        //     pose.orientation.x = 0.0;
        //     pose.orientation.y = 0.0;
        //     pose.orientation.z = 0.707;
        //     pose.orientation.w = 0.707;
            
        //     mg_interface.setTargetPose(pose);
        // }
        
        // ================================================
        // TEST 6: PLANNING FUNCTIONALITY
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "\n🚀 TEST 6: Planning Functionality");
        //     RCLCPP_INFO(logger, "=================================");
            
        //     // Set a reasonable target
        //     mg_interface.setTargetPose(2.0, 2.0, 1.5, 0.0, 0.0, M_PI/2);
            
        //     // Attempt planning
        //     RCLCPP_INFO(logger, "Attempting to plan to target...");
        //     auto plan = mg_interface.planToTarget();
            
        //     // Check planning result
        //     if (mg_interface.wasLastPlanningSuccessful()) {
        //         RCLCPP_INFO(logger, "✅ Planning succeeded!");
        //         RCLCPP_INFO(logger, "Plan details:");
        //         RCLCPP_INFO(logger, "  - Planning time: %.3f s", plan.planning_time_);
        //         RCLCPP_INFO(logger, "  - Trajectory points: %zu", 
        //                    plan.trajectory_.joint_trajectory.points.size());
                
        //         if (!plan.trajectory_.joint_trajectory.points.empty()) {
        //             auto duration = plan.trajectory_.joint_trajectory.points.back().time_from_start;
        //             RCLCPP_INFO(logger, "  - Trajectory duration: %.2f s", 
        //                        duration.sec + duration.nanosec * 1e-9);
        //         }
        //     } else {
        //         RCLCPP_WARN(logger, "❌ Planning failed!");
        //     }
            
        //     // Test planning to an unreachable target (optional)
        //     RCLCPP_INFO(logger, "\nTesting planning to potentially unreachable target...");
        //     mg_interface.setTargetPose(0.1, 0.1, 3.4);  // Very close to workspace edge
        //     auto plan2 = mg_interface.planToTarget();
            
        //     if (plan2.planning_time_ > 0) {
        //         RCLCPP_INFO(logger, "✅ Planning to edge target succeeded!");
        //     } else {
        //         RCLCPP_INFO(logger, "❌ Planning to edge target failed (expected)");
        //     }
        // }
        
        // ================================================
        // TEST 7: REMEMBERED POSITIONS FUNCTIONALITY
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "\n💾 TEST 7: Remembered Positions Functionality");
        //     RCLCPP_INFO(logger, "============================================");
            
        //     // Remember some positions
        //     mg_interface.setTargetPose(1.0, 1.0, 1.0);
        //     mg_interface.rememberCurrentPosition("home");
            
        //     mg_interface.setTargetPose(3.0, 2.0, 2.0, 0.0, 0.0, M_PI/2);
        //     mg_interface.rememberCurrentPosition("waypoint_1");
            
        //     mg_interface.setTargetPose(4.0, 3.0, 1.5, 0.0, 0.0, M_PI);
        //     mg_interface.rememberCurrentPosition("waypoint_2");
            
        //     // Test empty name (should fail)
        //     RCLCPP_INFO(logger, "Testing empty position name...");
        //     mg_interface.rememberCurrentPosition("");
            
        //     // List remembered positions
        //     auto names = mg_interface.getRememberedPositionNames();
        //     RCLCPP_INFO(logger, "Remembered positions (%zu):", names.size());
        //     for (const auto& name : names) {
        //         RCLCPP_INFO(logger, "  - %s", name.c_str());
        //     }
            
        //     // Test setting target to remembered positions
        //     RCLCPP_INFO(logger, "\nTesting recalled positions:");
        //     for (const auto& name : names) {
        //         if (mg_interface.setTargetToRememberedPosition(name)) {
        //             RCLCPP_INFO(logger, "✅ Successfully set target to '%s'", name.c_str());
        //         } else {
        //             RCLCPP_WARN(logger, "❌ Failed to set target to '%s'", name.c_str());
        //         }
        //     }
            
        //     // Test non-existent position
        //     RCLCPP_INFO(logger, "\nTesting non-existent position...");
        //     mg_interface.setTargetToRememberedPosition("non_existent");
            
        //     // Test clearing specific position
        //     if (mg_interface.clearRememberedPosition("waypoint_1")) {
        //         RCLCPP_INFO(logger, "✅ Successfully cleared 'waypoint_1'");
        //     }
            
        //     // Test clearing non-existent position
        //     mg_interface.clearRememberedPosition("non_existent");
            
        //     // Show updated list
        //     names = mg_interface.getRememberedPositionNames();
        //     RCLCPP_INFO(logger, "Remembered positions after clearing (%zu):", names.size());
        //     for (const auto& name : names) {
        //         RCLCPP_INFO(logger, "  - %s", name.c_str());
        //     }
        // }
        
        // ================================================
        // TEST 8: COMPLEX PLANNING SEQUENCE
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "\n🔄 TEST 8: Complex Planning Sequence");
        //     RCLCPP_INFO(logger, "===================================");
            
        //     // Plan a sequence of movements
        //     std::vector<std::array<double, 6>> waypoints = {
        //         {1.0, 1.0, 1.0, 0.0, 0.0, 0.0},           // Start
        //         {3.0, 1.0, 1.5, 0.0, 0.0, M_PI/4},        // Move right, turn 45°
        //         {3.0, 3.0, 2.0, 0.0, 0.0, M_PI/2},        // Move forward, turn 90°
        //         {1.0, 3.0, 2.0, 0.0, 0.0, 3*M_PI/4},      // Move left, turn 135°
        //         {1.0, 1.0, 1.0, 0.0, 0.0, M_PI}           // Return, turn 180°
        //     };
            
        //     int success_count = 0;
        //     int total_attempts = waypoints.size();
            
        //     for (size_t i = 0; i < waypoints.size(); ++i) {
        //         const auto& wp = waypoints[i];
        //         RCLCPP_INFO(logger, "\nWaypoint %zu: [%.1f,%.1f,%.1f] RPY=[%.2f,%.2f,%.2f]", 
        //                    i+1, wp[0], wp[1], wp[2], wp[3], wp[4], wp[5]);
                
        //         mg_interface.setTargetPose(wp[0], wp[1], wp[2], wp[3], wp[4], wp[5]);
        //         auto plan = mg_interface.planToTarget();
                
        //         if (plan.planning_time_ > 0) {
        //             success_count++;
        //             RCLCPP_INFO(logger, "✅ Waypoint %zu: Planning successful (%.0f ms)", 
        //                        i+1, plan.planning_time_ * 1000);
        //         } else {
        //             RCLCPP_WARN(logger, "❌ Waypoint %zu: Planning failed", i+1);
        //         }
        //     }
            
        //     RCLCPP_INFO(logger, "\nSequence Summary: %d/%d waypoints planned successfully", 
        //                success_count, total_attempts);
        // }
        
        // ================================================
        // TEST 9: ERROR CONDITIONS AND EDGE CASES
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "\n⚠️ TEST 9: Error Conditions and Edge Cases");
        //     RCLCPP_INFO(logger, "=========================================");
            
        //     // Test extreme workspace boundaries
        //     RCLCPP_INFO(logger, "Testing extreme workspace boundaries...");
        //     mg_interface.setTargetPose(-0.9, -0.9, 0.1);    // Near minimum
        //     mg_interface.setTargetPose(9.9, 7.9, 3.4);      // Near maximum
            
        //     // Test zero position
        //     RCLCPP_INFO(logger, "Testing zero position...");
        //     mg_interface.setTargetPose(0.0, 0.0, 0.5);
            
        //     // Test extreme orientations
        //     RCLCPP_INFO(logger, "Testing extreme orientations...");
        //     mg_interface.setTargetPose(2.0, 2.0, 1.5, M_PI, 0.0, 0.0);    // 180° roll
        //     mg_interface.setTargetPose(2.0, 2.0, 1.5, 0.0, M_PI/2, 0.0);  // 90° pitch
        //     mg_interface.setTargetPose(2.0, 2.0, 1.5, 0.0, 0.0, 2*M_PI);  // 360° yaw
            
        //     // Test rapid successive planning calls
        //     RCLCPP_INFO(logger, "Testing rapid successive planning calls...");
        //     for (int i = 0; i < 3; ++i) {
        //         mg_interface.setTargetPose(1.0 + i*0.5, 1.0 + i*0.3, 1.0 + i*0.2);
        //         auto plan = mg_interface.planToTarget();
        //         RCLCPP_INFO(logger, "Rapid plan %d: %s", i+1, 
        //                    plan.planning_time_ > 0 ? "SUCCESS" : "FAILED");
        //     }
        // }
        
        // ================================================
        // TEST 10: FINAL STATUS AND CLEANUP
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "\n📊 TEST 10: Final Status and Cleanup");
        //     RCLCPP_INFO(logger, "===================================");
            
        //     // Clear all remembered positions
        //     mg_interface.clearAllRememberedPositions();
            
        //     // Final status
        //     mg_interface.printStatus();
            
        //     // Test last plan access
        //     const auto& last_plan = mg_interface.getLastPlan();
        //     RCLCPP_INFO(logger, "Last plan planning time: %.3f s", last_plan.planning_time_);
        //     RCLCPP_INFO(logger, "Last planning successful: %s", 
        //                mg_interface.wasLastPlanningSuccessful() ? "YES" : "NO");
        // }
        
        RCLCPP_INFO(logger, "🎉 ALL TESTS COMPLETED!");
        RCLCPP_INFO(logger, "====================");
        RCLCPP_INFO(logger, "Check the logs above for detailed results.");
        RCLCPP_INFO(logger, "Comment/uncomment test blocks to focus on specific functionality.");
        
    } catch (const std::exception& e) {
        RCLCPP_FATAL(logger, "❌ Test suite failed with exception: %s", e.what());
        return -1;
    }
    
    // Keep node alive for a bit to see final logs
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    rclcpp::shutdown();
    return 0;
}