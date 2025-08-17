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
        
        // // ================================================
        // // TEST 1: BASIC STATUS AND INITIALIZATION
        // // ================================================
        {
            RCLCPP_INFO(logger, "🔍 TEST 1: Basic Status and Initialization");
            RCLCPP_INFO(logger, "==========================================");
            
            mg_interface.printStatus();
        }
        
        // // ================================================
        // // TEST 2: WORKSPACE CONSTRAINTS TESTING
        // // ================================================
        // {
        //     RCLCPP_INFO(logger, "🔧 TEST 2: Workspace Constraints Testing");
        //     RCLCPP_INFO(logger, "========================================");
            
        //     // Test valid workspace
        //     mg_interface.setWorkspaceConstraints(-2.0, -2.0, 0.0, 5.0, 5.0, 3.0);
            
        //     // Test invalid workspace (should log error)
        //     RCLCPP_INFO(logger, "Testing invalid workspace (min >= max)...");
        //     mg_interface.setWorkspaceConstraints(5.0, -2.0, 0.0, -2.0, 5.0, 3.0);
            
        //     // Restore valid workspace
        //     mg_interface.setWorkspaceConstraints(-1.0, -1.0, 0.0, 10.0, 8.0, 3.5);
        // }
        
        // // ================================================
        // // TEST 3: PLANNING PARAMETERS TESTING
        // // ================================================
        // {
        //     RCLCPP_INFO(logger, "⏱️ TEST 3: Planning Parameters Testing");
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
        
        // // ================================================
        // // TEST 4: TARGET POSE SETTING (RPY VERSION)
        // // ================================================
        // {
        //     RCLCPP_INFO(logger, "🎯 TEST 4: Target Pose Setting (RPY Version)");
        //     RCLCPP_INFO(logger, "============================================");
            
        //     // Test basic position
        //     mg_interface.setTargetPose(1.0, 2.0, 1.5);
            
        //     // Test with orientation
        //     mg_interface.setTargetPose(2.0, 1.0, 2.0, 0.0, 0.0, M_PI/4);  // 45° yaw
            
        //     // Test with full RPY
        //     mg_interface.setTargetPose(1.5, 1.5, 1.0, M_PI/6, M_PI/12, M_PI/3);  // 30°, 15°, 60°
            
        //     // Test edge of workspace (should be valid)
        //     mg_interface.setTargetPose(9.5, 7.5, 3.0);
            
        // }
        
        // // ================================================
        // // TEST 5: TARGET POSE SETTING (GEOMETRY_MSGS VERSION)
        // // ================================================
        // {
        //     RCLCPP_INFO(logger, "📐 TEST 5: Target Pose Setting (geometry_msgs Version)");
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
        //     RCLCPP_INFO(logger, "🚀 TEST 6: Planning Functionality");
        //     RCLCPP_INFO(logger, "=================================");
            
            // // Set a reasonable target
            // mg_interface.setTargetPose(8.5, 6.25, 2.0, 0.0, 0.0, M_PI/2);
            
            // // Attempt planning
            // RCLCPP_INFO(logger, "Attempting to plan to target...");
            // auto plan = mg_interface.planToTarget();
            
        //     // Check planning result
        //     if (mg_interface.wasLastPlanningSuccessful()) {
        //         RCLCPP_INFO(logger, "✅ Planning succeeded!");
        //         RCLCPP_INFO(logger, "Plan details:");
        //         RCLCPP_INFO(logger, "  - Planning time: %.3f s", plan.planning_time_);
        //         RCLCPP_INFO(logger, "  - Trajectory points: %zu", 
        //                 plan.trajectory_.multi_dof_joint_trajectory.points.size());
                
        //         if (!plan.trajectory_.multi_dof_joint_trajectory.points.empty()) {
        //             RCLCPP_INFO(logger, "\n📍 Trajectory waypoints:");
        //             RCLCPP_INFO(logger, "=====================================");
                    
        //             // Stampa ogni punto della traiettoria
        //             for (size_t i = 0; i < plan.trajectory_.multi_dof_joint_trajectory.points.size(); ++i) {
        //                 const auto& point = plan.trajectory_.multi_dof_joint_trajectory.points[i];
        //                 RCLCPP_INFO(logger, "Point %zu:", i);
                        
        //                 // Stampa le trasformazioni (dovrebbe essere una sola per il virtual joint)
        //                 if (!point.transforms.empty()) {
        //                     const auto& transform = point.transforms[0];  // Prima (e unica) trasformazione
                            
        //                     // Posizione
        //                     double x = transform.translation.x;
        //                     double y = transform.translation.y;
        //                     double z = transform.translation.z;
                            
        //                     // Orientamento (quaternion -> RPY)
        //                     tf2::Quaternion q(
        //                         transform.rotation.x,
        //                         transform.rotation.y,
        //                         transform.rotation.z,
        //                         transform.rotation.w
        //                     );
        //                     double roll, pitch, yaw;
        //                     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                            
        //                     RCLCPP_INFO(logger, "  Position: [%.3f, %.3f, %.3f] m", x, y, z);
        //                     RCLCPP_INFO(logger, "  Orientation: [%.3f, %.3f, %.3f] rad (RPY)", roll, pitch, yaw);
        //                     RCLCPP_INFO(logger, "  Orientation: [%.3f°, %.3f°, %.3f°] deg (RPY)", 
        //                             roll * 180.0/M_PI, pitch * 180.0/M_PI, yaw * 180.0/M_PI);
        //                 } else {
        //                     RCLCPP_WARN(logger, "  ⚠️ No transform data for point %zu", i);
        //                 }
        //                 if (i < plan.trajectory_.multi_dof_joint_trajectory.points.size() - 1) {
        //                     RCLCPP_INFO(logger, "  ---");  // Separatore tra i punti
        //                 }
        //             }
                    
        //             RCLCPP_INFO(logger, "=====================================");
                    
        //             RCLCPP_INFO(logger, "📊 Trajectory Statistics:");
        //             if (plan.trajectory_.multi_dof_joint_trajectory.points.size() >= 2) {
        //                 const auto& start_point = plan.trajectory_.multi_dof_joint_trajectory.points.front();
        //                 const auto& end_point = plan.trajectory_.multi_dof_joint_trajectory.points.back();
                        
        //                 // Numero di punti
        //                 size_t num_points = plan.trajectory_.multi_dof_joint_trajectory.points.size();
        //                 RCLCPP_INFO(logger, "  - Number of waypoints: %zu", num_points);
                        
        //                 if (!start_point.transforms.empty() && !end_point.transforms.empty()) {
        //                     // Calcola distanza euclidea (linea retta start->end)
        //                     double dx_euclidean = end_point.transforms[0].translation.x - start_point.transforms[0].translation.x;
        //                     double dy_euclidean = end_point.transforms[0].translation.y - start_point.transforms[0].translation.y;
        //                     double dz_euclidean = end_point.transforms[0].translation.z - start_point.transforms[0].translation.z;
        //                     double euclidean_distance = sqrt(dx_euclidean*dx_euclidean + dy_euclidean*dy_euclidean + dz_euclidean*dz_euclidean);
                            
        //                     // Calcola distanza lineare (somma delle distanze punto-punto)
        //                     double linear_distance = 0.0;
        //                     for (size_t i = 1; i < plan.trajectory_.multi_dof_joint_trajectory.points.size(); ++i) {
        //                         const auto& prev_point = plan.trajectory_.multi_dof_joint_trajectory.points[i-1];
        //                         const auto& curr_point = plan.trajectory_.multi_dof_joint_trajectory.points[i];
                                
        //                         if (!prev_point.transforms.empty() && !curr_point.transforms.empty()) {
        //                             double dx = curr_point.transforms[0].translation.x - prev_point.transforms[0].translation.x;
        //                             double dy = curr_point.transforms[0].translation.y - prev_point.transforms[0].translation.y;
        //                             double dz = curr_point.transforms[0].translation.z - prev_point.transforms[0].translation.z;
        //                             double segment_distance = sqrt(dx*dx + dy*dy + dz*dz);
        //                             linear_distance += segment_distance;
        //                         }
        //                     }
                            
        //                     RCLCPP_INFO(logger, "  - Euclidean distance (straight line): %.3f m", euclidean_distance);
        //                     RCLCPP_INFO(logger, "  - Linear distance (path length): %.3f m", linear_distance);
                            
        //                     // Calcola l'efficienza del percorso
        //                     if (euclidean_distance > 0.001) {  // Evita divisione per zero
        //                         double path_efficiency = euclidean_distance / linear_distance;
        //                         RCLCPP_INFO(logger, "  - Path efficiency: %.1f%% (100%% = perfectly straight)", path_efficiency * 100.0);
        //                     }
                            
        //                     // Distanza media tra waypoints consecutivi
        //                     if (num_points > 1) {
        //                         double avg_segment_length = linear_distance / (num_points - 1);
        //                         RCLCPP_INFO(logger, "  - Average segment length: %.3f m", avg_segment_length);
        //                     }
        //                 }
        //             }
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
        //     RCLCPP_INFO(logger, "💾 TEST 7: Remembered Positions Functionality");
        //     RCLCPP_INFO(logger, "============================================");
            
        //     // Remember some positions using the current target
        //     mg_interface.setTargetPose(1.0, 1.0, 1.0);
        //     mg_interface.rememberCurrentTarget("home");  
            
        //     mg_interface.setTargetPose(3.0, 2.0, 2.0, 0.0, 0.0, M_PI/2);
        //     mg_interface.rememberCurrentTarget("waypoint_1");  
            
        //     mg_interface.setTargetPose(4.0, 3.0, 1.5, 0.0, 0.0, M_PI);
        //     mg_interface.rememberCurrentTarget("waypoint_2"); 
            
        //     // Test empty name (should fail)
        //     RCLCPP_INFO(logger, "Testing empty position name...");
        //     mg_interface.rememberCurrentTarget("");  
            
        //     // Test alternative method: remember positions directly
        //     RCLCPP_INFO(logger, "Testing direct position remembering...");
        //     mg_interface.rememberPosition(5.0, 4.0, 2.5, 0.0, 0.0, M_PI/4, "direct_waypoint");
            
        //     // Test geometry_msgs::Pose version
        //     geometry_msgs::msg::Pose pose;
        //     pose.position.x = 6.0;
        //     pose.position.y = 5.0;
        //     pose.position.z = 3.0;
        //     pose.orientation.x = 0.0;
        //     pose.orientation.y = 0.0;
        //     pose.orientation.z = 0.707;  // 90° rotation around Z
        //     pose.orientation.w = 0.707;
        //     mg_interface.rememberPosition(pose, "pose_waypoint");
            
        //     // List remembered positions
        //     auto names = mg_interface.getRememberedPositionNames();
        //     RCLCPP_INFO(logger, "Remembered positions (%zu):", names.size());
        //     for (const auto& name : names) {
        //         RCLCPP_INFO(logger, "  - %s", name.c_str());
        //     }
            
        //     // Test setting target to remembered positions
        //     RCLCPP_INFO(logger, "Testing recalled positions:");
        //     for (const auto& name : names) {
        //         if (mg_interface.setTargetToRememberedPosition(name)) {
        //             RCLCPP_INFO(logger, "✅ Successfully set target to '%s'", name.c_str());
        //         } else {
        //             RCLCPP_WARN(logger, "❌ Failed to set target to '%s'", name.c_str());
        //         }
        //     }
            
        //     // Test non-existent position
        //     RCLCPP_INFO(logger, "Testing non-existent position...");
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
            
        //     // Test error case: remember target without setting one first
        //     RCLCPP_INFO(logger, "Testing remember target without setting target first...");
        //     X500MoveGroupInterface fresh_interface(node, "x500_group");
        //     fresh_interface.rememberCurrentTarget("should_fail");  // Should show error
        // }
        
        // ================================================
        // TODO: Implementare planning per waypoints
        // TEST 8: COMPLEX PLANNING SEQUENCE
        // ================================================
        // {
        //     RCLCPP_INFO(logger, "🔄 TEST 8: Complex Planning Sequence");
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
        //     RCLCPP_INFO(logger, "⚠️ TEST 9: Error Conditions and Edge Cases");
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
        //     RCLCPP_INFO(logger, "📊 TEST 10: Final Status and Cleanup");
        //     RCLCPP_INFO(logger, "===================================");
            
        //     // Set a reasonable target
        //     mg_interface.setTargetPose(8.5, 6.25, 2.0, 0.0, 0.0, M_PI/2);
            
        //     // Attempt planning
        //     RCLCPP_INFO(logger, "Attempting to plan to target...");
        //     auto plan = mg_interface.planToTarget();

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