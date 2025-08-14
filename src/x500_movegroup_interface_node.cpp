/*  Author: Salvatore Del Peschio <saviodp7@gmail.com>
    Description: C++ interface adapted for X500 Drone */

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.h>
// TODO: Repub odom correttamente, per adesso prendiamo start state da /uav/odometry
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/robot_state/conversions.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("x500_movegroup_interface");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  // TODO: Namespace per il nodo configurabile
  auto node = rclcpp::Node::make_shared("x500_movegroup_interface", "/uav" ,node_options);
  const auto& LOGGER = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  RCLCPP_INFO(LOGGER, "Starting X500 Moveit2 Interface");
  try
  {
    // === ROBOT STATE AND PLANNING GROUP ===
    // We will start by instantiating a `RobotModelLoader` object, which will look up/ the robot description 
    // on the ROS parameter server and construct a
    // :moveit_codedir:`RobotModel<moveit_core/robot_model/include/moveit/robot_model/robot_model.h>`.
    RCLCPP_INFO(LOGGER, "Loading X500 robot model...");
    auto robot_model_loader_ptr = std::make_shared<robot_model_loader::RobotModelLoader>(node);
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader_ptr->getModel();
    // Forzo bound
    auto* vj = robot_model->getJointModel("virtual_joint");
    if (!vj) {
      RCLCPP_FATAL(LOGGER, "virtual_joint not found in RobotModel");
      return -1;
    }

    RCLCPP_INFO(LOGGER, "Model frame: %s", robot_model->getModelFrame().c_str());
    RCLCPP_INFO(LOGGER, "✅ X500 Robot Model loaded successfully");

    if (!robot_model) {
      RCLCPP_ERROR(LOGGER, "❌ Failed to load robot model. Check namespace or if robot_description is published.");
      return -1;
    }

    const std::vector<std::string>& group_names = robot_model->getJointModelGroupNames();
    RCLCPP_DEBUG(LOGGER, "Available Joint Model Groups:");
    for (const auto& group_name : group_names) {
      RCLCPP_DEBUG(LOGGER, "  - %s", group_name.c_str());
    }

    // Using the RobotModel, we can construct a RobotState that maintains the configuration 
    // of the robot. We will set all joints in the state to their default values. 
    // We can then get a JointModelGroup, which represents the robot model for a particular group.
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    robot_state->setToDefaultValues();
    const std::string PLANNING_GROUP = "x500_virtual_movegroup";
    const moveit::core::JointModelGroup* joint_model_group = robot_model->getJointModelGroup(PLANNING_GROUP);

    if (!joint_model_group) {
      RCLCPP_ERROR(LOGGER, "❌ Joint model group '%s' not found. Available groups:", PLANNING_GROUP.c_str());
      for (const auto& group_name : group_names) {
        RCLCPP_ERROR(LOGGER, "  - %s", group_name.c_str());
      }
      return -1;
    }

    RCLCPP_INFO(LOGGER, "✅ Using Joint Model Group: %s", PLANNING_GROUP.c_str());
    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    
    RCLCPP_DEBUG(LOGGER, "Joints in group '%s':", PLANNING_GROUP.c_str());
    for (const auto& joint_name : joint_names) {
      RCLCPP_DEBUG(LOGGER, "  - %s", joint_name.c_str());
    }

    // Retrieve the current set of joint values.
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      RCLCPP_DEBUG(LOGGER, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }



    // === PLANNER ===
    // Construct a loader to load a planner, by name.
    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    // TODO: Rendere il plugin caricabile
    std::string planner_plugin_name = "ompl_interface/OMPLPlanner";

    // We will get the name of planning plugin we want to load from the ROS parameter server, and then load the planner
    // making sure to catch all exceptions.
    if (!node->get_parameter("planning_plugin", planner_plugin_name))
      RCLCPP_WARN(LOGGER, "⚠️  Could not find planner plugin name, OMPL loaded as default");
    try
    {
      planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL(LOGGER, "❌ Exception while creating planning plugin loader %s", ex.what());
    }
    try
    { // NOTE: Unmanaged significa che il class loader non tiene traccia dell'oggetto dopo averlo creato
      // è tua responsabilità gestirne la memoria (lo smart pointer lo fa automaticamente)!
      planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
      if (!planner_instance->initialize(robot_model, node, node->get_namespace()))
        RCLCPP_FATAL(LOGGER, "❌ Could not initialize planner instance");
      RCLCPP_INFO(LOGGER, "✅ Using planning interface '%s'", planner_instance->getDescription().c_str());
    }
    catch (pluginlib::PluginlibException& ex)
    {
      const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
      std::stringstream ss;
      for (const auto& cls : classes)
        ss << cls << " ";
      RCLCPP_ERROR(LOGGER, "❌ Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                  ex.what(), ss.str().c_str());
    }

    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, robot_model_loader_ptr, node->get_namespace());
    psm->startSceneMonitor();
    // psm->startWorldGeometryMonitor(); // TODO: controllare che senza questo continui a pianificare evitando le collisioni

    // We will now create a motion plan request for the drone specifying the start pose and the goal pose.
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    moveit_msgs::msg::RobotState start_msg;
    moveit::core::RobotState start_pose(robot_model);
    start_pose.setToDefaultValues();
    const auto* jmg = robot_model->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> start_joints(jmg->getVariableCount(), 0.0);
    const auto& vars = jmg->getVariableNames();
    // Se il tuo virtual_joint è floating con RPY:
    for (size_t i = 0; i < vars.size(); ++i) {
      if (vars[i] == "virtual_joint/trans_x")     start_joints[i] = 0.0;
      if (vars[i] == "virtual_joint/trans_y")     start_joints[i] = 0.0;
      if (vars[i] == "virtual_joint/trans_z")     start_joints[i] = 1.0;
      if (vars[i] == "virtual_joint/rot_x")     start_joints[i] = 0.0;
      if (vars[i] == "virtual_joint/rot_y")     start_joints[i] = 0.0;
      if (vars[i] == "virtual_joint/rot_z")     start_joints[i] = 0.0;
      if (vars[i] == "virtual_joint/rot_w")     start_joints[i] = 1.0;
    //   // Oppure, se il virtual_joint è in quaternion:
    //   if (vars[i] == "qx") start_joints[i] = 0.0;
    //   if (vars[i] == "qy") start_joints[i] = 0.0;
    //   if (vars[i] == "qz") start_joints[i] = 0.0;
    //   if (vars[i] == "qw") start_joints[i] = 1.0;
    }
    start_pose.setJointGroupPositions(jmg, start_joints);
    moveit::core::robotStateToRobotStateMsg(start_pose, start_msg);
    req.start_state = start_msg;

    // Goal in joint-space (coerente con il tuo target)
    moveit::core::RobotState goal_state(robot_model);
    goal_state.setToDefaultValues();
    std::vector<double> goal_joints(jmg->getVariableCount(), 0.0);
    for (size_t i = 0; i < vars.size(); ++i) {
      if (vars[i] == "virtual_joint/trans_x")     goal_joints[i] = 4.0;
      if (vars[i] == "virtual_joint/trans_y")     goal_joints[i] = 3.0;
      if (vars[i] == "virtual_joint/trans_z")     goal_joints[i] = 3.5;
      if (vars[i] == "virtual_joint/rot_x")     goal_joints[i] = 0.0;
      if (vars[i] == "virtual_joint/rot_y")     goal_joints[i] = 0.0;
      if (vars[i] == "virtual_joint/rot_z")     goal_joints[i] = 0.0;
      if (vars[i] == "virtual_joint/rot_w")     goal_joints[i] = 1.0;
      // Se quaternion:
      // if (vars[i] == "qx") goal_joints[i] = q.x();
      // if (vars[i] == "qy") goal_joints[i] = q.y();
      // if (vars[i] == "qz") goal_joints[i] = q.z();
      // if (vars[i] == "qw") goal_joints[i] = q.w();
    }
    goal_state.setJointGroupPositions(jmg, goal_joints);

    // Aggiungi questo debug nel codice
    RCLCPP_INFO(LOGGER, "Joint model group variable count: %lu", jmg->getVariableCount());
    for (size_t i = 0; i < vars.size(); ++i) {
        RCLCPP_INFO(LOGGER, "Variable %lu: %s", i, vars[i].c_str());
    }

    // Verifica bounds
    const auto& joint_bounds = jmg->getActiveJointModelsBounds();
    for (size_t i = 0; i < joint_bounds.size(); ++i) {
        const auto& bounds = *joint_bounds[i];
        for (size_t j = 0; j < bounds.size(); ++j) {
            RCLCPP_INFO(LOGGER, "Joint bounds [%lu][%lu]: [%f, %f]", 
                      i, j, bounds[j].min_position_, bounds[j].max_position_);
        }
    }

        // Test validità degli stati
    if (!robot_state->satisfiesBounds(jmg)) {
        RCLCPP_ERROR(LOGGER, "Start state does not satisfy bounds!");
    }
    if (!goal_state.satisfiesBounds(jmg)) {
        RCLCPP_ERROR(LOGGER, "Goal state does not satisfy bounds!");
    }

    // // We will create the request as a constraint using a helper function available from the
    // // :moveit_codedir:`kinematic_constraints<moveit_core/kinematic_constraints/include/moveit/kinematic_constraints/kinematic_constraint.h>`
    // const double tolerance = 0.05;
    // moveit_msgs::msg::Constraints pose_goal =
    //     kinematic_constraints::constructGoalConstraints(goal_state, jmg, tolerance);

    // // TODO: Passare tutti i dati da yaml
    // // TODO: IMPORTANTE!!! Adatta punti al campo di gara
    // req.group_name = PLANNING_GROUP;
    // req.workspace_parameters.header.frame_id = "map";
    // req.workspace_parameters.min_corner.x = -20.0;
    // req.workspace_parameters.min_corner.y = -20.0; 
    // req.workspace_parameters.min_corner.z = -5.0;
    // req.workspace_parameters.max_corner.x = 20.0;
    // req.workspace_parameters.max_corner.y = 20.0;
    // req.workspace_parameters.max_corner.z = 15.0;
    // req.allowed_planning_time = 15.0;
    // req.num_planning_attempts = 10;
    // req.goal_constraints.push_back(pose_goal);

    // planning_scene_monitor::LockedPlanningSceneRO lpscene(psm);
    // if (!lpscene){
    //   RCLCPP_ERROR(LOGGER, "❌ Could not instantiate a LockedPlanningSceneRO");
    //   return -1;
    // }

    // // We now construct a planning context that encapsulates the scene, the request and the response
    // planning_interface::PlanningContextPtr context = 
    //     planner_instance->getPlanningContext(lpscene, req, res.error_code_);
    // if (!context) {
    //   RCLCPP_ERROR(LOGGER, "❌ Could not create planning context");
    //   return -1;
    // }
    // context->solve(res);
    // if (res.error_code_.val != res.error_code_.SUCCESS)
    // {
    //   RCLCPP_ERROR(LOGGER, "❌ Could not compute plan successfully");
    //   return 0;
    // }

    RCLCPP_INFO(LOGGER, "🐣 Ciao, io qui ci arrivo");






    // // Visualize the result
    // // ^^^^^^^^^^^^^^^^^^^^
    // std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
    //     node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
    //                                                                                             1);
    // moveit_msgs::msg::DisplayTrajectory display_trajectory;

    // /* Visualize the trajectory */
    // moveit_msgs::msg::MotionPlanResponse response;
    // res.getMessage(response);

    // display_trajectory.trajectory_start = response.trajectory_start;
    // display_trajectory.trajectory.push_back(response.trajectory);
    // display_publisher->publish(display_trajectory);

    // /* Set the state in the planning scene to the final state of the last plan */
    // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    // planning_scene->setCurrentState(*robot_state.get());

    // // Joint Space Goals
    // // ^^^^^^^^^^^^^^^^^
    // // Now, setup a joint space goal
    // moveit::core::RobotState goal_state(robot_model);
    // std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
    // goal_state.setJointGroupPositions(joint_model_group, joint_values);
    // moveit_msgs::msg::Constraints joint_goal =
    //     kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
    // req.goal_constraints.clear();
    // req.goal_constraints.push_back(joint_goal);

    // // Call the planner and visualize the trajectory
    // /* Re-construct the planning context */
    // context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    // /* Call the Planner */
    // context->solve(res);
    // /* Check that the planning was successful */
    // if (res.error_code_.val != res.error_code_.SUCCESS)
    // {
    //   RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
    //   return 0;
    // }
    // /* Visualize the trajectory */
    // res.getMessage(response);
    // display_trajectory.trajectory.push_back(response.trajectory);

    // /* Now you should see two planned trajectories in series*/
    // display_publisher->publish(display_trajectory);

    // /* We will add more goals. But first, set the state in the planning
    //   scene to the final state of the last plan */
    // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    // planning_scene->setCurrentState(*robot_state.get());

    // /* Now, we go back to the first goal to prepare for orientation constrained planning */
    // req.goal_constraints.clear();
    // req.goal_constraints.push_back(pose_goal);
    // context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    // context->solve(res);
    // res.getMessage(response);

    // display_trajectory.trajectory.push_back(response.trajectory);
    // display_publisher->publish(display_trajectory);

    // /* Set the state in the planning scene to the final state of the last plan */
    // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    // planning_scene->setCurrentState(*robot_state.get());

    // // Adding Path Constraints
    // // ^^^^^^^^^^^^^^^^^^^^^^^
    // // Let's add a new pose goal again. This time we will also add a path constraint to the motion.
    // /* Let's create a new pose goal */

    // goal_pose.pose.position.x = 0.32;
    // goal_pose.pose.position.y = -0.25;
    // goal_pose.pose.position.z = 0.65;
    // goal_pose.pose.orientation.w = 1.0;
    // moveit_msgs::msg::Constraints pose_goal_2 =
    //     kinematic_constraints::constructGoalConstraints("panda_link8", goal_pose, tolerance_pose, tolerance_angle);

    // /* Now, let's try to move to this new pose goal*/
    // req.goal_constraints.clear();
    // req.goal_constraints.push_back(pose_goal_2);

    // /* But, let's impose a path constraint on the motion.
    //   Here, we are asking for the end-effector to stay level*/
    // geometry_msgs::msg::QuaternionStamped quaternion;
    // quaternion.header.frame_id = "panda_link0";
    // req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);

    // // Imposing path constraints requires the planner to reason in the space of possible positions of the end-effector
    // // (the workspace of the robot)
    // // because of this, we need to specify a bound for the allowed planning volume as well;
    // // Note: a default bound is automatically filled by the WorkspaceBounds request adapter (part of the OMPL pipeline,
    // // but that is not being used in this example).
    // // We use a bound that definitely includes the reachable space for the arm. This is fine because sampling is not done
    // // in this volume
    // // when planning for the arm; the bounds are only used to determine if the sampled configurations are valid.
    // req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
    //     req.workspace_parameters.min_corner.z = -2.0;
    // req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
    //     req.workspace_parameters.max_corner.z = 2.0;

    // // Call the planner and visualize all the plans created so far.
    // context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    // context->solve(res);
    // res.getMessage(response);
    // display_trajectory.trajectory.push_back(response.trajectory);
    // display_publisher->publish(display_trajectory);

    // /* Set the state in the planning scene to the final state of the last plan */
    // robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    // planning_scene->setCurrentState(*robot_state.get());

    // planner_instance.reset();

  }
  catch(const std::exception& e){
    RCLCPP_ERROR(LOGGER, "❌ Failed to load x500 movegroup interface! Exception : %s", e.what());
    return -1;
  }

  rclcpp::shutdown();
  return 0;
}