#include <memory>

#include <pluginlib/class_loader.hpp>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int motion_planning_api(const rclcpp::Node::SharedPtr node)
{
  // Get the logger in this scope
  auto const logger = rclcpp::get_logger("move_panda");

  robot_model_loader::RobotModelLoader robot_model_loader(node,
    "robot_description");
  const moveit::core::RobotModelPtr & kinematic_model = robot_model_loader.getModel();
  /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  const moveit::core::JointModelGroup * joint_model_group = robot_state->getJointModelGroup(
    "panda_arm");


  planning_scene::PlanningScenePtr planning_scene_ptr(
    new planning_scene::PlanningScene(kinematic_model));

  planning_scene_ptr->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");
  // Using ROS pluginlib library we construct a loader to load a planner
  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>>
  planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_names = "ompl_interface/OMPLPlanner";

  // Get the name of the planner from the parameter server.
  if (!node->get_parameter(
      "move_group/ompl.planning_plugin",
      planner_plugin_names))
  {
    RCLCPP_ERROR(logger, "Could not find planner plugin names");
  }
  try {
    planner_plugin_loader.reset(
      new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "moveit::planning_interface::PlannerManager"));
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(logger, "The plugin failed to load. Error: %s", ex.what());
  }

  try {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_names));
    if (!planner_instance->initialize(kinematic_model, node, node->get_namespace())) {
      RCLCPP_ERROR(logger, "Could not initialize planner instance");
    }
    RCLCPP_INFO(
      logger, "Using planning interface '%s'",
      planner_instance->getDescription().c_str());
  } catch (pluginlib::PluginlibException & ex) {
    const std::vector<std::string> & classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto & cls : classes) {
      ss << cls << " ";
    }
    RCLCPP_ERROR(
      logger, "Exception while loading planner '%s': %s",
      planner_plugin_names.c_str(), ex.what());
  }

  moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");

  // Pose goal
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  moveit_msgs::msg::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints(
    "panda_link8", pose, tolerance_pose,
    tolerance_angle);
  req.group_name = "panda_arm";
  req.goal_constraints.push_back(pose_goal);

  // Planning context to incorporate the planning scene
  planning_interface::PlanningContextPtr context =
    planner_instance->getPlanningContext(
    planning_scene_ptr, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS) {
    RCLCPP_ERROR(logger, "Could not compute plan successfully");
    return 0;
  }

  // Visualize the plan in RViz
  std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
    node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/display_planned_path", 1);

  moveit_msgs::msg::DisplayTrajectory display_trajectory;

  moveit_msgs::msg::MotionPlanResponse response;
  res.getMessage(response);

  namespace rvt = rviz_visual_tools;

  moveit_visual_tools::MoveItVisualTools visual_tools(node, "panda_link0", "/rviz_visual_tools",
    move_group.getRobotModel());

  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();
  visual_tools.trigger();

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);


  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher->publish(display_trajectory);

  robot_state->setJointGroupPositions(
    joint_model_group,
    response.trajectory.joint_trajectory.points.back().positions);

  // visualize the goal state
  visual_tools.publishAxisLabeled(pose.pose, "goal_1");
  visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();
  return -1;
}


int main(int argc, char * argv[])
{
  // Initialize ROS2
  rclcpp::init(argc, argv);
  // instantiate the node.
  // This enables loading undeclared parameters
  // best practice would be to declare parameters in the corresponding classes
  // and provide descriptions about expected use
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
    "move_panda",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Spin up a SingleThreadedExecutor to run the node, such that the current state monitor can get information about the robot's state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() {executor.spin();}).detach();

  // Get the logger, so we can log messages.
  auto const logger = rclcpp::get_logger("move_panda");

  // Create a move group interface for the panda arm
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");

  // Robot model
  robot_model_loader::RobotModelLoader robot_model_loader(node);
  const moveit::core::RobotModelPtr & kinematic_model = robot_model_loader.getModel();
  RCLCPP_INFO(logger, "Model frame: %s", kinematic_model->getModelFrame().c_str());

  // Robot State
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
  // robot_state->setToDefaultValues();
  const moveit::core::JointModelGroup * joint_model_group = kinematic_model->getJointModelGroup(
    "panda_arm");

  // Get the joint names from the joint model group. The number of joints is always the same as the getVariableCount() function.
  const std::vector<std::string> & joint_names = joint_model_group->getVariableNames();

  RCLCPP_INFO(logger, "Available Planning Groups:");
  std::copy(
    move_group_interface.getJointModelGroupNames().begin(),
    move_group_interface.getJointModelGroupNames().end(),
    std::ostream_iterator<std::string>(std::cout, ", "));


  // Get the current state of the robot.
  // moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(30);
  // std::vector<double> joint_group_positions;
  // current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const double * joint_state = robot_state->getJointPositions(joint_names[i]);
    RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), *joint_state);
  }


  // Get the joint state values.
  std::vector<double> joint_values;
  robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  // Print the joint state values and for the individual joints.
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Check if any joints are outside of the joint limits
  RCLCPP_INFO_STREAM(
    logger,
    "Current state is " << (robot_state->satisfiesBounds() ? "valid" : "not valid"));

  // Forward kinematics
  // robot_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d & end_effector_state = robot_state->getGlobalLinkTransform(
    "panda1_link7");

  // Print the end-effector pose
  RCLCPP_INFO_STREAM(
    logger,
    "End effector Translation: " << end_effector_state.translation() << "\n");
  RCLCPP_INFO_STREAM(
    logger,
    "End effector Rotation: " << end_effector_state.rotation() << "\n");

  // Inverse kinematics
  // double timeout = 0.1;
  // bool found_ik = robot_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // if (found_ik) {
  //   robot_state->copyJointGroupPositions(joint_model_group, joint_values);
  //   for (std::size_t i = 0; i < joint_names.size(); ++i) {
  //     RCLCPP_INFO(logger, "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  //   }
  // } else {
  //   RCLCPP_ERROR(logger, "Did not find IK solution");
  // }

  move_group_interface.setStartStateToCurrentState();

  tf2::Quaternion orientation;
  orientation.setRPY(-3.14, 0, 0);
  orientation.normalize();

  // Set the target pose
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.623;
  target_pose.position.y = -0.1;
  target_pose.position.z = 0.281;
  // target_pose.orientation.x = 0.0;
  // target_pose.orientation.y = 0.0;
  // target_pose.orientation.z = 0.0;
  target_pose.orientation.x = orientation.x();
  target_pose.orientation.y = orientation.y();
  target_pose.orientation.z = orientation.z();
  target_pose.orientation.w = orientation.w();

  // Set the target pose of the move group
  move_group_interface.setPoseTarget(target_pose);

  // Plan the trajectory
  auto const [success, plan] = [&move_group_interface] {
      moveit::planning_interface::MoveGroupInterface::Plan plan;
      bool const ok = static_cast<bool>(move_group_interface.plan(plan));
      return std::make_pair(ok, plan);
    }();

  // Execute the plan
  if (success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Failed to plan the trajectory");
  }


  // Planning scene tutorial
  // The planning scene is a single snapshot of the environment, including the robot and all objects in the environment. This tutorial is only for illustrative purposes.
  planning_scene::PlanningScene planning_scene(kinematic_model);

  // now we have a planning scene.
  // Then we can do self collision checking
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(logger, "Self collision: " << (collision_result.collision ? "yes" : "no"));


  // Change the internal state of the robot
  moveit::core::RobotState & current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(logger, "Self collision: " << (collision_result.collision ? "yes" : "no"));

  // checking for a group
  collision_request.group_name = "panda_arm";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(logger, "Self collision: " << (collision_result.collision ? "yes" : "no"));

  collision_request.group_name = "hand";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(logger, "Self collision: " << (collision_result.collision ? "yes" : "no"));

  // Getting contact information
  // Manually set the position of the arm to the starting position.
  std::vector<double> joint_values_start = {0.0, 0.0, 0.0, -1.571, 0.0, 1.571, -0.785};
  const moveit::core::JointModelGroup * joint_model_group_start =
    kinematic_model->getJointModelGroup(
    "panda_arm");
  current_state.setJointGroupPositions(joint_model_group_start, joint_values_start);
  RCLCPP_INFO_STREAM(
    logger,
    "Current state is " << (current_state.satisfiesBounds(
      joint_model_group) ? "valid" : "not valid"));

  // We can get the information about the contact points
  // Set the state specifically outside of the working area.
  std::vector<double> joint_values_outside = {0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0};
  current_state.setJointGroupPositions(joint_model_group_start, joint_values_outside);
  RCLCPP_INFO_STREAM(
    logger,
    "Current state is " << (current_state.satisfiesBounds(
      joint_model_group) ? "valid" : "not valid"));

  // Active contact points
  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(
    logger,
    "Test 5: Current state is " << (collision_result.collision ? "in" : "not in") <<
      " self collision");
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it) {
    RCLCPP_INFO_STREAM(
      logger,
      "Contact between: " << it->first.first << " and " << it->first.second);
  }


  // Motion API
  // planners are setup as plugins for MoveIt. We need a RobotModelLoader object which is needed for the plugin.
  // The RobotModelLoader object is used to load the robot model.


  motion_planning_api(node);

  rclcpp::shutdown();
  return 0;
}
