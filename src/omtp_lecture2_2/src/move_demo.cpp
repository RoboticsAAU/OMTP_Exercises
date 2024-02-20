#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_robot_state.h>
#include <moveit_msgs/msg/display_trajectory.h>
#include <moveit_msgs/msg/attached_collision_object.h>
#include <moveit_msgs/msg/collision_object.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.h>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  rclcpp::Node::SharedPtr move_group_node = rclcpp::Node::make_shared("move_demo", options);

  // Spin a single thread executor for the current state monitor to get information about the robots state.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() {executor.spin();}).detach();

  // move_groups
  static const std::string PLANNING_GROUP = "panda_arm";
  static const std::string HAND_GROUP = "hand";

  // Logger
  rclcpp::Logger logger = move_group_node->get_logger();

  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  moveit::planning_interface::MoveGroupInterface move_group_hand(move_group_node, HAND_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface::Plan plan;


  // Get the joint model groups
  const moveit::core::JointModelGroup * joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  const moveit::core::JointModelGroup * joint_model_group_hand =
    move_group.getCurrentState()->getJointModelGroup(HAND_GROUP);

  while (rclcpp::ok()) {
    // Set the gripper joint values
    std::vector<double> gripper_joints = {0.035, 0.035};
    bool success;
    // Get the joint names
    std::vector<std::string> joint_names_hand = joint_model_group_hand->getVariableNames();
    move_group_hand.setJointValueTarget(joint_names_hand, gripper_joints);
    success = static_cast<bool>(move_group_hand.plan(plan));
    RCLCPP_INFO(logger, "(gripper on) (joint space goal) %s", success ? "" : "FAILED");
    if (success) {
      move_group_hand.execute(plan);
    }


    // Set move_group parameters for the planner to effectively plan the motion
    move_group.setNumPlanningAttempts(10);
    move_group.setPoseReferenceFrame("panda1_link0");
    move_group.setMaxVelocityScalingFactor(1.0);
    move_group.setMaxAccelerationScalingFactor(1.0);
    move_group.setPlanningTime(5.0);
    // move_group.setGoalTolerance(0.01);
    // move_group.setGoalOrientationTolerance(0.2);
    move_group.setPlanningPipelineId("ompl");
    // Log the planning_frame
    RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());

    // Log the availble planning groups
    RCLCPP_INFO(logger, "available planning groups:");
    std::copy(
      move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
      std::ostream_iterator<std::string>(std::cout, ", "));


    // Set the target pose
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI, 0, M_PI_4);
    orientation.normalize();
    geometry_msgs::msg::Pose target_pose1;
    // Set the orientation in quaternions
    target_pose1.orientation.x = orientation.x();
    target_pose1.orientation.y = orientation.y();
    target_pose1.orientation.z = orientation.z();
    target_pose1.orientation.w = orientation.w();


    // set the target position
    target_pose1.position.x = 0.47;
    target_pose1.position.y = -0.45;
    target_pose1.position.z = 0.240;

    // Set the starting position
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose1);

    success = static_cast<bool>(move_group.plan(plan));
    RCLCPP_INFO(logger, "Planned to approach %s", success ? "" : "FAILED");
    if (success) {
      move_group.execute(plan);
      RCLCPP_INFO(logger, "Moved to approach");
    }


    RCLCPP_INFO(logger, "Planning approach");

    std::vector<geometry_msgs::msg::Pose> pick_waypoints;
    target_pose1.position.z -= 0.05;
    pick_waypoints.push_back(target_pose1);

    target_pose1.position.z -= 0.05;
    pick_waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory approach_trajectory;
    const double jump_threshold = 0.0;
    // Distance between points in the Cartesian path
    const double eef_step = 0.01;

    move_group.computeCartesianPath(
      pick_waypoints, eef_step, jump_threshold, approach_trajectory);

    move_group.execute(approach_trajectory);

    RCLCPP_INFO(logger, "Approach done. Planning to pick");

    sleep(1);

    // Set the gripper joint values
    gripper_joints = {0.024, 0.024};
    // Get the joint names
    move_group_hand.setJointValueTarget(joint_names_hand, gripper_joints);
    success = static_cast<bool>(move_group_hand.plan(plan));
    RCLCPP_INFO(logger, "(gripper on) Close path planned %s", success ? "" : "FAILED");
    if (success) {
      move_group_hand.execute(plan);
    }

    sleep(1);

    RCLCPP_INFO(logger, "Planning retreat");

    std::vector<geometry_msgs::msg::Pose> retreat_waypoints;
    target_pose1.position.z += 0.05;
    retreat_waypoints.push_back(target_pose1);

    target_pose1.position.z += 0.05;
    retreat_waypoints.push_back(target_pose1);

    moveit_msgs::msg::RobotTrajectory retreat_trajectory;

    move_group.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, retreat_trajectory);

    move_group.execute(retreat_trajectory);

    RCLCPP_INFO(logger, "Retreat done.");


    // set the target position
    target_pose1.position.x = 0.47;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.7;

    // Set the starting position
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose1);

    success = static_cast<bool>(move_group.plan(plan));
    RCLCPP_INFO(logger, "Planned to to drop point: %s", success ? "" : "FAILED");
    if (success) {
      move_group.execute(plan);
      RCLCPP_INFO(logger, "Moved to drop point");
    }


    // Set the gripper joint values
    gripper_joints = {0.04, 0.04};
    // Get the joint names
    move_group_hand.setJointValueTarget(joint_names_hand, gripper_joints);
    success = static_cast<bool>(move_group_hand.plan(plan));
    RCLCPP_INFO(logger, "Open path planned %s", success ? "" : "FAILED");
    if (success) {
      move_group_hand.execute(plan);
    }
  }

  // Shutdown the node
  move_group_node->get_node_base_interface()->get_context()->shutdown("");
  return 0;

}
