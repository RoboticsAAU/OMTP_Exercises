#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def main():
    """
    Function to control a Franka robot using MoveIt in ROS Noetic.

    This function initializes the MoveIt commander and ROS node, creates move group interfaces for
    the robot's arm and hand, and executes a series of movements. These movements include moving
    the arm and hand to predefined poses ('home_pos', 'up_pos', 'open_gripper', 'close_gripper')
    and performing a Cartesian movement.

    The arm and hand movements are achieved through named target positions, while the Cartesian
    movement is defined by specifying waypoints for the end effector to follow. The script provides
    feedback through ROS logging, indicating the progress and success of each action.
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('franka_moveit_script', anonymous=True)
    rospy.loginfo('Moveit script started for Franka robot.')

    # Create move group interface for the arm and hand
    arm_group = moveit_commander.MoveGroupCommander("panda1_arm")
    hand_group = moveit_commander.MoveGroupCommander("panda1_hand")
    rospy.loginfo('Move group commanders for arm and hand initialized.')

    # Function to move the arm to a predefined pose
    def move_arm_to_pose(pose_name):
        rospy.loginfo(f'- Moving arm to {pose_name}.')
        arm_group.set_named_target(pose_name)
        plan_success, plan, planning_time, error_code = arm_group.plan()
        if plan_success:
            rospy.loginfo(f'Planning successful. Execution time: {planning_time} seconds.')
            arm_group.execute(plan, wait=True)
            rospy.loginfo(f'Arm moved to {pose_name} pose.')
        else:
            rospy.logwarn(f'Arm planning to {pose_name} failed with error code: {error_code}.')

    # Function to move the hand to a predefined pose
    def move_hand_to_pose(pose_name):
        rospy.loginfo(f'- Moving hand to {pose_name}.')
        hand_group.set_named_target(pose_name)
        plan_success, plan, planning_time, error_code = hand_group.plan()
        if plan_success:
            rospy.loginfo(f'Planning successful. Execution time: {planning_time} seconds.')
            hand_group.execute(plan, wait=True)
            rospy.loginfo(f'Hand moved to {pose_name} pose.')
        else:
            rospy.logwarn(f'Hand planning to {pose_name} failed with error code: {error_code}.')

    # Function to move the arm to a specific joint configuration
    def move_arm_to_joint_values(joint_values):
        rospy.loginfo(f'- Moving arm to joint values: {joint_values}')
        arm_group.set_joint_value_target(joint_values)
        plan_success, plan, planning_time, error_code = arm_group.plan()
        if plan_success:
            rospy.loginfo(f'Planning successful. Execution time: {planning_time} seconds.')
            arm_group.execute(plan, wait=True)
            rospy.loginfo('Arm moved to specified joint configuration.')
        else:
            rospy.logwarn(f'Arm planning for joint values failed with error code: {error_code}.')

    # Cartesian movement
    def cartesian_movement():
        rospy.loginfo('- Starting Cartesian movement.')
        waypoints = []
        current_pose = arm_group.get_current_pose().pose
        new_pose = geometry_msgs.msg.Pose()

        # Define the new pose (Update as needed)
        new_pose.position.x = current_pose.position.x + 0.0
        new_pose.position.y = current_pose.position.y + 0.0
        new_pose.position.z = current_pose.position.z - 0.1
        new_pose.orientation = current_pose.orientation

        waypoints.append(new_pose)

        # Parameters for Cartesian path
        eef_step = 0.01  # End effector step in meters
        jump_threshold = 0.0  # Jump threshold for avoiding jumps in joint space

        # Compute the Cartesian path
        # fraction indicates the fraction of the path that was successfully planned
        (plan, fraction) = arm_group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            eef_step,    # eef_step
            jump_threshold)  # jump_threshold

        rospy.loginfo(f'Cartesian path fraction: {fraction:.2f}')
        if fraction == 1.0:
            rospy.loginfo('Cartesian path planning successful, executing.')
            arm_group.execute(plan, wait=True)
            rospy.loginfo('Cartesian movement executed.')
        else:
            rospy.logwarn('Cartesian path planning failed.')

    # Moving the arm and hand to predefined poses
    move_arm_to_pose("panda1_up")
    move_arm_to_pose("panda1_home")
    move_hand_to_pose("panda1_hand_open")
    move_hand_to_pose("panda1_hand_close")

    # Move the arm to the example joint values after the Cartesian movement
    example_joint_values = [-0.2700, -0.5000, 0.1600, -2.0000, -0.2200, 1.7000, 0.1100]
    move_arm_to_joint_values(example_joint_values)

    # Perform Cartesian movement
    cartesian_movement()

    rospy.loginfo('Script execution complete.')
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
