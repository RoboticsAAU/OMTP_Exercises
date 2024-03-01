from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description(): 
        moveit_config = MoveItConfigsBuilder("omtp_factory", package_name="lecture2_single_panda_moveit_config").to_moveit_configs()

        move_panda_node = Node(
                name="move_panda",
                package="omtp_lecture2_2",
                executable="move_panda",
                output="screen",
                parameters=[
                        moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        {'use_sim_time':True}
                    ],
        )

        move_demo_node = Node(
                name="move_demo",
                package="omtp_lecture2_2",
                executable="move_demo",
                output="screen",
                parameters=[
                        moveit_config.robot_description,
                        moveit_config.robot_description_semantic,
                        moveit_config.robot_description_kinematics,
                        {'use_sim_time':True}
                    ],
        )

        

        return LaunchDescription([move_demo_node]) 
