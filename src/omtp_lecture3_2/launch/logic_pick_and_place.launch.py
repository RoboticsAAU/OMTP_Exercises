
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import Node




import os

def generate_launch_description():

    moveit_config = MoveItConfigsBuilder("omtp_factory", package_name="lecture2_single_panda_moveit_config").to_moveit_configs()


    gazebo_launch =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('omtp_lecture2_2'), 'launch', 'gazebo.launch.py')])
    )

    spawn_box =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('omtp_lecture2_2'), 'launch', 'spawn_box.launch.py')])
    )

    spawn_camera =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('omtp_lecture3_2'), 'launch', 'spawn_logic_camera.launch.py')])
    )

    pick_and_place_node = Node(
        name="pick_and_place_node",
        package="omtp_lecture3_2",
        executable="logic_pick_and_place",
        output="screen",
        parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {'use_sim_time':True}
            ],
    )

    return LaunchDescription([
        gazebo_launch,
        spawn_box,
        spawn_camera,
        pick_and_place_node
    ])