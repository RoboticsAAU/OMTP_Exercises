from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os

def generate_launch_description():

    # Package information
    package_name = 'omtp_lecture1_2'
    pkg_path = os.path.join(get_package_share_directory(package_name))
 
    # Define the path to the rviz file
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'omtp.rviz')

    # Publish robot states
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
    )

    # Gazebo world file:
    # gazebo_world = os.path.join(pkg_path, 'worlds', 'empty.world')

    # Include launch description for gazebo from gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py' 
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Launch rviz
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config_path]
        )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        gazebo,
        rviz,
    ])
