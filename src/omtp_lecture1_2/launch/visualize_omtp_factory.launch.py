from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

import xacro
import os

def generate_launch_description():

    package_name = 'omtp_lecture1_2'
    # If the launch file is told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory(package_name))
    xacro_file = os.path.join(pkg_path, 'urdf', 'omtp_factory.xacro')
    robot_description_config = xacro.process_file(xacro_file)

 
#     rviz config path in share directory.
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'omtp.rviz')

    # Define parameters.
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time':use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
    )
    return LaunchDescription([
        # Publish robot states
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use sim time'
        ),
        node_robot_state_publisher,

        # Launch rviz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),
    ])
