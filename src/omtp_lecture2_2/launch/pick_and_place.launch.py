from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
import os



def generate_launch_description():
    
    ld = LaunchDescription()
    gazebo_launch =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('omtp_lecture2_2'), 'launch', 'gazebo.launch.py')])
    )
    ld.add_action(gazebo_launch)

    

    for i in range(1, 100):
        spawn_box = TimerAction(
            period=15.0 * i,
            actions=[
            IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('omtp_lecture2_2'), 'launch', 'spawn_box.launch.py')])
                )
            ]
        )  
        ld.add_action(spawn_box)

    move_launch = TimerAction(
        period=3.0,
        actions=[
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('omtp_lecture2_2'), 'launch', 'move.launch.py')])
            )
        ]
    )  
    ld.add_action(move_launch)

    return ld

