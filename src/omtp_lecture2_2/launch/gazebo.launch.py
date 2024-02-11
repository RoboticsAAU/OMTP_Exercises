from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess


def generate_launch_description():
# get the package path:
        package_path = os.path.join(get_package_share_directory('omtp_lecture2_2'))


# Moveit package path 
        moveit_config = os.path.join(get_package_share_directory('lecture2_moveit_config'))


        # Include moveit demo launch file
        moveit_demo_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(moveit_config, 'launch', 'demo.launch.py')]),# launch_arguments={'use_sim_time': 'true'}.items()
        )


        gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'use_sim_time': 'true'}.items()
        )

        # Include the spawn entity node
        spawn_entity = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'smart_lab'],
                output='screen',
                emulate_tty=True,
        )



        return LaunchDescription([
                moveit_demo_launch,
                gazebo,
                spawn_entity,
        ])

