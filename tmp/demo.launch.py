from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("omtp_factory", package_name="lecture2_moveit_config").to_moveit_configs()
    demo_launch_description = generate_demo_launch(moveit_config)
    demo_launch_description.add_action(
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py' 
                )]),
                launch_arguments={'use_sim_time': 'true'}.items()

        )
    )
    demo_launch_description.add_action(
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'smart_lab'],
            output='screen',
            emulate_tty=True,
        )
    )
    return demo_launch_description
