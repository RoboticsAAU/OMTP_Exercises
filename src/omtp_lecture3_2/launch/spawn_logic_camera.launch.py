from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import math
import os

def generate_launch_description():
    sdf_dir = os.path.join(get_package_share_directory('ariac_sensors'), 'models', 'basic_logical_camera')
    sdf_file = os.path.join(sdf_dir, 'model.sdf')

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'basic_logical_camera', '-file', sdf_file,
                    '-x', '-1.0',
                    '-y', '3.0',
                    '-z', '2.0',
                    '-R', '0.0',
                    '-P', str(math.pi/2),
                    # '-P', '1.5',
                    '-Y', '0.0'
                ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        spawn_entity
        ])