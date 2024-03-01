
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import random

def generate_launch_description():

    package_path = os.path.join(get_package_share_directory('omtp_lecture2_2'))

    box_id = random.randint(0, 100000)

    # Include the spawn entity node
    spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file',[os.path.join(package_path, 'descriptions', 'box.sdf')], '-entity', 'box'+ str(box_id), '-reference_frame', 'smart_lab::world_interface'],
            output='screen',
            emulate_tty=True,
    )
    
    return LaunchDescription([spawn_entity])