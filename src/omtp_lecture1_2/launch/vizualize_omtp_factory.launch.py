from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Load omtp factory to the parameter server under 'robot_description'
        Node(
            package='xacro',
            executable='xacro',
            name='xacro',
            output='screen',
            arguments=['$(find omtp_lecture1_2)/urdf/omtp_factory.xacro']
        ),
        
        # Publish robot states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
        ),
        
        # Publish joint states
        Node(
            package='joint_state_publisher_gui',  # or 'joint_state_publisher' depending on your needs
            executable='joint_state_publisher_gui',  # or 'joint_state_publisher'
            name='joint_state_publisher',
            output='screen',
            # condition=IfCondition(LaunchConfiguration('gui')),
        ),
        
        # Launch rviz
        Node(
            package='rviz',
            executable='rviz',
            name='rviz',
            output='screen',
            arguments=['-d', '$(find omtp_lecture1_2)/rviz/omtp.rviz']
        ),
    ])
