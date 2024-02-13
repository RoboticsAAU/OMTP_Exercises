from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import ExecuteProcess, TimerAction
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
        # get the package path:
        package_path = os.path.join(get_package_share_directory('omtp_lecture2_2'))


        # Moveit package path 
        moveit_config_path = os.path.join(get_package_share_directory('lecture2_single_panda_moveit_config'))

        # Moveit config with robot description publish parameters
        moveit_config = (MoveItConfigsBuilder("omtp_factory", package_name="lecture2_single_panda_moveit_config")
                        .planning_scene_monitor(
                                publish_robot_description=True,
                                publish_robot_description_semantic=True)
                        ).to_moveit_configs()


        # Use Sim time to be set for additional launch files that run this launch file.
        use_sim_time_argument = DeclareLaunchArgument('use_sim_time',
                                default_value='true',
                                description='Use Simulation (Gazebo) time')

        # Virtual joint transforms
        virtual_joints_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(moveit_config_path, 'launch','static_virtual_joint_tfs.launch.py')]),
                # launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        )

        # Robot State Publisher
        robot_state_publisher = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(moveit_config_path, 'launch','rsp.launch.py')]),
                launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        )


        # Movegroup node with use_sim_time to ensure that it is synced with simulation time
        move_group_node = Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), {'use_sim_time':LaunchConfiguration('use_sim_time')}],
        )


        # Define rviz parameters
        rviz_parameters = [
                moveit_config.planning_pipelines,
                moveit_config.robot_description_kinematics,
        ]

        # Include the moveit.rviz file
        rviz_config_argument = DeclareLaunchArgument(
            "rviz_config",
            default_value=[os.path.join(moveit_config_path, 'config','moveit.rviz')]
        )

        # Launch rviz2 node
        rviz = Node(
                package="rviz2",
                executable="rviz2",
                output="screen",
                respawn=False,
                arguments=["-d", LaunchConfiguration("rviz_config"), LaunchConfiguration('use_sim_time')],
                parameters=rviz_parameters,
        )


        # Spawn Controllers with a delay as until after gazebo has launched and the model has spawned, there is no controller plugins available. 
        spawn_controllers = TimerAction(
                period=0.1,
                actions=[
                        IncludeLaunchDescription(
                                PythonLaunchDescriptionSource([os.path.join(moveit_config_path, 'launch','spawn_controllers.launch.py')]),
                                # launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
                        )
                ]
        )

        # Spawn gazebo with use_sim_time to ensure that it is synced with simulation time (might not be necessary)
        gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
        )

        # Include the spawn entity node
        spawn_entity = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=['-topic', 'robot_description', '-entity', 'smart_lab'],
                output='screen',
                emulate_tty=True,
        )



        # Generate the launch description
        return LaunchDescription([
                use_sim_time_argument,
                rviz_config_argument,
                virtual_joints_launch,
                robot_state_publisher,
                move_group_node,
                rviz,
                spawn_controllers,
                gazebo,
                spawn_entity,
        ])

