import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Package directory
    pkg_share = get_package_share_directory('four_dof_arm')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_file = os.path.join(pkg_share, 'worlds', 'arm_world.world')
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm.urdf.xacro')

    return LaunchDescription([
        # Declare use_sim_time argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        # Start Gazebo server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gzserver.launch.py'
                )
            ),
            launch_arguments={'world': world_file}.items()
        ),

        # Start Gazebo client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('gazebo_ros'),
                    'launch', 'gzclient.launch.py'
                )
            )
        ),

        # Spawn the arm model
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'four_dof_arm',
                '-file', Command(['xacro ', urdf_file]),
                '-x', '0.0', '-y', '0.0', '-z', '0.05'
            ],
            output='screen'
        ),

        # Start the arm control node
        Node(
            package='four_dof_arm',
            executable='arm_control_node',
            name='arm_control_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Start RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'arm_config.rviz')],
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # Publish robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'robot_description': Command(['xacro ', urdf_file])}
            ]
        )
    ])