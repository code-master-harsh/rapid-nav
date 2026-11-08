"""Start Gazebo with the arena, spawn the robot and bridge its topics to ROS 2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Camera settings of the RGB-D sensor, as (argument name, default value).
CAMERA_ARGUMENTS = [
    ('camera_width', '640'),
    ('camera_height', '480'),
    ('camera_rate', '30'),
    ('camera_hfov', '1.211'),
    ('camera_pitch', '0.0'),
]


def generate_launch_description():
    """Declare the arguments and set the Gazebo resource path before the simulation starts."""
    share = get_package_share_directory('rapid_nav_gazebo')
    arguments = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(share, 'worlds', 'arena.sdf'),
            description='SDF world to load (default: the arena).',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run the server without a window, with software rendering.',
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x position (m).'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y position (m).'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn heading (rad).'),
    ]
    for name, default in CAMERA_ARGUMENTS:
        arguments.append(
            DeclareLaunchArgument(name, default_value=default, description='RGB-D camera setting.')
        )
    resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(share, 'models'),
    )
    return LaunchDescription(arguments + [resource_path, OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    """Build Gazebo, the description, the spawn and the bridge from the resolved arguments."""
    share = get_package_share_directory('rapid_nav_gazebo')
    world = LaunchConfiguration('world').perform(context)
    headless = LaunchConfiguration('headless').perform(context).strip().lower() == 'true'
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)
    camera_values = [
        (name, LaunchConfiguration(name).perform(context)) for name, _ in CAMERA_ARGUMENTS
    ]

    gz_flags = ['-r', '-v', '2']
    if headless:
        gz_flags += ['-s', '--headless-rendering']
    gz_args = ' '.join(gz_flags + [world])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rapid_nav_description'),
                'launch',
                'description.launch.py',
            )
        ),
        launch_arguments=[('use_sim_time', 'true')] + camera_values,
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_rapid_nav',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rapid_nav',
            '-x', x,
            '-y', y,
            '-z', '0.01',
            '-Y', yaw,
        ],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(share, 'config', 'bridge.yaml'),
            'use_sim_time': True,
        }],
    )
    return [gazebo, description, spawn, bridge]
