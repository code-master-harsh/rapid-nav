"""Start the whole simulation: Gazebo with the arena and the robot, the odometry and RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ODOM_TF_MODES = ('ekf', 'wheel')


def generate_launch_description():
    """Declare the arguments; the nodes are built once their values are known."""
    gazebo_share = get_package_share_directory('rapid_nav_gazebo')
    arguments = [
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(gazebo_share, 'worlds', 'arena.sdf'),
            description='SDF world to load (default: the arena).',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run without a window and without RViz (software rendering).',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the sim.rviz configuration (not when headless).',
        ),
        DeclareLaunchArgument(
            'odom_tf',
            default_value='ekf',
            description="Source of odom -> base_footprint: 'ekf' (robot_localization) or "
                        "'wheel' (the DiffDrive transform from Gazebo).",
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Spawn x position (m).'),
        DeclareLaunchArgument('y', default_value='0.0', description='Spawn y position (m).'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Spawn heading (rad).'),
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    """Build the nodes for the resolved odom_tf, rviz and headless arguments."""
    share = get_package_share_directory('rapid_nav_bringup')
    gazebo_share = get_package_share_directory('rapid_nav_gazebo')

    odom_tf = LaunchConfiguration('odom_tf').perform(context).strip().lower()
    if odom_tf not in ODOM_TF_MODES:
        raise RuntimeError(
            "odom_tf must be 'ekf' or 'wheel', got '{}'".format(odom_tf)
        )
    headless = LaunchConfiguration('headless').perform(context).strip().lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).strip().lower() == 'true'
    world = LaunchConfiguration('world').perform(context)
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    yaw = LaunchConfiguration('yaw').perform(context)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments=[
            ('world', world),
            ('headless', 'true' if headless else 'false'),
            ('x', x),
            ('y', y),
            ('yaw', yaw),
        ],
    )
    nodes = [gazebo]

    if odom_tf == 'ekf':
        nodes.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(share, 'config', 'ekf.yaml')],
        ))
    else:
        nodes.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_wheel_tf_bridge',
            output='screen',
            parameters=[{
                'config_file': os.path.join(gazebo_share, 'config', 'bridge_wheel_tf.yaml'),
                'use_sim_time': True,
            }],
        ))

    if rviz and not headless:
        nodes.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(share, 'rviz', 'sim.rviz')],
            parameters=[{'use_sim_time': True}],
        ))
    return nodes
