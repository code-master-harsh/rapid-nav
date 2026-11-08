"""Show the robot model in RViz without Gazebo, with joint sliders to move the joints."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Camera settings of the RGB-D sensor, as (argument name, default value).
# The same list as in description.launch.py.
CAMERA_ARGUMENTS = [
    ('camera_width', '640'),
    ('camera_height', '480'),
    ('camera_rate', '30'),
    ('camera_hfov', '1.211'),
    ('camera_pitch', '0.0'),
]


def generate_launch_description():
    """Declare the camera arguments and start the model with RViz and the joint sliders."""
    arguments = [
        DeclareLaunchArgument(name, default_value=default, description='RGB-D camera setting.')
        for name, default in CAMERA_ARGUMENTS
    ]
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    """Include the description (without simulated time) and build the GUI nodes."""
    share = get_package_share_directory('rapid_nav_description')
    description_launch = os.path.join(share, 'launch', 'description.launch.py')
    rviz_config = os.path.join(share, 'rviz', 'model.rviz')

    forwarded = {'use_sim_time': 'false'}
    for name, _ in CAMERA_ARGUMENTS:
        forwarded[name] = LaunchConfiguration(name)

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(description_launch),
        launch_arguments=forwarded.items(),
    )
    joint_sliders = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
    )
    return [description, joint_sliders, rviz]
