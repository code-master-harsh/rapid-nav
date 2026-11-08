"""Publish the robot description: the xacro model, through robot_state_publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Camera settings of the RGB-D sensor, as (argument name, default value).
CAMERA_ARGUMENTS = [
    ('camera_width', '640'),
    ('camera_height', '480'),
    ('camera_rate', '30'),
    ('camera_hfov', '1.211'),
    ('camera_pitch', '0.0'),
]


def generate_launch_description():
    """Declare the arguments and build the description node once they are known."""
    arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulated clock (true inside Gazebo, false for the model view).',
        ),
    ]
    for name, default in CAMERA_ARGUMENTS:
        arguments.append(
            DeclareLaunchArgument(name, default_value=default, description='RGB-D camera setting.')
        )
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context):
    """Resolve the argument values to strings and build robot_state_publisher for the model."""
    model = os.path.join(
        get_package_share_directory('rapid_nav_description'), 'urdf', 'rapid_nav.urdf.xacro'
    )
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).strip().lower() == 'true'
    camera_arguments = [
        '{}:={}'.format(name, LaunchConfiguration(name).perform(context))
        for name, _ in CAMERA_ARGUMENTS
    ]
    description = ParameterValue(
        Command(['xacro ', model, ' ', ' '.join(camera_arguments)]), value_type=str
    )
    node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': description, 'use_sim_time': use_sim_time}],
    )
    return [node]
