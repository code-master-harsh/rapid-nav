"""Checks of the Gazebo launch file and the bridge configurations."""

import importlib.util
import os

from launch.actions import DeclareLaunchArgument
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.normpath(os.path.join(HERE, '..'))
LAUNCH_FILE = os.path.join(PACKAGE, 'launch', 'gazebo.launch.py')
BRIDGE = os.path.join(PACKAGE, 'config', 'bridge.yaml')
BRIDGE_WHEEL_TF = os.path.join(PACKAGE, 'config', 'bridge_wheel_tf.yaml')

EXPECTED_ARGUMENTS = {
    'world', 'headless', 'x', 'y', 'yaw',
    'camera_width', 'camera_height', 'camera_rate', 'camera_hfov', 'camera_pitch',
}

# ROS topic -> ROS message type: the topic table without /tf and /odometry/filtered.
EXPECTED_BRIDGE_TOPICS = {
    '/clock': 'rosgraph_msgs/msg/Clock',
    '/cmd_vel': 'geometry_msgs/msg/Twist',
    '/odom': 'nav_msgs/msg/Odometry',
    '/joint_states': 'sensor_msgs/msg/JointState',
    '/imu': 'sensor_msgs/msg/Imu',
    '/scan': 'sensor_msgs/msg/LaserScan',
    '/camera/image': 'sensor_msgs/msg/Image',
    '/camera/depth_image': 'sensor_msgs/msg/Image',
    '/camera/camera_info': 'sensor_msgs/msg/CameraInfo',
    '/camera/points': 'sensor_msgs/msg/PointCloud2',
    '/ground_truth/odom': 'nav_msgs/msg/Odometry',
}


def load_launch_module():
    """Import gazebo.launch.py from the source tree."""
    spec = importlib.util.spec_from_file_location('gazebo_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_launch_declares_the_arguments():
    """gazebo.launch.py declares the world, headless, spawn and camera arguments."""
    module = load_launch_module()
    description = module.generate_launch_description()
    declared = {entity.name for entity in description.entities
                if isinstance(entity, DeclareLaunchArgument)}
    assert declared == EXPECTED_ARGUMENTS


def test_bridge_config_lists_the_topics():
    """bridge.yaml bridges the eleven topics of the table with the expected ROS types."""
    with open(BRIDGE, encoding='utf-8') as handle:
        entries = yaml.safe_load(handle)
    topics = {entry['ros_topic_name']: entry['ros_type_name'] for entry in entries}
    assert topics == EXPECTED_BRIDGE_TOPICS
    points = next(entry for entry in entries if entry['ros_topic_name'] == '/camera/points')
    assert points.get('lazy') is True


def test_wheel_tf_bridge_is_gz_to_ros():
    """bridge_wheel_tf.yaml carries the DiffDrive transform from gz to /tf and nothing else."""
    with open(BRIDGE_WHEEL_TF, encoding='utf-8') as handle:
        entries = yaml.safe_load(handle)
    assert len(entries) == 1
    entry = entries[0]
    assert entry['gz_topic_name'] == '/wheel_odom_tf'
    assert entry['ros_topic_name'] == '/tf'
    assert entry['ros_type_name'] == 'tf2_msgs/msg/TFMessage'
    assert entry['direction'] == 'GZ_TO_ROS'
