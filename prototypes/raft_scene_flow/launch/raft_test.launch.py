from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os


def generate_launch_description():

    workspace = os.path.expanduser('~/Robot vision')

    world = os.path.join(
        workspace,
        'src',
        'optical_flow',
        'worlds',
        'test_scene.sdf'
    )

    venv_python = os.path.join(
        workspace,
        '.venv',
        'bin',
        'python'
    )

    raft_script = os.path.join(
        workspace,
        'src',
        'optical_flow',
        'optical_flow',
        'raft_node.py'
    )

    rviz_config = os.path.join(
        workspace,
        'src',
        'optical_flow',
        'rviz',
        'raft.rviz'
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gz',
            'sim',
            '-r',
            world
        ],
        output='screen'
    )

    bridge = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_gz_bridge',
            'parameter_bridge',

            '/camera/rgbd/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/rgbd/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/camera/rgbd/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen'
    )

    tf = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'tf2_ros',
            'static_transform_publisher',
            '0', '0', '0',
            '0', '0', '0',
            'world',
            'camera_model/camera_link/rgbd_camera'
        ],
        output='screen'
    )

    raft = ExecuteProcess(
        cmd=[
            venv_python,
            raft_script
        ],
        output='screen',
        additional_env={
            'PYTHONUNBUFFERED': '1'
        }
    )

    rviz = ExecuteProcess(
        cmd=[
            'rviz2',
            '-d',
            rviz_config
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        tf,
        raft,
        rviz
    ])