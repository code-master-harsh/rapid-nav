"""Checks of the bringup package: the EKF configuration and the simulation launch file."""

import importlib.util
import os

from launch.actions import DeclareLaunchArgument
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.normpath(os.path.join(HERE, '..'))
EKF = os.path.join(PACKAGE, 'config', 'ekf.yaml')
SIM_LAUNCH = os.path.join(PACKAGE, 'launch', 'sim.launch.py')

EXPECTED_ARGUMENTS = {'world', 'headless', 'rviz', 'odom_tf', 'x', 'y', 'yaw'}


def load_ekf_parameters():
    """Return the ros__parameters of ekf_filter_node."""
    with open(EKF, encoding='utf-8') as handle:
        document = yaml.safe_load(handle)
    return document['ekf_filter_node']['ros__parameters']


def test_ekf_config_loads_with_the_expected_frames():
    """ekf.yaml names the odom and base frames the TF tree uses."""
    parameters = load_ekf_parameters()
    assert parameters['odom_frame'] == 'odom'
    assert parameters['base_link_frame'] == 'base_footprint'
    assert parameters['publish_tf'] is True
    assert parameters['two_d_mode'] is True
    assert parameters['use_sim_time'] is True


def test_ekf_fusion_masks_are_fifteen_elements():
    """The wheels fuse vx and vy; the IMU fuses yaw rate and forward acceleration."""
    parameters = load_ekf_parameters()
    odom_config = parameters['odom0_config']
    imu_config = parameters['imu0_config']
    assert len(odom_config) == 15
    assert len(imu_config) == 15
    assert [i for i, flag in enumerate(odom_config) if flag] == [6, 7]
    assert [i for i, flag in enumerate(imu_config) if flag] == [11, 12]
    assert parameters['imu0_remove_gravitational_acceleration'] is True


def test_sim_launch_declares_the_arguments():
    """sim.launch.py declares the arguments of the simulation."""
    spec = importlib.util.spec_from_file_location('sim_launch', SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    declared = {entity.name for entity in description.entities
                if isinstance(entity, DeclareLaunchArgument)}
    assert declared == EXPECTED_ARGUMENTS
