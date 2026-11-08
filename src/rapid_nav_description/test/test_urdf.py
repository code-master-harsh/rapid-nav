"""Checks of the robot model: links, joints, masses, sensors, plugins and the camera arguments."""

import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
import xacro

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, '..', 'urdf', 'rapid_nav.urdf.xacro')

EXPECTED_LINKS = {
    'base_footprint',
    'base_link',
    'left_wheel_link',
    'right_wheel_link',
    'front_caster_link',
    'rear_caster_link',
    'turret_link',
    'lidar_link',
    'camera_link',
    'camera_optical_frame',
    'imu_link',
}

# joint name -> (type, parent link, child link)
EXPECTED_JOINTS = {
    'base_link_joint': ('fixed', 'base_footprint', 'base_link'),
    'left_wheel_joint': ('continuous', 'base_link', 'left_wheel_link'),
    'right_wheel_joint': ('continuous', 'base_link', 'right_wheel_link'),
    'front_caster_joint': ('fixed', 'base_link', 'front_caster_link'),
    'rear_caster_joint': ('fixed', 'base_link', 'rear_caster_link'),
    'turret_joint': ('fixed', 'base_link', 'turret_link'),
    'lidar_joint': ('fixed', 'base_link', 'lidar_link'),
    'camera_joint': ('fixed', 'base_link', 'camera_link'),
    'camera_optical_joint': ('fixed', 'camera_link', 'camera_optical_frame'),
    'imu_joint': ('fixed', 'base_link', 'imu_link'),
}

# joint name -> origin xyz (relative to the parent link)
EXPECTED_ORIGINS = {
    'camera_joint': (0.175, 0.0, 0.175),
    'lidar_joint': (0.0, 0.0, 0.225),
    'imu_joint': (0.0, 0.0, 0.10),
    'left_wheel_joint': (0.0, 0.225, 0.0),
    'right_wheel_joint': (0.0, -0.225, 0.0),
}

# sensor name -> (type, gz topic, gz_frame_id)
EXPECTED_SENSORS = {
    'rgbd_camera': ('rgbd_camera', 'camera', 'camera_optical_frame'),
    'imu': ('imu', 'imu', 'imu_link'),
    'lidar': ('gpu_lidar', 'scan', 'lidar_link'),
}

EXPECTED_PLUGINS = {
    'gz-sim-diff-drive-system',
    'gz-sim-joint-state-publisher-system',
    'gz-sim-odometry-publisher-system',
}


def build(**arguments):
    """Process the xacro model with the arguments; return the parsed tree and the URDF text."""
    mappings = {name: str(value) for name, value in arguments.items()}
    text = xacro.process_file(MODEL, mappings=mappings).toxml()
    return ET.fromstring(text), text


def parse_floats(text):
    """Split a whitespace-separated string of numbers."""
    return [float(value) for value in text.split()]


@pytest.fixture(scope='module')
def model():
    """Return the default model."""
    return build()


def test_link_set_is_exact(model):
    """The model has exactly the eleven links of the frame table."""
    root, _ = model
    links = {link.get('name') for link in root.findall('link')}
    assert links == EXPECTED_LINKS


def test_joints_types_and_parents(model):
    """The model has exactly the ten joints, with their types, parents and children."""
    root, _ = model
    joints = {}
    for joint in root.findall('joint'):
        joints[joint.get('name')] = (
            joint.get('type'),
            joint.find('parent').get('link'),
            joint.find('child').get('link'),
        )
    assert joints == EXPECTED_JOINTS


def test_joint_origins_match_the_table(model):
    """Camera, lidar, IMU and both wheels sit where the frame table says (1e-6 m)."""
    root, _ = model
    for joint in root.findall('joint'):
        name = joint.get('name')
        if name not in EXPECTED_ORIGINS:
            continue
        xyz = parse_floats(joint.find('origin').get('xyz'))
        expected = EXPECTED_ORIGINS[name]
        assert len(xyz) == 3
        for value, wanted in zip(xyz, expected):
            assert abs(value - wanted) < 1e-6, name


def test_total_mass_is_7_3_kg(model):
    """The link masses add up to 7.3 kg (within 0.01 kg)."""
    root, _ = model
    total = sum(float(link.find('inertial/mass').get('value')) for link in root.findall('link')
                if link.find('inertial/mass') is not None)
    assert abs(total - 7.3) <= 0.01


def test_sensors_and_plugins(model):
    """The three sensors and the three model plugins exist with the names, topics and frames."""
    root, _ = model
    sensors = {}
    for sensor in root.iter('sensor'):
        sensors[sensor.get('name')] = (
            sensor.get('type'),
            sensor.find('topic').text,
            sensor.find('gz_frame_id').text,
        )
    assert sensors == EXPECTED_SENSORS

    plugins = {plugin.get('filename') for plugin in root.iter('plugin')}
    assert plugins == EXPECTED_PLUGINS


def test_camera_follows_the_arguments():
    """The camera's size and rate follow the xacro arguments."""
    root, _ = build(camera_width=320, camera_height=240, camera_rate=15)
    camera = next(sensor for sensor in root.iter('sensor') if sensor.get('name') == 'rgbd_camera')
    assert camera.find('update_rate').text == '15'
    assert camera.find('camera/image/width').text == '320'
    assert camera.find('camera/image/height').text == '240'


def test_default_camera_size_and_rate(model):
    """Without arguments the camera is 640 x 480 at 30 Hz with a 69.4 degree field of view."""
    root, _ = model
    camera = next(sensor for sensor in root.iter('sensor') if sensor.get('name') == 'rgbd_camera')
    assert camera.find('update_rate').text == '30'
    assert camera.find('camera/image/width').text == '640'
    assert camera.find('camera/image/height').text == '480'
    assert abs(float(camera.find('camera/horizontal_fov').text) - 1.211) < 1e-9


def test_check_urdf_accepts_the_model(model):
    """check_urdf parses the generated model (skipped when the tool is not installed)."""
    if shutil.which('check_urdf') is None:
        pytest.skip('check_urdf is not on PATH')
    _, text = model
    directory = tempfile.mkdtemp(prefix='rapid_nav_urdf_')
    try:
        path = os.path.join(directory, 'rapid_nav.urdf')
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(text)
        result = subprocess.run(['check_urdf', path], capture_output=True, text=True, check=False)
        assert result.returncode == 0, result.stdout + result.stderr
    finally:
        shutil.rmtree(directory, ignore_errors=True)
