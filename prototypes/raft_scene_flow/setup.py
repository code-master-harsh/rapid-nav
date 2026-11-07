from setuptools import find_packages, setup

package_name = 'optical_flow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/raft_test.launch.py']),
        ('share/' + package_name + '/worlds',
            ['worlds/test_scene.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='Optical flow estimation using pretrained models',
    entry_points={
        'console_scripts': [
            'raft_node = optical_flow.raft_node:main',
        ],
    },
)
