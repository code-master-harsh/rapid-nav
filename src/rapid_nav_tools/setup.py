from setuptools import find_packages, setup

package_name = 'rapid_nav_tools'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harsh Vardhan',
    maintainer_email='harsh.v.sharma@outlook.com',
    description='Checks of the RAPID-NAV simulation: sim_check and the pure functions behind it.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_check = rapid_nav_tools.sim_check:main',
        ],
    },
)
