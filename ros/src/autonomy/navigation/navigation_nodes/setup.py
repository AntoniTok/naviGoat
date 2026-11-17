from pathlib import Path

from setuptools import setup


package_name = 'navigation_nodes'
source_dir = Path(__file__).resolve().parent


def resource_files(*paths):
    return [str(source_dir / path) for path in paths]


setup(
    name=package_name,
    version='0.0.1',
    py_modules=['places_node', 'people_node', 'feedback_node', 'cloud_filter'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation.launch.py', 'launch/navigation_sim.launch.py', 'launch/navigation_hardware.launch.py']),
        ('share/' + package_name + '/config', resource_files('../config/nav2_params.yaml', '../config/navigation_hardware.rviz')),
        ('share/' + package_name + '/resources/autonomy/slam/launch', resource_files('../../slam/launch/mapping.launch.py', '../../slam/launch/slam.launch.py')),
        ('share/' + package_name + '/resources/autonomy/slam/config', resource_files('../../slam/config/rtabmap.yaml', '../../slam/config/rtabmap_hardware.yaml', '../../slam/config/mapping.rviz')),
        ('share/' + package_name + '/resources/bringup/description', resource_files('../../../bringup/description/pioneer3at_lidar.xacro')),
        ('share/' + package_name + '/resources/hardware/camera/launch', resource_files('../../../hardware/camera/launch/camera.launch.py')),
        ('share/' + package_name + '/resources/hardware/camera', resource_files('../../../hardware/camera/camera.py')),
    ],
    install_requires=['setuptools'],
    maintainer='nihil',
    maintainer_email='nihil@users.noreply.github.com',
    description='Drives the robot with Nav2 to places, objects and points, and puts people on the map',
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'places_node = places_node:main',
            'people_node = people_node:main',
            'feedback_node = feedback_node:main',
            'cloud_filter = cloud_filter:main',
        ],
    },
)
