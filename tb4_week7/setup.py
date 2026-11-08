from glob import glob
import os

from setuptools import setup


package_name = 'tb4_week7'

simulation_data_files = []
for source_file in glob(os.path.join('simulation', '**', '*'), recursive=True):
    if not os.path.isfile(source_file):
        continue
    relative_file = os.path.relpath(source_file, 'simulation')
    relative_dir = os.path.dirname(relative_file)
    install_dir = os.path.join('share', package_name, 'simulation', relative_dir)
    simulation_data_files.append((install_dir, [source_file]))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*')),
        (os.path.join('share', package_name, 'maps'),
         glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'),
         glob('rviz/*')),
    ] + simulation_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TurtleBot 4 student',
    maintainer_email='student@example.com',
    description='Semantic map and voice navigation for Small House.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_segmentation_node = tb4_week7.map_segmentation_node:main',
            'initial_pose_publisher = tb4_week7.initial_pose_publisher:main',
            'voice_recognition_node = tb4_week7.voice_recognition_node:main',
            'voice_navigation_node = tb4_week7.voice_navigation_node:main',
            'voice_monitor_node = tb4_week7.voice_monitor_node:main',
            'wheel_drop_state_publisher = tb4_week7.wheel_drop_state_publisher:main',
        ],
    },
)
