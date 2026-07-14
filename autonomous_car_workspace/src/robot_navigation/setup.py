import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yml') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zishan',
    maintainer_email='mdshihabulislam.mte.ruet@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_bridge = robot_navigation.serial_bridge_node:main',
            'brain = robot_navigation.brain_node:main',
            'lidar_monitor = robot_navigation.lidar_monitor:main',
            'vision = robot_navigation.vision_node:main',
            'sign_detector = robot_navigation.sign_detector_node:main',
        ],
    },
)
