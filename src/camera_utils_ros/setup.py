import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'camera_utils_ros'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # marker per l'ament index (obbligatorio)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # installa i launch file -> li trova ros2 launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giorgio Marmolino',
    maintainer_email='giorgio.marmolino@gmail.com',
    description='ROS 2 camera publisher utilities (port from ROS Noetic).',
    license='GPL-3.0-or-later',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # nome eseguibile = quello usato nel launch (executable="camera_publisher")
            'camera_publisher = camera_utils_ros.camera_publisher:main',
        ],
    },
)