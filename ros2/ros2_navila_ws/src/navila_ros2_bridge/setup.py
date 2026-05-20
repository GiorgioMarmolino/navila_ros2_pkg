from setuptools import setup
from glob import glob
import os

# Nome del modulo Python (con underscore)
package_name = 'navila_ros2_bridge'

setup(
    name='navila-ros2-bridge',
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Giorgio Marmolino',
    maintainer_email='giorgio.marmolino@gmail.com',
    description='ROS 2 bridge between NaVILA VLA model and Husky robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'navila_node = navila_ros2_bridge.navila_node:main',
            'action_to_cmdvel_node = navila_ros2_bridge.action_to_cmdvel_node:main',
            'instruction_node = navila_ros2_bridge.instruction_node:main',
        ],
    },
)