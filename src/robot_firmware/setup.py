from setuptools import setup
import os
from glob import glob
package_name = 'robot_firmware'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # 1. This fixes warning #1 (Registers the marker in the package index)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
            
        # 2. This fixes warning #2 (Installs the package.xml file)
        ('share/' + package_name, ['package.xml']),
        
        # Keeps any other file setups you might have (like launch files)
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abhi',
    maintainer_email='abhi@todo.todo',
    description='Robot Firmware',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_serial = robot_firmware.imu_serial:main',
            'led_control = robot_firmware.led_control:main' ,
            'motor_serial = robot_firmware.motor_serial:main' ,
        ],
    },
)