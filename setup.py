import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'xbox_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='natthawe',
    maintainer_email='natthawejumjai@gmail.com',
    description='xbox controller for robot',
    license='Apache-2.0',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'xbox_control_node = xbox_control.xbox_control_node:main',
            'xbox_control_debug = xbox_control.xbox_control_debug_node:main',
            'xbox_input = xbox_control.xbox_input:main',
            'xbox_cmdvel = xbox_control.xbox_cmdvel:main',
        ],
    },
)
