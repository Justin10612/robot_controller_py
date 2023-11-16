from setuptools import setup
from glob import glob
import os

package_name = 'rb_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('./launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('./config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sss0301',
    maintainer_email='atom.9031@gmail.com',
    description='Human following robot controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_controller = rb_controller.robot_controller:main',
            'human_follower = rb_controller.human_follow:main',
            'motor_pid_pub = rb_controller.wheel_pid_pub:main'
        ],
    },
)
