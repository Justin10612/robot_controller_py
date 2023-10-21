import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='rb_controller' #<--- CHANGE ME

    human_follower_node = Node(
        package=package_name,
        executable="human_follower",
    )

    # Launch them all!
    return LaunchDescription([
        human_follower_node,
    ])