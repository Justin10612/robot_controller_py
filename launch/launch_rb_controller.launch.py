import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node

def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='rb_controller' #<--- CHANGE ME

    twist_mux_params = os.path.join(get_package_share_directory(package_name),
                                    'config',
                                    'twist_mux.yaml')

    robot_controller_node = Node(
        package=package_name,
        executable="robot_controller",        
    )

    human_follower_node = Node(
        package=package_name,
        executable="human_follower",
    )
    
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[
                {'use_sim_time': False},
                twist_mux_params],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
    )

    # Launch them all!
    return LaunchDescription([
        robot_controller_node,
        human_follower_node,
        twist_mux,
    ])