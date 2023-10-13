import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='robot_one' #<--- CHANGE ME

    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [os.path.join(get_package_share_directory(package_name),'launch','rsp.launch.py')]
    #     ), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    # )

    robot_controller_node = Node(
        package="rb_controller",
        executable="robot_controller",
        # parameters=[{'robot_description':robot_description}, controller_params_file]
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': False}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )

    # Launch them all!
    return LaunchDescription([
        robot_controller_node,
        # joystick,
        # twist_mux,
        # ros2_agent,
    ])