import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

drone_ns= "drone1"

XACRO_FILE_NAME = "sjtu_drone_multi.urdf.xacro"
pkg_this = get_package_share_directory("sjtu_drone_description")
xacro_file = os.path.join(pkg_this, "urdf", XACRO_FILE_NAME)


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    #pkg_path = os.path.join(get_package_share_directory('sjtu_drone_description'))
    #xacro_file = os.path.join(pkg_path,'description',XACRO_FILE_NAME)
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time, "frame_prefix": f"{drone_ns}/"}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])