import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro

XACRO_FILE_NAME = "sjtu_drone_gimbal.xacro"
pkg_this = get_package_share_directory("sjtu_drone_description")
xacro_file = os.path.join(pkg_this, "urdf_turret", XACRO_FILE_NAME)
robot_description_config = xacro.process_file(xacro_file)
use_sim_time = True

def generate_launch_description():
    
    
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', ''],
                        output='screen')

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    reg_joint_state_broadcaster = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        )
    
    reg_joint_trajectory = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_joint_trajectory],
            )
        )

    LD = LaunchDescription([
        reg_joint_state_broadcaster,
        reg_joint_trajectory,
        rsp,
        gazebo,
        spawn_entity,
    ])
	
    return LD