import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():
    package_name='sjtu_drone_bringup' 

    drone_world_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','sjtu_gimbal_1x_empty_world.launch.py'
                )])
    )

    gimbal_camera_node = Node(
        package="gimbal_control",
        executable="camera_cv_node",
        name="camera_cv_node"
    )

    gimbal_stepper_node = Node(
        package = 'sjtu_drone_control',
        executable="gimbal_stepper",
        name="gimbal_stepper"
    )

    gimbal_controller_node = Node(
        package="gimbal_control",
        executable="gimbal_controller_node",
        name="gimbal_controller_node",
        output='screen',
        prefix='xterm -e'
    )

    teleop_node = Node(
        package='sjtu_drone_control',
        executable='teleop',
        name="teleop",
        output='screen',
        prefix='xterm -e'
    )


    LD = LaunchDescription([
        drone_world_launch,
        gimbal_stepper_node,
        gimbal_camera_node,
        #gimbal_controller_node,
        teleop_node
    ])
	
    return LD
