import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import xacro
drone_ns= "drone1"

XACRO_FILE_NAME = "sjtu_drone_multi.urdf.xacro"
pkg_this = get_package_share_directory("sjtu_drone_description")
xacro_file = os.path.join(pkg_this, "urdf", XACRO_FILE_NAME)

doc = xacro.process_file(xacro_file, mappings={"drone_id": drone_ns})
robot_description = doc.toprettyxml(indent="  ")
init_poses = {"drone1": "1.0 1.0 0.0"}

def generate_launch_description():
    bringup_name = 'sjtu_drone_bringup'
    package_name='sjtu_drone_description' 

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(bringup_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', "frame_prefix": f"{drone_ns}/"}.items()
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
             )

    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'my_bot'],
    #                     output='screen')


    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', drone_ns,
            # Pass the namespace for your robot
            '-robot_namespace', drone_ns
        ]
    )
    # spawn_entity = Node(
    #     package="sjtu_drone_bringup",  # your custom package
    #     executable="spawn_drone",      # your custom executable
    #     arguments=[robot_description, drone_ns, init_poses[drone_ns]],
    #     output="screen",
    # )


    spawner_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        namespace=drone_ns,
        arguments=["joint_state_broadcaster", "--controller-manager", f"/{drone_ns}/controller_manager"],
        output="screen",
    )

    spawner_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        namespace=drone_ns,
        arguments=["joint_trajectory_controller", "--controller-manager", f"/{drone_ns}/controller_manager"],
        output="screen",
    )


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
	
    return LaunchDescription([
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawn_entity,
        #         on_exit=[spawner_joint_state_broadcaster],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=spawner_joint_state_broadcaster,
        #         on_exit=[spawner_joint_trajectory_controller],
        #     )
        # ),
        rsp,
        gazebo,
        spawn_entity,
    ])