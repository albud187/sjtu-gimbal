import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro

from launch_ros.actions import Node
from launch.actions import ExecuteProcess

XACRO_FILE_NAME = "sjtu_drone_multi.urdf.xacro"
R_NS = "drone1"  # or pass via an arg if you want multiple
init_poses = {"drone1": "1.0 1.0 0.0"}  # just an example

def generate_launch_description():
    """Launch Gazebo with a drone that has a gimballed camera, then spawn controllers once ready."""

    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        choices=["true", "false"],
        description="Whether to execute gzclient"
    )

    # Retrieve paths
    pkg_this = get_package_share_directory("sjtu_drone_description")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    
    # Convert XACRO -> URDF
    xacro_file_path = os.path.join(pkg_this, "urdf", XACRO_FILE_NAME)
    drone_ns = R_NS  # "drone1"
    doc = xacro.process_file(xacro_file_path, mappings={"drone_id": drone_ns})
    robot_description = doc.toprettyxml(indent="  ")

    # World file
    world_file = os.path.join(pkg_this, "worlds", "empty_world.world")

    # Gazebo server
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world_file,
            "verbose": "true",
            "extra_gazebo_args": "verbose"
        }.items(),
    )

    # Conditional Gazebo client
    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get("use_gui") == "true":
            return [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
                    ),
                    launch_arguments={"verbose": "true"}.items(),
                )
            ]
        return []

    # Node: robot_state_publisher (namespaced)
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=drone_ns,
        parameters=[
            {
                "frame_prefix": f"{drone_ns}/",
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
        output="screen",
    )

    # Node: joint_state_publisher (optional but included as you had it)
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        namespace=drone_ns,
        output="screen",
    )

    # Node: spawn the drone + camera URDF in Gazebo
    spawn_drone_node = Node(
        package="sjtu_drone_bringup",  # your custom package
        executable="spawn_drone",      # your custom executable
        arguments=[robot_description, drone_ns, init_poses[drone_ns]],
        output="screen",
    )

    # Controller spawners (using the spawner node approach)
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

    # === Event Handlers ===
    # 1) Wait for spawn_drone_node to exit => start the joint_state_broadcaster
    event_spawn_jsp = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_drone_node,
            on_exit=[spawner_joint_state_broadcaster],
        )
    )

    # 2) Wait for joint_state_broadcaster to load => start the joint_trajectory_controller
    event_jsp_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawner_joint_state_broadcaster,
            on_exit=[spawner_joint_trajectory_controller],
        )
    )

    # Now assemble the LaunchDescription
    return LaunchDescription([
        use_gui_arg,

        # Start Gazebo server
        gzserver,

        # Start gzclient if "use_gui" == "true"
        OpaqueFunction(function=launch_gzclient),

        # Robot State Publisher
        rsp_node,

        # Joint State Publisher
        jsp_node,

        # Spawn the drone + gimbal
        spawn_drone_node,

        # Event handlers that sequentially spawn controllers
        event_spawn_jsp,
        event_jsp_jtc,
    ])
