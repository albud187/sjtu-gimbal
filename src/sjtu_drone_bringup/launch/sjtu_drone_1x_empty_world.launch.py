import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
import xacro

XACRO_FILE_NAME = "sjtu_drone_multi.urdf.xacro"
XACRO_FILE_PATH = os.path.join(get_package_share_directory("sjtu_drone_description"),"urdf", XACRO_FILE_NAME)
R_NS = ["drone0","drone1"]
init_poses = {R_NS[1]:"1.0 1.0 0.0"}

def generate_launch_description():

    controller_config = os.path.join(
        get_package_share_directory('sjtu_drone_description'),
        'config',
        'ros2_controllers.yaml'
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    use_gui = DeclareLaunchArgument("use_gui", default_value="true", choices=["true", "false"], description="Whether to execute gzclient")
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    
    r_1_doc = xacro.process_file(XACRO_FILE_PATH, mappings = {"drone_id":R_NS[1]})
    r_1_desc = r_1_doc.toprettyxml(indent='  ')


    world_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "worlds", "empty_world.world"
    )

    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get('use_gui') == 'true':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true'}.items()
            )]
        return []


    return LaunchDescription([
        use_gui,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=R_NS[1],
            parameters=[{'frame_prefix': R_NS[1]+'/','use_sim_time': use_sim_time, 'robot_description': r_1_desc}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name=R_NS[1]+"_" +'joint_state_publisher',
            namespace=R_NS[1],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'/drone1/robot_description': r_1_desc}, controller_config],
            #parameters=[controller_config],
            #remappings=[('/robot_description', '/drone1/robot_description')],
            namespace=R_NS[1],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '--controller-manager', f'/{R_NS[1]}/controller_manager'],
            namespace=R_NS[1],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['front_cam_joint_position_controller', '--controller-manager', f'/{R_NS[1]}/controller_manager'],
            namespace=R_NS[1],
            output='screen',
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file,
                              'verbose': "true",
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        OpaqueFunction(function=launch_gzclient),

        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone",
            arguments=[r_1_desc, R_NS[1], init_poses[R_NS[1]]],
            output="screen"
        )
    ])