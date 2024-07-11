from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():
    webots_dir = get_package_share_directory('webots_fsai')
    common_dir = get_package_share_directory('fsai_common')

    world = PathJoinSubstitution([webots_dir, "worlds", "simple_trackdrive.wbt"])
    urdf = PathJoinSubstitution([webots_dir, "resource", "imeche.urdf"])
    use_sim_time = "False"
    mission = "SKIDPAD"

    # Launch webots simulator
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([webots_dir, 'launch', 'start_simulator.py'])
        ]),
        launch_arguments={'world': world, 'urdf': urdf, 'use_sim_time': use_sim_time}.items()
    )

    vehicle_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([webots_dir, 'launch', 'start_vehicle.py']),
        ]),
        launch_arguments={'mission': mission}.items()
    )

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([common_dir, 'launch', 'tf_publish.py']),
        ]),
        launch_arguments={'urdf': urdf}.items()
    )

    dr_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([common_dir, 'launch', 'dr_odom.py']),
        ])
    )

    # === control nodes ===
    static_a = Node(
        package="fsai_common",
        executable="static_a",
        remappings=[('/ctrl', '/multiplexer/static_a_ctrl')]
    )

    static_b = Node(
        package="fsai_common",
        executable="static_b",
        remappings=[('/ctrl', '/multiplexer/static_b_ctrl')]
    )

    infinite_loop = ExecuteProcess(cmd=['ros2', 'topic', 'pub', 
            '/multiplexer/skidpad_ctrl', 'fsai_messages/Control', 
            '{steer_angle: -0.2, axle_speed_f: 80.0, axle_speed_r: 80.0}'])  

    multiplexer = Node(
        package="fsai_common",
        namespace="multiplexer",
        executable="multiplexer",
        remappings=[('/multiplexer/ctrl', '/ctrl'),
                    ('/multiplexer/status', '/status')]
    )

    return LaunchDescription([
        webots_launch,
        vehicle_launch,
        tf_launch,
        dr_launch,
        static_a,
        static_b,
        infinite_loop,
        multiplexer
    ])
