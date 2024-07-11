from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    webots_dir = get_package_share_directory('webots_fsai')

    world = PathJoinSubstitution([webots_dir, "worlds", "simple_trackdrive.wbt"])
    urdf = PathJoinSubstitution([webots_dir, "resource", "imeche.urdf"])
    use_sim_time = "False"

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
        launch_arguments={'mission': 'STATIC_INSPECTION_A'}.items()
    )

    static_a = Node(
        package="fsai_common",
        executable="static_a",
        output="screen"
    )

    return LaunchDescription([
        webots_launch,
        vehicle_launch,
        static_a,
    ])