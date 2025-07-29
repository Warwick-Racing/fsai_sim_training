import os, math
import launch
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    webots_dir = get_package_share_directory('webots_fsai')

    world = PathJoinSubstitution([webots_dir, "worlds", "simple_acceleration.wbt"])
    urdf = PathJoinSubstitution([webots_dir, "resource", "imeche.urdf"])
    use_sim_time = "False"

    # Launch webots simulator
    webots_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([webots_dir, 'launch', 'start_simulator.py'])
        ]),
        launch_arguments={'world': world, 'urdf': urdf, 'use_sim_time': use_sim_time}.items()
    )

    # The the Gui emulation of the manual controls
    manual_control = Node(
        package='webots_fsai',
        executable='gui',
        output='screen'
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

    autonomous_demo = Node(
        package="fsai_common",
        executable="autonomous_demo",
        remappings=[('/ctrl', '/multiplexer/demo_ctrl')]
    )

    diameter_inch = 20      # wheels diameter
    circumference = diameter_inch * 0.0254 * math.pi   
    ticks = 20              # per rot
    tick_distance = circumference / ticks
    track_length = 70       #meters
    acceleration = Node(
        package="fsai_common",
        executable="acceleration_simple",
        parameters=[{
            'tick_distance': int(track_length / tick_distance)
        }],
        remappings=[('/ctrl', '/multiplexer/accel_ctrl')]
    )

    multiplexer = Node(
        package="fsai_common",
        namespace="multiplexer",
        executable="multiplexer",
        remappings=[('/multiplexer/ctrl', '/ctrl'),
                    ('/multiplexer/status', '/status')]
    )

    return LaunchDescription([
        webots_launch,
        manual_control,
        
        multiplexer,

        static_a,
        static_b,
        autonomous_demo,
        acceleration
    ])