from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import math

def generate_launch_description():
    webots_dir = get_package_share_directory('webots_fsai')
    common_dir = get_package_share_directory('fsai_common')

    world = PathJoinSubstitution([webots_dir, "worlds", "simple_trackdrive.wbt"]) #"simple_trackdrive.wbt"
    urdf = PathJoinSubstitution([webots_dir, "resource", "imeche.urdf"])
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    mission = "AUTONOMOUS_DEMO"

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
        launch_arguments={'mission': mission, 'use_sim_time': use_sim_time}.items()
    )

    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([common_dir, 'launch', 'tf_publish.py']),
        ]),
        launch_arguments={'urdf': urdf, 'use_sim_time': use_sim_time}.items()
    )

    delaunay = Node(
        package='fsai_delaunay',
        executable='delaunay',
        name='delaunay',
        #output="screen",
        #parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('/cones', '/webots_driver/cones')],
    )

    breadcrumbs = Node(
        package='fsai_delaunay',
        executable='breadcrumbs',
        name='breadcrumbs',
        output="log"
        #parameters=[{'use_sim_time': use_sim_time}]
    )

    # ros2 run topic_tools transform /steer_angle /ctrl fsai_messages/Control "fsai_messages.msg.Control(steer_angle=m.data, axle_speed_f=1.0, axle_speed_r=1.0, state=fsai_messages.msg.Control.NORMAL_MS)" --import fsai_messages --wait-for-start
    ctrl = Node(
        package="topic_tools",
        executable="transform",
        arguments=["/steer_angle",
                   "/ctrl", "fsai_messages/Control", 
                   """fsai_messages.msg.Control(steer_angle=m.data,
                            axle_speed_f=3.0,
                            axle_speed_r=3.0,
                            state=fsai_messages.msg.Control.NORMAL_MS)""",
                   "--import", "fsai_messages", "--wait-for-start"],
        parameters=[{'use_sim_time': use_sim_time}]
    )   

    return LaunchDescription([
        webots_launch,
        vehicle_launch,
        tf_launch,
        delaunay,
        breadcrumbs,
        ctrl
    ])
