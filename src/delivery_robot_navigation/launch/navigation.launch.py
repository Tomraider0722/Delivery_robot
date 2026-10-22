import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start Nav2 (planner, controller, behaviours, BT) for the robot.

    Needs the simulation and slam.launch.py running: SLAM provides /map and
    the map->odom transform. Nav2 publishes Twist on /cmd_vel, which
    twist_to_stamped converts for the diff_drive_controller. The teleop
    launch (and so simulation.launch.py) already starts that converter, so
    only enable start_twist_bridge when it is not running.
    """
    params_file = os.path.join(
        get_package_share_directory('delivery_robot_navigation'),
        'config',
        'nav2_params.yaml'
    )

    start_twist_bridge = LaunchConfiguration('start_twist_bridge')

    twist_bridge = Node(
        package='delivery_robot_teleop',
        executable='twist_to_stamped',
        name='twist_to_stamped',
        output='screen',
        condition=IfCondition(start_twist_bridge)
    )

    nav_nodes = [
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file]
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file]
        ),
    ]

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
            ],
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_twist_bridge',
            default_value='false',
            description='Also start twist_to_stamped (not needed with '
                        'simulation.launch.py)'
        ),
        twist_bridge,
        *nav_nodes,
        lifecycle_manager,
    ])
