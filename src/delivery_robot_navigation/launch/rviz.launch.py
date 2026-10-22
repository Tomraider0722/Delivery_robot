import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Open RViz with the Nav2 view (map, costmaps, plan, AMCL particles).

    Use "2D Pose Estimate" to set the start pose for AMCL and "2D Goal Pose"
    to send the robot somewhere; bt_navigator listens on /goal_pose. A small
    speed slider window (speed_control) opens alongside; it publishes
    /speed_limit. Pass speed_slider:=false to leave it out.

    A route window (route_control) also opens: pick RViz's "Publish Point"
    tool once, click several stops, then press Go. Pass route_control:=false
    to leave it out.
    """
    rviz_config = os.path.join(
        get_package_share_directory('delivery_robot_navigation'),
        'rviz',
        'navigation.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    speed_slider = Node(
        package='delivery_robot_navigation',
        executable='speed_control',
        name='speed_control',
        output='screen',
        condition=IfCondition(LaunchConfiguration('speed_slider'))
    )

    route_control = Node(
        package='delivery_robot_navigation',
        executable='route_control',
        name='route_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('route_control'))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'route_control',
            default_value='true',
            description='Also open the multi-point route window'
        ),
        DeclareLaunchArgument(
            'speed_slider',
            default_value='true',
            description='Also open the speed limit slider window'
        ),
        rviz,
        speed_slider,
        route_control,
    ])
