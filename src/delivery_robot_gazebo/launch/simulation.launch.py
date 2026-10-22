from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("delivery_robot_gazebo"),
                "launch",
                "gazebo.launch.py"
            ])
        )
    )

    teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("delivery_robot_teleop"),
                "launch",
                "teleop.launch.py"
            ])
        )
    )

    return LaunchDescription([
        gazebo,
        teleop,
    ])