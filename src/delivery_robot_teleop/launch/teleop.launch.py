from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    twist_bridge = Node(
        package='delivery_robot_teleop',
        executable='twist_to_stamped',
        name='twist_to_stamped',
        output='screen'
    )

    teleop_keyboard = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e'
    )

    return LaunchDescription([
        twist_bridge,
        teleop_keyboard
    ])