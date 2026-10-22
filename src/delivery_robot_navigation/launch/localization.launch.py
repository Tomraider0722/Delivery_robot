import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Localize the robot on the saved map with map_server and AMCL.

    Use this instead of slam.launch.py once a map exists. It provides /map
    and the map->odom transform, so navigation.launch.py works unchanged.
    The simulated robot spawns at the map origin; for another start pose,
    set one in RViz ("2D Pose Estimate") or in config/nav2_params.yaml.
    """
    nav_pkg = get_package_share_directory('delivery_robot_navigation')
    params_file = os.path.join(nav_pkg, 'config', 'nav2_params.yaml')
    default_map = os.path.join(nav_pkg, 'maps', 'warehouse_map.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to the map yaml file'
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {'yaml_filename': LaunchConfiguration('map')},
        ]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }]
    )

    return LaunchDescription([
        map_arg,
        map_server,
        amcl,
        lifecycle_manager,
    ])
