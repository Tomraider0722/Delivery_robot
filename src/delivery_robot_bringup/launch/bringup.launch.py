from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

# Variables set by the VS Code snap that make Gazebo / RViz load snap
# libraries and crash (the same ones run_sim.sh removes).
SNAP_ENV_VARS = [
    'GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR', 'LOCPATH',
    'GDK_PIXBUF_MODULE_FILE', 'GDK_PIXBUF_MODULEDIR', 'GTK_IM_MODULE_FILE',
    'GSETTINGS_SCHEMA_DIR', 'XDG_DATA_DIRS',
]


def launch_file(package, name):
    """Return a launch description source for package/launch/name."""
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), 'launch', name]))


def generate_launch_description():
    """Start the whole delivery robot stack with one command.

    Order: Gazebo + robot + controllers; once the robot's odometry is
    publishing, localization on the saved map (or SLAM); once /map exists,
    Nav2; a few seconds later RViz with the speed slider and route window.
    Each stage waits for the previous one to be ready instead of using a
    fixed timer, because starting Nav2 or AMCL while Gazebo is still loading
    can make their lifecycle bring-up time out.

        ros2 launch delivery_robot_bringup bringup.launch.py
        ros2 launch delivery_robot_bringup bringup.launch.py mode:=slam
        ros2 launch delivery_robot_bringup bringup.launch.py rviz:=false
    """
    mode = LaunchConfiguration('mode')
    rviz = LaunchConfiguration('rviz')
    speed_slider = LaunchConfiguration('speed_slider')
    teleop = LaunchConfiguration('teleop')

    args = [
        DeclareLaunchArgument(
            'mode', default_value='localization',
            description="'localization' (saved map + AMCL) or 'slam' (build "
                        'a new map)'),
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Open RViz'),
        DeclareLaunchArgument(
            'speed_slider', default_value='true',
            description='Open the speed slider with RViz'),
        DeclareLaunchArgument(
            'teleop', default_value='false',
            description='Also open the keyboard teleop terminal'),
        DeclareLaunchArgument(
            'rviz_delay', default_value='8.0',
            description='Seconds after Nav2 starts before RViz opens'),
    ]

    simulation = IncludeLaunchDescription(
        launch_file('delivery_robot_gazebo', 'gazebo.launch.py'))

    # Teleop starts its own twist_to_stamped, so Nav2 only needs to start one
    # when teleop is off.
    teleop_launch = IncludeLaunchDescription(
        launch_file('delivery_robot_teleop', 'teleop.launch.py'),
        condition=IfCondition(teleop))

    def wait_for(topic, msg_type, *extra):
        """Process that exits once a message arrives on the topic.

        The message type is given so that ros2 topic echo waits for the topic
        instead of exiting when nothing is publishing yet.
        """
        return ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '--once', topic, msg_type, *extra],
            output='log')

    # Gate 1: the diff_drive_controller is active and publishing odometry.
    wait_sim = wait_for(
        '/diff_drive_controller/odom', 'nav_msgs/msg/Odometry')
    # Gate 2: /map is available (map_server or SLAM is active).
    wait_map = wait_for(
        '/map', 'nav_msgs/msg/OccupancyGrid',
        '--qos-durability', 'transient_local',
        '--qos-reliability', 'reliable')

    localization = [
        IncludeLaunchDescription(
            launch_file('delivery_robot_navigation',
                        'localization.launch.py'),
            condition=IfCondition(EqualsSubstitution(mode, 'localization'))),
        IncludeLaunchDescription(
            launch_file('delivery_robot_navigation', 'slam.launch.py'),
            condition=IfCondition(EqualsSubstitution(mode, 'slam'))),
        wait_map,
    ]

    navigation = [
        IncludeLaunchDescription(
            launch_file('delivery_robot_navigation', 'navigation.launch.py'),
            launch_arguments={'start_twist_bridge': 'true'}.items(),
            condition=UnlessCondition(teleop)),
        IncludeLaunchDescription(
            launch_file('delivery_robot_navigation', 'navigation.launch.py'),
            launch_arguments={'start_twist_bridge': 'false'}.items(),
            condition=IfCondition(teleop)),
        TimerAction(
            period=LaunchConfiguration('rviz_delay'),
            actions=[
                IncludeLaunchDescription(
                    launch_file('delivery_robot_navigation',
                                'rviz.launch.py'),
                    launch_arguments={
                        'speed_slider': speed_slider}.items(),
                    condition=IfCondition(rviz)),
            ]),
    ]

    start_localization = RegisterEventHandler(
        OnProcessExit(target_action=wait_sim, on_exit=localization))
    start_navigation = RegisterEventHandler(
        OnProcessExit(target_action=wait_map, on_exit=navigation))

    stack = GroupAction([
        *[UnsetEnvironmentVariable(name) for name in SNAP_ENV_VARS],
        simulation,
        teleop_launch,
        wait_sim,
        start_localization,
        start_navigation,
    ])

    return LaunchDescription([*args, stack])
