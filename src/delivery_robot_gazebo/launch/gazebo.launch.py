from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    description_pkg = get_package_share_directory(
        'delivery_robot_description'
    )

    xacro_file = os.path.join(
        description_pkg,
        'urdf',
        'delivery_robot.urdf.xacro'
    )
    world = os.path.join(
        get_package_share_directory('delivery_robot_gazebo'),
        'worlds',
        'delivery_warehouse.world'
    )

    controllers_file = os.path.join(
        os.path.expanduser('~'),
        'controllers.yaml'
    )

    robot_description = Command([
        'xacro ',
        xacro_file,
        ' controllers_config:=',
        controllers_file
    ])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description
            }
        ],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'delivery_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15'
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
        ],
        output='screen'
    )

    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
        ],
        output='screen'
    )

    load_joint_state_broadcaster = RegisterEventHandler(

        OnProcessExit(

            target_action=spawn_robot,

            on_exit=[
                joint_state_broadcaster
            ]

        )

    )

    load_diff_drive_controller = RegisterEventHandler(

        OnProcessExit(

            target_action=joint_state_broadcaster,

            on_exit=[
                diff_drive_controller
            ]

        )

    )

    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world],
        output='screen'
    )
    
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    scan_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        parameters=[
            {
                'override_frame_id': 'lidar_link'
            }
        ],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        parameters=[
            {
                'override_frame_id': 'camera_optical_frame'
            }
        ],
        output='screen'
    )


    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_robot,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        clock_bridge,
        scan_bridge,
        camera_bridge
    ])