from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Visual SLAM with RTAB-Map from the RGB-D camera only.

    The wheel odometry and the lidar are not used. rgbd_odometry estimates
    the motion from camera features (published as /vo, frame vo_odom, no TF
    so it does not clash with the wheel odometry TF), and rtabmap builds
    the map from those poses, loop closures and the depth image. The 2D
    occupancy grid is published on /vslam/map so it can run next to the
    lidar SLAM (slam_toolbox publishes /map).

    Needs the simulation running (gazebo.launch.py or bringup.launch.py).
    The map database is written to database_path and is deleted at every
    start.
    """
    rgbd_remaps = [
        ('rgb/image', '/camera/image'),
        ('depth/image', '/camera/depth_image'),
        ('rgb/camera_info', '/camera/camera_info'),
    ]

    # Planar robot: keep the pose estimate on the floor plane.
    common = {
        'frame_id': 'base_link',
        'use_sim_time': True,
        'approx_sync': False,
        'queue_size': 20,
        'Reg/Force3DoF': 'true',
        # The simulated walls have limited texture, so accept fewer matches
        # and extract more features than the defaults (20 and 1000).
        'Vis/MinInliers': '10',
        'Vis/MaxFeatures': '1500',
    }

    visual_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            **common,
            'odom_frame_id': 'vo_odom',
            'publish_tf': False,
            # '' = camera only. 'odom' uses the wheel odometry only as a
            # motion hint so tracking can recover after a bad frame.
            'guess_frame_id': LaunchConfiguration('guess_frame_id'),
        }],
        remappings=rgbd_remaps + [('odom', '/vo')],
    )

    slam = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            **common,
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'database_path': LaunchConfiguration('database_path'),
            'map_frame_id': 'map',
            'publish_tf': True,
            'Optimizer/Slam2D': 'true',
            # 2D occupancy grid built from the depth image
            'Grid/FromDepth': 'true',
            'Grid/RangeMax': '5.0',
            'Grid/CellSize': '0.05',
            'Grid/MaxObstacleHeight': '1.0',
            'Grid/MinGroundHeight': '0.05',
            'Grid/MaxGroundHeight': '0.10',
            'RGBD/ProximityBySpace': 'true',
        }],
        remappings=rgbd_remaps + [('odom', '/vo'), ('map', '/vslam/map')],
        arguments=['--delete_db_on_start'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'guess_frame_id',
            default_value='',
            description="Frame of an odometry TF used as motion guess "
                        "('odom' = wheel odometry); empty = camera only"),
        DeclareLaunchArgument(
            'database_path',
            default_value='/tmp/delivery_robot_rtabmap.db',
            description='Where RTAB-Map stores its map database'),
        visual_odometry,
        slam,
    ])
