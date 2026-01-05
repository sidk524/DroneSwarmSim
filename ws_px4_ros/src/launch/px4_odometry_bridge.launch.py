from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': True,

            # Frames
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',

            # LiDAR-only mode
            'subscribe_scan_cloud': True,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_rgbd': False,
            'use_visual_odometry': False,

            # Sync & timing
            'approx_sync': False,
            'queue_size': 10,

            # Mapping
            'Grid/FromDepth': 'False',
            'Grid/FromScan': 'True',
            'Grid/RangeMax': '50.0',

            # ICP-based odometry correction
            'Reg/Strategy': '1',          # 1 = ICP
            'Icp/PointToPlane': 'True',
            'Icp/Iterations': '30',
            'Icp/MaxCorrespondenceDistance': '0.3',
            'Icp/VoxelSize': '0.2',

            # Loop closure
            'RGBD/LoopClosureReextractFeatures': 'False',
            'RGBD/OptimizeFromGraphEnd': 'False',

            'Mem/IncrementalMemory': 'True',
            'Mem/InitWMWithAllNodes': 'False',

            # Performance sanity
            'Odom/ResetCountdown': '0',
            'Odom/Strategy': '0',

        }],
        remappings=[
            ('scan_cloud', '/lidar/points_deskewed'),
            ('odom', '/px4/odom'),
        ]
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        parameters=[{'use_sim_time': True}],
    )

    lidar_points_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_points_bridge',
        output='screen',
        arguments=[
            '/world/baylands/model/x500_lidar_3d_0/link/lidar_link/sensor/gpu_lidar_3d/scan/points'
            '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[
            (
                '/world/baylands/model/x500_lidar_3d_0/link/lidar_link/sensor/gpu_lidar_3d/scan/points',
                '/lidar/points'
            ),
        ],
        parameters=[{'use_sim_time': True}],
    )

    imu_data_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_data_bridge',
        output='screen',
        arguments=[
            '/world/baylands/model/x500_lidar_3d_0/link/base_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        remappings=[
            (
                '/world/baylands/model/x500_lidar_3d_0/link/base_link/sensor/imu_sensor/imu',
                '/imu/data'
            ),
        ],
        parameters=[{'use_sim_time': True}],
    )

    lidar_adapter = Node(
        package='lidar-adapter',
        executable='lidarPointCloudAdapter',
        name='lidar_deskewed_adapter',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    odom_and_tf = [
        Node(
            package='px4_ros_utils',
            executable='convert_odometry',
            name='convert_odometry',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='px4_ros_utils',
            executable='publish_dynamic_tf',
            name='publish_dynamic_tf',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
        Node(
            package='px4_ros_utils',
            executable='publish_static_tf',
            name='publish_static_tf',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ]


    manual_control = Node(
        package='drone_control',
        executable='manual_control',
        name='manual_control',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    delayed_rtabmap = TimerAction(
        period=1.0,
        actions=[rtabmap_node]
    )

    delayed_manual_control = TimerAction(
        period=2.0,
        actions=[manual_control]
    )

    return LaunchDescription([
        clock_bridge,
        lidar_points_bridge,
        lidar_adapter,
        *odom_and_tf,
        delayed_rtabmap,
        delayed_manual_control,
    ])
