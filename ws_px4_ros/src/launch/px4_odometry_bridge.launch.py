from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():

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

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        arguments=[
            '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan'
            '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            (
                '/world/walls/model/x500_lidar_2d_0/link/link/sensor/lidar_2d_v2/scan',
                '/lidar/scan'
            ),
        ],
        parameters=[{'use_sim_time': True}],
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

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'slam_params_file': PathJoinSubstitution([
                FindPackageShare('drone_control'),
                'config',
                'mapper_params_online_async.yaml'
            ]),
            'use_sim_time': 'true'
        }.items()
    )

    delayed_slam = TimerAction(
        period=7.0,   
        actions=[slam_toolbox_launch]
    )

    manual_control = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_control',
                executable='manual_control',
                name='manual_control',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        clock_bridge,
        TimerAction(period=1.0, actions=[lidar_bridge]),
        TimerAction(period=2.0, actions=odom_and_tf),
        delayed_slam,
        manual_control,
    ])
