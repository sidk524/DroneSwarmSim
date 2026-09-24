import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters = [{
            "tracking_mode": 1,
            "enable_localization_n_mapping": False,
            "publish_map_to_odom_tf": False,
            "publish_odom_to_base_tf": False,
            "camera_optical_frames": ["oak_left_rect_optical_frame", "oak_right_rect_optical_frame"],
            "imu_frame": "oak_imu_frame_corrected",
            "rectified_images": True,
            "sync_matching_threshold_ms": 10.0,
            "calibration_frequency": 200.0,
            "gyro_noise_density": 0.003,
            "gyro_random_walk": 0.0002,
            "accel_noise_density": 0.015,
            "accel_random_walk": 0.005,
            "imu_jitter_threshold_ms": 10.0
        }],
        remappings=[
        ("visual_slam/image_0",       "/oak/left/image_rect"),
        ("visual_slam/camera_info_0", "/oak/left/camera_info"),
        ("visual_slam/image_1",       "/oak/right/image_rect"),
        ("visual_slam/camera_info_1", "/oak/right/camera_info"),
        ("visual_slam/imu",           "/oak/imu/data"),
    ]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[visual_slam_node],
        output='screen',
    )

    return launch.LaunchDescription([visual_slam_launch_container])

