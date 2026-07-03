from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    remappings = [(
                "rgb/image", "/fmu/out/camera_image"
            ), (
                "rgb/camera_info", "/fmu/out/camera_info"
            ), (
                "scan_cloud", "/fmu/out/depth_camera_points_projected"
            ), (
                "depth/image", "/fmu/out/depth_image"
            )
            ]
    
    parameters = [{
        "frame_id": "base_link",
        "subscribe_rgb": True,
        "subscribe_depth" : False,
        "subscribe_scan_cloud" : True,
        "odom_frame_id": "odom",
        "use_sim_time": True,
        "approx_sync": False,
        "sync_queue_size": 30,
        "Grid/Sensor": "0",
        "Grid/RangeMin": "0.2",
        "Grid/RangeMax": "19.1",
        'Rtabmap/DetectionRate': '1.0', 
        "Grid/CellSize": "0.10"
    }]

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable="parameter_bridge",
            arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
            parameters = [{"use_sim_time": True}]

        ),
        Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments=["/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[(
                "/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
                "/fmu/out/camera_image"
            )],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments=["/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
               remappings=[(
                "/world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info",
                "/fmu/out/camera_info"
            )],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments=["/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"],
            remappings=[(
                "/depth_camera/points",
                "/fmu/out/depth_camera_points"
            )],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments=["/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[(
                "/depth_camera",
                "/fmu/out/depth_image"
            )],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments=["/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
            remappings=[(
                "/camera_info",
                "/fmu/out/depth_camera_info"
            )],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package="rtabmap_util",
            executable="point_cloud_xyz",
            remappings=[
                ("depth/image", "/fmu/out/depth_image"),
                ("depth/camera_info", "/fmu/out/depth_camera_info"),
                ("cloud", "/fmu/out/depth_camera_points_projected")
            ],
            parameters=[{
                "use_sim_time": True,
                "decimation": 1,
                "max_depth": 19.1,
                "voxel_size": 0.0
            }]
        ),
        Node(
            package='tf2_transforms',
            executable='publish_odom_to_base_link_enu',
            #arguments = ["--ros-args", "--log-level", "debug"]
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["--x", "0.12", "--y", "0.03", "--z", "0.242", "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "base_link", "--child-frame-id", "camera_link"],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["--x", "0.01233", "--y", "-0.03", "--z", "0.01878",
            "--yaw", "-1.57079632679", "--pitch", "0", "--roll", "-1.57079632679",
            "--frame-id", "camera_link", "--child-frame-id", "camera_optical_frame"],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            remappings=remappings,
            parameters=parameters,
            arguments=["-d"]
        ),
        Node( 
            package='urop_navigation_control',
            executable='keyboard_flight_mode'
        )
    ])
