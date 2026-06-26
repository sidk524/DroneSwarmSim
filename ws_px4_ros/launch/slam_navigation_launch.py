from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    remappings = [(
                "rgb/image", "/fmu/out/camera_image"
            ), (
                "rgb/camera_info", "/fmu/out/camera_info"
            ) , (
                "scan_cloud", "/fmu/out/depth_camera_points"
            ), (
                "depth/image", "/fmu/out/depth_image"
            )
            ]
    
    parameters = [{
        "subscribe_scan_cloud" : False,
        "odom_frame_id": "odom",
        "use_sim_time": True,
        "approx_sync": True,
        "approx_sync_max_interval": 1, 
        "queue_size": 30,            
    }]

    return LaunchDescription([
        Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments=["/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[(
                "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
                "/fmu/out/camera_image"
            )],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments=["/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
               remappings=[(
                "/world/baylands/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info",
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
            package='tf2_transforms',
            executable='publish_odom_to_base_link',
            #arguments = ["--ros-args", "--log-level", "debug"]
            parameters = [{"use_sim_time": True}]

        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["--x", ".12", "--y", ".03", "--z", ".242", "--yaw", "0", "--pitch", "0", "--roll", "0",
            "--frame-id", "base_link", "--child-frame-id", "camera_link"],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            remappings=remappings,
            parameters=parameters,
            arguments=["-d"]
        )
    ])