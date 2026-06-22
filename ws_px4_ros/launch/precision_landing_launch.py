from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable="parameter_bridge",
            arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        #     remappings = [(
        #         "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image",
        #         "/fmu/out/camera_image"
        # )]
        ),
        Node(
            package='ros_gz_bridge',
            executable="parameter_bridge",
            arguments = ["/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings = [(
                "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image",
                "/fmu/out/camera_image"
        )]
        ),
        Node(
            package='ros_gz_bridge',
            executable="parameter_bridge",
            arguments = ["/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        remappings = [( 
                "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image",
                "/fmu/out/camera_image"
        ),(
            "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/camera_info",
            "/camera_info"
        )]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["--x", "0", "--y", "0", "--z", "0", "--yaw", "0", "--pitch", "0", "--roll", "0",
                       "--frame-id", "map_ned", "--child-frame-id", "odom"]
        ),
        Node(
            package='tf2_transforms',
            executable='publish_map_ned'
        ),
        Node(
            package='tf2_transforms',
            executable='publish_odom_to_base_link',
            arguments = ["--ros-args", "--log-level", "debug"]
        ),
        Node( 
            package='precision_landing',
            executable='precision_landing_mode_executor',
            output='screen',
            remappings = [(
                "/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image",
                "/fmu/out/camera_image"
        )],
        #prefix=['gdb -ex run --args']
        )
    ])


