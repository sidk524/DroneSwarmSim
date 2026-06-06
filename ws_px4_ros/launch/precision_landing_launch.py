from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable="parameter_bridge",
            arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
        ),
        Node(
            package='ros_gz_bridge',
            executable="parameter_bridge",
            arguments = ["/world/aruco/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image"]
        ),
        Node( 
            package='precision_landing',
            executable='precision_landing_mode_executor',
            output='screen'
        )
    ])

