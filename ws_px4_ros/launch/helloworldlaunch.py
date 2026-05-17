from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            namespace='bridge',
            executable="parameter_bridge",
            arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"]
        ),
        Node( 
            package='test_node',
            namespace='example',
            executable='test_node'
        )
    ])

