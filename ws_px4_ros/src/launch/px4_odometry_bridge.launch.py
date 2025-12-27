from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_ros_utils',
            executable='convert_odometry',
            name='ConvertOdometryNode',
            output='screen'
        ),
        Node(
            package='drone_control',
            executable='manual_control',
            name='ManualControl',
            output='screen'
        )
    ])
