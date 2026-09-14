from launch_ros.actions import Node, LifecycleNode
from launch.actions import RegisterEventHandler, TimerAction
from launch_ros.event_handlers import OnStateTransition
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode



def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver_v3")

    name = LaunchConfiguration("name").perform(context)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "driver.launch.py")
            ),
            launch_arguments={
                "name": name,
                "params_file": params_file,
                "parent_frame": LaunchConfiguration("parent_frame"),
                "cam_pos_x": LaunchConfiguration("cam_pos_x"),
                "cam_pos_y": LaunchConfiguration("cam_pos_y"),
                "cam_pos_z": LaunchConfiguration("cam_pos_z"),
                "cam_roll": LaunchConfiguration("cam_roll"),
                "cam_pitch": LaunchConfiguration("cam_pitch"),
                "cam_yaw": LaunchConfiguration("cam_yaw"),
                "use_rviz": LaunchConfiguration("use_rviz"),
            }.items(),
        ),
        # LoadComposableNodes(
        #     target_container=name + "_container",
        #     composable_node_descriptions=[
        #         ComposableNode(
        #             package="depth_image_proc",
        #             plugin="depth_image_proc::PointCloudXyzNode",
        #             name="point_cloud_xyz",
        #             remappings=[ ("image_rect", name + "/stereo/image_raw"),
        #                 ("points", name + "/points"),
        #             ],
        #         ),
        #     ],
        # ),
    ]


def generate_launch_description():
#     arguments=["--x", "0.12", "--y", "0", "--z", "-0.06", "--yaw", "0", "--pitch", "0.174533", "--roll", "0",

    depthai_prefix = get_package_share_directory("depthai_ros_driver_v3")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument("camera_model", default_value="OAK-D"),
        DeclareLaunchArgument("parent_frame", default_value="base_link"),
        DeclareLaunchArgument("cam_pos_x", default_value="0.12"),
        DeclareLaunchArgument("cam_pos_y", default_value="0.0"),
        DeclareLaunchArgument("cam_pos_z", default_value="-0.06"),
        DeclareLaunchArgument("cam_roll", default_value="0.0"),
        DeclareLaunchArgument("cam_pitch", default_value="0.174533"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument(
           "params_file",
            default_value="/home/sidk524/Documents/DroneSwarmSim/ws_px4_ros/launch/camera_params.yaml",
        ),  
        DeclareLaunchArgument("use_rviz", default_value="False"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(depthai_prefix, "config", "rviz", "rgbd.rviz"),
        ),
        DeclareLaunchArgument("rs_compat", default_value="False"),
    ]


    world = "jetty"

    remappings = [(
                "rgb/image", "/oak/rgb/image_raw"
            ), (
                "rgb/camera_info", "/oak/rgb/camera_info"
            ), (
                "scan_cloud", "/oak/points"
            ), (
                "depth/image", "/oak/stereo/image_raw"
            )
            ]

    parameters = {
        "frame_id": "base_link",
        "subscribe_rgb": True,
        "subscribe_depth" : False,
        "subscribe_scan_cloud" : True,

        "odom_frame_id": "odom",
       # "use_sim_time": True,
        "approx_sync": True,
        "sync_queue_size": 10,
        # "topic_queue_size": 10,
        "approx_sync_max_interval": 0.1,
        "Grid/Sensor": "0",
        "Grid/RangeMin": "0.2",
        "Grid/RangeMax": "19.1",
        'Rtabmap/DetectionRate': '2', 
        "Grid/CellSize": "0.05",

        'fsm/flight_type': 1,              # 1 = /move_base_simple/goal, 2 = preset waypoints
        'fsm/thresh_replan_time': 0.5,
        'fsm/thresh_no_replan_meter': 2.0,
        'fsm/planning_horizon': 7.5,
        'fsm/planning_horizen_time': 3.0,
        'fsm/emergency_time': 1.0,
        'fsm/realworld_experiment': False,
        'fsm/fail_safe': True,

        # 'fsm/waypoint_num': 1,
        # 'fsm/waypoint0_x': 25.0,
        # 'fsm/waypoint0_y': -20.0,
        # 'fsm/waypoint0_z': 4.0,

        'grid_map/resolution': 0.1,
        'grid_map/map_size_x': 70.0,
        'grid_map/map_size_y': 50.0,
        'grid_map/map_size_z': 18.0,
        'grid_map/local_update_range_x': 10.0,
        'grid_map/local_update_range_y': 10.0,
        'grid_map/local_update_range_z': 10.0,
        'grid_map/obstacles_inflation': 0.4,
        'grid_map/local_map_margin': 10,
        'grid_map/ground_height': -0.01,
        'grid_map/virtual_ceil_height': 17.0,
        'grid_map/visualization_truncate_height': 30.0,
        'grid_map/frame_id': 'map',


        # planner limits
        'manager/max_vel': 1.0,
        'manager/max_acc': 2.0,
        'manager/max_jerk': 3.0,
        'manager/control_points_distance': 0.3,
        'manager/feasibility_tolerance': 0.05,
        'manager/planning_horizon': 7.5,
        'manager/use_distinctive_trajs': False,
        'manager/drone_id': 0,

        # optimizers
        'optimization/lambda_smooth': 1.0,
        'optimization/lambda_collision': 1.0,
        'optimization/lambda_feasibility': 0.1,
        'optimization/lambda_fitness': 1.0,
        'optimization/dist0': 0.7,
        'optimization/swarm_clearance': 0.5,
        'optimization/max_vel': 1.0,
        'optimization/max_acc': 2.0,

        'traj_server/time_forward' : 2.0
    }

    ego_remappings=[
        ('odom_world', '/nav_msgs/odom'),
        ('grid_map/odom', '/nav_msgs/odom'),
        ('grid_map/cloud', '/cloud_map'),
        ('planning/bspline', '/drone_0_planning/bspline'),
        ('planning/broadcast_bspline_from_planner', '/broadcast_bspline'),
        ('planning/broadcast_bspline_to_planner', '/broadcast_bspline'),
    ]

    traj_remappings=[
        ('planning/bspline', '/drone_0_planning/bspline')
    ]

    LifecycleAutoNavigationMode = LifecycleNode(
        package = 'urop_navigation_control',
        executable = 'lifecycle_navigation_node',
        name = "lifecycle_navigation_node",
        namespace=""
    )

    ego_planner_node = Node(
        package="ego_planner",
        executable="ego_planner_node",
        remappings=ego_remappings,
        parameters=[parameters]
    )

    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        remappings=traj_remappings,
        parameters=[parameters]
    )

    slam_ekf_node = Node(
        package='urop_navigation_control',
        executable = 'slam_ekf_node',
        parameters = [parameters]
    )

    rtabmap_slam_node = Node(
            package="rtabmap_slam",
            executable="rtabmap",
            remappings=remappings,
            parameters=[parameters],
            arguments=["-d"]
        )

    return LaunchDescription(
        
        declared_arguments + [OpaqueFunction(function=launch_setup)] + 
    
    [
        
        # Node(
        #     package='ros_gz_bridge',
        #     executable="parameter_bridge",
        #     arguments = ["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        #     parameters = [{"use_sim_time": True}]
        # ),
        # Node(
        #     package = "ros_gz_bridge",
        #     executable = "parameter_bridge",
        #     arguments=[f"/world/{world}/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image"],
        #     remappings=[(
        #         f"/world/{world}/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
        #         "/fmu/out/camera_image"
        #     )],
        #     parameters = [{"use_sim_time": True}]
        # ),
        # Node(
        #     package = "ros_gz_bridge",
        #     executable = "parameter_bridge",
        #     arguments=[f"/world/{world}/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        #        remappings=[(
        #         f"/world/{world}/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info",
        #         "/fmu/out/camera_info"
        #     )],
        #     parameters = [{"use_sim_time": True}]
        # ),
        # Node(
        #     package = "ros_gz_bridge",
        #     executable = "parameter_bridge",
        #     arguments=["/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image"],
        #     remappings=[(
        #         "/depth_camera",
        #         "/fmu/out/depth_image"
        #     )],
        #     parameters = [{"use_sim_time": True}]
        # ),
        # Node(
        #     package = "ros_gz_bridge",
        #     executable = "parameter_bridge",
        #     arguments=["/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        #     remappings=[(
        #         "/camera_info",
        #         "/fmu/out/depth_camera_info"
        #     )],
        #     parameters = [{"use_sim_time": True}]
        # ),
        # Node(
        #     package="ros_gz_bridge", 
        #     executable="parameter_bridge",
        #     arguments=[f"/world/{world}/dynamic_pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V"],
        #     remappings=[(f"/world/{world}/dynamic_pose/info", "/ground_truth_poses")],
        #     parameters=[{"use_sim_time": True}],
        # ),
        Node(
            package="rtabmap_util",
            executable="point_cloud_xyz",
            remappings=[
                #("depth/image", "/oak"),
                ("depth/camera_info", "/oak/stereo/camera_info"),
                (
                "cloud", "/oak/points"
            ), (
                "depth/image", "/oak/stereo/image_raw"
            )
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
  
        ),

        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=["--x", "0.01233", "--y", "-0.03", "--z", "0.01878",
        #     "--yaw", "-1.57079632679", "--pitch", "0", "--roll", "-1.57079632679",
        #     "--frame-id", "camera_link", "--child-frame-id", "camera_optical_frame"],
        #     parameters = [{"use_sim_time": True}]
        # ),
        TimerAction(period = 10.0, actions = [rtabmap_slam_node,         
        
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=["--x", "0.12", "--y", "0", "--z", "-0.06", "--yaw", "0", "--pitch", "0.174533", "--roll", "0",
        #     "--frame-id", "base_link", "--child-frame-id", "oak"],
        #     parameters = [{"use_sim_time": True}]
        # ),

        ])
        # Node(
        #     package="rtabmap_odom",
        #     executable="rgbd_odometry",
        #     remappings=remappings,
        #     # arguments=["--udebug"],
        #     # output="screen",
        #     # emulate_tty=True,
        #     parameters=[parameters | {"publish_tf": False, "Odom/ImageDecimation": "1"
        #     # , "Vis/DepthAsMask": "false"
        #     #                         "OdomF2M/ValidDepthRatio": "0.1",
        #     #                         "OdomF2M/BundleUpdateFeatureMapOnAllFrames": "true"
        # }]
        # ),
        # slam_ekf_node,
        # LifecycleAutoNavigationMode,
        # Node(
        #     package = 'urop_navigation_control',
        #     executable='auto_nav_mode_executor'
        # ),
        # RegisterEventHandler(
        #     OnStateTransition(
        #         target_lifecycle_node=LifecycleAutoNavigationMode,
        #         start_state="activating",  
        #         goal_state="active",
        #         entities=[
        #             ego_planner_node,
        #             traj_server_node
        #         ]
        #     )
        # )
    ])
