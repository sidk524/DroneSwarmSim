from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition


def generate_launch_description():

    # Must match the Gazebo world PX4 is launched with (PX4_GZ_WORLD / make target)
    world = "obstacle_course"

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
        'Rtabmap/DetectionRate': '2', 
        "Grid/CellSize": "0.10",

        'fsm/flight_type': 2,              # 1 = /move_base_simple/goal, 2 = preset waypoints
        'fsm/thresh_replan_time': 0.5,
        'fsm/thresh_no_replan_meter': 2.0,
        'fsm/planning_horizon': 7.5,
        'fsm/planning_horizen_time': 3.0,
        'fsm/emergency_time': 1.0,
        'fsm/realworld_experiment': False,
        'fsm/fail_safe': True,

        'fsm/waypoint_num': 1,
        'fsm/waypoint0_x': 45.0,
        'fsm/waypoint0_y': 0.0,
        'fsm/waypoint0_z': 1.5,

        'grid_map/resolution': 0.1,
        'grid_map/map_size_x': 100.0,
        'grid_map/map_size_y': 20.0,
        'grid_map/map_size_z': 10.0,
        'grid_map/local_update_range_x': 10.0,
        'grid_map/local_update_range_y': 10.0,
        'grid_map/local_update_range_z': 10.0,
        'grid_map/obstacles_inflation': 0.4,
        'grid_map/local_map_margin': 10,
        'grid_map/ground_height': -0.01,
        'grid_map/virtual_ceil_height': 4.0,
        'grid_map/visualization_truncate_height': 30.0,
        'grid_map/frame_id': 'map',


        # planner limits
        'manager/max_vel': 2.0,
        'manager/max_acc': 4.0,
        'manager/max_jerk': 4.0,
        'manager/control_points_distance': 0.3,
        'manager/feasibility_tolerance': 0.05,
        'manager/planning_horizon': 7.5,
        'manager/use_distinctive_trajs': False,
        'manager/drone_id': 0,


        # optimizers
        'optimization/lambda_smooth': 2.0,
        'optimization/lambda_collision': 1.0,
        'optimization/lambda_feasibility': 0.1,
        'optimization/lambda_fitness': 1.0,
        'optimization/dist0': 0.7,
        'optimization/swarm_clearance': 0.5,
        'optimization/max_vel': 2.0,
        'optimization/max_acc': 4.0,

        'traj_server/time_forward' : 2.0
    }]

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
        parameters=parameters
    )

    traj_server_node = Node(
        package="ego_planner",
        executable="traj_server",
        remappings=traj_remappings,
        parameters=parameters
    )


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
            arguments=[f"/world/{world}/model/x500_depth_0/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[(
                f"/world/{world}/model/x500_depth_0/link/camera_link/sensor/IMX214/image",
                "/fmu/out/camera_image"
            )],
            parameters = [{"use_sim_time": True}]
        ),
        Node(
            package = "ros_gz_bridge",
            executable = "parameter_bridge",
            arguments=[f"/world/{world}/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
               remappings=[(
                f"/world/{world}/model/x500_depth_0/link/camera_link/sensor/IMX214/camera_info",
                "/fmu/out/camera_info"
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
        ), 
        LifecycleAutoNavigationMode,
        Node(
            package='urop_navigation_control',
            executable='autonomous_navigation_mode'
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=LifecycleAutoNavigationMode,
                start_state="activating",  
                goal_state="active",
                entities=[
                    ego_planner_node,
                    traj_server_node
                ]
            )
        )
    ])
