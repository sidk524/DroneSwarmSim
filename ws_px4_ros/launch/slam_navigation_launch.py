from launch_ros.actions import Node, LifecycleNode
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnShutdown
from launch_ros.event_handlers import OnStateTransition
import os
import subprocess
import time

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
        LoadComposableNodes(
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="depth_image_proc",
                    plugin="depth_image_proc::PointCloudXyzNode",
                    name="point_cloud_xyz",
                    remappings=[ ("image_rect", name + "/stereo/image_raw"),
                        ("points", name + "/points"),
                    ],
                ),
            ],
        ),
    ]


ISAAC_VIO_LAUNCH = "/workspaces/isaac_ros-dev/launch/isaac_ros_vio_launch.py"


def container_running(container):
    result = subprocess.run(
        ["docker", "inspect", "-f", "{{.State.Running}}", container],
        capture_output=True, text=True)
    return result.returncode == 0 and result.stdout.strip() == "true"


# Image the Isaac ROS CLI last resolved; `isaac-ros activate` re-tags it on every run.
ISAAC_IMAGE = "cached_isaac_run_dev_image_local:latest"
ISAAC_CLI_DIR = "/usr/lib/isaac-ros-cli"
started_isaac_container = False


def start_isaac_container(context, *args, **kwargs):
    """Start the Isaac ROS dev container detached if it is not already running.

    `isaac-ros activate` runs `docker run -it` and needs a terminal, so it cannot be used from a
    launch file. This builds the same `docker run` as the CLI's run_dev.py (same mounts, devices,
    GPU and env, taken from its own helpers) but detached (-dit) instead of attached (-it).
    """
    global started_isaac_container
    if LaunchConfiguration("launch_isaac_vio").perform(context).lower() != "true":
        return []
    container = LaunchConfiguration("isaac_container").perform(context)
    if container_running(container):
        return [LogInfo(msg=f"[isaac_vio] using already running container '{container}'")]
    subprocess.run(["docker", "rm", container], capture_output=True)  # stale stopped container, if any

    import sys
    sys.path.insert(0, ISAAC_CLI_DIR)
    import run_dev  # noqa: E402  (Isaac ROS CLI helpers)
    isaac_ws = os.environ.get("ISAAC_ROS_WS", "/home/sidk524/Documents/DroneSwarmSim/isaac_ros_ws")
    platform = "arm64-fastos" if os.path.exists("/etc/fastos-release") else "arm64-jetpack"
    cmd = " ".join(
        ["docker run -dit --rm --privileged --network host --ipc=host",
         "-e TERM=xterm-256color -e COLORTERM=truecolor -e FORCE_COLOR=true",
         "--workdir /workspaces/isaac_ros-dev",
         f"-e ISAAC_ROS_PLATFORM={platform}"]
        + run_dev.get_docker_args(os.uname().machine)
        + run_dev.load_docker_args_from_file()
        + [f"-v {isaac_ws}:/workspaces/isaac_ros-dev",
           "-v /etc/localtime:/etc/localtime:ro",
           f"--name {container}",
           "--gpus all",
           "--entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh",
           ISAAC_IMAGE, "/bin/bash"])
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    if result.returncode != 0:
        return [LogInfo(msg=f"[isaac_vio] ERROR: failed to start container '{container}': {result.stderr.strip()}")]
    started_isaac_container = True
    return [LogInfo(msg=f"[isaac_vio] started container '{container}' from {ISAAC_IMAGE}")]


def wait_for_isaac_container(container, timeout=60.0):
    # The entrypoint creates the `admin` user before the container is usable.
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if container_running(container) and subprocess.run(
                ["docker", "exec", container, "id", "-u", "admin"], capture_output=True).returncode == 0:
            return True
        time.sleep(0.5)
    return False


def launch_isaac_vio(context, *args, **kwargs):
    if LaunchConfiguration("launch_isaac_vio").perform(context).lower() != "true":
        return []
    container = LaunchConfiguration("isaac_container").perform(context)
    if not wait_for_isaac_container(container):
        return [LogInfo(msg=f"[isaac_vio] ERROR: container '{container}' is not ready. "
                            "Isaac VSLAM NOT launched.")]
    return [ExecuteProcess(
        name="isaac_vio",
        cmd=["docker", "exec", "-u", "admin", container, "bash", "-c",
             f"source /opt/ros/jazzy/setup.bash && exec ros2 launch {ISAAC_VIO_LAUNCH}"],
        output="screen",
    )]


def stop_isaac_vio(context, *args, **kwargs):
    if LaunchConfiguration("launch_isaac_vio").perform(context).lower() != "true":
        return []
    container = LaunchConfiguration("isaac_container").perform(context)
    if not container_running(container):
        return []
    launch_pattern = "^/usr/bin/python3 /opt/ros/jazzy/bin/ros2 launch " + ISAAC_VIO_LAUNCH
    node_pattern = "^/opt/ros/jazzy/lib/rclcpp_components/component_container .*__node:=visual_slam_launch_container"
    subprocess.run(["docker", "exec", container, "pkill", "-INT", "-f", launch_pattern])
    deadline = time.monotonic() + 10.0
    while time.monotonic() < deadline:
        alive = [p for p in (launch_pattern, node_pattern)
                 if subprocess.run(["docker", "exec", container, "pgrep", "-f", p], capture_output=True).returncode == 0]
        if not alive:
            break
        time.sleep(0.5)
    else:
        for p in (node_pattern, launch_pattern):
            subprocess.run(["docker", "exec", container, "pkill", "-KILL", "-f", p])
    # Only stop the container if this launch started it (it was run with --rm, so it is removed).
    if started_isaac_container:
        subprocess.run(["docker", "stop", "-t", "5", container], capture_output=True)
    return []


RTABMAP_DB = os.path.expanduser("~/.ros/rtabmap.db")
# EKF2 resets its heading/position when Isaac's vision odometry first arrives; start RTAB-Map
# after that so its first nodes are not recorded with the pre-vision heading.
RTABMAP_SETTLE_S = 3.0


def delete_rtabmap_db(context, *args, **kwargs):
    if os.path.exists(RTABMAP_DB):
        os.remove(RTABMAP_DB)
        return [LogInfo(msg=f"[rtabmap] deleted {RTABMAP_DB}")]
    return []


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
        DeclareLaunchArgument("cam_pitch", default_value="0.516617"),  # 29.6 deg nose-down, from mount CAD
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
        DeclareLaunchArgument("launch_isaac_vio", default_value="true"),
        DeclareLaunchArgument("isaac_container", default_value="isaac_ros_dev_container"),
    ]


    world = "jetty"

    # RTAB-Map runs on the grayscale rectified stereo pair (colour camera is disabled).
    remappings = [(
                "left/image_rect", "/oak/left/image_rect"
            ), (
                "left/camera_info", "/oak/left/camera_info"
            ), (
                "right/image_rect", "/oak/right/image_rect"
            ), (
                "right/camera_info", "/oak/right/camera_info"
            ), (
                "scan_cloud", "/oak/points"
            )
            ]

    parameters = {
        "frame_id": "base_link",
        "subscribe_rgb": False,
        "subscribe_stereo": True,
        "subscribe_depth" : False,
        "subscribe_scan_cloud" : True,

        "odom_frame_id": "odom",
       # "use_sim_time": True,
        "approx_sync": True,
        "sync_queue_size": 30,
        # "topic_queue_size": 10,
        "approx_sync_max_interval": 0.01,
        "Grid/Sensor": "0",
        "Grid/RangeMin": "0.2",
        "Grid/RangeMax": "19.1",
        'Rtabmap/DetectionRate': '4', 
        "Grid/CellSize": "0.10",

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
            parameters=[parameters 
            | {
            "Kp/DetectorStrategy": "8",
            "Vis/FeatureType": "8",
            "GFTT/Gpu": "true",
            "ORB/Gpu": "true",
            "FAST/Gpu": "true",             
            "Vis/CorType": "0",             
            "Vis/CorFlowGpu": "true",
            "Stereo/Gpu": "true",}
            ],
            arguments=["-d"]
        )


    left_rect_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "oak_left_camera_optical_frame",
                "--child-frame-id", "oak_left_rect_optical_frame"],
    )

    right_rect_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x", "0.075", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "oak_left_rect_optical_frame",
                "--child-frame-id", "oak_right_rect_optical_frame"],
    )

    imu_fix_tf = Node(
        package="tf2_ros", executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0",
                "--roll", "1.5707963", "--pitch", "0", "--yaw", "0",
                "--frame-id", "oak_imu_frame",
                "--child-frame-id", "oak_imu_frame_corrected"],
    )

    # Exits when the first Isaac VSLAM odometry message arrives (i.e. Isaac is tracking).
    wait_for_isaac_tracking = ExecuteProcess(
        name="wait_for_isaac_tracking",
        # The type is given explicitly so it waits for the topic to appear instead of exiting.
        cmd=["ros2", "topic", "echo", "--once", "--qos-reliability", "best_effort",
             "/visual_slam/tracking/odometry", "nav_msgs/msg/Odometry", "--field", "header.stamp"],
        output="log",
    )

    return LaunchDescription(
        
        declared_arguments + [OpaqueFunction(function=delete_rtabmap_db),
                              OpaqueFunction(function=start_isaac_container),
                              OpaqueFunction(function=launch_setup)] + 
    
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
        # Node(
        #     package="rtabmap_util",
        #     executable="point_cloud_xyz",
        #     remappings=[
        #         #("depth/image", "/oak"),
        #         ("depth/camera_info", "/oak/stereo/camera_info"),
        #         (
        #         "cloud", "/oak/points"
        #     ), (
        #         "depth/image", "/oak/stereo/image_raw"
        #     )
        #     ],
        #     parameters=[{
        #         "use_sim_time": True,
        #         "decimation": 1,
        #         "max_depth": 19.1,
        #         "voxel_size": 0.0
        #     }]
        # ),
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
        TimerAction(period = 10.0, actions = [
            left_rect_tf,
            right_rect_tf,
            imu_fix_tf,
            slam_ekf_node ,
        
        # Node(
        #     package="rtabmap_odom",
        #     executable="rgbd_odometry",
        #     remappings=remappings,
        #     # arguments=["--udebug"],
        #     # output="screen",
        #     # emulate_tty=True,
        #     parameters=[parameters | {"publish_tf": False, "Odom/ImageDecimation": "1",
        #     "Vis/MaxFeatures": "1000",
        #     "OdomF2M/MaxSize": "1000"
        #     #  , "Vis/DepthAsMask": "false",
        #                             #"OdomF2M/ValidDepthRatio": "0.1",
        #                         #"OdomF2M/BundleUpdateFeatureMapOnAllFrames": "true"
        # } ])

        ]
        ),

        # Isaac VSLAM needs the camera TFs (incl. oak_imu_frame_corrected) at startup,
        # so start it after the TimerAction above.
        TimerAction(period=12.0, actions=[OpaqueFunction(function=launch_isaac_vio)]),
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=stop_isaac_vio)])),

        # RTAB-Map starts only once Isaac is tracking (first odometry message) + EKF2 settle time.
        wait_for_isaac_tracking,
        RegisterEventHandler(OnProcessExit(
            target_action=wait_for_isaac_tracking,
            # Only on a successful exit (message received), not on errors or Ctrl+C.
            on_exit=lambda event, context: [
                LogInfo(msg=f"[rtabmap] Isaac is tracking, starting RTAB-Map in {RTABMAP_SETTLE_S:.0f} s"),
                TimerAction(period=RTABMAP_SETTLE_S, actions=[rtabmap_slam_node]),
            ] if event.returncode == 0 else [
                LogInfo(msg=f"[rtabmap] ERROR: waiting for Isaac odometry failed "
                            f"(exit {event.returncode}), RTAB-Map NOT started"),
            ])),


        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=["--x", "0.12", "--y", "0", "--z", "-0.06", "--yaw", "0", "--pitch", "0.174533", "--roll", "0",
        #     "--frame-id", "base_link", "--child-frame-id", "oak"],
        #     parameters = [{"use_sim_time": True}]
        # ),

        # ]),

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
