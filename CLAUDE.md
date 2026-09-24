# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A PX4 + ROS 2 (Jazzy) research stack for autonomous drone navigation and precision landing, running on a **real, flying quadcopter**. The hardware is a Pixhawk flight controller running the PX4 fork, with a Jetson companion computer on a 3D-printed mount (aarch64, L4T/tegra kernel) and a Luxonis OAK-D camera. The Jetson runs the ROS 2 stack and talks to the Pixhawk through the uXRCE-DDS agent. PX4 SITL with Gazebo is still used for development and testing.

This code flies real hardware. Changes to flight modes, setpoints, odometry fed to EKF2 (`/fmu/in/*`), frame conversions, and failsafe behaviour can crash the vehicle. Be conservative in those areas and call out flight-safety implications of any change.

Most of the tree is vendored upstream code (git submodules or copied repos). The project's own code is a small set of ROS 2 packages in `ws_px4_ros/src`.

## Layout

- `px4_ws/PX4-Autopilot`: a submodule of the user's PX4 fork (`sidk524/PX4-Autopilot`, branch `droneswarm`). It has its own `CLAUDE.md`, which applies when you edit firmware there (including its "no Claude attribution" rule for that repo).
- `px4_ws/Micro-XRCE-DDS-Agent`: the uXRCE-DDS bridge between PX4 uORB and ROS 2 `/fmu/in/*` and `/fmu/out/*` topics.
- `ws_px4_ros/`: the colcon workspace. `ws_px4_ros/launch/` holds the project launch files, which are run by path rather than installed from a package.
- `isaac_ros_ws/`: the workspace for NVIDIA Isaac ROS Visual SLAM (cuVSLAM). `$ISAAC_ROS_WS` points here (set in `~/.zshrc`). The Isaac ROS packages were recently moved out of `ws_px4_ros/src`.
- `opencv/`, `opencv_contrib/`: OpenCV source, built locally (CUDA-enabled, for GPU features in RTAB-Map and ArUco).
- `tools/gen_canopy_world.py`: generates a Gazebo rainforest-canopy SDF world for SITL.

### Project-owned ROS 2 packages (`ws_px4_ros/src`)

| Package | Role |
|---|---|
| `urop_navigation_control` | PX4 external flight modes (`px4_ros2::ModeBase` / `ModeExecutorBase` from `px4-ros2-interface-lib`): ascend, autonomous navigation (follows ego-planner `/position_cmd`), keyboard flight, lifecycle navigation node. Also `slam_ekf_node`, which feeds SLAM odometry to PX4 EKF2. |
| `precision_landing` | Mode executor chaining InitialFlyUp → LocateMarker → MoveAboveMarker → Descend, driven by `/aruco_marker_position`. |
| `tf2_transforms` | TF publishers for map/odom/base_link/camera-optical frames in both NED and ENU variants, plus ArUco detection (`publish_aruco_marker`, OpenCV) on `/fmu/out/camera_image`. |
| `px4_ros_utils` | Converts `px4_msgs/VehicleOdometry` to `nav_msgs/Odometry` (`/px4/odom`) and publishes static and dynamic TF. |
| `oakd_camera` | OAK-D pipeline setup via depthai-core. |
| `my_msgs` | Custom messages (`TvecRvec`). |
| `test_node` | Scratch nodes (hello world, fly in circle). |

Vendored packages in `src/` include `ego-planner-swarm` (ROS 2 port; provides `ego_planner`, `traj_server`, `quadrotor_msgs`), `rtabmap` + `rtabmap_ros`, `depthai-core` + `depthai-ros` (the driver package is `depthai_ros_driver_v3`), `px4_msgs`, `px4_ros_com`, `px4-ros2-interface-lib`, `image_pipeline`, `vision_opencv`, `octomap_*`, and `navigation2`. `navigation2` and `RACER` have `COLCON_IGNORE`. `ws_px4_ros/setup/` (Python calibration and viewer scripts plus the `depthai-env` venv) is also ignored by colcon.

## Data flow (big picture)

1. **Perception:** the OAK-D publishes `/oak/rgb/*`, `/oak/stereo/image_raw`, `/oak/left|right/image_rect`, and `/oak/imu/data`. `depth_image_proc::PointCloudXyzNode` is loaded into the depthai container to produce `/oak/points`.
2. **SLAM / VIO:** either RTAB-Map (`slam_navigation_launch.py`, GPU feature flags on) or Isaac ROS Visual SLAM (`isaac_ros_ws/launch/isaac_ros_vio_launch.py`, stereo + IMU, rectified images, frames `oak_left_rect_optical_frame` / `oak_right_rect_optical_frame`, which `slam_navigation_launch.py` publishes as static TFs).
3. **Fusion into PX4:** `slam_ekf_node` subscribes to `/visual_slam/tracking/odometry` and publishes `px4_msgs/VehicleOdometry` to `/fmu/in/vehicle_visual_odometry`. ROS data is ENU/FLU and PX4 is NED/FRD, so conversions go through `px4_ros2::*EnuToNed` helpers. In sim only, it also compares against `/ground_truth_poses` and publishes the error on `/pythagDistance`; on hardware there is no ground-truth topic.
4. **Planning and control:** ego-planner consumes odometry and `/cloud_map`, and publishes B-splines, which `traj_server` turns into `/position_cmd`. The `AutonomousNavigationMode` PX4 external mode tracks `/position_cmd` and sends goals on `/move_base_simple/goal`.
5. **Sim only, not present on the real drone:** `ros_gz_bridge` remaps Gazebo camera topics to `/fmu/out/camera_image` and `/camera_info`, and the dynamic pose to `/ground_truth_poses`. World names (`aruco`, `jetty`, `canopy`) appear in topic paths.

Frame conventions matter everywhere. Many nodes come in `_ned` and `_enu` pairs; check which one a launch file uses before changing TF code.

## Commands

Build (from `ws_px4_ros`, ROS 2 Jazzy sourced). The existing build is `CMAKE_BUILD_TYPE=Debug` and `compile_commands.json` is exported:
```bash
cd ws_px4_ros && source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select urop_navigation_control   # single package
colcon build --symlink-install --packages-up-to precision_landing
source install/setup.zsh
```
A full workspace build is very slow on the Jetson (rtabmap, depthai-core, and image_pipeline are heavy), so prefer `--packages-select`.

Isaac ROS VSLAM:
```bash
cd ${ISAAC_ROS_WS} && colcon build --symlink-install --packages-up-to isaac_ros_visual_slam
ros2 launch isaac_ros_ws/launch/isaac_ros_vio_launch.py
```

Launch (run by path):
```bash
ros2 launch ws_px4_ros/launch/slam_navigation_launch.py      # OAK-D + RTAB-Map + slam_ekf_node (+ ego-planner, commented out)
ros2 launch ws_px4_ros/launch/precision_landing_launch.py    # SITL aruco world, gz bridges, TF, landing executor
ros2 launch ws_px4_ros/launch/oakd_point_cloud_launch.py
```
`slam_navigation_launch.py` hard-codes `params_file` to an absolute path, `ws_px4_ros/launch/camera_params.yaml`.

PX4 SITL and the DDS agent (in sim the agent uses UDP; on the real drone it connects to the Pixhawk over the Jetson↔Pixhawk link instead):
```bash
cd px4_ws/PX4-Autopilot && PX4_GZ_WORLD=<world> make px4_sitl gz_x500_depth   # or gz_x500_mono_cam_down for aruco
MicroXRCEAgent udp4 -p 8888   # SITL
python3 tools/gen_canopy_world.py --seed 42 --out px4_ws/PX4-Autopilot/Tools/simulation/gz/worlds/canopy.sdf
```

DDS: `ws_px4_ros/dds/large_dds_profile.xml` is a Fast DDS profile with a large SHM transport for image and point-cloud traffic. The shell uses `FASTRTPS_DEFAULT_PROFILES_FILE=/etc/fastdds/profile.xml`. The ego-planner README recommends CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) because of lag under Fast DDS.

There are no project tests or linters beyond the `ament_lint_auto` stubs in the package CMakeLists.

## Notes

- `.vscode/settings.json` points the CMake source dir at `urop_navigation_control`.
- Much iteration happens by commenting nodes in and out of launch files, so read the whole launch file to see what is actually active.
- `build/`, `install/`, `log/`, and `bags/` are gitignored. Stray files like `frames_*.pdf/.gv` come from `tf2_tools view_frames`.
