# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

DroneSwarmSim is a PX4-based drone simulation environment that integrates PX4 with ROS 2 for drone control. The project uses Gazebo for simulation and enables offboard control of drones through ROS 2 nodes.

## Repository Structure

The repository is organized into two main workspaces:

### px4_ws/ - PX4 Workspace
- **Micro-XRCE-DDS-Agent/**: DDS-XRCE bridge for PX4-ROS2 communication (submodule)

### ws_px4_ros/ - ROS 2 Workspace
Main ROS 2 workspace containing:
- **px4_msgs/**: PX4 message definitions for ROS 2 (submodule from PX4/px4_msgs)
- **px4_ros_com/**: PX4-ROS2 communication library (submodule from PX4/px4_ros_com)
  - Provides example nodes: listeners, advertisers, offboard control
  - Includes frame_transforms library for coordinate conversions
- **drone_control/**: Custom package for manual drone control
  - Contains `manual_control.cpp` for manual control via ROS 2
- **test_node/**: Custom testing package
  - Contains `test_node.cpp` for offboard position control testing

## Build System

### Building ROS 2 Workspace

```bash
cd ws_px4_ros
colcon build
source install/setup.bash
```

To build specific packages:
```bash
colcon build --packages-select drone_control test_node
```

## Running the System

### 1. Start Micro-XRCE-DDS Agent
In a separate terminal:
```bash
cd px4_ws/Micro-XRCE-DDS-Agent
./build/MicroXRCEAgent udp4 -p 8888
```

### 2. Run ROS 2 Nodes
In a separate terminal (after sourcing the workspace):
```bash
cd ws_px4_ros
source install/setup.bash

# Run test node for position control
ros2 run test_node test_node

# Or run manual control
ros2 run drone_control drone_control

# Or run example offboard control
ros2 run px4_ros_com offboard_control
```

## Key Concepts

### PX4-ROS2 Communication Architecture

The system uses a layered communication architecture:
1. **PX4 SITL** runs the flight controller in software-in-the-loop mode
2. **Micro-XRCE-DDS Agent** bridges PX4's uXRCE-DDS to ROS 2 DDS
3. **ROS 2 Nodes** communicate with PX4 via `/fmu/in/*` and `/fmu/out/*` topics

### Important Topic Namespaces

- `/fmu/in/vehicle_command`: Send commands to drone (arm, disarm, mode changes)
- `/fmu/in/offboard_control_mode`: Set offboard control mode flags
- `/fmu/in/trajectory_setpoint`: Send position/velocity setpoints
- `/fmu/in/manual_control_input`: Send manual control inputs
- `/fmu/out/vehicle_local_position`: Receive current drone position
- `/fmu/out/vehicle_status`: Receive drone status

### QoS Profile Requirements

PX4 topics require sensor data QoS profile:
```cpp
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth),
    qos_profile
);
```

### Offboard Control Pattern

To control a drone in offboard mode:
1. Publish `OffboardControlMode` messages continuously (min 2Hz)
2. Send `VEHICLE_CMD_DO_SET_MODE` (mode 1, submode 6) to enter offboard
3. Send `VEHICLE_CMD_COMPONENT_ARM_DISARM` to arm the drone
4. Publish trajectory setpoints or manual control inputs

This pattern is implemented in `test_node` and `offboard_control` examples.

### Timestamp Synchronization

PX4 messages require timestamps in microseconds:
```cpp
msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
```

## Development Workflow

### Adding New Control Nodes

1. Create a new package in `ws_px4_ros/src/`:
   ```bash
   cd ws_px4_ros/src
   ros2 pkg create --build-type ament_cmake my_package --dependencies rclcpp px4_msgs
   ```

2. Add your control logic following the offboard control pattern
3. Build and test:
   ```bash
   cd ws_px4_ros
   colcon build --packages-select my_package
   source install/setup.bash
   ros2 run my_package my_node
   ```

### Working with PX4 Messages

All PX4 message types are defined in `ws_px4_ros/src/px4_msgs/msg/`. Common messages:
- `VehicleCommand`: Command interface
- `OffboardControlMode`: Control mode selection
- `TrajectorySetpoint`: Position/velocity targets
- `VehicleLocalPosition`: Position feedback
- `ManualControlSetpoint`: Manual input

## Git Submodules

This repository uses git submodules. To update submodules:
```bash
git submodule update --init --recursive
```

## Common Issues

### Build Cache
If experiencing build issues, clear the build cache:
```bash
cd ws_px4_ros
rm -rf build/ install/ log/
colcon build
```

### DDS Agent Connection
Ensure Micro-XRCE-DDS-Agent is running before launching ROS 2 control nodes. Without it, messages won't reach PX4.

### Gazebo Version
The project supports both Gazebo Classic and newer Gazebo (formerly Ignition). Use appropriate make targets for PX4 builds.
