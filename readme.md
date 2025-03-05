# Waypoint Recorder

## Overview
The **Waypoint Recorder** is a ROS 2 node that records waypoints based on odometry data and user input. It allows users to create waypoints manually via a service or automatically based on movement and rotation thresholds. The recorded waypoints are stored in a YAML file for later use.

## Features
- Subscribes to `/odom` topic to receive odometry data.
- Provides a service `/create_waypoint` for manually adding waypoints.
- Automatically records waypoints based on distance and rotation thresholds.
- Saves waypoints to `waypoints.yaml` file.
- Ensures waypoints are valid by checking for NaN/Inf values.

## Installation

1. Clone the repository into your ROS 2 workspace:
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/hariChozhan/anscer_ws.git
   ```
2. Navigate to your workspace and build the package:
   ```bash
   cd ~/ros2_ws
   colcon build 
   ```
3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage

### 1. Launch the Waypoint Recorder Node
```bash
ros2 run waypoint_nav waypoint_recorder
```

### 2. Manually Create a Waypoint
Use the following ROS 2 service call to create a waypoint with a specific ID:
```bash
ros2 service call /create_waypoint anscer_msgs/srv/Waypoint "{id: 1}"
```

### 3. View Saved Waypoints
Waypoints are stored in `waypoints.yaml` in the current working directory. Example format:
```yaml
- id: 1
  x: 1.23
  y: 2.34
  theta: 0.56
- id: 2
  x: 3.45
  y: 4.56
  theta: 1.23
```

## Parameters
| Parameter | Description | Default Value |
|-----------|------------|--------------|
| `min_distance_threshold` | Minimum distance required between consecutive waypoints | 1.0 |
| `min_rotation_threshold` | Minimum rotation required between consecutive waypoints | 0.2 |

## Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/msg/Odometry` | Subscribed topic for receiving odometry data |

## Services
| Service | Type | Description |
|---------|------|-------------|
| `/create_waypoint` | `anscer_msgs/srv/Waypoint` | Manually create a waypoint |


