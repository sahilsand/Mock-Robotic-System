# ROS 2 Distributed Robotics System

A distributed robotics system built with ROS 2 Jazzy featuring four containerized nodes for sensor simulation, localization, mapping, and control.

## Overview

This project demonstrates a multi-node robotic system where each component runs in its own Docker container and communicates via ROS 2 topics. The system simulates a camera sensor feeding data to localization and mapping modules, while a control node generates commands based on position feedback.

## System Architecture

```
┌─────────────────────────────────────────────────────────┐
│                     Docker Network (172.20.0.0/16)      │
│                                                         │
│  ┌──────────────┐                                       │
│  │Camera Sensor │  publishes /sensor/data (Image)       │
│  │ 172.20.0.2   │                                       │
│  └──────┬───────┘                                       │
│         │                                               │
│         ├─────────────────┬─────────────────┐           │
│         ↓                 ↓                 │           │
│  ┌──────────────┐  ┌──────────────┐         │           │
│  │ Localization │  │   Mapping    │         │           │
│  │ 172.20.0.3   │  │ 172.20.0.4   │         │           │
│  └──────┬───────┘  └──────────────┘         │           │
│         │                                   │           │
│         │ publishes                         │           │
│         │ /localization/pose                │           │
│         ↓                                   │           │
│  ┌──────────────┐                           │           │
│  │   Control    │ ← ← ← ← ← ← ← ← ← ← ← ← ← ┘           │
│  │ 172.20.0.5   │   receives /sensor/data               │
│  └──────────────┘                                       │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### Components

| Component | Container IP | Function |
|-----------|--------------|----------|
| Camera Sensor | 172.20.0.2 | Publishes simulated 640x480 grayscale images at 1 Hz |
| Localization | 172.20.0.3 | Processes sensor data and estimates robot position |
| Mapping | 172.20.0.4 | Builds occupancy grid map from sensor data |
| Control | 172.20.0.5 | Generates velocity commands using feedback control |

## Prerequisites

- Docker
- Docker Compose
- ROS2

## Quick Start

### Clone repository

```bash
# Clone command
git clone https://github.com/sahilsand/ltu-rai-assignment-sahil.git

# Get inside the root folder
cd ltu-rai-assignment-sahil
```

### Build and Run

```bash
# Build all containers
docker compose build

# Start the system
docker compose up

# Stop the system
Ctrl+C
```

### Clean Rebuild

```bash
docker compose down
docker compose build --no-cache
docker compose up
```

## Project Structure

```
.
├── src/
│   ├── camera_sensor_pkg/     # Camera sensor node
│   ├── localization_pkg/      # Localization node
│   ├── mapping_pkg/           # Mapping node
│   └── control_pkg/           # Control node
├── Dockerfile                 # Container image definition
├── docker-compose.yml         # Multi-container orchestration
├── entrypoint.sh             # Container startup script
└── README.md                 # This file
```

## ROS 2 Topics

### Published Topics

| Topic | Message Type | Publisher | Rate |
|-------|-------------|-----------|------|
| `/sensor/data` | sensor_msgs/Image | camera_sensor_node | 1 Hz |
| `/localization/pose` | geometry_msgs/PoseStamped | localization_node | 1 Hz |
| `/map` | nav_msgs/OccupancyGrid | mapping_node | 1 Hz |
| `/control/action` | geometry_msgs/Twist | control_node | 1 Hz |

### Subscriptions

| Node | Subscribes To | Purpose |
|------|---------------|---------|
| localization_node | `/sensor/data` | Process images for position estimation |
| mapping_node | `/sensor/data` | Build environment map |
| control_node | `/localization/pose` | Generate control commands |

## Testing & Verification

### Check Container Status

```bash
# List running containers
docker ps

# View logs from all containers
docker compose logs

# View logs from specific container
docker logs camera_sensor
docker logs -f control  # Follow mode
```

### ROS 2 Commands

```bash
# List all active topics
docker exec camera_sensor ros2 topic list

# Command to monitor a specific topic
docker exec control ros2 topic echo /localization/pose

# For checking the topic publishing rate
docker exec camera_sensor ros2 topic hz /sensor/data

# Listing all nodes
docker exec camera_sensor ros2 node list

# For getting the node information
docker exec camera_sensor ros2 node info /localization_node
```

### Network Inspection

```bash
# For inspecting the Docker network
docker network inspect ltu-rai-assignment_rai_net

# TO check IP addresses of the container
docker inspect camera_sensor | grep IPAddress
```

## Expected Output

When running successfully, you should see continuous logs:

```
camera_sensor  | [INFO] Published Image Data
localization   | [INFO] Received sensor data: 640x480
localization   | [INFO] Published Estimated Pose: x=5.00, y=2.50
mapping        | [INFO] Received sensor data for mapping: 640x480
mapping        | [INFO] Updated Map (iteration 50)
control        | [INFO] Feedback: pos=(5.00, 2.50), Error=(5.00, 2.50), Control=(2.50, 1.25)
```

### Control Node Output Explanation

The control node displays:
- **pos**: Current position from localization
- **Error**: Distance from target position (10.0, 5.0)
- **Control**: Velocity command (proportional to error)

The error decreases as the simulated robot approaches the target, reaches zero at the goal, then becomes negative as it overshoots.

## Control System

The control node implements a proportional controller:

- **Target Position**: (10.0, 5.0) meters
- **Proportional Gain**: 0.5
- **Control Law**: velocity = 0.5 × (target - current_position)

## Troubleshooting

### Containers Not Communicating

```bash
# For verifing ROS_DOMAIN_ID
docker exec camera_sensor printenv | grep ROS_DOMAIN_ID

# To check network connectivity
docker exec camera_sensor ping -c 3 172.20.0.3
```

### Build Errors

```bash
# For complete cleanup
docker compose down -v
docker system prune -af

# Rebuilding from scratch
docker compose build --no-cache
```

### Node Not Publishing

```bash
# For checking if node is running
docker exec camera_sensor ros2 node list

# To verify if topic exists
docker exec camera_sensor ros2 topic list

# to check the topic info
docker exec camera_sensor ros2 topic info /sensor/data
```

## Technical Stack

- **ROS 2 Distribution**: Jazzy Jalisco
- **Base Image**: osrf/ros:jazzy-desktop
- **Build System**: colcon
- **DDS**: Fast-DDS
- **Network**: Docker bridge (172.20.0.0/16)
- **Language**: Python 3

## Development

### Adding a New Node

1. Create package structure in `src/`
2. Implement node in Python
3. Update `package.xml` and `setup.py`
4. Add service to `docker-compose.yml`
5. Rebuild: `docker compose build`

### Modifying Existing Nodes

```bash
# Rn these after code changes
docker compose down
docker compose build
docker compose up
```

## License

MIT License