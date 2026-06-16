# rosiwit_simulator - Deployment Guide (ROS2 Humble)

> **Version**: 2.0.0 | **ROS2 Humble** | **Updated**: 2026-05-05

## Table of Contents

1. [Quick Start](#quick-start)
2. [Local Build (Native ROS2)](#local-build-native-ros2)
3. [Docker Deployment](#docker-deployment)
4. [Docker Compose Stack](#docker-compose-stack)
5. [CI/CD Pipeline](#cicd-pipeline)
6. [Environment Variables](#environment-variables)
7. [Troubleshooting](#troubleshooting)

---

## Quick Start

```bash
# Clone workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <repository-url> rosiwit_simulator

# Build (ROS2 Humble)
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select simulator

# Run 3D LiDAR simulation
source install/setup.bash
ros2 launch simulator simulator_gazebo_3d.launch.py
```

---

## Local Build (Native ROS2)

### Prerequisites

- Ubuntu 22.04 LTS (or WSL2)
- ROS2 Humble Hawksbill installed
- Gazebo 11

### Build Steps

```bash
# 1. Source ROS2 Humble
source /opt/ros/humble/setup.bash

# 2. Navigate to workspace
cd ~/ros2_ws

# 3. Install dependencies (if rosdep initialized)
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build --packages-select simulator \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --symlink-install

# 5. Source workspace
source install/setup.bash

# 6. Verify
ros2 pkg list | grep simulator
```

### Available Launch Files

| Launch File | Description |
|---|---|
| `simulator_gazebo.launch.py` | 2D LiDAR simulation |
| `simulator_gazebo_3d.launch.py` | 3D LiDAR simulation (Velodyne VLP16) |
| `simulator_mapping_gmaping.launch.py` | GMapping SLAM |
| `simulator_mapping_cartographer.launch.py` | Cartographer SLAM |
| `simulator_amcl_diff.launch.py` | AMCL localization (differential drive) |
| `simulator_nav_movebase.launch.py` | Navigation with move base |
| `simulator_map_server.launch.py` | Map server |
| `simulator_rviz.launch.py` | RViz2 visualization |

---

## Docker Deployment

### Build

```bash
cd docker

# Build runtime image
./build.sh

# Build with cache
./build.sh --cache

# Build development image (with build tools)
./build.sh --devel

# Build and push to registry
DOCKER_REGISTRY=registry.example.com ./build.sh --push
```

### Run

```bash
cd docker

# Run 3D LiDAR simulation
./run.sh

# Run 2D LiDAR simulation
./run.sh --2d

# Run development environment
./run.sh --devel

# Run full SLAM stack
./run.sh --slam

# Stop all containers
./run.sh --stop
```

### Manual Docker Run

```bash
# Build
docker build -f docker/Dockerfile --target runtime -t rosiwit-simulator:2.0.0 .

# Run (with GUI)
docker run --rm -it \
    --network host \
    --ipc host \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=42 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v /dev/shm:/dev/shm \
    rosiwit-simulator:2.0.0
```

---

## Docker Compose Stack

### Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    ROS2 DDS (Domain 42)                   │
│              (Cyclone DDS, no bridge needed)              │
├──────────────┬──────────────────┬────────────────────────┤
│  simulator   │      slam        │      slam-viz          │
│  (Gazebo 11) │  (FAST-LIO2)     │     (RViz2)            │
│  3D LiDAR    │  3D Mapping      │   Visualization        │
│  Velodyne    │  PointCloud      │                        │
└──────────────┴──────────────────┴────────────────────────┘
```

### Key Changes from ROS1

| Aspect | ROS1 (Old) | ROS2 Humble (New) |
|---|---|---|
| Base image | `ros:noetic-ros-base-focal` | `ros:humble-ros-base-jammy` |
| Build system | `catkin_make` | `colcon build` |
| Launch system | XML `.launch` | Python `.launch.py` |
| Communication | ROS1 TCP (roscore) | DDS (Cyclone/FastRTPS) |
| ros1_bridge | Required | **Not needed** |
| DDS Domain | N/A | `ROS_DOMAIN_ID=42` |
| Security | root user | Non-root (`rosuser:1000`) |
| Capabilities | Full | `cap_drop ALL` + minimal |

### Starting the Stack

```bash
# Full SLAM stack
docker-compose -f docker/docker-compose.yml up -d

# Simulator only
docker-compose -f docker/docker-compose.yml up -d simulator

# View logs
docker-compose -f docker/docker-compose.yml logs -f simulator
```

---

## CI/CD Pipeline

### GitLab CI

4-stage pipeline defined in `.gitlab-ci.yml`:

1. **lint** - XML/Shell/Python syntax checks
2. **build** - Colcon build + Docker image build (parallel)
3. **test** - Colcon test + launch file validation
4. **deploy** - Docker push to registry (main/master only)

### Jenkins Pipeline

Declarative pipeline defined in `Jenkinsfile`:

1. **Checkout** - SCM checkout + git metadata
2. **Lint** - Parallel XML/Shell/Python checks
3. **Build** - Parallel Colcon + Docker builds
4. **Test** - Colcon test with JUnit results
5. **Deploy** - Docker push (main/master only)

### Required CI Variables

| Variable | GitLab CI | Jenkins | Description |
|---|---|---|---|
| `CI_REGISTRY` | Auto | - | GitLab container registry URL |
| `CI_REGISTRY_USER` | Auto | `DOCKER_REGISTRY_USER` | Registry username |
| `CI_REGISTRY_PASSWORD` | Auto | `DOCKER_REGISTRY_PASSWORD` | Registry password |

---

## Environment Variables

| Variable | Default | Description |
|---|---|---|
| `ROS_DOMAIN_ID` | `42` | DDS domain ID (must match across nodes) |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS middleware implementation |
| `DISPLAY` | `:0` | X11 display for GUI |
| `LIBGL_ALWAYS_SOFTWARE` | `0` | Force software rendering |
| `GAZEBO_MODEL_PATH` | `<pkg>/models` | Gazebo model search path |
| `ROS_LOG_LEVEL` | `INFO` | Logging verbosity |
| `DOCKER_REGISTRY` | (empty) | Docker registry URL for push |

---

## Troubleshooting

### Docker: Cannot connect to X11

```bash
# Allow Docker to access X11
xhost +local:docker

# Or use specific display
export DISPLAY=:0
```

### Docker: DDS nodes cannot discover each other

```bash
# Ensure all containers use the same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42

# Verify DDS discovery
docker exec -it rosiwit-simulator ros2 topic list
```

### Docker: Gazebo fails to start (GPU)

```bash
# Use software rendering
export LIBGL_ALWAYS_SOFTWARE=1
docker run --rm -e LIBGL_ALWAYS_SOFTWARE=1 ...
```

### Colcon build fails

```bash
# Clean and rebuild
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --packages-select simulator
```

### ROS2 topics not appearing

```bash
# Check ROS2 environment
ros2 doctor

# Check DDS configuration
echo $RMW_IMPLEMENTATION
echo $ROS_DOMAIN_ID

# List all topics
ros2 topic list
ros2 topic echo /velodyne_points
```
