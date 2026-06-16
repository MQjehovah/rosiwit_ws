# rosiwit_slam - Dependencies

## System Dependencies (Ubuntu 22.04 / ROS2 Humble)

### Core Build Tools
| Package | Version | Required | Purpose |
|---------|---------|----------|---------|
| cmake | >= 3.8 | Yes | Build system |
| build-essential | latest | Yes | C/C++ compiler toolchain |
| git | latest | Yes | Source control |

### ROS2 Humble Packages
| Package | Version | Required | Purpose |
|---------|---------|----------|---------|
| ros-humble-ros-base | humble | Yes | Base ROS2 installation |
| ros-humble-pcl-conversions | humble | Yes | PointCloud2 ↔ PCL conversion |
| ros-humble-pcl-ros | humble | Yes | PCL ROS2 integration |
| ros-humble-tf2 | humble | Yes | Transform library |
| ros-humble-tf2-ros | humble | Yes | TF2 ROS bindings |
| ros-humble-tf2-eigen | humble | Yes | TF2 Eigen integration |
| ros-humble-tf2-geometry-msgs | humble | Yes | TF2 geometry message types |
| ros-humble-tf2-sensor-msgs | humble | Yes | TF2 sensor message types |
| ros-humble-sensor-msgs | humble | Yes | Sensor message types |
| ros-humble-nav-msgs | humble | Yes | Navigation message types |
| ros-humble-geometry-msgs | humble | Yes | Geometry message types |
| ros-humble-std-srvs | humble | Yes | Standard service types |
| ros-humble-message-filters | humble | Yes | Message synchronization |
| ros-humble-cv-bridge | humble | No | OpenCV-ROS2 bridge |
| ros-humble-image-transport | humble | No | Image transport |
| ros-humble-rviz2 | humble | No | Visualization |
| ament-cmake | humble | Yes | ROS2 CMake build system |

### C++ Libraries
| Library | Version | Required | Purpose |
|---------|---------|----------|---------|
| Eigen3 | >= 3.3.7 | Yes | Linear algebra |
| PCL (Point Cloud Library) | >= 1.12 | Yes | 3D point cloud processing |
| Sophus | >= 1.22.10 | Yes | Lie groups (SO3, SE3) |
| GTSAM | >= 4.2 | Yes | Factor graph optimization |
| yaml-cpp | >= 0.7 | Yes | YAML config parsing |
| Boost | >= 1.74 | Yes | C++ utility libraries |
| Ceres Solver | >= 2.1 | No | Non-linear optimization |
| TBB | latest | No | Parallel algorithms |
| OpenCV | >= 4.5 | No | Image processing |

### Python Dependencies (Testing/Development)
| Package | Version | Required | Purpose |
|---------|---------|----------|---------|
| python3 | >= 3.10 | Yes | Python runtime |
| pytest | >= 7.0 | Dev | Test framework |
| pytest-cov | latest | Dev | Coverage reporting |
| flake8 | latest | Dev | Python linter |
| colcon-common-extensions | latest | Yes | ROS2 build tool |

## Install Commands

### Ubuntu 22.04 (Native)

```bash
# ROS2 Humble base
sudo apt update && sudo apt install -y ros-humble-ros-base

# Core dependencies
sudo apt install -y \
    libeigen3-dev libpcl-dev libyaml-cpp-dev libboost-all-dev \
    libceres-dev libtbb-dev

# ROS2 packages
sudo apt install -y \
    ros-humble-pcl-conversions ros-humble-pcl-ros \
    ros-humble-tf2 ros-humble-tf2-ros ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs ros-humble-tf2-sensor-msgs \
    ros-humble-sensor-msgs ros-humble-nav-msgs \
    ros-humble-geometry-msgs ros-humble-std-srvs \
    ros-humble-message-filters

# Sophus (from source)
git clone https://github.com/strasdat/Sophus.git /tmp/Sophus
cd /tmp/Sophus && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc) && sudo make install

# GTSAM (from source)
git clone https://github.com/borglab/gtsam.git /tmp/gtsam
cd /tmp/gtsam && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DGTSAM_BUILD_EXAMPLES=OFF -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc) && sudo make install

# Update library cache
sudo ldconfig
```

### Docker (Recommended)

```bash
# Production image
cd docker && ./build.sh

# Development image
cd docker && ./build.sh --devel
```

## Version Pinning

All third-party git dependencies are pinned to specific commit hashes for supply chain security (SEC-008):
- **Sophus**: `a79ba54d07b13f58105c6e3f4e472d4968a3e728`
- **GTSAM**: `316c4b40f4f26a7e4b0d89cc8d1f6a4fc3a0f40e`

## ROS2 DDS Configuration

Default middleware: **Cyclone DDS** (`rmw_cyclonedds_cpp`)
- Better performance for real-time SLAM workloads
- Lower latency than Fast-RTPS
- Compatible with ROS2 Humble

```bash
# Install Cyclone DDS
sudo apt install -y ros-humble-rmw-cyclonedds-cpp

# Set as default
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42
```
