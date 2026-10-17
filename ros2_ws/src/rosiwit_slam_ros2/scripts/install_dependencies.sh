# ============================================================
# FAST-LIO2 SLAM - Dependency Installation Script
# ============================================================
# Automates the installation of all required dependencies
# Supports: Ubuntu 22.04, Ubuntu 20.04
# ============================================================

#!/bin/bash
set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Detect Ubuntu version
UBUNTU_VERSION=$(lsb_release -rs)
ROS_DISTRO=""

if [ "$UBUNTU_VERSION" = "22.04" ]; then
    ROS_DISTRO="humble"
    echo -e "${GREEN}Detected Ubuntu 22.04, using ROS2 Humble${NC}"
elif [ "$UBUNTU_VERSION" = "20.04" ]; then
    ROS_DISTRO="foxy"
    echo -e "${YELLOW}Detected Ubuntu 20.04, using ROS2 Foxy${NC}"
else
    echo -e "${RED}Unsupported Ubuntu version: ${UBUNTU_VERSION}${NC}"
    echo -e "${YELLOW}Supported versions: 20.04, 22.04${NC}"
    exit 1
fi

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Installing FAST-LIO2 SLAM Dependencies${NC}"
echo -e "${BLUE}ROS2 Distro: ${ROS_DISTRO}${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Update system
echo -e "${YELLOW}[1/7] Updating system packages...${NC}"
sudo apt update
sudo apt install -y software-properties-common curl gnupg lsb-release

# Install ROS2
echo -e "${YELLOW}[2/7] Installing ROS2 ${ROS_DISTRO}...${NC}"
if ! dpkg -l | grep -q "ros-${ROS_DISTRO}-ros-base"; then
    # Add ROS2 repository
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    
    # Install ROS2 base
    sudo apt install -y ros-${ROS_DISTRO}-ros-base
    
    # Source ROS2
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
else
    echo -e "${GREEN}ROS2 ${ROS_DISTRO} already installed${NC}"
fi

# Install system dependencies
echo -e "${YELLOW}[3/7] Installing system dependencies...${NC}"
sudo apt install -y \
    build-essential \
    cmake \
    ninja-build \
    ccache \
    git \
    gdb \
    valgrind \
    htop \
    doxygen \
    graphviz \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Install PCL and Eigen
echo -e "${YELLOW}[4/7] Installing PCL and Eigen...${NC}"
sudo apt install -y \
    libeigen3-dev \
    libpcl-dev \
    libpcl-common-dev \
    libpcl-io-dev \
    libpcl-filters-dev \
    libpcl-kdtree-dev \
    libpcl-features-dev \
    libpcl-segmentation-dev \
    libpcl-surface-dev \
    libpcl-registration-dev \
    libpcl-visualization-dev \
    pcl-tools

# Install Boost and other libraries
echo -e "${YELLOW}[5/7] Installing Boost and optional libraries...${NC}"
sudo apt install -y \
    libboost-all-dev \
    libyaml-cpp-dev \
    libceres-dev \
    libtbb-dev

# Install ROS2 packages
echo -e "${YELLOW}[6/7] Installing ROS2 packages...${NC}"
sudo apt install -y \
    ros-${ROS_DISTRO}-pcl-conversions \
    ros-${ROS_DISTRO}-pcl-ros \
    ros-${ROS_DISTRO}-tf2 \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-sensor-msgs \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rosbag2

# Install Sophus from source
echo -e "${YELLOW}[7/7] Installing Sophus...${NC}"
if [ ! -d "/usr/local/include/sophus" ]; then
    cd /tmp
    git clone https://github.com/strasdat/Sophus.git
    cd Sophus
    git checkout 1.22.10
    mkdir build && cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j$(nproc)
    sudo make install
    cd /
    rm -rf /tmp/Sophus
    echo -e "${GREEN}Sophus installed successfully${NC}"
else
    echo -e "${GREEN}Sophus already installed${NC}"
fi

# Optional: Install GTSAM
echo ""
echo -e "${YELLOW}Would you like to install GTSAM for loop closure optimization? (y/n)${NC}"
read -r INSTALL_GTSAM
if [ "$INSTALL_GTSAM" = "y" ] || [ "$INSTALL_GTSAM" = "Y" ]; then
    echo -e "${YELLOW}Installing GTSAM...${NC}"
    cd /tmp
    git clone https://github.com/borglab/gtsam.git
    cd gtsam
    git checkout 4.2a9
    mkdir build && cd build
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DGTSAM_BUILD_EXAMPLES=OFF \
        -DGTSAM_BUILD_TESTS=OFF \
        -DGTSAM_WITH_TBB=ON \
        -DGTSAM_USE_SYSTEM_EIGEN=ON
    make -j$(nproc)
    sudo make install
    cd /
    rm -rf /tmp/gtsam
    echo -e "${GREEN}GTSAM installed successfully${NC}"
fi

# Update library cache
sudo ldconfig

# Install Python packages
echo -e "${YELLOW}Installing Python packages...${NC}"
pip3 install --user \
    numpy \
    scipy \
    matplotlib \
    pandas \
    pyyaml \
    rosbags \
    evo

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Installation complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "To start using ROS2, run:"
echo -e "  ${BLUE}source /opt/ros/${ROS_DISTRO}/setup.bash${NC}"
echo ""
echo -e "Or add to your .bashrc:"
echo -e "  ${BLUE}echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> ~/.bashrc${NC}"
echo ""