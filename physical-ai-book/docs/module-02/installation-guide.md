---
id: installation-guide
title: "Installation Guide"
sidebar_label: "Installation"
sidebar_position: 7
description: "Complete installation instructions for Gazebo Harmonic, Unity 2022 LTS, and ROS 2 Humble on Ubuntu, WSL2, and Docker"
keywords: [installation, gazebo harmonic, unity, ros 2 humble, ubuntu, wsl2, docker]
---

# Installation Guide: Module 2 Software Setup

This guide provides step-by-step installation instructions for all software required for Module 2.

## Prerequisites

Before starting, ensure you have:

- ✅ **Module 1 completed** (ROS 2 Humble already installed)
- ✅ **Ubuntu 22.04 LTS** (native, WSL2, or Docker)
- ✅ **Python 3.8+** with pip
- ✅ **At least 20 GB free disk space**
- ✅ **8 GB RAM minimum** (16 GB recommended)

---

## Installation Options

Choose the installation path that matches your system:

| Platform | ROS 2 Humble | Gazebo Harmonic | Unity 2022 LTS | Recommended? |
|----------|--------------|-----------------|----------------|--------------|
| **Ubuntu 22.04 Native** | ✅ Native | ✅ Native | ✅ Native | ⭐ **Best** (full GPU support) |
| **WSL2 (Windows)** | ✅ In WSL2 | ✅ In WSL2 | ⚠️ Windows host | Good (requires TCP bridge) |
| **Docker** | ✅ Container | ✅ Container | ❌ Host only | Fallback (headless Gazebo) |

---

## Option 1: Ubuntu 22.04 Native (Recommended)

### Step 1: Install ROS 2 Humble

If you completed Module 1, ROS 2 Humble should already be installed. Verify:

```bash
ros2 --version
# Should output: ros2 doctor 0.10.3
```

If not installed, follow the [official ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

**Quick install**:
```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release

# Add ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install -y ros-humble-desktop
```

**Source ROS 2**:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 2: Install Gazebo Harmonic

**Add Gazebo repository**:
```bash
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
```

**Install Gazebo Harmonic**:
```bash
sudo apt update
sudo apt install -y gz-harmonic
```

**Verify installation**:
```bash
gz sim --version
# Should output: Gazebo Sim, version 7.x.x
```

**Install ROS 2 - Gazebo bridge**:
```bash
sudo apt install -y ros-humble-ros-gz
```

### Step 3: Install Unity 2022 LTS

**Install Unity Hub**:
```bash
# Download Unity Hub
wget https://public-cdn.cloud.unity3d.com/hub/prod/UnityHub.AppImage

# Make executable
chmod +x UnityHub.AppImage

# Run Unity Hub
./UnityHub.AppImage
```

**In Unity Hub**:
1. **Sign in** with Unity account (create free account if needed)
2. **Installs** → Add → **2022.3 LTS** (Long Term Support)
3. **Select modules** during installation:
   - Linux Build Support (if on Linux)
   - Documentation
4. **Install** (this may take 15-30 minutes)

**Alternative: Manual Unity installation**:
```bash
# Download Unity 2022.3 LTS
wget https://download.unity3d.com/download_unity/[version]/UnitySetup-[version]

# Install
sudo ./UnitySetup-[version]
```

### Step 4: Install Unity Robotics Hub

**In Unity Editor**:
1. **Create new project** (3D Core template)
2. **Window → Package Manager**
3. **+ → Add package from git URL**
4. Enter: `https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector`
5. **Add**
6. Repeat for URDF Importer: `https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer`

### Step 5: Install Additional Tools

**RViz (ROS 2 visualization)**:
```bash
sudo apt install -y ros-humble-rviz2
```

**Python dependencies**:
```bash
pip3 install numpy scipy matplotlib
```

### Step 6: Verify Installation

**Test ROS 2**:
```bash
ros2 run demo_nodes_cpp talker
# Should see "Publishing: 'Hello World: 1'" messages
```

**Test Gazebo Harmonic**:
```bash
gz sim shapes.sdf
# Should open Gazebo GUI with basic shapes
```

**Test Unity** (manual):
- Open Unity Hub → Projects → New 3D Project
- Verify Unity Editor opens without errors

---

## Option 2: WSL2 (Windows Subsystem for Linux)

WSL2 allows running Linux on Windows. This setup uses **ROS 2 + Gazebo in WSL2** and **Unity on Windows host**.

### Step 1: Install WSL2

**In Windows PowerShell (Administrator)**:
```powershell
wsl --install -d Ubuntu-22.04
```

**Restart your computer** when prompted.

**Launch Ubuntu 22.04** from Start menu, create username/password.

### Step 2: Install ROS 2 Humble in WSL2

**In WSL2 terminal**:
```bash
# Update packages
sudo apt update && sudo apt upgrade -y

# Install ROS 2 Humble (same as Ubuntu native - see Option 1 Step 1)
sudo apt install -y ros-humble-desktop

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Step 3: Install Gazebo Harmonic in WSL2

**Enable X11 forwarding for GUI**:
```bash
# Install X server dependencies
sudo apt install -y x11-apps

# Set DISPLAY variable
echo "export DISPLAY=:0" >> ~/.bashrc
source ~/.bashrc
```

**Download and install VcXsrv** (Windows X server):
- Download: [https://sourceforge.net/projects/vcxsrv/](https://sourceforge.net/projects/vcxsrv/)
- Install with default settings
- Launch **XLaunch** with:
  - Multiple windows
  - Start no client
  - **Disable access control** (important!)

**Install Gazebo Harmonic** (same as Ubuntu native - see Option 1 Step 2):
```bash
sudo apt install -y gz-harmonic ros-humble-ros-gz
```

**Test Gazebo with X11**:
```bash
gz sim shapes.sdf
# Gazebo GUI should appear in Windows via VcXsrv
```

### Step 4: Install Unity on Windows Host

**Download Unity Hub** for Windows:
- [https://unity.com/download](https://unity.com/download)
- Install Unity 2022.3 LTS (same as Option 1 Step 3)

### Step 5: Configure ROS 2 - Unity TCP Bridge

**Find WSL2 IP address**:
```bash
ip addr show eth0 | grep inet
# Note the IP (e.g., 172.x.x.x)
```

**In Unity (Windows)**:
1. **Robotics → ROS Settings**
2. **ROS IP Address**: Enter WSL2 IP (e.g., `172.20.10.5`)
3. **ROS Port**: `10000`
4. **Protocol**: TCP

**In WSL2, start ROS TCP Endpoint**:
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

**Windows Firewall**: Allow port 10000 (Windows Defender Firewall → Advanced Settings → Inbound Rules → New Rule → Port 10000 TCP)

### Step 6: Verify WSL2 Setup

**Test ROS 2 in WSL2**:
```bash
ros2 run demo_nodes_cpp talker
```

**Test Gazebo in WSL2** (with VcXsrv running):
```bash
gz sim shapes.sdf
```

**Test Unity on Windows**:
- Open Unity project
- Verify ROS connection to WSL2 IP

---

## Option 3: Docker (Gazebo Only)

Docker is suitable for Gazebo headless mode (no GUI) or with X11 forwarding.

### Step 1: Install Docker

```bash
# Install Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### Step 2: Pull ROS 2 + Gazebo Image

```bash
docker pull osrf/ros:humble-desktop
```

### Step 3: Run Container with Gazebo

**Headless (no GUI)**:
```bash
docker run -it --rm \
  --name ros2-gazebo \
  osrf/ros:humble-desktop \
  bash
```

**With X11 forwarding** (Linux host):
```bash
xhost +local:docker

docker run -it --rm \
  --name ros2-gazebo \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop \
  bash
```

**Inside container, install Gazebo**:
```bash
apt update
apt install -y gz-harmonic ros-humble-ros-gz
```

**Note**: Unity must be installed on host system, not in Docker.

---

## Troubleshooting

### Common Issues

#### Gazebo Harmonic not found

**Error**: `gz: command not found`

**Solution**:
```bash
# Check if installed
dpkg -l | grep gz-harmonic

# If not installed, retry installation
sudo apt install -y gz-harmonic
```

#### Unity Robotics Hub installation fails

**Error**: Package download fails in Unity

**Solution**:
- Check internet connection
- Verify git URL is correct (copy-paste from this guide)
- Try adding package manually:
  1. Download ZIP from GitHub
  2. Unity → Window → Package Manager → + → Add package from disk

#### ROS 2 - Gazebo bridge not working

**Error**: Topics not visible in ROS 2

**Solution**:
```bash
# Verify ros_gz_bridge is installed
ros2 pkg list | grep ros_gz

# Reinstall if missing
sudo apt install -y ros-humble-ros-gz

# Check bridge status
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock
```

#### WSL2 Gazebo GUI not appearing

**Error**: Gazebo launches but no window appears

**Solution**:
- Ensure VcXsrv is running on Windows
- Check DISPLAY variable: `echo $DISPLAY` (should be `:0`)
- Disable Windows Firewall temporarily to test
- Retry XLaunch with "Disable access control"

---

## System Requirements Summary

### Minimum Requirements

| Component | Requirement |
|-----------|-------------|
| **OS** | Ubuntu 22.04 LTS |
| **CPU** | Intel i5 / AMD Ryzen 5 (4 cores) |
| **RAM** | 8 GB |
| **GPU** | Integrated graphics (Intel UHD, AMD Radeon) |
| **Storage** | 20 GB free space |

### Recommended Requirements

| Component | Requirement |
|-----------|-------------|
| **OS** | Ubuntu 22.04 LTS (native) |
| **CPU** | Intel i7 / AMD Ryzen 7 (8 cores) |
| **RAM** | 16 GB |
| **GPU** | NVIDIA GTX 1650 or better (Unity rendering) |
| **Storage** | 50 GB free space |

---

## Next Steps

After successful installation:

1. **Verify all software** works individually
2. **Test integration**: Launch Gazebo, publish ROS 2 topics, connect Unity
3. **Proceed to Chapter 1**: [Digital Twins in Robotics](./chapter-01-digital-twins-robotics.md)

---

## Additional Resources

- **ROS 2 Humble Docs**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- **Gazebo Harmonic Docs**: [https://gazebosim.org/docs/harmonic](https://gazebosim.org/docs/harmonic)
- **Unity Robotics Hub**: [https://github.com/Unity-Technologies/Unity-Robotics-Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)
- **WSL2 Setup Guide**: [https://learn.microsoft.com/en-us/windows/wsl/install](https://learn.microsoft.com/en-us/windows/wsl/install)

**Need Help?** Refer to the [Troubleshooting Guide](./troubleshooting.md) for common issues and solutions.
