---
id: troubleshooting
title: "Troubleshooting Guide"
sidebar_label: "Troubleshooting"
sidebar_position: 8
description: "Comprehensive troubleshooting guide for Gazebo Harmonic, Unity 2022 LTS, ROS 2 Humble, and cross-tool integration issues"
keywords: [troubleshooting, gazebo, unity, ros 2, debugging, common errors, wsl2, docker]
---

# Troubleshooting Guide: Module 2 Digital Twin Setup

This guide addresses common issues encountered when setting up and using Gazebo Harmonic, Unity 2022 LTS, and ROS 2 Humble for digital twin development.

---

## Quick Diagnostic Flowchart

**Start here if you're experiencing issues:**

```
1. Does ROS 2 work?
   └─ No → See "ROS 2 Issues" section
   └─ Yes → Continue to 2

2. Does Gazebo launch?
   └─ No → See "Gazebo Installation Issues" section
   └─ Yes → Continue to 3

3. Can you import URDF in Unity?
   └─ No → See "Unity URDF Import Issues" section
   └─ Yes → Continue to 4

4. Can ros_gz_bridge connect Gazebo and ROS 2?
   └─ No → See "ROS 2 - Gazebo Integration Issues" section
   └─ Yes → Continue to 5

5. Can Unity connect to ROS 2?
   └─ No → See "Unity - ROS 2 Integration Issues" section
   └─ Yes → See "Performance Optimization" section
```

---

## Gazebo Harmonic Issues

### Issue 1: "gz: command not found"

**Symptom**: Running `gz sim` returns "command not found"

**Causes**:
- Gazebo Harmonic not installed
- Wrong Gazebo version (Classic installed instead)
- PATH not configured

**Solutions**:

1. **Verify installation**:
```bash
dpkg -l | grep gz-harmonic
# Should show: ii  gz-harmonic  <version>
```

2. **If not installed**:
```bash
sudo apt update
sudo apt install -y gz-harmonic
```

3. **If installed but command not found**, check PATH:
```bash
which gz
# Should output: /usr/bin/gz
```

4. **Source environment**:
```bash
source /opt/ros/humble/setup.bash
```

---

### Issue 2: Gazebo GUI Not Appearing (WSL2)

**Symptom**: `gz sim world.sdf` runs but no window appears on Windows

**Causes**:
- VcXsrv not running on Windows
- DISPLAY variable not set
- Windows Firewall blocking X11 traffic

**Solutions**:

1. **Launch VcXsrv on Windows**:
   - Start → XLaunch
   - Select: Multiple windows, Start no client, **Disable access control** ✅
   - Finish

2. **Set DISPLAY variable in WSL2**:
```bash
echo $DISPLAY
# Should output: :0

# If empty, set it:
export DISPLAY=:0
echo "export DISPLAY=:0" >> ~/.bashrc
source ~/.bashrc
```

3. **Test X11 forwarding**:
```bash
sudo apt install -y x11-apps
xeyes
# A small window with eyes should appear on Windows
```

4. **Check Windows Firewall**:
   - Windows Defender Firewall → Advanced Settings
   - Inbound Rules → VcXsrv should be allowed

5. **Alternative: Use WSL2 IP directly**:
```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
```

---

### Issue 3: Gazebo Simulation Extremely Slow (RTF < 0.3)

**Symptom**: Real-Time Factor (RTF) is very low, simulation lags

**Causes**:
- Complex collision meshes
- Too small physics time step
- Insufficient system resources
- Too many models in world

**Solutions**:

1. **Simplify collision geometries**:
```xml
<!-- Instead of complex mesh -->
<collision>
  <geometry>
    <mesh><uri>complex_model.stl</uri></mesh>
  </geometry>
</collision>

<!-- Use primitive shapes -->
<collision>
  <geometry>
    <box><size>0.5 0.5 0.5</size></box>
  </geometry>
</collision>
```

2. **Increase physics time step** (in world SDF):
```xml
<physics name="1ms" type="ode">
  <max_step_size>0.002</max_step_size>  <!-- Increased from 0.001 -->
  <real_time_factor>1.0</real_time_factor>
</physics>
```

3. **Reduce sensor update rates**:
```xml
<sensor name="lidar" type="gpu_lidar">
  <update_rate>5</update_rate>  <!-- Reduced from 10 Hz -->
</sensor>
```

4. **Monitor resource usage**:
```bash
htop  # Check CPU usage
nvidia-smi  # Check GPU usage (if applicable)
```

---

### Issue 4: Robot Falls Through Ground Plane

**Symptom**: Robot penetrates ground and falls infinitely

**Causes**:
- Missing collision geometry on ground or robot
- Missing inertial properties on robot links
- Contact parameters too soft

**Solutions**:

1. **Verify ground plane has collision**:
```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">  <!-- Must be present -->
      <geometry>
        <plane><normal>0 0 1</normal></plane>
      </geometry>
    </collision>
  </link>
</model>
```

2. **Verify robot links have inertial properties**:
```xml
<link name="base_link">
  <inertial>
    <mass value="10.0"/>  <!-- Must be > 0 -->
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
  <!-- ... -->
</link>
```

3. **Increase contact stiffness**:
```xml
<collision>
  <surface>
    <contact>
      <ode>
        <kp>1e7</kp>  <!-- Increased stiffness -->
        <kd>100</kd>
      </ode>
    </contact>
  </surface>
</collision>
```

---

### Issue 5: "SDF Parsing Error" When Loading Model

**Symptom**: `gz sim` shows parsing errors, model fails to load

**Common causes**:
- Invalid XML syntax
- Wrong SDF version
- Missing required tags

**Solutions**:

1. **Validate SDF syntax**:
```bash
gz sdf -k model.sdf
# Shows validation errors
```

2. **Check SDF version**:
```xml
<?xml version="1.0" ?>
<sdf version="1.9">  <!-- Use 1.9 for Gazebo Harmonic -->
```

3. **Common missing tags**:
```xml
<model name="my_robot">
  <link name="base_link">
    <inertial>  <!-- Required for dynamic models -->
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
    <collision name="collision">  <!-- Required for physics -->
      <geometry><box><size>1 1 1</size></box></geometry>
    </collision>
    <visual name="visual">  <!-- Required for rendering -->
      <geometry><box><size>1 1 1</size></box></geometry>
    </visual>
  </link>
</model>
```

---

## Unity 2022 LTS Issues

### Issue 6: Unity Robotics Hub Package Installation Fails

**Symptom**: Unity Package Manager shows "Error adding package" when adding from git URL

**Causes**:
- No internet connection
- Git not installed
- Wrong URL format
- Unity version incompatibility

**Solutions**:

1. **Verify git is installed**:
```bash
git --version
# Should output: git version 2.x.x
```

2. **If git not found**, install:
```bash
# Ubuntu/WSL2
sudo apt install -y git

# Windows
# Download from https://git-scm.com/download/win
```

3. **Use exact git URLs**:
```
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

4. **Alternative: Manual installation**:
   - Download ZIP from GitHub
   - Extract to temporary folder
   - Unity → Window → Package Manager → + → Add package from disk
   - Navigate to `com.unity.robotics.ros-tcp-connector/package.json`

---

### Issue 7: URDF Import Creates Empty GameObject Hierarchy

**Symptom**: Robotics → Import URDF completes but robot has no meshes or joints

**Causes**:
- URDF uses absolute paths not accessible in Unity
- Mesh files missing
- URDF syntax errors

**Solutions**:

1. **Check URDF mesh paths**:
```xml
<!-- Bad: Absolute Linux path -->
<mesh filename="/home/user/catkin_ws/src/my_robot/meshes/arm.stl"/>

<!-- Good: Package URI (requires ROS package structure) -->
<mesh filename="package://my_robot/meshes/arm.stl"/>

<!-- Good: Relative path (if meshes in same folder) -->
<mesh filename="meshes/arm.stl"/>
```

2. **Verify mesh files exist**:
```bash
ls -lh meshes/
# Should show .stl, .dae, or .obj files
```

3. **Check Unity Console for errors**:
   - Window → General → Console
   - Look for "File not found" or "Parse error" messages

4. **Simplify URDF for testing**:
```xml
<!-- Replace meshes with basic shapes temporarily -->
<visual>
  <geometry>
    <cylinder radius="0.05" length="0.3"/>
  </geometry>
</visual>
```

---

### Issue 8: Unity ROS Connection Fails (TCP Endpoint Timeout)

**Symptom**: Unity shows "ROS connection timeout" or "Failed to connect to ROS"

**Causes**:
- ROS TCP Endpoint not running
- Wrong IP address
- Firewall blocking port 10000
- ROS 2 not sourced

**Solutions**:

1. **Verify ROS TCP Endpoint is running**:
```bash
# Install if not present
sudo apt install -y ros-humble-ros-tcp-endpoint

# Run endpoint
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
# Should output: "Starting server on 0.0.0.0:10000"
```

2. **Check Unity ROS Settings**:
   - Robotics → ROS Settings
   - ROS IP Address:
     - Ubuntu native: `127.0.0.1`
     - WSL2: Find WSL2 IP with `ip addr show eth0 | grep inet` (e.g., `172.20.10.5`)
   - ROS Port: `10000`
   - Protocol: TCP

3. **Test network connectivity**:
```bash
# From WSL2, test if port is listening
netstat -tuln | grep 10000
# Should show: tcp  0  0.0.0.0:10000  LISTEN

# From Windows, test connection to WSL2
# PowerShell:
Test-NetConnection -ComputerName 172.20.10.5 -Port 10000
```

4. **Windows Firewall (WSL2 only)**:
   - Windows Defender Firewall → Advanced Settings
   - Inbound Rules → New Rule
   - Port → TCP → 10000 → Allow connection

---

### Issue 9: Unity Render is Too Dark / No Reflections

**Symptom**: Robot appears very dark, no reflections visible

**Causes**:
- No lighting in scene
- Materials not set to PBR
- No HDRI skybox

**Solutions**:

1. **Add directional light**:
   - GameObject → Light → Directional Light
   - Intensity: 1.0
   - Color: White

2. **Convert materials to URP Lit shader**:
   - Select material in Project window
   - Shader: Universal Render Pipeline → Lit

3. **Add HDRI skybox**:
   - Download free HDRI from [Poly Haven](https://polyhaven.com/hdris)
   - Import `.hdr` file into Unity
   - Create new Material: Shader → Skybox → Panoramic
   - Assign HDRI to "Spherical (HDR)" slot
   - Window → Rendering → Lighting → Skybox Material → Select HDRI material

4. **Enable post-processing** (optional):
   - Window → Package Manager → Install "Post Processing"
   - Camera → Add Component → Post-process Layer
   - Volume → Add Component → Post-process Volume

---

## ROS 2 Humble Issues

### Issue 10: "ROS 2 Command Not Found"

**Symptom**: `ros2` command returns "command not found"

**Solutions**:

1. **Source ROS 2 setup**:
```bash
source /opt/ros/humble/setup.bash

# Add to .bashrc for persistence
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

2. **Verify ROS 2 installation**:
```bash
dpkg -l | grep ros-humble
# Should show multiple ros-humble-* packages
```

3. **If not installed**, follow [Installation Guide](./installation-guide.md#step-1-install-ros-2-humble)

---

### Issue 11: ros_gz_bridge Topics Not Appearing

**Symptom**: Running `ros2 topic list` doesn't show Gazebo topics

**Causes**:
- ros_gz_bridge not running
- Wrong topic names in bridge config
- ROS 2 and Gazebo using different RMW implementations

**Solutions**:

1. **Verify ros_gz_bridge is installed**:
```bash
ros2 pkg list | grep ros_gz
# Should show: ros_gz_bridge
```

2. **Run bridge manually**:
```bash
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

3. **Check Gazebo topics**:
```bash
gz topic -l
# Shows all Gazebo topics
```

4. **Verify topic name mapping**:
```bash
# Gazebo topic
gz topic -e -t /world/my_world/model/my_robot/link/lidar_link/sensor/lidar/scan

# ROS 2 topic (after bridge)
ros2 topic echo /scan
```

---

## Cross-Tool Integration Issues

### Issue 12: Unity and Gazebo Show Different Robot Poses

**Symptom**: Robot position/orientation differs between Unity and Gazebo

**Causes**:
- Coordinate system mismatch (ROS Z-up vs. Unity Y-up)
- Joint states not synchronized
- TF frames not published

**Solutions**:

1. **Enable axis conversion in Unity URDF Importer**:
   - Robotics → Import URDF → Settings → Axis Conversion: ✅ Y-up

2. **Verify joint states are published**:
```bash
ros2 topic echo /joint_states
# Should show current joint positions
```

3. **Check Unity joint state subscriber**:
```csharp
// In Unity C# script
void Start()
{
    ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_states", UpdateJoints);
}
```

4. **Verify TF frames**:
```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf showing TF tree
```

---

### Issue 13: Sensor Data Inconsistent Between Gazebo and Unity

**Symptom**: LiDAR or camera data differs significantly between tools

**Causes**:
- Different sensor parameters (FOV, range, resolution)
- Different update rates
- Noise models not matched

**Solutions**:

1. **Compare sensor configurations**:

**Gazebo (URDF)**:
```xml
<sensor name="lidar" type="gpu_lidar">
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.2</min>
      <max>10.0</max>
    </range>
  </lidar>
</sensor>
```

**Unity (ROS TCP Connector)**:
```csharp
// Ensure Unity sensor matches Gazebo parameters
public float minRange = 0.2f;
public float maxRange = 10.0f;
public int samples = 360;
public float updateRate = 10.0f;
```

2. **Accept reasonable tolerances**:
   - LiDAR: ±5% range difference acceptable
   - Depth camera: ±10% depth difference acceptable
   - IMU: ±1° orientation difference acceptable

---

## Performance Optimization

### Issue 14: Slow Performance in Gazebo + Unity + ROS 2

**Symptom**: All tools running simultaneously causes lag

**Solutions**:

1. **Run tools on separate machines/cores**:
   - Machine 1: Gazebo (physics simulation)
   - Machine 2: Unity (visualization)
   - Both connected via ROS 2 network

2. **Reduce sensor update rates**:
   - LiDAR: 5-10 Hz (not 30 Hz)
   - Cameras: 10-15 Hz (not 30 Hz)
   - IMU: 50-100 Hz (not 200 Hz)

3. **Use headless Gazebo for algorithm testing**:
```bash
gz sim -s world.sdf  # Server-only mode (no GUI)
```

4. **Optimize Unity rendering**:
   - Edit → Project Settings → Quality → Level: Medium
   - Reduce shadow distance
   - Disable post-processing during development

---

## WSL2-Specific Issues

### Issue 15: WSL2 IP Address Changes After Reboot

**Symptom**: Unity can't connect to ROS 2 after Windows reboot

**Cause**: WSL2 uses dynamic IP addressing

**Solutions**:

1. **Find new WSL2 IP after each reboot**:
```bash
ip addr show eth0 | grep inet | awk '{print $2}' | cut -d/ -f1
```

2. **Update Unity ROS Settings** with new IP

3. **Alternative: Use static IP script** (advanced):
```powershell
# Windows PowerShell (run as Administrator)
wsl -d Ubuntu-22.04 -u root ip addr add 192.168.50.2/24 broadcast 192.168.50.255 dev eth0 label eth0:1
```

---

## Docker-Specific Issues

### Issue 16: Gazebo GUI Not Working in Docker

**Symptom**: Gazebo launches but GUI doesn't appear

**Cause**: Docker container doesn't have access to host display

**Solutions**:

1. **Enable X11 forwarding** (Linux host only):
```bash
xhost +local:docker

docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  osrf/ros:humble-desktop \
  bash
```

2. **Use headless mode**:
```bash
gz sim -s world.sdf  # Server-only, no GUI
```

3. **Windows/Mac**: Use VNC or X server (VcXsrv, XQuartz)

---

## When to Seek Additional Help

If issues persist after trying solutions above:

1. **Check official documentation**:
   - [ROS 2 Humble Docs](https://docs.ros.org/en/humble/)
   - [Gazebo Harmonic Docs](https://gazebosim.org/docs/harmonic)
   - [Unity Robotics Hub](https://github.com/Unity-Technologies/Unity-Robotics-Hub)

2. **Search GitHub issues**:
   - [Gazebo Issues](https://github.com/gazebosim/gz-sim/issues)
   - [Unity Robotics Hub Issues](https://github.com/Unity-Technologies/Unity-Robotics-Hub/issues)

3. **Community forums**:
   - [ROS Discourse](https://discourse.ros.org/)
   - [Gazebo Community](https://community.gazebosim.org/)
   - [Unity Forums](https://forum.unity.com/)

4. **Provide detailed information when asking for help**:
   - Operating system and version
   - ROS 2 / Gazebo / Unity versions
   - Full error messages (copy-paste, not screenshots)
   - Minimal reproducible example (URDF, world file, etc.)
   - Steps already attempted

---

## Quick Reference: Common Commands

### Debugging Commands

```bash
# ROS 2
ros2 topic list
ros2 topic echo /topic_name
ros2 node list
ros2 doctor

# Gazebo
gz topic -l
gz model -m model_name -i
gz sim --version

# Unity (verify ROS connection)
# Check Unity Console (Window → General → Console)

# TF frames
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo source_frame target_frame

# Network
ping <IP_ADDRESS>
netstat -tuln | grep 10000
```

### Environment Verification Checklist

```bash
# ROS 2 installed?
ros2 --version

# Gazebo Harmonic installed?
gz sim --version

# ros_gz_bridge installed?
ros2 pkg list | grep ros_gz

# Unity 2022 LTS installed?
# Check Unity Hub → Installs

# DISPLAY set (WSL2)?
echo $DISPLAY

# ROS 2 sourced?
echo $ROS_DISTRO  # Should output: humble
```

---

**Return to**: [Module 2 Overview](./index.md) | [Installation Guide](./installation-guide.md)
