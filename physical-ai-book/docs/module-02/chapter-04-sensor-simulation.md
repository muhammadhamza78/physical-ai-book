---
id: chapter-04-sensor-simulation
title: "Chapter 4: Sensor Simulation & Validation"
sidebar_label: "Ch 4: Sensor Simulation"
sidebar_position: 5
description: "Simulate LiDAR, depth cameras, and IMU sensors in Gazebo with ROS 2 integration and cross-tool validation"
keywords: [lidar, depth camera, imu, sensor simulation, gazebo, rviz, ros 2, sensor plugins, point cloud]
---

# Chapter 4: Sensor Simulation & Validation

## Learning Objectives

By the end of this chapter, you will be able to:

- ✅ Add LiDAR, depth camera, and IMU plugins to Gazebo URDF models
- ✅ Configure sensor parameters (update rate, resolution, noise)
- ✅ Visualize sensor data in RViz (point clouds, images, IMU)
- ✅ Understand ROS 2 sensor message types and TF frames
- ✅ Validate sensor consistency across Gazebo and Unity

**Estimated Time**: 2 hours

---

## Why Simulate Sensors?

Before deploying perception algorithms on real hardware, sensor simulation enables:

- 🧪 **Algorithm Development**: Test without physical sensors
- 🎯 **Edge Case Testing**: Simulate rare scenarios (low light, occlusions)
- 📊 **Training Data Generation**: Create labeled datasets for ML
- 💰 **Cost Reduction**: Validate before purchasing expensive sensors

---

## Gazebo Sensor Plugins Overview

Gazebo provides sensor plugins that publish data to ROS 2 topics.

### Supported Sensors

| Sensor Type | Plugin Name | ROS 2 Message Type | Use Case |
|-------------|-------------|--------------------|----------|
| **LiDAR/Laser** | `gazebo_ros_ray_sensor` | `sensor_msgs/LaserScan` | Navigation, obstacle detection |
| **RGB Camera** | `gazebo_ros_camera` | `sensor_msgs/Image` | Visual perception, object detection |
| **Depth Camera** | `gazebo_ros_camera` (depth mode) | `sensor_msgs/Image` | 3D perception, depth estimation |
| **IMU** | `gazebo_ros_imu_sensor` | `sensor_msgs/Imu` | Orientation, acceleration, balance |
| **GPS** | `gazebo_ros_gps_sensor` | `sensor_msgs/NavSatFix` | Outdoor localization |

---

## LiDAR Configuration

### Adding LiDAR Plugin to URDF

```xml
<link name="lidar_link">
  <visual>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <cylinder radius="0.05" length="0.04"/>
    </geometry>
  </collision>
</link>

<joint name="lidar_joint" type="fixed">
  <parent link="base_link"/>
  <child link="lidar_link"/>
  <origin xyz="0.0 0.0 0.3" rpy="0 0 0"/>
</joint>

<gazebo reference="lidar_link">
  <sensor name="lidar" type="gpu_lidar">
    <topic>/scan</topic>
    <update_rate>10</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1.0</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.2</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </lidar>
  </sensor>
</gazebo>
```

### Key LiDAR Parameters

| Parameter | Description | Example Value |
|-----------|-------------|---------------|
| **samples** | Number of rays per scan | 360 (1° resolution) |
| **min_angle / max_angle** | Horizontal FOV | -π to π (360°) |
| **min / max range** | Detection distance | 0.2m to 10m |
| **update_rate** | Scans per second | 10 Hz |
| **noise stddev** | Gaussian noise std deviation | 0.01m |

### Visualizing LiDAR in RViz

```bash
# Terminal 1: Launch Gazebo
gz sim world_with_lidar.sdf

# Terminal 2: Launch RViz
ros2 run rviz2 rviz2

# In RViz:
# Add → LaserScan
# Topic: /scan
# Fixed Frame: base_link
```

---

## Depth Camera Configuration

### Adding Depth Camera Plugin to URDF

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.1 0.05"/>
    </geometry>
  </visual>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head_link"/>
  <child link="camera_link"/>
  <origin xyz="0.1 0.0 0.0" rpy="0 0 0"/>
</joint>

<gazebo reference="camera_link">
  <sensor name="depth_camera" type="depth_camera">
    <update_rate>15</update_rate>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <camera_name>depth_camera</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

### Published Topics

- `/depth_camera/rgb/image_raw`: RGB image
- `/depth_camera/depth/image_raw`: Depth image (meters)
- `/depth_camera/camera_info`: Camera calibration

### Key Depth Camera Parameters

| Parameter | Description | Example Value |
|-----------|-------------|---------------|
| **horizontal_fov** | Field of view (radians) | 1.047 (60°) |
| **image width/height** | Resolution | 640x480 |
| **clip near/far** | Depth range | 0.1m to 10m |
| **update_rate** | Frames per second | 15 Hz |

---

## IMU Configuration

### Adding IMU Plugin to URDF

```xml
<link name="imu_link">
  <visual>
    <geometry>
      <box size="0.02 0.02 0.01"/>
    </geometry>
  </visual>
</link>

<joint name="imu_joint" type="fixed">
  <parent link="torso"/>
  <child link="imu_link"/>
  <origin xyz="0.0 0.0 0.05" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.1</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_controller" filename="libgazebo_ros_imu_sensor.so">
      <topic_name>/imu</topic_name>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### IMU Data Fields

```
sensor_msgs/Imu:
  - orientation (quaternion): Current rotation
  - angular_velocity (rad/s): Rotation rate (gyroscope)
  - linear_acceleration (m/s²): Acceleration (accelerometer)
```

### Key IMU Parameters

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| **angular_velocity noise** | Gyroscope noise | 0.01 rad/s |
| **linear_acceleration noise** | Accelerometer noise | 0.1 m/s² |
| **update_rate** | Measurements per second | 100 Hz |

---

## Sensor Noise Modeling

Real-world sensors have noise. Gazebo simulates this for realistic algorithm testing.

### Noise Types

1. **Gaussian Noise**: Random variation (most common)
2. **Bias**: Constant offset
3. **Outliers**: Occasional bad readings

### Realistic vs. Ideal Sensors

| Sensor | Ideal (No Noise) | Realistic (With Noise) |
|--------|------------------|------------------------|
| **LiDAR** | Perfect distances | ±1cm Gaussian noise |
| **Depth Camera** | Perfect depth | ±5cm Gaussian noise, missing data |
| **IMU** | Perfect orientation | Drift, bias, 0.1 m/s² noise |

**Recommendation**: Start with ideal sensors for debugging, then add noise for realistic testing.

---

## TF (Transform) Frames

Sensors require TF frames to define their position and orientation relative to the robot.

### TF Tree Example

```
map
 └── odom
      └── base_link
           ├── lidar_link
           ├── camera_link
           └── imu_link
```

### Checking TF Frames

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Echo specific transform
ros2 run tf2_ros tf2_echo base_link lidar_link
```

### Common TF Issues

| Issue | Symptom | Solution |
|-------|---------|----------|
| **Missing frame** | RViz shows "No transform" error | Verify sensor plugin publishes TF |
| **Wrong orientation** | Sensor data appears rotated | Check URDF joint orientation |
| **Time synchronization** | Old transform errors | Ensure all nodes use same clock |

---

## Visualizing Sensor Data in RViz

### RViz Configuration

```bash
ros2 run rviz2 rviz2
```

**Add Displays**:
1. **LaserScan** (for LiDAR):
   - Topic: `/scan`
   - Size: 0.05
   - Color: By intensity

2. **Image** (for cameras):
   - Topic: `/depth_camera/rgb/image_raw`
   - Topic: `/depth_camera/depth/image_raw`

3. **Imu** (for IMU):
   - Topic: `/imu`
   - Display orientation as arrow

4. **RobotModel**:
   - Description Topic: `/robot_description`
   - TF Prefix: (blank)

**Set Fixed Frame**: `base_link` or `odom`

---

## Cross-Tool Sensor Validation

### Validating Gazebo and Unity Consistency

**Goal**: Ensure sensor data is similar across both tools (within tolerance).

### Validation Checklist

| Sensor | Gazebo Output | Unity Output | Expected Difference |
|--------|---------------|--------------|---------------------|
| **LiDAR** | Point cloud on `/scan` | Rendered rays in Unity | `<5%` range difference |
| **Depth Camera** | Depth image | Unity depth buffer | `<10%` depth difference |
| **IMU** | Orientation quaternion | Unity transform rotation | `<1°` orientation difference |

### Debugging Discrepancies

1. **Frame Rate Mismatch**: Gazebo 10Hz, Unity 30Hz → Interpolation needed
2. **Coordinate System**: ROS Z-up vs. Unity Y-up → Verify axis transform
3. **Sensor Configuration**: Different FOV, range, resolution → Align parameters

---

## Exercises

### Exercise 4.1: Add LiDAR Plugin, Visualize in RViz

**Objective**: Configure LiDAR sensor and visualize point cloud.

**Steps**:
1. Add LiDAR plugin to humanoid head URDF
2. Launch Gazebo with the model
3. Launch RViz and add LaserScan display
4. Verify point cloud updates at 10Hz

**Validation Criteria**:
- ✅ LiDAR publishes to `/scan` topic
- ✅ Point cloud visible in RViz
- ✅ Update rate is 10Hz

### Exercise 4.2: Add Depth Camera, Subscribe to RGB/Depth Topics

**Objective**: Configure depth camera and view RGB and depth images.

**Steps**:
1. Add depth camera plugin to URDF
2. Launch Gazebo
3. Use `ros2 topic echo` or RViz Image display
4. View both RGB and depth images

**Validation Criteria**:
- ✅ RGB image publishes to `/depth_camera/rgb/image_raw`
- ✅ Depth image publishes to `/depth_camera/depth/image_raw`
- ✅ Images update at 15Hz

### Exercise 4.3: Add IMU Plugin, Validate Data

**Objective**: Configure IMU sensor and verify orientation/acceleration data.

**Steps**:
1. Add IMU plugin to torso link
2. Launch Gazebo
3. Subscribe to `/imu` topic
4. Move robot and observe IMU data changes

**Validation Criteria**:
- ✅ IMU publishes orientation (quaternion)
- ✅ Angular velocity and linear acceleration present
- ✅ Noise characteristics match configuration

### Exercise 4.4: Cross-Tool Sensor Validation

**Objective**: Compare sensor outputs between Gazebo and Unity.

**Steps**:
1. Configure LiDAR in both Gazebo and Unity
2. Capture data from both tools
3. Compare range measurements
4. Document differences and explain causes

**Validation Criteria**:
- ✅ Sensor data consistent within 5-10% tolerance
- ✅ Differences explained (frame rate, noise, coordinate systems)

---

## Key Takeaways

1. **Sensor plugins** add LiDAR, cameras, and IMU to Gazebo simulations
2. **Configuration parameters** include update rate, resolution, FOV, and noise
3. **ROS 2 message types**: LaserScan, Image, Imu are standard sensor formats
4. **TF frames** define sensor position/orientation relative to robot
5. **RViz** visualizes sensor data (point clouds, images, orientations)
6. **Cross-tool validation** ensures sensor consistency across Gazebo and Unity

---

## Next Steps

**Module 2 Project**: [Building and Evaluating a Humanoid Digital Twin](./module-02-project.md)

Apply everything you've learned to create an integrative digital twin: URDF model, Gazebo physics testing, Unity visualization, and sensor integration.
