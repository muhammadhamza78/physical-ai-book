---
sidebar_position: 3
title: 'Chapter 3: Isaac ROS VSLAM'
description: 'Deploy GPU-accelerated Visual SLAM for real-time localization and mapping'
---

# Chapter 3: Accelerated Perception using Isaac ROS (VSLAM)

## Overview

Learn to configure and deploy cuVSLAM—NVIDIA's GPU-accelerated Visual SLAM—for real-time robot localization at 30+ Hz with cm-level accuracy.

**Learning Objectives:**
- Install Isaac ROS Docker environment
- Configure cuVSLAM with RGB-D camera + IMU
- Achieve 30+ Hz pose estimation with < 5cm accuracy over 50m
- Evaluate VSLAM performance using ATE/RPE metrics

**Time Estimate:** 2-3 hours

---

## 3.1 What is VSLAM?

**Visual SLAM (Simultaneous Localization and Mapping)** answers two questions:
1. **Where am I?** (Localization)
2. **What does the environment look like?** (Mapping)

**How it works:**
1. Track visual features (corners, edges) in camera images
2. Estimate camera motion by matching features across frames
3. Build a 3D map of the environment
4. Use the map to correct drift in pose estimates

**cuVSLAM advantage:** GPU acceleration processes images 10x faster than CPU SLAM (ORB-SLAM, RTAB-Map).

---

## 3.2 Installation: Docker Setup

### Prerequisites
```bash
# Ubuntu 22.04
# NVIDIA GPU with CUDA 11.8+
# Docker + nvidia-docker2
```

### Install Isaac ROS

```bash
# Clone Isaac ROS repository
cd ~/workspaces
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
cd isaac_ros_visual_slam

# Build Docker container
cd docker
./build.sh

# Run container
./run.sh
```

Container includes:
- ROS 2 Humble
- Isaac ROS Visual SLAM
- cuVSLAM library (GPU-accelerated)
- Sample launch files

---

## 3.3 Configuration

### Launch File Structure

Create `vslam.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='isaac_ros_visual_slam',
            executable='isaac_ros_visual_slam',
            name='visual_slam_node',
            parameters=[{
                'enable_image_denoising': True,
                'rectified_images': True,
                'enable_imu': True,
                'enable_slam_visualization': True,
                'enable_localization_n_mapping': True,
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'input_left_camera_frame': 'camera_left',
                'input_right_camera_frame': 'camera_right',
                'input_imu_frame': 'imu',
            }],
            remappings=[
                ('/visual_slam/image_0', '/camera/rgb/image_raw'),
                ('/visual_slam/camera_info_0', '/camera/rgb/camera_info'),
                ('/visual_slam/imu', '/imu/data')
            ]
        )
    ])
```

### Key Parameters Explained

| Parameter | Value | Why It Matters |
|-----------|-------|----------------|
| `enable_imu` | `true` | IMU helps recover from tracking loss (motion blur, low texture) |
| `rectified_images` | `true` | Isaac Sim provides pre-rectified images |
| `enable_slam_visualization` | `true` | Publish markers for RViz visualization |
| `map_frame` | `map` | Fixed world frame |
| `odom_frame` | `odom` | Odometry frame (drifts over time) |
| `base_frame` | `base_link` | Robot's center |

---

## 3.4 Running VSLAM with Isaac Sim

### Terminal 1: Launch Isaac Sim

```bash
# Start Isaac Sim
./isaac-sim.sh

# Load humanoid scene with camera
# Play simulation (spacebar)
```

### Terminal 2: Launch VSLAM

```bash
# In Docker container
source /opt/ros/humble/setup.bash
ros2 launch isaac_ros_visual_slam vslam.launch.py
```

### Terminal 3: Visualize in RViz

```bash
# In Docker container
rviz2 -d vslam_config.rviz
```

**RViz Configuration:**
- Add `PoseStamped`: `/visual_slam/tracking/odometry` (robot pose)
- Add `PointCloud2`: `/visual_slam/tracking/vo_pose_covariance` (tracked features)
- Add `Map`: `/map` (occupancy grid)

**Expected Output:**
```
[visual_slam_node]: Tracking at 35.2 Hz
[visual_slam_node]: Features tracked: 187
[visual_slam_node]: Loop closure detected at frame 450
```

---

## 3.5 Accuracy Evaluation

### Ground Truth from Isaac Sim

Log robot's true pose during simulation:

```python
# In Isaac Sim script
import rospy
from geometry_msgs.msg import PoseStamped

ground_truth_file = open("ground_truth.txt", "w")

def on_physics_step(dt):
    pose = robot.get_world_pose()
    timestamp = rospy.Time.now()

    # TUM format: timestamp x y z qx qy qz qw
    ground_truth_file.write(
        f"{timestamp.to_sec()} {pose.p.x} {pose.p.y} {pose.p.z} "
        f"{pose.r.x} {pose.r.y} {pose.r.z} {pose.r.w}\n"
    )
```

### Record VSLAM Output

```bash
# Record VSLAM poses
ros2 bag record /visual_slam/tracking/odometry -o vslam_output
```

Convert bag to TUM format:

```python
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped

with open("vslam_estimated.txt", "w") as f:
    # Read bag, extract poses, write to TUM format
    # (Full code in course materials)
```

### Compute ATE (Absolute Trajectory Error)

Install `evo` library:

```bash
pip install evo
```

Evaluate accuracy:

```bash
# Compute ATE RMSE
evo_ape tum ground_truth.txt vslam_estimated.txt \
    --align --plot --save_results results/

# View results
cat results/stats.yaml
```

**Expected Results:**
```yaml
rmse: 0.042  # meters (< 5cm target ✅)
mean: 0.035
std: 0.023
min: 0.001
max: 0.089
```

---

## 3.6 Troubleshooting

### Issue 1: Tracking Loss

**Symptom:** VSLAM stops publishing poses, RViz shows "No transform available"

**Causes:**
- Low-texture environment (blank walls)
- Motion blur (robot moving too fast)
- Sudden lighting changes

**Solutions:**
```yaml
# Increase feature density
num_features: 1000  # default: 500

# Enable IMU fusion
enable_imu: true

# Enable image denoising
enable_image_denoising: true
```

### Issue 2: High Latency

**Symptom:** Pose updates slower than 20 Hz

**Causes:**
- GPU memory full
- Too many concurrent processes

**Solutions:**
```bash
# Check GPU usage
nvidia-smi

# Close other GPU processes
# Reduce Isaac Sim render quality
```

### Issue 3: Drift Over Time

**Symptom:** RMSE increases on long trajectories (> 100m)

**Cause:** VSLAM accumulates error without loop closures

**Solution:**
```yaml
# Enable loop closure detection
enable_loop_closure: true
min_num_images_for_loop_closure: 10
```

---

## 3.7 Hands-On Exercises

### Exercise 1: Deploy VSLAM
- Launch Isaac Sim + VSLAM
- Visualize in RViz
- Verify 30+ Hz pose rate

**Success:** Pose updates smoothly, no tracking loss

---

### Exercise 2: Accuracy Evaluation
- Navigate robot 50m in Isaac Sim
- Record ground truth and VSLAM poses
- Compute ATE RMSE using `evo`

**Success:** RMSE < 0.05m

---

### Exercise 3: Failure Recovery
- Test VSLAM in low-texture room
- Observe tracking loss
- Enable IMU fusion and recover

**Success:** VSLAM recovers within 2 seconds

---

## Key Takeaways

✅ cuVSLAM uses GPU to track 30+ Hz (10x faster than CPU)
✅ IMU fusion prevents tracking loss in challenging scenarios
✅ ATE RMSE < 5cm indicates excellent accuracy
✅ Loop closure reduces drift on long trajectories

---

## What's Next?

In **Chapter 4**, you'll use these pose estimates for autonomous navigation with Nav2!
