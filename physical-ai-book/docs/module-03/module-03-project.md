---
sidebar_position: 5
title: 'Module 3 Project'
description: 'Build a complete AI-driven humanoid navigation system'
---

# Module 3 Project: AI-Driven Humanoid Navigation System

## Overview

Integrate Isaac Sim, Isaac ROS VSLAM, and Nav2 into a complete autonomous navigation system. Your humanoid robot will navigate 5 waypoints in a cluttered environment using AI-driven perception.

**Project Goals:**
- Launch integrated system (Isaac Sim + VSLAM + Nav2) in < 30 seconds
- Navigate autonomously to 5 waypoints (4/5 success target)
- Measure and report performance metrics

**Time Estimate:** 6-8 hours

---

## Project Requirements

### System Integration

Your system must:
1. ✅ Launch Isaac Sim with humanoid robot in cluttered warehouse
2. ✅ Start Isaac ROS cuVSLAM for real-time localization (30+ Hz)
3. ✅ Initialize Nav2 with humanoid configuration
4. ✅ Accept navigation goals and execute autonomously
5. ✅ Handle failures with recovery behaviors

### Performance Targets

| Metric | Target | How to Measure |
|--------|--------|----------------|
| **VSLAM Accuracy (ATE RMSE)** | < 0.05m over 50m | `evo_ape` comparison with ground truth |
| **Navigation Success Rate** | ≥ 80% (4/5 waypoints) | Custom evaluator script |
| **Path Efficiency** | > 80% | Actual path length / optimal path length |
| **System Startup Time** | < 30 seconds | Time from launch to Nav2 ready |
| **GPU Utilization** | Record average % | `nvidia-smi` logging |
| **CPU Utilization** | Record average % | `htop` or `psutil` |

---

## Project Setup

### 1. Environment Preparation

**Isaac Sim Scene:**
```
Warehouse (20m x 20m):
- 10-15 boxes (obstacles)
- 3-5 shelves
- Narrow corridors (1.5m wide)
- Varying lighting (fluorescent + windows)
```

**Robot Configuration:**
- Humanoid with RGB-D camera + IMU
- Spawn position: (0, 0, 0.5)
- Initial heading: 0° (facing +X)

### 2. Waypoint Definitions

```yaml
# waypoints.yaml
waypoints:
  - {x: 5.0, y: 3.0, theta: 1.57, name: "Checkpoint A"}
  - {x: 10.0, y: -2.0, theta: 0.0, name: "Checkpoint B"}
  - {x: 15.0, y: 5.0, theta: -1.57, name: "Checkpoint C"}
  - {x: 8.0, y: 8.0, theta: 3.14, name: "Checkpoint D"}
  - {x: 0.0, y: 0.0, theta: 0.0, name: "Home"}
```

---

## Implementation

### Step 1: Integrated Launch File

Create `full_system.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Isaac ROS VSLAM
        IncludeLaunchDescription(
            'isaac_ros_visual_slam/launch/vslam.launch.py'
        ),

        # Nav2
        IncludeLaunchDescription(
            'nav2_bringup/launch/navigation_launch.py',
            launch_arguments={
                'params_file': 'humanoid_nav2_params.yaml'
            }.items()
        ),

        # Performance Monitor
        Node(
            package='module3_project',
            executable='performance_monitor',
            name='performance_monitor',
            output='screen'
        ),

        # Waypoint Navigator
        Node(
            package='module3_project',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            parameters=[{'waypoints_file': 'waypoints.yaml'}],
            output='screen'
        )
    ])
```

### Step 2: Performance Monitoring

Create `performance_monitor.py`:

```python
import rclpy
from rclpy.node import Node
import psutil
import subprocess
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        self.timer = self.create_timer(1.0, self.record_metrics)
        self.metrics = []
        self.start_time = time.time()

    def record_metrics(self):
        # CPU
        cpu_percent = psutil.cpu_percent(interval=0.1)

        # GPU
        gpu_query = subprocess.check_output([
            'nvidia-smi',
            '--query-gpu=utilization.gpu,memory.used',
            '--format=csv,noheader,nounits'
        ]).decode().strip().split(',')
        gpu_util = float(gpu_query[0])
        gpu_mem = float(gpu_query[1])

        # Store
        self.metrics.append({
            'timestamp': time.time() - self.start_time,
            'cpu_percent': cpu_percent,
            'gpu_util': gpu_util,
            'gpu_mem_mb': gpu_mem
        })

    def report(self):
        import numpy as np
        metrics_array = np.array([
            [m['cpu_percent'], m['gpu_util']] for m in self.metrics
        ])

        print("\n=== Performance Report ===")
        print(f"CPU Average: {np.mean(metrics_array[:, 0]):.1f}%")
        print(f"GPU Average: {np.mean(metrics_array[:, 1]):.1f}%")
        print(f"GPU Memory Peak: {max(m['gpu_mem_mb'] for m in self.metrics):.0f} MB")
```

### Step 3: Waypoint Navigator

Create `waypoint_navigator.py`:

```python
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml

class WaypointNavigator:
    def __init__(self, waypoints_file):
        self.navigator = BasicNavigator()
        with open(waypoints_file) as f:
            self.waypoints = yaml.safe_load(f)['waypoints']
        self.results = []

    def execute_mission(self):
        for wp in self.waypoints:
            print(f"\n🎯 Navigating to {wp['name']}...")

            goal_pose = self.create_pose(wp['x'], wp['y'], wp['theta'])
            self.navigator.goToPose(goal_pose)

            # Wait for result
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # Log progress

            result = self.navigator.getResult()
            success = (result == NavigationResult.SUCCEEDED)

            self.results.append({
                'waypoint': wp['name'],
                'success': success,
                'result': result
            })

            if success:
                print(f"✅ Reached {wp['name']}")
            else:
                print(f"❌ Failed to reach {wp['name']}: {result}")

    def success_rate(self):
        successes = sum(1 for r in self.results if r['success'])
        return (successes / len(self.results)) * 100
```

---

## Testing & Evaluation

### Phase 1: System Integration Test

**Objective:** Verify all components launch and communicate

**Steps:**
1. Launch `full_system.launch.py`
2. Check ROS topics:
   ```bash
   ros2 topic list | grep -E "(visual_slam|cmd_vel|map)"
   ```
3. Verify TF tree: `ros2 run tf2_tools view_frames`
4. Confirm Nav2 ready: `ros2 service call /is_active nav2_msgs/srv/IsActive`

**Success Criteria:**
- ✅ All nodes launch without errors
- ✅ VSLAM publishes pose at 30+ Hz
- ✅ Nav2 accepts goals

**Time Limit:** System startup < 30 seconds

---

### Phase 2: VSLAM Accuracy Test

**Objective:** Validate localization accuracy

**Steps:**
1. Navigate robot 50m around warehouse
2. Record ground truth from Isaac Sim
3. Record VSLAM output (`/visual_slam/tracking/odometry`)
4. Compute ATE RMSE:
   ```bash
   evo_ape tum ground_truth.txt vslam_estimated.txt --align --plot
   ```

**Success Criteria:**
- ✅ ATE RMSE < 0.05m

---

### Phase 3: Navigation Success Test

**Objective:** Achieve 80%+ waypoint success rate

**Steps:**
1. Run `waypoint_navigator.py`
2. Navigate all 5 waypoints
3. Record results (success/failure per waypoint)
4. Calculate success rate

**Success Criteria:**
- ✅ 4/5 or 5/5 waypoints reached (≥ 80%)

---

### Phase 4: Performance Profiling

**Objective:** Measure computational load

**Steps:**
1. Run `performance_monitor.py` during navigation
2. Record CPU and GPU usage for full mission
3. Generate report

**Success Criteria:**
- ✅ Data collected for GPU%, CPU%, memory

---

## Deliverables

### 1. Source Code
- `full_system.launch.py`
- `performance_monitor.py`
- `waypoint_navigator.py`
- `humanoid_nav2_params.yaml`
- `waypoints.yaml`

### 2. Metrics Report

Create `metrics_report.md`:

```markdown
# Module 3 Project Metrics Report

**Student Name:** [Your Name]
**Date:** [Date]

## System Integration
- Launch time: [X] seconds
- All components operational: [Yes/No]

## VSLAM Accuracy
- Trajectory length: [X] meters
- ATE RMSE: [X.XXX] meters
- Target: < 0.05m → [PASS/FAIL]

## Navigation Success
- Waypoints attempted: 5
- Waypoints reached: [X]/5
- Success rate: [XX]%
- Target: ≥ 80% → [PASS/FAIL]

## Performance Metrics
- CPU average: [XX]%
- GPU average: [XX]%
- GPU memory peak: [XXXX] MB

## Observations
[Describe challenges, recovery behaviors observed, any manual interventions]
```

### 3. Demo Video (Optional)

Record screen capture showing:
- System launch
- VSLAM tracking in RViz
- Autonomous navigation to all 5 waypoints
- Performance metrics

**Length:** 3-5 minutes

---

## Rubric

| Category | Points | Criteria |
|----------|--------|----------|
| **System Integration** | 30 | All components launch and communicate (< 30s startup) |
| **VSLAM Accuracy** | 20 | ATE RMSE < 0.05m over 50m trajectory |
| **Navigation Success** | 30 | 4/5 or 5/5 waypoints reached (80%+) |
| **Metrics Reporting** | 20 | Complete report with all metrics |
| **Total** | 100 | |

**Grading Scale:**
- A (90-100): All targets met, excellent documentation
- B (80-89): 3/4 targets met, good documentation
- C (70-79): 2/4 targets met, adequate documentation
- D (60-69): 1/4 targets met
- F (&lt;60): Major components missing or non-functional

<!-- - F (<60): Major components missing or non-functional -->

---

## Troubleshooting

### Issue: Nav2 won't accept goals

**Check:**
```bash
# Is Nav2 lifecycle active?
ros2 service call /is_active nav2_msgs/srv/IsActive

# Are costmaps updating?
ros2 topic hz /global_costmap/costmap
ros2 topic hz /local_costmap/costmap
```

### Issue: VSLAM loses tracking

**Solutions:**
- Add more visual features to environment (posters, patterns)
- Enable `enable_imu: true`
- Slow down robot motion

### Issue: Robot gets stuck

**Check recovery behaviors:**
```bash
# Monitor recovery server
ros2 topic echo /recovery_server/transition_event
```

---

## Conclusion

This project demonstrates your mastery of the Isaac ecosystem. You've integrated:
- ✅ Photorealistic simulation (Isaac Sim)
- ✅ GPU-accelerated perception (Isaac ROS VSLAM)
- ✅ Autonomous navigation (Nav2)

**Next steps:** Deploy on real hardware, explore multi-robot coordination, add manipulation!

---

## Submission

Submit via course portal:
1. Source code (ZIP file)
2. Metrics report (PDF)
3. Demo video (optional, YouTube link)

**Deadline:** [See course calendar]

Good luck! 🤖🚀
