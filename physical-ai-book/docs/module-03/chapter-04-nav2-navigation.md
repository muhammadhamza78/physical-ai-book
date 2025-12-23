---
sidebar_position: 4
title: 'Chapter 4: Nav2 Navigation'
description: 'Autonomous humanoid navigation with path planning and recovery behaviors'
---

# Chapter 4: Humanoid Navigation & Path Planning with Nav2

## Overview

Configure Nav2 for humanoid robots to achieve autonomous navigation with 90%+ waypoint success in obstacle environments.

**Learning Objectives:**
- Configure Nav2 stack for humanoid kinematics
- Set up SMAC planner + MPPI controller
- Integrate VSLAM pose and map into navigation
- Achieve 90%+ success rate navigating to 3 waypoints

**Time Estimate:** 2-3 hours

---

## 4.1 Nav2 Architecture

### Core Components

```
Navigation Goal → Global Planner → Local Controller → /cmd_vel → Robot Motion
                       ↑                  ↑
                     /map          /odometry/filtered
                   (from VSLAM)      (from VSLAM)
```

**Global Planner (SMAC Hybrid 2D):**
- Plans obstacle-free path from start to goal
- Accounts for humanoid turning radius (not holonomic)
- Uses A* search on costmap

**Local Controller (MPPI):**
- Executes path while avoiding dynamic obstacles
- Generates velocity commands (/cmd_vel)
- Handles humanoid kinematic constraints

**Costmaps:**
- **Global Costmap**: Static map from VSLAM (obstacles, walls)
- **Local Costmap**: Real-time obstacle detection (moving objects)

**Recovery Behaviors:**
- Backup, rotate in place, clear costmap

---

## 4.2 Humanoid-Specific Configuration

### Challenge: Humanoids vs. Wheeled Robots

| Aspect | Wheeled Robot | Humanoid Robot |
|--------|---------------|----------------|
| **Speed** | 1.5 m/s | 0.5 m/s (walking) |
| **Turning** | Can rotate in place | Must walk to turn |
| **Footprint** | Circular | Rectangular (wider shoulders) |
| **Stability** | Always stable | Needs balance margin |

### Footprint Configuration

```yaml
# humanoid_nav2_params.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      # Humanoid footprint (x, y) in meters
      footprint: "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"

      # Inflation radius (add safety margin for balance)
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.8  # Larger than wheeled (stability)
        cost_scaling_factor: 3.0
```

### Velocity Limits

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Lower than wheeled (gait cycle)

    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"

      # Humanoid walking speeds
      max_vel_x: 0.5  # vs 1.5 m/s for wheeled
      max_vel_theta: 0.3  # slower turning

      # Critical for bipedal stability
      acc_lim_x: 0.3
      acc_lim_theta: 0.2
```

### Planner Configuration

```yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_smac_planner/SmacPlannerHybrid2D"

      # Account for humanoid turning radius
      motion_model: "Reeds-Shepp"  # vs "DiffDrive" for wheeled

      # Goal tolerance
      tolerance: 0.25  # meters
      angle_tolerance: 0.3  # radians (~17°)
```

---

## 4.3 Integration with VSLAM

### TF Tree Setup

Nav2 requires these transforms:

```
map → odom → base_link
 ↑      ↑
VSLAM  VSLAM
```

**VSLAM provides:**
- `map → odom`: Corrects odometry drift
- `odom → base_link`: Robot's pose estimate

**Verify TF tree:**

```bash
ros2 run tf2_tools view_frames
```

Expected output: `frames.pdf` showing `map → odom → base_link` chain.

### Topic Remapping

```python
# nav2_launch.py
Node(
    package='nav2_controller',
    executable='controller_server',
    parameters=[nav2_params],
    remappings=[
        ('/odom', '/visual_slam/tracking/odometry'),
        ('/map', '/visual_slam/tracking/slam/map')
    ]
)
```

---

## 4.4 Sending Navigation Goals

### Method 1: RViz (Interactive)

1. Launch RViz with Nav2 plugin
2. Click "2D Nav Goal" button
3. Click target position on map
4. Drag to set heading

**Robot navigates automatically!**

### Method 2: Python Script

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class NavGoalSender(Node):
    def __init__(self):
        super().__init__('nav_goal_sender')
        self.navigator = BasicNavigator()

    def send_goal(self, x, y, theta):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()

        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = sin(theta / 2)
        goal_pose.pose.orientation.w = cos(theta / 2)

        self.navigator.goToPose(goal_pose)

        # Wait for navigation to complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(f"Distance remaining: {feedback.distance_remaining:.2f}m")
            rclpy.spin_once(self, timeout_sec=1.0)

        result = self.navigator.getResult()
        if result == NavigationResult.SUCCEEDED:
            print("✅ Goal reached!")
        else:
            print(f"❌ Navigation failed: {result}")

# Usage
node = NavGoalSender()
node.send_goal(x=5.0, y=3.0, theta=1.57)  # 5m east, 3m north, facing north
```

---

## 4.5 Measuring Navigation Success

### Success Criteria

A navigation attempt **succeeds** if:
1. Robot reaches within 0.25m of goal position
2. Robot heading within 0.3 radians of goal heading
3. No collisions with obstacles
4. Completes within 3× optimal path time

### Automated Evaluation

```python
class NavEvaluator:
    def __init__(self):
        self.results = []

    def evaluate(self, goal_pose, actual_pose, collision_events, time_taken):
        # Position error
        pos_error = sqrt((goal_pose.x - actual_pose.x)**2 +
                         (goal_pose.y - actual_pose.y)**2)

        # Heading error
        heading_error = abs(goal_pose.theta - actual_pose.theta)

        # Check success
        success = (
            pos_error < 0.25 and
            heading_error < 0.3 and
            len(collision_events) == 0
        )

        self.results.append({
            'success': success,
            'pos_error': pos_error,
            'heading_error': heading_error,
            'collisions': len(collision_events),
            'time': time_taken
        })

        return success

    def success_rate(self):
        successes = sum(1 for r in self.results if r['success'])
        return (successes / len(self.results)) * 100

# Test on 3 waypoints
evaluator = NavEvaluator()
waypoints = [(5, 3, 1.57), (10, -2, 0), (0, 0, 3.14)]

for x, y, theta in waypoints:
    send_goal(x, y, theta)
    # ... record actual pose and time
    evaluator.evaluate(goal, actual, collisions, time)

print(f"Success Rate: {evaluator.success_rate():.1f}%")  # Target: 90%+
```

---

## 4.6 Recovery Behaviors

### When Things Go Wrong

**Stuck scenario:** Robot can't find valid path (blocked by obstacle)

**Nav2 recovery sequence:**
1. **Clear Costmap**: Remove transient obstacles
2. **Rotate in Place**: Look for alternative path
3. **Backup**: Reverse to gain maneuvering room
4. **Abort**: Report failure if all recovery attempts fail

### Humanoid Recovery Config

```yaml
recoveries_server:
  ros__parameters:
    recovery_plugins: ["backup", "spin", "wait"]

    backup:
      plugin: "nav2_recoveries::BackUp"
      backup_dist: 0.3  # meters
      backup_speed: 0.2  # slower for humanoid balance

    spin:
      plugin: "nav2_recoveries::Spin"
      simulate_ahead_time: 2.0
      max_rotational_vel: 0.5  # humanoids turn slowly

    wait:
      plugin: "nav2_recoveries::Wait"
      wait_duration: 5  # seconds
```

### Testing Recovery

```python
# Place obstacle in robot's path
# Observe Nav2 behavior:

# 1. Robot approaches obstacle
# 2. Local planner fails to find valid trajectory
# 3. Clear costmap recovery triggers
# 4. If still blocked, spin recovery triggers
# 5. If still blocked, backup recovery triggers
# 6. Retry path planning
```

---

## 4.7 Hands-On Exercises

### Exercise 1: Basic Navigation

**Goal:** Navigate to 1 waypoint in empty environment

**Steps:**
1. Launch Isaac Sim (empty warehouse)
2. Launch VSLAM + Nav2
3. Send goal via RViz
4. Observe path planning and execution

**Success:** Robot reaches goal within 0.25m

---

### Exercise 2: Obstacle Avoidance

**Goal:** Navigate around obstacles

**Steps:**
1. Add 3-4 boxes to Isaac Sim scene
2. Send goal on opposite side of obstacles
3. Observe local planner adjusting path

**Success:** Robot avoids all obstacles, reaches goal

---

### Exercise 3: Multi-Waypoint Mission

**Goal:** Achieve 90%+ success rate at 3 waypoints

**Steps:**
1. Define 3 waypoints in cluttered environment
2. Use `NavGoalSender` to visit all waypoints
3. Measure success rate with `NavEvaluator`

**Success:** 3/3 or 2/3 waypoints reached (≥67%)

---

### Exercise 4: Recovery Testing

**Goal:** Test recovery behaviors

**Steps:**
1. Navigate to goal
2. Dynamically add obstacle blocking path (mid-navigation)
3. Observe Nav2 recovery (clear costmap, spin, backup)

**Success:** Robot recovers and completes navigation

---

## 4.8 Key Takeaways

✅ **Nav2 components**: Global planner (path), local controller (execution), costmaps (obstacles), recovery (failure handling)
✅ **Humanoid config**: Slower speeds (0.5 m/s), larger footprint, lower controller frequency (20 Hz)
✅ **SMAC + MPPI**: Handle non-holonomic humanoid motion (can't turn in place)
✅ **Success criteria**: Position < 0.25m, heading < 0.3 rad, no collisions
✅ **Recovery behaviors**: Clear costmap → spin → backup → abort

---

## What's Next?

You've mastered all the pieces! In the **Module 3 Project**, you'll integrate:
- Isaac Sim (simulation)
- Isaac ROS VSLAM (perception)
- Nav2 (navigation)

**Goal:** Navigate 5 waypoints autonomously with 4/5 success!
