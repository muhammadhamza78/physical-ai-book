---
id: module-02-project
title: "Module 2 Project: Building a Humanoid Digital Twin"
sidebar_label: "Module Project"
sidebar_position: 6
description: "Integrative capstone project combining Gazebo physics, Unity visualization, and sensor simulation"
keywords: [digital twin, capstone project, gazebo, unity, urdf, sensors, ros 2, integration]
---

# Module 2 Project: Building and Evaluating a Humanoid Digital Twin

## Project Overview

In this integrative capstone project, you will create a complete digital twin of a humanoid robot subsystem, demonstrating your mastery of:

- **URDF modeling** with realistic inertial properties
- **Gazebo physics simulation** with stable dynamics
- **Unity photorealistic visualization** with PBR materials and HDRI lighting
- **Sensor integration** (vision + inertial sensors)
- **ROS 2 synchronization** across Gazebo and Unity

**Estimated Time**: 3-4 hours

---

## Project Requirements

### 1. URDF Model Creation

Create a humanoid subsystem URDF (arm, torso, or leg) with:

- ✅ **At least 3 joints** (revolute or prismatic)
- ✅ **Realistic inertial properties** (mass, inertia tensor) for all links
- ✅ **Collision geometry** (simplified for performance)
- ✅ **Visual geometry** (detailed meshes or basic shapes)
- ✅ **Joint limits** (position, velocity, effort)

**Recommended Subsystems**:
- **Humanoid Arm**: Shoulder (2 DOF), elbow (1 DOF), gripper
- **Humanoid Torso**: Waist (2 DOF), neck (1 DOF)
- **Humanoid Leg**: Hip (3 DOF), knee (1 DOF), ankle (1 DOF)

### 2. Gazebo Physics Testing

Simulate your URDF in Gazebo and validate:

- ✅ **Stable physics behavior** (no unstable oscillations, no penetration through ground)
- ✅ **Gravity response** (model falls realistically, joints behave as expected)
- ✅ **Joint control** via ROS 2 topics (publish joint commands, observe motion)
- ✅ **Collision detection** with obstacles (test with a box or wall)

### 3. Sensor Integration

Add **at least 2 sensors** to your URDF:

- ✅ **One vision-based sensor**: LiDAR OR depth camera
- ✅ **One inertial sensor**: IMU

**Sensor Requirements**:
- Publish data to ROS 2 topics
- Visualize in RViz (point cloud or images)
- Validate data quality (check update rate, noise, range)

### 4. Unity Visualization

Import your URDF into Unity and create a photorealistic render:

- ✅ **Successful URDF import** with correct hierarchy
- ✅ **PBR materials** applied (metallic for metal parts, plastic for others)
- ✅ **HDRI lighting** for realistic reflections and ambient light
- ✅ **Real-time synchronization** with ROS 2 joint states (optional but recommended)

### 5. Documentation

Provide clear documentation including:

- ✅ **README.md**: Installation steps, usage instructions
- ✅ **Design Rationale**: Why you chose this subsystem, joint configuration, sensor placement
- ✅ **Gazebo Test Results**: Screenshots, physics metrics (RTF, stability observations)
- ✅ **Unity Render Screenshots**: High-quality images showing photorealistic quality
- ✅ **Sensor Data Samples**: Example LaserScan or IMU data (text or screenshots)

---

## Project Rubric (100 Points)

| Category | Points | Criteria |
|----------|--------|----------|
| **Physics Accuracy (Gazebo)** | 40 | |
| - URDF inertial properties | 10 | All links have realistic mass and inertia tensors |
| - Stable simulation | 15 | No oscillations, realistic gravity response, joints move smoothly |
| - Collision detection | 10 | Obstacles correctly detected, no penetration |
| - Joint control via ROS 2 | 5 | Commands applied successfully, expected behavior |
| **Visualization Quality (Unity)** | 30 | |
| - URDF import success | 10 | Correct hierarchy, all links and joints present |
| - PBR materials & HDRI | 10 | Photorealistic quality, appropriate material properties |
| - Real-time synchronization | 10 | Joint states update from ROS 2 (or static render if sync not implemented) |
| **Sensor Fidelity** | 20 | |
| - Vision sensor configured | 10 | LiDAR or depth camera publishes valid data |
| - IMU sensor configured | 5 | IMU publishes orientation and acceleration |
| - Cross-tool consistency | 5 | Sensor outputs reasonable and consistent (if applicable) |
| **Documentation & Clarity** | 10 | |
| - README with setup steps | 3 | Clear installation and usage instructions |
| - Design rationale | 3 | Explains URDF choices and sensor placement |
| - Test results with evidence | 4 | Screenshots, data samples, observations |
| **TOTAL** | **100** | |

### Grading Thresholds

- **90-100 points**: Excellence - Professional-quality digital twin with all features working flawlessly
- **80-89 points**: Proficient - Solid implementation with minor issues
- **70-79 points**: Passing - Core requirements met, some issues present
- **Below 70**: Not passing - Significant gaps in requirements

---

## Implementation Guidance

### Step 1: Design Your URDF (30 minutes)

1. **Choose subsystem** (arm, torso, or leg)
2. **Sketch joint configuration** (how many DOF, which axes)
3. **Estimate link dimensions and masses**
4. **Create basic URDF structure** (links, joints, no sensors yet)

**Resources**:
- Module 1: URDF fundamentals
- Chapter 2: Inertial properties, collision geometry

### Step 2: Test Physics in Gazebo (1 hour)

1. **Add inertial properties** to all links (use online calculators for cylinders/boxes)
2. **Create basic world file** (ground plane, lighting)
3. **Launch Gazebo** and verify model loads
4. **Test gravity response** (does it fall realistically?)
5. **Add ros_gz_bridge** and test joint commands
6. **Add obstacle** and test collision detection

**Debugging Tips**:
- If unstable: Increase joint damping, reduce time step
- If floating: Check inertial properties (mass > 0)
- If joints don't move: Verify ros_gz_bridge topic mapping

### Step 3: Add Sensors (45 minutes)

1. **Add vision sensor** (LiDAR or depth camera) to URDF
2. **Add IMU sensor** to torso or base link
3. **Test sensor plugins** in Gazebo (check topics with `ros2 topic list`)
4. **Visualize in RViz** (LaserScan, Image, or Imu displays)
5. **Verify data quality** (update rate, reasonable values)

### Step 4: Import into Unity (1 hour)

1. **Install Unity Robotics Hub** and URDF Importer
2. **Import URDF** (Robotics → Import URDF)
3. **Create PBR materials** (metal for arms, plastic for base)
4. **Add HDRI skybox** for environment lighting
5. **Optional**: Set up ROS TCP Endpoint and synchronize joint states
6. **Capture screenshots** for documentation

### Step 5: Document Results (30 minutes)

1. **Write README.md** (installation, usage, design rationale)
2. **Screenshot Gazebo simulation** (robot with obstacle, RViz sensor visualization)
3. **Screenshot Unity render** (multiple angles, close-ups)
4. **Save sensor data samples** (use `ros2 topic echo /scan > lidar_sample.txt`)
5. **Reflect on challenges** and lessons learned

---

## Example Project: Humanoid Arm Digital Twin

### Design Choices

**Subsystem**: 3-DOF humanoid arm (shoulder pitch, shoulder roll, elbow pitch)

**Sensors**:
- LiDAR on end effector (obstacle detection for manipulation)
- IMU on upper arm (measure arm orientation during motion)

**Gazebo Tests**:
- Gravity: Arm falls naturally when no torque applied
- Joint Control: Commanded positions reached smoothly
- Collision: Arm stops when hitting table obstacle

**Unity Visualization**:
- Materials: Brushed aluminum (metallic=0.9, smoothness=0.7)
- Lighting: Indoor HDRI (office environment)
- Synchronization: Real-time joint states from Gazebo

### Expected Outcomes

- **Gazebo RTF**: 0.8-1.0 (acceptable for 3-link arm)
- **LiDAR Range**: 0.2m to 5m, 10Hz update
- **IMU Noise**: 0.1 m/s² linear acceleration, 0.01 rad/s angular velocity
- **Unity FPS**: 30+ FPS with PBR materials

---

## Bonus Challenges (Optional)

For students seeking extra challenge, consider:

### +5 Points: HRI Scenario in Unity

Add a virtual human model and demonstrate interaction (e.g., robot handing object to human).

### +3 Points: Advanced Sensor Configuration

- Add realistic noise models to sensors
- Implement sensor fusion (combine LiDAR + IMU)

### +2 Points: Performance Optimization

- Achieve Gazebo RTF > 0.9
- Achieve Unity >60 FPS with high-quality settings

---

## Submission Requirements

Submit a single compressed archive containing:

1. **URDF file(s)** (`.urdf` or `.xacro`)
2. **Gazebo world file** (`.sdf`)
3. **README.md** (documentation)
4. **Screenshots** (Gazebo + Unity + RViz)
5. **Sensor data samples** (optional but recommended)
6. **Unity project** (optional - can be large, provide link instead)

**File Structure Example**:
```
project-submission.zip
├── README.md
├── humanoid_arm.urdf
├── test_world.sdf
├── screenshots/
│   ├── gazebo_physics.png
│   ├── unity_render_front.png
│   ├── unity_render_side.png
│   └── rviz_sensors.png
└── sensor_data/
    ├── lidar_sample.txt
    └── imu_sample.txt
```

---

## Evaluation Criteria

Your project will be evaluated on:

1. **Technical Correctness**: Does it work as specified?
2. **Physics Realism**: Are dynamics stable and realistic?
3. **Visual Quality**: Are Unity renders presentation-worthy?
4. **Sensor Integration**: Do sensors publish valid, useful data?
5. **Documentation**: Can someone else understand and reproduce your work?

---

## Tips for Success

### Do's

- ✅ Start simple (basic URDF) and iterate
- ✅ Test frequently in Gazebo before adding complexity
- ✅ Use realistic inertial properties (online calculators help)
- ✅ Document as you go (easier than retroactively)
- ✅ Ask for help early if stuck

### Don'ts

- ❌ Skip inertial properties (causes physics issues)
- ❌ Use overly complex meshes for collision (slows simulation)
- ❌ Forget to test sensors before Unity import
- ❌ Wait until the last minute to document
- ❌ Ignore validation criteria in the rubric

---

## Resources

- **Module 1**: URDF fundamentals
- **Chapter 2**: Gazebo physics simulation
- **Chapter 3**: Unity rendering
- **Chapter 4**: Sensor simulation
- **Inertia Calculators**: [https://amesweb.info/inertia/](https://amesweb.info/inertia/)
- **Unity Asset Store**: Free HDRI packs
- **ROS 2 Humble Docs**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)

---

## Frequently Asked Questions

**Q: Can I use an existing URDF from online?**
A: Yes, but you must modify it (add sensors, tune physics, document changes). Simply downloading and submitting an existing model is not acceptable.

**Q: Do I need to implement Unity synchronization?**
A: No, it's optional. A static high-quality render is sufficient for full credit. Real-time sync is a bonus.

**Q: What if my Gazebo simulation is slow (RTF < 0.5)?**
A: Simplify collision geometries, increase time step, or reduce sensor update rates.

**Q: Can I work in a team?**
A: Check with your instructor. Typically this is an individual project.

---

## Next Steps

Congratulations on completing Module 2! You now have the skills to:

- Create digital twins for robot testing and visualization
- Simulate physics accurately in Gazebo
- Create photorealistic renders in Unity
- Integrate sensors for perception testing

**Continue Learning**:
- Module 3: [Add link to next module when available]
- Advanced Topics: Multi-robot simulation, ros2_control integration, ML perception

**Share Your Work**: Consider sharing your digital twin renders on social media or your portfolio. Digital twins are impressive demonstrations of robotics skills!
