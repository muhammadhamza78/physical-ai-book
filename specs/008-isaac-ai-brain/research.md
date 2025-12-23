# Research: Module 3: The AI-Robot Brain (NVIDIA Isaac™)

**Date**: 2025-12-23
**Feature**: `008-isaac-ai-brain`
**Purpose**: Resolve technical decisions for implementation planning

---

## Executive Summary

This research document consolidates findings from 5 research areas critical to implementing Module 3 educational content. All technical decisions (D1-D9) are now resolved with rationale based on NVIDIA Isaac ecosystem best practices, ROS 2 education patterns, and robotics curriculum design principles.

**Key Decisions**:
- **Teaching Approach**: Hybrid (GUI introduction → Code-first development)
- **Isaac ROS Deployment**: Docker containers (recommended) with native build appendix
- **Synthetic Data Workflow**: Hybrid (GUI demonstration → Replicator API scripting)
- **VSLAM Variant**: cuVSLAM (NVIDIA's GPU-accelerated implementation)
- **Nav2 Planner**: SMAC Hybrid 2D for global planning, MPPI Controller for local control
- **Hardware Requirement**: Simulation-only primary path with optional hardware extension
- **Evaluation Tooling**: Custom Python scripts wrapping evo library for VSLAM, ROS 2 nav metrics for navigation

---

## R1: Isaac Ecosystem Integration Best Practices

### Research Question
What is the recommended architecture for integrating Isaac Sim, Isaac ROS, and Nav2 in educational content?

### Findings

#### **Decision D1: Isaac Sim Teaching Approach**

**Options Considered**:
1. GUI-first (all tasks via Isaac Sim interface)
2. Code-first (Python API from start)
3. Hybrid (GUI introduction → Python API transition)

**Selected**: **Hybrid Approach**

**Rationale**:

**Phase 1: GUI Introduction (Chapter 1-2, Hours 1-4)**
- **Benefits for Learners**:
  - Familiar workflow for students coming from Gazebo GUI
  - Immediate visual feedback reduces cognitive load
  - Faster time-to-first-success (motivational)
  - Scene composition intuition before scripting

- **GUI Tasks**:
  - Import robot models (URDF/USD)
  - Configure sensors (cameras, IMU) visually
  - Adjust lighting and physics parameters
  - Run simulation and observe ROS topics in RViz

**Phase 2: Python API Transition (Chapter 2-3, Hours 5-8)**
- **Benefits for Learners**:
  - Reproducibility (scripts > manual GUI steps)
  - Automation (batch dataset generation)
  - Version control (Git-friendly Python vs binary USD files)
  - Industry-standard practice

- **Code Tasks**:
  - Script scene setup from Phase 1
  - Automate sensor configuration
  - Programmatic data collection (Replicator API)
  - Integration with ROS 2 launch files

**Teaching Progression**:
```
Chapter 1: GUI (concepts, manual exploration)
Chapter 2: Hybrid (GUI demo, then code equivalent)
Chapter 3-4: Code-first (Python API, ROS 2 integration)
Module 3 Project: Pure code (launch files, no GUI)
```

**Supporting Evidence**:
- NVIDIA's own tutorials follow this pattern (GUI quickstart → API deep dives)
- Aligns with constitution principle: "Progressive Complexity"
- Matches target audience (Gazebo GUI familiarity → professional practices)

---

#### **Decision D2: Isaac ROS Deployment**

**Options Considered**:
1. Docker containers only
2. Native build only
3. Both (Docker primary, native appendix)

**Selected**: **Docker Containers (Primary) + Native Build (Advanced Appendix)**

**Rationale**:

**Docker Containers (Recommended Path)**:

**Pros**:
- **Reproducibility**: "Works on my machine" eliminated across Ubuntu/WSL2/macOS
- **Dependency Isolation**: No conflicts with student system packages
- **GPU Compatibility**: Pre-configured CUDA/cuDNN versions tested by NVIDIA
- **Quick Setup**: Single `docker pull` vs hours of dependency hell
- **Official Support**: NVIDIA maintains `nvidia/isaac-ros` images
- **Easy Cleanup**: Delete container without affecting host system

**Cons**:
- **Initial Learning Curve**: Docker concepts (volumes, GPU passthrough, networking)
- **Performance Overhead**: ~5-10% slower than native (acceptable for education)
- **Debugging Complexity**: Harder to inspect processes (mitigated with `-it` shell access)

**Mitigation Strategies**:
1. **Pre-requisite Module**: "Docker for Robotics" (20-minute tutorial)
2. **Provide Template**:
   ```yaml
   # docker-compose.yml
   services:
     isaac-ros:
       image: nvidia/isaac-ros:humble-latest
       runtime: nvidia
       environment:
         - DISPLAY=${DISPLAY}
         - NVIDIA_VISIBLE_DEVICES=all
       volumes:
         - ./workspace:/workspace
         - /tmp/.X11-unix:/tmp/.X11-unix:rw
       network_mode: host
   ```
3. **Troubleshooting Guide**: Common issues (GPU not detected, X11 forwarding, ROS discovery)

**Native Build (Advanced Track)**:
- Documented in appendix for students who:
  - Want maximum performance
  - Need custom Isaac ROS modifications
  - Are comfortable with dependency management
- Prerequisite: Strong Linux/CMake skills

**Teaching Approach**:
- **Default**: Docker (90% of students)
- **Optional**: Native build for advanced learners

---

#### **Decision D3: Nav2 Humanoid Configuration**

**Challenge**: Nav2 is designed for differential drive robots; humanoids require adaptation.

**Selected Configuration**:

**Global Planner**: `nav2_smac_planner/SmacPlannerHybrid2D`
- Handles non-holonomic constraints
- Reeds-Shepp paths work better for humanoid turning
- Accounts for heading (not just XY position)

**Local Controller**: `nav2_mppi_controller/MPPIController`
- Model Predictive Path Integral control
- Better handles humanoid dynamics than DWB (Dynamic Window)
- Configurable critics for obstacle avoidance, path following, goal alignment

**Costmap Configuration**:
```yaml
# humanoid_nav2.yaml
local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"
      # Larger tolerance than actual robot (stability margin)
      inflation_layer:
        inflation_radius: 0.8  # Humanoids need more clearance
        cost_scaling_factor: 3.0

controller_server:
  ros__parameters:
    controller_frequency: 20.0  # Lower than wheeled (match gait cycle)
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      max_vel_x: 0.5  # Humanoid walking speed << wheeled robots
      max_vel_theta: 0.3
      acc_lim_x: 0.3  # Critical for bipedal stability
```

**Key Humanoid Adaptations**:
1. **Footprint**: Larger than actual robot for balance margin
2. **Velocity Limits**: ~0.5 m/s (humanoid walk) vs 1.5 m/s (wheeled)
3. **Controller Frequency**: 20 Hz (gait cycle) vs 50 Hz (continuous motion)
4. **Recovery Behaviors**: Disable in-place rotation (humanoids can't pivot)

**Teaching Progression**:
1. **Chapter 4 Start**: Use differential drive baseline (Nav2 defaults)
2. **Chapter 4 Middle**: Add humanoid constraints (velocity, footprint)
3. **Chapter 4 End**: Tune MPPI critics for obstacle avoidance
4. **Module 3 Project**: Full humanoid configuration

---

#### **Common Pitfalls & Troubleshooting**

**Top 5 Issues**:

1. **GPU Memory Exhaustion**
   - **Symptom**: Isaac Sim crashes with CUDA OOM
   - **Cause**: Simulation rendering + Isaac ROS nodes exceed VRAM
   - **Fix**:
     ```bash
     nvidia-smi  # Check usage
     # In Isaac Sim: Edit → Preferences → Rendering → Anti-aliasing: OFF
     # Reduce resolution: 720p instead of 1080p
     # Run VSLAM OR depth processing, not both
     ```

2. **TF Tree Timing Issues**
   - **Symptom**: Nav2 "transform timeout" warnings
   - **Cause**: Isaac Sim uses sim time, ROS nodes use wall clock
   - **Fix**:
     ```python
     # Isaac Sim script
     simulation_app = SimulationApp({"use_ros_time": True})

     # ROS 2 launch file
     parameters=[{'use_sim_time': True}]
     ```

3. **Docker Network Isolation**
   - **Symptom**: ROS 2 nodes can't discover each other
   - **Cause**: DDS multicast blocked by Docker bridge
   - **Fix**:
     ```bash
     # Use host network mode
     docker run --network=host --gpus all ...

     # Set consistent ROS_DOMAIN_ID
     export ROS_DOMAIN_ID=42
     ```

4. **Sensor Frame Conventions**
   - **Symptom**: Point clouds rotated/flipped in RViz
   - **Cause**: Isaac Sim Y-up vs ROS Z-up
   - **Fix**:
     ```python
     # Isaac Sim camera orientation (ROS standard: X-forward, Z-up)
     orientation=Gf.Quatd(0.5, -0.5, 0.5, -0.5)
     ```

5. **Nav2 Startup Failures**
   - **Symptom**: `lifecycle_manager` errors
   - **Cause**: Missing required parameters
   - **Fix**:
     ```bash
     # Start with Nav2 default params
     cp /opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml humanoid_nav2.yaml
     # Then modify for humanoid
     # Check: robot_base_frame, odom_topic, transform_tolerance
     ```

---

## R2: Synthetic Data Generation Workflows

### Research Question
What are best practices for teaching synthetic dataset generation to robotics learners?

### Findings

#### **Decision D3: Synthetic Data Workflow**

**Options Considered**:
1. Replicator API only (fully programmatic)
2. GUI recorder only (manual capture)
3. Hybrid (GUI demo → Replicator scripting)

**Selected**: **Hybrid (GUI Demonstration → Replicator API Scripting)**

**Rationale**:

**Phase 1: GUI Demonstration (Chapter 2 Intro, 30 minutes)**
- **Purpose**: Show what's possible before teaching how to code it
- **Demo Tasks**:
  - Load pre-built scene with humanoid
  - Add RGB-D camera via GUI
  - Manually adjust lighting (HDR dome)
  - Record 10-20 frames with annotations
  - Export to disk (show output format)

- **Learning Outcome**: Students understand:
  - What synthetic data looks like
  - Types of annotations (RGB, depth, semantic segmentation)
  - Quality factors (lighting, texture, occlusion)

**Phase 2: Replicator API Scripting (Chapter 2 Main, 2-4 hours)**
- **Purpose**: Automate what was shown in GUI for scalability
- **Code Tasks**:
  ```python
  import omni.replicator.core as rep

  # Setup scene
  with rep.new_layer():
      camera = rep.create.camera(position=(2, 2, 2))

      # Domain randomization
      with rep.trigger.on_frame(num_frames=500):
          # Randomize lighting
          rep.modify.light(
              intensity=rep.distribution.uniform(500, 2000),
              color=rep.distribution.uniform((0.8, 0.8, 0.8), (1, 1, 1))
          )

          # Randomize object poses
          rep.modify.pose(
              rep.get.prims(path="/World/Objects/*"),
              position=rep.distribution.uniform((-1, -1, 0), (1, 1, 2))
          )

      # Setup writer (export format)
      writer = rep.WriterRegistry.get("BasicWriter")
      writer.initialize(
          output_dir="/workspace/datasets/humanoid_nav",
          rgb=True,
          semantic_segmentation=True,
          distance_to_camera=True
      )

  rep.orchestrator.run()
  ```

- **Learning Outcome**: Students can:
  - Generate 500+ labeled images programmatically
  - Apply domain randomization (lighting, poses, textures)
  - Export in standard formats (COCO-compatible)
  - Reproduce datasets (seeding, config files)

**Domain Randomization Techniques (Top 3 to Teach)**:

1. **Lighting Randomization** (Most Impact for Vision)
   - Intensity: 500-2000 lumens (indoor → outdoor)
   - Color temperature: 3000K (warm) to 6500K (cool)
   - HDR environment maps: indoor, outdoor, warehouse

2. **Object Pose Randomization** (SLAM Robustness)
   - Position: Uniform distribution within scene bounds
   - Rotation: Random yaw (0-360°)
   - Density: Vary obstacle count (sparse → cluttered)

3. **Texture Randomization** (Generalization)
   - Materials: Switch between wood, metal, plastic, fabric
   - Surface properties: Roughness, metallic, reflectance
   - Background variations: Plain walls → patterned → realistic

**Reproducibility Best Practices**:
```python
# Seed for reproducibility
rep.settings.set_global_seed(42)

# Save configuration
config = {
    "num_frames": 500,
    "lighting_range": (500, 2000),
    "object_count": 10,
    "randomization_seed": 42
}
with open("dataset_config.yaml", "w") as f:
    yaml.dump(config, f)
```

**Teaching Approach**:
- **Chapter 2 Section 1**: GUI demo (15 min)
- **Chapter 2 Section 2**: Replicator API basics (30 min)
- **Chapter 2 Section 3**: Domain randomization (45 min)
- **Chapter 2 Section 4**: Dataset export and validation (30 min)
- **Exercise**: Generate custom dataset for perception training

---

#### **Decision D4: Dataset Formats**

**Options Considered**:
1. COCO (Common Objects in Context)
2. KITTI (Karlsruhe Institute of Technology dataset format)
3. ROS bag files
4. Custom format

**Selected**: **Hybrid (COCO for perception training + ROS bags for SLAM evaluation)**

**Rationale**:

**COCO Format** (Primary for Chapter 2):
- **Use Case**: Training perception models (object detection, segmentation)
- **Pros**:
  - Industry standard (compatible with PyTorch, TensorFlow)
  - Supports RGB + segmentation masks + bounding boxes
  - Tooling: COCO API for evaluation
- **Export**:
  ```python
  writer = rep.WriterRegistry.get("CocoWriter")
  writer.initialize(
      output_dir="/workspace/datasets/coco",
      annotations=["semantic_segmentation", "instance_segmentation"],
      categories=["floor", "wall", "obstacle", "humanoid"]
  )
  ```

**ROS Bag Format** (Secondary for Chapter 3):
- **Use Case**: VSLAM testing and accuracy evaluation
- **Pros**:
  - Native ROS 2 format (no conversion needed)
  - Includes timestamps, TF transforms, IMU data
  - Playback for repeatable SLAM evaluation
- **Export**:
  ```bash
  # Record during Isaac Sim simulation
  ros2 bag record /camera/rgb /camera/depth /imu/data /ground_truth/pose
  ```

**Custom Format** (Advanced):
- For specialized use cases (depth-only, normals, optical flow)
- Teach in "Deep Dive" optional section

**Teaching Approach**:
- **Chapter 2**: Focus on COCO (perception training)
- **Chapter 3**: Use ROS bags (SLAM evaluation)
- **Module 3 Project**: Students choose based on evaluation needs

---

## R3: Isaac ROS VSLAM Configuration

### Research Question
How should we teach Isaac ROS VSLAM setup to maximize learning while ensuring reproducibility?

### Findings

#### **Decision D5: VSLAM Variant**

**Options Considered**:
1. cuVSLAM (NVIDIA's GPU-accelerated)
2. ORB-SLAM3 (CPU-based, open source)
3. RTAB-Map (RGB-D SLAM)

**Selected**: **cuVSLAM (Isaac ROS Visual SLAM)**

**Rationale**:

**Advantages for Education**:
- **GPU Acceleration**: 30+ Hz on RTX 2060 (real-time requirement met)
- **Official Support**: NVIDIA maintains, Docker images available
- **Integration**: Designed for Isaac Sim (coordinate frames, sim time)
- **Performance**: 10x faster than CPU alternatives (enables real-time demos)
- **Learning Objective Alignment**: Module 3 focuses on GPU acceleration

**Disadvantages (Acknowledged)**:
- **Closed Source**: Can't inspect algorithm internals (mitigated with conceptual explanations)
- **NVIDIA Lock-in**: Requires NVIDIA GPU (addressed in constitution with min specs)

**Alternative (ORB-SLAM3) Comparison**:
- **ORB-SLAM3**: Better for understanding algorithms (open source)
- **cuVSLAM**: Better for performance and integration
- **Decision**: Use cuVSLAM, teach concepts separately (SLAM theory in Chapter 3 intro)

---

#### **Key Parameters to Teach (Top 5)**

**1. `rectified_images` (Input Configuration)**
```yaml
visual_slam:
  ros__parameters:
    rectified_images: true  # Isaac Sim provides rectified images
```
- **Plain Language**: "Are your camera images already straightened out?"
- **Why It Matters**: Affects feature extraction accuracy
- **Teaching**: Show rectified vs distorted image comparison

**2. `enable_imu` (Sensor Fusion)**
```yaml
enable_imu: true
imu_topic: "/imu/data"
```
- **Plain Language**: "Use IMU to help when camera can't see well"
- **Why It Matters**: Handles motion blur, low-texture environments
- **Teaching**: Demo VSLAM with/without IMU (show tracking loss recovery)

**3. `map_frame` and `odom_frame` (TF Configuration)**
```yaml
map_frame: "map"
odom_frame: "odom"
base_frame: "base_link"
```
- **Plain Language**: "Names of coordinate frames in ROS"
- **Why It Matters**: Must match robot's TF tree (Nav2 depends on this)
- **Teaching**: Visualize TF tree in RViz (`view_frames`)

**4. `enable_slam_visualization` (Debugging)**
```yaml
enable_slam_visualization: true
```
- **Plain Language**: "Show the map and tracked features in RViz"
- **Why It Matters**: Visual feedback for debugging
- **Teaching**: Always enable for education (performance cost acceptable)

**5. `enable_localization_n_mapping` (Mode Selection)**
```yaml
enable_localization_n_mapping: true  # Full SLAM
# enable_localization_n_mapping: false  # Localization-only (pre-built map)
```
- **Plain Language**: "Build a new map or use an existing one?"
- **Why It Matters**: Different modes for different use cases
- **Teaching**: Chapter 3 uses SLAM mode, Chapter 4 could use localization mode

**Teaching Approach**:
- **Chapter 3 Section 1**: Conceptual SLAM overview (no code)
- **Chapter 3 Section 2**: cuVSLAM installation and launch
- **Chapter 3 Section 3**: Parameter tuning (5 parameters above)
- **Chapter 3 Section 4**: Accuracy evaluation (next section)

---

#### **Accuracy Evaluation Methodology**

**Decision D6: VSLAM Metrics**

**Selected Metrics**:

1. **Absolute Trajectory Error (ATE)** - Primary
   - **Definition**: Root Mean Squared Error between estimated and ground truth poses
   - **Calculation**:
     ```python
     import numpy as np

     # Align trajectories (translation + rotation)
     estimated_aligned = align_trajectory(estimated, ground_truth)

     # Compute error at each timestamp
     errors = np.linalg.norm(estimated_aligned - ground_truth, axis=1)
     ate_rmse = np.sqrt(np.mean(errors**2))
     ```
   - **Interpretation**: "Average position error in meters"
   - **Target**: < 0.05m (5cm) over 50m trajectory (SC-003)

2. **Relative Pose Error (RPE)** - Secondary
   - **Definition**: Error in relative motion between consecutive poses
   - **Use Case**: Detects drift accumulation over time
   - **Teaching**: Show ATE (global error) vs RPE (local drift)

3. **Trajectory Visualization** - Qualitative
   - **Tools**: Plot in RViz or matplotlib
   - **Purpose**: Visual inspection (does path make sense?)

**Evaluation Tooling**:

**Recommended**: `evo` library (Python)
```bash
pip install evo

# Evaluate ATE
evo_ape tum ground_truth.txt estimated.txt --plot --save_results results/ate.zip

# Evaluate RPE
evo_rpe tum ground_truth.txt estimated.txt --plot --save_results results/rpe.zip
```

**Custom Script** (for educational transparency):
```python
# vslam_accuracy.py
def evaluate_vslam(ground_truth_file, estimated_file):
    gt = load_tum_trajectory(ground_truth_file)
    est = load_tum_trajectory(estimated_file)

    # Align trajectories
    est_aligned = align_umeyama(est, gt)

    # Compute ATE
    ate = compute_ate(est_aligned, gt)

    # Report
    print(f"ATE RMSE: {ate['rmse']:.4f} m")
    print(f"ATE Mean: {ate['mean']:.4f} m")
    print(f"ATE Std: {ate['std']:.4f} m")

    # Plot
    plot_trajectories(gt, est_aligned, "VSLAM Accuracy")
```

**Ground Truth from Isaac Sim**:
```python
# In Isaac Sim script, log ground truth poses
ground_truth_writer = open("ground_truth.txt", "w")

def on_physics_step(dt):
    pose = robot.get_world_pose()
    timestamp = simulation_time.get_current_time()
    ground_truth_writer.write(f"{timestamp} {pose.p.x} {pose.p.y} {pose.p.z} "
                               f"{pose.r.x} {pose.r.y} {pose.r.z} {pose.r.w}\n")
```

**Teaching Approach**:
- **Chapter 3 Exercise 4**: Run VSLAM on pre-recorded bag
- **Evaluation**: Use `evo` to compute ATE
- **Success**: ATE RMSE < 0.05m (aligns with SC-003)

---

#### **Failure Mode Handling**

**Common Failure: Tracking Loss**

**Causes**:
1. Low-texture environment (blank walls)
2. Motion blur (fast robot movement)
3. Lighting changes (sudden brightness shift)
4. Occlusions (obstacles blocking camera)

**Recovery Strategies** (teach in Chapter 3):

**1. IMU Fusion** (Preventive)
```yaml
enable_imu: true
gyro_noise_density: 0.0001  # Lower = trust IMU more
accel_noise_density: 0.001
```
- **Effect**: IMU maintains pose estimate during visual tracking loss
- **Demo**: Turn off lights mid-trajectory, VSLAM recovers

**2. Feature Density Tuning** (Preventive)
```yaml
num_features_per_frame: 1000  # Increase for low-texture scenes (default: 500)
```
- **Effect**: More features = better chance of finding matches
- **Trade-off**: Higher GPU/CPU usage

**3. Reinitialization** (Reactive)
```yaml
enable_image_denoising: true  # Pre-process images
min_num_images: 5  # Require 5 keyframes before declaring "lost"
```
- **Effect**: Wait for clear images before giving up
- **Demo**: Show VSLAM reinitializing after walking through narrow corridor

**Teaching Approach**:
- **Chapter 3 Section 5**: Failure modes demo
- **Exercise**: Navigate low-texture environment, observe tracking loss
- **Solution**: Enable IMU, increase feature density, show recovery

---

## R4: Hardware vs Simulation Trade-offs

### Research Question
How should we balance simulation-only vs real hardware deployment in educational content?

### Findings

#### **Decision D7: Hardware Requirement**

**Options Considered**:
1. Simulation-only (no hardware)
2. Hybrid (simulation + optional hardware)
3. Hardware-required (must have physical robot)

**Selected**: **Simulation-Only (Primary) with Optional Hardware Extension**

**Rationale**:

**Simulation-Only Justification**:

1. **Accessibility** (Constitution Principle I):
   - Hardware cost ($200 for sensors/actuators) is barrier for many students
   - Simulations work on any laptop with NVIDIA GPU (already assumed in prerequisites)
   - Global reach: Students in any country can participate

2. **Learning Focus**:
   - Module 3 teaches AI/perception concepts, not hardware integration
   - VSLAM and Nav2 algorithms are same in sim and reality
   - Sim-to-real transfer is advanced topic (beyond Module 3 scope)

3. **Reproducibility**:
   - Simulations produce consistent results (grading fairness)
   - No hardware variability (different sensors, calibration errors)
   - Instructor can verify student work exactly

4. **Safety**:
   - Humanoid navigation failures in sim = harmless
   - Real hardware failures = potential damage/injury

**Isaac Sim Physics Fidelity**:
- **VSLAM**: High fidelity (photorealistic rendering, accurate camera models)
- **Navigation**: Medium fidelity (physics accurate, but contact dynamics simplified)
- **Sim-to-Real Gap**: Exists but not critical for education (concepts transfer)

**Evidence**:
- NVIDIA's Isaac Sim is used for sim-to-real transfer research (high fidelity)
- Many universities teach robotics with sim-only courses (ROS Industrial, Coursera)
- Constitution allows affordable hardware (<$200) but doesn't require it

---

**Optional Hardware Extension** (Appendix):

For motivated students who want real deployment:

**Recommended Hardware (<$200 Budget)**:
- **Single Board Computer**: Raspberry Pi 4 (8GB) - $75
- **Camera**: Intel RealSense D435i (RGB-D + IMU) - $150 (or $50 for used)
- **Chassis**: 3D-printed or purchased robot base - $30
- **Total**: ~$200 (excluding actuators for humanoid gait)

**Appendix Content**:
- "Deploying to Real Hardware" guide (10 pages)
- ROS 2 cross-compilation to ARM
- Sensor calibration procedures
- Sim-to-real transfer tips (domain adaptation)

**Teaching Approach**:
- **Modules 1-3**: Simulation-only
- **Appendix (Optional)**: Hardware deployment for advanced learners
- **Callout in Chapter 3**: "This works on real hardware too! See Appendix C"

---

#### **Decision D8: GPU Alternatives**

**Challenge**: Not all students have NVIDIA GPUs.

**Options Considered**:
1. Cloud GPU (AWS, GCP, Azure)
2. Pre-recorded datasets/demos
3. No alternatives (strict GPU requirement)

**Selected**: **Multi-Tiered Approach**

**Tier 1: Local NVIDIA GPU (Recommended - 90% of students)**
- **Minimum**: RTX 2060 (6GB VRAM)
- **Recommended**: RTX 3060 (12GB VRAM) or better
- **Performance**: Real-time Isaac Sim + Isaac ROS VSLAM
- **Cost**: Assume students already have (gaming PC, workstation)

**Tier 2: Cloud GPU (Fallback - 5% of students)**
- **Providers**: AWS EC2 (G4/G5 instances), Google Cloud (T4/A100), Paperspace
- **Cost Estimate**:
  - AWS g4dn.xlarge: $0.526/hour × 12 hours = ~$6.30 per module
  - Google Cloud T4: $0.35/hour × 12 hours = ~$4.20 per module
- **Setup**: Provide Docker image + cloud setup guide
- **Pros**: No upfront hardware cost
- **Cons**: Recurring cost, network latency for GUI

**Tier 3: Pre-Recorded Content (Last Resort - 5% of students)**
- **Scenario**: No GPU, no cloud budget
- **Content**:
  - Pre-recorded Isaac Sim demos (video)
  - Pre-generated datasets (downloadable)
  - Pre-computed VSLAM trajectories for analysis
- **Limitations**:
  - Can't run own experiments
  - Can analyze data, can't generate it
  - Learn concepts but not hands-on practice
- **Compensation**: Provide CPU-based alternatives where possible (ORB-SLAM3 for VSLAM)

**Teaching Approach**:
- **Chapter 1 Prerequisites**: GPU requirement stated clearly
- **Chapter 1 Appendix A**: Cloud GPU setup guide
- **Chapter 1 Appendix B**: Pre-recorded content option
- **Callouts**: "No GPU? See Appendix A for cloud options"

**Cloud Setup Example** (AWS):
```bash
# Launch instance
aws ec2 run-instances --image-id ami-ubuntu-22.04-gpu \
  --instance-type g4dn.xlarge --key-name my-key

# SSH with X11 forwarding
ssh -X -i my-key.pem ubuntu@<instance-ip>

# Run Docker
docker run --gpus all -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  nvidia/isaac-ros:humble-latest
```

---

## R5: Performance Metrics and Evaluation

### Research Question
What metrics and evaluation methods should learners use to assess their systems?

### Findings

#### **Decision D9: Evaluation Tooling**

**Selected**: **Custom Python Scripts + Industry Tools**

**Rationale**: Balance educational transparency (custom scripts) with industry practice (standard tools).

---

### **1. VSLAM Accuracy Metrics**

**Metrics Defined**:

**Absolute Trajectory Error (ATE)** - Primary
- **Formula**: $\text{ATE}_{\text{RMSE}} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} \|T_{\text{gt},i} - T_{\text{est},i}\|^2}$
- **Units**: Meters
- **Target**: < 0.05m over 50m trajectory (SC-003)
- **Interpretation**: "How far off is the robot's estimated position from ground truth?"

**Relative Pose Error (RPE)** - Secondary
- **Formula**: Measures error in consecutive pose differences
- **Units**: Meters (translation) or degrees (rotation)
- **Use Case**: Detect drift accumulation
- **Target**: < 0.01m/m (1cm error per meter traveled)

**Implementation**:

**Option A: evo Library (Recommended)**
```python
from evo.core import metrics, trajectory
from evo.tools import file_interface

# Load trajectories (TUM format)
traj_gt = file_interface.read_tum_trajectory_file("ground_truth.txt")
traj_est = file_interface.read_tum_trajectory_file("estimated.txt")

# Compute ATE
ate = metrics.APE(metrics.PoseRelation.translation_part)
ate.process_data((traj_gt, traj_est))

print(f"ATE RMSE: {ate.get_statistic(metrics.StatisticsType.rmse):.4f} m")
```

**Option B: Custom Script (Educational)**
```python
# vslam_eval.py
import numpy as np
from scipy.spatial.transform import Rotation

def compute_ate(est_poses, gt_poses):
    """
    Compute Absolute Trajectory Error.

    Args:
        est_poses: Nx7 array (x, y, z, qx, qy, qz, qw)
        gt_poses: Nx7 array (x, y, z, qx, qy, qz, qw)

    Returns:
        dict: {'rmse', 'mean', 'std', 'min', 'max'}
    """
    # Align trajectories (Umeyama algorithm)
    est_aligned = align_trajectories(est_poses[:, :3], gt_poses[:, :3])

    # Compute position errors
    errors = np.linalg.norm(est_aligned - gt_poses[:, :3], axis=1)

    return {
        'rmse': np.sqrt(np.mean(errors**2)),
        'mean': np.mean(errors),
        'std': np.std(errors),
        'min': np.min(errors),
        'max': np.max(errors)
    }

# Example usage
ate_results = compute_ate(estimated, ground_truth)
print(f"VSLAM Accuracy: {ate_results['rmse']:.4f} m RMSE")
```

**Ground Truth from Isaac Sim**:
```python
# isaac_sim_logger.py
class GroundTruthLogger:
    def __init__(self, output_file):
        self.file = open(output_file, 'w')

    def log_pose(self, robot):
        pose = robot.get_world_pose()
        timestamp = simulation_context.get_current_time()

        # TUM format: timestamp x y z qx qy qz qw
        self.file.write(f"{timestamp} {pose.p.x} {pose.p.y} {pose.p.z} "
                        f"{pose.r.x} {pose.r.y} {pose.r.z} {pose.r.w}\n")
```

**Teaching Approach**:
- **Chapter 3 Exercise 4**: Evaluate VSLAM accuracy
- **Tool**: Use `evo` library (install via pip)
- **Success Criteria**: ATE RMSE < 0.05m (SC-003)

---

### **2. Navigation Success Rate**

**Metric Definition**:

**Navigation Success Rate** = $\frac{\text{Successful Navigations}}{\text{Total Navigation Attempts}} \times 100\%$

**Success Criteria** (per navigation goal):
1. **Reached Goal**: Robot within tolerance of target pose
   - Position tolerance: 0.2m (configurable)
   - Heading tolerance: 0.3 radians (~17°)
2. **Time Limit**: Reached within reasonable time (3× optimal path time)
3. **No Collisions**: No contact with obstacles (detected via collision sensors)

**Failure Conditions**:
- Timeout exceeded
- Collision occurred
- Nav2 reports "Failed to find valid path"
- Robot stuck (velocity < 0.01 m/s for > 10 seconds)

**Implementation**:

```python
# nav_evaluator.py
class NavigationEvaluator:
    def __init__(self, position_tolerance=0.2, heading_tolerance=0.3):
        self.position_tolerance = position_tolerance
        self.heading_tolerance = heading_tolerance
        self.results = []

    def evaluate_navigation(self, start_pose, goal_pose, trajectory, collision_events):
        """
        Evaluate single navigation attempt.

        Returns:
            dict: {'success': bool, 'time': float, 'path_length': float, 'collision': bool}
        """
        final_pose = trajectory[-1]

        # Check position
        position_error = np.linalg.norm(final_pose[:2] - goal_pose[:2])
        position_ok = position_error < self.position_tolerance

        # Check heading
        heading_error = abs(final_pose[2] - goal_pose[2])
        heading_ok = heading_error < self.heading_tolerance

        # Check collisions
        collision = len(collision_events) > 0

        # Compute metrics
        time = trajectory[-1, 3] - trajectory[0, 3]  # timestamp
        path_length = compute_path_length(trajectory)

        success = position_ok and heading_ok and not collision

        return {
            'success': success,
            'time': time,
            'path_length': path_length,
            'collision': collision,
            'position_error': position_error,
            'heading_error': heading_error
        }

    def compute_success_rate(self):
        successes = sum(1 for r in self.results if r['success'])
        return (successes / len(self.results)) * 100 if self.results else 0.0

# Module 3 Project usage
evaluator = NavigationEvaluator()

for goal in waypoints:
    result = evaluator.evaluate_navigation(start, goal, robot_trajectory, collisions)
    evaluator.results.append(result)

success_rate = evaluator.compute_success_rate()
print(f"Navigation Success Rate: {success_rate:.1f}%")  # Target: 90%+ (SC-004)
```

**ROS 2 Integration**:
```python
# nav_success_monitor.py (ROS 2 node)
class NavSuccessMonitor(Node):
    def __init__(self):
        super().__init__('nav_success_monitor')

        # Subscribe to Nav2 action results
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to collision sensors
        self.collision_sub = self.create_subscription(
            ContactsState, '/contact_sensor', self.collision_callback, 10
        )

        self.collisions = []

    def send_goal(self, x, y, theta):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # ... set orientation from theta

        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            print("Navigation succeeded!")
        else:
            print(f"Navigation failed: {result.status}")
```

**Teaching Approach**:
- **Chapter 4 Exercise 4**: Measure success rate across 3 waypoints
- **Target**: 90%+ (SC-004)
- **Module 3 Project**: Report success rate across 5 waypoints

---

### **3. Path Efficiency**

**Metric Definition**:

**Path Efficiency** = $\frac{\text{Optimal Path Length}}{\text{Actual Path Length}} \times 100\%$

- **Optimal Path**: Straight-line distance (Euclidean)
- **Actual Path**: Sum of segment lengths in executed trajectory

**Interpretation**:
- 100% = Perfect (straight line)
- 80-90% = Good (minor deviations for obstacles)
- <70% = Poor (excessive wandering)

**Implementation**:

```python
def compute_path_efficiency(trajectory, start, goal):
    """
    Args:
        trajectory: Nx3 array (x, y, timestamp)
        start: [x, y]
        goal: [x, y]

    Returns:
        float: Efficiency percentage (0-100)
    """
    # Optimal path (straight line)
    optimal_length = np.linalg.norm(np.array(goal) - np.array(start))

    # Actual path (sum of segments)
    actual_length = 0.0
    for i in range(1, len(trajectory)):
        segment = trajectory[i, :2] - trajectory[i-1, :2]
        actual_length += np.linalg.norm(segment)

    efficiency = (optimal_length / actual_length) * 100 if actual_length > 0 else 0.0
    return efficiency

# Example
efficiency = compute_path_efficiency(robot_trajectory, start_pos, goal_pos)
print(f"Path Efficiency: {efficiency:.1f}%")  # Target: >80%
```

**Teaching Approach**:
- **Chapter 4 Exercise 5**: Compute path efficiency
- **Interpretation**: High efficiency = good planning, low = excessive replanning
- **Module 3 Project**: Report average efficiency across all waypoints

---

### **4. Computational Load Profiling**

**Metrics**:
1. **GPU Utilization (%)**: VRAM usage, SM utilization
2. **CPU Utilization (%)**: Per-core usage
3. **Memory Usage (MB)**: RAM consumption
4. **Frame Rate (FPS)**: Isaac Sim rendering + ROS 2 node processing

**Tools**:

**NVIDIA GPU Profiling**:
```bash
# Real-time monitoring
nvidia-smi --query-gpu=utilization.gpu,utilization.memory,memory.used --format=csv -l 1

# Logged profiling
nvidia-smi dmon -s mu -c 60 > gpu_usage.log

# Nsight Systems (advanced)
nsys profile --trace=cuda,nvtx ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
```

**ROS 2 Performance**:
```bash
# Topic frequency
ros2 topic hz /visual_slam/tracking/odometry  # Target: 30+ Hz

# Node CPU usage
ros2 run rqt_top rqt_top

# Latency
ros2 topic delay /visual_slam/tracking/odometry
```

**Custom Profiler**:
```python
# performance_monitor.py
import psutil
import subprocess

class PerformanceMonitor:
    def __init__(self):
        self.samples = []

    def record_sample(self):
        # CPU
        cpu_percent = psutil.cpu_percent(interval=1, percpu=True)

        # Memory
        mem = psutil.virtual_memory()

        # GPU (via nvidia-smi)
        gpu_query = subprocess.check_output([
            'nvidia-smi',
            '--query-gpu=utilization.gpu,memory.used',
            '--format=csv,noheader,nounits'
        ]).decode('utf-8').strip().split(',')

        gpu_util = float(gpu_query[0])
        gpu_mem = float(gpu_query[1])

        sample = {
            'timestamp': time.time(),
            'cpu_avg': sum(cpu_percent) / len(cpu_percent),
            'cpu_max': max(cpu_percent),
            'memory_percent': mem.percent,
            'memory_mb': mem.used / (1024**2),
            'gpu_util': gpu_util,
            'gpu_mem_mb': gpu_mem
        }

        self.samples.append(sample)
        return sample

    def report(self):
        cpu_avg = np.mean([s['cpu_avg'] for s in self.samples])
        gpu_avg = np.mean([s['gpu_util'] for s in self.samples])
        gpu_mem_max = max([s['gpu_mem_mb'] for s in self.samples])

        print(f"Performance Report:")
        print(f"  CPU: {cpu_avg:.1f}% average")
        print(f"  GPU: {gpu_avg:.1f}% average")
        print(f"  GPU Memory: {gpu_mem_max:.0f} MB peak")

# Module 3 Project usage
monitor = PerformanceMonitor()

# Record during navigation
while navigating:
    monitor.record_sample()
    time.sleep(1.0)

monitor.report()
```

**Teaching Approach**:
- **Chapter 3 Exercise 5**: Profile VSLAM GPU usage
- **Chapter 4 Exercise 6**: Profile Nav2 CPU usage
- **Module 3 Project**: Report system-wide metrics (SC-007)

---

### **5. Automated vs Manual Evaluation**

**Recommendation**: **Automated for Metrics, Manual for Qualitative**

**Automated Evaluation** (Implemented in Code):
- VSLAM accuracy (ATE RMSE)
- Navigation success rate
- Path efficiency
- Computational load

**Pros**:
- Objective, reproducible
- Fast (run on all student submissions)
- Suitable for grading

**Manual Evaluation** (Instructor Review):
- Trajectory smoothness (visual inspection)
- Recovery behavior quality (how gracefully does it handle failures?)
- Code quality (comments, structure)

**Module 3 Project Grading Script**:
```python
# grader.py
def grade_module_3_project(submission_dir):
    """
    Automated grading for Module 3 Project.

    Returns:
        dict: Scores for each criterion
    """
    scores = {}

    # 1. VSLAM Accuracy (20 points)
    ate_rmse = evaluate_vslam(f"{submission_dir}/ground_truth.txt",
                               f"{submission_dir}/vslam_poses.txt")
    if ate_rmse < 0.05:
        scores['vslam'] = 20
    elif ate_rmse < 0.10:
        scores['vslam'] = 15
    else:
        scores['vslam'] = 10

    # 2. Navigation Success Rate (30 points)
    success_rate = evaluate_navigation(f"{submission_dir}/nav_results.json")
    if success_rate >= 80:  # 4/5 waypoints
        scores['navigation'] = 30
    elif success_rate >= 60:  # 3/5 waypoints
        scores['navigation'] = 20
    else:
        scores['navigation'] = 10

    # 3. Performance Metrics Reported (20 points)
    if os.path.exists(f"{submission_dir}/metrics_report.md"):
        scores['metrics'] = 20  # Full credit for submitting
    else:
        scores['metrics'] = 0

    # 4. System Integration (30 points)
    # Check launch file runs without errors
    launch_success = test_launch_file(f"{submission_dir}/full_system.launch.py")
    scores['integration'] = 30 if launch_success else 0

    total = sum(scores.values())
    return {'scores': scores, 'total': total, 'max': 100}

print(grade_module_3_project("student_submissions/alice"))
# {'scores': {'vslam': 20, 'navigation': 30, 'metrics': 20, 'integration': 30},
#  'total': 100, 'max': 100}
```

**Teaching Approach**:
- **Module 3 Project Rubric**: Published in advance (contracts/module-3-project-rubric.md)
- **Auto-Grader**: Provided to students for self-assessment before submission
- **Instructor Review**: Final 10% of grade based on code quality

---

## Summary: All Decisions Resolved

| ID | Decision Point | Selected Option | Rationale |
|----|----------------|-----------------|-----------|
| **D1** | Isaac Sim Teaching Approach | Hybrid (GUI → Code) | Balances accessibility with professional practice |
| **D2** | Isaac ROS Deployment | Docker (primary) | Reproducibility, isolation, official support |
| **D3** | Synthetic Data Workflow | Hybrid (GUI demo → Replicator API) | Show then teach; scales to 500+ images |
| **D4** | Dataset Formats | COCO (perception) + ROS bags (SLAM) | Industry standards for respective use cases |
| **D5** | VSLAM Variant | cuVSLAM (Isaac ROS) | GPU acceleration, integration, official support |
| **D6** | Nav2 Planner | SMAC Hybrid 2D + MPPI Controller | Non-holonomic humanoid constraints |
| **D7** | Hardware Requirement | Simulation-only (optional hardware appendix) | Accessibility, reproducibility, safety |
| **D8** | GPU Alternatives | Multi-tier (local GPU / cloud / pre-recorded) | Accommodates diverse student resources |
| **D9** | Evaluation Tooling | Custom scripts + evo library | Educational transparency + industry tools |

---

## Next Steps

1. **Phase 1 Execution**: Create `data-model.md` and `contracts/` using these decisions
2. **Update Agent Context**: Add Isaac Sim, Isaac ROS, Nav2 to technology stack
3. **Proceed to Tasks**: Generate implementation tasks via `/sp.tasks` command

**Research Complete**: All NEEDS CLARIFICATION items resolved. Plan ready for Phase 1 design.
