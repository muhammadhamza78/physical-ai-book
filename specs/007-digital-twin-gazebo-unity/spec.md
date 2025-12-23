# Feature Specification: Module 2 - The Digital Twin (Gazebo & Unity)

**Branch**: `007-digital-twin-gazebo-unity` | **Date**: 2025-12-22 | **Status**: Draft

## Overview

**Module**: Module 2 - The Digital Twin (Gazebo & Unity)
**Purpose**: Introduce students to digital twin concepts and dual-simulation environments, using Gazebo for physics-accurate testing and Unity for photorealistic rendering and human-robot interaction visualization.
**Audience**: Students who completed Module 1 (ROS 2 basics, nodes, topics, services, URDF fundamentals)
**Prerequisites**: ROS 2 Humble, basic URDF knowledge, Python 3.8+, Ubuntu 22.04 or WSL2
**Learning Duration**: 8-10 hours (self-paced)
**Outcome**: Students understand digital twin principles, simulate humanoid robots in Gazebo for physics validation, and create high-fidelity visualizations in Unity for stakeholder presentations and HRI testing.

### What is a Digital Twin?

A digital twin is a virtual replica of a physical robot that mirrors its structure, behavior, and sensor data in real-time or simulation. For humanoid robotics, digital twins serve two critical purposes:

1. **Physics-Based Testing (Gazebo)**: Validate control algorithms, test collision scenarios, simulate sensor noise, and verify kinematic/dynamic behavior before deploying to hardware.
2. **Visual Fidelity & HRI (Unity)**: Create photorealistic renders for design reviews, test human-robot interactions in realistic environments, and generate training data for perception systems.

This module teaches students when to use each tool and how to integrate both with ROS 2.

## User Scenarios & Testing

### User Story 1 - Student Understands Digital Twin Concepts (Priority: P1)

**As a** robotics student,
**I want to** understand what digital twins are and when to use physics simulation vs. visual rendering,
**So that** I can select the appropriate tool for testing robot behavior vs. presenting robot designs.

**Independent Test**: Student correctly identifies 3 use cases for Gazebo and 3 use cases for Unity from a list of scenarios. Success measured by 90% accuracy on scenario-based quiz.

**Acceptance Scenarios**:
1. **Given** no prior digital twin knowledge, **When** student reads Chapter 1, **Then** they can define a digital twin and explain its role in robotics development
2. **Given** a list of 10 testing scenarios, **When** student categorizes them, **Then** they correctly identify which require Gazebo (physics accuracy) vs. Unity (visual fidelity) with 90% accuracy
3. **Given** humanoid robot development workflow, **When** student maps digital twin stages, **Then** they correctly sequence: URDF → Gazebo testing → Unity visualization → hardware deployment

### User Story 2 - Student Simulates Humanoid Robot in Gazebo (Priority: P1)

**As a** robotics student,
**I want to** load a URDF humanoid model into Gazebo and test basic physics interactions,
**So that** I can validate joint movements, collision detection, and gravity effects before hardware testing.

**Independent Test**: Student successfully loads 3-link arm URDF in Gazebo, applies joint commands via ROS 2 topics, and observes realistic physics behavior. Success = 85% of students complete simulation without errors.

**Acceptance Scenarios**:
1. **Given** a valid URDF file from Module 1, **When** student launches Gazebo with the model, **Then** the robot loads without errors and responds to gravity
2. **Given** running Gazebo simulation, **When** student publishes joint commands via ROS 2 topics, **Then** robot joints move to commanded positions with realistic physics
3. **Given** humanoid arm near obstacle, **When** student commands motion, **Then** Gazebo detects collisions and prevents interpenetration
4. **Given** simulation with sensor plugins, **When** robot moves, **Then** sensor data (joint states, IMU) publishes correctly to ROS 2 topics

### User Story 3 - Student Creates Visualizations in Unity (Priority: P2)

**As a** robotics student,
**I want to** import a URDF model into Unity and create photorealistic renders,
**So that** I can present robot designs to stakeholders and test human-robot interaction scenarios.

**Independent Test**: Student imports humanoid URDF into Unity, applies materials/lighting, and generates a 30-second animation demonstrating robot movement. Success = 75% of students create acceptable renders.

**Acceptance Scenarios**:
1. **Given** a URDF model and Unity Robotics Hub package, **When** student imports the model, **Then** robot appears correctly in Unity scene with proper hierarchy
2. **Given** imported robot in Unity, **When** student applies PBR materials and HDRI lighting, **Then** renders achieve photorealistic quality suitable for presentations
3. **Given** Unity scene with robot, **When** student connects to ROS 2 via Unity Robotics Hub, **Then** joint states synchronize between ROS and Unity in real-time
4. **Given** HRI scenario setup, **When** student adds virtual human models, **Then** interaction visualization demonstrates realistic human-robot spatial relationships

### User Story 4 - Student Simulates Sensors (LiDAR, Depth, IMU) (Priority: P1)

**As a** robotics student,
**I want to** add sensor plugins to my Gazebo and Unity simulations,
**So that** I can test perception algorithms with simulated LiDAR, depth cameras, and IMU data.

**Independent Test**: Student adds LiDAR plugin to Gazebo model, visualizes point cloud in RViz, and subscribes to sensor data via ROS 2. Success = 80% correctly configure at least 2 sensor types.

**Acceptance Scenarios**:
1. **Given** humanoid URDF in Gazebo, **When** student adds LiDAR plugin to URDF, **Then** point cloud data publishes to /scan topic at 10Hz
2. **Given** depth camera plugin, **When** student launches simulation, **Then** RGB and depth images publish to /camera/rgb and /camera/depth topics
3. **Given** IMU plugin on robot torso, **When** robot moves, **Then** IMU data (linear acceleration, angular velocity) publishes with realistic noise characteristics
4. **Given** multiple sensors active, **When** student visualizes in RViz, **Then** sensor data overlays correctly on robot model

## Functional Requirements

### Module Structure

**FR-001**: Module 2 MUST use "Module" as the top-level unit label (e.g., "Module 2 - The Digital Twin")
**FR-002**: Module 2 subdivisions MUST be labeled "Chapter 1, Chapter 2, Chapter 3, Chapter 4" with sequential numbering
**FR-003**: The term "Lesson" MUST NOT be used as a structural label within Module 2
**FR-004**: Module 2 MUST conclude with "Module 2 Project" as the final integrative assignment

### Chapter Content

**FR-005**: Chapter 1 MUST explain digital twin concepts, differentiate physics simulation (Gazebo) from visual rendering (Unity), and provide use case decision criteria

**FR-006**: Chapter 2 MUST cover Gazebo Harmonic/Ignition installation, URDF loading, physics engines (ODE, Bullet, Simbody), world files, and ROS 2 integration via ros_gz_bridge

**FR-007**: Chapter 3 MUST cover Unity 2022 LTS installation, Unity Robotics Hub setup, URDF Importer package, TCP Endpoint connection to ROS 2, and photorealistic rendering techniques (PBR materials, HDRI lighting)

**FR-008**: Chapter 4 MUST cover sensor simulation plugins for LiDAR (gazebo_ros_ray_sensor), depth cameras (gazebo_ros_camera with depth), and IMU (gazebo_ros_imu_sensor) with configuration examples

**FR-009**: Each chapter MUST include hands-on exercises with starter code, expected outputs, and validation criteria

**FR-010**: Each chapter MUST provide troubleshooting guidance for common issues (URDF loading errors, plugin failures, ROS 2 bridge connection issues)

### Code Examples & Technology

**FR-011**: All Gazebo examples MUST use Gazebo Harmonic (formerly Ignition Fortress/Garden) with SDF 1.9+ format

**FR-012**: All Unity examples MUST use Unity 2022 LTS with Unity Robotics Hub 0.7+ and URDF Importer 0.5+

**FR-013**: Sensor plugins MUST publish data to standard ROS 2 message types (sensor_msgs/LaserScan, sensor_msgs/Image, sensor_msgs/Imu)

**FR-014**: All code examples MUST include inline comments explaining digital twin concepts, plugin parameters, and ROS 2 integration points

**FR-015**: Installation guides MUST provide Ubuntu 22.04 native and WSL2 instructions with Docker alternatives for Unity (Windows host + WSL2 ROS 2)

### Learning Experience

**FR-016**: Each chapter MUST define learning objectives, estimated completion time (1.5-2.5 hours), and success criteria

**FR-017**: Module 2 Project MUST require integrating Gazebo physics testing AND Unity visualization for a single humanoid subsystem (e.g., test arm physics in Gazebo, render in Unity)

**FR-018**: Prerequisites MUST explicitly state: Module 1 completion, URDF fundamentals, ROS 2 Humble, Python 3.8+

## Success Criteria

**SC-001**: 90% of students correctly categorize 9/10 scenarios as Gazebo (physics) vs. Unity (visual) use cases

**SC-002**: 85% of students successfully load a URDF model in Gazebo and apply joint commands via ROS 2 topics

**SC-003**: 80% of students correctly configure at least 2 sensor types (LiDAR, depth camera, or IMU) in Gazebo

**SC-004**: 75% of students import a URDF into Unity and create a photorealistic render with materials and lighting

**SC-005**: 80% of students complete Module 2 Project (Gazebo testing + Unity visualization) within 8-10 hours

**SC-006**: 90% of students score 80%+ on end-of-module quiz covering digital twin concepts, tool selection, and sensor configuration

**SC-007**: Chapter time estimates match actual completion times within ±20% (Ch1: 1.5hr, Ch2: 2.5hr, Ch3: 2.5hr, Ch4: 2hr)

**SC-008**: 85% of students report "confident" or "very confident" with Gazebo physics simulation and Unity rendering basics

**SC-009**: 80% of students correctly explain when to use Gazebo vs. Unity in technical interviews or project proposals

## Assumptions and Constraints

### Assumptions

1. **Module 1 Prerequisite**: Students have completed Module 1 and understand ROS 2 nodes, topics, services, and URDF basics
2. **Hardware Platform**: Students use Ubuntu 22.04 (native or WSL2) with GPU support preferred for Unity rendering
3. **ROS 2 Version**: All content uses ROS 2 Humble (LTS) for consistency with Module 1
4. **Gazebo Version**: Gazebo Harmonic (successor to Ignition) is the target simulator (NOT Gazebo Classic 11)
5. **Unity Version**: Unity 2022 LTS is the target engine with Unity Robotics Hub 0.7+ and URDF Importer 0.5+
6. **Humanoid Robot Focus**: All examples use humanoid subsystems (arms, torso, legs) for curriculum coherence
7. **Simulation-Only Scope**: No hardware integration; focuses on digital twin concepts and simulation workflows
8. **Time Commitment**: Students allocate 8-10 hours for Module 2 completion (4 chapters + project)
9. **Delivery Platform**: Docusaurus static site generator with MDX support for interactive code examples
10. **Internet Access**: Students can download Gazebo models from Fuel (fuel.gazebosim.org) and Unity assets from Asset Store

### Constraints

1. **Platform Compatibility**: Unity on WSL2 requires Windows host installation with TCP connection to WSL2 ROS 2
2. **GPU Requirements**: Unity rendering benefits from GPU but must provide CPU-only fallback instructions
3. **ROS 2 Bridge Complexity**: ros_gz_bridge and Unity TCP Endpoint require careful topic/message type mapping
4. **URDF Compatibility**: Some URDF features (e.g., transmissions) may not import correctly into Unity; troubleshooting required
5. **Version Sensitivity**: Gazebo Harmonic and Unity Robotics Hub APIs are evolving; examples may require updates
6. **Learning Curve**: Gazebo SDF format and Unity C# scripting add complexity beyond ROS 2/Python from Module 1

## Out of Scope

The following topics are explicitly excluded from Module 2:

1. **Hardware Integration**: Connecting simulations to physical humanoid robots (deferred to advanced modules)
2. **Advanced Physics Tuning**: Friction coefficients, contact dynamics, custom physics plugins (beyond basic configuration)
3. **Unity C# Scripting**: Custom Unity scripts for AI agents (deferred to Module 4 or advanced HRI modules)
4. **Gazebo Model Creation**: Building world models from scratch with Blender/CAD (provides pre-built worlds only)
5. **Custom Sensor Plugins**: Writing new Gazebo plugins in C++ (uses existing ros_gz_sensors only)
6. **Multi-Robot Simulation**: Simulating swarms or teams of robots (focuses on single humanoid)
7. **Real-Time Performance Optimization**: Tuning simulation speed, physics step sizes for real-time factor >1.0
8. **ROS 2 Control Integration**: Using ros2_control framework with Gazebo (deferred to Module 3 or control modules)
9. **Unity HDRP/URP Pipelines**: Advanced rendering pipelines (uses Built-in Render Pipeline for simplicity)
10. **Machine Learning Integration**: Training RL agents in simulation (deferred to ML/AI modules)
11. **Gazebo Classic (Version 11)**: All content uses Gazebo Harmonic; no backward compatibility with Classic
12. **Unreal Engine**: Only Unity covered for visual rendering; Unreal out of scope

## Dependencies

### Prerequisites (Must Have Before Starting)

1. **Module 1 Completion**: Students must understand ROS 2 nodes, topics, services, URDF structure
2. **ROS 2 Humble**: Installed and verified working (from Module 1)
3. **Python 3.8+**: Functional with rclpy library
4. **URDF Model**: Students should have created at least one simple URDF (e.g., 3-link arm from Module 1)

### Software Dependencies

5. **Gazebo Harmonic**: Install via `sudo apt install gz-harmonic` on Ubuntu 22.04
6. **ros_gz Packages**: `sudo apt install ros-humble-ros-gz` for ROS 2 ↔ Gazebo bridge
7. **Unity 2022 LTS**: Download from Unity Hub (Windows or Ubuntu; WSL2 requires Windows host)
8. **Unity Robotics Hub**: Clone from https://github.com/Unity-Technologies/Unity-Robotics-Hub
9. **URDF Importer**: Install via Unity Package Manager (com.unity.robotics.urdf-importer)

### System Dependencies

10. **GPU (Recommended)**: NVIDIA GPU with proprietary drivers for Unity rendering and Gazebo visualization
11. **Storage**: ~15 GB for Gazebo models, Unity assets, and development environment
12. **Memory**: 8 GB RAM minimum; 16 GB recommended for running Gazebo + RViz + Unity simultaneously

## Risks and Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|-------------|------------|
| **Gazebo installation failures** (dependency conflicts, wrong version) | High | Medium | Provide Docker container with Gazebo Harmonic pre-installed; detailed troubleshooting guide for apt conflicts |
| **Unity on WSL2 complexity** (requires Windows host + TCP bridge) | High | High | Provide dual installation guides (native Ubuntu Unity vs. Windows Unity + WSL2 ROS); test both paths |
| **GPU unavailable** (cloud environments, VMs without passthrough) | Medium | Medium | Provide CPU-only Unity rendering instructions; use Gazebo headless mode with RViz-only visualization |
| **ros_gz_bridge topic mapping errors** (message type mismatches) | Medium | High | Provide tested bridge configuration files; document common message type conversions |
| **URDF import failures in Unity** (unsupported tags, mesh paths) | Medium | High | Provide pre-validated URDF templates; troubleshoot common import errors in Chapter 3 |
| **Version incompatibilities** (Gazebo Harmonic API changes, Unity Robotics Hub updates) | Medium | Medium | Lock dependency versions in installation guide; test all examples on specified versions quarterly |
| **Module scope too broad for 8-10 hours** | High | Medium | Beta test with target audience; prioritize P1 user stories; mark Unity chapter (Ch3) as optional for project |
| **Sensor plugin configuration complexity** | Medium | Medium | Provide tested plugin XML snippets; include parameter explanation tables; offer pre-configured example models |

## Edge Cases

### Installation Edge Cases

1. **WSL2 + Windows Unity Setup**
   - **Scenario**: Student uses WSL2 for ROS 2 but needs Unity on Windows host
   - **Solution**: Provide TCP Endpoint configuration with Windows firewall rules; test localhost vs. WSL2 IP addressing

2. **Gazebo Classic vs. Harmonic Confusion**
   - **Scenario**: Student has Gazebo Classic (version 11) installed from prior courses
   - **Solution**: Explicitly differentiate `gazebo` (Classic) from `gz` (Harmonic) commands; provide uninstall instructions

3. **Ubuntu 20.04 Users**
   - **Scenario**: Student uses Ubuntu 20.04 (unsupported for Gazebo Harmonic)
   - **Solution**: Recommend Ubuntu 22.04 upgrade or Docker container; note incompatibility in prerequisites

### URDF Edge Cases

4. **Mesh File Paths in URDF**
   - **Scenario**: URDF uses absolute paths or missing `package://` URIs; models fail to load in Gazebo/Unity
   - **Solution**: Troubleshoot relative vs. absolute paths; provide URDF validation tool; document package:// syntax

5. **Missing Inertial Tags**
   - **Scenario**: URDF from Module 1 lacks inertial properties; Gazebo simulation behaves unrealistically (floating, no gravity)
   - **Solution**: Chapter 2 includes inertial tag tutorial; provide `check_urdf` and `gz sdf` validation steps

6. **Collision vs. Visual Geometry Mismatch**
   - **Scenario**: Collision and visual meshes differ significantly; robot appears to collide with empty space
   - **Solution**: Explain collision/visual distinction in Chapter 2; provide debug visualization in Gazebo

### Simulation Edge Cases

7. **ROS 2 Topic Name Mismatches**
   - **Scenario**: Gazebo plugin publishes to `/robot/joint_states` but student expects `/joint_states`
   - **Solution**: Document remapping in launch files; provide `ros2 topic list` debugging workflow

8. **Sensor Plugin Not Publishing**
   - **Scenario**: Student adds LiDAR plugin but no data appears on `/scan` topic
   - **Solution**: Checklist for common errors (update rate = 0, topic name typo, plugin not loaded); provide `gz topic list` debugging

9. **Unity URDF Importer Axis Mismatch**
   - **Scenario**: Robot imported into Unity appears rotated 90° or upside down (ROS Z-up vs. Unity Y-up)
   - **Solution**: Document coordinate system transformation; provide Unity import settings screenshot

10. **Physics Simulation Too Slow**
    - **Scenario**: Gazebo runs at <0.5x real-time factor due to complex models or insufficient CPU
    - **Solution**: Provide performance tuning tips (reduce mesh complexity, increase physics step size); use simpler models for exercises

## Chapter Structure

### Chapter 1: Introduction to Digital Twins for Humanoid Robots (1.5 hours)

**Learning Objectives**:
- Define digital twin and explain its role in robot development
- Differentiate physics-based simulation (Gazebo) from visual rendering (Unity)
- Identify appropriate use cases for each tool

**Content**:
- What is a digital twin? (virtual replica synchronized with physical or simulated robot)
- The dual-simulation approach: Gazebo for testing, Unity for presentation
- Decision criteria: When to use Gazebo (collision testing, dynamics validation, sensor simulation) vs. Unity (stakeholder demos, HRI testing, photorealistic renders)
- Workflow integration: URDF → Gazebo validation → Unity visualization → hardware deployment

**Exercise 1.1**: Categorize 10 scenarios as Gazebo or Unity use cases (quiz format)

### Chapter 2: Physics Simulation with Gazebo Harmonic (2.5 hours)

**Learning Objectives**:
- Install Gazebo Harmonic and ros_gz bridge
- Load URDF models into Gazebo with correct physics properties
- Apply joint commands via ROS 2 and observe realistic behavior
- Configure basic world environments

**Content**:
- Gazebo Harmonic vs. Gazebo Classic (architectural differences, SDF format)
- Installation: `sudo apt install gz-harmonic ros-humble-ros-gz`
- URDF to SDF conversion and loading
- Physics engines (ODE, Bullet, Simbody) and when to use each
- World files: adding ground plane, lighting, obstacles
- ros_gz_bridge: mapping Gazebo topics to ROS 2

**Exercise 2.1**: Load 3-link arm URDF from Module 1 into Gazebo
**Exercise 2.2**: Publish joint commands via `/joint_commands` topic and observe motion
**Exercise 2.3**: Add collision detection and test with obstacles

### Chapter 3: Photorealistic Rendering with Unity (2.5 hours)

**Learning Objectives**:
- Install Unity 2022 LTS and Unity Robotics Hub
- Import URDF models into Unity with correct hierarchy
- Apply PBR materials and HDRI lighting for photorealistic renders
- Connect Unity to ROS 2 via TCP Endpoint

**Content**:
- Unity 2022 LTS installation (Ubuntu native or Windows with WSL2 ROS 2)
- Unity Robotics Hub architecture (TCP Endpoint, ROS-Unity message serialization)
- URDF Importer package: installation and usage
- Coordinate system transformation (ROS Z-up → Unity Y-up)
- PBR materials (metallic, roughness, albedo) and HDRI lighting
- Real-time synchronization: subscribing to `/joint_states` in Unity

**Exercise 3.1**: Import humanoid arm URDF into Unity
**Exercise 3.2**: Apply materials and lighting for presentation-quality render
**Exercise 3.3**: Connect Unity to ROS 2 and synchronize joint movements

### Chapter 4: Sensor Simulation (LiDAR, Depth Cameras, IMU) (2 hours)

**Learning Objectives**:
- Add LiDAR, depth camera, and IMU plugins to Gazebo URDF
- Configure sensor parameters (update rate, resolution, noise)
- Visualize sensor data in RViz
- Understand sensor message types and frame transforms

**Content**:
- Gazebo sensor plugins: gazebo_ros_ray_sensor (LiDAR), gazebo_ros_camera (depth), gazebo_ros_imu_sensor
- Plugin XML syntax and parameters (topic name, update rate, FOV, resolution)
- Sensor noise modeling (Gaussian noise, bias)
- ROS 2 sensor message types (sensor_msgs/LaserScan, sensor_msgs/Image, sensor_msgs/Imu)
- TF frames and sensor coordinate systems

**Exercise 4.1**: Add LiDAR plugin to humanoid head, visualize in RViz
**Exercise 4.2**: Add depth camera plugin, subscribe to RGB and depth topics
**Exercise 4.3**: Add IMU plugin to torso, validate orientation and acceleration data

### Module 2 Project: Dual-Simulation Workflow

**Requirements**:
- Create humanoid subsystem URDF (arm, torso, or leg) with at least 3 joints
- Test physics in Gazebo: apply joint commands, verify collision detection, add 1 sensor (LiDAR or depth camera)
- Import into Unity: create photorealistic render with materials and lighting
- Document workflow: URDF design rationale, Gazebo test results, Unity render screenshots

**Rubric** (100 points):
- URDF model quality (valid syntax, realistic inertial properties): 20 pts
- Gazebo simulation (joint control, collision detection, sensor data): 30 pts
- Unity visualization (successful import, materials/lighting, photorealistic quality): 25 pts
- Sensor integration (configured correctly, publishes valid data): 15 pts
- Documentation (design rationale, test results, screenshots): 10 pts

**Passing**: 70/100 | **Excellence**: 90/100

## Non-Functional Requirements (NFRs)

### Usability

**NFR-001**: Installation guides MUST provide step-by-step instructions with screenshots for Ubuntu 22.04 native and WSL2 environments

**NFR-002**: All exercises MUST include validation criteria (expected outputs, common errors, troubleshooting steps) to enable self-assessment

**NFR-003**: Code examples MUST include inline comments explaining digital twin concepts, plugin parameters, and ROS 2 integration

### Accessibility

**NFR-004**: Content MUST use clear language avoiding unexplained jargon; technical terms MUST be defined on first use

**NFR-005**: Diagrams MUST include alt text describing key concepts for screen reader compatibility

**NFR-006**: Code snippets MUST use syntax highlighting and include copy-paste functionality in Docusaurus

### Performance

**NFR-007**: Gazebo examples MUST achieve >0.8x real-time factor on reference hardware (Intel i5, 8GB RAM, integrated graphics)

**NFR-008**: Unity examples MUST render at >30 FPS on reference hardware (Intel i5, 8GB RAM, NVIDIA GTX 1650 or equivalent)

**NFR-009**: ROS 2 ↔ Gazebo bridge latency MUST be <100ms for joint command → simulation response

### Maintainability

**NFR-010**: All dependency versions MUST be explicitly specified (Gazebo Harmonic 7.x, Unity 2022.3 LTS, Unity Robotics Hub 0.7.0)

**NFR-011**: Examples MUST be tested quarterly against latest patch releases of specified versions

**NFR-012**: Troubleshooting guides MUST be updated within 2 weeks of new common issues being reported

### Compatibility

**NFR-013**: Content MUST support Ubuntu 22.04 LTS (native and WSL2) and provide Docker alternative for unsupported platforms

**NFR-014**: Unity examples MUST work on both Windows (with WSL2 ROS 2) and Ubuntu native installations

**NFR-015**: ROS 2 message types MUST follow standard conventions (sensor_msgs, geometry_msgs) to ensure interoperability

---

**Next Steps**: Run `/sp.plan` to create implementation plan for Module 2 content development.

