# Implementation Plan: Module 2 - The Digital Twin (Gazebo & Unity)

**Branch**: `007-digital-twin-gazebo-unity` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)

## Summary

Module 2 teaches students digital twin concepts using a dual-simulation approach: Gazebo for physics-accurate testing (joint dynamics, collision detection, sensor validation) and Unity for photorealistic rendering (stakeholder presentations, HRI testing). Students learn conceptual foundations (Chapter 1), Gazebo physics simulation (Chapter 2), Unity visualization (Chapter 3), and sensor integration across both tools (Chapter 4), culminating in an integrative Module 2 Project requiring synchronized Gazebo testing and Unity visualization of a humanoid subsystem.

## Technical Context

**Content Format**: Docusaurus MDX (Markdown + React components)
**Physics Simulation**: Gazebo Harmonic 7.x (successor to Ignition; NOT Gazebo Classic 11)
**Visual Rendering**: Unity 2022.3 LTS with Unity Robotics Hub 0.7.0+
**ROS 2 Integration**: ros_gz_bridge (Gazebo ↔ ROS 2), Unity TCP Endpoint (Unity ↔ ROS 2)
**Scripting Language**: Python 3.8+ with rclpy library
**Robot Modeling**: URDF (shared between Gazebo and Unity via URDF Importer)
**Sensor Types**: LiDAR (sensor_msgs/LaserScan), Depth cameras (sensor_msgs/Image), IMU (sensor_msgs/Imu)
**Testing Approach**: Dual-environment validation (physics correctness in Gazebo, visual fidelity in Unity), sensor cross-validation
**Target Platform**: Static site generator (Docusaurus) deployed as web-based curriculum
**Project Type**: Educational content (documentation-based, not software application)
**Learning Duration**: 8-10 hours total (self-paced)
**Content Scope**: 4 chapters + 1 integrative project
**Assets Required**: Gazebo world files, Unity scenes, URDF models with sensor plugins, installation scripts, troubleshooting documentation


## Constitution Check

*GATE: Must pass before Phase 0 research*

### ✅ Accessibility First - PASS

- Spec requires clear explanations with dual-tool decision criteria (when to use Gazebo vs. Unity)
- Prerequisites explicitly stated (Module 1 completion, ROS 2 Humble, basic URDF knowledge)
- Progressive introduction: concepts → physics testing → visual rendering → sensor integration
- Edge cases address common beginner issues (WSL2 setup, Gazebo Classic confusion, coordinate system mismatches, URDF import failures)
- NFR-004: Content MUST use clear language avoiding unexplained jargon; technical terms MUST be defined on first use

### ✅ Hands-On Learning Priority - PASS

- FR-009: Each chapter includes hands-on exercises with starter code, expected outputs, and validation criteria
- User stories 2-4 require practical implementation (Gazebo simulation, Unity rendering, sensor configuration)
- Module 2 Project is hands-on integrative capstone requiring dual-environment testing
- SC-002: 85% of students successfully load URDF in Gazebo and apply joint commands
- SC-003: 80% of students correctly configure at least 2 sensor types

### ✅ Progressive Complexity - PASS

- FR-005 to FR-008: Chapter progression from conceptual (Ch1: digital twin decision criteria) → physics testing (Ch2: Gazebo) → visual rendering (Ch3: Unity) → sensor integration (Ch4: cross-tool validation)
- User story priorities: P1 (understanding, Gazebo, sensors) before P2 (Unity rendering)
- SC-007: Time estimates increase with complexity and dual-tool integration (Ch1: 1.5hr, Ch2: 2.5hr, Ch3: 2.5hr, Ch4: 2hr)
- Chapter 4 builds on both Ch2 (Gazebo sensor plugins) and Ch3 (Unity sensor visualization)

### ✅ Production-Ready Examples - PASS

- FR-011: All Gazebo examples MUST use Gazebo Harmonic with SDF 1.9+ format (industry standard)
- FR-012: All Unity examples MUST use Unity 2022 LTS with Unity Robotics Hub 0.7+
- FR-013: Sensor plugins MUST publish data to standard ROS 2 message types (sensor_msgs/LaserScan, sensor_msgs/Image, sensor_msgs/Imu)
- FR-014: All code examples MUST include inline comments explaining digital twin concepts, plugin parameters, and ROS 2 integration
- NFR-007: Gazebo examples MUST achieve >0.8x real-time factor on reference hardware
- NFR-008: Unity examples MUST render at >30 FPS on reference hardware

### ✅ Clear Documentation Standards - PASS

- FR-016: Each chapter defines learning objectives, estimated completion time, and success criteria
- FR-009: Exercises include expected outputs and validation criteria
- FR-010: Each chapter provides troubleshooting guidance for common issues
- NFR-001: Installation guides MUST provide step-by-step instructions with screenshots for Ubuntu 22.04 native and WSL2
- NFR-002: All exercises MUST include validation criteria to enable self-assessment
- Dependencies explicitly listed with versions (Gazebo Harmonic 7.x, Unity 2022.3 LTS, Unity Robotics Hub 0.7.0)

**Gate Result**: ✅ PASS - Proceed to Phase 0


## Project Structure

### Documentation (this feature)

```text
specs/007-digital-twin-gazebo-unity/
├── plan.md              # This file
├── research.md          # Phase 0 - Best practices research
├── chapter-outlines.md  # Phase 1 - Content structure
├── exercise-plans.md    # Phase 1 - Exercise specifications
├── project-rubric.md    # Phase 1 - Module 2 Project grading criteria
└── tasks.md             # Phase 2 - /sp.tasks output
```

### Content Structure (Docusaurus)

```text
physical-ai-book/docs/module-02/
├── index.md
├── chapter-01-digital-twins-robotics.md
├── chapter-02-gazebo-physics-simulation.md
├── chapter-03-unity-rendering-interaction.md
├── chapter-04-sensor-simulation-validation.md
└── module-02-project.md

physical-ai-book/static/code-examples/module-02/
├── chapter-02/
│   ├── basic_world.sdf
│   ├── humanoid_arm_gazebo.urdf
│   ├── joint_command_publisher.py
│   ├── collision_test_world.sdf
│   └── physics_validation_script.py
├── chapter-03/
│   ├── unity_urdf_import_guide.md
│   ├── ros_tcp_endpoint_setup.py
│   ├── joint_state_subscriber.cs
│   └── camera_controller.cs
└── chapter-04/
    ├── lidar_plugin_urdf.xml
    ├── depth_camera_plugin_urdf.xml
    ├── imu_plugin_urdf.xml
    ├── sensor_visualizer_rviz.py
    └── cross_tool_sensor_validator.py
```


## Phase 0: Research

### Research Tasks

1. **Gazebo Harmonic Best Practices** - Survey Gazebo documentation, SDF format specifications, and community examples for beginner-friendly physics simulation tutorials; identify differences from Gazebo Classic that must be highlighted

2. **Unity Robotics Hub Integration Patterns** - Research Unity TCP Endpoint configuration, URDF Importer workflows, and ROS message serialization; identify common pitfalls in Unity-ROS 2 integration

3. **Dual-Tool Workflow Examples** - Find or create examples of synchronized Gazebo-Unity simulations; evaluate existing humanoid robot models compatible with both tools

4. **Sensor Plugin Configuration** - Research Gazebo sensor plugin syntax (gazebo_ros_ray_sensor, gazebo_ros_camera, gazebo_ros_imu_sensor) and Unity sensor equivalents; compile parameter reference tables

5. **WSL2 Installation Troubleshooting** - Document WSL2 + Gazebo setup (X11 forwarding, GPU passthrough), WSL2 + Windows Unity TCP connection issues, and performance optimization tips

6. **Cross-Platform Installation Validation** - Test Gazebo Harmonic on Ubuntu 22.04 native, WSL2, and Docker; test Unity 2022 LTS on Ubuntu native and Windows with WSL2 backend; compile compatibility matrix

7. **Physics Debugging Techniques** - Research common Gazebo physics issues (unstable simulations, unrealistic behavior, collision failures) and diagnostic tools (gz topic, gz model, SDF validation)

8. **Assessment Design for Dual-Tool Competency** - Research quiz question formats and project rubric criteria evaluating both physics accuracy (Gazebo) and visual fidelity (Unity)

**Output**: research.md


## Phase 1: Design

### Chapter Content (`chapter-outlines.md`)

**Chapter 1: Digital Twins in Robotics (Gazebo + Unity Overview)**

- What is a digital twin? (virtual replica mirroring physical robot structure, behavior, sensor data)
- Dual-simulation philosophy: physics testing vs. visual rendering
  - Gazebo: Physics accuracy (collision detection, joint dynamics, gravity, sensor noise modeling)
  - Unity: Visual fidelity (photorealistic rendering, lighting, materials, HRI scenarios)
- Decision criteria: When to use Gazebo vs. Unity
  - Gazebo use cases: Control algorithm validation, collision testing, dynamic stability analysis, sensor noise characterization
  - Unity use cases: Stakeholder presentations, HRI scenario testing, design reviews, perception training data generation
  - Combined workflow: Develop in Gazebo → Validate physics → Visualize in Unity → Present to stakeholders
- Data flow architecture: ROS 2 as integration layer
  - Gazebo → ros_gz_bridge → ROS 2 topics → Unity TCP Endpoint → Unity scene
  - Bidirectional communication: Unity user inputs → ROS 2 → Gazebo simulation
- Humanoid robot digital twin examples: balance testing (Gazebo) + gait visualization (Unity)
- Learning objectives, 1.5-hour time estimate
- Exercise 1.1: Categorize 10 robotics scenarios as Gazebo-appropriate, Unity-appropriate, or requiring both tools

**Chapter 2: Physics Simulation in Gazebo**

- Gazebo Harmonic vs. Gazebo Classic (architectural differences, migration considerations)
  - Command line: `gz` (Harmonic) vs. `gazebo` (Classic)
  - World format: SDF 1.9+ (Harmonic) vs. SDF 1.6 (Classic)
  - Why Harmonic: Better ROS 2 integration, modern physics engines, active development
- SDF (Simulation Description Format) structure: worlds, models, links, joints, plugins
- Physics engines in Gazebo: ODE (default, fast), Bullet (accurate contacts), Simbody (biomechanics)
  - When to choose each engine for humanoid robots (ODE for general testing, Bullet for complex contacts, Simbody for muscle-driven systems)
- World files: ground plane, lighting, gravity configuration, model includes
- Loading URDF models in Gazebo: URDF-to-SDF conversion, package paths, mesh resolution
- Joint dynamics: effort limits, velocity limits, damping, friction
- Collision detection: collision vs. visual geometry, contact properties (kp, kd, mu1, mu2)
- Gravity and stability: validating humanoid standing pose, center of mass debugging
- ros_gz_bridge: topic mapping syntax, message type conversions (geometry_msgs, sensor_msgs)
- Debugging physics issues: `gz topic list`, `gz model info`, real-time factor analysis
- Learning objectives, 2.5-hour time estimate
- Exercise 2.1: Load humanoid arm URDF in Gazebo, verify gravity response
- Exercise 2.2: Publish joint commands via ROS 2 topic, observe realistic motion
- Exercise 2.3: Create collision test world, validate contact detection
- Exercise 2.4: Modify physics engine (ODE → Bullet), compare simulation behavior

**Chapter 3: High-Fidelity Rendering & Interaction in Unity**

- Unity's role in robotics: visualization, HRI testing, stakeholder communication (NOT real-time control)
- Unity 2022 LTS installation: Unity Hub, project creation, package management
- Unity Robotics Hub architecture:
  - TCP Endpoint: Unity ↔ ROS 2 communication via TCP sockets
  - ROS message serialization: converting ROS 2 messages to Unity C# objects
  - URDF Importer: parsing URDF XML, creating Unity GameObjects with correct hierarchy
- Importing URDF models: URDF Importer package workflow
  - Package installation via Unity Package Manager
  - URDF import settings: coordinate system (ROS Z-up → Unity Y-up), mesh scale, joint axis mapping
  - Troubleshooting import failures: mesh paths, unsupported URDF tags, kinematic tree errors
- Coordinate system transformation: ROS right-handed Z-up vs. Unity left-handed Y-up
- PBR (Physically-Based Rendering) materials: metallic, roughness, albedo, normal maps
- HDRI (High Dynamic Range Imaging) lighting: realistic environment reflections, sky boxes
- Camera systems: perspective cameras, orthographic cameras, multi-camera setups for HRI
- Real-time synchronization: subscribing to `/joint_states` topic in Unity, updating robot pose
- HRI scenarios: adding virtual human models, interaction visualization, safety zone rendering
- Learning objectives, 2.5-hour time estimate
- Exercise 3.1: Import humanoid arm URDF into Unity, verify hierarchy
- Exercise 3.2: Apply PBR materials and HDRI lighting for photorealistic render
- Exercise 3.3: Connect Unity to ROS 2 via TCP Endpoint, synchronize joint movements
- Exercise 3.4: Create simple HRI scenario with virtual human model

**Chapter 4: Sensor Simulation & Cross-Tool Validation**

- Why simulate sensors: Algorithm development, edge case testing, training data generation (before hardware)
- Gazebo sensor plugins overview:
  - `gazebo_ros_ray_sensor`: LiDAR/laser scanners (publishes sensor_msgs/LaserScan)
  - `gazebo_ros_camera`: RGB cameras, depth cameras (publishes sensor_msgs/Image, sensor_msgs/CameraInfo)
  - `gazebo_ros_imu_sensor`: Inertial measurement units (publishes sensor_msgs/Imu)
- Sensor plugin XML syntax: update rate, topic name, frame ID, sensor-specific parameters
- LiDAR configuration: range (min/max), resolution (samples, ray count), FOV (horizontal/vertical)
- Depth camera configuration: image resolution, clip planes (near/far), intrinsic parameters
- IMU configuration: linear acceleration noise, angular velocity noise, orientation drift
- Sensor noise modeling: Gaussian noise, bias, outliers (realistic vs. ideal sensors)
- ROS 2 sensor message types: sensor_msgs/LaserScan, sensor_msgs/Image (RGB/depth), sensor_msgs/Imu
- TF (Transform) frames: sensor coordinate systems, robot base frame, world frame
- Visualizing sensor data in RViz: point clouds (LaserScan), image viewers, IMU orientation
- Unity sensor equivalents: Unity Lidar Sensor, Camera component (RGB/depth), custom IMU scripts
- Cross-tool validation: comparing Gazebo and Unity sensor outputs for consistency
  - Expected: Similar data patterns with minor rendering differences
  - Debugging discrepancies: frame rate mismatches, coordinate system errors, plugin configuration
- Learning objectives, 2-hour time estimate
- Exercise 4.1: Add LiDAR plugin to humanoid head URDF, visualize point cloud in RViz
- Exercise 4.2: Add depth camera plugin, subscribe to RGB and depth topics
- Exercise 4.3: Add IMU plugin to torso, validate orientation and acceleration data
- Exercise 4.4: Compare sensor outputs between Gazebo and Unity, debug discrepancies


### Exercises (`exercise-plans.md`)

| Exercise | Chapter | Description | Validation Criteria | Common Errors |
|----------|---------|-------------|---------------------|---------------|
| 1.1 | 1 | Categorize 10 scenarios as Gazebo, Unity, or both | 9/10 correct categorizations | Confusing physics testing with visualization; not recognizing combined workflows |
| 2.1 | 2 | Load humanoid arm URDF in Gazebo | Model loads without errors, responds to gravity, joints move freely | URDF syntax errors, missing mesh files, incorrect package paths |
| 2.2 | 2 | Publish joint commands via ROS 2 | Robot joints move to commanded positions with realistic dynamics | Topic name mismatch, message type errors, ros_gz_bridge not running |
| 2.3 | 2 | Create collision test world | Gazebo detects collisions, prevents interpenetration | Missing collision geometry, incorrect contact properties (kp, kd) |
| 2.4 | 2 | Compare physics engines (ODE vs. Bullet) | Identify behavioral differences (contact stability, computation speed) | Not restarting simulation after engine change, unrealistic expectations of differences |
| 3.1 | 3 | Import URDF into Unity | Robot appears with correct hierarchy, all links/joints present | Mesh path errors, unsupported URDF tags, coordinate system mismatch |
| 3.2 | 3 | Apply PBR materials and HDRI lighting | Render achieves photorealistic quality suitable for presentations | Incorrect material properties, missing HDRI asset, poor lighting setup |
| 3.3 | 3 | Connect Unity to ROS 2 via TCP Endpoint | Joint states synchronize in real-time between ROS and Unity | Firewall blocking TCP connection, incorrect IP address (localhost vs. WSL2 IP), message type mismatch |
| 3.4 | 3 | Create HRI scenario with virtual human | Human and robot positioned realistically, interaction visible | Human model scale mismatch, unrealistic positioning, missing animation |
| 4.1 | 4 | Add LiDAR plugin, visualize in RViz | Point cloud data publishes to /scan at specified rate, visualizes correctly | Update rate = 0, topic name typo, plugin not loaded in URDF |
| 4.2 | 4 | Add depth camera plugin | RGB and depth images publish to separate topics | Image encoding errors, clip plane misconfiguration, camera frame not in TF tree |
| 4.3 | 4 | Add IMU plugin, validate data | IMU publishes orientation, linear acceleration, angular velocity with realistic noise | Unrealistic noise values, missing frame_id, orientation quaternion not normalized |
| 4.4 | 4 | Cross-tool sensor validation | Sensor outputs consistent between Gazebo and Unity (within tolerance) | Frame rate mismatch, coordinate system not transformed, sensor parameters differ between tools |


### Module 2 Project (`project-rubric.md`)

**Project Title**: Building and Evaluating a Humanoid Digital Twin

**Project Requirements**:
- Create humanoid subsystem URDF (arm, torso, or leg) with at least 3 joints and realistic inertial properties
- Simulate in Gazebo: load model, apply joint commands, verify stable physics behavior (no unstable oscillations, correct gravity response)
- Add at least 2 sensors: one vision-based (LiDAR or depth camera) and one inertial (IMU)
- Import same URDF into Unity: create photorealistic render with PBR materials and HDRI lighting
- Demonstrate synchronized behavior: joint commands sent via ROS 2 update both Gazebo and Unity in real-time
- Document workflow: URDF design rationale, Gazebo test results (screenshots, physics metrics), Unity render screenshots, sensor validation (data samples)

**Project Rubric** (100 points):

| Category | Points | Criteria |
|----------|--------|----------|
| **Physics Accuracy (Gazebo)** | 40 | URDF model with valid inertial properties (10 pts); stable simulation with realistic gravity and joint dynamics (15 pts); collision detection working correctly (10 pts); joint commands applied via ROS 2 with expected behavior (5 pts) |
| **Visualization Quality (Unity)** | 30 | URDF successfully imported with correct hierarchy (10 pts); PBR materials and HDRI lighting applied for photorealistic quality (10 pts); real-time synchronization with ROS 2 joint states (10 pts) |
| **Sensor Fidelity** | 20 | Vision sensor (LiDAR or depth camera) configured and publishing valid data (10 pts); IMU sensor configured and publishing valid data (5 pts); sensor outputs consistent between Gazebo and Unity (5 pts) |
| **Documentation & Clarity** | 10 | README with installation steps and usage instructions (3 pts); design rationale explaining URDF choices (3 pts); test results with screenshots and sensor data samples (4 pts) |
| **Total** | **100** | |

**Passing**: 70/100
**Excellence**: 90/100

**Bonus Points** (optional, max +10):
- HRI scenario in Unity with virtual human model (+5 pts)
- Advanced sensor configuration with realistic noise modeling (+3 pts)
- Performance optimization (Gazebo real-time factor >0.9, Unity >60 FPS) (+2 pts)

**Output**: chapter-outlines.md, exercise-plans.md, project-rubric.md


## Phase 2: Tasks

**Run**: `/sp.tasks` to generate task breakdown

**Output**: tasks.md

## Success Metrics

1. **Tool Selection Accuracy**: 90% correctly categorize 9/10 scenarios as Gazebo (physics) vs. Unity (visual) use cases
2. **Gazebo Loading Success**: 85% successfully load URDF model in Gazebo and apply joint commands via ROS 2 topics
3. **Sensor Configuration**: 80% correctly configure at least 2 sensor types (LiDAR, depth camera, or IMU) in Gazebo
4. **Unity Rendering**: 75% import URDF into Unity and create photorealistic render with materials and lighting
5. **Project Completion**: 80% complete Module 2 Project (Gazebo testing + Unity visualization) within 8-10 hours
6. **Quiz Performance**: 90% score 80%+ on end-of-module quiz covering digital twin concepts, tool selection, and sensor configuration
7. **Time Accuracy**: Average completion times match estimates within ±20% (Ch1: 1.5hr, Ch2: 2.5hr, Ch3: 2.5hr, Ch4: 2hr)
8. **Confidence**: 85% report "confident" or "very confident" with Gazebo physics simulation and Unity rendering basics
9. **Interview Performance**: 80% correctly explain when to use Gazebo vs. Unity in technical interviews or project proposals


## Implementation Notes

### Content Development Order

1. Phase 0: Research (all technical choices finalized, best practices identified, dual-tool workflows validated)
2. Phase 1: Chapter outlines + exercises (parallel development possible for content vs. code examples)
3. Phase 2: Content creation with recommended sequencing:
   - Chapter 1: Digital twin concepts (prerequisite for all)
   - Chapter 2 + Chapter 3: Can develop in parallel (different tools - Gazebo vs. Unity)
   - Chapter 4: Requires both Ch2 and Ch3 complete (integrates sensors across both tools)
   - Module 2 Project: Depends on all 4 chapters

### Dependencies

**Chapter Dependencies**:
- Chapter 1: No dependencies (foundational concepts)
- Chapter 2 (Gazebo): Requires Chapter 1 (digital twin decision criteria)
- Chapter 3 (Unity): Requires Chapter 1 (digital twin decision criteria)
- Chapter 4 (Sensors): Requires Chapter 2 (Gazebo sensor plugins) AND Chapter 3 (Unity visualization)
- Module 2 Project: Requires all 4 chapters complete

**Parallel Development Opportunities**:
- Chapter 2 (Gazebo) and Chapter 3 (Unity) can be developed concurrently by different authors
- Code examples can be developed in parallel with content writing
- Installation guides (Ubuntu native vs. WSL2) can be written independently

**Critical Path**: Chapter 1 → (Chapter 2 ∥ Chapter 3) → Chapter 4 → Module 2 Project

### Installation Environment Matrix

| Environment | ROS 2 Humble | Gazebo Harmonic | Unity 2022 LTS | Notes |
|-------------|--------------|-----------------|----------------|-------|
| **Ubuntu 22.04 Native** | ✅ Native install | ✅ Native install | ✅ Native install | Recommended environment; full GPU support |
| **WSL2 (Ubuntu 22.04)** | ✅ Native install | ✅ With X11 forwarding | ⚠️ Windows host only | Unity on Windows connects to ROS 2 in WSL2 via TCP; requires firewall configuration |
| **Docker (Ubuntu 22.04 base)** | ✅ In container | ✅ In container | ❌ Not recommended | Gazebo visualization requires X11; Unity requires host installation |
| **Ubuntu 20.04** | ⚠️ Requires building from source | ❌ Not supported | ✅ Native install | NOT recommended; upgrade to 22.04 or use Docker |

**Recommended Setup**:
- **Best**: Ubuntu 22.04 native with GPU (NVIDIA proprietary drivers)
- **WSL2 Users**: ROS 2 + Gazebo in WSL2, Unity 2022 LTS on Windows host
- **Fallback**: Docker for Gazebo (headless mode with RViz on host), Unity on host


### Risk Mitigation

| Risk | Mitigation |
|------|------------|
| **Gazebo Harmonic installation failures** (dependency conflicts, version confusion with Classic) | Provide explicit uninstall instructions for Gazebo Classic; Docker container with Gazebo Harmonic pre-installed; detailed apt troubleshooting guide; command differentiation chart (`gazebo` vs. `gz`) |
| **WSL2 + Unity complexity** (TCP bridge setup, firewall rules, IP addressing) | Provide step-by-step WSL2 installation guide with screenshots; pre-configured Unity TCP Endpoint settings; Windows Defender firewall rule templates; troubleshooting flowchart for connection issues |
| **GPU unavailable or incompatible** (cloud environments, VMs, integrated graphics) | CPU-only rendering instructions for Unity; Gazebo headless mode with RViz-only visualization; reference hardware recommendations; cloud alternatives (e.g., AWS EC2 with GPU passthrough) |
| **ros_gz_bridge topic mapping errors** (message type mismatches, remapping syntax) | Provide tested bridge configuration files as templates; document common message type conversions (geometry_msgs, sensor_msgs); include `ros2 topic echo` debugging workflow |
| **URDF import failures in Unity** (mesh paths, unsupported tags, coordinate mismatch) | Pre-validated URDF templates with known-good structure; troubleshooting guide for common import errors; coordinate system transformation diagram (ROS Z-up → Unity Y-up) |
| **Version incompatibilities** (Gazebo Harmonic API changes, Unity Robotics Hub updates) | Lock dependency versions in installation guide (Gazebo Harmonic 7.x, Unity 2022.3 LTS, Unity Robotics Hub 0.7.0); quarterly testing against latest patch releases; version-specific troubleshooting sections |
| **Module scope too broad for 8-10 hours** | Beta test with target audience; prioritize P1 user stories (Gazebo, sensors); make Chapter 3 (Unity) optional for minimal project completion; provide "quick start" path skipping advanced rendering |
| **Sensor plugin configuration complexity** (XML syntax errors, parameter tuning) | Provide tested sensor plugin XML snippets; parameter explanation tables with units and typical ranges; pre-configured example models with sensors already integrated |
| **Students lack Module 1 prerequisites** (URDF fundamentals, ROS 2 topics) | Prerequisite quiz before Module 2 access; Module 1 refresher appendix (10-min crash course on URDF, nodes, topics); link to Module 1 completion certificate |
| **Dual-tool synchronization issues** (timing delays, data inconsistency) | Document expected latency (ROS 2 ↔ Gazebo: <50ms, ROS 2 ↔ Unity: <100ms); troubleshooting guide for desynchronization; frame rate matching recommendations |


### QA Plan

**Phase 0 QA (Research Validation)**:
1. Verify all dependency versions are current stable releases (Gazebo Harmonic 7.x, Unity 2022.3 LTS, Unity Robotics Hub 0.7.0)
2. Test installation procedures on Ubuntu 22.04 native and WSL2
3. Validate dual-tool workflow examples (Gazebo + Unity synchronized simulation)
4. Confirm sensor plugin examples publish valid ROS 2 messages

**Phase 1 QA (Design Review)**:
1. Technical review by Gazebo/Unity experts (verify architectural accuracy, identify missing edge cases)
2. Pedagogical review by robotics educators (assess learning progression, exercise difficulty calibration)
3. Terminology compliance check (Module/Chapter structure, no "Lesson" usage)
4. Exercise validation criteria completeness (all exercises have clear success metrics)

**Phase 2 QA (Content Testing)**:
1. **Technical Accuracy Testing**:
   - All Gazebo examples run on Gazebo Harmonic 7.x without errors
   - All Unity examples work with Unity 2022.3 LTS and Unity Robotics Hub 0.7.0
   - Sensor plugins publish data to correct ROS 2 topics with valid message formats
   - ros_gz_bridge and Unity TCP Endpoint connect successfully

2. **Beginner Testing** (users with Module 1 completion, no prior Gazebo/Unity experience):
   - Installation guides enable successful setup (target: 90% success rate)
   - Exercises completable with provided instructions (target: 80% completion rate)
   - Common errors anticipated and documented in troubleshooting guides
   - Time estimates validated (±20% accuracy)

3. **Platform Testing**:
   - Ubuntu 22.04 native: Full feature testing (Gazebo + Unity + sensors)
   - WSL2: Gazebo in WSL2, Unity on Windows host, TCP connection validation
   - Docker: Gazebo headless mode, RViz visualization, Unity on host
   - GPU vs. CPU-only rendering performance benchmarking

4. **Accessibility Check**:
   - All technical terms defined on first use
   - Diagrams include descriptive alt text
   - Code examples use syntax highlighting and include explanatory comments
   - No unexplained jargon or assumed knowledge beyond Module 1 prerequisites

5. **Cross-Tool Integration Testing**:
   - Gazebo simulation data visible in Unity in real-time
   - Joint commands from Unity control Gazebo simulation
   - Sensor data consistent between Gazebo and Unity (within documented tolerance)
   - Frame rate synchronization validated (Gazebo RTF >0.8, Unity >30 FPS)

6. **Time Validation**:
   - Track actual completion times with beta testers
   - Chapter 1: Target 1.5 hours (±20 minutes)
   - Chapter 2: Target 2.5 hours (±30 minutes)
   - Chapter 3: Target 2.5 hours (±30 minutes)
   - Chapter 4: Target 2 hours (±25 minutes)
   - Module 2 Project: Target 3-4 hours (±1 hour)
   - Total: 8-10 hours

7. **Module 3 Integration Testing** (if Module 3 exists):
   - Ensure Module 2 outputs (digital twin models, simulation configurations) work as inputs for Module 3
   - Validate terminology consistency across modules
   - Check prerequisite alignment (Module 2 skills enable Module 3 tasks)

**Acceptance Criteria for Plan Approval**:
- [ ] Constitution check passes all 5 principles
- [ ] All chapter outlines include learning objectives, time estimates, exercises
- [ ] Exercise plans include validation criteria and common errors for all exercises
- [ ] Project rubric totals 100 points with clear grading criteria
- [ ] Dependencies explicitly listed with version numbers
- [ ] Risk mitigation strategies documented for all high-impact risks
- [ ] Installation environment matrix covers Ubuntu native, WSL2, Docker
- [ ] QA plan addresses technical accuracy, beginner usability, platform compatibility


## Architectural Decisions

### Decision 1: Dual-Simulation Approach (Gazebo + Unity)

**Context**: Students need to understand both physics accuracy and visual presentation for robotics development.

**Options Considered**:
1. Gazebo only (physics-focused)
2. Unity only (visualization-focused)
3. Gazebo + Unity (dual-tool workflow)
4. Alternative simulators (Isaac Sim, Webots, CoppeliaSim)

**Decision**: Use Gazebo + Unity dual-simulation approach

**Rationale**:
- **Physics Accuracy**: Gazebo provides industry-standard physics engines (ODE, Bullet, Simbody) for validation
- **Visual Fidelity**: Unity offers photorealistic rendering for stakeholder presentations and HRI testing
- **Real-World Workflow**: Industry uses specialized tools for testing vs. presentation; students learn professional practices
- **ROS 2 Integration**: Both tools have mature ROS 2 integration (ros_gz_bridge, Unity Robotics Hub)
- **Educational Value**: Teaching tool selection criteria prepares students for architecture decisions in projects

**Trade-offs**:
- ✅ Comprehensive skill set (physics + visualization)
- ✅ Real-world workflow alignment
- ❌ Increased installation complexity (two tools vs. one)
- ❌ Higher time commitment (8-10 hours vs. 4-6 hours for single tool)

### Decision 2: Gazebo Harmonic over Gazebo Classic

**Context**: Gazebo has two major versions with different architectures.

**Options Considered**:
1. Gazebo Classic 11 (legacy, widely documented)
2. Gazebo Harmonic (modern, actively developed)

**Decision**: Use Gazebo Harmonic exclusively

**Rationale**:
- **ROS 2 Native Integration**: Harmonic designed for ROS 2; Classic retrofitted
- **Active Development**: Harmonic receives updates; Classic in maintenance mode
- **Modern Architecture**: Ignition Transport (Harmonic) vs. custom middleware (Classic)
- **Future-Proofing**: Students learn current industry standard, not legacy system
- **SDF Evolution**: SDF 1.9+ (Harmonic) supports advanced features

**Trade-offs**:
- ✅ Modern skills, active community support
- ✅ Better performance and ROS 2 integration
- ❌ Less online documentation compared to Classic
- ❌ Migration complexity for students with prior Classic experience

**Mitigation**: Provide Gazebo Classic vs. Harmonic comparison chart, explicit command differentiation (`gazebo` vs. `gz`)

### Decision 3: Unity 2022 LTS over Latest Unity Version

**Context**: Unity releases new versions frequently; LTS provides stability.

**Options Considered**:
1. Unity 2022 LTS (stable, long-term support)
2. Unity 2023/2024 (latest features, shorter support)
3. Unity 6 LTS (next LTS, released late 2024)

**Decision**: Use Unity 2022.3 LTS

**Rationale**:
- **Stability**: LTS versions receive bug fixes for 2 years, feature updates for 1 year
- **Unity Robotics Hub Compatibility**: Tested and verified with Unity 2022 LTS
- **Educational Environment**: Stability preferred over cutting-edge features
- **Cross-Platform Support**: Well-tested on Ubuntu native and Windows with WSL2

**Trade-offs**:
- ✅ Stable, well-documented, compatible with Unity Robotics Hub
- ✅ Long-term support ensures content remains valid
- ❌ Missing latest rendering features (Unity 6 has improved HDRP)
- ❌ Eventual migration needed when Unity 2022 LTS support ends (2026)

**Review Date**: 2025 Q4 (evaluate Unity 6 LTS migration)

### Decision 4: WSL2 Support as Secondary Path

**Context**: Many students use Windows as primary OS.

**Options Considered**:
1. Ubuntu native only (simplest, most performant)
2. WSL2 as primary recommendation
3. WSL2 as secondary path with Ubuntu native preferred
4. Docker containers for cross-platform support

**Decision**: Recommend Ubuntu native, provide WSL2 as secondary path

**Rationale**:
- **Performance**: Native installation provides best GPU access, lowest latency
- **Simplicity**: Single-OS environment easier to troubleshoot
- **WSL2 Complexity**: Requires Windows Unity + WSL2 ROS 2 TCP bridge configuration
- **Student Choice**: Support diverse environments without mandating dual-boot

**Implementation**:
- Primary installation guide: Ubuntu 22.04 native
- Secondary installation guide: WSL2 + Windows Unity with TCP Endpoint setup
- Fallback: Docker for Gazebo (headless), Unity on host

**Trade-offs**:
- ✅ Supports diverse student environments
- ✅ Teaches cross-platform integration (valuable skill)
- ❌ Higher support burden (two installation paths)
- ❌ WSL2 users experience higher complexity and potential latency

---

**Next Command**: `/sp.tasks`

