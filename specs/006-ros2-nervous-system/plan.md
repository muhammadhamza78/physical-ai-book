# Implementation Plan: Module 1 - The Robotic Nervous System (ROS 2)

**Branch**: `006-ros2-nervous-system` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)

## Summary

Module 1 teaches students ROS 2 as the "nervous system" of a humanoid robot, covering middleware concepts, nodes, topics, services, and URDF modeling. Students learn conceptual foundations (Chapter 1), node creation with rclpy (Chapter 2), communication patterns (Chapter 3), and robot modeling with URDF (Chapter 4), culminating in an integrative Module 1 Project requiring a multi-node system with at least one topic, one service, and a URDF model.

## Technical Context

**Content Format**: Docusaurus MDX (Markdown + React components)
**Primary Technology**: ROS 2 Humble (LTS release on Ubuntu 22.04 LTS)
**Scripting Language**: Python 3.8+ with rclpy library
**Visualization Tools**: RViz (included in ros-humble-desktop)
**Robot Modeling**: URDF (Unified Robot Description Format) with XML syntax
**Testing Approach**: Student exercises with validation rubrics, quiz assessments, Module 1 Project grading
**Target Platform**: Static site generator (Docusaurus) deployed as web-based curriculum
**Project Type**: Educational content (documentation-based, not software application)
**Learning Duration**: 6-8 hours total (self-paced)
**Content Scope**: 4 chapters + 1 integrative project
**Assets Required**: Sample ROS 2 code (nodes, topics, services), URDF templates, installation guides, troubleshooting documentation

## Constitution Check

*GATE: Must pass before Phase 0 research*

### ✅ Accessibility First - PASS
- Spec requires clear explanations with real-world analogies ("nervous system" metaphor)
- Prerequisites explicitly stated (basic Python knowledge)
- Progressive introduction: concepts → implementation → patterns → modeling
- Edge cases address common beginner issues (installation, Python gaps, URDF complexity)

### ✅ Hands-On Learning Priority - PASS
- FR-007: Each chapter includes hands-on exercises with starter code and validation criteria
- User stories 2-4 require practical implementation (nodes, topics, services, URDF)
- Module 1 Project is hands-on integrative capstone
- SC-002: 85% of students create functional publisher-subscriber node pair

### ✅ Progressive Complexity - PASS
- FR-014: Chapter progression from conceptual (Ch1) → basic implementation (Ch2) → communication patterns (Ch3) → robot modeling (Ch4)
- User story priorities: P1 (understanding, nodes, communication) before P2 (URDF modeling)
- SC-007: Time estimates increase with complexity (Ch1: 1hr, Ch2: 1.5hr, Ch3: 2hr, Ch4: 2hr)

### ✅ Production-Ready Examples - PASS
- FR-008: Code examples use Python 3.8+ and rclpy (ROS 2 Humble - industry standard)
- FR-013: All code includes inline comments explaining ROS 2 concepts
- FR-015: Troubleshooting guide for common setup issues
- Spec specifies testing on Ubuntu 22.04 (native and WSL2)

### ✅ Clear Documentation Standards - PASS
- FR-016: Each chapter defines learning objectives, estimated completion time, and success criteria
- FR-007: Exercises include expected outputs and validation criteria
- Dependencies explicitly listed (ROS 2 Humble, Python 3.8+, rclpy, RViz)
- Prerequisites stated in FR-010

**Gate Result**: ✅ PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/006-ros2-nervous-system/
├── plan.md              # This file
├── research.md          # Phase 0 - Best practices research
├── chapter-outlines.md  # Phase 1 - Content structure
├── exercise-plans.md    # Phase 1 - Exercise specifications
├── project-rubric.md    # Phase 1 - Module 1 Project grading criteria
└── tasks.md             # Phase 2 - /sp.tasks output
```

### Content Structure (Docusaurus)

```text
physical-ai-book/docs/module-01/
├── index.md
├── chapter-01-ros2-middleware.md
├── chapter-02-nodes-topics-services.md
├── chapter-03-python-rclpy-bridge.md
├── chapter-04-urdf-humanoid-robots.md
└── module-01-project.md

physical-ai-book/static/code-examples/module-01/
├── chapter-02/
│   ├── minimal_publisher.py
│   ├── minimal_subscriber.py
│   └── node_lifecycle_demo.py
├── chapter-03/
│   ├── imu_topic_publisher.py
│   ├── calibration_service_server.py
│   ├── calibration_service_client.py
│   └── multi_subscriber_demo.py
└── chapter-04/
    ├── simple_arm_urdf.xml
    ├── humanoid_torso_urdf.xml
    └── urdf_rviz_launcher.py
```

## Phase 0: Research

### Research Tasks

1. **ROS 2 Humble Best Practices** - Survey ROS 2 tutorials, official docs, and community examples for beginner-friendly teaching approaches
2. **rclpy Code Patterns** - Identify minimal viable examples for publisher, subscriber, service server, service client that work on Humble
3. **URDF Humanoid Models** - Find or create simple humanoid URDF examples (3-5 links) suitable for beginners; evaluate existing open-source models
4. **Exercise Design** - Research effective hands-on exercise formats for programming tutorials (validation criteria, common errors, troubleshooting)
5. **Installation Troubleshooting** - Compile common ROS 2 Humble installation issues on Ubuntu 22.04, WSL2, Docker with solutions
6. **Assessment Design** - Research quiz question formats and project rubric criteria for ROS 2 skill evaluation

**Output**: research.md

## Phase 1: Design

### Chapter Content (`chapter-outlines.md`)

**Chapter 1: ROS 2 as Middleware for Robot Control**
- What is middleware? (definition, role in robotics)
- The "robotic nervous system" analogy (nodes as neurons, topics as nerve signals, services as reflex arcs)
- High-level ROS 2 architecture (graph-based communication, distributed processing)
- ROS 1 vs. ROS 2 conceptual differences (DDS middleware, real-time capabilities) - optional background
- Learning objectives, 1-hour time estimate
- Exercise 1.1: Diagram a simple robot system with ROS 2 components

**Chapter 2: ROS 2 Nodes, Topics, and Services**
- What are nodes? (computational units, responsibilities)
- Node lifecycle (initialization, spinning, shutdown)
- Topics: Publish-subscribe pattern (many-to-many, asynchronous streaming)
- Services: Request-response pattern (one-to-one, synchronous calls)
- When to use topics vs. services (decision criteria for humanoid robots)
- Learning objectives, 1.5-hour time estimate
- Exercise 2.1: Create minimal publisher node
- Exercise 2.2: Create minimal subscriber node
- Exercise 2.3: Test topic communication between nodes
- Exercise 2.4: Implement simple service (e.g., "get robot status")

**Chapter 3: Bridging Python Agents to ROS Controllers using rclpy**
- Role of Python agents in robot intelligence (decision-making, planning)
- rclpy as Python interface to ROS 2 (node creation, topic/service APIs)
- Data flow: Agent → ROS → Controller (architectural overview)
- Practical examples: IMU sensor publisher, joint command publisher
- Learning objectives, 2-hour time estimate
- Exercise 3.1: Create IMU topic publisher with simulated data
- Exercise 3.2: Create calibration service (server + client)
- Exercise 3.3: Multi-subscriber demo (one topic, multiple subscribers)

**Chapter 4: Understanding URDF for Humanoid Robots**
- Purpose of URDF (robot description, simulation, visualization, control)
- URDF XML syntax (links, joints, inertial properties)
- Kinematic chains and parent-child relationships
- Joint types (revolute, prismatic, fixed) and their uses
- Humanoid-specific considerations (joint limits, symmetry)
- Loading URDF in RViz for visualization
- Learning objectives, 2-hour time estimate
- Exercise 4.1: Examine simple 3-link arm URDF
- Exercise 4.2: Modify URDF joint limits and visualize
- Exercise 4.3: Create URDF for simple humanoid torso (torso + 2 arms)

**Module 1 Project**
- Create multi-node ROS 2 system with:
  - At least one topic (e.g., joint angle publisher)
  - At least one service (e.g., reset pose service)
  - URDF model for humanoid subsystem (arm, torso, or leg)
- Integrate all components and demonstrate in RViz
- Document installation, usage, and design choices

### Exercises (`exercise-plans.md`)

| Exercise | Chapter | Description | Validation Criteria | Common Errors |
|----------|---------|-------------|---------------------|---------------|
| 1.1 | 1 | Diagram robot system with ROS 2 components | Correct identification of 3+ nodes, 2+ topics, 1+ service | Confusing topics with services; missing communication arrows |
| 2.1 | 2 | Create minimal publisher node | Node runs without errors, publishes to named topic at 1Hz | Forgetting rclpy.init(), incorrect message import |
| 2.2 | 2 | Create minimal subscriber node | Receives and prints messages from publisher | Callback function not registered, topic name mismatch |
| 2.3 | 2 | Test publisher-subscriber communication | Both nodes run concurrently, data transfers correctly | Topic name typos, message type mismatch |
| 2.4 | 2 | Implement simple service | Service server responds to client request with correct data | Service name mismatch, incorrect request/response types |
| 3.1 | 3 | IMU topic publisher with simulated data | Publishes sensor_msgs/Imu at 10Hz with realistic values | Incorrect message fields, unrealistic sensor ranges |
| 3.2 | 3 | Calibration service (server + client) | Client sends request, server processes and returns success/failure | Blocking I/O in service callback, no error handling |
| 3.3 | 3 | Multi-subscriber demo | All subscribers receive same data independently | Assuming message consumption (topics do not work that way) |
| 4.1 | 4 | Examine 3-link arm URDF | Correctly identifies links, joints, parent-child relationships | Misunderstanding joint types, confusing link/joint |
| 4.2 | 4 | Modify URDF joint limits | URDF loads in RViz, joint moves within new limits | Invalid XML syntax, incorrect limit units (degrees vs. radians) |
| 4.3 | 4 | Create URDF for simple humanoid torso | URDF loads without errors, visualizes correctly, joint hierarchy correct | Mesh file path errors, missing robot tag, broken kinematic tree |

### Project Rubric (`project-rubric.md`)

| Category | Points | Criteria |
|----------|--------|----------|
| **ROS 2 Architecture** | 25 | Multi-node system with clear separation of concerns; at least 1 publisher, 1 subscriber, 1 service |
| **Topic Communication** | 20 | Topic publishes meaningful data (e.g., joint angles, sensor readings); subscriber processes data correctly |
| **Service Implementation** | 15 | Service handles requests and returns appropriate responses; error handling included |
| **URDF Model** | 25 | Valid URDF for humanoid subsystem (3+ links, 2+ joints); loads successfully in RViz; kinematic tree correct |
| **Code Quality** | 10 | Code includes inline comments explaining ROS 2 concepts; follows Python best practices; no hardcoded values |
| **Documentation** | 5 | README with installation steps, usage instructions, design rationale |
| **Total** | **100** | |

**Passing**: 70/100
**Excellence**: 90/100

**Output**: chapter-outlines.md, exercise-plans.md, project-rubric.md

## Phase 2: Tasks

**Run**: `/sp.tasks` to generate task breakdown

**Output**: tasks.md

## Success Metrics

1. **Middleware Understanding**: 90% correctly explain ROS 2 role as middleware
2. **Node Creation**: 85% successfully create publisher-subscriber node pair
3. **Communication Pattern Selection**: 80% correctly identify topic vs. service usage in 8/10 scenarios
4. **URDF Modeling**: 75% create valid URDF loading in RViz
5. **Project Completion**: 80% complete Module 1 Project within 6-8 hours
6. **Quiz Performance**: 90% score 80%+ on end-of-module quiz
7. **Time Accuracy**: Average completion times match estimates (Ch1: 1hr, Ch2: 1.5hr, Ch3: 2hr, Ch4: 2hr)
8. **Confidence**: 85% report "confident" or "very confident" with ROS 2 basics

## Implementation Notes

### Content Development Order

1. Phase 0: Research (all technical choices finalized, best practices identified)
2. Phase 1: Chapter outlines + exercises (parallel development possible)
3. Phase 2: Content creation (sequential: Ch1 → Ch2 → Ch3 → Ch4 → Project)

### Dependencies

- Chapter 1: prerequisite for all others (foundational concepts)
- Chapter 2: prerequisite for Chapters 3-4 (needs node understanding)
- Chapter 3: builds on Chapter 2 (rclpy uses nodes/topics/services)
- Chapter 4: can be developed in parallel with Chapter 3 (different focus)
- Module 1 Project: depends on all 4 chapters

### Risk Mitigation

| Risk | Mitigation |
|------|------------|
| ROS 2 installation failures | Provide Docker container + VM image; detailed troubleshooting guide for Ubuntu 22.04, WSL2 |
| Python prerequisite gaps | Include Python refresher appendix (30-min crash course); prerequisite quiz before Chapter 2 |
| URDF complexity overwhelms beginners | Start with 3-link arm example; provide annotated templates; make Chapter 4 optional for Module 1 Project completion |
| RViz visualization issues (no GPU) | Provide command-line alternatives (ros2 topic echo, ros2 node list); cloud-based RViz option |
| Module scope too broad for 6-8 hours | Beta test with target audience; prioritize P1 user stories; mark advanced topics as optional |

### QA Plan

1. Technical review by ROS 2 experts (verify code examples run on Humble)
2. Beginner testing with students having only Python knowledge (no ROS experience)
3. Platform testing (Ubuntu 22.04 native, WSL2, Docker)
4. Accessibility check (clear explanations, no jargon without definitions)
5. Time validation (track actual completion times, adjust estimates)
6. Integration testing with Module 2 (ensure URDF models work in Gazebo/Unity)

---

**Next Command**: `/sp.tasks`
