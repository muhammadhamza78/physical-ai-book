---
id: glossary
title: "Glossary"
sidebar_label: "Glossary"
sidebar_position: 9
description: "Glossary of digital twin, simulation, and robotics visualization terminology for Module 2"
keywords: [glossary, digital twin, gazebo, unity, ros 2, simulation, terminology]
---

# Glossary: Digital Twin & Simulation Terms

This glossary defines key terms used throughout Module 2: The Digital Twin.

---

## A

### Articulation Body
Unity physics component representing multi-body dynamics (robot joints). Replaces older Joint components for robotics applications.

### Axis Conversion
Transformation between coordinate systems. ROS uses Z-up (right-handed), Unity uses Y-up (left-handed). URDF Importer handles this automatically.

---

## B

### Bullet Physics Engine
Physics simulation engine alternative to ODE. Provides more accurate contact dynamics, useful for manipulation tasks. Supported by Gazebo Harmonic.

---

## C

### Collision Geometry
Simplified geometric shapes (boxes, cylinders, spheres) used for physics collision detection. Separate from visual geometry for performance optimization.

### Coordinate System
Framework defining origin and axis directions:
- **ROS/Gazebo**: X=forward, Y=left, Z=up (right-handed)
- **Unity**: X=right, Y=up, Z=forward (left-handed)

---

## D

### Depth Camera
Sensor that provides distance measurements for each pixel, creating a 3D point cloud. Simulated in Gazebo using `gazebo_ros_camera` plugin with depth mode.

### Digital Twin
Virtual replica of a physical robot that mirrors its geometry, physics, sensors, and behavior for testing, visualization, and analysis.

### Dual-Simulation Philosophy
Approach using two complementary tools: Gazebo for physics-accurate simulation + Unity for photorealistic visualization.

---

## E

### Effort Limit
Maximum torque (for revolute joints) or force (for prismatic joints) a joint can apply. Specified in URDF `<limit>` tag.

---

## F

### Fixed Joint
Joint type with no degrees of freedom. Used to rigidly attach sensors or child links to parent links.

### FOV (Field of View)
Angular extent of the observable scene for a camera or sensor. Specified in radians (e.g., 1.047 rad = 60°).

---

## G

### Gazebo Classic
Legacy version of Gazebo (version 11 and earlier). Uses `gazebo` command. Primarily for ROS 1.

### Gazebo Harmonic
Modern version of Gazebo (version 7.x+). Uses `gz sim` command. Native ROS 2 support, modular architecture.

### GPU LiDAR
LiDAR sensor plugin using GPU acceleration for faster ray-tracing. Plugin name: `gpu_lidar`.

---

## H

### HDRI (High Dynamic Range Imaging)
Image format storing wider range of luminosity values. Used in Unity as skybox for realistic environment lighting and reflections.

### Headless Mode
Running Gazebo without GUI for faster simulation. Command: `gz sim -s world.sdf` (server-only).

---

## I

### IMU (Inertial Measurement Unit)
Sensor measuring orientation (quaternion), angular velocity (gyroscope), and linear acceleration (accelerometer). ROS 2 message type: `sensor_msgs/msg/Imu`.

### Inertial Properties
Physical properties of a link: mass, center of mass, and inertia tensor. Required for realistic physics simulation.

### Inertia Tensor
3x3 matrix describing rotational inertia about X, Y, Z axes. Specified in URDF as `ixx`, `ixy`, `ixz`, `iyy`, `iyz`, `izz`.

---

## J

### Joint Damping
Energy dissipation in joint motion. Higher damping = slower, more controlled movement. Units: Nm·s/rad (revolute) or N·s/m (prismatic).

### Joint Dynamics
Physical behavior of joints including damping, friction, effort limits, and velocity limits.

### Joint State
Current position, velocity, and effort of all robot joints. Published on ROS 2 topic `/joint_states` as `sensor_msgs/msg/JointState`.

---

## L

### LiDAR (Light Detection and Ranging)
Laser-based distance sensor creating 2D or 3D point clouds. ROS 2 message type: `sensor_msgs/msg/LaserScan`.

### Link
Rigid body element in URDF/SDF representing a robot part (arm, leg, torso). Contains visual, collision, and inertial properties.

---

## M

### Metallic
PBR material property defining how metallic a surface appears. Range: 0.0 (dielectric/plastic) to 1.0 (pure metal).

### Mesh
3D model file (`.stl`, `.dae`, `.obj`) representing detailed geometry. Used for visual accuracy.

---

## N

### Noise Model
Simulation of real-world sensor imperfections. Types include Gaussian (random), bias (constant offset), and outliers.

---

## O

### ODE (Open Dynamics Engine)
Default physics engine in Gazebo. Fast and stable, suitable for general robotics simulation.

---

## P

### PBR (Physically-Based Rendering)
Rendering approach simulating realistic light-material interactions. Key properties: albedo, metallic, smoothness, normal maps.

### Physics Engine
Software component calculating forces, collisions, and motion. Gazebo supports ODE, Bullet, Simbody.

### Prismatic Joint
Joint type allowing linear motion along one axis. Used for grippers, elevators, sliders.

---

## R

### Real-Time Factor (RTF)
Ratio of simulation speed to real-world time. RTF=1.0 means real-time, RTF=0.5 means half-speed, RTF=2.0 means twice real-time.

### Revolute Joint
Joint type allowing rotational motion about one axis. Used for elbows, knees, shoulders, wheels.

### ros_gz_bridge
ROS 2 package mapping Gazebo topics to ROS 2 topics for bidirectional communication.

### ROS TCP Connector
Unity package enabling TCP/IP connection between Unity and ROS 2. Part of Unity Robotics Hub.

### RTF
See **Real-Time Factor**.

### RViz
ROS 2 visualization tool for sensor data (point clouds, images, TF frames). Command: `ros2 run rviz2 rviz2`.

---

## S

### SDF (Simulation Description Format)
XML format used by Gazebo to define worlds, models, links, joints, and plugins. Current version: 1.9.

### Sensor Plugin
Gazebo plugin adding sensor functionality (LiDAR, cameras, IMU) and publishing data to ROS 2 topics.

### Simbody
Physics engine specialized for biomechanical simulations. Supports muscle simulation and soft robotics.

### Smoothness
PBR material property (inverse of roughness). Range: 0.0 (matte/rough) to 1.0 (glossy/mirror-like).

### Skybox
360-degree background image or HDRI in Unity providing environment lighting and reflections.

---

## T

### TCP Endpoint
Unity component managing TCP/IP connection to ROS 2. Configured with ROS IP address and port (default: 10000).

### TF (Transform Frames)
ROS 2 coordinate frame system defining spatial relationships between robot parts and sensors. Visualized as tree structure.

### Time Step
Duration of each physics simulation step. Smaller steps = more accurate but slower. Typical: 0.001s (1ms).

---

## U

### Unity Hub
Application for managing Unity versions and projects. Required for installing Unity 2022.3 LTS.

### Unity Robotics Hub
Official Unity package suite for ROS integration. Includes ROS TCP Connector and URDF Importer.

### URDF (Unified Robot Description Format)
XML format for describing robot kinematics, dynamics, and visual appearance. Used in ROS and imported into Gazebo/Unity.

### URDF Importer
Unity package converting URDF XML files into Unity GameObjects with correct hierarchy and transforms.

---

## V

### VcXsrv
X server for Windows enabling GUI display for WSL2 Linux applications (including Gazebo).

### Visual Geometry
Detailed mesh or primitive shape used for rendering. Separate from collision geometry.

---

## W

### World File
SDF file defining simulation environment: ground plane, lighting, gravity, physics engine, and initial models.

### WSL2 (Windows Subsystem for Linux 2)
Virtualization layer allowing native Linux kernel on Windows. Enables running ROS 2 and Gazebo on Windows.

---

## X

### X11 Forwarding
Protocol for displaying Linux GUI applications on remote or Windows displays. Required for Gazebo GUI in WSL2.

---

## Acronyms

| Acronym | Full Term | Definition |
|---------|-----------|------------|
| **DOF** | Degrees of Freedom | Number of independent motions (e.g., 3-DOF arm = 3 joints) |
| **FOV** | Field of View | Angular extent of camera/sensor view |
| **FPS** | Frames Per Second | Rendering frame rate (Unity) |
| **HDRI** | High Dynamic Range Imaging | Image format for realistic lighting |
| **HRI** | Human-Robot Interaction | Study/simulation of humans and robots interacting |
| **IMU** | Inertial Measurement Unit | Sensor measuring orientation and acceleration |
| **LTS** | Long-Term Support | Software version with extended support (e.g., Unity 2022.3 LTS) |
| **ODE** | Open Dynamics Engine | Default Gazebo physics engine |
| **PBR** | Physically-Based Rendering | Realistic material rendering approach |
| **ROS** | Robot Operating System | Middleware for robot software |
| **RTF** | Real-Time Factor | Simulation speed ratio |
| **SDF** | Simulation Description Format | Gazebo world/model file format |
| **TCP** | Transmission Control Protocol | Network protocol (used by Unity-ROS connection) |
| **TF** | Transform Frames | ROS coordinate system framework |
| **URDF** | Unified Robot Description Format | Robot description XML format |
| **WSL** | Windows Subsystem for Linux | Linux environment on Windows |

---

## Common Units

| Measurement | Unit | Symbol | Example |
|-------------|------|--------|---------|
| **Angle** | Radians | rad | `1.57 rad` = 90° |
| **Angular Velocity** | Radians per second | rad/s | `2.0 rad/s` |
| **Distance** | Meters | m | `0.5 m` = 50 cm |
| **Force** | Newtons | N | `10 N` |
| **Frequency** | Hertz | Hz | `10 Hz` = 10 updates/second |
| **Linear Acceleration** | Meters per second squared | m/s² | `9.81 m/s²` (gravity) |
| **Mass** | Kilograms | kg | `10 kg` |
| **Time** | Seconds | s | `0.001 s` = 1 ms |
| **Torque** | Newton-meters | Nm | `50 Nm` |

---

## Related Resources

- **Module 1 Glossary**: [ROS 2 terminology](../module-01/glossary.md) (if available)
- **ROS 2 Glossary**: [Official ROS 2 concepts](https://docs.ros.org/en/humble/Concepts.html)
- **Gazebo Tutorials**: [Gazebo Harmonic documentation](https://gazebosim.org/docs/harmonic)
- **Unity Manual**: [Unity 2022 LTS documentation](https://docs.unity3d.com/2022.3/Documentation/Manual/)

---

**Return to**: [Module 2 Overview](./index.md)
