---
id: chapter-02-gazebo-physics
title: "Chapter 2: Physics Simulation in Gazebo"
sidebar_label: "Ch 2: Gazebo Physics"
sidebar_position: 3
description: "Master Gazebo Harmonic for physics-accurate robot simulation including world files, SDF format, and ROS 2 integration"
keywords: [gazebo, gazebo harmonic, physics simulation, SDF, world files, ros_gz_bridge, collision detection, joint dynamics]
---

# Chapter 2: Physics Simulation in Gazebo

## Learning Objectives

By the end of this chapter, you will be able to:

- ✅ Differentiate between Gazebo Harmonic and Gazebo Classic
- ✅ Create world files using SDF format
- ✅ Select appropriate physics engines for humanoid robots
- ✅ Configure joint dynamics (effort limits, damping, friction)
- ✅ Set up collision detection and test with obstacles
- ✅ Integrate Gazebo with ROS 2 using ros_gz_bridge

**Estimated Time**: 2.5 hours

---

## Gazebo Harmonic vs. Gazebo Classic

### Why Gazebo Harmonic?

**Gazebo Harmonic** (successor to Ignition Garden/Fortress) is the modern version of Gazebo designed specifically for ROS 2.

| Feature | Gazebo Classic 11 | Gazebo Harmonic 7.x |
|---------|-------------------|---------------------|
| **Command Line** | `gazebo` | `gz sim` or `gz` |
| **ROS Integration** | ROS 1 native, ROS 2 retrofitted | ROS 2 native |
| **World Format** | SDF 1.6 | SDF 1.9+ |
| **Architecture** | Monolithic | Modular (Ignition Transport) |
| **Development Status** | Maintenance mode | Active development |
| **Performance** | Good | Better (multithreading) |

**Migration Note**: If you have prior Gazebo Classic experience, the key differences are:
- Command: `gazebo world.world` → `gz sim world.sdf`
- Plugin syntax updated for ROS 2 message types
- Improved physics engine options

---

## SDF (Simulation Description Format)

SDF is the XML format used by Gazebo to define worlds, models, links, joints, and plugins.

### SDF Structure Hierarchy

```
<sdf version="1.9">
  <world name="my_world">
    <physics>         <!-- Physics engine configuration -->
    <light>           <!-- Lighting -->
    <model>           <!-- Robot or object -->
      <link>          <!-- Rigid body -->
        <collision>   <!-- Collision geometry -->
        <visual>      <!-- Visual geometry -->
        <sensor>      <!-- Sensor plugins -->
      </link>
      <joint>         <!-- Connection between links -->
        <dynamics>    <!-- Joint dynamics (damping, friction) -->
      </joint>
      <plugin>        <!-- Model plugins (sensors, controllers) -->
    </model>
  </world>
</sdf>
```

### Key SDF Elements

- **world**: Top-level container for the simulation environment
- **model**: Represents a robot or object (can be composed of links and joints)
- **link**: Rigid body with collision, visual, and inertial properties
- **joint**: Connection between two links (revolute, prismatic, fixed, etc.)
- **plugin**: Adds functionality (sensors, controllers, ROS 2 integration)

---

## Physics Engines in Gazebo

Gazebo Harmonic supports multiple physics engines. Choose based on your simulation needs.

### Physics Engine Comparison

| Engine | Strengths | Best For | Humanoid Robots? |
|--------|-----------|----------|------------------|
| **ODE** | Fast, stable, good general performance | General robotics, mobile robots | ✅ Default choice |
| **Bullet** | Accurate contact dynamics, realistic friction | Complex contacts, manipulation | ✅ High-accuracy grasping |
| **Simbody** | Biomechanical accuracy, muscle simulation | Biomimetic robots, soft robotics | ⚠️ Advanced use |

**Recommendation for Humanoid Robots**: Start with **ODE** for general testing. Switch to **Bullet** if you need more accurate contact forces for manipulation tasks.

### Configuring Physics Engine

In your world SDF file:

```xml
<physics name="1ms" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

To switch to Bullet:

```xml
<physics name="1ms" type="bullet">
  <max_step_size>0.001</max_step_size>
</physics>
```

---

## Creating World Files

A **world file** defines the simulation environment: ground plane, lighting, gravity, and any models.

### Basic World Example

```xml
<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="humanoid_test_world">

    <!-- Gravity (default: -9.81 m/s^2 in Z) -->
    <gravity>0 0 -9.81</gravity>

    <!-- Physics engine -->
    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.5 -1.0</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

**Save as**: `basic_world.sdf`

**Launch**: `gz sim basic_world.sdf`

---

## Loading URDF Models in Gazebo

Gazebo can load URDF files (from Module 1) with automatic URDF-to-SDF conversion.

### Loading URDF via Command Line

```bash
gz sim -r world.sdf --spawn-urdf my_robot.urdf --model-name my_robot --pose "0 0 0.5 0 0 0"
```

### Important Considerations

1. **Inertial Properties**: Gazebo requires `<inertial>` tags in URDF links. Missing inertia causes unrealistic physics.

```xml
<link name="base_link">
  <inertial>
    <mass value="10.0"/>
    <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
  </inertial>
  <!-- collision and visual geometry -->
</link>
```

2. **Mesh Paths**: Use `package://` URIs or absolute paths for mesh files
3. **Package Paths**: Set `GZ_SIM_RESOURCE_PATH` environment variable

---

## Joint Dynamics

Joints connect links and define how they move. Proper joint configuration is critical for realistic humanoid simulation.

### Joint Types

| Type | Description | Humanoid Use Case |
|------|-------------|-------------------|
| **revolute** | Rotational motion with limits | Elbow, knee, hip, shoulder |
| **prismatic** | Linear motion | Gripper fingers (parallel) |
| **fixed** | No motion | Base to world, sensor mounts |
| **ball** | 3-DOF rotation | Experimental (not recommended) |

### Joint Configuration Example

```xml
<joint name="shoulder_pitch" type="revolute">
  <parent>torso</parent>
  <child>upper_arm</child>
  <axis>
    <xyz>0 1 0</xyz>  <!-- Rotation axis -->
    <limit>
      <lower>-1.57</lower>  <!-- -90 degrees in radians -->
      <upper>1.57</upper>   <!-- +90 degrees -->
      <effort>50.0</effort> <!-- Max torque (Nm) -->
      <velocity>2.0</velocity> <!-- Max angular velocity (rad/s) -->
    </limit>
    <dynamics>
      <damping>0.5</damping>    <!-- Joint damping (Nm·s/rad) -->
      <friction>0.1</friction>  <!-- Joint friction (Nm) -->
    </dynamics>
  </axis>
</joint>
```

### Joint Dynamics Parameters

- **damping**: Resists motion (energy dissipation). Higher values = slower motion.
- **friction**: Static and dynamic friction. Models real-world joint resistance.
- **effort**: Maximum torque the joint can apply.
- **velocity**: Maximum angular velocity.

**Tuning Tips**:
- Start with low damping (0.1-1.0) and increase if joints oscillate
- Friction should be small (0.01-0.5) unless modeling sticky joints
- Set realistic effort limits based on motor specifications

---

## Collision Detection

Gazebo uses separate geometries for **collision** (physics) and **visual** (rendering).

### Collision vs. Visual Geometry

- **Collision**: Simplified geometry for fast physics computation (boxes, cylinders, spheres)
- **Visual**: Detailed meshes for realistic appearance

### Collision Configuration

```xml
<link name="upper_arm">
  <!-- Collision geometry (simplified) -->
  <collision name="collision">
    <geometry>
      <cylinder>
        <radius>0.05</radius>
        <length>0.3</length>
      </cylinder>
    </geometry>
    <surface>
      <contact>
        <ode>
          <kp>1e6</kp>  <!-- Contact stiffness -->
          <kd>100</kd>  <!-- Contact damping -->
        </ode>
      </contact>
      <friction>
        <ode>
          <mu>0.8</mu>  <!-- Coefficient of friction -->
          <mu2>0.8</mu2>
        </ode>
      </friction>
    </surface>
  </collision>

  <!-- Visual geometry (detailed) -->
  <visual name="visual">
    <geometry>
      <mesh>
        <uri>package://my_robot/meshes/upper_arm.stl</uri>
      </mesh>
    </geometry>
  </visual>
</link>
```

### Contact Properties

- **kp** (stiffness): Higher values = harder contacts (less penetration)
- **kd** (damping): Higher values = less bouncy contacts
- **mu** (friction): Coefficient of friction (0.0 = frictionless, 1.0 = high friction)

---

## ROS 2 Integration with ros_gz_bridge

The **ros_gz_bridge** package maps Gazebo topics to ROS 2 topics for bidirectional communication.

### Installing ros_gz_bridge

```bash
sudo apt install ros-humble-ros-gz-bridge
```

### Bridge Configuration

Create a bridge configuration YAML file (`bridge_config.yaml`):

```yaml
# Gazebo topic -> ROS 2 topic mapping
- topic_name: "/world/my_world/model/my_robot/joint_state"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

### Launching the Bridge

```bash
ros2 run ros_gz_bridge parameter_bridge bridge_config.yaml
```

Or in a launch file:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['bridge_config.yaml'],
            output='screen'
        )
    ])
```

---

## Debugging Physics Issues

### Common Issues and Solutions

| Issue | Symptom | Solution |
|-------|---------|----------|
| **Robot falls through floor** | Model penetrates ground | Check collision geometries, increase contact kp |
| **Unstable oscillations** | Robot shakes violently | Reduce time step, increase joint damping |
| **Slow simulation** | RTF < 0.5 | Simplify collision meshes, increase max_step_size |
| **Joints don't move** | No response to commands | Check joint limits, verify ros_gz_bridge topics |
| **Unrealistic motion** | Robot floats or behaves strangely | Add inertial properties to all links |

### Debugging Tools

1. **Check topics**:
   ```bash
   gz topic -l
   ```

2. **Inspect model**:
   ```bash
   gz model -m my_robot -i
   ```

3. **Monitor real-time factor**:
   - RTF = 1.0: Real-time
   - RTF > 1.0: Faster than real-time
   - RTF < 1.0: Slower than real-time

4. **Visualize contacts** (in Gazebo GUI):
   - View → Contacts (shows collision points)

---

## Exercises

### Exercise 2.1: Load Humanoid Arm URDF in Gazebo

**Objective**: Load a 3-link humanoid arm URDF and verify it responds to gravity.

**Steps**:
1. Use the humanoid arm URDF from Module 1
2. Ensure all links have `<inertial>` tags
3. Launch Gazebo with the model
4. Observe the arm falling due to gravity

**Validation Criteria**:
- ✅ Model loads without errors
- ✅ Arm falls downward under gravity
- ✅ Joints move freely (no locked behavior)

**Troubleshooting**:
- Missing inertia → Add `<inertial>` tags to all links
- Model not found → Check `GZ_SIM_RESOURCE_PATH`
- Mesh errors → Verify mesh file paths

### Exercise 2.2: Publish Joint Commands via ROS 2

**Objective**: Control joint positions by publishing ROS 2 messages.

**Steps**:
1. Add a joint state publisher plugin to your URDF
2. Launch Gazebo and ros_gz_bridge
3. Publish joint commands using `ros2 topic pub`
4. Observe robot joints moving to commanded positions

**Validation Criteria**:
- ✅ Joints move to commanded positions
- ✅ Motion is smooth (no jerky behavior)
- ✅ Realistic physics (gravity, inertia)

**Example Command**:
```bash
ros2 topic pub /joint_commands std_msgs/msg/Float64MultiArray "{data: [0.5, 1.0, -0.5]}"
```

### Exercise 2.3: Create Collision Test World

**Objective**: Test collision detection with obstacles.

**Steps**:
1. Create a world with a box obstacle
2. Load your humanoid arm model
3. Command the arm to move toward the obstacle
4. Verify Gazebo prevents interpenetration

**Validation Criteria**:
- ✅ Gazebo detects collision
- ✅ Arm stops at obstacle (no penetration)
- ✅ Contact forces appear realistic

### Exercise 2.4: Compare Physics Engines (ODE vs. Bullet)

**Objective**: Understand differences between physics engines.

**Steps**:
1. Run the same simulation with ODE
2. Switch to Bullet in the world file
3. Compare behavior (contact stability, computation speed)
4. Document RTF and qualitative differences

**Validation Criteria**:
- ✅ Both engines run successfully
- ✅ Behavioral differences identified
- ✅ Performance metrics recorded (RTF)

---

## Key Takeaways

1. **Gazebo Harmonic** is the modern version for ROS 2 (command: `gz sim`)
2. **SDF format** defines worlds, models, links, joints, and plugins
3. **Physics engines**: ODE (default), Bullet (accurate contacts), Simbody (biomechanics)
4. **Joint dynamics** require proper damping, friction, effort, and velocity limits
5. **Collision detection** uses simplified geometry for performance
6. **ros_gz_bridge** integrates Gazebo with ROS 2 topics

---

## Next Steps

**Next Chapter**: [Chapter 3: High-Fidelity Rendering in Unity](./chapter-03-unity-rendering-interaction.md)

Learn how to import URDF models into Unity, apply photorealistic materials and lighting, and synchronize with ROS 2 for real-time visualization.
