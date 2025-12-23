---
id: chapter-03-unity-rendering
title: "Chapter 3: High-Fidelity Rendering in Unity"
sidebar_label: "Ch 3: Unity Rendering"
sidebar_position: 4
description: "Create photorealistic robot visualizations in Unity with PBR materials, HDRI lighting, and ROS 2 synchronization"
keywords: [unity, unity robotics hub, urdf importer, pbr materials, hdri lighting, ros 2, tcp endpoint, photorealistic rendering]
---

# Chapter 3: High-Fidelity Rendering & Interaction in Unity

## Learning Objectives

By the end of this chapter, you will be able to:

- ✅ Understand Unity Robotics Hub architecture and components
- ✅ Import URDF models into Unity using the URDF Importer
- ✅ Apply PBR materials and HDRI lighting for photorealistic renders
- ✅ Connect Unity to ROS 2 via TCP Endpoint for real-time synchronization
- ✅ Create human-robot interaction (HRI) scenarios with virtual humans

**Estimated Time**: 2.5 hours

---

## Unity's Role in Robotics

Unity is a game engine repurposed for robotics **visualization** and **human-robot interaction** testing. It is NOT used for real-time robot control (that's Gazebo's job).

### Why Unity for Robotics?

- 🎨 **Photorealistic Rendering**: PBR materials, HDRI lighting, ray tracing
- 🤝 **HRI Visualization**: Add virtual humans to test interaction scenarios
- 📸 **Perception Training Data**: Generate synthetic images for ML models
- 💼 **Stakeholder Communication**: Professional-quality renders for presentations

### Unity vs. Gazebo

| Feature | Gazebo | Unity |
|---------|--------|-------|
| **Primary Purpose** | Physics simulation | Visual rendering |
| **Accuracy** | Physics-accurate | Visually-accurate |
| **Use Cases** | Algorithm testing | Demos, HRI, perception |
| **Rendering Quality** | Basic | Photorealistic |
| **Real-Time Control** | ✅ Yes | ❌ No |

---

## Unity Robotics Hub Architecture

**Unity Robotics Hub** provides ROS 2 integration for Unity projects.

### Key Components

1. **TCP Endpoint**: Unity plugin that connects to ROS 2 via TCP sockets
2. **ROS Message Serialization**: Converts ROS 2 messages to Unity C# objects
3. **URDF Importer**: Parses URDF XML and creates Unity GameObjects with correct hierarchy

### Architecture Diagram

```
Unity Editor <--> TCP Endpoint Plugin <-TCP/IP-> ROS TCP Connector Node <-ROS 2-> Gazebo/Nodes
```

### Installation

1. **Unity 2022.3 LTS**: Download from Unity Hub
2. **Unity Robotics Hub**: Add via Unity Package Manager

```
Window → Package Manager → + → Add package from git URL
https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
```

3. **URDF Importer**: Add via Unity Package Manager

```
https://github.com/Unity-Technologies/URDF-Importer.git?path=/com.unity.robotics.urdf-importer
```

---

## Importing URDF Models into Unity

### URDF Importer Workflow

1. **Open Unity Project**
2. **Select**: `Robotics → Import URDF`
3. **Choose URDF file**
4. **Configure import settings**
5. **Generate GameObjects**

### Import Settings

| Setting | Description | Recommended Value |
|---------|-------------|-------------------|
| **Axis Conversion** | ROS uses Z-up, Unity uses Y-up | ✅ Enable Z-to-Y conversion |
| **Mesh Scale** | Scale factor for imported meshes | 1.0 (if URDF uses meters) |
| **Joint Axis** | Joint rotation axis mapping | Auto-detect (default) |

### GameObject Hierarchy

After import, Unity creates a hierarchy matching the URDF link-joint structure:

```
Robot (root GameObject)
├── base_link
│   ├── MeshRenderer (visual geometry)
│   ├── Collider (collision geometry)
│   └── ArticulationBody (physics)
├── shoulder_joint
│   └── upper_arm
│       ├── MeshRenderer
│       └── elbow_joint
│           └── forearm
```

### Coordinate System Transformation

**ROS uses Z-up** (right-handed):
- X: Forward
- Y: Left
- Z: Up

**Unity uses Y-up** (left-handed):
- X: Right
- Y: Up
- Z: Forward

The URDF Importer automatically handles this transformation with axis conversion enabled.

---

## PBR Materials (Physically-Based Rendering)

PBR materials simulate how light interacts with surfaces for photorealistic results.

### Key PBR Properties

| Property | Description | Example Values |
|----------|-------------|----------------|
| **Albedo** | Base color (diffuse) | RGB values |
| **Metallic** | How metallic the surface is | 0.0 (plastic) to 1.0 (metal) |
| **Smoothness** | Surface roughness (inverse) | 0.0 (rough) to 1.0 (glossy) |
| **Normal Map** | Surface detail without geometry | Texture file (.png) |
| **Emission** | Self-illumination | Color + intensity |

### Creating PBR Materials in Unity

1. **Create Material**: `Assets → Create → Material`
2. **Set Shader**: Universal Render Pipeline → Lit
3. **Configure Properties**:
   - Albedo: Base color texture or RGB
   - Metallic: Slider (0-1)
   - Smoothness: Slider (0-1)
   - Normal Map: Texture (optional)

### Example: Metal Robot Arm

```
Albedo: Gray (RGB: 0.5, 0.5, 0.5)
Metallic: 0.9 (very metallic)
Smoothness: 0.7 (slightly rough metal)
Normal Map: metal_normal.png (adds surface detail)
```

### Example: Plastic Robot Base

```
Albedo: White (RGB: 0.9, 0.9, 0.9)
Metallic: 0.0 (non-metallic)
Smoothness: 0.4 (matte plastic)
```

---

## HDRI Lighting

**HDRI (High Dynamic Range Imaging)** provides realistic environment lighting and reflections.

### Why HDRI?

- 🌅 **Realistic Reflections**: Robots reflect the environment (sky, buildings)
- 💡 **Natural Lighting**: Mimics outdoor or indoor lighting conditions
- 🎨 **Ambient Occlusion**: Soft shadows in crevices

### Setting Up HDRI in Unity

1. **Download HDRI**: Free sources include [Poly Haven](https://polyhaven.com/hdris)
2. **Import to Unity**: Drag `.hdr` file into `Assets`
3. **Create Skybox Material**:
   - `Assets → Create → Material`
   - Shader: `Skybox → Panoramic`
   - Assign HDRI texture to `Spherical (HDR)` slot
4. **Assign to Scene**:
   - `Window → Rendering → Lighting`
   - Environment → Skybox Material → Select your HDRI material

### Recommended HDRI Settings

- **Exposure**: 1.0 (adjust for brightness)
- **Rotation**: 0-360 (rotate environment)
- **Resolution**: 4K minimum for quality reflections

---

## Real-Time ROS 2 Synchronization

### Unity TCP Endpoint Setup

1. **Add ROS TCP Connector** to your Unity scene
2. **Configure Settings**:
   - ROS IP Address: `127.0.0.1` (localhost) or WSL2 IP
   - ROS Port: `10000` (default)
   - Protocol: TCP

3. **Create ROS Connection GameObject**:

```csharp
using Unity.Robotics.ROSTCPConnector;

public class ROSConnection : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Connect();
    }
}
```

### Subscribing to /joint_states

Create a C# script to subscribe to ROS 2 `/joint_states` topic:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class JointStateSubscriber : MonoBehaviour
{
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<JointStateMsg>("/joint_states", UpdateJointStates);
    }

    void UpdateJointStates(JointStateMsg jointState)
    {
        // Update Unity ArticulationBody joints to match ROS joint states
        for (int i = 0; i < jointState.name.Length; i++)
        {
            string jointName = jointState.name[i];
            float position = (float)jointState.position[i];

            // Find corresponding ArticulationBody and update position
            ArticulationBody joint = FindJoint(jointName);
            if (joint != null)
            {
                joint.SetDriveTarget(ArticulationDriveAxis.X, position * Mathf.Rad2Deg);
            }
        }
    }

    ArticulationBody FindJoint(string name)
    {
        // Implement joint lookup logic
        return null; // Placeholder
    }
}
```

### Publishing Unity Data to ROS 2

Publish Unity camera images or user inputs back to ROS 2:

```csharp
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;

public class UnityToROSPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/unity_input";

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>(topicName);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            StringMsg msg = new StringMsg("User pressed spacebar");
            ros.Publish(topicName, msg);
        }
    }
}
```

---

## Human-Robot Interaction (HRI) Scenarios

### Adding Virtual Humans

1. **Download Human Models**: Unity Asset Store or free sources (Mixamo)
2. **Import to Unity**: Drag FBX file into `Assets`
3. **Position Human**: Place near robot in scene
4. **Animate** (optional): Use Unity Animator for walking/gesturing

### Example HRI Scenario: Hospital Delivery Robot

**Setup**:
1. Import humanoid robot URDF
2. Add hospital environment (hallway 3D model)
3. Place virtual nurse model
4. Add wheelchair model
5. Synchronize robot motion with Gazebo

**Goal**: Visualize robot navigating hallway, avoiding nurse, delivering medicine to patient

---

## Exercises

### Exercise 3.1: Import URDF into Unity

**Objective**: Import a humanoid arm URDF and verify correct hierarchy.

**Steps**:
1. Install Unity Robotics Hub and URDF Importer
2. Import your humanoid arm URDF
3. Inspect GameObject hierarchy
4. Verify all links, joints, and meshes present

**Validation Criteria**:
- ✅ All links appear in hierarchy
- ✅ Joints connect correct parent-child links
- ✅ Meshes render correctly in Scene view

### Exercise 3.2: Apply PBR Materials and HDRI Lighting

**Objective**: Create a photorealistic render suitable for presentations.

**Steps**:
1. Create PBR materials (metal for arms, plastic for base)
2. Apply materials to robot mesh renderers
3. Add HDRI skybox for environment lighting
4. Adjust exposure and rotation for best appearance
5. Capture screenshot

**Validation Criteria**:
- ✅ Materials have realistic metallic/smoothness values
- ✅ HDRI provides natural lighting and reflections
- ✅ Screenshot is presentation-quality

### Exercise 3.3: Connect Unity to ROS 2 via TCP Endpoint

**Objective**: Synchronize Unity visualization with ROS 2 joint states.

**Steps**:
1. Configure ROS TCP Connector (IP, port)
2. Create C# script subscribing to `/joint_states`
3. Update Unity ArticulationBody joints in real-time
4. Verify synchronization with Gazebo simulation

**Validation Criteria**:
- ✅ Unity connects to ROS 2 without errors
- ✅ Joint states update in real-time (`<100ms` latency)
- ✅ Robot motion in Unity matches Gazebo

### Exercise 3.4: Create HRI Scenario with Virtual Human

**Objective**: Visualize human-robot interaction in Unity.

**Steps**:
1. Import virtual human model
2. Position human near robot (1-2 meters away)
3. Add environment props (table, chair)
4. Animate robot approaching human
5. Capture video or screenshot

**Validation Criteria**:
- ✅ Human and robot positioned realistically
- ✅ Scene includes environment context
- ✅ Interaction is visually clear

---

## Key Takeaways

1. **Unity Robotics Hub** integrates Unity with ROS 2 via TCP Endpoint
2. **URDF Importer** converts ROS URDF to Unity GameObjects with axis transformation
3. **PBR materials** require albedo, metallic, and smoothness for photorealism
4. **HDRI lighting** provides realistic reflections and ambient lighting
5. **Real-time synchronization** uses ROS 2 joint states to update Unity visualization
6. **HRI scenarios** add virtual humans for interaction visualization

---

## Next Steps

**Next Chapter**: [Chapter 4: Sensor Simulation & Validation](./chapter-04-sensor-simulation-validation.md)

Learn how to simulate sensors (LiDAR, depth cameras, IMU) in Gazebo, visualize data in RViz, and validate consistency across Gazebo and Unity.
