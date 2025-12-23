---
sidebar_position: 2
title: 'Chapter 2: Synthetic Data Generation'
description: 'Create photorealistic training datasets with Isaac Sim Replicator API'
---

# Chapter 2: Synthetic Data & Perception Training with Isaac Sim

## Overview

In this chapter, you'll learn to generate photorealistic synthetic datasets using NVIDIA Isaac Sim. You'll create custom environments, configure virtual sensors, and use the Replicator API to produce thousands of labeled images for training perception models.

**Learning Objectives:**
- Build custom Isaac Sim environments with obstacles and lighting variations
- Configure RGB-D cameras and capture synchronized sensor data
- Generate 500+ labeled images using domain randomization
- Export datasets in COCO format for perception model training

**Time Estimate:** 2-3 hours

---

## 2.1 Why Synthetic Data?

### The Real-World Data Problem

Training a robust object detection model typically requires:
- 10,000+ labeled images per object class
- Diverse lighting conditions (indoor, outdoor, dawn, dusk)
- Various backgrounds and occlusions
- Multiple camera angles and distances

**Collecting real-world data is expensive:**
- 📸 Capture thousands of photos manually
- 🏷️ Label each object (bounding boxes, segmentation masks)
- 🔄 Repeat for different environments and conditions
- ⏱️ Time investment: weeks to months

**Cost estimate:** $50,000+ for a comprehensive dataset (labor + equipment).

### The Synthetic Data Solution

Isaac Sim can generate the same dataset in **hours instead of months**:

- ✅ **Free labels**: Ground truth comes directly from simulation
- ✅ **Perfect diversity**: Randomize lighting, poses, textures automatically
- ✅ **Infinite data**: Generate as many samples as needed
- ✅ **No privacy concerns**: No real people or proprietary environments

**Cost estimate:** $0 (after Isaac Sim setup).

### Real-World Success Stories

Companies using synthetic data:
- **Waymo**: Generates billions of miles of self-driving simulations
- **Amazon Robotics**: Trains warehouse robots in virtual fulfillment centers
- **Boston Dynamics**: Tests Spot and Atlas in virtual obstacle courses

**Research shows:** Models trained on 70% synthetic + 30% real data often **match or exceed** models trained on 100% real data.

---

## 2.2 Isaac Sim GUI Workflow

Before we automate dataset generation with code, let's understand the GUI workflow.

### Step 1: Launch Isaac Sim

```bash
# Start Omniverse Launcher
./omniverse-launcher

# Launch Isaac Sim from the launcher
# Select: Isaac Sim 2023.1.1
```

**First launch takes ~5 minutes** (loads assets and shaders).

### Step 2: Create a New Scene

1. **File → New** (or Ctrl+N)
2. **Create → Physics → Ground Plane**
3. **Create → Light → Dome Light**
   - Set Intensity: 1000
   - Load HDR map: `CloudyVondelpark.hdr` (built-in asset)

You now have a basic scene with lighting and a floor.

### Step 3: Add a Humanoid Robot

1. **Content Browser** (bottom panel)
2. Navigate to: `Isaac/Robots/Humanoid/`
3. Drag `Humanoid.usd` into the viewport
4. **Transform Tool** (W): Position robot at (0, 0, 1) so it's above ground

### Step 4: Add Objects for Perception Training

Let's add some obstacles the robot needs to detect:

1. **Create → Shapes → Cube** (add 3-4 cubes)
2. **Create → Shapes → Cylinder** (add 2-3 cylinders)
3. **Create → Shapes → Sphere** (add 2-3 spheres)

Randomly position them around the robot using the Transform Tool.

### Step 5: Configure Camera Sensor

1. Select the humanoid robot in the **Stage** panel (left side)
2. Navigate to: `Humanoid → head → CameraRig → Camera`
3. **Add → Isaac Sensors → Camera**
4. Set properties:
   - Resolution: 1280 x 720
   - Focal Length: 24mm
   - Depth: Enabled

### Step 6: Visualize Camera View

1. **Window → Isaac Sim → Viewport Selector**
2. Add new viewport
3. Set viewport camera to: `Humanoid/head/CameraRig/Camera`

You should now see the robot's POV.

### Step 7: Manual Data Capture (10 samples)

1. **Window → Isaac Sim → Replicator → Writer**
2. Select **BasicWriter**
3. Output directory: `/workspace/datasets/manual_capture`
4. Enable: RGB, Depth, Semantic Segmentation
5. **Randomize object poses manually** (drag objects to new positions)
6. Click **Write Data**
7. Repeat 10 times with different object arrangements

**Result:** 10 images with labels in `/workspace/datasets/manual_capture/`.

---

## 2.3 Domain Randomization Concepts

To train robust models, we need **diversity**. Domain randomization systematically varies environment parameters.

### What to Randomize?

| Parameter | Range | Impact on Model |
|-----------|-------|-----------------|
| **Lighting Intensity** | 500-2000 lumens | Handles indoor ↔ outdoor transitions |
| **Light Color** | 3000K-6500K | Robust to warm/cool lighting |
| **Object Positions** | Random within bounds | Learns spatial invariance |
| **Object Rotations** | 0-360° yaw | Recognizes objects from any angle |
| **Textures/Materials** | Wood, metal, plastic | Avoids overfitting to specific appearance |
| **Camera Angles** | ±30° pitch/yaw | Works from different viewpoints |
| **Backgrounds** | Plain, cluttered, textured | Ignores irrelevant features |

### Balancing Realism vs. Randomness

**Too little randomness:**
- Model overfits to specific environment
- Fails when lighting changes or objects move

**Too much randomness:**
- Unrealistic scenarios (purple lighting, floating objects)
- Model learns nonsense patterns

**Sweet spot:** Randomize within physically plausible ranges.

### Example: Lighting Randomization

```python
# Physically plausible
light_intensity = random.uniform(500, 2000)  # ✅ Indoor to outdoor
light_color = random.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0))  # ✅ Warm to neutral

# Unrealistic (don't do this)
light_intensity = random.uniform(0, 10000)  # ❌ Includes pitch black and blinding
light_color = (random.random(), random.random(), random.random())  # ❌ Purple, green, etc.
```

---

## 2.4 Replicator API: Automating Dataset Generation

Now let's automate what we just did manually. The Replicator API is a Python library for procedural dataset generation.

### Basic Workflow

```python
import omni.replicator.core as rep

# 1. Define what to randomize
# 2. Set trigger (how many frames)
# 3. Configure writer (output format)
# 4. Run orchestrator
```

### Example: Generate 500 Images with Randomization

Create a file: `generate_dataset.py`

```python
import omni.replicator.core as rep
import numpy as np

# Set random seed for reproducibility
rep.settings.set_global_seed(42)

# Load the scene we created
rep.create.from_usd("/workspace/scenes/humanoid_environment.usd")

# Get camera reference
camera = rep.create.camera(position=(2, 2, 2), look_at=(0, 0, 1))

# Get all objects we want to randomize
objects = rep.get.prims(path_pattern="/World/Objects/*")

# Define randomization for 500 frames
with rep.trigger.on_frame(num_frames=500):

    # Randomize lighting
    with rep.create.light(
        light_type="Dome",
        intensity=rep.distribution.uniform(500, 2000),
        color=rep.distribution.uniform((0.8, 0.8, 0.8), (1, 1, 1))
    ):
        pass

    # Randomize object positions
    with rep.modify.pose(
        objects,
        position=rep.distribution.uniform((-2, -2, 0.5), (2, 2, 2)),
        rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))
    ):
        pass

    # Randomize object materials
    with rep.randomizer.materials(
        objects,
        materials=rep.get.prims(path_pattern="/World/Materials/*")
    ):
        pass

# Configure output writer (COCO format)
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/workspace/datasets/synthetic_humanoid_nav",
    rgb=True,
    semantic_segmentation=True,
    distance_to_camera=True,
    bounding_box_2d_tight=True,
    bounding_box_2d_loose=True
)

# Attach writer to render products
render_product = rep.create.render_product(camera, (1280, 720))
writer.attach([render_product])

# Run the data generation
rep.orchestrator.run()

print("✅ Generated 500 images with labels!")
```

### Running the Script

```bash
# From Isaac Sim Script Editor
# Or from terminal:
./python.sh generate_dataset.py
```

**Time:** ~10-15 minutes for 500 images (depends on GPU).

---

## 2.5 Understanding the Output

After running the script, check the output directory:

```
/workspace/datasets/synthetic_humanoid_nav/
├── rgb/
│   ├── rgb_0000.png
│   ├── rgb_0001.png
│   └── ... (500 images)
├── semantic_segmentation/
│   ├── semantic_segmentation_0000.png
│   └── ... (500 masks)
├── distance_to_camera/
│   ├── distance_to_camera_0000.npy
│   └── ... (500 depth maps)
├── bounding_box_2d_tight/
│   └── bounding_box_2d_tight_0000.npy
└── metadata.json
```

### RGB Images (rgb/)

Regular camera view, like a photo. Use for training object detection/segmentation models.

### Semantic Segmentation (semantic_segmentation/)

Pixel-wise labels where each color represents a class:
- Red = Cube
- Green = Cylinder
- Blue = Sphere
- Gray = Ground
- Black = Background

### Distance to Camera (depth maps)

Numpy arrays with distance (in meters) from camera to each pixel. Use for depth estimation training.

### Bounding Boxes (2D tight/loose)

Numpy arrays with [x_min, y_min, x_max, y_max, class_id] for each object. Use for YOLO/Faster R-CNN training.

### Converting to COCO Format

The COCO format is the standard for object detection. Use this script:

```python
import json
import numpy as np
from pathlib import Path

def convert_to_coco(data_dir):
    coco_output = {
        "images": [],
        "annotations": [],
        "categories": [
            {"id": 1, "name": "cube"},
            {"id": 2, "name": "cylinder"},
            {"id": 3, "name": "sphere"}
        ]
    }

    annotation_id = 0

    for img_id, img_path in enumerate(sorted(Path(data_dir, "rgb").glob("*.png"))):
        # Add image entry
        coco_output["images"].append({
            "id": img_id,
            "file_name": img_path.name,
            "width": 1280,
            "height": 720
        })

        # Load bounding boxes
        bbox_file = Path(data_dir, "bounding_box_2d_tight", f"bounding_box_2d_tight_{img_id:04d}.npy")
        bboxes = np.load(bbox_file)

        for bbox in bboxes:
            x_min, y_min, x_max, y_max, class_id = bbox
            width = x_max - x_min
            height = y_max - y_min

            coco_output["annotations"].append({
                "id": annotation_id,
                "image_id": img_id,
                "category_id": int(class_id),
                "bbox": [float(x_min), float(y_min), float(width), float(height)],
                "area": float(width * height),
                "iscrowd": 0
            })
            annotation_id += 1

    # Save COCO JSON
    with open(Path(data_dir, "annotations.json"), "w") as f:
        json.dump(coco_output, f, indent=2)

    print(f"✅ Converted to COCO format: {annotation_id} annotations")

# Run conversion
convert_to_coco("/workspace/datasets/synthetic_humanoid_nav")
```

---

## 2.6 Advanced Techniques

### Multi-Camera Capture

Generate data from multiple viewpoints simultaneously:

```python
# Create 4 cameras in a circle around the robot
cameras = []
for angle in [0, 90, 180, 270]:
    rad = np.radians(angle)
    x = 3 * np.cos(rad)
    y = 3 * np.sin(rad)
    camera = rep.create.camera(position=(x, y, 1.5), look_at=(0, 0, 1))
    cameras.append(camera)

# Each camera generates its own image per frame
# Result: 4x more data (2000 images from 500 frames)
```

### Time-of-Day Randomization

Simulate different times of day:

```python
# Morning: Cool light, low intensity
# Noon: Neutral light, high intensity
# Evening: Warm light, medium intensity

time_of_day = rep.distribution.choice(["morning", "noon", "evening"])

with time_of_day == "morning":
    intensity = rep.distribution.uniform(600, 1000)
    color = (0.8, 0.9, 1.0)  # Cool blue tint

with time_of_day == "noon":
    intensity = rep.distribution.uniform(1500, 2000)
    color = (1.0, 1.0, 1.0)  # Neutral

with time_of_day == "evening":
    intensity = rep.distribution.uniform(800, 1200)
    color = (1.0, 0.9, 0.8)  # Warm orange tint
```

### Texture Randomization

Apply different materials to objects:

```python
# Create material library
materials = [
    "/World/Materials/Wood",
    "/World/Materials/Metal",
    "/World/Materials/Plastic",
    "/World/Materials/Concrete"
]

with rep.trigger.on_frame():
    with rep.randomizer.materials(
        objects=rep.get.prims(path_pattern="/World/Objects/*"),
        materials=rep.get.prims(path_pattern=materials)
    ):
        pass
```

---

## 2.7 Validating Dataset Quality

Before training a model, validate your synthetic data:

### Visual Inspection

```python
import matplotlib.pyplot as plt
import cv2

# Display 9 random samples
fig, axes = plt.subplots(3, 3, figsize=(15, 15))

for i, ax in enumerate(axes.flat):
    img_path = f"/workspace/datasets/synthetic_humanoid_nav/rgb/rgb_{i*50:04d}.png"
    img = cv2.imread(img_path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    ax.imshow(img)
    ax.axis('off')
    ax.set_title(f"Sample {i*50}")

plt.tight_layout()
plt.show()
```

**Check for:**
- ✅ Sufficient lighting variation (not all bright or all dark)
- ✅ Objects in diverse poses (not all upright)
- ✅ Varied backgrounds
- ✅ No unrealistic scenarios (floating objects, pitch black scenes)

### Label Accuracy Check

```python
# Overlay bounding boxes on RGB image
import cv2
import numpy as np

img = cv2.imread("/workspace/datasets/synthetic_humanoid_nav/rgb/rgb_0000.png")
bboxes = np.load("/workspace/datasets/synthetic_humanoid_nav/bounding_box_2d_tight/bounding_box_2d_tight_0000.npy")

for bbox in bboxes:
    x_min, y_min, x_max, y_max, class_id = bbox
    color = [(255, 0, 0), (0, 255, 0), (0, 0, 255)][int(class_id) - 1]  # RGB for each class
    cv2.rectangle(img, (int(x_min), int(y_min)), (int(x_max), int(y_max)), color, 2)

cv2.imshow("Labels", img)
cv2.waitKey(0)
```

**Verify:**
- ✅ Bounding boxes tightly fit objects
- ✅ No missing objects
- ✅ Correct class IDs

### Distribution Analysis

```python
import json

with open("/workspace/datasets/synthetic_humanoid_nav/annotations.json") as f:
    coco = json.load(f)

# Count annotations per class
class_counts = {1: 0, 2: 0, 3: 0}
for ann in coco["annotations"]:
    class_counts[ann["category_id"]] += 1

print("Class Distribution:")
print(f"  Cubes: {class_counts[1]}")
print(f"  Cylinders: {class_counts[2]}")
print(f"  Spheres: {class_counts[3]}")

# Aim for balanced distribution (within 20% of each other)
```

---

## 2.8 Hands-On Exercise

### Exercise 1: Generate Your First Dataset

**Goal:** Create 100 labeled images with randomized lighting and object poses.

**Steps:**
1. Launch Isaac Sim and load the humanoid scene
2. Create `my_dataset.py` using the Replicator API template
3. Set `num_frames=100`
4. Randomize lighting intensity (500-2000) and object positions
5. Run the script and verify output

**Success Criteria:**
- ✅ 100 RGB images in `/rgb/` folder
- ✅ 100 segmentation masks in `/semantic_segmentation/`
- ✅ Visual inspection shows lighting variation

**Time:** 30 minutes

---

### Exercise 2: Domain Randomization Challenge

**Goal:** Improve model robustness by adding texture randomization.

**Steps:**
1. Create 3 materials (wood, metal, plastic) in Isaac Sim
2. Modify your `my_dataset.py` script to randomize materials
3. Generate 100 images
4. Compare with Exercise 1 output

**Success Criteria:**
- ✅ Objects have different textures across images
- ✅ Same object class (cube) appears with multiple materials

**Time:** 45 minutes

---

### Exercise 3: Convert to COCO Format

**Goal:** Prepare data for PyTorch/TensorFlow training.

**Steps:**
1. Use the `convert_to_coco()` function provided
2. Run on your Exercise 1 dataset
3. Validate `annotations.json` structure

**Success Criteria:**
- ✅ `annotations.json` file created
- ✅ Contains "images", "annotations", "categories" keys
- ✅ Annotation count matches bounding boxes in numpy files

**Time:** 20 minutes

---

## 2.9 Key Takeaways

By the end of this chapter, you should be able to:

✅ Explain why synthetic data is cost-effective (hours vs. months, free labels)
✅ Navigate Isaac Sim GUI to create scenes, add objects, configure cameras
✅ Use Replicator API to generate 500+ images with domain randomization
✅ Randomize lighting, poses, textures, and cameras for robust datasets
✅ Export data in COCO format for object detection model training
✅ Validate dataset quality (visual inspection, label accuracy, class distribution)

---

## What's Next?

Now that you can generate synthetic training data, let's put it to use! In **Chapter 3**, you'll:

- Install Isaac ROS and configure Docker
- Deploy cuVSLAM for real-time localization (30+ Hz)
- Integrate RGB-D camera and IMU sensors
- Evaluate VSLAM accuracy with ground truth from Isaac Sim

Ready to enable your robot's spatial awareness? Let's go! 🚀

---

## Additional Resources

- 📘 [Isaac Sim Replicator Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/)
- 📘 [COCO Dataset Format Specification](https://cocodataset.org/#format-data)
- 📝 [Synthetic Data for Robotics (NVIDIA Blog)](https://developer.nvidia.com/blog/getting-started-with-nvidia-isaac-sim-replicator/)
- 🎥 [Replicator API Tutorial (YouTube)](https://www.youtube.com/nvidia)

---

:::tip Configuration Tip
Save your randomization parameters (lighting ranges, material lists, etc.) in a YAML config file. This makes experiments reproducible and easy to tweak.
:::
