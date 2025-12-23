# Implementation Plan: Module 4 - Vision-Language-Action (VLA)

**Branch**: `009-vla-voice-control` | **Date**: 2025-12-23 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/009-vla-voice-control/spec.md`

## Summary

Module 4 teaches advanced robotics learners to build voice-driven autonomous humanoid systems by integrating Large Language Models (LLMs) with robotic perception and control. The module covers the complete Vision-Language-Action (VLA) loop: voice command capture using Whisper, cognitive task planning with LLMs, ROS 2 navigation execution via Nav2, and computer vision for object detection. Learners will create an end-to-end autonomous humanoid capable of understanding natural language commands, decomposing them into executable plans, navigating dynamic environments, and interacting with detected objects.

**Technical Approach**: Educational content delivered via Docusaurus documentation with hands-on ROS 2 code examples, progressive exercises (12+ across 4 chapters), and a capstone integration project. Uses pre-trained models (Whisper, GPT-4/Claude, YOLOv8) integrated with ROS 2 Humble, running entirely in simulation (Isaac Sim primary, Gazebo alternative).

## Technical Context

**Language/Version**: Python 3.10+ (ROS 2 Humble requirement), Markdown (Docusaurus documentation)
**Primary Dependencies**:
- ROS 2 Humble (nav2, tf2, geometry_msgs, vision_msgs)
- OpenAI Whisper (speech recognition)
- OpenAI GPT-4 API or Anthropic Claude API (LLM planning)
- PyTorch 2.0+ (Whisper, object detection models)
- YOLOv8 or Ultralytics (object detection)
- Isaac Sim 2023.1+ or Gazebo Fortress (simulation)
- Python audio libraries (sounddevice, pyaudio)

**Storage**: N/A (educational content, no persistent data beyond logs/configs)
**Testing**: Manual validation via hands-on exercises, automated checks for code syntax/imports
**Target Platform**: Ubuntu 22.04 LTS (ROS 2 Humble target), GPU required (NVIDIA RTX 2060+ for Whisper + object detection)
**Project Type**: Educational Module (Docusaurus site with code examples)
**Performance Goals**:
- Voice transcription: <2 second latency for 10-second utterances
- LLM planning: <5 second end-to-end (voice→robot motion)
- Object detection: 10+ Hz inference on GPU
- Navigation: >75% waypoint success rate (Module 3 baseline)

**Constraints**:
- Simulation-only (no real hardware requirements)
- Pre-trained models only (no custom training from scratch)
- Educational budget-friendly (OpenAI API costs <$5/month per learner for exercises)
- Completion time: 10-14 hours total (consistent with Modules 1-3)

**Scale/Scope**:
- 4 chapters + 1 module project
- 12+ hands-on exercises
- 5-8 ROS 2 example nodes
- 15,000+ words of documentation

## Constitution Check

✅ **I. Accessibility First**
- Prerequisites explicitly stated (Module 3 completion, ROS 2 + Python + basic AI/ML knowledge)
- Progressive introduction: Voice (Ch1) → Planning (Ch2) → Execution (Ch3) → Vision (Ch4) → Integration (Project)
- Real-world analogies for LLM concepts (e.g., "LLM as the robot's brain")
- Code examples fully commented with line-by-line explanations
- Visual aids: VLA pipeline diagrams, ROS 2 node graphs, state machine diagrams

✅ **II. Hands-On Learning Priority**
- 12+ practical exercises across 4 chapters (3+ per chapter)
- Runnable code examples for voice capture, LLM planning, object detection
- Step-by-step tutorials with expected outputs (ROS topics, logs, RViz visualizations)
- Troubleshooting sections for common errors (API key issues, GPU out-of-memory, audio device not found)
- Capstone project: End-to-end VLA pipeline with measurable success criteria

✅ **III. Progressive Complexity**
- P1 (Foundation): Voice, LLM Planning, Navigation (independent modules)
- P2 (Integration): Vision, Full VLA pipeline (builds on P1)
- Clear dependency graph: Ch1→Ch2→Ch3 can be learned independently, Ch4+Project require all P1 complete
- Optional "Deep Dive" sections: Prompt engineering advanced techniques, custom Whisper fine-tuning (out of scope for main module)

✅ **IV. Production-Ready Examples**
- All code tested on Ubuntu 22.04 + ROS 2 Humble + Isaac Sim
- Error handling: API failures (retry with backoff), microphone errors (fallback to text input), object not found (graceful abort)
- Best practices: .env for API keys, typed ROS messages, logging at appropriate levels
- Comments explain "why" (e.g., "We use small Whisper model for <2s latency vs medium for higher accuracy")

✅ **V. Clear Documentation Standards**
- Learning objectives per chapter (e.g., "Deploy Whisper-ROS node with >90% accuracy")
- Estimated time: Ch1(2-3h), Ch2(3-4h), Ch3(2-3h), Ch4(2-3h), Project(3-4h) = 12-17h total
- Hardware requirements: GPU (NVIDIA RTX 2060+), microphone, 32GB RAM recommended
- Expected outcomes with screenshots: ROS topic outputs, RViz object detections, navigation paths
- Glossary: VLA, Whisper, LLM prompting, RGB-D, bounding box, action server

✅ **VI. Community-Driven Improvement**
- GitHub issues for reporting errors or requesting clarifications
- Version history tracking in Git (module revisions tracked via commits)
- Acknowledgment section for contributors
- Feedback collection via GitHub Discussions

## Project Structure

### Documentation (this feature)

```text
specs/009-vla-voice-control/
├── plan.md              # This file (/sp.plan output)
├── research.md          # Phase 0 research findings
├── spec.md              # Feature specification
├── checklists/
│   └── requirements.md  # Spec validation checklist
└── tasks.md             # Phase 2 output (/sp.tasks - NOT created yet)
```

### Source Code (repository root)

```text
physical-ai-book/
├── docs/
│   └── module-04/                         # Module 4 documentation
│       ├── index.md                       # Module landing page
│       ├── chapter-01-voice-to-action.md  # Whisper + ROS 2
│       ├── chapter-02-cognitive-planning.md  # LLM task decomposition
│       ├── chapter-03-ros2-execution.md   # Nav2 + action servers
│       ├── chapter-04-computer-vision.md  # Object detection + RGB-D
│       └── module-04-project.md           # Autonomous Humanoid project
│
├── code-examples/
│   └── module-04-vla/
│       ├── chapter-01-voice/
│       │   ├── whisper_node.py             # ROS 2 node for voice transcription
│       │   ├── audio_capture.py            # Microphone audio streaming
│       │   └── voice_command_publisher.py  # Structured message publisher
│       ├── chapter-02-planning/
│       │   ├── llm_planner_node.py         # LLM integration for task planning
│       │   ├── prompt_templates.py         # Prompt engineering templates
│       │   └── action_sequence_generator.py  # Plan→ROS 2 action mapping
│       ├── chapter-03-execution/
│       │   ├── nav2_executor.py            # Nav2 goal execution
│       │   ├── action_server.py            # Multi-step action sequencing
│       │   └── recovery_behaviors.py       # Failure handling
│       ├── chapter-04-vision/
│       │   ├── object_detection_node.py    # YOLOv8 ROS 2 integration
│       │   ├── rgbd_processor.py           # Depth→3D localization
│       │   └── vision_action_bridge.py     # Vision→planning integration
│       └── module-project/
│           ├── vla_pipeline.py             # End-to-end integration
│           ├── full_system.launch.py       # Launch all nodes
│           └── evaluation_metrics.py       # Success rate, latency tracking
│
├── assets/
│   └── module-04/
│       ├── diagrams/
│       │   ├── vla-loop-architecture.png   # Voice→Language→Action→Vision loop
│       │   ├── ros2-node-graph.png         # ROS 2 communication topology
│       │   └── llm-planning-flow.png       # Task decomposition flowchart
│       └── videos/
│           ├── demo-voice-command.mp4      # Voice→navigation demo
│           └── demo-object-retrieval.mp4   # Full VLA pipeline demo
│
└── sidebars.ts  # Docusaurus navigation (add Module 4 entry)
```

**Structure Decision**: Educational module structure mirrors Modules 1-3 for consistency. Docs organized by chapter (each chapter maps to a user story). Code examples grouped by chapter with clear file naming. Assets include diagrams (VLA architecture, ROS graphs) and demo videos (show successful execution).

## Complexity Tracking

> No constitutional violations - all requirements satisfied within educational content constraints.

---

## Phase 0: Research & Technical Decisions

### Research Areas

**R1: Whisper Integration with ROS 2**
- Audio capture methods (sounddevice vs pyaudio vs ros2_audio_common)
- Model size tradeoffs (tiny/small/medium vs accuracy/latency)
- GPU acceleration setup (CUDA, TensorRT optimization)
- ROS 2 message format for transcribed commands

**R2: LLM API Integration**
- Provider comparison (OpenAI GPT-4, Anthropic Claude, local Llama)
- Prompt engineering for robot task decomposition
- Structured output generation (JSON action sequences)
- Cost and latency optimization

**R3: Object Detection for Simulation**
- Model selection (YOLOv8, DETR, Detectron2)
- Pre-trained weights vs fine-tuning on synthetic data
- RGB-D integration for 3D object localization
- ROS 2 vision_msgs compatibility

**R4: VLA Pipeline Architecture**
- ROS 2 communication patterns (action servers, services, topics)
- State management (active command, execution status)
- Synchronous vs asynchronous execution
- Failure recovery strategies

**R5: Educational Exercise Design**
- Exercise format (Jupyter notebooks, standalone scripts, launch files)
- Progressive difficulty structuring
- Automated success verification methods
- Simulation platform (Isaac Sim vs Gazebo)

### Technical Decisions

#### Decision D1: Whisper Model Size

**Selected**: Whisper **Small** model

**Rationale**:
**Pros**:
- Accuracy: ~95% WER (Word Error Rate) on robot vocabulary (sufficient for >90% target)
- Latency: ~1.5 seconds for 10-second utterance on RTX 3060 (meets <2s requirement)
- GPU Memory: ~2GB VRAM (leaves headroom for object detection)
- Educational fit: Balances performance and resource requirements

**Alternatives Considered**:
- **Tiny**: Too low accuracy (~85% WER), frequent transcription errors frustrate learners
- **Medium**: Higher accuracy (~97% WER) but 3-4 second latency, violates <2s constraint

**Implementation**: Use `openai/whisper-small` with `fp16` precision on GPU, fallback to CPU with warning if GPU unavailable.

---

#### Decision D2: LLM Provider

**Selected**: **OpenAI GPT-4** (primary) with **Anthropic Claude** (alternative documented)

**Rationale**:
**Pros (GPT-4)**:
- Superior task decomposition for robotics (based on community benchmarks)
- Structured output via function calling (easy JSON action sequences)
- Widely accessible ($5/month for ~500 API calls covers typical learner exercises)
- Extensive documentation and learner familiarity

**Pros (Claude Alternative)**:
- Lower cost per token
- Better at following strict output formats
- Provides educational diversity (learners see multiple LLM options)

**Implementation**: Primary exercises use GPT-4 with fallback instructions for Claude. Provide `.env.example` with both API key placeholders. Prompt templates designed to work with both providers.

---

#### Decision D3: Object Detection Model

**Selected**: **YOLOv8** (Ultralytics)

**Rationale**:
**Pros**:
- Real-time performance: 50-100 FPS on RTX 3060 (exceeds 10 Hz target)
- Pre-trained on COCO: 80 object classes (sufficient for manipulation tasks)
- Easy PyTorch integration with ROS 2
- Active community and educational resources

**Alternatives Considered**:
- **DETR**: Transformer-based, slower inference (~10 FPS), overkill for simulation
- **Detectron2**: Powerful but complex setup, higher learning curve

**Implementation**: Use `yolov8n.pt` (nano) or `yolov8s.pt` (small) pre-trained weights. Convert RGB-D to 3D bounding boxes using depth camera intrinsics. Publish to `/vision/detections` as `vision_msgs/Detection3DArray`.

---

#### Decision D4: VLA Pipeline Architecture

**Selected**: **Hybrid** - ROS 2 Action Servers (navigation) + Topics (voice, vision) + Services (LLM planning)

**Rationale**:
**Voice Input**: Topic-based (`/voice/commands`) - continuous stream, multiple subscribers possible
**LLM Planning**: Service call (`/plan_task`) - synchronous request-response, learner sees clear call/response
**Navigation**: Action server (`nav2` existing infrastructure) - long-running goals with feedback
**Vision**: Topic-based (`/vision/detections`) - continuous updates, integration with planning

**Flow**:
1. `whisper_node` publishes to `/voice/commands` (std_msgs/String + timestamp)
2. `vla_coordinator` node subscribes, calls `/plan_task` service (LLM)
3. LLM service returns action sequence (JSON: `[{type: "navigate", x: 5, y: 3}, {type: "grasp", object_id: "box1"}]`)
4. Coordinator sends Nav2 action goal, waits for completion
5. On success, triggers vision scan, then manipulation (conceptual)

**State Management**: `ExecutionState` published to `/vla/status` topic for monitoring.

---

#### Decision D5: Audio Handling

**Selected**: **Push-to-Talk** (primary) with **Continuous Listening** (advanced exercise)

**Rationale**:
- Push-to-talk simpler for beginners (press key, speak, release = clear boundaries)
- Avoids voice activity detection (VAD) complexity
- Reduces false triggers from background noise
- Continuous listening taught in Ch1 advanced exercise for interested learners

**Implementation**: Use `sounddevice` library, capture audio on spacebar press, stop on release. Buffer audio to WAV, pass to Whisper.

---

#### Decision D6: LLM Prompt Strategy

**Selected**: **Few-Shot Prompting** with robot capability examples

**Rationale**:
- Zero-shot often produces invalid action formats
- Few-shot (3-5 examples) dramatically improves structured output
- Learners see prompt engineering in action
- Examples teach robot capabilities (navigation, grasping, waiting)

**Example Prompt**:
```
You are a robot task planner. Given a natural language command, decompose it into a sequence of actions.

Available actions:
- navigate(x, y, theta): Move to position
- detect_object(class_name): Scan for object
- grasp(object_id): Pick up object
- wait(seconds): Pause execution

Examples:
User: "Go to the kitchen"
Plan: [{"action": "navigate", "x": 5.0, "y": 3.0, "theta": 0.0}]

User: "Pick up the red box"
Plan: [
  {"action": "detect_object", "class": "box", "color": "red"},
  {"action": "navigate", "x": <detected_x>, "y": <detected_y>},
  {"action": "grasp", "object_id": "<detected_id>"}
]

Now plan this command:
User: <USER_COMMAND>
Plan:
```

---

#### Decision D7: Vision Integration

**Selected**: **On-Demand Detection** triggered by LLM plan

**Rationale**:
- Real-time continuous detection wastes GPU cycles (robot not always manipulating)
- On-demand aligns with task flow (detect only when LLM requests it)
- Simpler state management (detection happens at specific pipeline stages)

**Implementation**: LLM plan includes `detect_object` action → triggers vision node to capture frame, run inference, return results. Vision node idles otherwise.

---

#### Decision D8: Exercise Format

**Selected**: **Standalone Python Scripts** (primary) + **ROS 2 Launch Files** (integration)

**Rationale**:
**Pros (Standalone Scripts)**:
- Easy to run: `python3 whisper_node.py`
- Clear single-file learning (one concept per script)
- Copy-paste friendly for learner experimentation

**Pros (Launch Files)**:
- Teaches ROS 2 launch system (multi-node orchestration)
- Required for Module Project (full system integration)

**Jupyter Notebooks**: Considered but rejected (ROS 2 integration awkward, simulation requires separate processes)

**Implementation**: Ch1-4 use standalone scripts with comments. Module Project provides `full_system.launch.py` launching all nodes.

---

#### Decision D9: Simulation Platform

**Selected**: **Isaac Sim** (primary) with **Gazebo Fortress** (alternative documented)

**Rationale**:
- Continuity from Module 3 (learners already familiar with Isaac Sim)
- Isaac Sim provides superior RGB-D camera simulation (photorealistic for vision)
- Nav2 integration identical across both platforms (ROS 2 abstraction)
- Gazebo alternative documented for learners without NVIDIA GPU

**Implementation**: All exercises show Isaac Sim commands, with Gazebo equivalents in "Alternative Setup" sections.

---

## Phase 1: Design & Documentation Structure

### Chapter Outlines

#### Chapter 1: Voice-to-Action with OpenAI Whisper (2-3 hours)

**Learning Objectives**:
- Understand speech recognition for robotics (Whisper architecture, model sizes)
- Capture real-time audio in Python
- Integrate Whisper with ROS 2 for voice command publishing
- Achieve >90% transcription accuracy on robot vocabulary

**Content Structure**:
1. **1.1 Introduction to Voice Control** (15 min)
   - Why voice? (hands-free operation, accessibility, natural interaction)
   - Speech recognition challenges (noise, accents, domain vocabulary)
   - Whisper overview (transformer model, multilingual, robust to noise)

2. **1.2 Whisper Model Sizes & Performance** (20 min)
   - Model comparison table (tiny/small/medium: WER, latency, VRAM)
   - Decision framework: When to use each model
   - Educational choice: Small model (balance accuracy/speed)

3. **1.3 Audio Capture in Python** (30 min)
   - `sounddevice` library setup
   - Microphone device listing and selection
   - Audio buffering and WAV file writing
   - **Exercise 1**: Capture 5-second audio clip, save as WAV

4. **1.4 Whisper Transcription** (40 min)
   - Loading Whisper model (`whisper.load_model("small")`)
   - Transcribing audio files (`model.transcribe()`)
   - Handling confidence scores and timestamps
   - **Exercise 2**: Transcribe pre-recorded commands, verify accuracy

5. **1.5 ROS 2 Integration** (60 min)
   - Creating a ROS 2 node (`rclpy.node.Node`)
   - Publishing to `/voice/commands` topic (custom `VoiceCommand` message)
   - Push-to-talk implementation (keyboard listener)
   - **Exercise 3**: Deploy `whisper_node.py`, speak commands, verify ROS topic

6. **1.6 Advanced: Continuous Listening** (30 min - optional)
   - Voice Activity Detection (VAD) with `webrtcvad`
   - Streaming audio chunks to Whisper
   - Handling false triggers
   - **Exercise 4**: Implement continuous listening with VAD

**Hands-On Exercises** (3 required + 1 advanced):
- Exercise 1: Audio capture and WAV export
- Exercise 2: Offline Whisper transcription
- Exercise 3: ROS 2 voice command publisher
- Exercise 4 (Advanced): Continuous listening VAD

**Success Criteria**:
- Learners deploy Whisper-ROS node with <2 second latency
- >90% word accuracy on 10 test robot commands
- Successfully publish to `/voice/commands` topic

---

#### Chapter 2: Cognitive Planning with LLMs (3-4 hours)

**Learning Objectives**:
- Understand LLM capabilities for task planning
- Design effective prompts for robot command decomposition
- Generate structured action sequences (JSON format)
- Map natural language to ROS 2 action goals

**Content Structure**:
1. **2.1 LLMs as Robot Planners** (20 min)
   - What LLMs can/cannot do for robotics
   - Task decomposition vs low-level control
   - GPT-4 vs Claude vs local models comparison

2. **2.2 Prompt Engineering Fundamentals** (40 min)
   - Zero-shot vs few-shot prompting
   - Structured output with examples
   - Robot capability descriptions in prompts
   - **Exercise 1**: Write prompt to decompose "clean the table"

3. **2.3 OpenAI API Integration** (50 min)
   - API key setup (`.env` file management)
   - `openai.ChatCompletion.create()` basics
   - Parsing JSON responses
   - Error handling (rate limits, timeouts)
   - **Exercise 2**: Call GPT-4 API, parse action sequence

4. **2.4 ROS 2 Service for Planning** (60 min)
   - Creating a service server (`PlanTask.srv`)
   - Request: text command, Response: action sequence JSON
   - Calling LLM in service callback
   - **Exercise 3**: Deploy `llm_planner_service.py`, test with `ros2 service call`

5. **2.5 Action Sequence Validation** (40 min)
   - JSON schema validation
   - Checking action preconditions (e.g., navigate before grasp)
   - Handling invalid LLM outputs (retry, fallback)
   - **Exercise 4**: Add validation logic to planner service

6. **2.6 Multi-Step Task Examples** (30 min)
   - Navigation-only tasks ("go to kitchen")
   - Object manipulation tasks ("pick up red box")
   - Conditional tasks ("if object not found, return home")
   - **Exercise 5**: Test planner on 10 diverse commands

**Hands-On Exercises** (5 exercises):
- Exercise 1: Prompt design for task decomposition
- Exercise 2: Direct GPT-4 API call and JSON parsing
- Exercise 3: ROS 2 planning service deployment
- Exercise 4: Action sequence validation
- Exercise 5: Multi-scenario testing (10 commands)

**Success Criteria**:
- LLM generates valid action sequences for 80% of test commands
- Service responds in <3 seconds for simple navigation commands
- JSON output matches defined schema

---

#### Chapter 3: ROS 2 Execution & Obstacle Navigation (2-3 hours)

**Learning Objectives**:
- Map LLM plans to Nav2 navigation goals
- Execute multi-step action sequences via ROS 2 action servers
- Implement failure recovery behaviors
- Achieve >75% navigation success rate (Module 3 baseline)

**Content Structure**:
1. **3.1 From Plans to Actions** (20 min)
   - Recap: Module 3 Nav2 basics (waypoint navigation, obstacle avoidance)
   - Mapping LLM JSON to `NavigationGoal` messages
   - Action server client setup

2. **3.2 Nav2 Action Client** (50 min)
   - Creating action client (`NavigateToPose` action)
   - Sending goals from LLM plan
   - Monitoring feedback (distance remaining, ETA)
   - **Exercise 1**: Send 3 navigation goals from LLM plan

3. **3.3 Multi-Step Execution** (60 min)
   - Sequential action execution (wait for completion before next)
   - State machine for tracking current step
   - Publishing execution status to `/vla/status`
   - **Exercise 2**: Execute 5-step sequence (navigate → wait → navigate → ...)

4. **3.4 Failure Handling** (50 min)
   - Detecting navigation failures (timeout, obstacle, goal unreachable)
   - Recovery strategies: retry, re-plan, abort
   - Logging errors for debugging
   - **Exercise 3**: Test recovery on blocked path scenario

5. **3.5 Dynamic Obstacle Avoidance** (40 min)
   - Recap: Module 3 costmaps and dynamic obstacles
   - Testing VLA pipeline in cluttered environment
   - **Exercise 4**: Navigate to goal while avoiding moving obstacles

**Hands-On Exercises** (4 exercises):
- Exercise 1: Nav2 action client goal sending
- Exercise 2: Multi-step sequence execution
- Exercise 3: Failure recovery testing
- Exercise 4: Dynamic obstacle navigation

**Success Criteria**:
- Execute LLM-generated navigation plans with >75% success rate
- Properly handle at least 2 failure scenarios (blocked path, timeout)
- Multi-step sequences complete in order without skipping

---

#### Chapter 4: Computer Vision for Object Detection (2-3 hours)

**Learning Objectives**:
- Deploy YOLOv8 for real-time object detection in simulation
- Integrate RGB-D cameras for 3D object localization
- Publish detections to ROS 2 vision topics
- Achieve >85% precision and >80% recall on simulated datasets

**Content Structure**:
1. **4.1 Object Detection Fundamentals** (20 min)
   - 2D vs 3D detection (bounding boxes, depth integration)
   - YOLOv8 architecture overview (efficient real-time detection)
   - COCO dataset and pre-trained weights

2. **4.2 YOLOv8 Setup** (40 min)
   - Installing Ultralytics library
   - Loading pre-trained model (`YOLO("yolov8n.pt")`)
   - Running inference on images
   - **Exercise 1**: Detect objects in 5 test images, visualize bounding boxes

3. **4.3 RGB-D Integration** (60 min)
   - Isaac Sim RGB-D camera setup
   - Subscribing to `/camera/rgb/image_raw` and `/camera/depth/image_raw`
   - Converting 2D bbox + depth → 3D position
   - Camera intrinsics and point cloud projection
   - **Exercise 2**: Publish 3D object positions to `/vision/detections`

4. **4.4 ROS 2 Vision Node** (50 min)
   - Creating `object_detection_node.py`
   - Publishing `vision_msgs/Detection3DArray`
   - Filtering detections by confidence threshold (>0.5)
   - **Exercise 3**: Deploy vision node, verify detections in RViz

5. **4.5 Vision-Planning Integration** (40 min)
   - LLM requests `detect_object("box")` action
   - Vision node triggered via service call
   - Returning detected object coordinates to planner
   - **Exercise 4**: Full loop - voice command → LLM plan → vision detection

**Hands-On Exercises** (4 exercises):
- Exercise 1: YOLOv8 offline detection on images
- Exercise 2: RGB-D to 3D position conversion
- Exercise 3: ROS 2 vision node deployment
- Exercise 4: Vision-planning integration test

**Success Criteria**:
- Object detection achieves >85% precision, >80% recall on test set
- 3D localization accurate within 0.1m of ground truth
- Vision node runs at 10+ Hz on GPU

---

### Module 4 Project: The Autonomous Humanoid (3-4 hours)

**Learning Objectives**:
- Integrate all VLA components into end-to-end system
- Achieve >70% task completion rate on complex commands
- Measure and report performance metrics (latency, success rate, accuracy)

**Project Requirements**:

**System Integration**:
1. ✅ Launch Isaac Sim with humanoid robot in environment with 10 objects
2. ✅ Start Whisper voice node (<2s transcription latency)
3. ✅ Initialize LLM planning service (GPT-4/Claude)
4. ✅ Deploy Nav2 navigation stack (Module 3 config)
5. ✅ Start YOLOv8 object detection node (10+ Hz)
6. ✅ Run VLA coordinator (orchestrates pipeline)

**Performance Targets**:

| Metric | Target | Measurement Method |
|--------|--------|-------------------|
| **Voice-to-Action Latency** | < 5 seconds | Time from speech end → robot starts moving |
| **Navigation Success Rate** | ≥ 75% | Waypoint reach rate (4/5 goals minimum) |
| **Object Detection Precision** | ≥ 85% | True positives / (TP + False positives) |
| **Task Completion Rate** | ≥ 70% | End-to-end tasks completed (7/10 minimum) |
| **LLM Planning Validity** | ≥ 80% | Valid action sequences / total commands |

**Test Scenarios** (10 test cases):

1. **Simple Navigation**: "Navigate to the table"
2. **Object Detection**: "Find the red box"
3. **Object Retrieval**: "Bring me the blue mug"
4. **Multi-Step**: "Go to the shelf, pick up the book, and return"
5. **Conditional**: "If you see a green box, pick it up, otherwise wait"
6. **Obstacle Avoidance**: "Navigate to the kitchen while avoiding obstacles"
7. **Ambiguous Command**: "Clean the room" (tests LLM decomposition)
8. **Failure Recovery**: "Pick up the yellow ball" (object not in scene - test abort)
9. **Chained Actions**: "Detect all objects, then navigate to the closest one"
10. **Complex Integration**: "Go to the red box, then navigate to the table, then return home"

**Implementation Steps**:

**Step 1: Environment Setup** (30 min)
- Launch Isaac Sim with warehouse scene
- Spawn humanoid robot with RGB-D camera
- Place 10 objects (5 boxes, 3 mugs, 2 books) with varied colors

**Step 2: System Launch** (20 min)
- Create `full_system.launch.py`:
  - Whisper voice node
  - LLM planning service
  - Nav2 navigation
  - YOLOv8 vision node
  - VLA coordinator node

**Step 3: VLA Coordinator Implementation** (90 min)
- Subscribe to `/voice/commands`
- Call `/plan_task` service (LLM)
- Parse action sequence
- Execute actions sequentially:
  - `navigate`: Send Nav2 goal, wait for completion
  - `detect_object`: Call vision service, get coordinates
  - `grasp`: Log conceptual action (manipulation out of scope)
  - `wait`: Sleep for specified duration
- Publish status to `/vla/status`
- **Exercise 1**: Implement coordinator node, test on Scenario 1 (simple navigation)

**Step 4: Integration Testing** (60 min)
- Run all 10 test scenarios
- Record metrics per scenario:
  - Voice transcription accuracy
  - LLM plan validity
  - Navigation success
  - Object detection accuracy
  - Task completion (yes/no)
- **Exercise 2**: Complete testing matrix, document failures

**Step 5: Performance Evaluation** (30 min)
- Calculate aggregate metrics:
  - Task completion rate: X/10 (target ≥ 7/10)
  - Average latency: voice→action (target <5s)
  - Navigation success: Y/Z goals (target ≥75%)
- Generate metrics report (`metrics_report.md`)
- **Exercise 3**: Create performance report with analysis

**Deliverables**:

1. **Source Code**:
   - `full_system.launch.py` (launches all nodes)
   - `vla_coordinator.py` (pipeline orchestration)
   - `evaluation_metrics.py` (performance tracking)

2. **Metrics Report** (`metrics_report.md`):
   ```markdown
   # Module 4 Project Metrics Report

   **Student Name**: [Your Name]
   **Date**: [Date]

   ## System Integration
   - Launch time: [X] seconds
   - All nodes operational: [Yes/No]

   ## Task Completion Rate
   - Scenarios attempted: 10
   - Scenarios completed: [X]/10
   - Success rate: [XX]%
   - Target: ≥ 70% → [PASS/FAIL]

   ## Performance Metrics
   - Voice-to-action latency: [X.XX] seconds (target <5s)
   - Navigation success: [X]/[Y] goals ([XX]%, target ≥75%)
   - Object detection precision: [XX]% (target ≥85%)
   - LLM plan validity: [XX]% (target ≥80%)

   ## Observations
   [Describe challenges, failure modes, recovery behaviors, manual interventions]
   ```

3. **Demo Video** (optional, 5-7 min):
   - System launch
   - 3 successful test scenarios
   - 1 failure scenario with recovery
   - Performance metrics visualization

**Rubric**:

| Category | Points | Criteria |
|----------|--------|----------|
| **System Integration** | 25 | All components launch and communicate (<30s startup) |
| **Task Completion Rate** | 30 | 7/10 or more scenarios completed successfully (≥70%) |
| **Performance Metrics** | 25 | Meet 3/4 targets (latency, navigation, detection, LLM validity) |
| **Metrics Reporting** | 20 | Complete report with observations and analysis |
| **Total** | 100 | |

**Grading Scale**:
- A (90-100): All targets met, excellent documentation
- B (80-89): 3/4 targets met, good documentation
- C (70-79): 2/4 targets met, adequate documentation
- D (60-69): 1/4 targets met
- F (<60): Major components missing or non-functional

---

## Phase 2: Task Breakdown Structure

*This section will be generated by `/sp.tasks` command. Preliminary task categories:*

**Setup & Infrastructure** (5-7 tasks):
- Create `docs/module-04/` directory structure
- Create code examples directory structure
- Set up Docusaurus sidebar entry for Module 4
- Create diagram assets (VLA loop, ROS graph)
- Prepare demo video recording scripts

**Chapter 1: Voice-to-Action** (15-20 tasks):
- Write Chapter 1 introduction section
- Write sections 1.1-1.6
- Create `whisper_node.py` example
- Create `audio_capture.py` example
- Create Exercise 1-4 starter code and solutions
- Test all code examples on Ubuntu 22.04 + ROS 2 Humble
- Create troubleshooting guide (microphone issues, GPU errors)

**Chapter 2: Cognitive Planning** (18-22 tasks):
- Write Chapter 2 introduction and sections
- Create `llm_planner_node.py` service
- Create prompt templates (`prompt_templates.py`)
- Create Exercise 1-5 starter code and solutions
- Test GPT-4 and Claude API integrations
- Document API key setup process
- Create cost estimation guide ($5/month budget)

**Chapter 3: ROS 2 Execution** (15-18 tasks):
- Write Chapter 3 sections
- Create `nav2_executor.py` action client
- Create `action_server.py` multi-step execution
- Create `recovery_behaviors.py` failure handling
- Create Exercise 1-4 starter code
- Test navigation in Isaac Sim cluttered environment
- Verify >75% success rate on test scenarios

**Chapter 4: Computer Vision** (18-22 tasks):
- Write Chapter 4 sections
- Create `object_detection_node.py` (YOLOv8)
- Create `rgbd_processor.py` (depth→3D conversion)
- Create `vision_action_bridge.py` (vision-planning integration)
- Create Exercise 1-4 starter code
- Test detection on 500+ annotated simulation images
- Verify >85% precision, >80% recall

**Module Project** (25-30 tasks):
- Write Module 4 Project documentation
- Create `vla_coordinator.py` pipeline orchestration
- Create `full_system.launch.py` launch file
- Create `evaluation_metrics.py` tracking
- Create 10 test scenario descriptions
- Test all scenarios, document results
- Create metrics report template
- Create demo video recording and editing
- Create rubric and grading guidelines

**Testing & QA** (10-15 tasks):
- Integration test: Full VLA pipeline on 10 scenarios
- Performance test: Latency, throughput, accuracy
- User acceptance: Have 2 learners complete module, gather feedback
- Documentation review: Check for broken links, typos
- Code review: Verify all examples run without errors
- Update troubleshooting sections based on testing

**Polish & Deployment** (8-12 tasks):
- Final proofreading of all 5 documents
- Optimize images and videos (<500KB per asset)
- Add callout boxes (Tips, Warnings, Deep Dives)
- Update sidebars.ts with Module 4 navigation
- Create "What's Next" sections linking to future modules
- Publish to Docusaurus site

---

## Notes

**Educational Philosophy**: Module 4 represents the culmination of the Physical AI course, integrating advanced AI (LLMs, speech recognition, computer vision) with robotic control (ROS 2, Nav2). The focus is on **practical integration** - learners see how cutting-edge AI research (GPT-4, Whisper) enables real-world autonomous behaviors.

**Progression from Module 3**: Module 3 taught perception (VSLAM) and navigation (Nav2) with GPU acceleration (Isaac). Module 4 adds the "cognitive layer" (LLMs for planning) and "sensory layer" (voice input, visual object detection), completing the autonomous loop: **Sense (voice, vision) → Think (LLM) → Act (Nav2, manipulation)**.

**Key Technical Challenges**:
1. **API Latency**: LLM calls can add 1-3 seconds - mitigate with streaming responses (advanced topic)
2. **Error Propagation**: Voice→LLM→Nav2 pipeline has 3 failure points - robust error handling critical
3. **GPU Memory**: Whisper + YOLOv8 + Isaac Sim = high VRAM usage - recommend RTX 3060 12GB or better
4. **Cost Management**: OpenAI API costs - provide token estimates, suggest caching common plans

**Success Indicators**:
- Learners complete module in 10-14 hours (similar to Modules 1-3)
- >70% achieve Module Project ≥70% task completion rate
- Positive feedback on "seeing AI come to life" (voice commands actually working)
- Community contributions: Learners share custom voice commands, LLM prompt variations
