# Feature Specification: Physical AI Book Structure and Content Organization

**Feature Branch**: `001-book-structure`
**Created**: 2025-12-22
**Status**: Draft
**Input**: User description: "Based on the constitution, create a detailed specification for the physical AI book. Include: 1. Book structure with 1 chapter and 3 lessons each (Title and Description) 2. Content guidelines and lesson format 3. Docusaurus-specific requirements for organization"

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Reader Completes First Chapter Successfully (Priority: P1)

A beginner with basic Python knowledge but no AI experience opens the Physical AI book and works through the first chapter. They read three progressive lessons, complete all hands-on exercises, and successfully build their first physical AI project (a simple sensor-based system). The reader understands fundamental concepts and feels confident to continue learning.

**Why this priority**: This is the foundation of the entire learning experience. If readers cannot successfully complete the first chapter, they will abandon the book. This story validates that our content structure, pacing, and hands-on approach work for the target audience.

**Independent Test**: Can be fully tested by having a beginner reader work through Chapter 1 in isolation. Success is measured by: (1) completing all three lessons within the expected time, (2) successfully running all code examples, (3) building the chapter project, and (4) passing a brief comprehension quiz.

**Acceptance Scenarios**:

1. **Given** a reader with basic Python knowledge and no AI background, **When** they start Lesson 1 and follow the step-by-step tutorial, **Then** they understand what Physical AI is, can explain it in their own words, and complete the lesson in 15 minutes or less
2. **Given** the reader has completed Lesson 1, **When** they proceed to Lesson 2 with hands-on sensor examples, **Then** they successfully connect a sensor, read data, and display output within 30 minutes including hardware setup
3. **Given** the reader has completed Lessons 1 and 2, **When** they tackle Lesson 3 on basic actuation, **Then** they control an actuator based on sensor input and understand the sense-think-act loop
4. **Given** the reader has completed all three lessons, **When** they attempt the chapter project, **Then** they successfully build a simple reactive system (e.g., temperature-triggered fan) using provided code templates and troubleshooting guides

---

### User Story 2 - Content Author Creates New Lesson Following Standards (Priority: P2)

A content author needs to add a new lesson to the Physical AI book. They reference the content guidelines, follow the standardized lesson template, and create a lesson that includes all required elements: learning objectives, prerequisites, hardware/software requirements, step-by-step tutorial, code examples, troubleshooting section, and exercises. The lesson integrates seamlessly into the Docusaurus site structure.

**Why this priority**: Consistent content quality across all lessons is essential for reader success. This story ensures authors have clear standards to follow, preventing inconsistent pacing, missing elements, or varying quality that would confuse readers.

**Independent Test**: Can be tested by having an author create a sample lesson following the guidelines, then validating the lesson against the quality checklist (all required sections present, code examples tested, estimated time accurate, screenshots included).

**Acceptance Scenarios**:

1. **Given** an author wants to create a new lesson, **When** they access the lesson template and content guidelines, **Then** they understand all required sections, formatting standards, and quality criteria
2. **Given** the author has written lesson content, **When** they add code examples, **Then** all code is syntactically correct, fully commented, tested, and includes expected output
3. **Given** the lesson is complete, **When** the author places it in the Docusaurus folder structure, **Then** the lesson appears in the correct navigation order with proper sidebar categorization
4. **Given** the lesson is published, **When** a reader views it on mobile and desktop, **Then** all text, images, code blocks, and interactive elements render correctly on both platforms

---

### User Story 3 - Educator Navigates Book Structure for Curriculum Planning (Priority: P3)

An educator planning a Physical AI course reviews the book structure to understand the learning progression, topic coverage, and time requirements. They can quickly see the chapter outline, lesson titles, learning objectives, and estimated completion times. This allows them to select appropriate sections for their curriculum and plan class sessions effectively.

**Why this priority**: While not critical for individual learners, educators are important stakeholders who can amplify the book's impact. Clear structure and metadata make it easy for them to adopt the book for teaching, but this is secondary to the actual learning experience.

**Independent Test**: Can be tested by showing an educator the table of contents with metadata (objectives, time, prerequisites) and asking them to plan a 4-week curriculum. Success means they can quickly identify relevant sections and understand dependencies.

**Acceptance Scenarios**:

1. **Given** an educator views the book's table of contents, **When** they review chapter and lesson titles, **Then** they understand the learning progression from fundamentals to advanced topics
2. **Given** the educator clicks on a lesson in the navigation, **When** they view the lesson metadata, **Then** they see learning objectives, prerequisites, estimated time, and required hardware
3. **Given** the educator is planning a curriculum, **When** they review the complete book structure, **Then** they can identify which lessons fit their time constraints and student skill levels
4. **Given** the educator wants to sequence topics differently, **When** they check lesson prerequisites, **Then** they understand dependencies and can safely reorder independent lessons

---

### Edge Cases

- What happens when a reader's hardware differs from examples (different sensor model, different microcontroller)?
  - Each lesson includes a "Hardware Substitutions" section listing compatible alternatives and any code changes required

- How does the system handle readers who skip lessons or jump directly to advanced topics?
  - Each lesson lists explicit prerequisites at the top; readers attempting advanced content without prerequisites see a warning callout with links to required background lessons

- What if code examples fail due to library version mismatches?
  - All tutorials specify exact library versions in requirements files; each lesson includes a "Version Compatibility" section noting tested versions and known issues with other versions

- How does the book accommodate readers using different operating systems (Windows, macOS, Linux)?
  - Platform-specific installation steps are provided in collapsible tabs; code examples avoid OS-specific paths or commands, using cross-platform libraries where possible

- What happens when embedded videos or external resources become unavailable?
  - All critical content exists in text form; videos are supplementary; external links have archive.org backups referenced in footnotes

## Requirements *(mandatory)*

### Functional Requirements

#### Book Structure Requirements

- **FR-001**: The book MUST contain exactly one introductory chapter titled "Foundations of Physical AI"
- **FR-002**: Chapter 1 MUST contain exactly three lessons in sequential order: (1) Understanding Physical AI, (2) Sensing the World, (3) Acting on Information
- **FR-003**: Each lesson MUST include these mandatory sections: Title, Learning Objectives, Prerequisites, Estimated Time, Required Hardware/Software, Introduction, Step-by-Step Tutorial, Code Examples, Troubleshooting, Summary, Exercises, What's Next
- **FR-004**: The book structure MUST support expansion to multiple chapters without modifying existing lesson URLs or navigation structure
- **FR-005**: Each lesson MUST be independently accessible via unique URL following pattern: `/docs/chapter-[number]/lesson-[number]-[slug]`

#### Content Guidelines Requirements

- **FR-006**: Every code example MUST include: syntax highlighting, line numbers, descriptive title, full comments explaining logic, and expected output in a separate code block
- **FR-007**: All technical terms MUST be defined on first use and added to the glossary with backlinks
- **FR-008**: Each lesson MUST state learning objectives using action verbs (understand, build, explain, debug, etc.)
- **FR-009**: Estimated completion time MUST be provided for each lesson and include separate estimates for reading time and hands-on exercises
- **FR-010**: Hardware requirements MUST specify exact models used in tutorials plus at least two tested alternatives under $50 USD
- **FR-011**: Every tutorial MUST include at least one troubleshooting scenario with symptom-diagnosis-solution format
- **FR-012**: Code examples MUST be tested on all three major platforms (Windows, macOS, Linux) before publication
- **FR-013**: Images and diagrams MUST include descriptive alt text and follow the visual style guide (consistent colors, simple icons, clear labels)

#### Lesson Format Requirements

- **FR-014**: Lesson sections MUST appear in this order: (1) Metadata (objectives, time, prerequisites, requirements), (2) Introduction (why this matters), (3) Tutorial, (4) Examples, (5) Troubleshooting, (6) Summary, (7) Exercises, (8) What's Next
- **FR-015**: Introduction sections MUST use real-world scenarios or analogies to motivate learning before presenting technical content
- **FR-016**: "What's Next" sections MUST link to the subsequent lesson and preview upcoming concepts
- **FR-017**: Exercises MUST include starter code or templates; no "exercise left to the reader" without scaffolding
- **FR-018**: Each lesson MUST end with a mini-project that applies concepts learned in that lesson

#### Docusaurus Organization Requirements

- **FR-019**: Content MUST be organized in Docusaurus folder structure: `/docs/chapter-01/lesson-01-understanding-physical-ai.md`
- **FR-020**: Sidebar navigation MUST be configured via `sidebars.js` with collapsible chapter categories
- **FR-021**: Each lesson file MUST include Docusaurus frontmatter with: `id`, `title`, `sidebar_label`, `sidebar_position`, `description`, `keywords`
- **FR-022**: Code examples MUST use Docusaurus code block features: language syntax highlighting, title attribute, line highlighting for important sections
- **FR-023**: The site MUST use Docusaurus tabs component for platform-specific instructions (Windows/macOS/Linux)
- **FR-024**: Callout boxes MUST use Docusaurus admonitions with semantic types: `note`, `tip`, `warning`, `danger`, `info`
- **FR-025**: The site MUST configure Docusaurus search using Algolia DocSearch or local search plugin
- **FR-026**: Navigation MUST include breadcrumbs showing: Home > Chapter Title > Lesson Title
- **FR-027**: The homepage MUST display chapter overview with lesson cards showing: title, description, estimated time, difficulty level
- **FR-028**: All assets (images, videos, code files) MUST be stored in `/static/assets/chapter-[N]/lesson-[N]/` for organization
- **FR-029**: The site MUST be configured as a PWA with offline support for all lesson content
- **FR-030**: Code examples MUST be available as downloadable files linked from each lesson

### Key Entities

- **Chapter**: Represents a major topic area (e.g., "Foundations of Physical AI"); contains ordered lessons; includes chapter-level overview, learning path, and project

- **Lesson**: Represents a single learning unit within a chapter; contains structured content sections; includes metadata (objectives, time, prerequisites, requirements); has unique URL and navigation position

- **Code Example**: Represents a runnable code snippet; includes source code, language identifier, comments, expected output, and downloadable file; associated with specific lesson

- **Exercise**: Represents a hands-on activity for readers; includes description, starter code/template, hints, and solution reference; has difficulty level indicator

- **Hardware Requirement**: Represents physical component needed for lesson; includes exact model, purpose, cost estimate, tested alternatives, purchase links

- **Learning Objective**: Represents measurable learning outcome for a lesson; uses action verb; can be validated through exercises or comprehension checks

- **Glossary Term**: Represents technical term defined in book; includes definition, first usage location, related terms, and backlinks to all occurrences

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Beginners with basic Python knowledge complete Chapter 1 (all three lessons plus project) within 3 hours on first attempt
- **SC-002**: 95% of readers successfully run all code examples in Chapter 1 without modifications when following documented setup steps
- **SC-003**: Readers complete at least 70% of hands-on exercises within each lesson
- **SC-004**: All lesson pages load in under 3 seconds on 3G network connections
- **SC-005**: 90% of readers report understanding Physical AI concepts well enough to explain them in their own words after Chapter 1 (measured via post-chapter survey)
- **SC-006**: Zero broken internal links or missing assets in published book
- **SC-007**: All code examples pass automated testing in CI/CD for Python 3.9+, 3.10, and 3.11
- **SC-008**: Content maintains readability score appropriate for target audience (Flesch-Kincaid Grade Level 8-10)
- **SC-009**: Mobile readers complete lessons at same success rate as desktop readers (95%+ tutorial completion)
- **SC-010**: Educators successfully identify and extract lesson subsets for curriculum planning within 15 minutes of first viewing the structure

### Quality Gates

- **SC-011**: Each lesson reviewed by minimum 2 technical experts for accuracy and 1 beginner tester for clarity before publication
- **SC-012**: All hardware components listed are available for purchase from at least 2 major suppliers at time of publication
- **SC-013**: Image and video assets total less than 5MB per lesson page
- **SC-014**: All external dependencies (libraries, frameworks) are actively maintained with releases in past 6 months

## Assumptions

1. **Target Audience**: Assumes readers have basic programming knowledge (variables, functions, loops, conditionals) in at least one language, preferably Python
2. **Hardware Access**: Assumes readers can acquire affordable hardware (under $200 total for all Chapter 1 components) and have access to a computer with USB ports
3. **Internet Connectivity**: Assumes readers have internet access for initial setup (downloading libraries, viewing documentation) but content should work offline after setup
4. **Time Commitment**: Assumes readers can dedicate 30-60 minute blocks for hands-on lessons; shorter lessons preferred over long sessions
5. **Learning Style**: Assumes hands-on learners who prefer building projects over reading theory; theoretical deep-dives are optional "Deep Dive" sections
6. **Platform**: Assumes readers use Windows, macOS, or Linux; examples prioritize cross-platform tools but may include platform-specific setup steps
7. **Tool Familiarity**: Assumes readers can use command line for basic tasks (installing packages, running scripts) with guided instructions provided

## Out of Scope

- Advanced robotics topics (SLAM, computer vision, path planning) - reserved for future chapters beyond Chapter 1
- Hardware design and electronics theory - book focuses on using sensors/actuators, not designing them
- Production deployment and scaling - examples are educational, not production-ready systems
- Machine learning model training - book uses pre-trained models where applicable
- Real-time operating systems - examples use standard Python on desktop/laptop OS
- Multi-robot systems and swarms - limited to single-agent systems in Chapter 1
- Custom PCB design - uses off-the-shelf development boards and breadboards
- Video production - supporting videos are supplementary, text is primary medium

## Dependencies

- **Docusaurus 3.x**: Static site generator for documentation website; must be installed and configured
- **Python 3.9+**: Programming language for all code examples; readers must have Python installed
- **Hardware Suppliers**: Availability of specific sensors/actuators from vendors like Adafruit, SparkFun, Arduino
- **Open-Source Libraries**: Dependencies on maintained libraries (e.g., pyserial, RPi.GPIO, Adafruit libraries)
- **Hosting Platform**: Requires web hosting for published Docusaurus site (e.g., Netlify, Vercel, GitHub Pages)
- **Git/GitHub**: For version control, issue tracking, and community contributions
- **CI/CD Platform**: For automated testing of code examples (e.g., GitHub Actions)

## Risks & Mitigations

1. **Risk**: Hardware components become unavailable or discontinued
   - **Mitigation**: List multiple tested alternatives for each component; focus on popular, widely-used sensors with many manufacturers

2. **Risk**: Library versions change and break examples
   - **Mitigation**: Pin exact library versions in requirements files; include version compatibility notes; schedule quarterly review of dependencies

3. **Risk**: Readers lack hardware access or budget
   - **Mitigation**: Provide simulator/emulator options for software-only learning; suggest borrowing hardware through maker spaces or libraries

4. **Risk**: Content becomes outdated quickly
   - **Mitigation**: Focus on fundamental concepts over trendy frameworks; establish quarterly review cycle; track reader-reported issues via GitHub

5. **Risk**: Examples too simple to be engaging vs. too complex to complete
   - **Mitigation**: Multiple beginner testers validate each lesson; provide optional "Deep Dive" and "Challenge" sections for advanced readers

## Detailed Book Structure

### Chapter 1: Foundations of Physical AI

**Chapter Overview**: This introductory chapter establishes the foundation for physical AI by explaining core concepts, demonstrating basic sensing and actuation, and guiding readers through their first complete sense-think-act system. By the end of this chapter, readers understand what physical AI is, can work with sensors and actuators, and have built a simple reactive system.

**Chapter Learning Objectives**:
- Define physical AI and explain how it differs from traditional software AI
- Describe the sense-think-act loop and identify examples in real-world systems
- Connect sensors to read environmental data
- Control actuators based on sensor input
- Build a complete reactive system from scratch

**Estimated Chapter Time**: 3 hours (1 hour reading, 2 hours hands-on)

**Chapter Project**: Build a temperature-responsive fan controller that reads temperature via sensor and adjusts fan speed based on configurable thresholds

---

#### Lesson 1: Understanding Physical AI

**Lesson Title**: Understanding Physical AI: From Bits to Atoms

**Description**: This lesson introduces the concept of physical AI and explains how it differs from traditional software AI. Readers learn the sense-think-act loop, see real-world examples, and understand why physical AI matters. No hardware required for this lesson - it's purely conceptual with visual examples.

**Learning Objectives**:
- Define physical AI in your own words and give three examples
- Explain the sense-think-act loop and identify its components in everyday devices
- Distinguish between software AI (purely digital) and physical AI (embodied in the world)
- Describe at least two applications of physical AI in modern technology

**Prerequisites**:
- Basic understanding of what AI is (e.g., knowing AI can recognize images or play chess)
- No programming or hardware experience required for this lesson

**Estimated Time**: 15 minutes reading

**Required Hardware/Software**: None (conceptual lesson)

**Content Outline**:
1. **Introduction - Why Physical AI?**: Real-world scenario of a smart thermostat that learns and adapts
2. **What is Physical AI?**: Definition with visual diagrams showing how AI connects to physical world through sensors/actuators
3. **The Sense-Think-Act Loop**: Detailed explanation with animated diagram showing the cycle
4. **Real-World Examples**:
   - Self-driving cars (sensors: cameras, lidar; think: path planning; act: steering, braking)
   - Robot vacuums (sensors: bump sensors, cameras; think: mapping, navigation; act: motors, brushes)
   - Smart thermostats (sensors: temperature, occupancy; think: learning schedule; act: HVAC control)
5. **Physical AI vs. Software AI**: Comparison table highlighting differences (environment interaction, real-time constraints, physical embodiment)
6. **Why It Matters**: Discussion of physical AI's impact on robotics, automation, IoT
7. **Summary**: Key takeaways reinforcing main concepts
8. **Exercises**:
   - Identify sense-think-act components in three household devices
   - Brainstorm a physical AI application for a problem you care about
9. **What's Next**: Preview of Lesson 2 (getting hands-on with sensors)

---

#### Lesson 2: Sensing the World

**Lesson Title**: Sensing the World: Reading Data from Physical Sensors

**Description**: This hands-on lesson teaches readers how to connect a temperature sensor to their computer and read data using Python. Readers learn about analog vs. digital sensors, sensor interfacing basics, and how to process sensor data. By the end, they can reliably read temperature values and display them in real-time.

**Learning Objectives**:
- Connect a temperature sensor to a computer via USB-serial or GPIO
- Write Python code to read sensor data at regular intervals
- Understand the difference between analog and digital sensors
- Handle sensor errors and missing data gracefully
- Display sensor readings in human-readable format

**Prerequisites**:
- Completion of Lesson 1 (Understanding Physical AI)
- Python basics (variables, loops, functions, print statements)
- Ability to run Python scripts from command line

**Estimated Time**: 45 minutes (15 min reading, 30 min hands-on)

**Required Hardware**:
- **Primary Option**: DHT22 temperature/humidity sensor ($10, digital, simple wiring)
  - Alternative 1: DS18B20 waterproof temperature sensor ($8, digital, one-wire protocol)
  - Alternative 2: TMP36 analog temperature sensor ($2, requires ADC like MCP3008)
- **Microcontroller/Interface**:
  - Primary: Raspberry Pi Pico ($4, USB connection, easy setup)
  - Alternative 1: Arduino Uno ($25, widely available)
  - Alternative 2: ESP32 ($8, has WiFi for future lessons)
- **Accessories**: Breadboard ($5), jumper wires ($3), USB cable (usually included)

**Required Software**:
- Python 3.9 or higher
- Libraries: `pyserial` (for USB communication), `time` (built-in)
- Platform-specific: `RPi.GPIO` (Raspberry Pi), `Adafruit_CircuitPython` (generic)

**Content Outline**:
1. **Introduction - Why Sensors?**: Scenario of building a plant watering system that needs to know soil moisture
2. **Sensor Basics**:
   - What sensors measure (temperature, light, distance, etc.)
   - Analog vs. digital sensors explained with diagrams
   - How computers read sensor values (ADC, GPIO, serial protocols)
3. **Hardware Setup - Step by Step**:
   - Wiring diagram (color-coded, breadboard view)
   - Connection checklist
   - Power and ground safety
   - Platform-specific tabs (Windows/macOS/Linux) for drivers
4. **Software Setup**:
   - Installing Python libraries (`pip install pyserial`)
   - Testing connection with simple script
   - Troubleshooting common setup issues
5. **Reading Sensor Data - Tutorial**:
   - Complete code example with line-by-line explanation
   - Reading single value vs. continuous reading loop
   - Converting raw values to meaningful units (Celsius, Fahrenheit)
   - Printing formatted output
6. **Handling Errors**:
   - What happens when sensor disconnected
   - Dealing with noisy data (smoothing, filtering basics)
   - Retry logic for failed reads
7. **Code Example - Complete Working Script**:
   ```python
   # Title: Continuous Temperature Monitoring
   # Full commented code here with expected output shown separately
   ```
8. **Troubleshooting**:
   - Symptom: "No data from sensor" → Check wiring, verify power, test with multimeter
   - Symptom: "Erratic readings" → Check loose connections, add delays between reads
   - Symptom: "Permission denied on serial port" → Platform-specific fixes
9. **Summary**: What readers learned and built
10. **Exercises**:
    - Modify code to read every 5 seconds instead of 1 second
    - Add logging to save readings to a text file
    - Calculate and display average temperature over last 10 readings
    - Challenge: Plot readings in real-time using matplotlib
11. **What's Next**: Preview of Lesson 3 (using sensor data to control actuators)

---

#### Lesson 3: Acting on Information

**Lesson Title**: Acting on Information: Controlling Actuators Based on Sensor Data

**Description**: This lesson completes the sense-think-act loop by adding actuation. Readers learn to control a simple actuator (LED or fan) based on sensor input, implementing basic logic to create a reactive system. This brings together sensing from Lesson 2 with actuation to build their first autonomous physical AI system.

**Learning Objectives**:
- Connect and control an actuator (LED or DC fan) programmatically
- Implement conditional logic to respond to sensor thresholds
- Build a complete sense-think-act loop in a single program
- Understand PWM (Pulse Width Modulation) for variable control
- Test and debug reactive systems

**Prerequisites**:
- Completion of Lessons 1 and 2
- Working sensor setup from Lesson 2
- Understanding of if/else statements and comparison operators

**Estimated Time**: 60 minutes (15 min reading, 45 min hands-on)

**Required Hardware**:
- All hardware from Lesson 2 (temperature sensor + microcontroller)
- **Actuator Options**:
  - Primary: 5V DC fan ($5, simple, visual feedback)
  - Alternative 1: LED ($0.25, simplest for testing logic)
  - Alternative 2: Relay module ($3, can control higher power devices)
- **Additional Components**:
  - Transistor (2N2222 or similar, $0.50) for controlling fan
  - Resistor (220Ω for LED or 1kΩ for transistor base, $0.10)
  - Diode (1N4007, $0.10, for back-EMF protection if using fan)

**Required Software**:
- All software from Lesson 2
- Additional library: `gpiozero` (Raspberry Pi) or `Adafruit_Blinka` (generic)

**Content Outline**:
1. **Introduction - Closing the Loop**: Revisit sense-think-act diagram, now focusing on "act" component
2. **Actuator Basics**:
   - Types of actuators (motors, servos, LEDs, relays, solenoids)
   - Choosing actuator for task
   - Power requirements and safety (avoiding damage to microcontroller)
3. **Hardware Setup - Adding Actuator**:
   - Wiring diagram showing sensor + actuator
   - Transistor circuit explanation (why needed, how it works)
   - Safety: diode for inductive loads (fans, motors)
   - Component checklist
4. **Digital Control - On/Off**:
   - Simple code to turn LED/fan on or off
   - GPIO output basics
   - Testing actuator independently before integration
5. **PWM for Variable Control**:
   - What is PWM (visual diagram with duty cycle)
   - How PWM controls brightness/speed
   - Code example: dimming LED or varying fan speed
6. **Implementing Logic - The "Think" Part**:
   - Threshold-based control (if temp > 25°C, turn on fan)
   - Hysteresis to prevent flickering (on at 25°C, off at 23°C)
   - Proportional control (fan speed increases with temperature)
7. **Complete System - Tutorial**:
   - Full code example integrating sensor reading + actuator control
   - Main loop: read sensor → evaluate logic → control actuator → repeat
   - Adding user feedback (print statements showing state)
8. **Code Example - Temperature-Controlled Fan**:
   ```python
   # Title: Reactive Temperature Controller
   # Full commented code with clear sections for sense, think, act
   ```
9. **Testing and Debugging**:
   - How to test logic without hardware (simulation)
   - Verifying sensor reads correctly
   - Checking actuator responds to commands
   - End-to-end testing procedure
10. **Troubleshooting**:
    - Symptom: "Actuator doesn't respond" → Check wiring, verify GPIO pin number, test with simple on/off script
    - Symptom: "Actuator stuck on or off" → Check logic conditions, add debug prints, verify threshold values
    - Symptom: "System behaves erratically" → Add delays, check for race conditions, verify power supply adequate
11. **Summary**: Readers built their first complete physical AI system with sense-think-act loop
12. **Exercises**:
    - Add a second threshold (e.g., different fan speeds for different temps)
    - Implement hysteresis to prevent rapid switching
    - Add a manual override button to turn fan on/off regardless of temperature
    - Challenge: Create a data logger that records when fan turns on/off and why
13. **What's Next**: Congratulations message, preview of Chapter 2 topics (more sophisticated sensing, learning behaviors), link to chapter project

---

## Lesson Content Template

Each lesson must follow this standardized template structure:

```markdown
---
id: lesson-[number]-[slug]
title: [Full Lesson Title]
sidebar_label: [Short Label]
sidebar_position: [number]
description: [One-sentence description for SEO]
keywords: [physical ai, sensors, actuators, python, robotics]
---

# [Lesson Title]: [Subtitle]

:::info Learning Objectives
By the end of this lesson, you will be able to:
- [Objective 1 with action verb]
- [Objective 2 with action verb]
- [Objective 3 with action verb]
:::

## Prerequisites

- [Prerequisite 1 with link to relevant lesson if applicable]
- [Prerequisite 2]

## Estimated Time

- Reading: [X] minutes
- Hands-on exercises: [Y] minutes
- **Total: [X+Y] minutes**

## Required Hardware

<Tabs groupId="hardware-option">
  <TabItem value="primary" label="Recommended">
    - [Component name] ([price], [reason for recommendation])
    - [Purchase links]
  </TabItem>
  <TabItem value="alt1" label="Budget Alternative">
    - [Alternative components with tradeoffs noted]
  </TabItem>
</Tabs>

## Required Software

<Tabs groupId="operating-system">
  <TabItem value="windows" label="Windows">
    [Windows-specific setup]
  </TabItem>
  <TabItem value="mac" label="macOS">
    [macOS-specific setup]
  </TabItem>
  <TabItem value="linux" label="Linux">
    [Linux-specific setup]
  </TabItem>
</Tabs>

## Introduction

[Real-world scenario or analogy that motivates this lesson]

[Explanation of why this topic matters]

## [Main Content Sections]

[Tutorial-style content with step-by-step instructions]

### Code Example: [Descriptive Title]

```python title="temperature_reader.py" showLineNumbers
# Full code example with comprehensive comments
# explaining the "why" not just the "what"
```

**Expected Output:**
```
[Show what user should see when running the code]
```

:::tip Pro Tip
[Helpful insight or best practice]
:::

:::warning Common Mistake
[Frequent error students make and how to avoid it]
:::

## Troubleshooting

<details>
  <summary>Symptom: [Problem description]</summary>

  **Possible Causes:**
  - [Cause 1]
  - [Cause 2]

  **Solutions:**
  1. [Step-by-step fix]
  2. [Alternative approach]
</details>

## Summary

In this lesson, you learned:
- [Key takeaway 1]
- [Key takeaway 2]
- [Key takeaway 3]

You built: [Concrete outcome - what they created]

## Exercises

### Exercise 1: [Title] (Difficulty: ⭐)

[Description of task]

**Starter Code:** [Link to downloadable file or embedded code block]

**Hints:**
- [Hint 1]
- [Hint 2]

<details>
  <summary>View Solution</summary>

  ```python
  # Solution code
  ```

  **Explanation:** [Why this solution works]
</details>

### Exercise 2: [Title] (Difficulty: ⭐⭐)

[More challenging task]

### Challenge Exercise: [Title] (Difficulty: ⭐⭐⭐)

[Optional advanced exercise for motivated learners]

## What's Next?

[Preview of next lesson with one-sentence hook]

[Link to next lesson]

---

**Download:** [All code examples for this lesson](link) | **Discuss:** [Ask questions on forum](link)
```

## Docusaurus Configuration Requirements

### File Structure

```
physical-ai-book/
├── docs/
│   ├── intro.md (homepage)
│   ├── chapter-01/
│   │   ├── _category_.json (chapter metadata)
│   │   ├── index.md (chapter overview)
│   │   ├── lesson-01-understanding-physical-ai.md
│   │   ├── lesson-02-sensing-the-world.md
│   │   ├── lesson-03-acting-on-information.md
│   │   └── chapter-project.md
│   └── glossary.md
├── static/
│   ├── assets/
│   │   └── chapter-01/
│   │       ├── lesson-01/
│   │       │   └── (images, diagrams)
│   │       ├── lesson-02/
│   │       │   └── (wiring diagrams, screenshots)
│   │       └── lesson-03/
│   │           └── (circuit diagrams, photos)
│   ├── code-examples/
│   │   └── chapter-01/
│   │       ├── lesson-02/
│   │       │   └── temperature_reader.py
│   │       └── lesson-03/
│   │           └── reactive_controller.py
│   └── img/ (logos, icons, brand assets)
├── src/
│   └── components/ (custom React components if needed)
├── sidebars.js
├── docusaurus.config.js
└── package.json
```

### sidebars.js Configuration

```javascript
module.exports = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Welcome',
    },
    {
      type: 'category',
      label: 'Chapter 1: Foundations of Physical AI',
      collapsed: false,
      items: [
        'chapter-01/index',
        'chapter-01/lesson-01-understanding-physical-ai',
        'chapter-01/lesson-02-sensing-the-world',
        'chapter-01/lesson-03-acting-on-information',
        'chapter-01/chapter-project',
      ],
    },
    {
      type: 'doc',
      id: 'glossary',
      label: 'Glossary',
    },
  ],
};
```

### docusaurus.config.js Key Settings

```javascript
module.exports = {
  title: 'Physical AI: A Hands-On Introduction',
  tagline: 'Learn to build intelligent systems that sense and act in the real world',
  url: 'https://your-domain.com',
  baseUrl: '/',

  themeConfig: {
    navbar: {
      title: 'Physical AI',
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Book',
        },
        {
          to: '/glossary',
          label: 'Glossary',
          position: 'left',
        },
        {
          href: 'https://github.com/your-repo',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Learning',
          items: [
            {label: 'Get Started', to: '/docs/intro'},
            {label: 'Chapter 1', to: '/docs/chapter-01'},
          ],
        },
        {
          title: 'Community',
          items: [
            {label: 'Discussions', href: 'https://github.com/your-repo/discussions'},
            {label: 'Report Issue', href: 'https://github.com/your-repo/issues'},
          ],
        },
      ],
    },

    prism: {
      theme: require('prism-react-renderer/themes/github'),
      darkTheme: require('prism-react-renderer/themes/dracula'),
      additionalLanguages: ['python', 'cpp', 'bash'],
    },

    algolia: {
      appId: 'YOUR_APP_ID',
      apiKey: 'YOUR_API_KEY',
      indexName: 'physical-ai',
    },
  },

  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your-repo/edit/main/',
          showLastUpdateTime: true,
          breadcrumbs: true,
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-pwa',
      {
        offlineModeActivationStrategies: ['appInstalled', 'queryString'],
        pwaHead: [
          {
            tagName: 'link',
            rel: 'icon',
            href: '/img/icon.png',
          },
          {
            tagName: 'link',
            rel: 'manifest',
            href: '/manifest.json',
          },
        ],
      },
    ],
  ],
};
```

### Custom CSS (src/css/custom.css) - Key Styles

```css
:root {
  /* Brand colors aligned with constitution visual identity */
  --ifm-color-primary: #2e8555; /* Adjust to your brand */
  --ifm-code-font-size: 95%;
  --ifm-font-size-base: 16px;
}

/* High contrast for accessibility */
.markdown h1, .markdown h2, .markdown h3 {
  font-weight: 600;
}

/* Code block title styling */
div[class^='codeBlockTitle'] {
  background-color: var(--ifm-color-primary);
  color: white;
  font-weight: bold;
}

/* Consistent callout box styling */
.admonition {
  margin-bottom: 1.5rem;
}
```

## Content Guidelines Summary

1. **Writing Style**: Friendly, encouraging, active voice; address reader as "you"
2. **Technical Depth**: Explain "why" not just "what"; use analogies before technical terms
3. **Code Quality**: All code tested, fully commented, follows PEP 8, includes error handling
4. **Visual Assets**: Diagrams use consistent colors (hardware=blue, software=green, theory=purple), simple icons, clear labels
5. **Accessibility**: Alt text for images, high contrast, mobile-responsive, keyboard navigable
6. **Interactivity**: Use Docusaurus tabs for alternatives, collapsible details for solutions, admonitions for tips/warnings
7. **Consistency**: All lessons follow same template structure, same metadata format, same exercise format
8. **Offline Support**: PWA configuration, downloadable code, text-first content (videos supplementary)
9. **Version Control**: Pin library versions, document compatibility, schedule quarterly reviews
10. **Community**: Contribution guidelines, issue templates, acknowledge contributors
