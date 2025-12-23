# Tasks: Physical AI Book in Docusaurus

**Input**: Design documents from `/specs/001-book-structure/`
**Prerequisites**: plan.md (Docusaurus infrastructure and content phases), spec.md (3 user stories with priorities)

**Tests**: No test tasks included - spec does not explicitly request TDD approach. Quality validation occurs through beginner testing and technical review in Phase 8.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story. User Story 1 (Reader Success) is the MVP focus.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1=Reader, US2=Author, US3=Educator)
- Include exact file paths in descriptions

## Path Conventions

This is a Docusaurus static site project with this structure:
- **Docusaurus root**: `physical-ai-book/`
- **Content**: `physical-ai-book/docs/`
- **Assets**: `physical-ai-book/static/`
- **Components**: `physical-ai-book/src/`
- **Templates**: `physical-ai-book/templates/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Initialize Docusaurus project and configure for Physical AI book

- [x] T001 Create physical-ai-book directory and initialize Docusaurus 3.x project with TypeScript template using `npx create-docusaurus@latest physical-ai-book classic --typescript`
- [x] T002 Install Docusaurus plugins: @docusaurus/plugin-pwa, @docusaurus/plugin-ideal-image, @docusaurus/theme-live-codeblock in physical-ai-book/package.json
- [x] T003 [P] Install development tools: webpack-bundle-analyzer for bundle size monitoring in physical-ai-book/package.json
- [x] T004 [P] Create directory structure for static assets: physical-ai-book/static/assets/chapter-01/{lesson-01,lesson-02,lesson-03}/
- [x] T005 [P] Create directory structure for code examples: physical-ai-book/static/code-examples/chapter-01/{lesson-02,lesson-03}/
- [x] T006 [P] Create directory structure for brand assets: physical-ai-book/static/img/ (logo.svg, favicon.ico, icon.png for PWA)
- [x] T007 [P] Create directory structure for templates: physical-ai-book/templates/
- [x] T008 [P] Create directory structure for tests: physical-ai-book/tests/{code-examples,components}/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core Docusaurus configuration and infrastructure that MUST be complete before content development

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [x] T009 Configure docusaurus.config.js with site metadata (title: "Physical AI: A Hands-On Introduction", tagline, URL, organization) in physical-ai-book/docusaurus.config.js
- [x] T010 Configure navbar with Book/Glossary/GitHub links in physical-ai-book/docusaurus.config.js themeConfig.navbar
- [x] T011 Configure footer with Learning/Community/More sections and copyright in physical-ai-book/docusaurus.config.js themeConfig.footer
- [x] T012 Configure Prism theme for code syntax highlighting (Python, C++, JavaScript) in physical-ai-book/docusaurus.config.js themeConfig.prism
- [x] T013 Add @docusaurus/plugin-pwa configuration with offline support and PWA head tags in physical-ai-book/docusaurus.config.js plugins
- [x] T014 Add @docusaurus/plugin-ideal-image configuration for responsive images in physical-ai-book/docusaurus.config.js plugins
- [x] T015 Add Algolia DocSearch placeholder configuration in physical-ai-book/docusaurus.config.js themeConfig.algolia (will configure later)
- [x] T016 Configure sidebars.js with Welcome page, Chapter 1 category (collapsed: false), and Glossary page in physical-ai-book/sidebars.js
- [x] T017 Create custom.css with brand colors (primary: #2e8555), code block styling, lesson card styles, hardware table styles, mobile responsiveness in physical-ai-book/src/css/custom.css
- [x] T018 Create PWA manifest.json with app name, icons, theme colors, display mode in physical-ai-book/static/manifest.json
- [x] T019 Create docs directory structure: physical-ai-book/docs/chapter-01/
- [x] T020 Create chapter-01/_category_.json with metadata (label: "Chapter 1: Foundations of Physical AI", position: 1, collapsed: false) in physical-ai-book/docs/chapter-01/_category_.json
- [x] T021 [P] Create .github/workflows directory for CI/CD: physical-ai-book/.github/workflows/
- [x] T022 [P] Create .github/ISSUE_TEMPLATE directory: physical-ai-book/.github/ISSUE_TEMPLATE/
- [x] T023 Test local development server with `npm start` to verify Docusaurus configuration is functional

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Reader Completes First Chapter Successfully (Priority: P1) 🎯 MVP

**Goal**: Enable beginner readers to learn Physical AI through 3 progressive lessons and successfully build their first physical AI project

**Independent Test**: Have a beginner reader work through Chapter 1 in isolation. Success = (1) complete 3 lessons in <3 hours, (2) run all code examples successfully, (3) build chapter project, (4) pass comprehension quiz

### Implementation for User Story 1 - Foundation

- [ ] T024 [P] [US1] Create lesson template with all required sections (frontmatter, learning objectives, prerequisites, time estimate, hardware/software, intro, tutorial, code examples, troubleshooting, summary, exercises, what's next) in physical-ai-book/templates/lesson-template.mdx
- [ ] T025 [P] [US1] Create chapter overview template with learning objectives, time estimate, prerequisites, project description in physical-ai-book/templates/chapter-template.md
- [ ] T026 [P] [US1] Create React component LessonCard.tsx for displaying lesson metadata (title, description, time, difficulty, link) in physical-ai-book/src/components/LessonCard.tsx
- [ ] T027 [P] [US1] Create React component HardwareList.tsx for rendering hardware requirements table with pricing and alternatives in physical-ai-book/src/components/HardwareList.tsx
- [ ] T028 [P] [US1] Create React component CodeDownload.tsx for download button with file size and link in physical-ai-book/src/components/CodeDownload.tsx
- [ ] T029 [P] [US1] Create React component PlatformTabs.tsx wrapping Docusaurus Tabs for OS-specific instructions in physical-ai-book/src/components/PlatformTabs.tsx

### Implementation for User Story 1 - Lesson 1 (Conceptual)

- [ ] T030 [US1] Create chapter-01/index.md chapter overview page with Chapter 1 title, learning objectives (5 objectives), estimated time (3 hours), chapter project description (temperature-responsive fan) in physical-ai-book/docs/chapter-01/index.md
- [ ] T031 [US1] Create lesson-01-understanding-physical-ai.md with frontmatter (id, title, sidebar_label, sidebar_position: 1, description, keywords) in physical-ai-book/docs/chapter-01/lesson-01-understanding-physical-ai.md
- [ ] T032 [US1] Write Lesson 1 Learning Objectives section (4 objectives: define physical AI, explain sense-think-act loop, distinguish from software AI, describe applications) in lesson-01-understanding-physical-ai.md
- [ ] T033 [US1] Write Lesson 1 Prerequisites section (basic AI understanding, no programming required) and Estimated Time (15 min reading) in lesson-01-understanding-physical-ai.md
- [ ] T034 [US1] Write Lesson 1 Introduction with real-world scenario (smart thermostat) in lesson-01-understanding-physical-ai.md
- [ ] T035 [US1] Write Lesson 1 "What is Physical AI?" section with definition and visual diagram description in lesson-01-understanding-physical-ai.md
- [ ] T036 [US1] Write Lesson 1 "Sense-Think-Act Loop" section with detailed explanation and diagram description in lesson-01-understanding-physical-ai.md
- [ ] T037 [US1] Write Lesson 1 "Real-World Examples" section with 3 examples (self-driving cars, robot vacuums, smart thermostats) in lesson-01-understanding-physical-ai.md
- [ ] T038 [US1] Write Lesson 1 "Physical AI vs Software AI" comparison table section in lesson-01-understanding-physical-ai.md
- [ ] T039 [US1] Write Lesson 1 Summary and Exercises sections (identify components in 3 devices, brainstorm application) in lesson-01-understanding-physical-ai.md
- [ ] T040 [US1] Write Lesson 1 "What's Next" section with preview of Lesson 2 (hands-on sensors) in lesson-01-understanding-physical-ai.md
- [ ] T041 [P] [US1] Create sense-think-act loop diagram (SVG or PNG with alt text) in physical-ai-book/static/assets/chapter-01/lesson-01/sense-think-act-loop.png
- [ ] T042 [P] [US1] Create Physical AI vs Software AI comparison table image or embedded markdown table in physical-ai-book/static/assets/chapter-01/lesson-01/comparison-table.png
- [ ] T043 [P] [US1] Source or create example images (smart thermostat, robot vacuum, self-driving car) with proper alt text in physical-ai-book/static/assets/chapter-01/lesson-01/

### Implementation for User Story 1 - Lesson 2 (Hands-On Sensing)

- [ ] T044 [US1] Create lesson-02-sensing-the-world.md with frontmatter (id, title, sidebar_label, sidebar_position: 2, description, keywords including DHT22, sensors, Python) in physical-ai-book/docs/chapter-01/lesson-02-sensing-the-world.md
- [ ] T045 [US1] Write Lesson 2 Learning Objectives (5 objectives: connect sensor, write Python code, understand analog vs digital, error handling, display readings) in lesson-02-sensing-the-world.md
- [ ] T046 [US1] Write Lesson 2 Prerequisites (Lesson 1, Python basics, command line) and Estimated Time (15 min reading, 30 min hands-on) in lesson-02-sensing-the-world.md
- [ ] T047 [US1] Write Lesson 2 Required Hardware section with Docusaurus Tabs for 3 options (DHT22 primary $22, DS18B20 alternative, TMP36 budget) using HardwareList component in lesson-02-sensing-the-world.md
- [ ] T048 [US1] Write Lesson 2 Required Software section with PlatformTabs for Windows/macOS/Linux installation instructions (Python, pyserial, adafruit-circuitpython-dht) in lesson-02-sensing-the-world.md
- [ ] T049 [US1] Write Lesson 2 Introduction (plant watering system scenario) and Sensor Basics section (analog vs digital, ADC, GPIO explained) in lesson-02-sensing-the-world.md
- [ ] T050 [US1] Write Lesson 2 Hardware Setup section with step-by-step wiring instructions and reference to wiring diagram in lesson-02-sensing-the-world.md
- [ ] T051 [US1] Write Lesson 2 Software Setup section (pip install, testing connection, troubleshooting) in lesson-02-sensing-the-world.md
- [ ] T052 [US1] Write Lesson 2 Reading Sensor Data tutorial section with line-by-line code explanation in lesson-02-sensing-the-world.md
- [ ] T053 [US1] Create temperature_reader.py with full comments, error handling, Celsius/Fahrenheit conversion, continuous reading loop in physical-ai-book/static/code-examples/chapter-01/lesson-02/temperature_reader.py
- [ ] T054 [US1] Create requirements.txt for Lesson 2 with pinned library versions (Adafruit-Blinka==8.20.0, adafruit-circuitpython-dht==3.7.9, pyserial==3.5) in physical-ai-book/static/code-examples/chapter-01/lesson-02/requirements.txt
- [ ] T055 [US1] Write Lesson 2 Code Example section with title, showLineNumbers, expected output block, Pro Tip admonition, Common Mistake warning in lesson-02-sensing-the-world.md
- [ ] T056 [US1] Write Lesson 2 Handling Errors section (sensor disconnect, noisy data, retry logic) in lesson-02-sensing-the-world.md
- [ ] T057 [US1] Write Lesson 2 Troubleshooting section with 3+ scenarios (no data, erratic readings, permission denied) using Docusaurus details component in lesson-02-sensing-the-world.md
- [ ] T058 [US1] Write Lesson 2 Summary and Exercises sections (modify read interval, add logging, calculate average, challenge: matplotlib plotting) in lesson-02-sensing-the-world.md
- [ ] T059 [US1] Write Lesson 2 "What's Next" section with preview of Lesson 3 (actuators) and CodeDownload component links in lesson-02-sensing-the-world.md
- [ ] T060 [P] [US1] Create color-coded breadboard wiring diagram for DHT22 to Raspberry Pi Pico with clear labels in physical-ai-book/static/assets/chapter-01/lesson-02/dht22-wiring-diagram.png
- [ ] T061 [P] [US1] Create alternative wiring diagrams for DS18B20 and TMP36 sensors in physical-ai-book/static/assets/chapter-01/lesson-02/
- [ ] T062 [P] [US1] Create photo of actual hardware setup (breadboard, sensor, microcontroller, wires) in physical-ai-book/static/assets/chapter-01/lesson-02/hardware-photo.jpg
- [ ] T063 [P] [US1] Create screenshot of expected terminal output showing temperature readings in physical-ai-book/static/assets/chapter-01/lesson-02/terminal-output.png

### Implementation for User Story 1 - Lesson 3 (Hands-On Actuation)

- [ ] T064 [US1] Create lesson-03-acting-on-information.md with frontmatter (id, title, sidebar_label, sidebar_position: 3, description, keywords including actuators, PWM, fan control) in physical-ai-book/docs/chapter-01/lesson-03-acting-on-information.md
- [ ] T065 [US1] Write Lesson 3 Learning Objectives (5 objectives: connect actuator, implement logic, build sense-think-act loop, understand PWM, test/debug) in lesson-03-acting-on-information.md
- [ ] T066 [US1] Write Lesson 3 Prerequisites (Lessons 1-2, working sensor, if/else understanding) and Estimated Time (15 min reading, 45 min hands-on) in lesson-03-acting-on-information.md
- [ ] T067 [US1] Write Lesson 3 Required Hardware section with previous components + actuator options (5V DC fan primary $5, LED alternative, relay module) and additional components (transistor, resistor, diode) in lesson-03-acting-on-information.md
- [ ] T068 [US1] Write Lesson 3 Required Software section (same as Lesson 2 + gpiozero or Adafruit_Blinka) in lesson-03-acting-on-information.md
- [ ] T069 [US1] Write Lesson 3 Introduction (closing the loop), Actuator Basics (types, choosing, power/safety) in lesson-03-acting-on-information.md
- [ ] T070 [US1] Write Lesson 3 Hardware Setup section with wiring diagram reference showing sensor + actuator + transistor circuit in lesson-03-acting-on-information.md
- [ ] T071 [US1] Write Lesson 3 Digital Control (on/off) section with simple GPIO output code in lesson-03-acting-on-information.md
- [ ] T072 [US1] Write Lesson 3 PWM for Variable Control section with duty cycle explanation and code example for dimming/speed control in lesson-03-acting-on-information.md
- [ ] T073 [US1] Write Lesson 3 Implementing Logic section (threshold-based control, hysteresis to prevent flickering, proportional control) in lesson-03-acting-on-information.md
- [ ] T074 [US1] Create reactive_controller.py with full sense-think-act loop integrating sensor reading + threshold logic + actuator control with clear section comments in physical-ai-book/static/code-examples/chapter-01/lesson-03/reactive_controller.py
- [ ] T075 [US1] Create requirements.txt for Lesson 3 with same libraries as Lesson 2 + gpiozero==2.0.1 in physical-ai-book/static/code-examples/chapter-01/lesson-03/requirements.txt
- [ ] T076 [US1] Write Lesson 3 Complete System tutorial section with full code example, main loop explanation, user feedback (print statements) in lesson-03-acting-on-information.md
- [ ] T077 [US1] Write Lesson 3 Testing and Debugging section (logic testing without hardware, end-to-end procedure) in lesson-03-acting-on-information.md
- [ ] T078 [US1] Write Lesson 3 Troubleshooting section with 3+ scenarios (actuator no response, stuck on/off, erratic behavior) using details components in lesson-03-acting-on-information.md
- [ ] T079 [US1] Write Lesson 3 Summary and Exercises sections (add second threshold, implement hysteresis, manual override button, challenge: data logger) in lesson-03-acting-on-information.md
- [ ] T080 [US1] Write Lesson 3 "What's Next" section with congratulations, Chapter 2 preview, link to chapter project in lesson-03-acting-on-information.md
- [ ] T081 [P] [US1] Create circuit diagram showing complete system (sensor + actuator + transistor + safety diode) with clear labels in physical-ai-book/static/assets/chapter-01/lesson-03/complete-circuit-diagram.png
- [ ] T082 [P] [US1] Create PWM duty cycle diagram (visual representation of 25%, 50%, 75%, 100% duty cycles) in physical-ai-book/static/assets/chapter-01/lesson-03/pwm-duty-cycle.png
- [ ] T083 [P] [US1] Create photo of working project (fan running, temperature sensor visible) in physical-ai-book/static/assets/chapter-01/lesson-03/working-project-photo.jpg
- [ ] T084 [P] [US1] Create safety callout diagram showing correct diode placement for back-EMF protection in physical-ai-book/static/assets/chapter-01/lesson-03/safety-diode-placement.png

### Implementation for User Story 1 - Chapter Project & Glossary

- [ ] T085 [US1] Create chapter-project.md with frontmatter (id, title, sidebar_label, sidebar_position: 4, description) in physical-ai-book/docs/chapter-01/chapter-project.md
- [ ] T086 [US1] Write Chapter Project overview (temperature-responsive fan controller combining Lessons 2-3) with learning objectives in chapter-project.md
- [ ] T087 [US1] Write Chapter Project features section (hysteresis, data logging, manual override button) with code walkthrough in chapter-project.md
- [ ] T088 [US1] Write Chapter Project complete wiring diagram description and troubleshooting guide in chapter-project.md
- [ ] T089 [US1] Write Chapter Project extension ideas section (challenge readers with advanced features) in chapter-project.md
- [ ] T090 [P] [US1] Create complete project wiring diagram (all components integrated) in physical-ai-book/static/assets/chapter-01/lesson-03/chapter-project-wiring.png
- [ ] T091 [US1] Create glossary.md with frontmatter and minimum 20 terms from Chapter 1 (physical AI, embodied AI, sense-think-act loop, sensor, actuator, analog, digital, ADC, GPIO, PWM, microcontroller, breadboard, jumper wires, transistor, diode, Python, library, serial communication, threshold, hysteresis) in physical-ai-book/docs/glossary.md
- [ ] T092 [US1] Add backlinks from glossary terms to first usage locations in lessons (using Docusaurus link syntax) in glossary.md
- [ ] T093 [US1] Add cross-references between related terms in glossary in glossary.md

### Implementation for User Story 1 - Homepage & Site Polish

- [ ] T094 [US1] Create intro.md homepage with introduction, target audience, Chapter 1 overview using LessonCard components, getting started guide in physical-ai-book/docs/intro.md
- [ ] T095 [US1] Write hardware shopping list section with links to all required components for Chapter 1 (<$50 total) in intro.md
- [ ] T096 [US1] Write Python setup instructions with PlatformTabs for Windows/macOS/Linux in intro.md
- [ ] T097 [US1] Add contribution guidelines link and project overview in intro.md
- [ ] T098 [P] [US1] Create or source brand assets: logo.svg, favicon.ico, icon-192.png, icon-512.png for PWA in physical-ai-book/static/img/
- [ ] T099 [P] [US1] Optimize all images in static/assets/chapter-01/ to <500KB each using WebP or optimized PNG/JPG
- [ ] T100 [P] [US1] Add descriptive alt text to all images in all lesson markdown files
- [ ] T101 [US1] Run accessibility audit using WAVE or axe DevTools and fix any WCAG 2.1 AA violations
- [ ] T102 [US1] Test mobile responsiveness on tablet and phone (Chrome DevTools device emulation) and fix layout issues
- [ ] T103 [US1] Run Lighthouse performance audit and ensure score >90 and page load <3 seconds
- [ ] T104 [US1] Validate all internal links (lesson navigation, glossary backlinks, code download links) using link checker
- [ ] T105 [US1] Test cross-browser compatibility (Chrome, Firefox, Safari) and fix rendering issues

**Checkpoint**: At this point, User Story 1 should be fully functional - beginner readers can complete Chapter 1 end-to-end

---

## Phase 4: User Story 2 - Content Author Creates New Lesson Following Standards (Priority: P2)

**Goal**: Enable content authors to create consistent, high-quality lessons using templates and guidelines

**Independent Test**: Have an author create a sample lesson following guidelines, then validate against quality checklist (all required sections present, code tested, timing accurate, screenshots included)

### Implementation for User Story 2

- [ ] T106 [P] [US2] Create author-guidelines.md with writing style guide (tone: friendly/encouraging, active voice, "you" addressing, analogies, breaking up text) in physical-ai-book/templates/author-guidelines.md
- [ ] T107 [P] [US2] Write content quality standards in author-guidelines.md (code fully commented, error handling, PEP 8, cross-platform, asset optimization)
- [ ] T108 [P] [US2] Write visual design standards in author-guidelines.md (consistent colors: hardware=blue/software=green/theory=purple, simple icons, clear labels, high contrast)
- [ ] T109 [P] [US2] Write accessibility requirements in author-guidelines.md (alt text mandatory, keyboard navigation, WCAG 2.1 AA, semantic HTML)
- [ ] T110 [P] [US2] Write Docusaurus component usage guide in author-guidelines.md (tabs for alternatives, admonitions for tips/warnings, code blocks with titles, details for troubleshooting)
- [ ] T111 [US2] Create quickstart.md author getting started guide with local dev setup (npm install, npm start), preview workflow in physical-ai-book/templates/quickstart.md
- [ ] T112 [US2] Write frontmatter field reference in quickstart.md (id, title, sidebar_label, sidebar_position, description, keywords with examples)
- [ ] T113 [US2] Write asset organization guide in quickstart.md (where to put images: static/assets/chapter-N/lesson-N/, code files: static/code-examples/)
- [ ] T114 [US2] Write lesson creation workflow in quickstart.md (copy template, fill frontmatter, write content, add assets, test locally, validate checklist)
- [ ] T115 [US2] Enhance lesson-template.mdx with inline comments explaining each section purpose and requirements in physical-ai-book/templates/lesson-template.mdx
- [ ] T116 [US2] Add example frontmatter with placeholder values and comments in lesson-template.mdx
- [ ] T117 [US2] Add example code block with proper syntax (title, showLineNumbers, language, comments) in lesson-template.mdx
- [ ] T118 [US2] Add example Tabs usage for hardware alternatives and platform-specific instructions in lesson-template.mdx
- [ ] T119 [US2] Add example admonitions (:::info, :::tip, :::warning, :::danger) with use cases in lesson-template.mdx
- [ ] T120 [US2] Add example details component for troubleshooting scenarios in lesson-template.mdx
- [ ] T121 [US2] Add example CodeDownload component usage at end of lesson in lesson-template.mdx
- [ ] T122 [P] [US2] Create content quality checklist markdown file with all required sections, code standards, asset requirements, accessibility criteria in physical-ai-book/templates/content-quality-checklist.md
- [ ] T123 [P] [US2] Create CONTRIBUTING.md with contribution workflow (fork, branch, write lesson, test, PR, review process) in physical-ai-book/.github/CONTRIBUTING.md
- [ ] T124 [P] [US2] Add code of conduct section in CONTRIBUTING.md
- [ ] T125 [P] [US2] Add acknowledgments policy (how contributors are credited) in CONTRIBUTING.md

**Checkpoint**: At this point, User Story 2 should be complete - authors can create lessons following clear templates and guidelines

---

## Phase 5: User Story 3 - Educator Navigates Book Structure for Curriculum Planning (Priority: P3)

**Goal**: Enable educators to quickly understand learning progression, extract lesson subsets, and plan curriculum based on metadata

**Independent Test**: Show an educator the table of contents with metadata and ask them to plan a 4-week curriculum. Success = quickly identify relevant sections and understand dependencies

### Implementation for User Story 3

- [ ] T126 [US3] Update chapter-01/index.md to include comprehensive chapter-level metadata (total time: 3 hours, prerequisites: basic Python, difficulty: beginner, topics covered) in physical-ai-book/docs/chapter-01/index.md
- [ ] T127 [US3] Add chapter learning path section to chapter-01/index.md showing lesson progression and dependencies (Lesson 1→2→3→Project) with estimated times
- [ ] T128 [US3] Add "For Educators" section in chapter-01/index.md with curriculum planning guidance (suggested class structure, homework assignments, assessment ideas)
- [ ] T129 [US3] Enhance intro.md homepage to include educator-focused overview section with course planning use cases in physical-ai-book/docs/intro.md
- [ ] T130 [US3] Add curriculum planning examples in intro.md (4-week course structure, 8-week course structure, workshop format)
- [ ] T131 [US3] Create educator resources section in intro.md with links to all lesson metadata, downloadable syllabus template, assessment rubrics
- [ ] T132 [US3] Ensure all lesson frontmatter includes educator-relevant metadata (difficulty, prerequisites, time breakdown, hardware list) - validate in all 3 lesson files
- [ ] T133 [US3] Add prerequisite checking to each lesson with prominent callout linking to required background lessons (verify in lesson-02 and lesson-03)
- [ ] T134 [US3] Update sidebar configuration in sidebars.js to show lesson metadata on hover or in collapsed state (time, difficulty indicators)
- [ ] T135 [P] [US3] Create visual learning path diagram showing Chapter 1 progression (Lesson 1→2→3→Project) with time estimates in physical-ai-book/static/assets/learning-path-chapter-01.png
- [ ] T136 [P] [US3] Create downloadable syllabus template PDF for educators in physical-ai-book/static/downloads/syllabus-template-chapter-01.pdf

**Checkpoint**: At this point, User Story 3 should be complete - educators can navigate structure and plan curriculum effectively

---

## Phase 6: CI/CD & Deployment Infrastructure

**Purpose**: Automate code testing, performance validation, and deployment

- [ ] T137 [P] Create GitHub Actions workflow for Python code example testing (test-code-examples.yml) with matrix for Python 3.9/3.10/3.11 × Windows/macOS/Linux in physical-ai-book/.github/workflows/test-code-examples.yml
- [ ] T138 [P] Write pytest test for temperature_reader.py (validate imports, test error handling, mock sensor reads) in physical-ai-book/tests/code-examples/test_lesson_02.py
- [ ] T139 [P] Write pytest test for reactive_controller.py (validate imports, test threshold logic, mock sensor/actuator) in physical-ai-book/tests/code-examples/test_lesson_03.py
- [ ] T140 [P] Create GitHub Actions workflow for Lighthouse CI (lighthouse.yml) to enforce performance budget (>90 score, <3s load, <5MB bundle) in physical-ai-book/.github/workflows/lighthouse.yml
- [ ] T141 [P] Create GitHub Actions workflow for deployment (deploy.yml) to Netlify/Vercel/GitHub Pages with build command `npm run build` in physical-ai-book/.github/workflows/deploy.yml
- [ ] T142 [P] Create issue templates: bug_report.md for code errors in physical-ai-book/.github/ISSUE_TEMPLATE/bug_report.md
- [ ] T143 [P] Create issue template: content_error.md for lesson mistakes/typos in physical-ai-book/.github/ISSUE_TEMPLATE/content_error.md
- [ ] T144 [P] Create issue template: feature_request.md for new content suggestions in physical-ai-book/.github/ISSUE_TEMPLATE/feature_request.md

---

## Phase 7: Quality Assurance & Launch Preparation

**Purpose**: Final testing, validation, and deployment readiness

- [ ] T145 Recruit 3 beginner testers (basic Python, no AI experience) and provide hardware kits for Chapter 1
- [ ] T146 Conduct beginner testing sessions (testers complete Chapter 1 independently, record time, errors, feedback)
- [ ] T147 Collect beginner tester feedback via structured survey (success rate, comprehension, suggestions)
- [ ] T148 Analyze beginner testing results and identify content improvements needed (unclear instructions, missing steps, broken code)
- [ ] T149 Iterate on lesson content based on beginner feedback (simplify instructions, add troubleshooting, update code comments)
- [ ] T150 Re-test updated content with subset of beginner testers if major changes made
- [ ] T151 Recruit 2 technical reviewers (AI/robotics experts) for accuracy validation
- [ ] T152 Technical reviewers validate Chapter 1 content (concept accuracy, code quality, hardware recommendations, safety warnings)
- [ ] T153 Address all critical technical review feedback (fix inaccuracies, improve code, enhance safety warnings)
- [ ] T154 Complete pre-launch infrastructure checklist: Docusaurus builds without errors, all CI/CD workflows passing, PWA functional offline, search configured
- [ ] T155 Complete pre-launch content checklist: all 3 lessons complete with required sections, chapter project complete, glossary with 20+ terms, all code tested on 3 platforms
- [ ] T156 Complete pre-launch quality checklist: beginner testing 95% success rate, technical review approved, accessibility audit passed (WCAG 2.1 AA), performance audit passed (Lighthouse >90)
- [ ] T157 Complete pre-launch community checklist: contribution guidelines published, issue templates configured, code of conduct added, license files added (content: CC BY-SA 4.0, code: MIT)
- [ ] T158 Validate all success criteria from spec: beginners complete Chapter 1 in <3 hours, 95% code success rate, 70% exercise completion, page load <3 seconds, zero broken links
- [ ] T159 Configure deployment platform (Netlify/Vercel/GitHub Pages) with custom domain, HTTPS, build settings
- [ ] T160 Deploy to production and verify all features work (navigation, offline PWA, search, code downloads, responsive design)
- [ ] T161 Submit sitemap to Google Search Console for search indexing
- [ ] T162 Configure uptime monitoring (UptimeRobot or Pingdom) for production site
- [ ] T163 Apply for Algolia DocSearch (if using Algolia) and configure search indexing

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements affecting multiple user stories

- [ ] T164 [P] Write README.md for project root with project overview, local development setup, contribution link, license info in physical-ai-book/README.md
- [ ] T165 [P] Create LICENSE files: LICENSE-CONTENT (CC BY-SA 4.0) and LICENSE-CODE (MIT) in physical-ai-book/
- [ ] T166 [P] Add changelog.md with version history and content update tracking in physical-ai-book/CHANGELOG.md
- [ ] T167 [P] Create Jest tests for React components (LessonCard.test.tsx, HardwareList.test.tsx, CodeDownload.test.tsx) in physical-ai-book/tests/components/
- [ ] T168 Run bundle analyzer to identify optimization opportunities and reduce bundle size if needed
- [ ] T169 Implement lazy loading for images in lessons (Docusaurus Ideal Image plugin usage validation)
- [ ] T170 Add error boundaries to custom React components for graceful error handling
- [ ] T171 Review and optimize custom.css for performance (remove unused styles, minimize specificity)
- [ ] T172 Configure robots.txt and sitemap.xml for SEO in physical-ai-book/static/
- [ ] T173 Add Open Graph meta tags for social media sharing in docusaurus.config.js
- [ ] T174 Create social media preview image (1200×630px) in physical-ai-book/static/img/social-card.png
- [ ] T175 Validate HTML semantics (proper heading hierarchy, ARIA labels where needed) across all pages
- [ ] T176 Final cross-browser testing (Chrome, Firefox, Safari, Edge) on desktop and mobile
- [ ] T177 Final performance testing on slow network (3G simulation) to validate <3s load time
- [ ] T178 Document quarterly maintenance tasks (dependency updates, link checking, asset freshness, community feedback review) in physical-ai-book/MAINTENANCE.md

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion (T001-T008) - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion (T009-T023)
  - User Story 1 (Phase 3): Can start after T023 - No dependencies on other stories (MVP FOCUS)
  - User Story 2 (Phase 4): Can start after T023 - Independent of US1, but references US1 lessons as examples
  - User Story 3 (Phase 5): Can start after T023 - Depends on US1 lessons existing (T030-T105)
- **CI/CD (Phase 6)**: Can run in parallel with User Stories after Foundational - Depends on code examples from US1 existing
- **QA (Phase 7)**: Depends on User Story 1 completion (T030-T105) for beginner testing
- **Polish (Phase 8)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1 - Reader Success)**: INDEPENDENT after Foundational - Can complete and test fully on its own
- **User Story 2 (P2 - Author Standards)**: Mostly independent - References US1 lessons as examples but can be developed in parallel
- **User Story 3 (P3 - Educator Navigation)**: Depends on US1 lessons existing to enhance with metadata

### Within Each User Story

**User Story 1 (Reader Success)**:
1. Foundation tasks (T024-T029: templates, components) can run in parallel
2. Lesson 1 content (T030-T043) mostly sequential, diagrams (T041-T043) parallel
3. Lesson 2 content (T044-T063) mostly sequential, code (T053-T054) and assets (T060-T063) parallel
4. Lesson 3 content (T064-T084) mostly sequential, code (T074-T075) and assets (T081-T084) parallel
5. Chapter project and glossary (T085-T093) mostly sequential
6. Homepage and polish (T094-T105) many tasks can run in parallel (T098-T100)

**User Story 2 (Author Standards)**:
- All tasks (T106-T125) can largely run in parallel as they're creating different template files

**User Story 3 (Educator Navigation)**:
- Most tasks (T126-T136) sequential as they enhance existing US1 content

### Parallel Opportunities

- **Setup (Phase 1)**: T003-T008 all parallel (different directories)
- **Foundational (Phase 2)**: T021-T022 parallel (CI/CD setup separate from Docusaurus config)
- **US1 Foundation**: T024-T029 all parallel (different component files)
- **US1 Lesson 1**: T041-T043 parallel (different diagrams)
- **US1 Lesson 2**: T053-T054 parallel, T060-T063 parallel (code vs assets)
- **US1 Lesson 3**: T074-T075 parallel, T081-T084 parallel (code vs assets)
- **US1 Polish**: T098-T100, T101-T105 can run concurrently
- **US2 All tasks**: T106-T125 highly parallelizable (different template files)
- **US3 Diagrams**: T135-T136 parallel
- **CI/CD (Phase 6)**: T137-T144 all parallel (different workflow files, templates)
- **Polish (Phase 8)**: T164-T167 parallel (different docs), T168-T178 various parallel opportunities

### Critical Path

1. T001-T008 (Setup) → 2. T009-T023 (Foundational) → 3. T024-T105 (User Story 1) → 4. T145-T163 (QA & Launch)

This is the minimum path to MVP. User Stories 2 and 3 can be added later without breaking User Story 1.

---

## Parallel Example: User Story 1 - Lesson 2 Assets

```bash
# Launch all Lesson 2 asset creation tasks together:
Task: "T060 [P] [US1] Create color-coded breadboard wiring diagram for DHT22"
Task: "T061 [P] [US1] Create alternative wiring diagrams for DS18B20 and TMP36"
Task: "T062 [P] [US1] Create photo of actual hardware setup"
Task: "T063 [P] [US1] Create screenshot of expected terminal output"

# These can all be created simultaneously by different team members or in parallel
```

## Parallel Example: User Story 2 - Templates

```bash
# Launch all template creation tasks together:
Task: "T106 [P] [US2] Create author-guidelines.md with writing style guide"
Task: "T107 [P] [US2] Write content quality standards in author-guidelines.md"
Task: "T108 [P] [US2] Write visual design standards in author-guidelines.md"
Task: "T109 [P] [US2] Write accessibility requirements in author-guidelines.md"
Task: "T110 [P] [US2] Write Docusaurus component usage guide"

# All template sections can be written in parallel
```

---

## Implementation Strategy

### MVP First (User Story 1 Only) 🎯

**This is the recommended starting approach**

1. ✅ Complete Phase 1: Setup (T001-T008) - ~1-2 days
2. ✅ Complete Phase 2: Foundational (T009-T023) - ~3-5 days
3. ✅ Complete Phase 3: User Story 1 (T024-T105) - ~4-6 weeks
   - Foundation (T024-T029): 2-3 days
   - Lesson 1 (T030-T043): 3-4 days
   - Lesson 2 (T044-T063): 4-5 days
   - Lesson 3 (T064-T084): 5-6 days
   - Chapter Project (T085-T093): 3-4 days
   - Homepage & Polish (T094-T105): 2-3 days
4. ✅ Complete Phase 7: QA for US1 (T145-T163) - ~3-4 weeks
5. **STOP and VALIDATE**: Test User Story 1 independently with 3 beginner testers
6. Deploy MVP to production
7. Gather feedback before proceeding to User Stories 2-3

**MVP Success Criteria**:
- Beginners complete Chapter 1 in <3 hours
- 95% of code examples run successfully
- 70% of exercises completed
- Page load <3 seconds on 3G
- Lighthouse score >90

### Incremental Delivery (After MVP)

1. ✅ MVP deployed (User Story 1 complete)
2. Add User Story 2 (T106-T125) → Test independently → Deploy (~1-2 weeks)
   - Enables content authors to create consistent lessons
3. Add User Story 3 (T126-T136) → Test independently → Deploy (~1 week)
   - Enables educators to plan curriculum
4. Add CI/CD (Phase 6: T137-T144) → Automate quality checks (~3-5 days)
5. Add Polish (Phase 8: T164-T178) → Final improvements (~1 week)

Each increment adds value without breaking previous functionality.

### Parallel Team Strategy (If Multiple Developers Available)

**After Foundational phase (T023) completes:**

- **Developer A**: User Story 1 (T024-T105) - 4-6 weeks
- **Developer B**: User Story 2 (T106-T125) - 1-2 weeks, then help with US1 or CI/CD
- **Developer C**: CI/CD setup (T137-T144) - 3-5 days, then help with US1

**Optimal parallelization within US1**:
- One developer on Lesson 1 content
- One developer on Lesson 2 content
- One developer on components and templates
- Asset creation (diagrams, photos) can be outsourced or done in parallel

---

## Task Summary

**Total Tasks**: 178

**By Phase**:
- Phase 1 (Setup): 8 tasks
- Phase 2 (Foundational): 15 tasks
- Phase 3 (User Story 1 - Reader): 82 tasks (MVP FOCUS)
- Phase 4 (User Story 2 - Author): 20 tasks
- Phase 5 (User Story 3 - Educator): 11 tasks
- Phase 6 (CI/CD): 8 tasks
- Phase 7 (QA & Launch): 19 tasks
- Phase 8 (Polish): 15 tasks

**By User Story**:
- US1 (Reader Completes Chapter): 82 tasks
- US2 (Author Creates Lesson): 20 tasks
- US3 (Educator Plans Curriculum): 11 tasks
- Infrastructure/Shared: 65 tasks

**Parallel Tasks Identified**: 67 tasks marked with [P] can run in parallel within their phase

**MVP Scope** (Recommended first delivery):
- Phase 1 (T001-T008): 8 tasks
- Phase 2 (T009-T023): 15 tasks
- Phase 3 (T024-T105): 82 tasks
- Phase 7 (T145-T163): 19 tasks
- **Total MVP: 124 tasks (~10-14 weeks)**

**Post-MVP Additions**:
- User Story 2: 20 tasks (~1-2 weeks)
- User Story 3: 11 tasks (~1 week)
- CI/CD: 8 tasks (~3-5 days)
- Polish: 15 tasks (~1 week)

---

## Validation Checklist

Before considering each phase complete:

**Phase 1 (Setup)**: ✓ Directory structure exists, npm packages installed, local server runs

**Phase 2 (Foundational)**: ✓ Docusaurus config complete, navigation works, PWA manifest present, custom CSS applied, sidebars configured

**Phase 3 (User Story 1)**: ✓ All 3 lessons complete with required sections, code examples tested, assets optimized, glossary with 20+ terms, homepage with lesson cards, performance >90

**Phase 4 (User Story 2)**: ✓ Templates documented, author guidelines complete, quickstart guide functional, contribution workflow defined

**Phase 5 (User Story 3)**: ✓ Chapter metadata complete, learning path visible, educator resources available, prerequisite checking functional

**Phase 6 (CI/CD)**: ✓ All workflows passing, code tests green, Lighthouse CI enforcing budget, deployment pipeline functional

**Phase 7 (QA)**: ✓ 3 beginner testers completed Chapter 1, 95% success rate, technical review approved, all checklists complete

**Phase 8 (Polish)**: ✓ README complete, licenses added, SEO configured, accessibility validated, cross-browser tested

---

## Notes

- **[P] marker**: Tasks marked with [P] operate on different files or have no dependencies, enabling parallel execution
- **[Story] label**: Maps each task to its user story (US1, US2, US3) for traceability and independent testing
- **MVP Focus**: User Story 1 (Reader Success) is the critical path - delivers complete value independently
- **File paths**: All paths assume Docusaurus project root at `physical-ai-book/` - adjust if different
- **Asset creation**: Diagram and photo tasks (marked [P]) can be outsourced to designers/photographers
- **Testing**: No test tasks for content (per spec) - validation through beginner testing and technical review
- **Incremental value**: Each user story delivers independently testable value
- **Constitutional alignment**: All tasks support accessibility, hands-on learning, progressive complexity, production-ready examples, clear documentation, and community improvement

**Next Steps**: Begin with T001 (Setup) and proceed through Foundational phase before starting User Story 1 implementation.
