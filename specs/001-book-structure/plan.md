# Implementation Plan: Physical AI Book in Docusaurus

**Branch**: `001-book-structure` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-book-structure/spec.md` + User request for development plan with Docusaurus setup, content development phases, and file structure

## Summary

Build a beginner-friendly Physical AI book using Docusaurus 3.x as a static site generator. The book will feature Chapter 1 "Foundations of Physical AI" with 3 progressive lessons teaching the sense-think-act loop through hands-on tutorials. The implementation focuses on creating an accessible, mobile-responsive documentation site with offline PWA support, optimized for learners with basic Python knowledge but no AI experience.

**Technical Approach**:
- Docusaurus 3.x for content delivery with MDX support
- Static site generation for fast loading (<3 seconds per page)
- PWA configuration for offline lesson access
- Custom React components for lesson cards and interactive elements
- Algolia DocSearch for site-wide search
- GitHub Actions CI/CD for automated code example testing

**Content Development Phases**:
1. Phase 1: Docusaurus infrastructure setup and configuration
2. Phase 2: Content templates and authoring guidelines
3. Phase 3: Chapter 1 lesson development (3 lessons)
4. Phase 4: Code examples, assets, and downloadables
5. Phase 5: Quality assurance and beginner testing

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+), Python 3.9+ (for code examples)
**Primary Dependencies**:
- Docusaurus 3.x (@docusaurus/core, @docusaurus/preset-classic)
- React 18+
- MDX for enhanced markdown
- @docusaurus/plugin-pwa for offline support
- Algolia DocSearch or @docusaurus/plugin-search-local
**Storage**: Static files (Markdown, images, code examples); Git for version control
**Testing**:
- Python: pytest for code example validation
- JavaScript: Jest for React components
- Lighthouse for performance/accessibility audits
- Manual beginner testing per constitution
**Target Platform**: Web (desktop + mobile browsers), PWA-capable, cross-platform (Windows/macOS/Linux for development)
**Project Type**: Static documentation website (JAMstack architecture)
**Performance Goals**:
- Page load < 3 seconds on 3G connections
- Lighthouse performance score > 90
- Time to Interactive (TTI) < 5 seconds
- Bundle size < 5MB per page
**Constraints**:
- Mobile-responsive (readable on tablets/phones)
- Offline-capable (PWA features for lesson content)
- Cross-platform code examples (Windows/macOS/Linux)
- Accessible (WCAG 2.1 AA minimum)
- Asset optimization (images <500KB each)
**Scale/Scope**:
- MVP: 1 chapter, 3 lessons, ~10-15 content pages
- Future: Expandable to multiple chapters without URL changes
- Target: 300-400 pages (digital equivalent) at completion
- Assets: ~50-100 images/diagrams, ~20-30 code examples for Chapter 1

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Accessibility First
- ✅ **Pass**: Docusaurus supports WCAG accessibility out-of-box; progressive lesson structure (conceptual → hands-on); frontmatter for prerequisites; glossary with backlinks
- ✅ **Pass**: MDX allows inline definitions, code comments, visual diagrams; tabs for platform-specific instructions

### II. Hands-On Learning Priority
- ✅ **Pass**: Each lesson includes runnable code examples stored in `/static/code-examples/`; downloadable files; exercises with starter code
- ✅ **Pass**: Troubleshooting sections use Docusaurus `<details>` component; expected outputs shown in separate code blocks

### III. Progressive Complexity
- ✅ **Pass**: Chapter-based navigation with clear prerequisite chains; sidebar shows learning path; optional "Deep Dive" sections using admonitions
- ✅ **Pass**: URL structure supports expansion: `/docs/chapter-[N]/lesson-[N]-[slug]`

### IV. Production-Ready Examples
- ✅ **Pass**: CI/CD pipeline tests all Python code on Windows/macOS/Linux with pytest; code examples include error handling per spec FR-006
- ✅ **Pass**: Hardware requirements documented with alternatives; requirements.txt pins exact library versions

### V. Clear Documentation Standards
- ✅ **Pass**: Frontmatter enforces learning objectives, time estimates, prerequisites per FR-021; lesson template mandates all required sections
- ✅ **Pass**: Docusaurus displays metadata prominently; custom components render lesson cards with time/difficulty indicators

### VI. Community-Driven Improvement
- ✅ **Pass**: GitHub integration for issue tracking; Docusaurus `editUrl` for "Edit this page" links; contribution guidelines in docs
- ✅ **Pass**: Version tracking via Git; changelog automated; contributor acknowledgments in footer

### Technical Constraints
- ✅ **Pass**: Docusaurus 3.x specified; bundle size monitored with webpack-bundle-analyzer plugin
- ✅ **Pass**: PWA plugin configured for offline; mobile-responsive via Docusaurus default theme; code highlighting for Python/C++/JavaScript

### Content Constraints
- ✅ **Pass**: Python examples assume basic programming only; hardware <$200 per spec; open-source libraries (MIT/Apache licenses)
- ✅ **Pass**: Cross-platform tabs for Windows/macOS/Linux; platform-agnostic code paths

**Result**: All constitutional principles satisfied. No violations requiring justification.

## Project Structure

### Documentation (this feature)

```text
specs/001-book-structure/
├── plan.md              # This file (/sp.plan output)
├── spec.md              # Feature specification
├── research.md          # Phase 0: Docusaurus best practices, PWA setup, content strategy
├── data-model.md        # Phase 1: Content entities (Chapter, Lesson, CodeExample, Exercise, etc.)
├── quickstart.md        # Phase 1: Getting started guide for content authors
├── contracts/           # Phase 1: Content schemas (frontmatter, lesson template JSON schema)
│   ├── frontmatter-schema.json
│   ├── lesson-structure.json
│   └── code-example-schema.json
├── checklists/
│   └── requirements.md  # Spec quality validation
└── tasks.md             # Phase 2: Generated by /sp.tasks (NOT created by /sp.plan)
```

### Source Code (repository root - Docusaurus project)

```text
physical-ai-book/           # Docusaurus project root
├── docs/                   # Content (markdown/MDX files)
│   ├── intro.md           # Homepage/landing page
│   ├── chapter-01/        # Chapter 1: Foundations of Physical AI
│   │   ├── _category_.json          # Chapter metadata (label, position, collapsed)
│   │   ├── index.md                 # Chapter overview page
│   │   ├── lesson-01-understanding-physical-ai.md
│   │   ├── lesson-02-sensing-the-world.md
│   │   ├── lesson-03-acting-on-information.md
│   │   └── chapter-project.md       # Hands-on chapter project
│   └── glossary.md        # Glossary with term definitions
│
├── static/                # Static assets (not processed by webpack)
│   ├── assets/           # Images, diagrams, videos
│   │   └── chapter-01/
│   │       ├── lesson-01/  # Diagrams for Lesson 1 (sense-think-act visuals)
│   │       ├── lesson-02/  # Wiring diagrams, sensor photos
│   │       └── lesson-03/  # Circuit diagrams, actuator photos
│   ├── code-examples/    # Downloadable code files
│   │   └── chapter-01/
│   │       ├── lesson-02/
│   │       │   ├── temperature_reader.py
│   │       │   └── requirements.txt
│   │       └── lesson-03/
│   │           ├── reactive_controller.py
│   │           └── requirements.txt
│   └── img/              # Brand assets (logo, icons, favicon)
│       ├── logo.svg
│       ├── favicon.ico
│       └── icon.png      # PWA icon
│
├── src/                  # Custom React components and pages
│   ├── components/       # Reusable components
│   │   ├── LessonCard.tsx         # Displays lesson metadata (time, difficulty)
│   │   ├── HardwareList.tsx       # Renders hardware requirements with pricing
│   │   ├── CodeDownload.tsx       # Download button for code examples
│   │   └── PlatformTabs.tsx       # Custom wrapper for OS-specific instructions
│   ├── pages/            # Custom pages (if needed beyond docs)
│   │   └── index.tsx     # Custom homepage (optional, or use docs/intro.md)
│   └── css/              # Custom styling
│       └── custom.css    # Brand colors, code block styles, admonition themes
│
├── sidebars.js           # Sidebar navigation configuration
├── docusaurus.config.js  # Main Docusaurus configuration
├── package.json          # Node.js dependencies
├── tsconfig.json         # TypeScript configuration
├── babel.config.js       # Babel configuration
│
├── .github/              # CI/CD and community files
│   ├── workflows/
│   │   ├── test-code-examples.yml   # Pytest on code examples (Win/Mac/Linux)
│   │   ├── deploy.yml               # Deploy to Netlify/Vercel/GitHub Pages
│   │   └── lighthouse.yml           # Performance/accessibility audits
│   ├── ISSUE_TEMPLATE/
│   │   ├── bug_report.md
│   │   ├── content_error.md
│   │   └── feature_request.md
│   └── CONTRIBUTING.md   # Contribution guidelines for authors
│
├── templates/            # Content authoring templates
│   ├── lesson-template.mdx          # Standard lesson structure
│   ├── chapter-template.md          # Chapter overview template
│   └── author-guidelines.md         # Writing style guide
│
└── tests/                # Automated tests
    ├── code-examples/    # Pytest tests for Python code
    │   └── test_chapter_01.py
    └── components/       # Jest tests for React components
        └── LessonCard.test.tsx
```

**Structure Decision**: Selected static documentation site structure (Docusaurus standard). This is not a traditional web app (no backend/frontend split) but a JAMstack documentation site. All content is pre-rendered as static HTML for optimal performance. Custom React components enhance interactivity without requiring a backend.

**Rationale**:
- Docusaurus convention: `docs/` for content, `src/` for custom code, `static/` for assets
- Chapter-based organization in `docs/chapter-N/` enables scalable expansion per FR-004
- Assets organized by chapter and lesson for maintainability per FR-028
- Separation of concerns: content (MDX), presentation (React), configuration (JS)

## Complexity Tracking

No constitutional violations detected. This section is empty.

## Docusaurus Setup Steps

### Step 1: Initialize Docusaurus Project

```bash
# Install Node.js 18+ (if not already installed)
# Windows: Download from nodejs.org
# macOS: brew install node@18
# Linux: nvm install 18

# Create new Docusaurus project
npx create-docusaurus@latest physical-ai-book classic --typescript

cd physical-ai-book

# Install additional dependencies
npm install --save @docusaurus/plugin-pwa
npm install --save @docusaurus/plugin-ideal-image
npm install --save @docusaurus/theme-live-codeblock
npm install --save-dev webpack-bundle-analyzer
```

### Step 2: Configure docusaurus.config.js

```javascript
// @ts-check
const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI: A Hands-On Introduction',
  tagline: 'Learn to build intelligent systems that sense and act in the real world',
  favicon: 'img/favicon.ico',

  url: 'https://your-domain.com', // Update with actual domain
  baseUrl: '/',

  organizationName: 'your-org', // GitHub org/user name
  projectName: 'physical-ai-book', // GitHub repo name

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/your-org/physical-ai-book/edit/main/',
          showLastUpdateTime: true,
          breadcrumbs: true,
        },
        blog: false, // Disable blog for now
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.png', // Social media preview image
      navbar: {
        title: 'Physical AI',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'doc',
            docId: 'intro',
            position: 'left',
            label: 'Book',
          },
          {
            to: '/docs/glossary',
            label: 'Glossary',
            position: 'left',
          },
          {
            href: 'https://github.com/your-org/physical-ai-book',
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
              {
                label: 'Get Started',
                to: '/docs/intro',
              },
              {
                label: 'Chapter 1',
                to: '/docs/chapter-01',
              },
              {
                label: 'Glossary',
                to: '/docs/glossary',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Discussions',
                href: 'https://github.com/your-org/physical-ai-book/discussions',
              },
              {
                label: 'Report Issue',
                href: 'https://github.com/your-org/physical-ai-book/issues',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/physical-ai-book',
              },
              {
                label: 'License',
                href: '/docs/license',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Book. Content: CC BY-SA 4.0, Code: MIT`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'cpp', 'bash'],
      },
      algolia: {
        // Add after configuring Algolia DocSearch
        appId: 'YOUR_APP_ID',
        apiKey: 'YOUR_API_KEY',
        indexName: 'physical-ai',
        contextualSearch: true,
      },
    }),

  plugins: [
    [
      '@docusaurus/plugin-pwa',
      {
        debug: true,
        offlineModeActivationStrategies: [
          'appInstalled',
          'standalone',
          'queryString',
        ],
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
          {
            tagName: 'meta',
            name: 'theme-color',
            content: '#2e8555',
          },
          {
            tagName: 'meta',
            name: 'apple-mobile-web-app-capable',
            content: 'yes',
          },
          {
            tagName: 'meta',
            name: 'apple-mobile-web-app-status-bar-style',
            content: '#2e8555',
          },
        ],
      },
    ],
    [
      '@docusaurus/plugin-ideal-image',
      {
        quality: 85,
        max: 1200,
        min: 640,
        steps: 2,
        disableInDev: false,
      },
    ],
  ],
};

module.exports = config;
```

### Step 3: Configure sidebars.js

```javascript
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
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
      link: {
        type: 'doc',
        id: 'chapter-01/index',
      },
      items: [
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

module.exports = sidebars;
```

### Step 4: Customize src/css/custom.css

```css
:root {
  /* Primary brand color */
  --ifm-color-primary: #2e8555;
  --ifm-color-primary-dark: #29784c;
  --ifm-color-primary-darker: #277148;
  --ifm-color-primary-darkest: #205d3b;
  --ifm-color-primary-light: #33925d;
  --ifm-color-primary-lighter: #359962;
  --ifm-color-primary-lightest: #3cad6e;

  /* Code and content */
  --ifm-code-font-size: 95%;
  --ifm-font-size-base: 16px;
  --ifm-line-height-base: 1.65;

  /* Spacing */
  --ifm-spacing-horizontal: 1rem;
  --ifm-spacing-vertical: 1rem;
}

/* Dark mode overrides */
[data-theme='dark'] {
  --ifm-color-primary: #25c2a0;
  --ifm-color-primary-dark: #21af90;
  --ifm-color-primary-darker: #1fa588;
  --ifm-color-primary-darkest: #1a8870;
  --ifm-color-primary-light: #29d5b0;
  --ifm-color-primary-lighter: #32d8b4;
  --ifm-color-primary-lightest: #4fddbf;
}

/* High contrast for accessibility */
.markdown h1,
.markdown h2,
.markdown h3 {
  font-weight: 600;
  margin-top: 2rem;
  margin-bottom: 1rem;
}

/* Code block title styling */
div[class^='codeBlockTitle'] {
  background-color: var(--ifm-color-primary);
  color: white;
  font-weight: bold;
  padding: 0.5rem 1rem;
  border-radius: 0.5rem 0.5rem 0 0;
}

/* Consistent callout box styling */
.admonition {
  margin-bottom: 1.5rem;
  border-left-width: 4px;
}

/* Lesson card styling */
.lesson-card {
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  padding: 1.5rem;
  margin-bottom: 1rem;
  transition: transform 0.2s, box-shadow 0.2s;
}

.lesson-card:hover {
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
}

.lesson-card h3 {
  margin-top: 0;
  color: var(--ifm-color-primary);
}

.lesson-meta {
  display: flex;
  gap: 1rem;
  margin-top: 0.5rem;
  font-size: 0.9rem;
  color: var(--ifm-color-emphasis-600);
}

/* Hardware list table */
.hardware-table {
  width: 100%;
  border-collapse: collapse;
  margin: 1rem 0;
}

.hardware-table th {
  background-color: var(--ifm-color-emphasis-100);
  padding: 0.75rem;
  text-align: left;
}

.hardware-table td {
  padding: 0.75rem;
  border-bottom: 1px solid var(--ifm-color-emphasis-200);
}

/* Code download button */
.code-download-btn {
  display: inline-flex;
  align-items: center;
  gap: 0.5rem;
  background-color: var(--ifm-color-primary);
  color: white;
  padding: 0.5rem 1rem;
  border-radius: 0.5rem;
  text-decoration: none;
  font-weight: 500;
  transition: background-color 0.2s;
}

.code-download-btn:hover {
  background-color: var(--ifm-color-primary-dark);
  color: white;
  text-decoration: none;
}

/* Mobile responsiveness */
@media screen and (max-width: 768px) {
  .lesson-meta {
    flex-direction: column;
    gap: 0.5rem;
  }

  .hardware-table {
    font-size: 0.875rem;
  }
}
```

### Step 5: Create static/manifest.json (PWA)

```json
{
  "name": "Physical AI: A Hands-On Introduction",
  "short_name": "Physical AI",
  "description": "Learn to build intelligent systems that sense and act in the real world",
  "start_url": "/",
  "display": "standalone",
  "theme_color": "#2e8555",
  "background_color": "#ffffff",
  "icons": [
    {
      "src": "/img/icon-192.png",
      "sizes": "192x192",
      "type": "image/png"
    },
    {
      "src": "/img/icon-512.png",
      "sizes": "512x512",
      "type": "image/png"
    }
  ]
}
```

### Step 6: Test Local Development Server

```bash
# Start development server
npm start

# Should open http://localhost:3000
# Hot reload enabled for content changes
```

## Content Development Phases

### Phase 1: Infrastructure & Templates (Week 1-2)

**Deliverables**:
1. Docusaurus site running locally with custom branding
2. Lesson template (MDX) with all required sections
3. Custom React components (LessonCard, HardwareList, CodeDownload)
4. CI/CD workflows configured
5. Author quickstart guide

**Tasks**:
- Complete Docusaurus setup steps 1-6
- Create `templates/lesson-template.mdx`
- Build React components in `src/components/`
- Set up GitHub Actions workflows
- Write `quickstart.md` for authors

### Phase 2: Chapter 1 Infrastructure (Week 3)

**Deliverables**:
1. `docs/chapter-01/` directory structure
2. Chapter overview page (`index.md`)
3. Asset directories created
4. Navigation configured

**Tasks**:
- Create `docs/chapter-01/_category_.json`
- Write chapter overview with learning objectives
- Create `static/assets/chapter-01/lesson-{01,02,03}/` directories
- Update `sidebars.js` with chapter structure

### Phase 3: Lesson 1 - Understanding Physical AI (Week 3-4)

**Deliverables**:
1. Complete `lesson-01-understanding-physical-ai.md`
2. Sense-think-act loop diagram (SVG/PNG)
3. Physical AI vs Software AI comparison table
4. Glossary terms initialized

**Content Checklist**:
- [x] Frontmatter (id, title, sidebar_label, description, keywords)
- [x] Learning objectives admonition
- [x] Prerequisites section
- [x] Introduction with real-world hook
- [x] Sense-think-act loop explanation
- [x] Real-world examples (3+)
- [x] Comparison table (Physical AI vs Software AI)
- [x] Summary section
- [x] Exercises (identify components, brainstorm applications)
- [x] "What's Next" preview

**Asset Checklist**:
- [ ] Sense-think-act loop diagram (animated or static)
- [ ] Example photos (smart thermostat, robot vacuum, self-driving car)
- [ ] Comparison table (embedded or image)

### Phase 4: Lesson 2 - Sensing the World (Week 4-5)

**Deliverables**:
1. Complete `lesson-02-sensing-the-world.md`
2. Wiring diagrams (DHT22, DS18B20, TMP36)
3. Code example: `temperature_reader.py`
4. Platform-specific setup tabs
5. Troubleshooting guide

**Content Checklist**:
- [x] Frontmatter with hardware/software requirements
- [x] Learning objectives
- [x] Prerequisites (Lesson 1, Python basics)
- [x] Introduction (plant watering scenario)
- [x] Sensor basics (analog vs digital, ADC, GPIO)
- [x] Hardware setup with wiring diagram
- [x] Platform-specific tabs (Windows/macOS/Linux)
- [x] Software setup (pip install, testing)
- [x] Code tutorial with line-by-line explanation
- [x] Error handling section
- [x] Troubleshooting (3+ scenarios)
- [x] Summary
- [x] Exercises (modify code, add features)
- [x] "What's Next" preview

**Code Checklist**:
- [ ] `temperature_reader.py` (tested on Win/Mac/Linux)
- [ ] `requirements.txt` (pinned versions)
- [ ] Expected output example
- [ ] Error handling (sensor disconnect, bad reads)

**Asset Checklist**:
- [ ] Wiring diagram (color-coded breadboard view)
- [ ] Alternative diagrams (DS18B20, TMP36)
- [ ] Hardware photo (actual setup)
- [ ] Screenshot (terminal output)

### Phase 5: Lesson 3 - Acting on Information (Week 5-6)

**Deliverables**:
1. Complete `lesson-03-acting-on-information.md`
2. Circuit diagrams (sensor + actuator + transistor)
3. Code example: `reactive_controller.py`
4. PWM explanation diagram
5. Troubleshooting guide

**Content Checklist**:
- [x] Frontmatter with additional hardware requirements
- [x] Learning objectives (actuator control, logic, PWM)
- [x] Prerequisites (Lessons 1-2, if/else statements)
- [x] Introduction (closing the loop)
- [x] Actuator basics (types, power, safety)
- [x] Hardware setup (circuit diagram, transistor explanation)
- [x] Digital control (on/off)
- [x] PWM explanation with diagram
- [x] Logic implementation (threshold, hysteresis, proportional)
- [x] Complete system tutorial
- [x] Testing and debugging
- [x] Troubleshooting (3+ scenarios)
- [x] Summary
- [x] Exercises (add thresholds, hysteresis, override button)
- [x] "What's Next" (Chapter project preview)

**Code Checklist**:
- [ ] `reactive_controller.py` (tested on Win/Mac/Linux)
- [ ] PWM example code
- [ ] requirements.txt (additional libraries)
- [ ] Expected behavior description

**Asset Checklist**:
- [ ] Circuit diagram (complete system)
- [ ] PWM duty cycle diagram
- [ ] Hardware photo (working project)
- [ ] Safety callout image (diode placement)

### Phase 6: Chapter Project & Glossary (Week 6-7)

**Deliverables**:
1. Complete `chapter-project.md`
2. Expanded `glossary.md` with all Chapter 1 terms
3. Backlinks from glossary to lesson first usage

**Project Checklist**:
- [x] Project overview (temperature-responsive fan)
- [x] Complete code (combines Lessons 2-3)
- [x] Wiring diagram (full system)
- [x] Features (hysteresis, logging, manual override)
- [x] Troubleshooting guide
- [x] Extension ideas (challenge readers)

**Glossary Checklist**:
- [ ] 20+ terms from Chapter 1
- [ ] Definitions (clear, beginner-friendly)
- [ ] Backlinks to first usage
- [ ] Related terms cross-referenced
- [ ] Alphabetical organization

### Phase 7: Homepage & Site Polish (Week 7-8)

**Deliverables**:
1. Complete `intro.md` homepage
2. Lesson cards with metadata
3. Footer customization
4. PWA icons and favicon
5. Accessibility audit

**Homepage Checklist**:
- [x] Introduction paragraph
- [x] Target audience description
- [x] Chapter 1 overview
- [x] Lesson cards (time, difficulty, link)
- [x] Getting started guide
- [x] Hardware shopping list
- [x] Python setup instructions
- [x] Contribution link

**Polish Checklist**:
- [ ] Accessibility audit (WAVE, axe)
- [ ] Mobile testing (Chrome DevTools, real devices)
- [ ] Lighthouse performance (>90 score)
- [ ] Cross-browser testing (Chrome, Firefox, Safari)
- [ ] Link validation (all internal, download links)
- [ ] Image optimization (WebP, lazy loading)

### Phase 8: QA & Launch (Week 9-12)

**Deliverables**:
1. Beginner testing complete (3 testers)
2. Technical review complete (2 reviewers)
3. All feedback addressed
4. Pre-launch checklist complete
5. Deployed to production

**Testing Process**:
1. Recruit 3 beginners (basic Python, no AI)
2. Provide hardware kits
3. Testers complete Chapter 1 independently
4. Collect feedback (time, errors, suggestions)
5. Iterate based on feedback
6. Re-test if major changes

**Review Process**:
1. 2 technical reviewers validate accuracy
2. Submit detailed feedback
3. Address critical issues
4. Final approval for launch

## File Structure Detail

### Lesson File Template (lesson-02-sensing-the-world.md example)

```markdown
---
id: lesson-02-sensing-the-world
title: "Sensing the World: Reading Data from Physical Sensors"
sidebar_label: "Lesson 2: Sensing"
sidebar_position: 2
description: "Hands-on tutorial: Connect a temperature sensor and read data with Python. Learn analog vs digital sensors, error handling, and cross-platform code."
keywords: [physical ai, sensors, temperature, DHT22, python, hardware, raspberry pi pico, arduino]
---

# Sensing the World: Reading Data from Physical Sensors

:::info Learning Objectives
By the end of this lesson, you will be able to:
- Connect a temperature sensor to a computer via USB-serial or GPIO
- Write Python code to read sensor data at regular intervals
- Understand the difference between analog and digital sensors
- Handle sensor errors and missing data gracefully
- Display sensor readings in human-readable format
:::

## Prerequisites

- Completion of [Lesson 1: Understanding Physical AI](./lesson-01-understanding-physical-ai.md)
- Python basics (variables, loops, functions, print statements)
- Ability to run Python scripts from command line

## Estimated Time

- **Reading**: 15 minutes
- **Hands-on exercises**: 30 minutes
- **Total**: 45 minutes

## Required Hardware

import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

<Tabs groupId="hardware-option">
  <TabItem value="primary" label="Recommended (DHT22)">

    - **DHT22 Temperature/Humidity Sensor** - $10
      - Digital sensor, simple wiring, accurate readings
      - [Buy on Adafruit](https://www.adafruit.com/product/385)
    - **Raspberry Pi Pico** - $4
      - USB connection, easy setup, beginner-friendly
      - [Buy on Adafruit](https://www.adafruit.com/product/4864)
    - **Breadboard** - $5
    - **Jumper wires** (M-M, M-F) - $3
    - **USB cable** (usually included with Pico)

    **Total**: ~$22
  </TabItem>

  <TabItem value="alt1" label="Alternative: DS18B20">

    - **DS18B20 Waterproof Temperature Sensor** - $8
      - Digital, one-wire protocol, waterproof (good for liquids)
      - [Buy on Adafruit](https://www.adafruit.com/product/381)
    - Same microcontroller and accessories as above

  </TabItem>

  <TabItem value="alt2" label="Budget: TMP36">

    - **TMP36 Analog Temperature Sensor** - $2
      - Requires ADC (analog-to-digital converter) like MCP3008
      - Slightly more complex wiring
    - **MCP3008 ADC** - $3.75
    - Same microcontroller and accessories as above

  </TabItem>
</Tabs>

## Required Software

<Tabs groupId="operating-system">
  <TabItem value="windows" label="Windows">

    ```bash
    # Install Python 3.9+ from python.org
    # Install pip (usually included)

    pip install pyserial adafruit-circuitpython-dht
    ```

    **Driver**: Install CP2102 USB driver if using Pico

  </TabItem>

  <TabItem value="mac" label="macOS">

    ```bash
    # Install Python via Homebrew
    brew install python@3.11

    pip3 install pyserial adafruit-circuitpython-dht
    ```

    **Permissions**: May need to add user to `dialout` group for serial access

  </TabItem>

  <TabItem value="linux" label="Linux">

    ```bash
    # Python usually pre-installed
    # If not: sudo apt install python3 python3-pip

    pip3 install pyserial adafruit-circuitpython-dht

    # Add user to dialout group for serial port access
    sudo usermod -a -G dialout $USER
    # Log out and back in for changes to take effect
    ```

  </TabItem>
</Tabs>

## Introduction

Imagine you're building a smart plant watering system. To water plants at the right time, your system needs to know the soil moisture level. This requires a **sensor** - a device that measures physical properties (moisture, temperature, light, etc.) and converts them to electrical signals your computer can read.

In this lesson, you'll learn how to connect your first sensor and read data from it. We'll use a temperature sensor because it's simple, affordable, and teaches fundamental concepts you'll use with all sensors.

[Content continues with sensor basics, wiring, code examples, troubleshooting...]

## Code Example: Continuous Temperature Monitoring

```python title="temperature_reader.py" showLineNumbers
"""
Continuous temperature monitoring with DHT22 sensor.
Reads temperature and humidity every second and displays results.
"""

import time
import board
import adafruit_dht

# Initialize DHT22 sensor on GPIO pin 4
# board.D4 refers to digital pin 4 on Raspberry Pi Pico
dht_device = adafruit_dht.DHT22(board.D4, use_pulseio=False)

print("Temperature Monitor Started")
print("Press Ctrl+C to stop")
print("-" * 40)

try:
    while True:
        try:
            # Read temperature and humidity
            temperature_c = dht_device.temperature
            humidity = dht_device.humidity

            # Convert Celsius to Fahrenheit
            temperature_f = temperature_c * (9 / 5) + 32

            # Display readings
            print(f"Temp: {temperature_c:.1f}°C ({temperature_f:.1f}°F) | Humidity: {humidity:.1f}%")

        except RuntimeError as error:
            # DHT22 can occasionally fail to read; this is normal
            print(f"Reading error: {error.args[0]}")

        except Exception as error:
            # Unexpected error; stop and report
            dht_device.exit()
            raise error

        # Wait 1 second before next reading
        # DHT22 can only be read every 2 seconds, so this is safe
        time.sleep(1.0)

except KeyboardInterrupt:
    print("\nMonitoring stopped by user")
    dht_device.exit()
```

**Expected Output:**
```
Temperature Monitor Started
Press Ctrl+C to stop
----------------------------------------
Temp: 22.3°C (72.1°F) | Humidity: 45.2%
Temp: 22.4°C (72.3°F) | Humidity: 45.1%
Temp: 22.3°C (72.1°F) | Humidity: 45.3%
...
```

:::tip Pro Tip
DHT sensors can be sensitive to timing. If you get frequent read errors, increase the `time.sleep()` to 2 seconds. The sensor can only provide new data every 2 seconds anyway.
:::

:::warning Common Mistake
Make sure to connect the sensor to a GPIO pin with pull-up resistor capability. Some pins on microcontrollers don't support pull-ups, which DHT sensors need to function properly.
:::

[Content continues...]

## Download Code

<a href="/code-examples/chapter-01/lesson-02/temperature_reader.py" download className="code-download-btn">
  📥 Download temperature_reader.py (2.1 KB)
</a>

<a href="/code-examples/chapter-01/lesson-02/requirements.txt" download className="code-download-btn">
  📥 Download requirements.txt (0.1 KB)
</a>

[Rest of lesson content: troubleshooting, summary, exercises, what's next...]
```

## Success Metrics

Track these metrics to validate implementation success:

### Technical Performance
- [ ] Page load < 3 seconds (Lighthouse CI)
- [ ] Lighthouse performance score > 90
- [ ] Bundle size < 5MB per page
- [ ] Zero broken links (automated checker)
- [ ] PWA installable and functional offline

### Code Quality
- [ ] All Python code passes pytest on Win/Mac/Linux
- [ ] Code examples follow PEP 8
- [ ] Error handling in all examples
- [ ] Cross-platform compatibility verified

### Content Quality
- [ ] All lessons have required sections per template
- [ ] Frontmatter complete and valid
- [ ] Glossary has 20+ terms with backlinks
- [ ] Images optimized (<500KB each)
- [ ] Alt text on all images

### User Success (from Constitution)
- [ ] 95% code example success rate (beginner testing)
- [ ] 70% exercise completion (beginner testing)
- [ ] <3 hour Chapter 1 completion (beginner testing)
- [ ] 90% comprehension (post-chapter quiz)

## Timeline Summary

**Phase 0 (Research)**: 1 week
**Phase 1 (Infrastructure)**: 2 weeks
**Phase 2 (Chapter Setup)**: 3 days
**Phase 3 (Lesson 1)**: 3-4 days
**Phase 4 (Lesson 2)**: 4-5 days
**Phase 5 (Lesson 3)**: 5-6 days
**Phase 6 (Project & Glossary)**: 4-5 days
**Phase 7 (Homepage & Polish)**: 5-7 days
**Phase 8 (QA & Launch)**: 3-4 weeks

**Total**: 10-14 weeks

## Next Steps

1. ✅ **Review this plan** with stakeholders
2. **Run `/sp.tasks`** to generate detailed, dependency-ordered tasks
3. **Begin Phase 0** research and document findings
4. **Phase 1 kickoff** after research approval
5. **Iterative development** following content phases
6. **Launch** after QA complete

---

**Plan Status**: ✅ Ready for implementation
**Constitutional Check**: ✅ All principles satisfied
**Next Command**: `/sp.tasks` to generate implementation tasks
