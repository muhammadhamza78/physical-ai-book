# Physical AI Book Constitution

## Version

**Version**: 1.0.0
**Ratified**: 2025-12-22
**Last Amended**: 2025-12-22

## Core Principles

### I. Accessibility First

All content must be accessible to beginners with no prior AI experience. Technical concepts are introduced progressively with:
- Clear definitions before usage
- Real-world analogies and examples
- Visual aids (diagrams, images, videos) for complex topics
- Code examples that are fully commented and explained line-by-line
- Prerequisites explicitly stated at the start of each section

### II. Hands-On Learning Priority

Learning happens through doing. Every major concept must include:
- At least one practical, runnable example
- Step-by-step tutorials with expected outputs
- Troubleshooting sections for common errors
- Mini-projects that build on previous knowledge
- Interactive components where technically feasible

### III. Progressive Complexity

Content is structured in layers from foundational to advanced:
- Basic concepts (what is physical AI, sensing, actuation)
- Intermediate integration (connecting sensors, basic control loops)
- Advanced topics (multi-agent systems, optimization) clearly marked
- Optional "Deep Dive" sections for curious readers
- Clear navigation paths for different learning goals

### IV. Production-Ready Examples

All code examples must be:
- Tested and verified to work as documented
- Include error handling and edge cases
- Follow industry best practices
- Include comments explaining the "why" not just the "what"
- Accompanied by hardware requirements and setup instructions

### V. Clear Documentation Standards

Every tutorial/guide must include:
- Learning objectives stated upfront
- Estimated completion time
- Required hardware and software
- Expected outcomes with screenshots/videos
- Links to related concepts and resources
- Glossary terms highlighted on first use

### VI. Community-Driven Improvement

The book evolves with reader feedback:
- Clear contribution guidelines
- Issue templates for reporting errors or requesting clarifications
- Regular review cycles based on reader questions
- Acknowledgment of community contributors
- Version history tracking content updates

## Success Criteria

### For Readers
- **Beginners** can complete Chapter 1 examples within 2 hours with zero AI background
- **Intermediate learners** successfully build at least 3 functioning physical AI projects by book completion
- **95% success rate** on hands-on tutorials when following documented steps
- **Engagement**: Readers complete 70%+ of hands-on exercises

### For Content Quality
- All code examples pass automated testing in CI/CD
- Zero broken links or missing resources
- Screenshots/videos are up-to-date with current library versions
- Each chapter reviewed by at least 2 technical reviewers and 1 beginner tester
- Loading time < 3 seconds for all pages

### For Learning Outcomes
- Readers can explain physical AI concepts in their own words
- Readers can debug common issues independently using provided troubleshooting guides
- Readers can adapt example code to their own projects
- Readers understand when to use different sensing/actuation strategies

## Constraints

### Technical Constraints
- **Docusaurus 3.x** as the static site generator
- Maximum bundle size: 5MB per page for fast loading
- Mobile-responsive design required (readable on tablets/phones)
- Offline-capable documentation (PWA features)
- Support for code syntax highlighting in Python, C++, JavaScript

### Content Constraints
- No assumed knowledge beyond basic programming (Python preferred)
- Hardware requirements must be affordable (<$200 USD for full project setups)
- Third-party library dependencies must be open-source and actively maintained
- No platform-specific content (cross-platform examples preferred)
- Examples must work on Windows, macOS, and Linux where applicable

### Pedagogical Constraints
- Maximum 15-minute read time per section (excluding exercises)
- No more than 3 new technical terms introduced per section
- Every chapter must be self-contained (can be read independently)
- No "exercise left to the reader" without hints or starter code
- Mathematical notation explained in plain language

### Resource Constraints
- Total page count target: 300-400 pages (digital equivalent)
- Image/video assets optimized for web (<500KB per asset)
- External dependencies documented with fallback options
- Hosting costs: Target < $50/month for documentation site

## Stakeholders

### Primary Audience
- **Beginners**: Developers/hobbyists with basic programming knowledge, no AI experience
- **Students**: University/bootcamp students learning robotics or embedded AI
- **Makers**: DIY enthusiasts building physical computing projects

### Secondary Audience
- **Educators**: Instructors seeking curriculum materials for physical AI courses
- **Intermediate Practitioners**: Engineers transitioning to physical AI from web/software
- **Technical Reviewers**: Domain experts ensuring accuracy

### Contributors
- **Authors**: Primary content creators (architecture, writing, examples)
- **Technical Reviewers**: AI/robotics experts validating technical accuracy
- **Community Contributors**: Readers submitting fixes, improvements, examples
- **Maintainers**: Team ensuring dependencies stay current and examples work

## Brand Voice

### Tone
- **Friendly and Encouraging**: "Let's build together" not "You must do X"
- **Clear and Direct**: Avoid jargon; when used, define immediately
- **Patient**: Anticipate confusion and address it proactively
- **Enthusiastic**: Convey excitement about physical AI without hype
- **Humble**: Acknowledge complexity; respect reader's learning journey

### Style Guidelines
- Use active voice ("Connect the sensor" not "The sensor should be connected")
- Address reader as "you" to create direct connection
- Celebrate small wins ("Great! Your first sensor is working!")
- Use analogies from everyday life (not obscure technical references)
- Break long explanations into bullet points or numbered steps
- Code comments written as teaching moments, not reference docs

### Visual Identity
- Clean, modern design with high contrast for readability
- Consistent color coding (e.g., hardware = blue, software = green, theory = purple)
- Diagrams use simple icons and clear labels
- Code blocks have descriptive titles ("Example: Reading Temperature Sensor")
- Callout boxes for Tips, Warnings, and Deep Dives

### Writing Patterns
- **Start sections with "Why?"**: Motivate learning before diving in
- **Show, then explain**: Code example first, then breakdown
- **End with "What's Next?"**: Connect to upcoming topics
- **Use real-world scenarios**: "Imagine you're building a plant watering robot..."
- **Iterate openly**: "This works, but we can improve it by..."

## Governance

### Amendment Process
1. Proposed changes submitted as GitHub issues with rationale
2. Discussion period (minimum 1 week for major changes)
3. Approval requires consensus from 2+ core maintainers
4. Version number incremented (major/minor/patch per semantic versioning)
5. Changelog updated with migration notes if applicable

### Compliance Verification
- All PRs must include checklist confirming constitutional alignment
- Automated checks for code example testing, link validation, image optimization
- Quarterly audits of reader feedback for constitutional adherence
- Community moderators empowered to flag non-compliant content

### Conflict Resolution
- Constitution principles override individual preferences
- When principles conflict, prioritize in order: Accessibility → Hands-On → Progressive Complexity
- Technical compromises documented as ADRs (Architecture Decision Records)
- Reader experience takes precedence over technical elegance

### Living Document
- This constitution evolves based on reader feedback and learning outcomes data
- Major revisions (version 2.0+) require retrospective on version 1.x learnings
- All amendments tracked in `history/adr/` with decision rationale

**Version**: 1.0.0 | **Ratified**: 2025-12-22 | **Last Amended**: 2025-12-22
