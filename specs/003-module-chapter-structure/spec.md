# Feature Specification: Curriculum Structure Terminology Standardization

**Feature Branch**: `003-module-chapter-structure`
**Created**: 2025-12-22
**Status**: Draft
**Input**: User description: "Structure Rules: Use the term 'Module' ONLY for top-level sections. Inside each Module, use the term 'Chapter' (Chapter 1, Chapter 2, etc.). DO NOT use the words 'Lesson', 'Unit', or 'Project'. Each Module must contain Chapters only."

## Clarifications

### Session 2025-12-22

- Q: What platform/tool is currently hosting the curriculum content? → A: Static site generator like Docusaurus, MkDocs, or GitBook
- Q: What is the target timeframe for completing the terminology migration? → A: Phased approach over 1-3 months
- Q: Approximately how many Modules and Chapters exist in the current curriculum? → A: Small curriculum (1-3 Modules, fewer than 20 Chapters)
- Q: How will terminology compliance be validated during and after migration? → A: Automated script validation + spot-check manual review
- Q: How will content authors be trained on the new Module/Chapter terminology? → A: Written style guide with examples + updated templates

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Curriculum Designer Updates Documentation (Priority: P1)

A curriculum designer needs to standardize all educational content to use consistent terminology: "Module" for top-level sections and "Chapter" for sub-sections, eliminating inconsistent terms like "Lesson", "Unit", and "Project".

**Why this priority**: Consistent terminology is critical for curriculum clarity and prevents confusion among students, educators, and content contributors. This is a foundational change that affects all downstream content.

**Independent Test**: Review all curriculum documentation and verify that only "Module" and "Chapter" terms appear in the structure hierarchy, with no instances of "Lesson", "Unit", or "Project" in structural contexts.

**Acceptance Scenarios**:

1. **Given** existing curriculum documentation using mixed terminology, **When** the structure is updated, **Then** all top-level sections are labeled as "Module [N]" where N is the module number
2. **Given** content within a Module, **When** organizing sub-sections, **Then** all sub-sections are labeled as "Chapter [N]" where N is the chapter number within that module
3. **Given** a search for prohibited terms ("Lesson", "Unit", "Project"), **When** searching in structural/hierarchical contexts, **Then** zero results are found (terms may still exist in narrative content)
4. **Given** navigation menus and table of contents, **When** a user browses the curriculum, **Then** they see clear "Module" → "Chapter" hierarchy without confusion

---

### User Story 2 - Student Navigates Curriculum Structure (Priority: P1)

A student browsing the curriculum needs to quickly understand the hierarchical organization using clear, consistent terminology (Modules contain Chapters).

**Why this priority**: Clear navigation directly impacts student learning experience and completion rates. Inconsistent terminology creates cognitive load and confusion.

**Independent Test**: Have a new student navigate the curriculum and explain the structure. Success = student correctly identifies Modules as top-level containers and Chapters as subdivisions.

**Acceptance Scenarios**:

1. **Given** a student views the curriculum table of contents, **When** they identify the structure, **Then** they recognize Modules as major learning units and Chapters as topics within modules
2. **Given** a student is in "Module 2, Chapter 3", **When** they describe their location, **Then** they use the correct terminology without reverting to "Lesson" or "Unit"
3. **Given** breadcrumb navigation showing "Module X > Chapter Y", **When** a student follows the path, **Then** the hierarchy is immediately clear without additional explanation

---

### User Story 3 - Content Author Creates New Material (Priority: P2)

A content author creating new curriculum material needs clear guidelines on which terms to use for structuring content (Module for top-level, Chapter for subdivisions).

**Why this priority**: Prevents introduction of inconsistent terminology in new content. Authors need explicit rules to maintain standards.

**Independent Test**: Provide guidelines to a new author and have them create a sample curriculum outline. Success = outline uses only Module and Chapter terminology correctly.

**Acceptance Scenarios**:

1. **Given** style guide and templates, **When** an author creates a new module, **Then** they label it "Module [N]" and include only "Chapter [N]" subdivisions
2. **Given** an author wants to add a hands-on activity, **When** they structure it, **Then** they place it within a Chapter without creating a "Project" or "Unit" structural level
3. **Given** existing content using old terminology, **When** an author updates it, **Then** they replace "Lesson X" with "Chapter X" and "Unit Y" with appropriate Module or Chapter designation

---

### Edge Cases

- What happens when content mentions "lesson" or "unit" in narrative text (e.g., "In this lesson, you will learn...")? **Answer**: Narrative usage is acceptable; the rule applies only to structural/hierarchical labeling.
- How to handle existing content with embedded "Project" sections? **Answer**: Rename to "Chapter N: [Project Name]" or integrate into relevant chapters without structural "Project" level.
- What if a Module has only 1 Chapter? **Answer**: Still use "Chapter 1" for consistency, even if it's the only chapter in that module.
- How to handle cross-references to old terminology in URLs or anchors? **Answer**: Update slugs/URLs during migration; maintain redirects if needed for backward compatibility.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: All top-level curriculum sections MUST be labeled "Module [number]" (e.g., "Module 1", "Module 2", "Module 3")
- **FR-002**: All subdivisions within a Module MUST be labeled "Chapter [number]" (e.g., "Chapter 1", "Chapter 2", "Chapter 3")
- **FR-003**: The terms "Lesson", "Unit", and "Project" MUST NOT be used in structural or hierarchical labeling contexts
- **FR-004**: The curriculum structure MUST follow a strict two-level hierarchy: Module → Chapter (no additional structural levels)
- **FR-005**: Navigation elements (table of contents, breadcrumbs, menus) MUST display only "Module" and "Chapter" terminology
- **FR-006**: Templates for creating new curriculum content MUST use "Module" and "Chapter" placeholders exclusively
- **FR-007**: A written style guide MUST be created that explicitly defines the Module/Chapter structure rules with practical examples of correct usage, common mistakes to avoid, and the distinction between structural vs narrative terminology
- **FR-008**: Existing content using deprecated terms ("Lesson", "Unit", "Project") MUST be identified and flagged for updating
- **FR-009**: Chapter numbering MUST restart at 1 for each new Module (e.g., Module 1 has Chapters 1-3, Module 2 has Chapters 1-4)
- **FR-010**: File paths and directory structures SHOULD reflect the Module/Chapter hierarchy (e.g., `module-01/chapter-01/`, `module-01/chapter-02/`)
- **FR-011**: Metadata and frontmatter in content files MUST use "module" and "chapter" fields consistently
- **FR-012**: Narrative text within content MAY use "lesson", "unit", or "project" in non-structural contexts (e.g., "In this lesson, you will learn...")
- **FR-013**: The migration MUST be completed within a 1-3 month timeframe, with high-traffic content sections updated first, followed by moderate-traffic, then low-traffic sections
- **FR-014**: Terminology compliance MUST be validated through automated scripts that check file paths, frontmatter, headings, and navigation for deprecated terms, supplemented by manual spot-check review of content structure and context

### Key Entities

- **Module**: Top-level organizational unit in the curriculum; represents a major learning objective or subject area; contains multiple Chapters
- **Chapter**: Subdivision within a Module; represents a specific topic or learning segment; contains actual learning content (text, exercises, examples)
- **Curriculum Structure**: The hierarchical organization of learning content following the Module → Chapter pattern
- **Structural Terminology**: Terms used to label and organize curriculum hierarchy (restricted to "Module" and "Chapter")
- **Narrative Terminology**: Terms used within content text that do not affect structural hierarchy (may include "lesson", "unit", "project" for descriptive purposes)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of curriculum top-level sections are labeled "Module [number]" with no exceptions
- **SC-002**: 100% of subdivisions within Modules are labeled "Chapter [number]" with no exceptions
- **SC-003**: Zero instances of "Lesson", "Unit", or "Project" used in structural/hierarchical contexts (navigation, headings, file paths)
- **SC-004**: 95% of students can correctly identify the curriculum hierarchy as "Module contains Chapters" when surveyed
- **SC-005**: 100% of new content created after standardization uses correct Module/Chapter terminology
- **SC-006**: Content authors report 90%+ clarity on which terms to use based on style guide
- **SC-007**: Automated validation scripts report zero instances of deprecated structural terms in file paths, frontmatter, headings, and navigation
- **SC-009**: Manual spot-check review of 100% of Modules confirms proper structural terminology usage and appropriate narrative context
- **SC-008**: Table of contents and navigation menus display consistent Module → Chapter hierarchy across all platforms (web, PDF, mobile)

## Assumptions

1. **Scope of Change**: This is a terminology standardization for a small curriculum (1-3 Modules, fewer than 20 Chapters), not a restructuring of content itself; existing content topics remain but are relabeled
2. **Two-Level Hierarchy**: The curriculum is best served by a two-level structure (Module → Chapter); no need for additional levels like Sub-chapters
3. **Narrative Flexibility**: Using "lesson", "unit", or "project" within content narrative (not as structural labels) is acceptable and doesn't confuse students
4. **Migration Strategy**: Existing content will be updated over a 1-3 month phased approach; both old and new terminology may coexist during this transition period, with high-traffic sections prioritized first
5. **Numbering Convention**: Chapters are numbered sequentially within each Module, restarting at 1 for each new Module
6. **File System Impact**: Directory structures and file paths should be updated to reflect new terminology but can be done incrementally
7. **Backward Compatibility**: Legacy URLs or bookmarks to old terminology may need redirects during transition period
8. **Author Training**: Content authors will be trained through a written style guide with practical examples and updated content templates; no live training sessions required

## Out of Scope

- Restructuring or reorganizing the actual content topics (only terminology changes)
- Creating new content beyond templates and style guides
- Implementing technical systems or platforms (this is a content/terminology specification)
- Defining pedagogical approaches or learning methodologies
- Translating content to other languages (terminology rules apply to English version only, unless specified)
- Changing the number of Modules or Chapters (structure size remains as-is)
- Defining what content goes into each Module or Chapter (content mapping is separate)

## Dependencies

- Access to all curriculum documentation files for review and updating
- Written style guide document with practical examples of Module/Chapter usage, common mistakes, and structural vs narrative terminology distinction
- Template files for creating new curriculum content
- Static site generator (Docusaurus, MkDocs, or GitBook) that supports hierarchical labeling and custom navigation
- List of existing content using deprecated terminology (for migration planning)
- Automated validation script to scan for deprecated terminology in file paths, frontmatter, headings, and navigation structures

## Risks and Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Authors continue using old terminology out of habit | Medium - Inconsistency in new content | Distribute written style guide with practical examples; update templates to enforce new terms; templates serve as training reference |
| Students confused during transition period (mixed terminology) | Medium - Learning disruption | Communicate changes clearly; update high-traffic sections first; use redirects |
| Search engine or bookmark disruption from URL changes | Low - Discoverability issues | Implement 301 redirects; update sitemap; gradual rollout |
| Ambiguity about narrative vs structural usage | Medium - Over-correction or under-correction | Provide clear examples in style guide; "structural = hierarchy labels, narrative = content text" |
| Content volume to update (1-3 Modules, <20 Chapters) | Low - Manageable within 1-3 month timeline | Systematic manual updates feasible; use automated search/replace for file paths and metadata; track progress with checklist |
