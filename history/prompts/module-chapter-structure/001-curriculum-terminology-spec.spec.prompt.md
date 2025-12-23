---
id: 001
title: Curriculum Terminology Spec
stage: spec
date: 2025-12-22
surface: agent
model: claude-sonnet-4-5
feature: module-chapter-structure
branch: 003-module-chapter-structure
user: DELL
command: /sp.specify
labels: ["curriculum", "terminology", "standardization", "documentation", "structure"]
links:
  spec: ../specs/003-module-chapter-structure/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - ../specs/003-module-chapter-structure/spec.md
  - ../specs/003-module-chapter-structure/checklists/requirements.md
tests:
  - Specification quality validation: 17/17 checks passed
---

## Prompt

/sp.specify Structure Rules:
- Use the term "Module" ONLY for top-level sections.
- Inside each Module, use the term "Chapter" (Chapter 1, Chapter 2, etc.).
- DO NOT use the words "Lesson", "Unit", or "Project".
- Each Module must contain Chapters only.

## Response snapshot

Created comprehensive specification for standardizing curriculum structure terminology on branch `003-module-chapter-structure`.

**Specification includes**:
- 3 prioritized user stories covering all stakeholders: Curriculum Designer (P1) updates documentation, Student (P1) navigates clear hierarchy, Content Author (P2) creates consistent new material
- 12 functional requirements defining terminology rules (FR-001: Modules for top-level, FR-002: Chapters for subdivisions, FR-003: No "Lesson"/"Unit"/"Project" in structural contexts, FR-012: Narrative flexibility allowed)
- 8 measurable success criteria (100% terminology compliance, 95% student comprehension, 0 deprecated structural terms, 90% author clarity)
- Clear distinction between structural terminology (hierarchy labels - restricted to Module/Chapter) and narrative terminology (content text - flexible usage allowed)
- Edge cases addressing single-chapter modules, narrative usage vs structural usage, legacy URL handling, project sections

**Key design decisions**:
- Two-level hierarchy only (Module → Chapter, no sub-chapters)
- Chapter numbering restarts at 1 for each new Module
- Deprecated terms acceptable in narrative/descriptive contexts, forbidden in structural/hierarchical contexts
- Gradual migration strategy allowing temporary coexistence of old and new terminology
- File paths and directory structures should reflect Module/Chapter hierarchy

**Validation results**: 17/17 quality checks passed - specification is complete, testable, and ready for planning phase.

## Outcome

- ✅ Impact: Created complete, validated specification for curriculum terminology standardization affecting all educational content
- 🧪 Tests: Specification quality validation completed with 17/17 checks passed
- 📁 Files: Created spec.md (3 user stories, 12 requirements, 8 success criteria) and requirements.md checklist
- 🔁 Next prompts: `/sp.plan` to design migration strategy, template updates, style guide modifications, and implementation phases
- 🧠 Reflection: Successfully translated simple structural rules into comprehensive specification with clear scope boundaries. Critical insight: distinguishing structural terminology (hierarchy labels) from narrative terminology (content usage) prevents over-correction and maintains writing flexibility while achieving consistency goals.

## Evaluation notes (flywheel)

- Failure modes observed: None - specification workflow executed successfully
- Graders run and results (PASS/FAIL): Quality validation checklist - PASS (17/17 items)
- Prompt variant (if applicable): Standard /sp.specify workflow with content/terminology standardization context
- Next experiment (smallest change to try): N/A - workflow successful; consider documenting terminology standardization pattern as reusable template for other documentation/curriculum projects requiring consistent vocabulary
