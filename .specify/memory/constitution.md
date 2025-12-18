<!--
Sync Impact Report:
Version change: None -> 1.0.0
Modified principles: None
Added sections: Core Principles, Content Standards, Technical Standards, Knowledge Constraints, Writing Constraints, Output Format Requirements
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/sp.constitution.md: ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Book Project Constitution

## Core Principles

### Course Outline Adherence
Content MUST follow the official course outline provided in the project brief (Physical AI → ROS2 → Gazebo/Unity → NVIDIA Isaac → VLA → Capstone).

### Completeness and Structure
All modules MUST be complete, structured, and fully written — no placeholder headings allowed.

### Pedagogical Utility
The book MUST be usable directly for teaching: examples, diagrams (Mermaid/ASCII), exercises, and mini-projects are MANDATORY.

### Voice and Readability
Voice MUST be clear, modern technical writing that a CS student or robotics beginner can understand.

### Production Readiness
Everything MUST be production-ready for GitHub Pages.

### Content Standards
Every module MUST be a full folder with multiple Markdown pages. Every page MUST contain: 5–10 clear sections, real explanations (not abstract fluff), practical examples (ROS2 code, Gazebo configs, Isaac snippets, etc.), Mermaid diagrams where relevant, end-of-page quizzes or tasks. No empty headings. No skeleton files. No TODOs.

## Technical Standards

File output MUST exactly match Docusaurus + Spec-Kit Plus conventions: Correct sidebar category format, Valid Markdown frontmatter, Folder structure ready to paste into /docs/, Sidebar snippet included for each module. All examples MUST run on the required stack (Ubuntu 22.04 + ROS2 Humble + Isaac Sim + Jetson).

## Knowledge Constraints

All technical claims MUST be accurate to: ROS2 official docs, NVIDIA Isaac Sim/Isaac ROS docs, Gazebo/Fortress docs. If an advanced concept is mentioned (VSLAM, Orin deployment, Nav2, etc.), include a short correct explanation.

## Governance

**Writing Constraints**: Tone: clear, direct, teaching-focused. Grade readability: Undergraduate CS level. Allow diagrams + concise formulas. No citations required (this is a textbook, not a research paper).
**Output Format Requirements**: When generating content for any module/page: Show folder structure first, Then full Markdown for every file, Then sidebar snippet. No commentary outside structure.

**Version**: 1.0.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10