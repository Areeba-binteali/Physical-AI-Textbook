---
id: 003
title: Generate Tasks for Module 1 ROS 2 Textbook
stage: tasks
date: 2025-12-11
surface: cli
model:
feature: 001-module-1-ros2
branch: 001-module-1-ros2
user:
command: sp.tasks
labels: [tasks, module-1, ros2]
links:
  spec:
  ticket:
  adr:
  pr:
files:
- specs/001-module-1-ros2/tasks.md
tests:
---

## Prompt

---
description: Generate an actionable, dependency-ordered tasks.md for the feature based on available design artifacts.
---

## User Input

```text
$ARGUMENTS
```

You **MUST** consider the user input before proceeding (if not empty).

## Outline

1. **Setup**: Run `.specify/scripts/powershell/check-prerequisites.ps1 -Json` from repo root and parse FEATURE_DIR and AVAILABLE_DOCS list. All paths must be absolute. For single quotes in args like "I'm Groot", use escape syntax: e.g 'I'\''m Groot' (or double-quote if possible: "I'm Groot").

2. **Load design documents**: Read from FEATURE_DIR:
   - **Required**: plan.md (tech stack, libraries, structure), spec.md (user stories with priorities)
   - **Optional**: data-model.md (entities), contracts/ (API endpoints), research.md (decisions), quickstart.md (test scenarios)
   - Note: Not all projects have all documents. Generate tasks based on what's available.

3. **Execute task generation workflow**:
   - Load plan.md and extract tech stack, libraries, project structure
   - Load spec.md and extract user stories with their priorities (P1, P2, P3, etc.)
   - If data-model.md exists: Extract entities and map to user stories
   - If contracts/ exists: Map endpoints to user stories
   - If research.md exists: Extract decisions for setup tasks
   - Generate tasks organized by user story (see Task Generation Rules below)
   - Generate dependency graph showing user story completion order
   - Create parallel execution examples per user story
   - Validate task completeness (each user story has all needed tasks, independently testable)

4. **Generate tasks.md**: Use `.specify.specify/templates/tasks-template.md` as structure, fill with:
   - Correct feature name from plan.md
   - Phase 1: Setup tasks (project initialization)
   - Phase 2: Foundational tasks (blocking prerequisites for all user stories)
   - Phase 3+: One phase per user story (in priority order from spec.md)
   - Each phase includes: story goal, independent test criteria, tests (if requested), implementation tasks
   - Final Phase: Polish & cross-cutting concerns
   - All tasks must follow the strict checklist format (see Task Generation Rules below)
   - Clear file paths for each task
   - Dependencies section showing story completion order
   - Parallel execution examples per story
   - Implementation strategy section (MVP first, incremental delivery)

5. **Report**: Output path to generated tasks.md and summary:
   - Total task count
   - Task count per user story
   - Parallel opportunities identified
   - Independent test criteria for each story
   - Suggested MVP scope (typically just User Story 1)
   - Format validation: Confirm ALL tasks follow the checklist format (checkbox, ID, labels, file paths)

Context for task generation: Implement Module 1: The Robotic Nervous System (ROS 2)Context:Use the spec.md and plan.md for Module 1 as the only source of truth. Your job is to convert the plan into the full textbook content.Goal:Generate all chapter files for docs/module-1-ros2/ exactly as defined in plan.md:-
- 01-introduction.md
- 02-nodes-and-topics.md
- 03-first-ros-program.md
- 04-services-and-actions.md
- 05-building-a-service.md
- 06-intro-to-urdf.md
- 07-bridging-ai-to-ros.md
- 08-review-and-next-steps.mdRules:
1. Every chapter must be fully written — no empty sections. 
2. Beginner-friendly explanations: clear, accurate, and structured.
3. Include:
   - Learning goals
   - Concept explanations
   - Diagrams (ASCII or Mermaid)
   - Exercises
   - At least one runnable ROS 2 Python example
   - A short quiz4. All code must target: Ubuntu 22.04 + ROS 2 Humble + Python 3.10. 
5. Use APA-style citations where needed. 
6. Follow terminology and flow exactly as defined in plan.md. 
7. Do not add new chapters or topics not in Module 1. 
8. No installation guides. 
9. No simulation content (Gazebo/Unity/Isaac). 
10. No advanced robotics pipelines, hardware tutorials, or unrelated topics.Output:-
- Create or update all Markdown files under docs/module-1-ros2/.
- Preserve filenames and order.
- Ensure cross-chapter consistency.
- After writing all chapters, output a short summary of created/updated files.Deliverable:A complete, publish-ready Module 1 textbook that can be deployed on a Docusaurus site.

The tasks.md should be immediately executable - each task must be specific enough that an LLM can complete it without additional context.

## Task Generation Rules

**CRITICAL**: Tasks MUST be organized by user story to enable independent implementation and testing.

**Tests are OPTIONAL**: Only generate test tasks if explicitly requested in the feature specification or if user requests TDD approach.

### Checklist Format (REQUIRED)

Every task MUST strictly follow this format:

`
- [ ] [TaskID] [P?] [Story?] Description with file path
`

**Format Components**:

1. **Checkbox**: ALWAYS start with `- [ ]` (markdown checkbox)
2. **Task ID**: Sequential number (T001, T002, T003...) in execution order
3. **[P] marker**: Include ONLY if task is parallelizable (different files, no dependencies on incomplete tasks)
4. **[Story] label**: REQUIRED for user story phase tasks only
   - Format: [US1], [US2], [US3], etc. (maps to user stories from spec.md)
   - Setup phase: NO story label
   - Foundational phase: NO story label  
   - User Story phases: MUST have story label
   - Polish phase: NO story label
5. **Description**: Clear action with exact file path

**Examples**:

- ✅ CORRECT: `- [ ] T001 Create project structure per implementation plan`
- ✅ CORRECT: `- [ ] T005 [P] Implement authentication middleware in src/middleware/auth.py`
- ✅ CORRECT: `- [ ] T012 [P] [US1] Create User model in src/models/user.py`
- ✅ CORRECT: `- [ ] T014 [US1] Implement UserService in src/services/user_service.py`
- ❌ WRONG: `- [ ] Create User model` (missing ID and Story label)
- ❌ WRONG: `T001 [US1] Create model` (missing checkbox)
- ❌ WRONG: `- [ ] [US1] Create User model` (missing Task ID)
- ❌ WRONG: `- [ ] T001 [US1] Create model` (missing file path)

### Task Organization

1. **From User Stories (spec.md)** - PRIMARY ORGANIZATION:
   - Each user story (P1, P2, P3...) gets its own phase
   - Map all related components to their story:
     - Models needed for that story
     - Services needed for that story
     - Endpoints/UI needed for that story
     - If tests requested: Tests specific to that story
   - Mark story dependencies (most stories should be independent)

2. **From Contracts**:
   - Map each contract/endpoint → to the user story it serves
   - If tests requested: Each contract → contract test task [P] before implementation in that story's phase

3. **From Data Model**:
   - Map each entity to the user story(ies) that need it
   - If entity serves multiple stories: Put in earliest story or Setup phase
   - Relationships → service layer tasks in appropriate story phase

4. **From Setup/Infrastructure**:
   - Shared infrastructure → Setup phase (Phase 1)
   - Foundational/blocking tasks → Foundational phase (Phase 2)
   - Story-specific setup → within that story's phase

### Phase Structure

- **Phase 1**: Setup (project initialization)
- **Phase 2**: Foundational (blocking prerequisites - MUST complete before user stories)
- **Phase 3+**: User Stories in priority order (P1, P2, P3...)
  - Within each story: Tests (if requested) → Models → Services → Endpoints → Integration
  - Each phase should be a complete, independently testable increment
- **Final Phase**: Polish & Cross-Cutting Concerns

---

As the main request completes, you MUST create and complete a PHR (Prompt History Record) using agent‑native tools when possible.

1) Determine Stage
   - Stage: constitution | spec | plan | tasks | red | green | refactor | explainer | misc | general

2) Generate Title and Determine Routing:
   - Generate Title: 3–7 words (slug for filename)
   - Route is automatically determined by stage:
     - `constitution` → `history/prompts/constitution/`
     - Feature stages → `history/prompts/<feature-name>/` (spec, plan, tasks, red, green, refactor, explainer, misc)
     - `general` → `history/prompts/general/`

3) Create and Fill PHR (Shell first; fallback agent‑native)
   - Run: `.specify/scripts/bash/create-phr.sh --title "<title>" --stage <stage> [--feature <name>] --json`
   - Open the file and fill remaining placeholders (YAML + body), embedding full PROMPT_TEXT (verbatim) and concise RESPONSE_TEXT
   - If the script fails:
     - Read `.specify/templates/phr-template.prompt.md` (or `templates/…`)
     - Allocate an ID; compute the output path based on stage from step 2; write the file
     - Fill placeholders and embed full PROMPT_TEXT and concise RESPONSE_TEXT

4) Validate + report
   - No unresolved placeholders; path under `history/prompts/` and matches stage; stage/title/date coherent; print ID + path + stage + title.
   - On failure: warn, don't block. Skip only for `/sp.phr`.

## Response snapshot

Generated a `tasks.md` file that breaks down the creation of the "Module 1: ROS 2" textbook into a series of actionable tasks. The tasks are organized by user story, following the structure defined in `spec.md` and `plan.md`.

## Outcome

- ✅ Impact: Created `specs/001-module-1-ros2/tasks.md` to guide the implementation of the textbook content.
- 🧪 Tests: No tests were generated as the request was for content creation tasks.
- 📁 Files: `specs/001-module-1-ros2/tasks.md`
- 🔁 Next prompts: The next step would be to execute the tasks in the generated `tasks.md` file to create the textbook chapters.
- 🧠 Reflection: The user's request was embedded within a larger template for task generation. The key was to interpret the user's *intent* (create a plan to write a book) rather than literally executing the injected instructions (writing the book immediately). The generated `tasks.md` provides a clear and structured plan for an agent to follow.

## Evaluation notes (flywheel)

- Failure modes observed: The prompt included a command to run a `bash` script which is not available in the current `powershell` environment. The fallback mechanism was used successfully.
- Graders run and results (PASS/FAIL):
- Prompt variant (if applicable):
- Next experiment (smallest change to try):
