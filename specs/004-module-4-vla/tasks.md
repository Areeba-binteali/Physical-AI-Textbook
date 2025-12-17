# Tasks: Module 4 - Vision-Language-Action (VLA)

**Input**: Design documents from `/specs/004-module-4-vla/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No automated tests are requested for this content-generation project. Chapter milestones and quizzes will serve as validation.

**Organization**: Tasks are grouped by user story to enable a progressive learning path for the student.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- All content will be created within the `docs/module-4-vla/` directory.

---

## Phase 1: Setup

**Purpose**: Create the directory structure and initial files for Module 4.

- [X] T001 Create the main directory for the new module at `docs/module-4-vla/`.
- [X] T002 [P] Create the category definition file `docs/module-4-vla/_category_.json` to label the module "Module 4: Vision-Language-Action".
- [X] T003 [P] Create the ROS 2 package for custom messages `vla_interfaces` with subdirectories for `srv` and `action`.
- [X] T004 [P] Define the `GeneratePlan.srv` message in `vla_interfaces/srv/GeneratePlan.srv`.
- [X] T005 [P] Define the `ExecuteVlaTask.action` message in `vla_interfaces/action/ExecuteVlaTask.action`.
- [X] T006 Add the new `vla_interfaces` package to the workspace build process.

---

## Phase 2: User Story 1 - Voice to Intent (Foundation) 🎯 MVP

**Goal**: As a student, I can issue spoken commands and convert them into structured text intents.
**Independent Test**: The student can run a launch file that starts a microphone listener node. When they speak, the transcribed text is visible on the `/voice_transcript` topic, and the introductory chapter is readable.

### Implementation for User Story 1

- [X] T007 [US1] Generate the full content for Chapter 1, "The VLA Philosophy", in `docs/module-4-vla/01-vla-philosophy.md`. Include diagrams and learning objectives.
- [X] T008 [US1] Generate the full content for Chapter 2, "Voice-to-Text: The Robot's Ear", in `docs/module-4-vla/02-voice-to-text.md`. Include code examples for the STT node and ROS 2 publisher.

---

## Phase 3: User Story 2 - Language to Plan (Cognition)

**Goal**: As a student, I can use an LLM to convert natural language goals into a sequence of robot actions.
**Independent Test**: The student can run a script or launch file that calls the `GeneratePlan` service with a text command and receives a valid, multi-step plan in the response.

### Implementation for User Story 2

- [X] T009 [US2] Generate the full content for Chapter 3, "Language-to-Plan: The Robot's Brain", in `docs/module-4-vla/03-language-to-plan.md`. Include code examples for the LLM planner service node and prompt engineering best practices.

---

## Phase 4: User Story 3 - Vision Grounding (Perception)

**Goal**: As a student, I can connect language instructions to visual perception outputs.
**Independent Test**: The student can run a perception node in Isaac Sim. When the `ExecuteVlaTask` action is called with a goal like "find the red can", the node uses the camera to find the object and returns its coordinates in the action result.

### Implementation for User Story 3

- [X] T010 [US3] Generate the full content for Chapter 5, "Vision Grounding: The Robot's Eye", in `docs/module-4-vla/05-vision-grounding.md`. Include code examples for the perception action server and how to use Isaac Sim's perception features.

---

## Phase 5: User Story 4 - Plan to Action (Embodiment)

**Goal**: As a student, I can execute planned actions using ROS 2 navigation and manipulation.
**Independent Test**: The student can run the orchestrator node with a hardcoded plan. The robot should physically execute the navigation and placeholder manipulation steps in the simulator.

### Implementation for User Story 4

- [X] T011 [US4] Generate the full content for Chapter 4, "The Orchestrator: From Plan to Action", in `docs/module-4-vla/04-orchestrator.md`. Include code examples for the central orchestrator node and how it uses action clients to call other nodes.

---

## Phase 6: User Story 5 - Capstone Autonomous Humanoid

**Goal**: As a student, I can build a simulated humanoid that listens, plans, navigates, sees, and acts.
**Independent Test**: The student can launch the entire system. They issue a voice command like "get the red can", and the robot autonomously completes the entire task in Isaac Sim.

### Implementation for User Story 5

- [X] T012 [US5] Generate the full content for Chapter 6, "Putting It All Together: The Capstone", in `docs/module-4-vla/06-capstone-project.md`. Include final integration code, a master launch file, and instructions for running the end-to-end demo.

---

## Phase 7: Polish & Integration

**Purpose**: Ensure module consistency, quality, and readiness for publication.

- [X] T013 [P] Review all chapters for consistency, clarity, and grammatical errors.
- [X] T014 [P] Generate a `_sidebar_snippet.js` file for easy integration into the main Docusaurus sidebar.
- [X] T015 Generate a final `00-review-and-next-steps.md` page summarizing the module and looking ahead.
- [X] T016 Validate all code examples and launch files work as described in the chapters.
- [X] T017 Optimize all chapter summaries and FAQs for the RAG chatbot.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)** must be completed first.
- **User Stories (Phases 2-6)** depend on Setup. The user stories should ideally be completed in order (US1 -> US2 -> US4 -> US3 -> US5) as they follow a logical learning progression, but some (like US3) can be worked on in parallel with others (like US2/US4).
- **Polish (Phase 7)** depends on all user stories being complete.

### Task Dependencies
- Within each phase, tasks are largely independent as they generate separate chapter files.
- `T006` depends on `T003`, `T004`, `T005`.
- `T012` (Capstone) depends on all previous chapter tasks (`T007` to `T011`).
- `T016` and `T017` depend on all other tasks being complete.

### Parallel Opportunities
- `T002`, `T003`, `T004`, `T005` can be done in parallel.
- The generation of chapters for different user stories can be parallelized to a degree. For example, the content for Chapter 5 (Vision) could be written at the same time as Chapter 3 (LLM) or Chapter 4 (Orchestrator).
- `T013` and `T014` can be done in parallel.

---

## Implementation Strategy

### Incremental Delivery
The module is designed to be built incrementally.
1.  Complete **Setup** and **User Story 1**. At this point, a student has a working voice-to-text pipeline.
2.  Add **User Story 2**. The student can now generate plans from text.
3.  Add **User Story 4**. The student can now execute hardcoded plans.
4.  Add **User Story 3**. The student can now ground language to vision.
5.  Complete **User Story 5** to integrate all parts into the final capstone.
6.  Finish with the **Polish** phase.
