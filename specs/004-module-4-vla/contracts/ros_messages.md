# ROS 2 Message Contracts: Module 4 - VLA

**Version**: 1.0
**Status**: DRAFT
**Author**: Gemini
**Created**: 2025-12-16
**Last Updated**: 2025-12-16

---

## 1. Overview

This document defines the custom ROS 2 Service and Action message types for the VLA module. These will be located in a new ROS 2 package named `vla_interfaces`.

## 2. Service Definitions (`.srv`)

### 2.1. `GeneratePlan.srv`

Used by the Orchestrator to request a task plan from the LLM Planner Node.

```
# Request
string command
---
# Response
string[] plan_steps
bool success
string error_message
```

-   **command**: The natural language command from the user (e.g., "get me the red can").
-   **plan_steps**: An array of strings representing the sequential plan from the LLM.
-   **success**: True if the plan was generated successfully.
-   **error_message**: Contains an error message if `success` is false.

## 3. Action Definitions (`.action`)

### 3.1. `ExecuteVlaTask.action`

A high-level action for the Orchestrator to command the robot to perform a single step of the LLM's plan, which might involve complex, multi-stage behavior.

```
# Goal
string task_description
---
# Result
bool success
string final_status
---
# Feedback
string current_step
float32 percent_complete
```

-   **task_description**: A single step from the LLM's plan (e.g., "find the red can on the table"). The relevant node (e.g., a perception node) will parse this to execute the task.
-   **success**: True if the task was completed successfully.
-   **final_status**: A message indicating the final outcome (e.g., "Object found at [x,y,z]" or "Failed to find object").
-   **current_step**: Feedback on the current internal state of the task execution (e.g., "Scanning environment", "Object detected").
-   **percent_complete**: A float from 0.0 to 1.0 indicating progress.

---
