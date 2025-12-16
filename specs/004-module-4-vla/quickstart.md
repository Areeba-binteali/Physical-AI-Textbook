# Quickstart Guide: Module 4 - VLA Capstone

**Version**: 1.0
**Status**: DRAFT
**Author**: Gemini
**Created**: 2025-12-16
**Last Updated**: 2025-12-16

---

## 1. Project Goal

This document provides a high-level overview of the final VLA capstone project. The goal is to build a system where a simulated humanoid robot can understand and execute voice commands by integrating perception, language models, and robotic control.

## 2. System Architecture

The system is composed of several interconnected ROS 2 nodes, orchestrated to form a complete Vision-Language-Action pipeline.

![VLA Architecture Diagram](https://mermaid.ink/img/pako:eNqNVMtqwzAQ_Jd8Cl94gS1wGxpKKaWlFh66UOOxE8cm2ZLYlXYSQf_7rpxkKZS0Bw97dmb2zE5iQoU7i10yLw21F8w9h5o1o02b0c4hA1Gz0lI2g2xV9w03tWJ8j80Qz-l00W9YkI7C4yqW5e30J3wIeyqQjX4gR7jW_hG9kI52jUjMvXk1zWlGgY80s-pD27xKj5f06t-Z9lYjC12aZ7yZDmXh_CqH68b3s5b7C50rK-U642a8bL0T1NqV5l1TzNlD5N-kCj-j5QhB9XzE1-aB-B6q8D5iN3bW4N-6z2d3T1R-r_5r6v2T4xQ8Zq_Yy65q9-qQ9x_5lZ1x-n76l7tXy80a42M4oT1_gVn6hGgUjR3Q_nB6h7cZgYnI2dD4k13jC1VwLz3Z9bJ0Xwz5SCT2BfWfJ4Wn5E3uK0n0mN_y5bS2a00q0sI_tWnI7wI31D8V2s6T0yK6Vp2ZzPq-N8Q7zH_X9L1AF8D6B_4L3Xp?type=png)

```mermaid
graph TD
    A[Voice Input] --> B(STT Node);
    B --> C{/voice_transcript};
    C --> D(LLM Planner Node);
    D -- GeneratePlan.srv --> E(Orchestrator Node);
    E -- ExecuteVlaTask.action --> F[Perception Node];
    E -- ExecuteVlaTask.action --> G[Navigation Node];
    E -- ExecuteVlaTask.action --> H[Manipulation Node];

    subgraph "ROS 2 System"
        B; C; D; E; F; G; H;
    end

    subgraph "External Services"
        I(OpenAI Whisper);
        J(OpenAI GPT-4);
    end

    B --> I;
    D --> J;
```

### Node Descriptions

-   **STT Node**: Captures microphone audio and uses the OpenAI Whisper API to transcribe it to text. Publishes the transcript to the `/voice_transcript` topic.
-   **LLM Planner Node**: A service server that, when called with a text command, queries the OpenAI GPT-4 API to generate a multi-step, natural-language plan.
-   **Orchestrator Node**: The "brain" of the system. It calls the `GeneratePlan` service and then iterates through the plan steps. For each step, it calls the `ExecuteVlaTask` action, directing the task to the appropriate subsystem.
-   **Perception Node**: An action server that handles tasks like "find the red can." It uses the robot's camera feed and computer vision techniques to identify and locate objects.
-   **Navigation Node**: An action server (wrapping Nav2) that handles tasks like "navigate to the kitchen table."
-   **Manipulation Node**: An action server (wrapping MoveIt2 or an Isaac Sim equivalent) that handles tasks like "pick up the object."

## 3. Setup and Execution

1.  **Prerequisites**:
    -   Ubuntu 22.04 with ROS 2 Humble.
    -   NVIDIA Isaac Sim installed.
    -   Python 3.10+.
    -   OpenAI API Key.
2.  **Run the System**:
    -   Launch the Isaac Sim environment with the robot and scene.
    -   Launch the ROS 2 nodes using a master launch file:
        ```bash
        ros2 launch vla_bringup final_project.launch.py
        ```
    -   Issue a voice command and observe the robot.

---
