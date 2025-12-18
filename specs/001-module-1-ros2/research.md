# Research & Decisions for Module 1

This document records the key architectural and content decisions made during the planning phase for the "Robotic Nervous System (ROS 2)" module.

---

### Decision 1: Depth of ROS 2 Primitives Coverage

-   **Decision**: ROS 2 primitives (nodes, topics, services, actions) will be covered conceptually, focusing on the "what" and "why" of each component. Implementation examples will be limited to simple, runnable "hello world" style Python scripts.
-   **Rationale**: As a beginner-focused module, the primary goal is to build a strong, correct mental model of ROS 2's architecture. Overloading the student with complex implementation details or configuration options would be counter-productive and violate the "beginner-friendly" constraint. This approach ensures students can write and understand basic programs without getting lost.
-   **Alternatives Considered**:
    -   *Deep Dive*: A deep dive into all `rclpy` options and command-line tools. Rejected as too advanced for an introductory module.
    -   *Concepts Only*: No code examples at all. Rejected as it would violate the "Pedagogical Utility" principle requiring practical examples.

---

### Decision 2: Format for Examples

-   **Decision**: A hybrid approach will be used. MermaidJS diagrams will be used to illustrate the high-level architecture and relationships between components (e.g., a graph of nodes and topics). Minimal, complete, and runnable Python code snippets will be provided for all `rclpy` examples.
-   **Rationale**: This combination serves two distinct pedagogical purposes. Diagrams are optimal for abstract, conceptual understanding, while runnable code is essential for hands-on learning, debugging, and building practical skills, as mandated by the constitution.
-   **Alternatives Considered**:
    -   *Diagrams Only*: Rejected as insufficient for practical learning.
    -   *Pseudo-code*: Rejected because runnable Python code provides higher value and allows students to experiment directly.

---

### Decision 3: Positioning of URDF

-   **Decision**: URDF will be presented as a format for describing a robot's "anatomy." The content will focus exclusively on the XML syntax for defining `<link>` (physical parts) and `<joint>` (connections between parts). It will not cover simulation aspects, physics properties, or how URDF files are loaded by simulators.
-   **Rationale**: This creates a clean and unambiguous boundary between Module 1 (describing the robot) and Module 2 (simulating the robot). It fulfills the requirement to introduce URDF without overlapping with the scope of later modules.
-   **Alternatives Considered**:
    -   *Full URDF Tutorial*: Including `<gazebo>` tags and simulation properties. Rejected as a clear scope violation.
    -   *Ignoring URDF*: Rejected because it is a fundamental prerequisite for humanoid robotics and simulation.

---

### Decision 4: Boundaries for Agent-to-ROS Bridging

-   **Decision**: The concept of bridging an external AI agent to ROS 2 will be covered at a high-level, conceptual level only. A block diagram will be used to illustrate the pattern: an external process (the "AI Agent") communicates with the ROS 2 graph by using ROS 2 client libraries to publish to topics or call services. No implementation of a bridge will be provided.
-   **Rationale**: This introduces students to a core concept of "Physical AI" early on, setting the stage for later modules. It respects the constraint to avoid complex implementation pipelines while still providing a valuable conceptual link between the world of AI and the world of robotics.
-   **Alternatives Considered**:
    -   *Full Bridge Implementation*: Rejected as too complex and out of scope for an introductory module.
    -   *Skipping the Topic*: Rejected as it would miss a key opportunity to connect the module to the book's overall theme of Physical AI.
