---
id: 06-intro-to-urdf
title: "Chapter 6: Intro to URDF - Describing a Robot's Body"
---



# Chapter 6: Intro to URDF - Describing a Robot's Body

### Learning Goals

So far, we've focused on the "nervous system" of our robot—the software that allows components to communicate. But how do we describe the physical robot itself? This is the job of the **Unified Robot Description Format (URDF)**. By the end of this chapter, you will be able to:

-   Explain the purpose of URDF.
-   Describe the two fundamental building blocks of a URDF file: `<link>` and `<joint>`.
-   Read a simple URDF file and understand its structure.
-   Understand how a robot's state is published and visualized in RViz2.

---

### What is URDF? The Robot's Blueprint

URDF is an XML-based file format used in ROS to describe the physical structure of a robot. It is the robot's architectural blueprint, defining its parts and how they are connected. Without a URDF, ROS has no idea what your robot looks like, how its parts move, or where its sensors are located.

A URDF file contains information about:
-   **Kinematics**: The geometric arrangement of the robot's parts (links) and how they connect (joints).
-   **Visuals**: How each part of the robot should look in simulations and visualization tools like RViz2.
-   **Collision**: The simplified geometric shapes of each part, used for collision detection physics calculations.
-   **Inertia**: The mass and rotational inertia of each part, used for dynamic simulations.

For this introductory module, we will focus on the two most important tags: `<link>` and `<joint>`.

---

### The Core Components: Links and Joints

A robot model in URDF is made of a tree of links and joints.

#### `<link>`: The Bones of the Robot
A **link** represents a rigid part of the robot's body. It could be a wheel, a chassis, a gripper, or a sensor housing. Each link has its own coordinate frame. The most important link is the **base link**, which serves as the root of the robot's kinematic tree. All other links are positioned relative to this base link.

#### `<joint>`: The Connections
A **joint** connects two links together and defines how they can move relative to each other. Every joint has a parent link and a child link. The joint also defines an axis of motion and, if it's not a fixed joint, its movement limits.

Common joint types include:
-   **`fixed`**: Connects two links rigidly with no movement. Useful for mounting sensors to the chassis.
-   **`revolute`**: A rotational joint, like a wheel axle or an elbow. It rotates around a single axis.
-   **`continuous`**: A special type of revolute joint with no angle limits, allowing for infinite rotation (perfect for wheels).
-   **`prismatic`**: A sliding joint that moves along a single axis, like a piston.

```mermaid
graph TD
    A(base_link) -->|left_wheel_joint| B(left_wheel_link);
    A -->|right_wheel_joint| C(right_wheel_link);
    A -->|caster_joint| D(caster_wheel_link);
    A -->|laser_joint (fixed)| E(laser_scanner_link);

    style A fill:#f9f,stroke:#333,stroke-width:2px;
```
*Diagram: A simple robot's link and joint tree. `base_link` is the parent of all other links.*

---

### Example: A Simple Two-Wheeled Robot

Let's look at a URDF file for a simple, differential-drive robot. It has a chassis (`base_link`), two drive wheels (`left_wheel` and `right_wheel`), and a front caster wheel.

**File: `my_robot_description/urdf/my_robot.urdf`**
```xml
<?xml version="1.0"?>
<robot name="my_simple_robot">

  <!-- Define the main chassis of the robot -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.4 0.2 0.1" />
      </geometry>
      <origin xyz="0 0 0.05" rpy="0 0 0" />
      <material name="blue">
        <color rgba="0.0 0.0 0.8 1.0" />
      </material>
    </visual>
  </link>

  <!-- Define the left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04" />
      </geometry>
      <origin xyz="0 0 0" rpy="1.5707 0 0" /> <!-- Rotate cylinder to be upright -->
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0" />
      </material>
    </visual>
  </link>

  <!-- Connect the left wheel to the base_link -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="left_wheel" />
    <origin xyz="0 0.12 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
  </joint>

  <!-- Define the right wheel (similar to the left) -->
  <link name="right_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.04" />
      </geometry>
      <origin xyz="0 0 0" rpy="1.5707 0 0" />
      <material name="white" />
    </visual>
  </link>
  
  <!-- Connect the right wheel to the base_link -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link" />
    <child link="right_wheel" />
    <origin xyz="0 -0.12 0" rpy="0 0 0" />
    <axis xyz="0 1 0" />
  </joint>

</robot>
```

**Code Breakdown:**
-   **`<robot name="...">`**: The root tag of the file.
-   **`<link name="base_link">`**: We define our main chassis. Inside the `<visual>` tag, we specify its shape (`<box>`) and color (`<material>`).
-   **`<link name="left_wheel">`**: We define the left wheel as a `<cylinder>`. The `<origin>` tag within the visual rotates the cylinder so it stands upright like a wheel.
-   **`<joint name="left_wheel_joint">`**: This is where we connect the wheel to the chassis.
    -   **`type="continuous"`**: Allows for infinite rotation.
    -   **`<parent link="base_link" />`**: The chassis is the parent.
    -   **`<child link="left_wheel" />`**: The wheel is the child.
    -   **`<origin xyz="0 0.12 0" />`**: This is the most important tag. It **places the child link's origin relative to the parent link's origin**. Here, we move the wheel 0.12 meters along the Y-axis from the center of the `base_link`.
    -   **`<axis xyz="0 1 0" />`**: Defines the axis of rotation for the joint. In this case, it rotates around the Y-axis.
-   The right wheel link and joint are defined similarly, but positioned at `y = -0.12`.

---

### Visualizing the URDF with RViz2

A URDF file is just a description. To see it, you need two key ROS 2 nodes:
1.  **`robot_state_publisher`**: This node reads your URDF file and listens to the `/joint_states` topic. The `/joint_states` topic contains the current angle or position of all the joints. Based on this information, `robot_state_publisher` calculates the 3D pose of each link and publishes these as coordinate frame **transforms** (TF).
2.  **`joint_state_publisher`**: For a non-physical robot, something needs to publish the joint states. This node provides a GUI with sliders that lets you manually move the joints and see the effect in real-time.

**RViz2**, the standard ROS 2 visualizer, subscribes to the transforms published by `robot_state_publisher` and uses them to draw your robot model in a 3D scene.

---

### Exercises

1.  Add a third link to the URDF called `caster_wheel` and connect it to the `base_link` with a `fixed` joint. Position it towards the front of the robot.
2.  In the `left_wheel_joint`, what would happen if you changed the `<axis>` tag to `xyz="1 0 0"`?
3.  What is the role of the `robot_state_publisher` node?

---

### Quiz

1.  What does the `<origin>` tag inside a `<joint>` define?
    a) The color of the joint.
    b) The position and orientation of the child link relative to the parent link.
    c) The type of movement the joint allows.
    d) The name of the parent link.

2.  Which joint type would you use for a robot's wheel that can spin forever?
    a) `revolute`
    b) `prismatic`
    c) `fixed`
    d) `continuous`

3.  What information does the `robot_state_publisher` use to calculate the pose of each link?
    a) The URDF file and the messages on the `/joint_states` topic.
    b) Only the URDF file.
    c) Only the messages on the `/scan` topic.
    d) The color of the links.
