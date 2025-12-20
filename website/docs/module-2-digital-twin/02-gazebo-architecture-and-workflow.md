---
id: 02-gazebo-architecture-and-workflow
title: "02 - Gazebo Architecture and Workflow"
---



# 02 - Gazebo Architecture and Workflow

## Introduction to Gazebo

Gazebo is a powerful open-source 3D robotics simulator. It is widely used in research and industry to accurately simulate robots in complex indoor and outdoor environments. Gazebo offers the ability to simulate high-fidelity physics, generate realistic sensor data, and supports various robot models. It's particularly popular in the ROS (Robot Operating System) community due to its deep integration with ROS tools and libraries.

## Key Components of Gazebo

Understanding Gazebo's architecture is crucial for effective simulation. Here are its primary components:

1.  **Server (gzserver)**: This is the core physics engine and simulation backend. It handles all the computations related to physics, sensor data generation, and robot dynamics. It runs headlessly, meaning it doesn't have a graphical user interface (GUI) by itself.

2.  **Client (gzclient)**: This is the graphical user interface (GUI) that allows users to visualize the simulation, interact with robots, and inspect various properties. It connects to the `gzserver` to receive visual and telemetry data.

3.  **Worlds**: A world file (typically `.world` or `.sdf` format) defines the environment in which the simulation takes place. This includes:
    *   **Models**: Static objects (e.g., walls, furniture, terrain) and dynamic objects (e.g., robots, other moving entities).
    *   **Lights**: Illumination sources within the environment.
    *   **Sensors**: Simulated sensors attached to robots or static objects (e.g., cameras, LiDAR, IMUs).
    *   **Physics Engine Parameters**: Configuration for the underlying physics engine (e.g., gravity, time step).

4.  **Models (SDF - Simulation Description Format)**: Robots and other objects within a world are defined using SDF files. SDF is an XML-based format that describes the geometric, kinematic, and dynamic properties of a robot or object. It specifies:
    *   **Links**: Rigid bodies (e.g., robot base, arm segments).
    *   **Joints**: Connections between links, defining their relative motion.
    *   **Visuals**: How the link appears (color, texture, mesh).
    *   **Collisions**: The simplified geometry used for physics interactions.
    *   **Sensors**: Definitions of sensors attached to links.
    *   **Plugins**: Custom behaviors or integrations (e.g., ROS control interfaces).

5.  **Plugins**: Gazebo's functionality can be extended through plugins. These are shared libraries that can be loaded into `gzserver` or `gzclient` to add custom features, such as:
    *   **Sensor Plugins**: To simulate specific types of sensors.
    *   **Controller Plugins**: To interface with robot control systems (e.g., ROS controllers).
    *   **World Plugins**: To add dynamic elements to the simulation environment.

```mermaid
graph TD
    A[Physical Robot] --> B(Sensors/Actuators);
    B --> C{ROS 2 Nodes/Drivers};
    C --> D[Robot Controller (ROS 2)];

    subgraph Gazebo Simulation
        E[gzserver (Physics Engine)] <--> F[gzclient (GUI)];
        G[World File (.world/.sdf)] --> E;
        H[Robot Model (SDF/URDF)] --> G;
        I[Gazebo Plugins] --> E;
    end

    D <--> J(ROS 2 Bridge / Gazebo ROS Plugins);
    J <--> E;
```
## Gazebo Workflow for Robot Simulation

A typical workflow for simulating robots in Gazebo involves these steps:

1.  **Define the Robot Model (SDF/URDF)**:
    *   Robots are primarily defined using URDF (Unified Robot Description Format) in the ROS ecosystem. URDF can be converted to SDF for Gazebo.
    *   This involves specifying links, joints, and visual/collision geometries, often using mesh files (e.g., `.stl`, `.dae`).

2.  **Create the World File**:
    *   Design the environment in which your robot will operate. This includes adding static objects, lighting, and defining the initial pose of your robot.
    *   You can use existing models from Gazebo's model database or create your own.

3.  **Launch the Simulation**:
    *   Use command-line tools (`gzserver`, `gzclient`) or ROS launch files to start the Gazebo server and client.
    *   A common approach in ROS is to use a launch file that starts `gzserver`, loads your robot model and world, and then optionally starts `gzclient` and any necessary ROS nodes (e.g., robot controllers, teleoperation interfaces).

4.  **Interact and Monitor**:
    *   Once the simulation is running, you can:
        *   Visualize the robot's behavior in `gzclient`.
        *   Publish commands to the robot (e.g., joint velocities, force commands) via ROS topics.
        *   Subscribe to sensor data (e.g., camera images, LiDAR scans) via ROS topics.
        *   Inspect physical properties of objects within the `gzclient` GUI.

## Running a Basic Humanoid Robot Simulation

While detailed setup and command execution are out of scope for this textbook, understanding the conceptual steps to run a basic humanoid robot simulation is important.

1.  **Ensure ROS 2 and Gazebo are installed**: On an Ubuntu 22.04 system, a typical ROS 2 Humble installation includes Gazebo Fortress.
2.  **Source ROS 2 environment**: This makes ROS 2 commands available in your terminal.
3.  **Launch a pre-built robot simulation**: Many ROS 2 packages provide example launch files for various robots. For a humanoid, you might use a package that defines a simple bipedal or humanoid robot.
    *   Conceptually, you would execute a ROS launch command that points to a specific launch file (e.g., `ros2 launch <robot_description_pkg> display_humanoid.launch.py`). This launch file internally handles starting Gazebo with the robot model.
4.  **Observe in Gazebo GUI**: Once launched, the Gazebo client (`gzclient`) would open, displaying the humanoid robot in its simulated environment. You could then observe its initial pose and potentially interact with it via ROS topics (e.g., sending commands to make it stand or move).

This conceptual understanding forms the basis for diving deeper into physics and sensor simulation.

## Conceptual Gazebo Examples

To provide a practical feel without diving into code execution, let's consider conceptual examples of how a simple robot and world might be defined.

### Conceptual Robot Model (SDF/URDF Fragment)

Imagine a very simple wheeled robot. Its definition would involve:

*   **A `base_link`**: This is the main body of the robot.
*   **Two `wheel_link`s**: One for each wheel.
*   **Two `continuous` `joint`s**: Connecting the `base_link` to each `wheel_link`, allowing the wheels to rotate.
*   **Visual properties**: Simple shapes (e.g., box for base, cylinders for wheels) and colors.
*   **Collision properties**: Simplified shapes for physics interactions (e.g., slightly smaller boxes/cylinders than visuals to avoid snagging).

Conceptually, an SDF fragment might look like:

```xml
<model name='simple_robot'>
  <link name='base_link'>
    <inertial>...</inertial>
    <visual name='base_visual'>...</visual>
    <collision name='base_collision'>...</collision>
  </link>
  <link name='left_wheel_link'>...</link>
  <link name='right_wheel_link'>...</link>

  <joint name='left_wheel_joint' type='continuous'>
    <parent>base_link</parent>
    <child>left_wheel_link</child>
    <axis>...</axis>
  </joint>
  <joint name='right_wheel_joint' type='continuous'>
    <parent>base_link</parent>
    <child>right_wheel_link</child>
    <axis>...</axis>
  </joint>
</model>
```

### Conceptual World File Fragment

Now, let's place this robot in a simple world.

*   **A `ground_plane`**: For the robot to sit on.
*   **Some `light`**: To illuminate the scene.
*   **The `simple_robot`**: Spawned at a specific `pose` (position and orientation).
*   **Gravity**: Set to a standard value (e.g., `9.8 m/s^2` downwards).

Conceptually, a `.world` fragment might look like:

```xml
<sdf version='1.7'>
  <world name='empty_world'>
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name='simple_robot'>
      <pose>0 0 0.1 0 0 0</pose> <!-- x y z roll pitch yaw -->
      <!-- ... (contents of simple_robot model definition) ... -->
    </model>

    <physics name='default_physics' default='true' type='ode'>
      <ode>
        <solver>
          <type>quick</type>
          <iters>50</iters>
        </solver>
        <constraints>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <gravity>0 0 -9.8</gravity>
    </physics>
  </world>
</sdf>
```

These fragments illustrate the declarative nature of Gazebo's configuration, where you describe what you want in the simulation, and Gazebo's engine brings it to life. This abstraction allows engineers and researchers to focus on robot behavior and environment design without getting bogged down in low-level rendering or physics engine programming.

## Exercises and Review Questions

1.  Describe the primary function of `gzserver` and `gzclient` in Gazebo's architecture. Why is it beneficial for them to operate separately?
2.  What is a "World" file in Gazebo, and what key elements does it define?
3.  Explain the purpose of SDF files in Gazebo. What kind of information about a robot or object is typically specified in an SDF file?
4.  How do "Plugins" enhance Gazebo's functionality? Provide an example of a type of plugin and its use.
5.  Outline the conceptual steps involved in running a basic robot simulation in Gazebo, assuming you have a robot model and world file ready.
6.  Consider the conceptual SDF fragment for a simple wheeled robot. What would be the implications if the `collision` geometry was exactly the same as the `visual` geometry, especially for complex shapes?
7.  Why is it important for a robotics simulator like Gazebo to be deeply integrated with ROS 2? What advantages does this integration offer to robot developers?

