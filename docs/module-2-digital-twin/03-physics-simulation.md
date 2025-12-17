---
id: physics-simulation
title: 03 - Physics Simulation
---

# 03 - Physics Simulation

## The Role of Physics in Robotics Simulation

Physics simulation is the backbone of any realistic robotics environment. It dictates how objects move, interact, and respond to forces. In a digital twin, accurate physics simulation is paramount for ensuring that a robot's behavior in the virtual world closely mirrors its behavior in the physical world. This fidelity is crucial for tasks like path planning, grasping, manipulation, and ensuring dynamic stability.

Gazebo, like most advanced simulators, relies on powerful physics engines (such as ODE - Open Dynamics Engine, Bullet, DART, Simbody) to perform these complex calculations. These engines model fundamental physical principles to create a believable simulation.

## Key Physics Concepts in Gazebo

### Gravity

Gravity is a fundamental force that pulls objects towards the center of a celestial body. In Gazebo, gravity is a configurable parameter within the world file.

*   **Impact**: Gravity causes objects to fall, robots to maintain contact with the ground, and influences the dynamics of any projectile motion. Without accurate gravity simulation, a robot's interaction with its environment would be unrealistic (e.g., a robot lifting an object would require the same force regardless of the object's mass if gravity were absent).
*   **Configuration**: Typically set as a 3D vector `(x, y, z)` in meters per second squared (m/s²). For Earth, it's commonly `(0, 0, -9.8)` in a Z-up coordinate system.

### Collisions

Collisions occur when two or more physical objects come into contact. In simulation, this involves detecting when the geometries of objects overlap and then calculating the forces that prevent them from passing through each other.

*   **Collision Geometries**: For performance reasons, simulators often use simplified geometries for collision detection, rather than the highly detailed visual meshes. These `collision` geometries are typically basic primitives (boxes, spheres, cylinders) or convex hulls that approximate the shape of the `visual` mesh.
*   **Contact Points and Normals**: When a collision is detected, the physics engine identifies contact points and the direction (normal) of the collision.
*   **Restitution (Bounciness)**: This property determines how much kinetic energy is conserved during a collision. A restitution of 0 means objects stick together (no bounce), while 1 means a perfectly elastic collision (maximum bounce).
*   **Penetration**: Due to the discrete nature of simulation steps, objects can sometimes "penetrate" slightly into each other. Physics engines employ various algorithms to resolve this penetration and apply corrective forces to separate the objects.

### Friction

Friction is a force that opposes relative motion or attempted motion between two surfaces in contact. It's essential for simulating realistic interactions, such as a robot's wheels gripping the ground or an end-effector holding an object.

*   **Static Friction**: The force that prevents objects from moving relative to each other when they are in contact and at rest.
*   **Dynamic (Kinetic) Friction**: The force that opposes the motion of objects sliding past each other.
*   **Friction Coefficients**: These dimensionless values quantify the amount of friction between two surfaces.
    *   **Mu1 and Mu2**: Represent the coefficients of friction in two orthogonal directions along the contact plane (e.g., x and y axes relative to the contact normal). These are crucial for modeling anisotropic friction (friction that varies with direction), though often set to be equal for simplicity.
*   **Impact**: Friction allows robots to accelerate, decelerate, turn, and manipulate objects effectively. Without friction, robots would slide uncontrollably, and grippers would be unable to hold anything.

```mermaid
graph TD
    A[Object Mass & Volume] --> B{Gravity};
    B --> C{Force of Impact};

    D[Object Geometry (Collision Mesh)] --> C;
    C --> E{Collision Detection & Resolution};

    F[Surface Properties] --> G{Friction (Static & Kinetic)};
    G --> E;

    E --> H[Simulated Motion & Interaction];
```
## Conceptual Examples in Gazebo

### Gravity's Effect

Imagine a simple box model spawned above a ground plane in Gazebo. When the simulation starts, the box will accelerate downwards due to gravity until it collides with the ground.

```xml
<world name='gravity_test_world'>
  <gravity>0 0 -9.8</gravity> <!-- Earth's gravity -->
  <model name='falling_box'>
    <link name='box_link'>
      <inertial>
        <mass>1.0</mass>
        <pose>0 0 0 0 0 0</pose>
        <inertia>...</inertia>
      </inertial>
      <visual name='visual'>...</visual>
      <collision name='collision'>...</collision>
    </link>
    <pose>0 0 5 0 0 0</pose> <!-- Spawn 5 meters above ground -->
  </model>
  <include><uri>model://ground_plane</uri></include>
</world>
```
When this simulation runs, the box will drop, demonstrating the effect of gravity.

### Collision Resolution

Consider two simple sphere models. If one sphere is launched towards another, the physics engine will detect their collision. Depending on their `restitution` properties, they might bounce off each other (high restitution) or come to a stop after impact (low restitution). The simplified `collision` geometries are used for this, ensuring fast and stable calculations.

### Friction in Action

A wheeled robot trying to move on a very low-friction surface (like ice, `mu1=0.01`, `mu2=0.01`) would struggle to gain traction, leading to wheel slippage and difficulty in steering. Conversely, on a high-friction surface (like rubber on concrete, `mu1=0.8`, `mu2=0.8`), the robot would respond precisely to wheel commands.

```xml
<surface>
  <friction>
    <ode>
      <mu>0.8</mu> <!-- Simplified, often mu1 and mu2 are set -->
      <mu2>0.8</mu2>
    </ode>
  </friction>
</surface>
```
These properties are typically defined within the `surface` element of a link's `collision` geometry in SDF.

<h2>Importance of Tuning Physics Parameters</h2>

Tuning these physics parameters (gravity, collision properties, friction coefficients) is crucial for creating a "realistic" simulation. Incorrect values can lead to unstable simulations (e.g., objects vibrating or exploding), unrealistic behavior (e.g., robots sliding too much), or simply a mismatch between simulated and real-world performance. Achieving high fidelity often involves iterative tuning and comparison with real-world data.

<h2>Exercises and Review Questions</h2>

<ol>
  <li>Explain why accurate physics simulation is crucial for a robot's digital twin.</li>
  <li>Describe how gravity is typically configured in Gazebo and its impact on simulated objects.</li>
  <li>What is the difference between <code>visual</code> and <code>collision</code> geometries in Gazebo models, and why are simplified collision geometries often preferred?</li>
  <li>Define <code>restitution</code> in the context of collisions. What would a restitution value of 0 imply for two colliding objects?</li>
  <li>Distinguish between static and dynamic friction. How do friction coefficients (<code>mu1</code>, <code>mu2</code>) influence the behavior of a wheeled robot in a simulation?</li>
  <li>You are simulating a humanoid robot trying to walk on a smooth, slippery surface. What physics parameter would you adjust to model this scenario, and how would it affect the robot's motion?</li>
  <li>Why is iterative tuning of physics parameters important for creating a realistic simulation? What are the potential consequences of using incorrect parameter values?</li>
</ol>

<h2>Conceptual Examples in Gazebo</h2>

<h3>Gravity's Effect</h3>
Imagine a simple box model spawned above a ground plane in Gazebo. When the simulation starts, the box will accelerate downwards due to gravity until it collides with the ground.

<h3>Collision Resolution</h3>
Consider two simple sphere models. If one sphere is launched towards another, the physics engine will detect their collision. Depending on their restitution properties, they might bounce off each other (high restitution) or come to a stop after impact (low restitution).

<h3>Friction in Action</h3>
A wheeled robot trying to move on a very low-friction surface (like ice) would struggle to gain traction, leading to wheel slippage and difficulty in steering. Conversely, on a high-friction surface, the robot would respond precisely to wheel commands.