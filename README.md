## Overview
Physically based simulation of mass-spring systems in Unity (C#), from simple node-and-spring setups to full cloth built from triangle meshes. 
It implements several explicit and implicit integration methods on top of a common multidimensional state formulation, so their stability and behavior can be compared directly.

## Features
**Mass-spring model**
* Nodes with mass, gravity and damping, connected by springs with stiffness and damping
* Analytical force and Jacobian computation (with respect to both position and velocity) for nodes and springs
* Fixed nodes, handled consistently in forces and matrices
* Integration methods:
  * Explicit Euler
  * Symplectic Euler
  * Midpoint
  * Verlet
  * Implicit Euler, solving a linear system every step with the force Jacobians

**Cloth simulation**
* Mass-spring model generated automatically at startup from a triangle mesh (planes, quads...), with one node per vertex
* **Stretch springs** along every edge of the mesh and **bend springs** joining the two opposite vertices of every edge shared by two triangles, with independent stiffness for each type
* Rayleigh damping: a mass-proportional term at the nodes and a stiffness-proportional term at the springs
* Fixers to pin the nodes inside a region, allowing hanging and anchored cloth
* Debug visualization of the model: nodes drawn as spheres and springs as lines, colored by type (cyan for stretch, green for bend)

## Technologies
* Unity (Universal Render Pipeline)
* C#
* Math.NET Numerics

## Usage
* Open the project with Unity
* Open the scene of the example you want to try
* Select the integration method, time step and spring parameters in the `PhysicsManager` and `MassSpring` components
* Pressing **P** in play mode will pause/resume the simulation

## Authorship
I developed the simulation code: the integration steps in `PhysicsManager.cs`, the force and Jacobian computation in `Node.cs` and `Spring.cs`, 
and the mesh-based model creation, bend springs and visual update in `MassSpring.cs`. The rest of the project was provided by the professor and is included only so the project can run.
