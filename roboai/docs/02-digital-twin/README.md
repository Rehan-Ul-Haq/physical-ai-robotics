---
title: "Part 2: The Digital Twin"
sidebar_position: 0
description: "Build and test robots in simulation before physical deployment. Master URDF modeling, Gazebo physics, and Unity visualization."
---

# Part 2: The Digital Twin (Gazebo & Unity)

**Weeks 6-7 | Hardware: Simulation Only**

---

## What You'll Learn

A **Digital Twin** is a virtual replica of a physical robot—identical in shape, sensors, and behavior. Before risking expensive hardware, you develop, test, and validate in simulation.

By the end of Part 2, you will:

- **Model robots with URDF** — Define links, joints, sensors, and collision geometry
- **Simulate in Gazebo** — Create physics-accurate worlds for robot testing
- **Visualize in Unity** — Build high-fidelity visualizations and HRI scenarios
- **Bridge simulation to ROS 2** — Get sensor data and control robots via ROS

---

## Chapter Overview

| Chapter | Title | Week | Key Topics |
|---------|-------|------|------------|
| 6 | Robot Modeling with URDF | 6 | Links, joints, sensors, collision, RViz2 |
| 7 | Gazebo Simulation | 7 | Worlds, physics, sensors, ROS integration |
| 8 | Unity Visualization | 7 | High-fidelity rendering, HRI, ROS-TCP |

---

## Why Simulation First?

| Risk | Physical Robot | Simulation |
|------|----------------|------------|
| **Cost of failure** | Expensive repairs | Free reset |
| **Iteration speed** | Hours/days | Seconds |
| **Safety** | Real-world hazards | No physical danger |
| **Reproducibility** | Variable conditions | Perfect control |
| **Scale** | One robot at a time | Thousands parallel |

**The mantra:** *Develop in simulation, deploy to reality.*

---

## The Digital Twin Stack

```
┌─────────────────────────────────────┐
│          Unity (Visualization)       │  ← High-fidelity rendering
├─────────────────────────────────────┤
│          Gazebo (Physics)            │  ← Physics simulation
├─────────────────────────────────────┤
│          URDF/SDF (Description)      │  ← Robot model definition
├─────────────────────────────────────┤
│          ROS 2 (Communication)       │  ← From Part 1
└─────────────────────────────────────┘
```

---

## Hardware Requirements

Part 2 continues to work in simulation only:

- Same requirements as Part 1
- Additional disk space for Gazebo worlds
- Unity is optional (Chapter 8)

---

## Learning Path

```
Week 6: Robot Description
  └── Chapter 6: URDF syntax, links, joints, sensors

Week 7: Simulation Environments
  ├── Chapter 7: Gazebo worlds, physics, ROS integration
  └── Chapter 8: Unity visualization (optional)
```

---

## The Sim-to-Real Preview

While Part 2 focuses on simulation, everything you build here prepares you for **Sim-to-Real transfer** in Part 3. The robots you model, the sensors you simulate, and the behaviors you test—all will transfer to physical systems.
