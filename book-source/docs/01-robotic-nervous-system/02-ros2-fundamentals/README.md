---
title: "Chapter 2: ROS 2 Fundamentals"
sidebar_position: 2
---

# Chapter 2: ROS 2 Fundamentals

**Part 1**: The Robotic Nervous System (ROS 2)
**Week**: 2
**Proficiency**: A2 (Beginner)
**Hardware Tier**: Simulation Only (any computer + Ubuntu 22.04)
**Estimated Time**: 9 hours

---

## Overview

This chapter introduces you to ROS 2 (Robot Operating System 2)—the middleware framework that connects all components of a modern robot. You'll move from understanding ROS 2 conceptually to installing it, exploring it with command-line tools, and creating your own nodes that communicate via topics and services.

By the end of this chapter, you'll be able to build the communication infrastructure for a robot system, creating nodes that sense, process, and act in a coordinated manner.

---

## Learning Objectives

By completing this chapter, you will be able to:

1. **Explain** what ROS 2 is and why robots need middleware (LO-01)
2. **Install and configure** ROS 2 Humble on Ubuntu 22.04 (LO-02)
3. **Create and execute** basic ROS 2 nodes using the CLI (LO-03)
4. **Understand** the ROS 2 graph concept (nodes, topics, services) (LO-04)
5. **Write** Python publisher and subscriber nodes using rclpy (LO-05)
6. **Debug** common ROS 2 connection issues using CLI tools (LO-06)

---

## Chapter Structure

This chapter follows a **Layer 1 → Layer 2 progression**:
- **Lessons 1-3** (Layer 1): Manual foundation—understand concepts, install tools, explore with CLI
- **Lessons 4-6** (Layer 2): AI collaboration—create nodes, implement communication patterns, debug with AI assistance

### Lessons

| Lesson | Title | Time | Layer | Concepts |
|--------|-------|------|-------|----------|
| **1** | What is ROS 2 and Why Robots Need Middleware | 45 min | L1 | 3 |
| **2** | Installing ROS 2 Humble | 60 min | L1 | 2 |
| **3** | Exploring the ROS 2 Graph with CLI Tools | 90 min | L1 | 5 |
| **4** | Create Your First Python Node | 120 min | L2 | 4 |
| **5** | Publisher-Subscriber Pattern | 150 min | L2 | 5 |
| **6** | Services (Request-Response Communication) | 120 min | L2 | 3 |

**Total**: ~9 hours (585 minutes)

---

## Prerequisites

Before starting this chapter, you should have:

- ✅ Completed **Chapter 1** (Physical AI concepts)
- ✅ **Ubuntu 22.04** installed (native, WSL2, or VM)
- ✅ **Python fundamentals** (variables, functions, classes)
- ✅ **Terminal comfort** (navigate directories, execute commands)
- ✅ **Internet access** (~2 GB download for ROS 2 installation)
- ✅ **System resources**: 4+ GB RAM, 10+ GB free disk space

---

## What You'll Build

By the end of this chapter, you'll have created:

### Lesson 1 Outcomes
- Mental model of ROS 2 middleware architecture
- Understanding of 3 key problems ROS 2 solves

### Lesson 2 Outcomes
- Working ROS 2 Humble installation
- Verified environment with turtlesim demo

### Lesson 3 Outcomes
- Proficiency with `ros2 node`, `ros2 topic`, `ros2 service` CLI tools
- Ability to explore any ROS 2 system independently

### Lesson 4 Outcomes
- Your first ROS 2 Python node with timer callbacks
- Understanding of node lifecycle (init → spin → shutdown)

### Lesson 5 Outcomes
- Publisher node broadcasting messages
- Subscriber node receiving messages
- Working pub-sub communication system

### Lesson 6 Outcomes
- Service server responding to requests
- Service client making requests
- Understanding of request-response pattern

---

## Key Concepts Introduced

### Core ROS 2 Concepts (22 total)

**Lesson 1** (3 concepts):
- ROS 2 middleware
- Problems ROS 2 solves (inter-process communication, language interoperability, distributed computing)
- ROS 2 vs Operating System distinction

**Lesson 2** (2 concepts):
- ROS 2 Humble distribution
- Environment setup (`source /opt/ros/humble/setup.bash`)

**Lesson 3** (5 concepts):
- Nodes
- Topics
- Topic direction (command vs state)
- Services
- ROS 2 graph structure

**Lesson 4** (4 concepts):
- rclpy initialization (`rclpy.init()`)
- Node class inheritance
- Spinning (`rclpy.spin()`)
- Timer callbacks

**Lesson 5** (5 concepts):
- Publisher creation (`create_publisher`)
- Subscriber creation (`create_subscription`)
- Message types (`std_msgs`, `geometry_msgs`, etc.)
- Topic naming conventions
- Queue size implications

**Lesson 6** (3 concepts):
- Service concept (request-response vs streaming)
- Service server (`create_service`)
- Service client (`create_client`, async calls)

---

## Teaching Approach

### Hands-On Discovery Modality

This chapter uses **execute → observe → understand** teaching:

1. **Execute commands first** — Run ROS 2 tools and see output
2. **Observe what happens** — Notice patterns, behaviors, errors
3. **Understand why** — Learn the underlying concepts

This differs from Chapter 1's direct conceptual teaching and builds intuition through exploration.

### Layer Progression

**Layer 1 (Lessons 1-3)**: Manual foundation
- Direct teaching by the book
- No AI collaboration yet
- Focus on building mental models and CLI proficiency
- Minimal "Try With AI" sections (conceptual exploration only)

**Layer 2 (Lessons 4-6)**: AI collaboration
- Students create nodes with AI assistance
- Three Roles framework demonstrated through natural narrative:
  - **AI suggests** patterns you might not discover independently
  - **You refine** AI recommendations based on your constraints
  - **Together you converge** on solutions balancing best practices with practical needs
- "Try With AI" sections provide concrete prompts for exploration

---

## Success Criteria

You've successfully completed this chapter when you can:

- ✅ **Install ROS 2 Humble** and verify with turtlesim (85%+ success rate)
- ✅ **Use CLI tools** to identify nodes, topics, and services in a running system (3/4 components correctly identified)
- ✅ **Create a minimal ROS 2 node** that runs without errors (first or second attempt after debugging)
- ✅ **Build a pub-sub system** where two nodes successfully exchange messages
- ✅ **Explain topics vs services** with 90%+ accuracy on concept quiz
- ✅ **Complete chapter in <6 hours** (average A2 learner, includes installation)
- ✅ **Feel confident** creating basic ROS nodes (80%+ self-reported confidence)

---

## Common Pitfalls and How to Avoid Them

### Pitfall 1: Environment Not Sourced

**Problem**: Running ROS 2 commands gives "command not found"

**Solution**: Always source the setup file:
```bash
source /opt/ros/humble/setup.bash
```

Make it automatic:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Pitfall 2: Topic/Service Names Don't Match

**Problem**: Publisher and subscriber run without errors, but no communication

**Solution**: Verify names match exactly:
```bash
ros2 topic list  # Check publisher's topic name
ros2 topic info /topic_name  # Verify pub/sub counts
```

### Pitfall 3: Forgetting to Return Response in Service

**Problem**: Service client hangs indefinitely

**Solution**: Always return the response object:
```python
def service_callback(self, request, response):
    # ... process request ...
    return response  # REQUIRED
```

### Pitfall 4: Not Calling rclpy.init()

**Problem**: RuntimeError when creating nodes

**Solution**: Always call `rclpy.init()` before creating any nodes:
```python
def main(args=None):
    rclpy.init(args=args)  # REQUIRED FIRST
    node = MyNode()
    # ...
```

---

## Next Chapter Preview

**Chapter 3: ROS 2 Communication Patterns** will build on this foundation:
- **Parameters**: Dynamic configuration without restarting nodes
- **Actions**: Long-running tasks with feedback (e.g., navigation)
- **Launch files**: Start multiple nodes with one command
- **Custom message types**: Define your own data structures
- **Quality of Service (QoS)**: Tune reliability and performance

You'll move from basic nodes to production-grade robot systems.

---

## Hardware Requirements

### Required (This Chapter)
- **Computer**: Any with Ubuntu 22.04
- **RAM**: 4+ GB (8 GB recommended)
- **Disk**: 10+ GB free space
- **Internet**: ~2 GB download for installation

### Optional Alternatives
- **WSL2** (Windows): Ubuntu 22.04 on Windows Subsystem for Linux
- **VM** (VirtualBox/VMware): Ubuntu 22.04 virtual machine
- **Cloud** (AWS RoboMaker, GitHub Codespaces): Remote development environment

### Not Required
- ❌ Physical robot hardware
- ❌ GPU/RTX graphics card
- ❌ Sensors (cameras, lidars)
- ❌ Motors or actuators

**This chapter is 100% simulation-based** — any computer with Ubuntu 22.04 is sufficient.

---

## Getting Help

### Debugging Resources

1. **ROS 2 CLI tools**:
   ```bash
   ros2 node list          # Check nodes
   ros2 topic list         # Check topics
   ros2 topic echo /topic  # See messages
   ros2 service list       # Check services
   ```

2. **Official documentation**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)

3. **"Try With AI" sections**: Each lesson includes prompts for exploring concepts with AI assistance

4. **Troubleshooting sections**: Every lesson includes "Common Issues" with solutions

### When Stuck

1. Check if ROS environment is sourced
2. Verify exact topic/service names match
3. Use `ros2 topic/service/node` commands to inspect system
4. Review lesson's "Debugging" section
5. Ask AI assistant with specific error messages

---

## Let's Begin!

Start with [Lesson 1: What is ROS 2 and Why Robots Need Middleware](./lesson-1-what-is-ros2.md).

By the end of this chapter, you'll have the foundational skills to build the communication layer for any robot system—from simple mobile robots to complex humanoids.
