# Feature Specification: Chapter 02 - ROS 2 Fundamentals

**Feature Branch**: `002-chapter-02-ros2-fundamentals`
**Created**: 2025-11-28
**Status**: Draft
**Input**: User description: "Let's implement chapter 2. use context7 mcp server to find the relevant information. also the context of the book is in this overview. @context\physical_ai_book.md . Always use relevant subagents and skills and generate content inline with constitution"

## Overview

This specification defines the educational content for Chapter 02 of the "Physical AI & Humanoid Robotics" book. The chapter introduces students to ROS 2 (Robot Operating System 2)—the middleware that serves as the "nervous system" connecting all components of a robot. This is the first hands-on technical chapter where students install ROS 2, execute commands, and create their first nodes.

### Chapter Context

| Attribute | Value |
|-----------|-------|
| **Chapter** | 2 |
| **Part** | 1 - The Robotic Nervous System (ROS 2) |
| **Week** | 2 |
| **Proficiency** | A2 (Beginner) |
| **Hardware Tier** | Simulation Only (any computer + Ubuntu 22.04) |
| **Prerequisites** | Chapter 1 (Physical AI concepts), Python fundamentals |

### Learning Objectives

By completing this chapter, students will be able to:

1. **LO-01**: Explain what ROS 2 is and why it's essential for robot development
2. **LO-02**: Install and configure ROS 2 Humble on Ubuntu 22.04
3. **LO-03**: Create and execute basic ROS 2 nodes using the CLI
4. **LO-04**: Understand the ROS 2 graph concept (nodes, topics, services)
5. **LO-05**: Write a simple Python publisher and subscriber using rclpy
6. **LO-06**: Debug common ROS 2 connection issues using CLI tools

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Understanding ROS 2's Role (Priority: P1)

A student with Python fundamentals but no robotics middleware experience learns what ROS 2 is, why robots need middleware, and how ROS 2 solves the communication problem between robot components (sensors, actuators, planning algorithms).

**Why this priority**: This is the foundational mental model for all future ROS work. Without understanding WHY ROS 2 exists, students will view it as "arbitrary complexity" rather than "essential architecture."

**Independent Test**: Student can explain to a peer why a robot needs middleware like ROS 2 (not just Python scripts), and identify at least 3 problems ROS 2 solves (inter-process communication, language interoperability, distributed computing).

**Acceptance Scenarios**:

1. **Given** a student understands basic Python programming, **When** they complete the "What is ROS 2?" lesson, **Then** they can articulate that ROS 2 is middleware connecting robot components (not an operating system)
2. **Given** a student sees a robot system diagram, **When** they identify components, **Then** they can explain how ROS 2 enables these components to communicate regardless of programming language or physical location
3. **Given** a student considers building a robot without ROS 2, **When** they analyze the challenges, **Then** they can list at least 3 problems ROS 2 solves (process communication, timing synchronization, message serialization)

---

### User Story 2 - Installing and Configuring ROS 2 (Priority: P1)

A student follows step-by-step instructions to install ROS 2 Humble on Ubuntu 22.04, configure their environment, and verify the installation is working correctly.

**Why this priority**: Cannot proceed with any hands-on ROS work without successful installation. This is the critical gate for all subsequent lessons.

**Independent Test**: Student runs `ros2 --version` and sees Humble output, can source the ROS setup file, and successfully launches `turtlesim` demo.

**Acceptance Scenarios**:

1. **Given** a student has Ubuntu 22.04 installed, **When** they follow the installation guide, **Then** they successfully install ROS 2 Humble using apt packages
2. **Given** a student completes installation, **When** they run `ros2 --version`, **Then** the system outputs "ros2 cli version: humble" (or similar)
3. **Given** a student sources the ROS 2 environment, **When** they run `ros2 run turtlesim turtlesim_node`, **Then** the turtlesim window appears without errors
4. **Given** a student encounters installation errors, **When** they follow troubleshooting steps, **Then** they can resolve common issues (missing dependencies, incorrect sources.list)

---

### User Story 3 - Exploring the ROS 2 Graph with CLI Tools (Priority: P2)

A student uses ROS 2 command-line tools (`ros2 node`, `ros2 topic`, `ros2 service`) to explore a running robot system (turtlesim), building an intuitive understanding of the ROS 2 graph structure.

**Why this priority**: Hands-on discovery builds mental models more effectively than reading. Students learn by DOING (executing commands, observing outputs) before coding.

**Independent Test**: Student can launch turtlesim, use CLI tools to list nodes/topics/services, and explain what they observe using correct ROS terminology.

**Acceptance Scenarios**:

1. **Given** turtlesim is running, **When** student executes `ros2 node list`, **Then** they see `/turtlesim` node and can explain what a node is
2. **Given** student runs `ros2 topic list`, **When** they see `/turtle1/cmd_vel` and `/turtle1/pose`, **Then** they can distinguish between command (input) and state (output) topics
3. **Given** student executes `ros2 topic echo /turtle1/pose`, **When** they observe streaming data, **Then** they understand topics are continuous data streams (not request-response)
4. **Given** student runs `ros2 service list`, **When** they see `/spawn` service, **Then** they understand services are for on-demand operations (not continuous streaming)

---

### User Story 4 - Creating First ROS 2 Node with Python (Priority: P2)

A student writes their first ROS 2 Python node using rclpy, understanding node initialization, spinning (message processing loop), and proper shutdown.

**Why this priority**: Transitioning from CLI exploration to code creation. Students apply their mental model (from Story 3) to write actual nodes.

**Independent Test**: Student writes a minimal "Hello ROS 2" node that prints a message every second, runs it successfully, and can explain each line of code.

**Acceptance Scenarios**:

1. **Given** a student has rclpy installed, **When** they create a minimal node with `rclpy.init()` and `rclpy.spin()`, **Then** the node runs without errors and responds to Ctrl+C shutdown
2. **Given** a student adds a timer callback, **When** the node runs, **Then** the callback executes periodically (e.g., prints "Hello ROS 2" every second)
3. **Given** a student reviews the node code, **When** asked about each section, **Then** they can explain: initialization, node creation, timer setup, spinning, and cleanup

---

### User Story 5 - Building Publisher-Subscriber Communication (Priority: P1)

A student creates a publisher node that sends messages to a topic, and a subscriber node that receives those messages, understanding the fundamental ROS 2 communication pattern.

**Why this priority**: Pub-sub is THE core ROS 2 pattern. This demonstrates how separate nodes communicate—the foundation for all distributed robot systems.

**Independent Test**: Student creates two separate nodes (publisher.py and subscriber.py), runs both simultaneously, and observes messages flowing from publisher to subscriber.

**Acceptance Scenarios**:

1. **Given** a student writes a publisher node, **When** they run `ros2 topic echo /chatter` in another terminal, **Then** they see the messages being published
2. **Given** a student writes a subscriber node, **When** they run both publisher and subscriber, **Then** the subscriber receives and prints the publisher's messages
3. **Given** a student reviews the publisher code, **When** asked about `self.create_publisher()` parameters, **Then** they can explain message type, topic name, and queue size
4. **Given** a student reviews the subscriber code, **When** asked about callbacks, **Then** they can explain that the callback function executes automatically when messages arrive

---

### User Story 6 - Understanding Services for Request-Response (Priority: P3)

A student learns the difference between topics (continuous streams) and services (on-demand request-response), and creates a simple service that responds to client requests.

**Why this priority**: Completes the basic ROS 2 communication patterns (topics for streaming, services for requests). Lower priority than pub-sub since services are less common in basic robots.

**Independent Test**: Student writes a service node that responds to addition requests (inputs: two numbers, output: sum) and a client node that calls the service.

**Acceptance Scenarios**:

1. **Given** a student has used topics extensively, **When** they learn about services, **Then** they can explain when to use topics vs services (continuous data vs on-demand requests)
2. **Given** a student writes a service server, **When** they run `ros2 service call /add_two_ints ...`, **Then** the service responds with the correct sum
3. **Given** a student writes a service client, **When** the client calls the service, **Then** the client receives and prints the response

---

### Edge Cases

- **What happens when** a student doesn't have Ubuntu 22.04? → Provide Docker alternative or Windows WSL2 setup guide (in appendix, not main flow)
- **How does system handle** students who cannot install ROS 2 due to system constraints? → Provide cloud-based alternative (e.g., AWS RoboMaker, GitHub Codespaces with ROS preinstalled)
- **What happens when** a student's ROS nodes fail to communicate? → Debugging lesson covers common issues: environment not sourced, incorrect topic names, mismatched message types, DDS configuration issues
- **What happens when** a student runs multiple nodes with the same name? → ROS 2 handles this gracefully with automatic name remapping; lesson demonstrates this behavior
- **How does system handle** Python version mismatches? → Installation guide specifies Python 3.10+ required for Humble, with verification step

## Requirements *(mandatory)*

### Functional Requirements

#### Content Requirements

- **FR-001**: Chapter MUST explain what ROS 2 is (middleware for robot communication) and why robots need it (inter-process communication, language interoperability, distributed systems)
- **FR-002**: Chapter MUST provide step-by-step installation instructions for ROS 2 Humble on Ubuntu 22.04 using apt packages
- **FR-003**: Chapter MUST include verification steps confirming successful ROS 2 installation (version check, turtlesim demo)
- **FR-004**: Chapter MUST teach ROS 2 CLI tools for exploring the graph (`ros2 node`, `ros2 topic`, `ros2 service`, `ros2 interface`)
- **FR-005**: Chapter MUST include hands-on exercises using turtlesim to demonstrate nodes, topics, and services
- **FR-006**: Chapter MUST teach creating ROS 2 Python nodes using rclpy (initialization, spinning, shutdown)
- **FR-007**: Chapter MUST teach publisher-subscriber pattern with complete working example code
- **FR-008**: Chapter MUST teach service-client pattern with complete working example code
- **FR-009**: Chapter MUST include debugging section covering common ROS 2 issues (environment not sourced, topic name mismatches, message type errors)
- **FR-010**: All Python code examples MUST be tested on ROS 2 Humble with Ubuntu 22.04

#### Pedagogical Requirements

- **FR-011**: Chapter MUST follow A2 proficiency guidelines (5-7 concepts per section, heavy scaffolding, step-by-step instructions)
- **FR-012**: Chapter MUST use hands-on discovery teaching modality (execute → observe → understand), varying from Chapter 1's direct conceptual teaching
- **FR-013**: Chapter MUST follow Layer 1 → Layer 2 progression:
  - **Lessons 1-3**: Layer 1 (Manual foundation—students execute ROS commands, NO AI collaboration)
  - **Lessons 4-6**: Layer 2 (AI collaboration—students create nodes with AI assistance, debugging with AI)
- **FR-014**: Chapter MUST end lessons with "Try With AI" sections for AI-assisted exploration (Layer 2 lessons only)
- **FR-015**: Chapter MUST connect all ROS concepts to robot behavior (NOT generic programming—always explain "what does this do for the robot?")
- **FR-016**: Chapter MUST NOT include advanced ROS topics (actions, parameters, launch files—defer to Chapter 3)

#### Constitutional Compliance

- **FR-017**: Chapter MUST state hardware tier explicitly: "Simulation Only—any computer with Ubuntu 22.04"
- **FR-018**: Chapter MUST provide cloud alternative for students without local Ubuntu installation
- **FR-019**: Chapter MUST follow Specification Primacy—show WHAT nodes do before HOW to code them
- **FR-020**: Chapter MUST demonstrate Three Roles framework in Layer 2 lessons:
  - AI teaches student (suggests ROS patterns student didn't know)
  - Student teaches AI (corrects outdated APIs or hardware constraints)
  - Convergence loop (iterative refinement toward working node)

#### Technical Accuracy Requirements

- **FR-021**: All ROS 2 commands MUST be verified on Humble distribution (not Rolling or other versions)
- **FR-022**: All rclpy code MUST use current API (no deprecated methods like `Node.get_logger().info()` should be presented as primary)
- **FR-023**: All Python code MUST include proper imports, initialization, and cleanup
- **FR-024**: All example nodes MUST follow ROS 2 best practices (meaningful node names, appropriate QoS settings for basic use cases, proper exception handling)

### Key Entities

- **ROS 2 Humble**: The specific ROS 2 distribution used (Long-Term Support release for Ubuntu 22.04)
- **Node**: Single-purpose, modular executable that performs computation (e.g., sensor driver, motion planner)
- **Topic**: Named channel for continuous asynchronous message streaming (pub-sub pattern)
- **Publisher**: Node component that sends messages to a topic
- **Subscriber**: Node component that receives messages from a topic
- **Service**: Request-response communication pattern for on-demand operations
- **Message Type**: Data structure definition for messages (e.g., `std_msgs/String`, `geometry_msgs/Twist`)
- **ROS 2 Graph**: Network of nodes and their communication connections (topics, services, actions)
- **rclpy**: ROS 2 client library for Python (equivalent to rospy in ROS 1)
- **DDS (Data Distribution Service)**: Underlying middleware that ROS 2 uses for communication (students learn concept, not details)
- **turtlesim**: Simple robot simulator used for hands-on learning (like "Hello World" for ROS)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can successfully install ROS 2 Humble and pass verification steps (version check + turtlesim launch) on first attempt with 85%+ success rate
- **SC-002**: Students can use CLI tools to explore a running ROS system and correctly identify at least 3/4 components (nodes, topics, services, message types)
- **SC-003**: Students can create a minimal ROS 2 Python node that runs without errors on first or second attempt (after debugging)
- **SC-004**: Students can create a working publisher-subscriber pair that successfully exchanges messages, demonstrating understanding of the pub-sub pattern
- **SC-005**: Students can explain the difference between topics and services with 90%+ accuracy on a concept quiz
- **SC-006**: Chapter completion time is under 6 hours for average A2 learner (includes installation time)
- **SC-007**: 80%+ of students report feeling confident to create basic ROS nodes after chapter completion

### Content Quality Metrics

- **SC-008**: All ROS 2 code examples execute without errors on Ubuntu 22.04 with ROS 2 Humble (verified through testing)
- **SC-009**: Zero technical inaccuracies (verified against official ROS 2 documentation via Context7)
- **SC-010**: Every section maps to at least one learning objective (no tangential content)
- **SC-011**: Cognitive load per section stays within A2 limits (5-7 concepts maximum)
- **SC-012**: All installation steps include error handling / troubleshooting for common issues
- **SC-013**: All CLI commands include expected output examples so students can verify correct execution

### Teaching Effectiveness Metrics

- **SC-014**: Each Layer 2 lesson (4-6) demonstrates all Three Roles (AI teaches, student teaches, convergence)
- **SC-015**: Debugging section enables 85%+ of students to resolve common ROS issues independently
- **SC-016**: Hands-on exercises result in 80%+ task completion rate without instructor intervention

## Assumptions

1. **Operating System**: Students have access to Ubuntu 22.04 (native, dual-boot, VM, or WSL2) or can use cloud alternative
2. **Python Version**: Students have Python 3.10+ installed (default with Ubuntu 22.04)
3. **Internet Access**: Students can download ROS 2 packages (~2 GB) and access ROS documentation
4. **System Resources**: Students have at least 4 GB RAM and 10 GB free disk space for ROS 2 installation
5. **Prerequisite Knowledge**: Students completed Chapter 1 and understand Physical AI concepts, Sim-to-Real transfer, and basic robot architecture
6. **Python Fundamentals**: Students know Python basics (variables, functions, classes, imports, basic OOP concepts like `self`)
7. **Terminal Comfort**: Students can navigate terminal, execute commands, edit text files (minimal Linux experience assumed)
8. **AI Access**: Students have access to AI assistant (Claude/ChatGPT) for "Try With AI" sections in Layer 2 lessons
9. **No Prior ROS Experience**: Chapter assumes zero ROS 1 or ROS 2 knowledge; all concepts explained from scratch
10. **Development Environment**: Students can use any code editor (VS Code recommended but not required)

## Non-Goals (Explicit Exclusions)

- Teaching advanced ROS 2 concepts (parameters, actions, lifecycle nodes, component nodes—covered in Chapter 3)
- Creating launch files (covered in Chapter 3 or 4)
- Building custom message types (covered in Chapter 3)
- Setting up colcon workspaces and building packages from source (covered in Chapter 3)
- Teaching C++ for ROS 2 (book focuses on Python only)
- Deep dive into DDS configuration (students learn concept only)
- Teaching ROS 1 or migration from ROS 1 to ROS 2 (ROS 2 only)
- Installing ROS 2 on Windows natively or macOS (Ubuntu focus, cloud alternative provided)
- Real-time operating systems or real-time ROS extensions (out of scope)
- Security features, encryption, or access control (advanced topics)
- Performance optimization or benchmarking (premature at A2 level)

## Constraints

### Pedagogical Constraints

- **A2 Proficiency**: Maximum 5-7 concepts per section with heavy scaffolding
- **No Prerequisites Beyond Chapter 1**: Cannot assume any ROS knowledge; all concepts explained from first principles
- **Layer Progression**: L1 (Manual, Lessons 1-3) → L2 (AI Collaboration, Lessons 4-6); NO L3/L4 in this chapter
- **Single Closing Section**: Lessons end ONLY with "Try With AI" (no "What's Next", "Summary", "Key Takeaways")
- **Hands-On Discovery Modality**: Students learn by executing commands and observing results (not just reading explanations)

### Technical Constraints

- **ROS 2 Humble Only**: Cannot use Rolling or other distributions (Humble is LTS for Ubuntu 22.04)
- **Python Only**: No C++ code examples (book is Python-focused)
- **Simulation Only**: No physical robot hardware required (any computer sufficient)
- **Ubuntu 22.04 Focus**: Installation instructions target Ubuntu 22.04 specifically (cloud alternative for other OSes)
- **Standard DDS**: Use default CycloneDDS configuration (no custom DDS settings)

### Constitutional Constraints

- **Specification Primacy**: Show WHAT nodes do (behavior) before HOW to code them
- **Factual Accuracy**: All ROS commands and APIs verified against official documentation
- **Minimal Content**: Every section must map to learning objectives; remove tangential content
- **Anti-Convergence**: Teaching modality must differ from Chapter 1 (which used direct conceptual teaching)
- **Hardware Tier Explicit**: Must state "Simulation Only" and provide cloud alternatives

### Resource Constraints

- **Installation Size**: ~2 GB download for ROS 2 Humble (document bandwidth requirements)
- **Disk Space**: Minimum 10 GB free space required
- **RAM**: Minimum 4 GB (8 GB recommended for comfortable development)
- **Internet**: Required for installation and optional cloud-based alternatives

## Open Questions / Clarifications Needed

*[Clarification questions removed—proceeding with informed defaults documented in Assumptions]*
