---
title: Exploring the ROS 2 Graph with CLI Tools
chapter: 2
lesson: 3
learning_objectives:
  - Use ros2 node commands to discover and inspect nodes
  - Use ros2 topic commands to explore message streams
  - Distinguish between command topics (inputs) and state topics (outputs)
  - Use ros2 service commands to discover available services
  - Understand the ROS 2 graph concept (nodes, topics, services)
estimated_time: 90 minutes
skills:
  ros2_cli:
    proficiency: A2
  system_exploration:
    proficiency: A2
generated_by: content-implementer v1.1.0
source_spec: specs/002-chapter-02-ros2-fundamentals/spec.md
created: 2025-11-28
last_modified: 2025-11-28
workflow: /sp.implement
version: 1.0.0
---

# Exploring the ROS 2 Graph with CLI Tools

You've installed ROS 2. Now it's time to understand how it actually works by exploring a live robot system. In this lesson, you'll use **command-line tools** to discover nodes, topics, and services in action.

We'll use **turtlesim**—the same demo you verified in the installation lesson—as our robot system. Turtlesim is simple enough to understand quickly, but sophisticated enough to demonstrate all the core ROS 2 concepts.

**Teaching approach**: You'll **execute commands first**, **observe the output**, and **then** understand what it means. This hands-on discovery builds intuition better than reading theory.

## Launch Turtlesim

First, get the system running. Open a terminal and start the turtlesim node:

```bash
ros2 run turtlesim turtlesim_node
```

**Expected result**: A window appears with a turtle in the center.

**Leave this running** and open a **second terminal** for exploration.

## Discovering Nodes

### What is a Node?

A **node** is a single-purpose executable in ROS 2. Think of it as one module in your robot system—for example:
- A camera driver node
- An object detection node
- A motion planning node
- A motor controller node

Each node does one job. Complex robot behavior emerges from many simple nodes working together.

### Command: ros2 node list

In your **second terminal**, list all running nodes:

```bash
ros2 node list
```

**Expected output**:
```
/turtlesim
```

**What you're seeing**: Right now, only one node is running—the turtlesim simulator. Its name is `/turtlesim`.

**Observation**: Node names start with `/`. This is ROS 2's namespace convention.

### Command: ros2 node info

Get detailed information about the turtlesim node:

```bash
ros2 node info /turtlesim
```

**Expected output** (abbreviated):
```
/turtlesim
  Subscribers:
    /turtle1/cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
    /turtle1/color_sensor: turtlesim/msg/Color
    /turtle1/pose: turtlesim/msg/Pose
  Service Servers:
    /clear: std_srvs/srv/Empty
    /kill: turtlesim/srv/Kill
    /reset: std_srvs/srv/Empty
    /spawn: turtlesim/srv/Spawn
    [... more services ...]
  Service Clients:

  Action Servers:
    /turtle1/rotate_absolute: turtlesim/action/RotateAbsolute
  Action Clients:
```

**What you're seeing**:
- **Subscribers**: Topics this node **receives** messages from (inputs)
- **Publishers**: Topics this node **sends** messages to (outputs)
- **Service Servers**: Request-response operations this node provides
- **Service Clients**: Services this node can call on other nodes
- **Action Servers**: Long-running tasks this node can perform
- **Action Clients**: Long-running tasks this node can request

Don't worry about understanding everything yet. Focus on **Publishers** and **Subscribers** for now.

### Key Insight: Nodes Communicate via Topics

Notice `/turtle1/cmd_vel` under **Subscribers** and `/turtle1/pose` under **Publishers**:

- `/turtle1/cmd_vel` — The turtlesim node **receives** velocity commands (inputs telling the turtle how to move)
- `/turtle1/pose` — The turtlesim node **publishes** the turtle's position and orientation (outputs describing where the turtle is)

This is the fundamental ROS 2 pattern: **nodes communicate by publishing and subscribing to topics**.

## Exploring Topics

### What is a Topic?

A **topic** is a named channel for messages. Think of it as a broadcast stream:
- Any node can **publish** (send) messages to a topic
- Any node can **subscribe** (listen) to a topic
- Publishers don't know who's listening
- Subscribers don't know who's publishing

This **decoupling** is powerful. You can add sensors, swap algorithms, or change hardware without rewriting integration code.

### Command: ros2 topic list

List all active topics:

```bash
ros2 topic list
```

**Expected output**:
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

**What you're seeing**:
- `/turtle1/cmd_vel` — Velocity commands (how to move)
- `/turtle1/pose` — Position and orientation (where the turtle is)
- `/turtle1/color_sensor` — Color under the turtle
- `/parameter_events` — Parameter changes (ROS 2 internal)
- `/rosout` — Log messages (ROS 2 internal)

### Command: ros2 topic info

Get details about a specific topic:

```bash
ros2 topic info /turtle1/cmd_vel
```

**Expected output**:
```
Type: geometry_msgs/msg/Twist
Publisher count: 0
Subscription count: 1
```

**What this tells you**:
- **Type**: `geometry_msgs/msg/Twist` — The message format for this topic (velocity in 3D)
- **Publisher count**: 0 — No nodes are currently publishing commands
- **Subscription count**: 1 — One node (turtlesim) is listening for commands

**Why publisher count is 0**: You haven't started the teleop node yet. Turtlesim is waiting for commands, but no one is sending them.

Now check the pose topic:

```bash
ros2 topic info /turtle1/pose
```

**Expected output**:
```
Type: turtlesim/msg/Pose
Publisher count: 1
Subscription count: 0
```

**What this tells you**:
- **Type**: `turtlesim/msg/Pose` — Custom message type for turtle position
- **Publisher count**: 1 — Turtlesim is publishing the turtle's position
- **Subscription count**: 0 — No nodes are listening (yet)

### Command: ros2 topic echo

**This is the most useful debugging command.** It lets you **see the actual messages** flowing on a topic.

Watch the turtle's position in real-time:

```bash
ros2 topic echo /turtle1/pose
```

**Expected output** (streaming continuously):
```
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
[... repeats every ~100ms ...]
```

**What you're seeing**:
- `x` and `y` — Turtle's position (starts around 5.5, 5.5)
- `theta` — Orientation in radians (0 = facing right)
- `linear_velocity` and `angular_velocity` — Current speed (0 because it's not moving)
- `---` — Message separator

**Press Ctrl+C** to stop echoing.

### Understanding Topic Direction: Inputs vs Outputs

This is crucial for robotics. Let's distinguish:

**Command Topics (Inputs)**: Tell the robot **what to do**
- `/turtle1/cmd_vel` — Velocity commands (move forward, turn, etc.)
- Named with `cmd_*` by convention

**State Topics (Outputs)**: Report **what's happening**
- `/turtle1/pose` — Where the turtle is
- `/turtle1/color_sensor` — What color the turtle sees
- Report sensor data, positions, status

**Rule of thumb**: If it's a `cmd_*` topic, it's a command (input). If it's reporting data, it's state (output).

## Making the Turtle Move

Now let's publish commands and see the turtle respond.

### Start the Teleop Node

In a **third terminal**, run:

```bash
ros2 run turtlesim turtle_teleop_key
```

**Expected output**:
```
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
```

**Press the up arrow key** a few times. Watch the turtlesim window—the turtle should move forward!

### Observe the Command Topic

Now that the teleop node is running, check the `/turtle1/cmd_vel` topic again:

In a **fourth terminal**:

```bash
ros2 topic info /turtle1/cmd_vel
```

**Expected output**:
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

**Publisher count changed!** Now one node (turtle_teleop_key) is publishing commands, and one node (turtlesim) is subscribed.

### See the Commands in Real-Time

Echo the command topic while pressing arrow keys:

```bash
ros2 topic echo /turtle1/cmd_vel
```

In the **teleop terminal**, press the **up arrow** a few times.

**Expected output** (in the echo terminal):
```
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

**What this means**:
- `linear.x: 2.0` — Move forward at 2 units/second
- `angular.z: 0.0` — No rotation

Now press the **left arrow** key.

**Expected output**:
```
linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 2.0
---
```

**What this means**:
- `linear.x: 0.0` — Not moving forward
- `angular.z: 2.0` — Rotating counterclockwise at 2 radians/second

**Key insight**: The teleop node publishes `Twist` messages to `/turtle1/cmd_vel`, and turtlesim receives those messages and moves the turtle accordingly.

Press **Ctrl+C** in the echo terminal when done observing.

## Discovering Services

### What is a Service?

While **topics** are for continuous streaming data (sensor readings, positions, commands), **services** are for **on-demand request-response** operations:

- Spawn a new turtle (request: position and name, response: success or error)
- Clear the path the turtle drew
- Reset the simulation

Services are **synchronous** (you wait for a response) unlike topics (asynchronous streaming).

### Command: ros2 service list

List available services:

```bash
ros2 service list
```

**Expected output** (abbreviated):
```
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
[... more ...]
```

**What you're seeing**: Operations you can request from the turtlesim node.

### Command: ros2 service type

Check what kind of request a service expects:

```bash
ros2 service type /spawn
```

**Expected output**:
```
turtlesim/srv/Spawn
```

This tells you the service uses the `Spawn` service type from the `turtlesim` package.

### Command: ros2 service call

**Call a service** to spawn a second turtle:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

**Expected output**:
```
requester: making request: turtlesim.srv.Spawn_Request(x=2.0, y=2.0, theta=0.0, name='turtle2')

response:
turtlesim.srv.Spawn_Response(name='turtle2')
```

**Look at the turtlesim window**—a second turtle should appear at position (2, 2)!

**What just happened**:
1. You sent a **request** to the `/spawn` service with parameters (x, y, theta, name)
2. The turtlesim node received the request
3. It created a new turtle
4. It sent a **response** confirming the name

This is the request-response pattern. Services are **synchronous**—you wait for the response before continuing.

### Try Another Service: Clear

Clear the path the turtle drew:

```bash
ros2 service call /clear std_srvs/srv/Empty
```

**Watch the turtlesim window**—the drawn lines should disappear, leaving only the turtles.

**Note**: `std_srvs/srv/Empty` means this service takes no parameters. It's just a trigger.

## Understanding the ROS 2 Graph

Now you've seen the core components. Let's connect them:

### The Graph Concept

The **ROS 2 graph** is the network of:
- **Nodes** (executables doing computation)
- **Topics** (channels for continuous message streams)
- **Services** (request-response operations)

These connections form a graph structure:

```
┌──────────────┐    /turtle1/cmd_vel    ┌──────────────┐
│ turtle_teleop├───────────────────────►│  turtlesim   │
│     _key     │  (geometry_msgs/Twist) │              │
└──────────────┘                        └───────┬──────┘
                                                │
                                                │ /turtle1/pose
                                                │ (turtlesim/msg/Pose)
                                                ▼
                                         [anyone listening]
```

**Key points**:
- Nodes are **independent processes**
- They communicate **only** through topics, services, and actions
- You can add, remove, or replace nodes without rewriting other nodes
- This is what makes ROS 2 **modular** and **flexible**

### Visualize with rqt_graph (Optional)

If you installed the desktop version, you can visualize the graph:

```bash
rqt_graph
```

**Expected result**: A graphical window showing nodes as ovals and topics as arrows connecting them.

This visualization helps you understand complex robot systems with dozens of nodes.

## Hands-On Exercise: Spawn a Third Turtle

**Challenge**: Spawn a third turtle at position (8, 8) named "turtle3".

<details>
<summary>Solution</summary>

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.0, y: 8.0, theta: 0.0, name: 'turtle3'}"
```

Verify with:
```bash
ros2 node list
```

You should see `/turtle3` in addition to `/turtlesim`.

Check its topics:
```bash
ros2 topic list | grep turtle3
```

You should see `/turtle3/cmd_vel`, `/turtle3/pose`, etc.

</details>

## Hands-On Exercise: Control Turtle 2

**Challenge**: The teleop node is controlling turtle1. How would you control turtle2?

<details>
<summary>Hint</summary>

Look at the topic name for turtle1's commands: `/turtle1/cmd_vel`

What would turtle2's command topic be?

</details>

<details>
<summary>Solution</summary>

Turtle2's command topic is `/turtle2/cmd_vel`.

You could:

**Option 1**: Publish commands manually:
```bash
ros2 topic pub /turtle2/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
```

**Option 2**: Start a second teleop node remapped to turtle2 (advanced, not covered yet).

Try Option 1. Watch turtle2 move in a circle!

Press **Ctrl+C** to stop publishing.

</details>

## Check Your Understanding

Before moving on, verify you can answer these:

1. **What is a node in ROS 2?**
   <details>
   <summary>Reveal answer</summary>

   A node is a single-purpose executable that performs computation. Examples: sensor driver, planning algorithm, controller. Nodes communicate via topics, services, and actions.
   </details>

2. **What's the difference between a topic and a service?**
   <details>
   <summary>Reveal answer</summary>

   - **Topic**: Continuous asynchronous message streaming (pub-sub). Used for sensor data, commands, state information.
   - **Service**: On-demand synchronous request-response. Used for triggering actions (spawn, reset, configure).
   </details>

3. **How would you see messages being published on `/turtle1/pose`?**
   <details>
   <summary>Reveal answer</summary>

   ```bash
   ros2 topic echo /turtle1/pose
   ```

   This subscribes to the topic and prints messages to the terminal.
   </details>

4. **How can you tell if a topic is a command (input) or state (output)?**
   <details>
   <summary>Reveal answer</summary>

   - **Command topics** often have `cmd` in the name (e.g., `/turtle1/cmd_vel`) and tell the robot what to do
   - **State topics** report sensor data or status (e.g., `/turtle1/pose`, `/camera/image_raw`)
   - Use `ros2 node info <node>` to see which topics a node publishes (outputs) vs subscribes (inputs)
   </details>

5. **What does the ROS 2 graph consist of?**
   <details>
   <summary>Reveal answer</summary>

   The graph consists of:
   - **Nodes** (computational processes)
   - **Topics** (message streams connecting nodes)
   - **Services** (request-response operations)
   - **Actions** (long-running tasks with feedback, covered later)

   These components form a network that defines how robot components communicate.
   </details>

## Key Commands Summary

Here's your ROS 2 CLI toolkit:

| Command | Purpose | Example |
|---------|---------|---------|
| `ros2 node list` | List running nodes | Find all active nodes in system |
| `ros2 node info <node>` | Inspect node details | See node's publishers, subscribers, services |
| `ros2 topic list` | List active topics | Find all message streams |
| `ros2 topic info <topic>` | Topic details | See message type, pub/sub counts |
| `ros2 topic echo <topic>` | Watch messages | Debug by seeing live data |
| `ros2 topic pub <topic> <type> <data>` | Publish messages | Test by sending data manually |
| `ros2 service list` | List available services | Find operations you can call |
| `ros2 service type <service>` | Service type | See request/response format |
| `ros2 service call <service> <type> <data>` | Call service | Trigger an operation |

**Pro tip**: You don't need to memorize these. Use `ros2 --help` or `ros2 topic --help` to see available commands.

## What's Next?

You've explored ROS 2 using existing nodes (turtlesim). In the next lesson, you'll **create your own ROS 2 node** in Python.

You'll learn:
- How to initialize a ROS 2 Python node
- What "spinning" means
- How to use timer callbacks
- How to properly shut down a node

## Try With AI

**Setup**: Open your AI assistant to deepen your understanding of the ROS 2 graph.

**Exploration Prompts**:

```
Prompt 1: "I'm looking at turtlesim. It subscribes to /turtle1/cmd_vel and publishes to /turtle1/pose. Explain why this design (separate topics for commands and state) is better than a single bidirectional connection."

Prompt 2: "In ROS 2, when would I use a topic vs a service? Give me 3 concrete examples of each in a real robot system like a warehouse robot."

Prompt 3: "I ran 'ros2 topic echo /turtle1/cmd_vel' and saw 'geometry_msgs/msg/Twist' with linear and angular fields. Explain what each field controls and why velocity is split into linear and angular components."
```

**Expected Outcomes**: Your AI should help you understand the architectural decisions behind ROS 2's design, provide concrete examples from real robot systems, and clarify the mathematics of robot motion (linear vs angular velocity).
