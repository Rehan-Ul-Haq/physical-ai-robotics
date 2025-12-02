---
title: Create Your First Python Node
chapter: 2
lesson: 4
learning_objectives:
  - Understand the anatomy of a ROS 2 Python node
  - Initialize rclpy and create a Node instance
  - Use timer callbacks to execute code periodically
  - Properly shut down a ROS 2 node with Ctrl+C
estimated_time: 120 minutes
skills:
  rclpy_programming:
    proficiency: A2
  node_creation:
    proficiency: A2
generated_by: content-implementer v1.1.0
source_spec: specs/002-chapter-02-ros2-fundamentals/spec.md
created: 2025-11-28
last_modified: 2025-11-28
workflow: /sp.implement
version: 1.0.0
---

# Create Your First Python Node

You've explored ROS 2 using existing nodes. Now it's time to build your own.

In this lesson, you'll create a simple ROS 2 node in Python that prints "Hello, ROS 2!" every second. This will teach you the fundamental structure that every ROS 2 Python program follows.

**Hardware Tier**: Simulation Only — any computer with ROS 2 Humble installed

## The Anatomy of a ROS 2 Node

Before writing code, let's understand what a ROS 2 node needs:

1. **Initialization** — Tell ROS 2 you're starting a node
2. **Node creation** — Create an instance of the Node class
3. **Functionality** — Define what your node does (timers, publishers, subscribers, etc.)
4. **Spinning** — Keep the node alive and process callbacks
5. **Shutdown** — Clean up when exiting

Every ROS 2 Python node follows this pattern. Let's build one step by step.

## Create Your First Node

### Step 1: Create a Python File

Open a terminal and create a new directory for your ROS 2 experiments:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Create a Python file:

```bash
touch hello_ros2.py
chmod +x hello_ros2.py
```

**What `chmod +x` does**: Makes the file executable so you can run it directly.

### Step 2: Write the Minimal Node

Open `hello_ros2.py` in your text editor and add this code:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello, ROS 2!')


def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Save the file.**

### Step 3: Run Your Node

In the terminal:

```bash
python3 hello_ros2.py
```

**Expected output**:
```
[INFO] [1234567890.123456789] [hello_node]: Hello, ROS 2!
```

**Success!** You've created your first ROS 2 node.

Press **Ctrl+C** to stop it.

## Understanding the Code

Let's break down each part:

### The Shebang Line

```python
#!/usr/bin/env python3
```

This tells the system to use Python 3 to execute the script. Required for executable scripts.

### Imports

```python
import rclpy
from rclpy.node import Node
```

- **`rclpy`** — The ROS 2 Python client library (equivalent to `rospy` in ROS 1)
- **`Node`** — Base class for all ROS 2 nodes

### The Node Class

```python
class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.get_logger().info('Hello, ROS 2!')
```

**What's happening**:
1. **`class HelloNode(Node)`** — Create a new class that inherits from `Node`
2. **`super().__init__('hello_node')`** — Call the parent constructor with the node name
3. **`self.get_logger().info(...)`** — Log a message (ROS 2's way of printing)

**Node naming**: The string `'hello_node'` is your node's name in the ROS 2 graph. It's what appears when you run `ros2 node list`.

### The Main Function

```python
def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

**Line by line**:

1. **`rclpy.init(args=args)`** — Initialize the ROS 2 Python library. Must be called before creating nodes.

2. **`node = HelloNode()`** — Create an instance of your node.

3. **`rclpy.spin(node)`** — Enter the event loop. This keeps the node running and processes callbacks (timers, messages, etc.). Without this, the node would exit immediately.

4. **`except KeyboardInterrupt: pass`** — Handle Ctrl+C gracefully without printing error messages.

5. **`node.destroy_node()`** — Clean up node resources.

6. **`rclpy.shutdown()`** — Shut down ROS 2 properly.

### The Entry Point

```python
if __name__ == '__main__':
    main()
```

This ensures `main()` runs when you execute the script directly (but not when importing it as a module).

## Adding a Timer: Periodic Execution

Right now, the node prints once and then just sits there spinning. Let's make it print every second using a **timer callback**.

### What is a Callback?

A **callback** is a function that ROS 2 calls automatically when something happens:
- **Timer callback** — Called at regular intervals
- **Subscription callback** — Called when a message arrives (next lesson)
- **Service callback** — Called when a service is requested (lesson 6)

Callbacks are the fundamental way ROS 2 nodes react to events.

### Update Your Code

Replace the `HelloNode` class with this version:

```python
class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.counter = 0

        # Create a timer that calls timer_callback every 1 second
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Hello Node started!')

    def timer_callback(self):
        self.counter += 1
        self.get_logger().info(f'Hello, ROS 2! (count: {self.counter})')
```

**Save and run**:

```bash
python3 hello_ros2.py
```

**Expected output**:
```
[INFO] [1234567890.123456789] [hello_node]: Hello Node started!
[INFO] [1234567891.123456789] [hello_node]: Hello, ROS 2! (count: 1)
[INFO] [1234567892.123456789] [hello_node]: Hello, ROS 2! (count: 2)
[INFO] [1234567893.123456789] [hello_node]: Hello, ROS 2! (count: 3)
[... continues every second ...]
```

**Press Ctrl+C** to stop.

### Understanding Timers

```python
self.timer = self.create_timer(1.0, self.timer_callback)
```

**What this does**:
- **`create_timer(period, callback)`** — Creates a timer
- **`1.0`** — Period in seconds (1.0 = every 1 second)
- **`self.timer_callback`** — Function to call (note: no parentheses)

**How it works**:
1. ROS 2's executor (the `spin()` loop) checks timers periodically
2. When 1 second has passed, it calls `timer_callback()`
3. The callback executes and returns
4. ROS 2 waits another 1 second and calls it again
5. This continues until the node shuts down

**Why `self.timer = ...`**: Storing the timer in `self.timer` keeps a reference to it. If you don't store it, Python's garbage collector might delete it, and the timer won't fire.

## Verifying Your Node with ROS 2 Tools

While your node is running, open a **second terminal** and use the tools you learned in Lesson 3:

### Check the Node is Running

```bash
ros2 node list
```

**Expected output**:
```
/hello_node
```

### Get Node Information

```bash
ros2 node info /hello_node
```

**Expected output** (abbreviated):
```
/hello_node
  Subscribers:

  Publishers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    [... various parameter services ...]
  Service Clients:

  Action Servers:

  Action Clients:
```

**What you're seeing**:
- Your node isn't publishing to any custom topics yet (next lesson)
- `/rosout` is ROS 2's logging system (your `get_logger().info()` messages go here)
- Various parameter services are automatically created for every node

### View Log Messages

```bash
ros2 topic echo /rosout
```

You'll see your "Hello, ROS 2!" messages with full metadata (timestamp, logger name, severity level, etc.).

Press **Ctrl+C** to stop echoing.

## Modifying the Timer Rate

Let's experiment with different rates:

### Fast Timer (10 Hz)

Change the timer creation line to:

```python
self.timer = self.create_timer(0.1, self.timer_callback)
```

**`0.1` seconds = 100 milliseconds = 10 Hz** (10 times per second)

Run it:

```bash
python3 hello_ros2.py
```

Messages should appear 10 times per second. Press **Ctrl+C** to stop.

### Slow Timer (5 seconds)

Change to:

```python
self.timer = self.create_timer(5.0, self.timer_callback)
```

Run it and observe the 5-second intervals.

**Understanding Hz (Hertz)**:
- **1 Hz** = 1 time per second (period = 1.0 seconds)
- **10 Hz** = 10 times per second (period = 0.1 seconds)
- **100 Hz** = 100 times per second (period = 0.01 seconds)

In robotics:
- **Sensor data**: Often 30 Hz (cameras) to 1000 Hz (IMUs)
- **Control loops**: Typically 100-1000 Hz for motor control
- **Planning**: 1-10 Hz for high-level decision making

## Handling Errors: What Happens Without init()?

Understanding errors teaches you what's required. Let's intentionally break the code.

Comment out the `rclpy.init()` line:

```python
def main(args=None):
    # rclpy.init(args=args)  # COMMENTED OUT
    node = HelloNode()
    ...
```

Run it:

```bash
python3 hello_ros2.py
```

**Expected error**:
```
RuntimeError: rclpy.create_node() requires rclpy.init() to be called first
```

**What this teaches**: You **must** call `rclpy.init()` before creating nodes. This initializes the ROS 2 middleware.

**Uncomment the line** to fix it:

```python
def main(args=None):
    rclpy.init(args=args)
    ...
```

## Improving with AI: Exploring Callback Patterns

Now that you've created a basic node, let's explore how an AI assistant can help you understand and extend it.

### Asking for Clarification

Suppose you're unsure why the timer callback pattern works. You might ask:

> "In ROS 2, I created a timer with `self.create_timer(1.0, self.timer_callback)`. Why do I pass the function name without parentheses? What would happen if I wrote `self.timer_callback()` instead?"

**What the AI explains**:

When you pass `self.timer_callback`, you're passing a reference to the function object itself. This allows ROS 2 to call it later. If you wrote `self.timer_callback()`, you'd be calling the function immediately and passing its return value (None) to create_timer, which would cause an error.

This is called a "callback" because ROS 2 "calls back" your function when the timer fires.

### Exploring Error Handling

You might want to make your node more robust:

> "My ROS 2 node uses a timer callback. Should I add error handling inside the callback? What happens if the callback raises an exception?"

**What the AI suggests**:

If a callback raises an unhandled exception, it propagates up to the executor, potentially crashing your node. For production code, you should wrap callback logic in try-except blocks:

```python
def timer_callback(self):
    try:
        self.counter += 1
        self.get_logger().info(f'Hello, ROS 2! (count: {self.counter})')
        # ... more logic ...
    except Exception as e:
        self.get_logger().error(f'Error in timer callback: {e}')
```

This prevents a single callback error from bringing down your entire node.

Through this exchange, you discovered a defensive programming pattern you might not have thought of initially. The AI's suggestion came from understanding production robotics best practices.

### Refining Logging Approaches

Suppose you're using `print()` instead of the logger:

> "I'm using `print()` in my ROS 2 node to debug. Is there a better way?"

**What the AI recommends**:

Yes—use ROS 2's built-in logger instead:

```python
self.get_logger().info('Informational message')
self.get_logger().warn('Warning message')
self.get_logger().error('Error message')
self.get_logger().debug('Debug message (hidden by default)')
```

Advantages over print():
- Messages include timestamps and node names
- Can be published to /rosout topic for remote monitoring
- Support severity levels for filtering
- Integrate with ROS 2 logging tools

You might respond:

> "For this MVP, I'll stick with print() for speed, but I'll use the logger for the final version."

**The AI adapts**:

That's a reasonable trade-off for prototyping. Just remember that print() output doesn't appear in ROS 2's logging infrastructure, so it won't be captured by tools like rqt_console for remote debugging.

**What emerged from this dialogue**: A practical approach (logger for production, print() acceptable for quick prototypes) that balances best practices with your specific development constraints. Neither you nor the AI dictated the solution—you converged on it through iteration.

## Debugging Common Issues

### Issue 1: "ModuleNotFoundError: No module named 'rclpy'"

**Symptom**: When running the script, Python can't find rclpy.

**Cause**: ROS 2 environment not sourced.

**Solution**:
```bash
source /opt/ros/humble/setup.bash
python3 hello_ros2.py
```

### Issue 2: Node Doesn't Respond to Ctrl+C

**Symptom**: Pressing Ctrl+C doesn't stop the node cleanly.

**Cause**: Missing `KeyboardInterrupt` exception handling.

**Solution**: Ensure your main function has:
```python
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    rclpy.shutdown()
```

### Issue 3: "Timer Not Firing" (Callback Never Executes)

**Symptom**: Timer callback never runs.

**Causes**:
1. Timer not stored in `self.timer` (garbage collected)
2. Node not spinning (`rclpy.spin()` not called)

**Solution**: Always store timer reference and ensure spin() is called:
```python
self.timer = self.create_timer(1.0, self.timer_callback)  # Store reference
# Later in main():
rclpy.spin(node)  # Required for timers to fire
```

## Hands-On Exercise: Custom Timer Node

**Challenge**: Create a node called `countdown_node` that counts down from 10 to 0 (one number per second), then prints "Liftoff!" and exits.

<details>
<summary>Hint 1</summary>

Use a timer with 1-second period. Start a counter at 10 and decrement in the callback.

</details>

<details>
<summary>Hint 2</summary>

When counter reaches 0, print "Liftoff!" and call `rclpy.shutdown()` to exit.

</details>

<details>
<summary>Solution</summary>

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class CountdownNode(Node):
    def __init__(self):
        super().__init__('countdown_node')
        self.counter = 10
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Countdown started!')

    def timer_callback(self):
        if self.counter > 0:
            self.get_logger().info(f'T-minus {self.counter}')
            self.counter -= 1
        else:
            self.get_logger().info('Liftoff!')
            self.timer.cancel()  # Stop the timer
            rclpy.shutdown()      # Exit cleanly


def main(args=None):
    rclpy.init(args=args)
    node = CountdownNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Run it:
```bash
python3 countdown_node.py
```

Expected output:
```
[INFO] [countdown_node]: Countdown started!
[INFO] [countdown_node]: T-minus 10
[INFO] [countdown_node]: T-minus 9
...
[INFO] [countdown_node]: T-minus 1
[INFO] [countdown_node]: Liftoff!
```

</details>

## What You've Learned

You can now:
- ✅ Create a ROS 2 Python node from scratch
- ✅ Initialize rclpy and create Node instances
- ✅ Use timer callbacks for periodic execution
- ✅ Understand the node lifecycle (init → spin → shutdown)
- ✅ Debug nodes using ROS 2 CLI tools
- ✅ Handle Ctrl+C gracefully

**Next lesson**: You'll extend this knowledge to create **publishers and subscribers**, enabling nodes to communicate with each other.

## Try With AI

**Setup**: Open your AI assistant to explore node creation patterns and solve problems.

**Exploration Prompts**:

```
Prompt 1: "I created a ROS 2 timer that fires every 0.01 seconds (100 Hz). My callback takes 0.015 seconds to execute. What happens? How does ROS 2 handle callbacks that take longer than the timer period?"

Prompt 2: "In my ROS 2 node, I need to execute two tasks at different rates: read a sensor at 50 Hz and publish a status message at 1 Hz. How should I structure this with timers?"

Prompt 3: "My ROS 2 node crashed with 'RuntimeError: rclpy.shutdown() has already been called'. What causes this error and how do I fix it?"
```

**Expected Outcomes**: Your AI should help you understand timer scheduling behavior, design multi-rate node architectures, and debug initialization/shutdown issues. Through these exchanges, you'll develop intuition for robust node design patterns.

**Safety Note**: When experimenting with timers, be aware that very high-frequency timers (1000+ Hz) can consume significant CPU. Start with moderate rates (1-10 Hz) and increase only when needed.
