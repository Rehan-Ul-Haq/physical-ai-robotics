---
title: Publisher-Subscriber Pattern
chapter: 2
lesson: 5
learning_objectives:
  - Create a ROS 2 publisher that sends messages to a topic
  - Create a ROS 2 subscriber that receives messages from a topic
  - Understand message types and how to use them
  - Debug topic communication issues
  - Apply the pub-sub pattern to robot communication
estimated_time: 150 minutes
skills:
  publisher_creation:
    proficiency: A2
  subscriber_creation:
    proficiency: A2
  message_types:
    proficiency: A2
generated_by: content-implementer v1.1.0
source_spec: specs/002-chapter-02-ros2-fundamentals/spec.md
created: 2025-11-28
last_modified: 2025-11-28
workflow: /sp.implement
version: 1.0.0
---

# Publisher-Subscriber Pattern

You've created a ROS 2 node with timers. Now let's make nodes **communicate**.

The **publisher-subscriber pattern** (pub-sub) is the foundation of ROS 2 communication. One node **publishes** messages to a topic, and other nodes **subscribe** to receive those messages.

In this lesson, you'll create two nodes:
- **talker.py** — Publishes string messages
- **listener.py** — Subscribes and prints those messages

**Hardware Tier**: Simulation Only — any computer with ROS 2 Humble installed

## Understanding Publishers

### What is a Publisher?

A **publisher** sends messages to a named topic. Think of it as broadcasting on a radio frequency—anyone tuned to that frequency can listen.

**Key characteristics**:
- **One-way communication** — Publisher doesn't know who's listening (if anyone)
- **Asynchronous** — No waiting for confirmation
- **Continuous streaming** — Publishes at regular intervals (typically)

**When to use publishers**:
- Sensor data (camera images, lidar scans, IMU readings)
- Robot state (position, velocity, battery level)
- Commands (motor commands, gripper control)

### Publisher Components

Every publisher needs:

1. **Message type** — What kind of data? (e.g., String, Image, Twist)
2. **Topic name** — Which channel? (e.g., `/chatter`, `/camera/image_raw`)
3. **Queue size** — How many messages to buffer if subscribers are slow?

Let's build one.

## Create the Publisher Node (talker.py)

### Step 1: Create the File

```bash
cd ~/ros2_ws/src
touch talker.py
chmod +x talker.py
```

### Step 2: Write the Publisher Code

Open `talker.py` and add this code:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker')

        # Create publisher: message type, topic name, queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Publish every 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.counter = 0
        self.get_logger().info('Talker node started, publishing to /chatter')

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS 2! Message #{self.counter}'

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = TalkerNode()

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

### Step 3: Run the Publisher

```bash
python3 talker.py
```

**Expected output**:
```
[INFO] [talker]: Talker node started, publishing to /chatter
[INFO] [talker]: Publishing: "Hello, ROS 2! Message #0"
[INFO] [talker]: Publishing: "Hello, ROS 2! Message #1"
[INFO] [talker]: Publishing: "Hello, ROS 2! Message #2"
[... continues every 0.5 seconds ...]
```

**Leave this running** and open a **second terminal**.

### Step 4: Verify Messages are Being Published

In the second terminal:

```bash
ros2 topic echo /chatter
```

**Expected output**:
```
data: 'Hello, ROS 2! Message #3'
---
data: 'Hello, ROS 2! Message #4'
---
data: 'Hello, ROS 2! Message #5'
---
[... streaming continuously ...]
```

**Success!** Your publisher is sending messages, and `ros2 topic echo` is receiving them.

Press **Ctrl+C** in both terminals when done observing.

## Understanding the Publisher Code

### Creating the Publisher

```python
self.publisher_ = self.create_publisher(String, 'chatter', 10)
```

**Parameters**:
1. **`String`** — Message type (from `std_msgs.msg`)
2. **`'chatter'`** — Topic name (nodes will subscribe to this)
3. **`10`** — Queue size (buffer up to 10 messages if subscriber is slow)

**Why `self.publisher_`**: The underscore suffix is a Python convention indicating a private attribute. Storing the publisher keeps it alive.

### Creating the Message

```python
msg = String()
msg.data = f'Hello, ROS 2! Message #{self.counter}'
```

**What's happening**:
1. Create an instance of the `String` message type
2. Set the `data` field to your string content

**Message structure**: Each message type has specific fields. `String` has one field: `data` (a string). Other types have different fields (e.g., `Twist` has `linear` and `angular`).

### Publishing the Message

```python
self.publisher_.publish(msg)
```

This sends the message to the `/chatter` topic. Any node subscribed to `/chatter` will receive it.

### Understanding Queue Size

The queue size (10 in our example) controls buffering:

- **If subscribers keep up**: Queue stays small, messages delivered immediately
- **If subscriber is slow**: Publisher buffers up to 10 messages
- **If queue fills**: Publisher starts dropping old messages (or blocks, depending on QoS settings)

**For beginners**: 10 is a reasonable default. In production, you'd tune this based on message rate and subscriber performance.

## Create the Subscriber Node (listener.py)

Now let's create a node that receives messages.

### Step 1: Create the File

```bash
cd ~/ros2_ws/src
touch listener.py
chmod +x listener.py
```

### Step 2: Write the Subscriber Code

Open `listener.py` and add:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener')

        # Create subscriber: message type, topic name, callback, queue size
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        self.get_logger().info('Listener node started, subscribed to /chatter')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()

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

### Step 3: Run Publisher and Subscriber Together

**Terminal 1** (publisher):
```bash
python3 talker.py
```

**Terminal 2** (subscriber):
```bash
python3 listener.py
```

**Expected output in Terminal 1** (publisher):
```
[INFO] [talker]: Publishing: "Hello, ROS 2! Message #0"
[INFO] [talker]: Publishing: "Hello, ROS 2! Message #1"
...
```

**Expected output in Terminal 2** (subscriber):
```
[INFO] [listener]: Listener node started, subscribed to /chatter
[INFO] [listener]: I heard: "Hello, ROS 2! Message #0"
[INFO] [listener]: I heard: "Hello, ROS 2! Message #1"
...
```

**What you're seeing**: Messages published by `talker.py` are received by `listener.py` in real-time.

Press **Ctrl+C** in both terminals when done.

## Understanding the Subscriber Code

### Creating the Subscription

```python
self.subscription = self.create_subscription(
    String,
    'chatter',
    self.listener_callback,
    10
)
```

**Parameters**:
1. **`String`** — Message type (must match publisher)
2. **`'chatter'`** — Topic name (must match publisher)
3. **`self.listener_callback`** — Function to call when message arrives
4. **`10`** — Queue size (buffer up to 10 messages)

**Critical**: Message type and topic name must **exactly match** the publisher, or messages won't be received.

### The Callback Function

```python
def listener_callback(self, msg):
    self.get_logger().info(f'I heard: "{msg.data}"')
```

**What happens**:
1. When a message arrives on `/chatter`, ROS 2 calls this function
2. The `msg` parameter contains the received message
3. You can access `msg.data` to get the string content
4. The callback executes and returns
5. ROS 2 waits for the next message

**Important**: Callbacks should execute quickly. Long-running operations can block other callbacks.

## Message Types: Beyond Strings

`std_msgs/String` is simple, but robots need richer data. Let's explore other message types.

### Common Message Packages

| Package | Purpose | Example Types |
|---------|---------|---------------|
| `std_msgs` | Basic types | String, Int32, Float64, Bool |
| `geometry_msgs` | Positions, velocities | Point, Pose, Twist, Transform |
| `sensor_msgs` | Sensor data | Image, LaserScan, Imu, PointCloud2 |
| `nav_msgs` | Navigation | Odometry, Path, OccupancyGrid |

### Example: Publishing Robot Velocity

Robot velocity uses `geometry_msgs/Twist`:

```python
from geometry_msgs.msg import Twist

# In __init__:
self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

# In callback:
msg = Twist()
msg.linear.x = 0.5   # Move forward at 0.5 m/s
msg.angular.z = 0.1  # Turn at 0.1 rad/s
self.vel_publisher.publish(msg)
```

**Message structure**:
```
linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1
```

This is the same message type turtlesim uses for `/turtle1/cmd_vel`.

### Finding Message Definitions

To see what fields a message type has:

```bash
ros2 interface show std_msgs/msg/String
```

**Output**:
```
string data
```

For Twist:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

**Output**:
```
Vector3  linear
        float64 x
        float64 y
        float64 z
Vector3  angular
        float64 x
        float64 y
        float64 z
```

Use `ros2 interface show <package>/msg/<Type>` to explore any message type.

## Refining Topic Communication

Now that you have basic pub-sub working, let's explore how to improve it.

### Choosing Appropriate Queue Sizes

Suppose you ask:

> "I have a camera publishing images at 30 Hz. Should I use queue size 10, 100, or 1 for the subscriber?"

**The reasoning you might explore**:

Queue size determines buffering. Large queues (100) increase latency—if processing is slow, you might be looking at old frames. Small queues (1) minimize latency but might drop frames if processing takes too long.

For vision processing where you want the latest frame, queue size 1 might be ideal. This ensures you always process the most recent image, dropping old ones.

For data logging where you want every frame, a larger queue (100) might be appropriate to avoid drops during processing spikes.

Through this exploration, you realize queue size is a trade-off between **latency** (how fresh the data is) and **completeness** (not dropping data).

### Optimizing Message Publishing Rate

You might wonder:

> "My node publishes robot position at 100 Hz, but subscribers only need updates at 10 Hz. Am I wasting bandwidth?"

**The analysis**:

Yes—publishing at 100 Hz when consumers need 10 Hz wastes network bandwidth and subscriber CPU. However, you might have multiple subscribers with different needs: one for logging (needs all data), one for visualization (needs only 10 Hz).

Options:
1. Publish at highest required rate (100 Hz) and let low-rate subscribers drop messages
2. Publish at lower rate (10 Hz) and create separate high-rate topic for logging
3. Use topic filtering (ROS 2 content filtering, advanced)

For your MVP, publishing at 100 Hz with low-rate subscribers using small queue sizes (1-2) is acceptable.

**What emerged**: A practical understanding of pub-sub performance trade-offs that balances simplicity (one topic) with efficiency (appropriate queue sizes per subscriber).

## Debugging Topic Communication

Common issues and how to solve them:

### Issue 1: Subscriber Receives Nothing

**Symptoms**: Publisher runs without errors, subscriber runs without errors, but subscriber never receives messages.

**Debugging steps**:

```bash
# Verify publisher is actually publishing
ros2 topic list

# Check if /chatter appears
# If not, publisher isn't creating the topic correctly
```

```bash
# Verify someone is publishing
ros2 topic info /chatter

# Should show "Publisher count: 1"
```

```bash
# Manually subscribe to verify messages exist
ros2 topic echo /chatter

# If you see messages here but listener doesn't, issue is in listener code
```

**Common causes**:
- Topic names don't match (e.g., publisher uses `/chatter`, subscriber uses `chatter`)
- Message types don't match
- Nodes on different ROS domains (advanced, see ROS_DOMAIN_ID)

### Issue 2: Message Type Mismatch

**Symptom**: Error like "Failed to create subscription: message type mismatch"

**Cause**: Publisher and subscriber use different message types.

**Solution**: Verify both use the same type:

```bash
ros2 topic info /chatter
```

**Output**:
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

Ensure both nodes import and use `std_msgs/msg/String`.

### Issue 3: Messages Arriving Late

**Symptom**: Subscriber logs timestamps showing messages are delayed.

**Cause**: Subscriber callback takes too long, queue fills up.

**Solution**: Reduce queue size to 1 (always process latest), or optimize callback to execute faster.

## Hands-On Exercise: Temperature Publisher

**Challenge**: Create a publisher that simulates a temperature sensor:
- Node name: `temp_sensor`
- Topic: `/temperature`
- Message type: `std_msgs/Float64`
- Publishes a random temperature between 20.0 and 30.0 every second

<details>
<summary>Hint 1</summary>

Use `from std_msgs.msg import Float64` and `import random`.

</details>

<details>
<summary>Hint 2</summary>

```python
msg = Float64()
msg.data = random.uniform(20.0, 30.0)
```

</details>

<details>
<summary>Solution</summary>

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import random


class TempSensorNode(Node):
    def __init__(self):
        super().__init__('temp_sensor')
        self.publisher_ = self.create_publisher(Float64, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Temperature sensor started')

    def timer_callback(self):
        msg = Float64()
        msg.data = random.uniform(20.0, 30.0)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing temperature: {msg.data:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TempSensorNode()

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

Test it:
```bash
python3 temp_sensor.py
```

In another terminal:
```bash
ros2 topic echo /temperature
```

</details>

## Hands-On Exercise: Temperature Monitor

**Challenge**: Create a subscriber that listens to `/temperature` and logs a warning if temperature exceeds 28.0°C.

<details>
<summary>Solution</summary>

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class TempMonitorNode(Node):
    def __init__(self):
        super().__init__('temp_monitor')
        self.subscription = self.create_subscription(
            Float64,
            'temperature',
            self.temp_callback,
            10
        )
        self.get_logger().info('Temperature monitor started')

    def temp_callback(self, msg):
        temp = msg.data
        if temp > 28.0:
            self.get_logger().warn(f'High temperature detected: {temp:.2f}°C')
        else:
            self.get_logger().info(f'Temperature normal: {temp:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = TempMonitorNode()

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

Run both nodes:

**Terminal 1**:
```bash
python3 temp_sensor.py
```

**Terminal 2**:
```bash
python3 temp_monitor.py
```

Watch the monitor log warnings when temperature exceeds 28°C.

</details>

## What You've Learned

You can now:
- ✅ Create ROS 2 publishers to send messages
- ✅ Create ROS 2 subscribers to receive messages
- ✅ Use different message types (String, Float64, Twist)
- ✅ Debug topic communication issues
- ✅ Understand queue sizes and their impact
- ✅ Find and inspect message type definitions

**Next lesson**: You'll learn about **services**, a different communication pattern for request-response operations (not continuous streaming).

## Try With AI

**Setup**: Open your AI assistant to explore pub-sub patterns and solve real-world problems.

**Exploration Prompts**:

```
Prompt 1: "I have a ROS 2 publisher sending images at 30 Hz (large messages, ~1 MB each). My subscriber is processing them but occasionally drops frames. Should I increase the queue size from 10 to 100, or is there a better approach?"

Prompt 2: "In ROS 2, I want multiple subscribers to receive the same messages from one publisher. Do I need to do anything special, or does pub-sub handle this automatically? What happens if one subscriber is slower than others?"

Prompt 3: "I'm publishing robot joint commands at 100 Hz. The subscriber callback takes 50ms to process (motor control). Will this cause problems? How does ROS 2 handle callbacks that take longer than the publishing rate?"
```

**Expected Outcomes**: Your AI should help you understand pub-sub scaling behavior, multi-subscriber patterns, and callback timing constraints. Through these discussions, you'll develop intuition for designing robust real-time robot communication systems.

**Safety Note**: When publishing high-rate data (images, point clouds), be aware of network bandwidth limits. Monitor with `ros2 topic hz /topic_name` and `ros2 topic bw /topic_name` to measure actual rates and bandwidth usage.
