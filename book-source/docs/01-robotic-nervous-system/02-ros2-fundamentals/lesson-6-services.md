---
title: Services (Request-Response Communication)
chapter: 2
lesson: 6
learning_objectives:
  - Understand when to use services vs topics
  - Create a ROS 2 service server that responds to requests
  - Create a ROS 2 service client that makes requests
  - Use service types and understand request/response structure
  - Debug service communication issues
estimated_time: 120 minutes
skills:
  service_server:
    proficiency: A2
  service_client:
    proficiency: A2
  request_response_pattern:
    proficiency: A2
generated_by: content-implementer v1.1.0
source_spec: specs/002-chapter-02-ros2-fundamentals/spec.md
created: 2025-11-28
last_modified: 2025-11-28
workflow: /sp.implement
version: 1.0.0
---

# Services (Request-Response Communication)

You've learned topics for continuous streaming data. Now let's explore **services**—ROS 2's pattern for **request-response** communication.

Think of services as function calls across nodes:
- **Client** sends a request with input parameters
- **Server** processes the request and returns a response
- Client **waits** for the response (synchronous)

In this lesson, you'll create an addition service where a client sends two numbers, and the server returns their sum.

**Hardware Tier**: Simulation Only — any computer with ROS 2 Humble installed

## Topics vs Services: When to Use Each

Before building services, understand when to use them.

### Topics (Continuous Streaming)

**Use topics when**:
- Data streams continuously (sensor readings, robot state)
- One-to-many communication (one publisher, multiple subscribers)
- No response needed
- Asynchronous (fire and forget)

**Examples**:
- Camera images (30 Hz stream)
- Robot position (100 Hz stream)
- Motor commands (continuous control)

### Services (Request-Response)

**Use services when**:
- On-demand operations (not continuous)
- You need a response/confirmation
- One-to-one communication (client waits for server)
- Synchronous (client blocks until response)

**Examples**:
- Spawn a new robot in simulation
- Reset odometry
- Save map to file
- Query robot configuration
- Trigger calibration sequence

### Rule of Thumb

**Topic**: "Here's data, anyone listening can use it" (broadcast)

**Service**: "Please do X and tell me when done" (request-response)

If you need confirmation or a return value, use a service. If you're streaming data, use a topic.

## Understanding Service Types

Like messages for topics, services have **types** defining request and response structure.

### Service Definition Structure

A service type has two parts:
1. **Request** — Input parameters sent by client
2. **Response** — Output values returned by server

**Example**: `example_interfaces/srv/AddTwoInts`

```
int64 a
int64 b
---
int64 sum
```

**Structure**:
- Top half (before `---`): Request fields
- Bottom half (after `---`): Response fields

The server receives `a` and `b`, computes `sum`, and returns it.

### Finding Service Types

List available service types:

```bash
ros2 interface list | grep srv
```

**Output** (abbreviated):
```
example_interfaces/srv/AddTwoInts
std_srvs/srv/Empty
std_srvs/srv/SetBool
std_srvs/srv/Trigger
...
```

View a service definition:

```bash
ros2 interface show example_interfaces/srv/AddTwoInts
```

**Output**:
```
int64 a
int64 b
---
int64 sum
```

## Create the Service Server

Let's build a server that adds two numbers.

### Step 1: Create the File

```bash
cd ~/ros2_ws/src
touch add_server.py
chmod +x add_server.py
```

### Step 2: Write the Server Code

Open `add_server.py` and add:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddServerNode(Node):
    def __init__(self):
        super().__init__('add_server')

        # Create service: service type, service name, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

        self.get_logger().info('Addition service ready')

    def add_callback(self, request, response):
        """
        Callback function for service requests.

        Args:
            request: AddTwoInts.Request with fields 'a' and 'b'
            response: AddTwoInts.Response with field 'sum'

        Returns:
            response: Modified response object
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = AddServerNode()

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

### Step 3: Run the Server

```bash
python3 add_server.py
```

**Expected output**:
```
[INFO] [add_server]: Addition service ready
```

**Leave this running** and open a **second terminal**.

### Step 4: Test with Command Line

In the second terminal, call the service manually:

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 7}"
```

**Expected output**:
```
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=5, b=7)

response:
example_interfaces.srv.AddTwoInts_Response(sum=12)
```

**In the server terminal**, you should see:
```
[INFO] [add_server]: Request: 5 + 7 = 12
```

**Success!** The server received the request, computed the sum, and returned the response.

## Understanding the Server Code

### Creating the Service

```python
self.srv = self.create_service(
    AddTwoInts,
    'add_two_ints',
    self.add_callback
)
```

**Parameters**:
1. **`AddTwoInts`** — Service type (from `example_interfaces.srv`)
2. **`'add_two_ints'`** — Service name (clients will call this)
3. **`self.add_callback`** — Function to call when request arrives

### The Callback Function

```python
def add_callback(self, request, response):
    response.sum = request.a + request.b
    self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
    return response
```

**What happens**:
1. **`request`** — Contains client's input (`request.a`, `request.b`)
2. **`response`** — Empty object you fill with output (`response.sum`)
3. **Compute** — Perform the operation (addition)
4. **Return** — Send `response` back to client

**Important**: You **must** return the `response` object. The client is waiting for it.

## Create the Service Client

Now create a client that calls the service.

### Step 1: Create the File

```bash
cd ~/ros2_ws/src
touch add_client.py
chmod +x add_client.py
```

### Step 2: Write the Client Code

Open `add_client.py` and add:

```python
#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class AddClientNode(Node):
    def __init__(self):
        super().__init__('add_client')

        # Create client: service type, service name
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b

        # Call service asynchronously
        self.future = self.cli.call_async(self.req)
        return self.future


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        print('Usage: add_client.py <a> <b>')
        return

    node = AddClientNode()
    future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))

    # Wait for response
    rclpy.spin_until_future_complete(node, future)

    if future.done():
        try:
            response = future.result()
            node.get_logger().info(
                f'Result: {sys.argv[1]} + {sys.argv[2]} = {response.sum}'
            )
        except Exception as e:
            node.get_logger().error(f'Service call failed: {e}')
    else:
        node.get_logger().error('Service call timed out')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Step 3: Run Server and Client Together

**Terminal 1** (server):
```bash
python3 add_server.py
```

**Terminal 2** (client):
```bash
python3 add_client.py 10 20
```

**Expected output in Terminal 2**:
```
[INFO] [add_client]: Result: 10 + 20 = 30
```

**Expected output in Terminal 1**:
```
[INFO] [add_server]: Request: 10 + 20 = 30
```

Try different numbers:

```bash
python3 add_client.py 100 25
```

**Output**:
```
[INFO] [add_client]: Result: 100 + 25 = 125
```

Press **Ctrl+C** in both terminals when done.

## Understanding the Client Code

### Creating the Client

```python
self.cli = self.create_client(AddTwoInts, 'add_two_ints')
```

**Parameters**:
1. **`AddTwoInts`** — Service type (must match server)
2. **`'add_two_ints'`** — Service name (must match server)

### Waiting for Service

```python
while not self.cli.wait_for_service(timeout_sec=1.0):
    self.get_logger().info('Service not available, waiting...')
```

**What this does**:
- Checks if server is running
- Waits up to 1 second
- Repeats until service available

**Why this matters**: If you call a service before the server starts, the call fails. This loop ensures the server is ready.

### Making the Request

```python
def send_request(self, a, b):
    self.req.a = a
    self.req.b = b
    self.future = self.cli.call_async(self.req)
    return self.future
```

**What's happening**:
1. Set request fields (`a`, `b`)
2. Call service **asynchronously** (`call_async`)
3. Return a **future** (promise of eventual result)

**Asynchronous vs synchronous**:
- **Async** (`call_async`): Non-blocking, continue immediately, check future later
- **Sync** (`call`): Blocking, wait for response (requires separate thread)

We use async because it's simpler and more flexible.

### Getting the Response

```python
rclpy.spin_until_future_complete(node, future)
response = future.result()
```

**What this does**:
1. **`spin_until_future_complete`** — Wait for response (blocks here)
2. **`future.result()`** — Extract response when ready

## Exploring Async vs Sync Service Patterns

Now that you have working service code, let's explore when to use different calling patterns.

### When to Use Async Calls

Suppose you're designing a multi-service client:

> "My robot node needs to call three services: get_map, plan_path, and execute_trajectory. Should I call them synchronously one after another, or use async calls?"

**The reasoning**:

If services are independent (get_map doesn't depend on plan_path), you could call all three async simultaneously and wait for all futures together. This would be much faster than sequential sync calls.

However, in your case:
- plan_path requires map (depends on get_map)
- execute_trajectory requires path (depends on plan_path)

These are **sequential dependencies**. You must wait for each before calling the next.

Options:
1. Async with sequential futures (call get_map, wait, call plan_path, wait, call execute)
2. Sync calls in sequence (simpler, but requires threading)

For your MVP with clear dependencies, async with sequential waiting is clearest. If you later add parallel operations (e.g., update display while planning), async is already set up for that.

**What emerged**: Understanding that async isn't just about parallelism—it's also about flexible control flow that can grow with your system's complexity.

### Handling Service Timing Issues

You might encounter:

> "My service client sometimes times out even though the server is running. What causes this?"

**The debugging process**:

First, verify the service is actually available:

```bash
ros2 service list
```

If `/add_two_ints` appears, the server is advertising. If not, server isn't running properly.

Next, check if the service responds:

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 1}"
```

If this works but your client doesn't, the issue is in client code.

Common causes:
1. Client calling before `wait_for_service()` completes
2. Timeout too short for slow service
3. Service callback blocking (takes too long to compute)

For debugging, increase timeout:

```python
while not self.cli.wait_for_service(timeout_sec=5.0):  # Increased from 1.0
    self.get_logger().info('Service not available, waiting...')
```

If this fixes it, your service startup is just slow. Acceptable for non-time-critical operations.

Through this investigation, you discovered that service timing is about **waiting strategy** (how long to wait) and **service performance** (how fast it responds), not just code correctness.

## Common Service Patterns

### Pattern 1: Trigger Service (No Parameters)

Some services just trigger an action without parameters.

**Example**: Reset odometry

```python
from std_srvs.srv import Empty

# Server
self.srv = self.create_service(Empty, 'reset_odom', self.reset_callback)

def reset_callback(self, request, response):
    # Reset odometry (no request data to use)
    self.x = 0.0
    self.y = 0.0
    self.theta = 0.0
    self.get_logger().info('Odometry reset')
    return response  # Empty response
```

**Client**:
```bash
ros2 service call /reset_odom std_srvs/srv/Empty
```

### Pattern 2: Boolean Response

Return success/failure with message.

**Service type**: `std_srvs/srv/SetBool`

```
bool data
---
bool success
string message
```

**Server example**:
```python
from std_srvs.srv import SetBool

def enable_callback(self, request, response):
    if request.data:
        self.enabled = True
        response.success = True
        response.message = 'System enabled'
    else:
        self.enabled = False
        response.success = True
        response.message = 'System disabled'
    return response
```

## Debugging Services

### Issue 1: Service Not Found

**Symptom**: Client waits forever with "Service not available, waiting..."

**Debug**:
```bash
ros2 service list
```

If your service doesn't appear, server isn't running or service name is wrong.

**Fix**: Ensure server is running and names match exactly.

### Issue 2: Type Mismatch

**Symptom**: Error like "Service type mismatch"

**Cause**: Client and server use different service types.

**Debug**:
```bash
ros2 service type /add_two_ints
```

**Output**: Should be `example_interfaces/srv/AddTwoInts`

**Fix**: Ensure both nodes import and use the same service type.

### Issue 3: Service Callback Doesn't Return Response

**Symptom**: Client hangs, server shows no errors.

**Cause**: Forgot to `return response` in callback.

**Fix**:
```python
def add_callback(self, request, response):
    response.sum = request.a + request.b
    return response  # REQUIRED
```

## Hands-On Exercise: String Length Service

**Challenge**: Create a service that:
- Service name: `/string_length`
- Service type: Create a custom request/response
  - Request: `string text`
  - Response: `int64 length`
- Server computes `len(text)` and returns it

**Note**: Since we haven't learned custom service types yet, use existing types creatively or wait for advanced chapters. For now, practice with `AddTwoInts` variations.

**Simplified challenge**: Create a subtraction service:
- Service name: `/subtract`
- Request: two integers `a` and `b`
- Response: `a - b`

<details>
<summary>Solution</summary>

**subtract_server.py**:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts  # Reuse for subtraction


class SubtractServerNode(Node):
    def __init__(self):
        super().__init__('subtract_server')
        self.srv = self.create_service(
            AddTwoInts,
            'subtract',
            self.subtract_callback
        )
        self.get_logger().info('Subtraction service ready')

    def subtract_callback(self, request, response):
        response.sum = request.a - request.b  # 'sum' holds difference
        self.get_logger().info(f'Request: {request.a} - {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SubtractServerNode()

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

Test:
```bash
python3 subtract_server.py
```

In another terminal:
```bash
ros2 service call /subtract example_interfaces/srv/AddTwoInts "{a: 10, b: 3}"
```

**Output**: `sum: 7`

</details>

## What You've Learned

You can now:
- ✅ Distinguish between topics (streaming) and services (request-response)
- ✅ Create ROS 2 service servers
- ✅ Create ROS 2 service clients
- ✅ Use existing service types
- ✅ Debug service communication issues
- ✅ Handle async service calls with futures

**Congratulations!** You've completed the ROS 2 Fundamentals chapter. You now have the foundation to:
- Create nodes
- Publish and subscribe to topics
- Create and call services
- Debug ROS 2 systems with CLI tools

**Next chapter**: You'll learn advanced ROS 2 concepts like parameters, actions, and launch files.

## Try With AI

**Setup**: Open your AI assistant to explore service patterns and solve real-world problems.

**Exploration Prompts**:

```
Prompt 1: "In ROS 2, I need to call a service that might take 10 seconds to respond (e.g., mapping a large area). Should I use async or sync calls? What happens if the client node has other work to do while waiting?"

Prompt 2: "I'm designing a robot calibration system. Calibration involves: (1) move to home position (service), (2) capture images while moving (topic), (3) compute calibration (service). Should I use services for all three steps, or mix services and topics?"

Prompt 3: "My ROS 2 service server receives 10 requests simultaneously. How does ROS 2 handle this? Are they queued and processed sequentially, or does each request get its own thread?"
```

**Expected Outcomes**: Your AI should help you understand service concurrency models, design patterns for mixing services and topics, and async programming strategies for long-running operations. Through these discussions, you'll develop intuition for architecting robust robot control systems.
