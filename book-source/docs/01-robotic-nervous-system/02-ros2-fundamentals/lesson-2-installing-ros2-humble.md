---
title: Installing ROS 2 Humble
chapter: 2
lesson: 2
learning_objectives:
  - Successfully install ROS 2 Humble on Ubuntu 22.04
  - Configure the ROS 2 environment correctly
  - Verify installation with turtlesim demo
  - Troubleshoot common installation issues
estimated_time: 60 minutes
skills:
  ros2_installation:
    proficiency: A2
  linux_terminal:
    proficiency: A2
generated_by: content-implementer v1.1.0
source_spec: specs/002-chapter-02-ros2-fundamentals/spec.md
created: 2025-11-28
last_modified: 2025-11-28
workflow: /sp.implement
version: 1.0.0
---

# Installing ROS 2 Humble

Before you can build robots with ROS 2, you need to get it running on your computer. This lesson walks through the complete installation process for **ROS 2 Humble** on **Ubuntu 22.04**.

## Prerequisites Check

Before starting, verify you have:

### 1. Ubuntu 22.04 LTS

ROS 2 Humble is designed for Ubuntu 22.04 (Jammy Jellyfish). Check your version:

```bash
lsb_release -a
```

**Expected output**:
```
Distributor ID: Ubuntu
Description:    Ubuntu 22.04.x LTS
Release:        22.04
Codename:       jammy
```

If you see a different version, you have three options:
- **Option A**: Upgrade to Ubuntu 22.04 (if on older Ubuntu)
- **Option B**: Use WSL2 with Ubuntu 22.04 (if on Windows)
- **Option C**: Use cloud alternative (see end of lesson)

### 2. Python 3.10+

Ubuntu 22.04 comes with Python 3.10 by default. Verify:

```bash
python3 --version
```

**Expected output**: `Python 3.10.x` (or higher)

### 3. Sufficient Disk Space

Check free space (need at least 10 GB):

```bash
df -h /
```

Look for the "Avail" column. If under 10 GB, free up space before proceeding.

### 4. Internet Connection

The installation downloads approximately **2 GB of packages**. Ensure you're on a stable connection.

## Installation Steps

### Step 1: Enable Ubuntu Universe Repository

ROS 2 packages depend on software in the Ubuntu Universe repository:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
```

**What this does**: The Universe repository contains community-maintained free and open-source software. ROS 2 requires some of these packages.

### Step 2: Add ROS 2 APT Repository

Next, add the official ROS 2 package repository:

```bash
sudo apt update && sudo apt install curl -y
```

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

**What this does**: These commands add ROS 2's package repository to your system and import the security key. This lets you install ROS 2 using `apt` (Ubuntu's package manager) and receive automatic updates.

### Step 3: Update Package Index

Refresh the package list to include ROS 2 packages:

```bash
sudo apt update
```

**What this does**: Downloads the latest package lists from all configured repositories, including the ROS 2 repository you just added.

### Step 4: Install ROS 2 Humble Desktop

Now install ROS 2:

```bash
sudo apt install ros-humble-desktop
```

**What you're installing**: The "Desktop" variant includes:
- Core ROS 2 libraries (communication, message types)
- RViz (3D visualization tool)
- Example demos and tutorials
- Documentation

**Installation time**: Expect 5-15 minutes depending on your internet speed. The download is about 2 GB.

**Alternative**: If you have limited disk space or don't need visualization tools, install the minimal version instead:

```bash
sudo apt install ros-humble-ros-base
```

(This guide assumes you installed `ros-humble-desktop`.)

### Step 5: Install Development Tools (Optional but Recommended)

If you plan to build ROS packages from source, install development tools:

```bash
sudo apt install ros-dev-tools
```

**What this provides**: `colcon` (ROS 2 build tool), `rosdep` (dependency manager), and other utilities for package development.

## Environment Setup

### Understand the Setup File

ROS 2 installed files to `/opt/ros/humble/`, but your terminal doesn't know about them yet. The **setup file** configures your shell environment with paths to ROS 2 commands and libraries.

Every time you open a new terminal, you need to "source" the setup file:

```bash
source /opt/ros/humble/setup.bash
```

**What this does**: Sets environment variables like `ROS_DISTRO`, `AMENT_PREFIX_PATH`, and `PATH` so your terminal can find ROS 2 commands.

**Verification**: After sourcing, verify ROS 2 is available:

```bash
printenv | grep -i ROS
```

**Expected output** (abbreviated):
```
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

### Automatic Sourcing (Recommended)

Sourcing manually every terminal is tedious. Add it to your shell's startup file:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

**What this does**: Appends the source command to `~/.bashrc`, which runs automatically when you open a new terminal.

**Apply immediately** (without opening new terminal):

```bash
source ~/.bashrc
```

Now ROS 2 will be available in all new terminals automatically.

## Verification Steps

### Test 1: Check ROS 2 Version

```bash
ros2 --version
```

**Expected output**:
```
ros2 cli version: 0.25.x
```

If you see this, ROS 2 is installed and accessible.

### Test 2: Run Turtlesim Demo

Turtlesim is ROS 2's "Hello World" application—a simple 2D robot simulator.

**Open a terminal** and run:

```bash
ros2 run turtlesim turtlesim_node
```

**Expected result**: A window should appear with a blue background and a turtle in the center:

![Turtlesim Window](https://docs.ros.org/en/humble/_images/turtlesim.png)

**Leave this running** and open a **second terminal**.

In the second terminal, run the keyboard teleop node:

```bash
ros2 run turtlesim turtle_teleop_key
```

**Expected output**:
```
Reading from keyboard
---------------------------
Use arrow keys to move the turtle.
Use G|B|V|C|D|E|R|T keys to rotate to absolute orientations.
'Q' to quit.
```

**Press arrow keys** in the second terminal. The turtle in the first window should move!

If you see the turtle moving, **congratulations—ROS 2 is working correctly**.

Press **Ctrl+C** in both terminals to stop the nodes.

### Test 3: Verify ROS 2 Commands

Test a few core ROS 2 commands:

```bash
ros2 node list
ros2 topic list
ros2 pkg list
```

If these commands run without "command not found" errors, your installation is complete.

## Troubleshooting Common Issues

### Issue 1: "ros2: command not found"

**Symptom**: After installation, typing `ros2` gives "command not found".

**Cause**: You haven't sourced the setup file.

**Solution**:
```bash
source /opt/ros/humble/setup.bash
```

Make it permanent:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Issue 2: "Unable to locate package ros-humble-desktop"

**Symptom**: During installation, apt can't find the ROS 2 package.

**Causes**:
1. ROS 2 repository not added correctly
2. Package index not updated

**Solution**:
```bash
# Re-add the repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package index
sudo apt update

# Try installation again
sudo apt install ros-humble-desktop
```

### Issue 3: Turtlesim Window Doesn't Appear

**Symptom**: `ros2 run turtlesim turtlesim_node` runs but no window appears.

**Causes**:
1. Missing display server (if on WSL2 or remote connection)
2. Graphics driver issues

**Solution for WSL2**: Install an X server on Windows (e.g., VcXsrv or WSLg for Windows 11).

**Solution for remote SSH**: Enable X11 forwarding with `ssh -X`.

### Issue 4: Import Errors with Python

**Symptom**: Running ROS 2 Python nodes gives import errors.

**Cause**: Python can't find ROS 2 Python libraries.

**Solution**: Ensure you sourced the setup file:
```bash
source /opt/ros/humble/setup.bash
python3 -c "import rclpy; print('ROS 2 Python working!')"
```

**Expected output**: `ROS 2 Python working!`

### Issue 5: Disk Space Issues

**Symptom**: Installation fails with "No space left on device".

**Solution**:
```bash
# Check disk usage
df -h

# Free up space (clean apt cache)
sudo apt clean
sudo apt autoremove

# Try installation again
```

## Cloud Alternative (For Non-Ubuntu Users)

If you can't install Ubuntu 22.04 locally, use a cloud environment:

### Option A: AWS RoboMaker (Free Tier Available)

1. Sign up for AWS Free Tier
2. Create a RoboMaker development environment
3. Choose ROS 2 Humble environment
4. Access via browser-based IDE

### Option B: GitHub Codespaces with ROS 2

1. Create a repository with ROS 2 Humble Dockerfile
2. Launch Codespace
3. Install ROS 2 in container
4. Develop via VS Code in browser

### Option C: Ubuntu VM with VirtualBox

1. Download VirtualBox (free)
2. Download Ubuntu 22.04 ISO
3. Create VM with at least 4 GB RAM and 20 GB disk
4. Install Ubuntu in VM
5. Follow normal installation steps

**Note**: Cloud and VM environments work well for learning but may have limitations for real robot hardware access later in the course.

## What's in Your Installation?

Now that ROS 2 is installed, you have access to:

**Core Tools**:
- `ros2` — Command-line interface for ROS 2
- `rviz2` — 3D visualization tool
- `rqt` — Graphical tools for debugging and visualization

**Example Nodes**:
- `turtlesim` — 2D robot simulator (you just ran this!)
- `demo_nodes_cpp` — C++ example nodes
- `demo_nodes_py` — Python example nodes

**Libraries**:
- `rclpy` — ROS 2 Python client library
- `rclcpp` — ROS 2 C++ client library
- Standard message types (geometry, sensor data, etc.)

In the next lesson, you'll use the `ros2` command-line tool to explore a running ROS 2 system.

## Try With AI

**Setup**: Open your AI assistant and ask for help if you encounter installation issues.

**Troubleshooting Prompts**:

```
Prompt 1: "I'm installing ROS 2 Humble on Ubuntu 22.04 and getting error: [paste your error message]. What's the cause and how do I fix it?"

Prompt 2: "Explain what the 'source /opt/ros/humble/setup.bash' command does. Why do I need to run it every time I open a terminal?"

Prompt 3: "I can't install Ubuntu 22.04 on my computer. What are my alternatives for running ROS 2 Humble?"
```

**Expected Outcomes**: Your AI can help diagnose specific error messages, explain ROS 2 environment configuration, and suggest alternative installation methods for your specific situation.
