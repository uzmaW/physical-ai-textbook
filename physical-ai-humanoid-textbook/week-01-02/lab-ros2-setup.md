---
id: lab-ros2-setup
title: "Lab 1: ROS 2 Development Environment Setup"
sidebar_label: "Lab 1: ROS 2 Setup"
sidebar_position: 4
---

# Lab 1: ROS 2 Humble Development Environment Setup

## Lab Overview

**Estimated Time**: 2-3 hours
**Difficulty**: Beginner
**Prerequisites**: Ubuntu 22.04 LTS, basic terminal knowledge

### Learning Objectives
- Install ROS 2 Humble Hawksbill
- Configure the ROS 2 environment
- Create your first ROS 2 package
- Run a simple publisher-subscriber example
- Install and test NVIDIA Isaac Sim (optional)

---

## Part 1: System Requirements

### Hardware Minimum
- **CPU**: Intel Core i5/AMD Ryzen 5 or better
- **RAM**: 8 GB (16 GB recommended)
- **Storage**: 50 GB free space
- **GPU**: NVIDIA GPU with 4+ GB VRAM (for Isaac Sim)

### Software Requirements
- **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish)
- **Python**: 3.10+
- **Build Tools**: CMake, GCC/G++

### Check Your System

```bash
# Check Ubuntu version
lsb_release -a
# Output should show: Ubuntu 22.04.x LTS

# Check Python version
python3 --version
# Output should show: Python 3.10.x or higher

# Check available disk space
df -h /
# Ensure at least 50 GB free
```

---

## Part 2: Installing ROS 2 Humble

### Step 1: Set Locale

```bash
# Ensure UTF-8 locale
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify
locale  # should show en_US.UTF-8
```

### Step 2: Setup Sources

```bash
# Enable Ubuntu Universe repository
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 3: Install ROS 2 Humble

```bash
# Update package index
sudo apt update
sudo apt upgrade

# Install ROS 2 Humble Desktop (includes RViz, demos, tutorials)
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools

# Install additional packages for robotics
sudo apt install \
  ros-humble-joint-state-publisher \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers

# This will take 10-20 minutes depending on your internet speed
```

### Step 4: Environment Setup

```bash
# Source ROS 2 setup script
source /opt/ros/humble/setup.bash

# Add to bashrc for automatic sourcing
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --version
# Output: ros2 cli version: 0.18.x

# Check available commands
ros2 --help
```

---

## Part 3: Creating Your First ROS 2 Workspace

### Step 1: Create Workspace

```bash
# Create workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone example packages
git clone https://github.com/ros2/examples.git -b humble

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
source install/setup.bash

# Add to bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Step 2: Test Installation

```bash
# Terminal 1: Run demo nodes
ros2 run demo_nodes_cpp talker

# Terminal 2: In a new terminal, run listener
source ~/ros2_ws/install/setup.bash
ros2 run demo_nodes_cpp listener

# You should see messages being published and received
```

**Expected Output**:
```
Terminal 1 (talker):
[INFO] [1234567890.123456789] [talker]: Publishing: 'Hello World: 1'
[INFO] [1234567890.223456789] [talker]: Publishing: 'Hello World: 2'

Terminal 2 (listener):
[INFO] [1234567890.124456789] [listener]: I heard: 'Hello World: 1'
[INFO] [1234567890.224456789] [listener]: I heard: 'Hello World: 2'
```

---

## Part 4: Your First ROS 2 Package

### Step 1: Create Package

```bash
cd ~/ros2_ws/src

# Create Python package
ros2 pkg create --build-type ament_python physical_ai_basics \
  --dependencies rclpy std_msgs

cd physical_ai_basics
```

### Step 2: Create Simple Publisher

Create file: `physical_ai_basics/robot_status_publisher.py`

```python
#!/usr/bin/env python3
"""
Simple ROS 2 publisher for robot status messages.
Demonstrates basic pub/sub pattern for Physical AI systems.

Author: PIAIC Humanoid AI Course
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class RobotStatusPublisher(Node):
    """
    Publishes simulated robot status at 1 Hz.

    Topics Published:
        /robot/status (std_msgs/String): JSON-formatted robot state
    """

    def __init__(self):
        super().__init__('robot_status_publisher')

        # Create publisher
        self.publisher_ = self.create_publisher(
            String,
            '/robot/status',
            10  # Queue size
        )

        # Create timer (1 Hz = once per second)
        self.timer = self.create_timer(1.0, self.publish_status)

        self.status_count = 0
        self.get_logger().info('Robot Status Publisher started')

    def publish_status(self):
        """Publish simulated robot status."""
        # Simulate robot state
        status = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'robot_id': 'humanoid_01',
            'battery_percentage': max(0, 100 - self.status_count),
            'status': 'operational' if self.status_count < 90 else 'low_battery',
            'position': {
                'x': 0.0,
                'y': 0.0,
                'z': 1.0
            },
            'message_count': self.status_count
        }

        # Create message
        msg = String()
        msg.data = json.dumps(status)

        # Publish
        self.publisher_.publish(msg)

        self.get_logger().info(f'Published status {self.status_count}: '
                               f'Battery {status["battery_percentage"]}%')

        self.status_count += 1


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    node = RobotStatusPublisher()

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

### Step 3: Create Subscriber

Create file: `physical_ai_basics/robot_status_subscriber.py`

```python
#!/usr/bin/env python3
"""
Simple ROS 2 subscriber that monitors robot status.
Demonstrates callback pattern and JSON parsing.

Author: PIAIC Humanoid AI Course
License: MIT
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class RobotStatusSubscriber(Node):
    """
    Subscribes to robot status and logs important events.

    Topics Subscribed:
        /robot/status (std_msgs/String): Robot state updates
    """

    def __init__(self):
        super().__init__('robot_status_subscriber')

        # Create subscription
        self.subscription = self.create_subscription(
            String,
            '/robot/status',
            self.status_callback,
            10
        )

        self.get_logger().info('Robot Status Subscriber started')

    def status_callback(self, msg):
        """Handle incoming status messages."""
        try:
            status = json.loads(msg.data)

            # Log basic info
            robot_id = status.get('robot_id', 'unknown')
            battery = status.get('battery_percentage', 0)
            state = status.get('status', 'unknown')

            self.get_logger().info(
                f"Robot {robot_id}: Battery {battery}% | Status: {state}"
            )

            # Alert on low battery
            if battery < 20:
                self.get_logger().warn(
                    f"⚠️  LOW BATTERY ALERT: {battery}% remaining!"
                )

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse status message: {e}")


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    node = RobotStatusSubscriber()

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

### Step 4: Configure Package

Edit `setup.py`:

```python
from setuptools import setup

package_name = 'physical_ai_basics'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PIAIC Student',
    maintainer_email='student@piaic.org',
    description='Physical AI basics for ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_status_publisher = physical_ai_basics.robot_status_publisher:main',
            'robot_status_subscriber = physical_ai_basics.robot_status_subscriber:main',
        ],
    },
)
```

### Step 5: Build and Run

```bash
# Build package
cd ~/ros2_ws
colcon build --packages-select physical_ai_basics

# Source workspace
source install/setup.bash

# Run publisher (Terminal 1)
ros2 run physical_ai_basics robot_status_publisher

# Run subscriber (Terminal 2)
ros2 run physical_ai_basics robot_status_subscriber
```

---

## Part 5: ROS 2 CLI Tools

### Inspect Running System

```bash
# List all nodes
ros2 node list

# Get info about a node
ros2 node info /robot_status_publisher

# List all topics
ros2 topic list

# Echo topic messages
ros2 topic echo /robot/status

# Show topic info
ros2 topic info /robot/status

# Visualize node graph
rqt_graph
```

### Recording and Playback

```bash
# Record all topics
ros2 bag record -a

# Record specific topic
ros2 bag record /robot/status

# Play back recorded data
ros2 bag play <bag_file>

# Get bag info
ros2 bag info <bag_file>
```

---

## Part 6: Installing NVIDIA Isaac Sim (Optional)

### Requirements
- **GPU**: NVIDIA RTX series (2060 or better)
- **Drivers**: NVIDIA driver 525+
- **VRAM**: 8 GB+ recommended

### Installation Steps

```bash
# Install NVIDIA Container Toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list

sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# Download Isaac Sim from NVIDIA
# Visit: https://developer.nvidia.com/isaac-sim
# Follow download instructions for your system

# Or use Docker (recommended for consistency)
docker pull nvcr.io/nvidia/isaac-sim:2023.1.0

# Run Isaac Sim
docker run --name isaac-sim --entrypoint bash -it --gpus all \
  -e "ACCEPT_EULA=Y" --rm --network=host \
  -v ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
  -v ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
  -v ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
  -v ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
  -v ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
  -v ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
  -v ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
  -v ~/docker/isaac-sim/documents:/root/Documents:rw \
  nvcr.io/nvidia/isaac-sim:2023.1.0
```

### Test Isaac Sim

```bash
# Inside Isaac Sim container
./python.sh standalone_examples/api/omni.isaac.core/add_cube.py

# You should see a GUI window with a cube in the scene
```

---

## Part 7: Troubleshooting

### Common Issues

**Issue**: `ros2: command not found`
```bash
# Solution: Source ROS 2 setup
source /opt/ros/humble/setup.bash
```

**Issue**: `colcon: command not found`
```bash
# Solution: Install colcon
sudo apt install python3-colcon-common-extensions
```

**Issue**: Permission denied on scripts
```bash
# Solution: Make scripts executable
chmod +x physical_ai_basics/*.py
```

**Issue**: Import error for rclpy
```bash
# Solution: Ensure package dependencies are installed
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

---

## Lab Deliverables

### Required
1. Screenshot of `ros2 topic list` showing `/robot/status`
2. Screenshot of both publisher and subscriber running simultaneously
3. Output of `ros2 node info /robot_status_publisher`

### Submission Format
- Create a PDF with screenshots and brief explanations
- Include your `robot_status_publisher.py` and `robot_status_subscriber.py` code
- Answer the exercises below

---

## Exercises

### Basic
1. Modify the publisher to send status updates at 2 Hz instead of 1 Hz.
2. Add a new field to the status JSON: `temperature` (simulated, random 20-30°C).
3. Make the subscriber print a warning when temperature > 28°C.

### Intermediate
4. Create a third node `battery_monitor` that subscribes to `/robot/status` and publishes to `/robot/battery` (Float64) just the battery percentage.
5. Use `ros2 bag` to record 30 seconds of `/robot/status` messages. Analyze the bag file and report the message frequency.
6. Modify the publisher to read initial position from a YAML config file.

### Advanced
7. Implement a service `reset_robot` that resets the battery to 100% and position to origin.
8. Create a launch file that starts both publisher and subscriber automatically.
9. **Challenge**: Integrate with RViz—visualize the robot's position using a TF broadcaster and RViz markers.

---

## Next Steps

Congratulations! You've set up your ROS 2 development environment and created your first publisher/subscriber nodes. In the next lab, you'll work with URDF models and visualize a humanoid robot in RViz.

**Next Lab**: [Lab 2: URDF Modeling and Visualization](../week-03/lab-urdf-modeling)

---

**Copyright Notice**: © 2025 PIAIC. Licensed under CC BY-SA 4.0.

## References

[1] ROS 2 Documentation, "Installation Guide," https://docs.ros.org/en/humble/Installation.html

[2] ROS 2 Tutorials, "Creating Your First ROS 2 Package," https://docs.ros.org/en/humble/Tutorials.html

[3] NVIDIA Isaac Sim Documentation, https://docs.omniverse.nvidia.com/isaacsim/latest/index.html

