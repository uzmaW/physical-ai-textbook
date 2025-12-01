# ROS 2 Lab Generator

Generate complete ROS 2 Python node templates for robotics lab exercises.

## Capabilities

- Create ROS 2 Humble compatible Python nodes
- Generate publishers, subscribers, services, and action servers/clients
- Include proper docstrings with citations
- Add error handling and logging
- Follow ROS 2 best practices (rclpy)

## Usage

User specifies:
- Node functionality (e.g., "joint state publisher", "velocity controller")
- Topic/service names
- Message types
- Update rate (Hz)

## Output Format

Complete Python file (`.py`) with:

1. **Shebang and docstring**:
```python
#!/usr/bin/env python3
"""
[Node description]

Author: PIAIC Humanoid AI Course
License: MIT
ROS 2 Version: Humble
"""
```

2. **Imports**:
```python
import rclpy
from rclpy.node import Node
from [msg_package].msg import [MessageType]
```

3. **Node class**:
- Inherit from `Node`
- `__init__` with publishers/subscribers
- Callback methods
- Proper logging

4. **Main function**:
```python
def main(args=None):
    rclpy.init(args=args)
    node = NodeName()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

## Example

**Input**: "Create a publisher for robot battery status at 1 Hz"

**Output**: Complete `battery_status_publisher.py` with:
- Publishes to `/robot/battery` topic
- Message type: `std_msgs/Float32`
- Simulated battery drain (100% → 0%)
- Logging every publish
- Proper error handling

## Quality Standards

- ✅ ROS 2 Humble compatible syntax
- ✅ Type hints where appropriate
- ✅ Docstrings for all methods
- ✅ Error handling (try/except)
- ✅ Clear variable names
- ✅ Comments for complex logic
- ✅ Citation if implementing known algorithm

